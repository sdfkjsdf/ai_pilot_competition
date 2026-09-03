#include "LadyLuck/guidance/obfm/G5bDelayedClimb.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/control/route5/Route5Guidance.hpp"
#include "LadyLuck/guidance/ThreatRecoveryMargin.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::ControlIntent;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::committed::G16ProductionEvidenceReceipt;
using LadyLuck::guidance::obfm::G5bBoomZoomEligibilityReason;
using LadyLuck::guidance::obfm::G5bBoomZoomEligibilityReceipt;
using LadyLuck::guidance::obfm::G5bControlBackend;
using LadyLuck::guidance::obfm::G5bDelayedClimbObservation;
using LadyLuck::guidance::obfm::G5bDelayedClimbPhase;
using LadyLuck::guidance::obfm::G5bDelayedClimbSelection;
using LadyLuck::guidance::obfm::G5bDelayedClimbSnapshot;
using LadyLuck::guidance::obfm::G5bEnergyComparisonAuthority;
using LadyLuck::guidance::obfm::G5bFeedbackFreshness;
using LadyLuck::guidance::obfm::G5bReleaseReason;
using LadyLuck::guidance::obfm::G5bSafetyAdmissionReason;
using LadyLuck::guidance::obfm::G5bSafetyAdmissionReceipt;
using LadyLuck::guidance::obfm::G5bSafetyEvidence;
using LadyLuck::guidance::obfm::G5bSelectedBranch;
using LadyLuck::guidance::obfm::G5bSpeedFloorEvidence;

constexpr double kCapabilityTableMachEdge = 2.0;
constexpr double kAirHeatCapacityRatio = 1.4;
constexpr double kAirSpecificGasConstantJpkgK = 287.05287;
constexpr double kSeaLevelTemperatureK = 288.15;
constexpr double kTroposphereLapseKpm = 0.0065;
constexpr double kTropopauseAltitudeM = 11000.0;
constexpr double kTropopauseTemperatureK = 216.65;
constexpr double kBattleServerRpyQuantumRad =
    LadyLuck::constants::Pi / 180.0 / 1000.0;
constexpr double kBattleServerBodyVelocityQuantumMps =
    0.001 * LadyLuck::constants::FeetToMeters;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

bool NumpyNorm3(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }

    // Preserve the established NumPy-order dot/sqrt result for ordinary
    // inputs, but reject an arithmetic domain that could set OVERFLOW or
    // UNDERFLOW before the later finite-result check can run.  Four-way
    // headroom keeps the fixed three-square sum below DBL_MAX; the lower
    // bound keeps every executed nonzero square normal.
    const double maximum_component = std::sqrt(
        (std::numeric_limits<double>::max)() / 4.0);
    const double minimum_component = 2.0 * std::sqrt(
        (std::numeric_limits<double>::min)());
    for (const double component : value)
    {
        const double magnitude = std::fabs(component);
        if (magnitude > maximum_component
            || (magnitude != 0.0 && magnitude < minimum_component))
        {
            return false;
        }
    }

    output = std::sqrt(Dot(value, value));
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

double Hypot3(
    const double first,
    const double second,
    const double third) noexcept
{
    return std::hypot(std::hypot(first, second), third);
}

bool SafeAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    if (!std::isfinite(right))
    {
        output = 0.0;
        return false;
    }
    return SafeAdd(left, -right, output);
}

bool SafeMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if (left == 0.0 || right == 0.0)
    {
        return true;
    }

    const double left_magnitude = std::fabs(left);
    const double right_magnitude = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((left_magnitude > 1.0
            && right_magnitude >= maximum / left_magnitude)
        || (right_magnitude > 1.0
            && left_magnitude >= maximum / right_magnitude))
    {
        return false;
    }
    const int exponent_sum = std::ilogb(left_magnitude)
        + std::ilogb(right_magnitude);
    if (exponent_sum <= std::numeric_limits<double>::min_exponent - 1)
    {
        return false;
    }

    output = left * right;
    if (!std::isfinite(output)
        || (output != 0.0 && std::fpclassify(output) == FP_SUBNORMAL))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeDivide(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(numerator)
        || !std::isfinite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    if (numerator == 0.0)
    {
        return true;
    }

    const int exponent_difference = std::ilogb(std::fabs(numerator))
        - std::ilogb(std::fabs(denominator));
    // Reject the two mantissa-sensitive boundary exponents as well as the
    // certain overflow/underflow regions.  This is deliberately conservative:
    // unavailable extreme finite evidence must not execute a quotient that
    // can set FE_OVERFLOW or FE_UNDERFLOW.
    if (exponent_difference
            >= std::numeric_limits<double>::max_exponent - 1
        || exponent_difference
            <= std::numeric_limits<double>::min_exponent)
    {
        return false;
    }

    output = numerator / denominator;
    if (!std::isfinite(output)
        || (output != 0.0 && std::fpclassify(output) == FP_SUBNORMAL))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeScaledNorm3(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const double scale = (std::max)(
        std::fabs(value[0]),
        (std::max)(std::fabs(value[1]), std::fabs(value[2])));
    if (scale == 0.0)
    {
        return true;
    }
    if (std::fpclassify(scale) == FP_SUBNORMAL)
    {
        return false;
    }

    Vector3 scaled{};
    if (!SafeDivide(value[0], scale, scaled[0])
        || !SafeDivide(value[1], scale, scaled[1])
        || !SafeDivide(value[2], scale, scaled[2]))
    {
        return false;
    }
    double square_x = 0.0;
    double square_y = 0.0;
    double square_z = 0.0;
    double sum_xy = 0.0;
    double sum = 0.0;
    if (!SafeMultiply(scaled[0], scaled[0], square_x)
        || !SafeMultiply(scaled[1], scaled[1], square_y)
        || !SafeMultiply(scaled[2], scaled[2], square_z)
        || !SafeAdd(square_x, square_y, sum_xy)
        || !SafeAdd(sum_xy, square_z, sum))
    {
        return false;
    }
    const double scaled_norm = std::sqrt(sum);
    return SafeMultiply(scale, scaled_norm, output);
}

struct G5bEnergyStandingEvaluation
{
    bool evaluated = false;
    double delta_specific_energy_m = 0.0;
    double evidence_band_m = 0.0;
    bool advantage_proven = false;
};

bool EnergyPrimitiveArithmeticRepresentable(
    const double altitude_m,
    const double speed_mps) noexcept
{
    if (!std::isfinite(altitude_m)
        || !std::isfinite(speed_mps)
        || speed_mps < 0.0)
    {
        return false;
    }

    // The public HABFM primitives currently evaluate v*v/(2g),
    // q*(2v+q)/(2g), |h|*2^-23, and a four-term band sum with raw double
    // arithmetic.  Keep their inputs inside a conservative normal interval
    // before calling them.  The factors are arithmetic headroom only; they
    // do not alter a tactical threshold or an admitted physical value.
    const double maximum_input = std::sqrt(
        (std::numeric_limits<double>::max)()) / 4.0;
    const double minimum_nonzero_speed = 2.0 * std::sqrt(
        2.0 * LadyLuck::constants::StandardGravityMps2
            * (std::numeric_limits<double>::min)());
    const double minimum_nonzero_altitude =
        2.0 * (std::numeric_limits<double>::min)() / 0x1.0p-23;
    const double altitude_magnitude = std::fabs(altitude_m);
    return altitude_magnitude <= maximum_input
        && speed_mps <= maximum_input
        && (altitude_magnitude == 0.0
            || altitude_magnitude >= minimum_nonzero_altitude)
        && (speed_mps == 0.0
            || speed_mps >= minimum_nonzero_speed);
}

void EvaluateEnergyStanding(
    const G16ProductionEvidenceReceipt& evidence,
    G5bEnergyStandingEvaluation& output) noexcept
{
    output = G5bEnergyStandingEvaluation{};
    const auto& frame = evidence.frame;
    double own_speed_mps = 0.0;
    double opponent_speed_mps = 0.0;
    if (!SafeScaledNorm3(frame.own.velocity_ned_mps, own_speed_mps)
        || !SafeScaledNorm3(
            frame.opponent.velocity_ned_mps,
            opponent_speed_mps))
    {
        return;
    }
    const double own_altitude_m = -frame.own.position_ned_m[2];
    const double opponent_altitude_m = -frame.opponent.position_ned_m[2];
    if (!EnergyPrimitiveArithmeticRepresentable(
            own_altitude_m,
            own_speed_mps)
        || !EnergyPrimitiveArithmeticRepresentable(
            opponent_altitude_m,
            opponent_speed_mps))
    {
        return;
    }

    const auto own_energy = LadyLuck::SpecificEnergyM(
        own_altitude_m,
        own_speed_mps);
    const auto opponent_energy = LadyLuck::SpecificEnergyM(
        opponent_altitude_m,
        opponent_speed_mps);
    const auto band = LadyLuck::EnergyEvidenceBandM(
        own_altitude_m,
        own_speed_mps,
        opponent_altitude_m,
        opponent_speed_mps);
    double delta_specific_energy_m = 0.0;
    if (!own_energy.ok()
        || !opponent_energy.ok()
        || !band.ok()
        || !std::isfinite(band.value)
        || band.value < 0.0
        || !SafeSubtract(
            own_energy.value,
            opponent_energy.value,
            delta_specific_energy_m))
    {
        return;
    }

    output.evaluated = true;
    output.delta_specific_energy_m = delta_specific_energy_m;
    output.evidence_band_m = band.value;
    output.advantage_proven = delta_specific_energy_m > band.value;
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale(const Vector3& value, const double scale) noexcept
{
    return Vector3{{
        value[0] * scale,
        value[1] * scale,
        value[2] * scale}};
}

Vector3 Add(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

void FailIntent(
    ControlIntent& output,
    Status& status,
    const StatusCode code) noexcept
{
    output.Clear();
    status.code = code;
}

bool ValidEvidence(const G16ProductionEvidenceReceipt& evidence) noexcept
{
    return evidence.valid
        && LadyLuck::IsValidControlFrameIdentity(evidence.frame_identity)
        && LadyLuck::SameControlFrameIdentity(
            evidence.frame_identity,
            evidence.frame.frame_identity)
        && FiniteVector(evidence.frame.own.position_ned_m)
        && FiniteVector(evidence.frame.own.velocity_body_mps)
        && FiniteVector(evidence.frame.own.velocity_ned_mps)
        && FiniteVector(evidence.frame.opponent.position_ned_m)
        && FiniteVector(evidence.frame.opponent.velocity_body_mps)
        && FiniteVector(evidence.frame.opponent.velocity_ned_mps)
        && FiniteVector(evidence.frame.opponent.nose_ned)
        && std::isfinite(evidence.frame.t_sec)
        && std::isfinite(evidence.enemy_range_m)
        && std::isfinite(evidence.enemy_outer_wez_range_m);
}

void Float32WireVectorErrorBound(
    const Vector3& value,
    double& output,
    Status& status) noexcept
{
    output = 0.0;
    status = Status{};
    if (!FiniteVector(value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    Vector3 residual{};
    Vector3 full_cell{};
    const float positive_infinity =
        (std::numeric_limits<float>::infinity)();
    const float negative_infinity = -positive_infinity;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (value[index] == 0.0)
        {
            residual[index] = 0.0;
            full_cell[index] = 0.0;
            continue;
        }
        const double magnitude = std::fabs(value[index]);
        if (magnitude > static_cast<double>(
                (std::numeric_limits<float>::max)())
            || magnitude <= static_cast<double>(
                (std::numeric_limits<float>::min)()))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        const float represented_float = static_cast<float>(value[index]);
        const double represented = static_cast<double>(represented_float);
        const float represented_magnitude = std::fabs(represented_float);
        if (!std::isfinite(represented)
            || represented_magnitude
                <= (std::numeric_limits<float>::min)()
            || represented_magnitude
                >= (std::numeric_limits<float>::max)())
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        const double upper = static_cast<double>(std::nextafter(
            represented_float,
            positive_infinity));
        const double lower = static_cast<double>(std::nextafter(
            represented_float,
            negative_infinity));
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        residual[index] = value[index] - represented;
        full_cell[index] = (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }

    const double infinity = (std::numeric_limits<double>::infinity)();
    const double raw_residual_norm =
        Hypot3(residual[0], residual[1], residual[2]);
    const double residual_norm = raw_residual_norm == 0.0
        ? 0.0
        : std::nextafter(raw_residual_norm, infinity);
    const double raw_cell_norm =
        Hypot3(full_cell[0], full_cell[1], full_cell[2]);
    const double cell_norm = raw_cell_norm == 0.0
        ? 0.0
        : std::nextafter(raw_cell_norm, infinity);
    const double raw_output = residual_norm + cell_norm;
    output = raw_output == 0.0
        ? 0.0
        : std::nextafter(raw_output, infinity);
    if (!std::isfinite(output))
    {
        output = 0.0;
        status.code = StatusCode::NonFiniteInput;
    }
}

void Binary64VectorRoundoffBound(
    const Vector3& value,
    double& output,
    Status& status) noexcept
{
    output = 0.0;
    status = Status{};
    if (!FiniteVector(value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    Vector3 full_cell{};
    const double infinity = (std::numeric_limits<double>::infinity)();
    const double minimum_normal_roundoff_input = 2.0 * std::sqrt(
        (std::numeric_limits<double>::min)());
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (value[index] == 0.0)
        {
            // Binary64 zero is exact; asking nextafter for an artificial
            // adjacent subnormal both overstates this roundoff source and
            // raises FE_UNDERFLOW under /fp:strict.
            full_cell[index] = 0.0;
            continue;
        }
        const double magnitude = std::fabs(value[index]);
        // Below this conservative bound, the adjacent-cell width and its
        // vector norm can enter the subnormal domain even while the input is
        // itself normal.  Reject before either nextafter is executed.
        if (magnitude < minimum_normal_roundoff_input
            || magnitude >= (std::numeric_limits<double>::max)())
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        const double upper = std::nextafter(value[index], infinity);
        const double lower = std::nextafter(value[index], -infinity);
        full_cell[index] = (std::max)(
            std::fabs(upper - value[index]),
            std::fabs(value[index] - lower));
        if (!std::isfinite(full_cell[index]))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    const double raw_output =
        Hypot3(full_cell[0], full_cell[1], full_cell[2]);
    output = raw_output == 0.0
        ? 0.0
        : std::nextafter(raw_output, infinity);
    if (!std::isfinite(output))
    {
        output = 0.0;
        status.code = StatusCode::NonFiniteInput;
    }
}

void WorldVelocityErrorBound(
    const LadyLuck::AircraftGeometryKinematics& aircraft,
    double& output,
    Status& status) noexcept
{
    output = 0.0;
    status = Status{};
    double body_wire_error = 0.0;
    Float32WireVectorErrorBound(
        aircraft.velocity_body_mps,
        body_wire_error,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    double result_roundoff = 0.0;
    Binary64VectorRoundoffBound(
        aircraft.velocity_ned_mps,
        result_roundoff,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    const double body_error = body_wire_error
        + std::sqrt(3.0) * kBattleServerBodyVelocityQuantumMps;
    const double attitude_error = 3.0 * kBattleServerRpyQuantumRad;
    double body_speed = 0.0;
    if (!NumpyNorm3(aircraft.velocity_body_mps, body_speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double rotation_error = 2.0 * (body_speed + body_error)
        * std::sin(attitude_error / 2.0);
    output = body_error + rotation_error + result_roundoff;
    if (!std::isfinite(body_speed)
        || !std::isfinite(output)
        || output < 0.0)
    {
        output = 0.0;
        status.code = StatusCode::NonFiniteInput;
    }
}

bool OutwardRound(
    const double value,
    const double direction,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(value) || std::isnan(direction))
    {
        return false;
    }
    if (value == 0.0)
    {
        return true;
    }
    if (std::fpclassify(value) != FP_NORMAL)
    {
        return false;
    }
    output = std::nextafter(value, direction);
    return std::isfinite(output)
        && (output == 0.0 || std::fpclassify(output) == FP_NORMAL);
}

bool KineticHeadM(
    const double speed_mps,
    const bool upper,
    double& output) noexcept
{
    output = 0.0;
    double speed_squared = 0.0;
    double raw = 0.0;
    return std::isfinite(speed_mps)
        && speed_mps >= 0.0
        && SafeMultiply(speed_mps, speed_mps, speed_squared)
        && SafeDivide(
            speed_squared,
            2.0 * LadyLuck::constants::StandardGravityMps2,
            raw)
        && OutwardRound(
            raw,
            upper
                ? (std::numeric_limits<double>::infinity)()
                : -(std::numeric_limits<double>::infinity)(),
            output);
}

bool EnergyCeilingM(
    const double altitude_m,
    const double speed_mps,
    const double floor_mps,
    const bool upper,
    double& output) noexcept
{
    output = 0.0;
    double speed_head_m = 0.0;
    double floor_head_m = 0.0;
    double surplus_head_m = 0.0;
    double raw = 0.0;
    if (!KineticHeadM(speed_mps, upper, speed_head_m)
        || !KineticHeadM(floor_mps, !upper, floor_head_m)
        || !SafeSubtract(speed_head_m, floor_head_m, surplus_head_m)
        || !SafeAdd(altitude_m, (std::max)(0.0, surplus_head_m), raw))
    {
        return false;
    }
    return OutwardRound(
        raw,
        upper
            ? (std::numeric_limits<double>::infinity)()
            : -(std::numeric_limits<double>::infinity)(),
        output);
}

bool SpecificEnergyM(
    const double altitude_m,
    const double speed_mps,
    const bool upper,
    double& output) noexcept
{
    output = 0.0;
    double kinetic_head_m = 0.0;
    double raw = 0.0;
    if (!KineticHeadM(speed_mps, upper, kinetic_head_m)
        || !SafeAdd(altitude_m, kinetic_head_m, raw))
    {
        return false;
    }
    return OutwardRound(
        raw,
        upper
            ? (std::numeric_limits<double>::infinity)()
            : -(std::numeric_limits<double>::infinity)(),
        output);
}

bool ChronologyConsistent(
    const G5bDelayedClimbSnapshot& snapshot,
    const ControlFrameIdentity& current) noexcept
{
    if (!snapshot.strict_outward_completion_lineage_valid
        || !LadyLuck::IsValidControlFrameIdentity(
            snapshot.strict_outward_completion_identity)
        || !LadyLuck::IsValidControlFrameIdentity(current)
        || current.episode_epoch
            != snapshot.strict_outward_completion_identity.episode_epoch
        || current.frame_index
            < snapshot.strict_outward_completion_identity.frame_index
        || current.source_time_s
            < snapshot.strict_outward_completion_identity.source_time_s)
    {
        return false;
    }
    if (!snapshot.last_observation_identity_valid)
    {
        return true;
    }
    return current.episode_epoch
            == snapshot.last_observation_identity.episode_epoch
        && current.frame_index
            > snapshot.last_observation_identity.frame_index
        && current.source_time_s
            > snapshot.last_observation_identity.source_time_s;
}

void EvaluateBoomZoomEligibility(
    const G16ProductionEvidenceReceipt& evidence,
    const G5bSafetyEvidence& safety,
    const G5bSpeedFloorEvidence& speed_floor,
    const G5bDelayedClimbSnapshot& snapshot,
    const G5bSafetyAdmissionReceipt& entry_safety,
    const bool opponent_turn_aimed,
    const LadyLuck::guidance::ThreatRecoveryMarginReceipt& threat_margin,
    const double own_speed_lower_mps,
    const double own_altitude_lower_m,
    const double own_altitude_error_m,
    G5bBoomZoomEligibilityReceipt& output) noexcept
{
    output = G5bBoomZoomEligibilityReceipt{};
    output.evaluated = true;
    output.frame_identity = evidence.frame_identity;
    output.strict_outward_completion_identity =
        snapshot.strict_outward_completion_identity;
    output.strict_outward_wez_complete =
        snapshot.strict_outward_completion_lineage_valid;
    output.estimator_chronology_consistent =
        ChronologyConsistent(snapshot, evidence.frame_identity);
    output.extend_start_range_upper_m =
        snapshot.extend_start_range_upper_m;
    if (evidence.enemy_range_interval.valid
        && std::isfinite(evidence.enemy_range_interval.lower_m))
    {
        output.current_range_lower_m.has_value = true;
        output.current_range_lower_m.value =
            evidence.enemy_range_interval.lower_m;
    }
    output.extend_range_increase_observed =
        output.extend_start_range_upper_m.has_value
        && output.current_range_lower_m.has_value
        && evidence.frame_identity.frame_index
            > snapshot.strict_outward_completion_identity.frame_index
        && evidence.frame_identity.source_time_s
            > snapshot.strict_outward_completion_identity.source_time_s
        && output.current_range_lower_m.value
            > output.extend_start_range_upper_m.value;
    output.energy_comparison_authority =
        safety.energy_comparison_authority;
    output.same_f16_type_and_mass_assumed =
        safety.same_f16_type_and_mass_assumed;
    const bool boom_zoom_evidence_current = safety.boom_zoom_evidence_valid
        && LadyLuck::IsValidControlFrameIdentity(
            safety.boom_zoom_frame_identity)
        && LadyLuck::SameControlFrameIdentity(
            safety.boom_zoom_frame_identity,
            evidence.frame_identity);
    output.comparison_authority_available = boom_zoom_evidence_current
        && safety.energy_comparison_authority
            == G5bEnergyComparisonAuthority::CompetitionSameF16TypeAndMass
        && safety.same_f16_type_and_mass_assumed;
    output.official_enemy_gun_receipt_current =
        safety.official_enemy_gun_receipt_current;
    output.predictive_prefire_receipt_current =
        safety.predictive_prefire_receipt_current;
    output.predictive_prefire_absence_resolved =
        safety.predictive_prefire_absence_resolved;
    output.enemy_fire_opportunity_evaluated = boom_zoom_evidence_current
        && safety.official_enemy_gun_receipt_current
        && safety.enemy_fire_opportunity_evaluated;
    output.enemy_fire_opportunity_absent =
        output.enemy_fire_opportunity_evaluated
        && !safety.enemy_fire_opportunity_active;
    output.safety_admitted = entry_safety.admitted;

    const bool own_floor_admitted = speed_floor.valid
        && speed_floor.sample.admitted()
        && std::isfinite(speed_floor.sample.floor_mps.value)
        && speed_floor.sample.floor_mps.value > 0.0;
    const bool opponent_floor_admitted = speed_floor.opponent_valid
        && speed_floor.opponent_sample.admitted()
        && std::isfinite(speed_floor.opponent_sample.floor_mps.value)
        && speed_floor.opponent_sample.floor_mps.value > 0.0;
    output.own_speed_floor_admitted = own_floor_admitted;
    output.opponent_speed_floor_admitted = opponent_floor_admitted;
    output.own_tactical_speed_floor_mps = own_floor_admitted
        ? speed_floor.sample.floor_mps.value
        : 0.0;
    output.opponent_tactical_speed_floor_mps = opponent_floor_admitted
        ? speed_floor.opponent_sample.floor_mps.value
        : 0.0;
    output.own_speed_lower_mps = own_speed_lower_mps;
    output.own_altitude_lower_m = own_altitude_lower_m;

    double opponent_speed_mps = 0.0;
    double opponent_speed_error_mps = 0.0;
    double opponent_position_error_m = 0.0;
    Status bound_status{};
    WorldVelocityErrorBound(
        evidence.frame.opponent,
        opponent_speed_error_mps,
        bound_status);
    const bool velocity_bound_valid = bound_status.ok()
        && SafeScaledNorm3(
            evidence.frame.opponent.velocity_ned_mps,
            opponent_speed_mps);
    bound_status = Status{};
    Float32WireVectorErrorBound(
        evidence.frame.opponent.position_ned_m,
        opponent_position_error_m,
        bound_status);
    const bool position_bound_valid = bound_status.ok();
    const double infinity = (std::numeric_limits<double>::infinity)();
    double opponent_speed_upper_mps = 0.0;
    double opponent_altitude_upper_m = 0.0;
    bool opponent_bounds_valid = velocity_bound_valid
        && position_bound_valid
        && SafeAdd(
            opponent_speed_mps,
            opponent_speed_error_mps,
            opponent_speed_upper_mps)
        && SafeAdd(
            -evidence.frame.opponent.position_ned_m[2],
            opponent_position_error_m,
            opponent_altitude_upper_m)
        && OutwardRound(
            opponent_speed_upper_mps,
            infinity,
            opponent_speed_upper_mps)
        && OutwardRound(
            opponent_altitude_upper_m,
            infinity,
            opponent_altitude_upper_m);
    output.opponent_speed_upper_mps = opponent_bounds_valid
        ? opponent_speed_upper_mps
        : 0.0;
    output.opponent_altitude_upper_m = opponent_bounds_valid
        ? opponent_altitude_upper_m
        : 0.0;

    double own_specific_energy_lower_m = 0.0;
    double opponent_specific_energy_upper_m = 0.0;
    double own_zoom_target_lower_m = 0.0;
    double opponent_follow_ceiling_upper_m = 0.0;
    const bool energy_bounds = output.comparison_authority_available
        && own_floor_admitted
        && opponent_floor_admitted
        && opponent_bounds_valid
        && SpecificEnergyM(
            own_altitude_lower_m,
            own_speed_lower_mps,
            false,
            own_specific_energy_lower_m)
        && SpecificEnergyM(
            opponent_altitude_upper_m,
            opponent_speed_upper_mps,
            true,
            opponent_specific_energy_upper_m)
        && EnergyCeilingM(
            own_altitude_lower_m,
            own_speed_lower_mps,
            output.own_tactical_speed_floor_mps,
            false,
            own_zoom_target_lower_m)
        && EnergyCeilingM(
            opponent_altitude_upper_m,
            opponent_speed_upper_mps,
            output.opponent_tactical_speed_floor_mps,
            true,
            opponent_follow_ceiling_upper_m);
    output.energy_bounds_evaluated = energy_bounds;
    output.own_specific_energy_lower_m = energy_bounds
        ? own_specific_energy_lower_m
        : 0.0;
    output.opponent_specific_energy_upper_m = energy_bounds
        ? opponent_specific_energy_upper_m
        : 0.0;
    output.own_energy_advantage_resolved = energy_bounds
        && own_specific_energy_lower_m > opponent_specific_energy_upper_m;
    output.own_zoom_target_lower_m = energy_bounds
        ? own_zoom_target_lower_m
        : 0.0;
    output.opponent_follow_ceiling_upper_m = energy_bounds
        ? opponent_follow_ceiling_upper_m
        : 0.0;
    output.opponent_vertical_follow_insufficient = energy_bounds
        && own_zoom_target_lower_m > opponent_follow_ceiling_upper_m;

    double twice_altitude_error_m = 0.0;
    double zoom_gain_lower_m = 0.0;
    output.zoom_energy_budget_available = energy_bounds
        && own_speed_lower_mps > output.own_tactical_speed_floor_mps
        && SafeMultiply(
            2.0,
            own_altitude_error_m,
            twice_altitude_error_m)
        && SafeSubtract(
            own_zoom_target_lower_m,
            own_altitude_lower_m,
            zoom_gain_lower_m)
        && zoom_gain_lower_m > twice_altitude_error_m;
    output.strict_current_wez_clear = evidence.enemy_range_interval.valid
        && std::isfinite(evidence.enemy_range_interval.lower_m)
        && std::isfinite(evidence.enemy_outer_wez_range_m)
        && evidence.enemy_range_interval.lower_m
            > evidence.enemy_outer_wez_range_m;

    if (!output.strict_outward_wez_complete)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            StrictOutwardCompletionLineageUnresolved;
    }
    else if (!output.estimator_chronology_consistent)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            EstimatorChronologyUnresolved;
    }
    else if (!output.extend_range_increase_observed)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            ExtendRangeIncreaseUnresolved;
    }
    else if (!output.comparison_authority_available)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            EnergyComparisonAuthorityUnavailable;
    }
    else if (!own_floor_admitted)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            OwnSpeedFloorUnavailable;
    }
    else if (!opponent_floor_admitted)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            OpponentSpeedFloorUnavailable;
    }
    else if (!energy_bounds)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            EnergyBoundsUnresolved;
    }
    else if (!output.own_energy_advantage_resolved)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            OwnEnergyAdvantageNotProven;
    }
    else if (!output.strict_current_wez_clear)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            StrictCurrentWezClearUnresolved;
    }
    else if (!output.zoom_energy_budget_available)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            ZoomEnergyBudgetUnavailable;
    }
    else if (!output.opponent_vertical_follow_insufficient)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            OpponentVerticalFollowNotInsufficient;
    }
    else if (!output.enemy_fire_opportunity_evaluated)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            EnemyFireOpportunityUnresolved;
    }
    else if (!output.enemy_fire_opportunity_absent)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            EnemyFireOpportunityActive;
    }
    else if (!opponent_turn_aimed)
    {
        // Missing or negative turn/lead evidence is ordinary EXTEND.  It is
        // neither a transaction failure nor authority for a vertical command.
        output.reason = G5bBoomZoomEligibilityReason::OpponentTurnNotAimed;
    }
    else if (!entry_safety.admitted)
    {
        output.reason = G5bBoomZoomEligibilityReason::SafetyNotAdmitted;
    }
    else if (!threat_margin.evaluated)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            ThreatRecoveryMarginUnresolved;
    }
    else if (threat_margin.exhausted)
    {
        output.reason = G5bBoomZoomEligibilityReason::
            ThreatRecoveryMarginExhausted;
    }
    else
    {
        output.zoom_admitted = true;
        output.reason = G5bBoomZoomEligibilityReason::Admitted;
    }
}

void OpponentTurnCue(
    const DogfightGeometryFrame& frame,
    bool& aimed,
    LadyLuck::guidance::obfm::G5bTurnCueReason& reason,
    Status& status) noexcept
{
    aimed = false;
    reason = LadyLuck::guidance::obfm::
        G5bTurnCueReason::AdversaryNoseAimed;
    status = Status{};
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.nose_ned))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const Vector3 los = Subtract(
        frame.own.position_ned_m,
        frame.opponent.position_ned_m);
    double los_norm = 0.0;
    double own_speed = 0.0;
    double nose_norm = 0.0;
    if (!NumpyNorm3(los, los_norm)
        || !NumpyNorm3(frame.own.velocity_ned_mps, own_speed)
        || !NumpyNorm3(frame.opponent.nose_ned, nose_norm))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (los_norm < LadyLuck::constants::Tiny
        || own_speed < LadyLuck::constants::Tiny
        || nose_norm < LadyLuck::constants::Tiny)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const Vector3 los_hat = Scale(los, 1.0 / los_norm);
    const Vector3 nose_hat = Scale(frame.opponent.nose_ned, 1.0 / nose_norm);
    const Vector3 velocity_hat = Scale(
        frame.own.velocity_ned_mps,
        1.0 / own_speed);
    const double nose_los = Dot(nose_hat, los_hat);
    const bool nose_forward = nose_los > 0.0;
    const Vector3 nose_perpendicular = Subtract(
        nose_hat,
        Scale(los_hat, nose_los));
    const double lag_sign = Dot(nose_perpendicular, velocity_hat);
    const bool extend_candidate = !nose_forward || lag_sign < 0.0;
    aimed = !extend_candidate;
    reason = !nose_forward
        ? LadyLuck::guidance::obfm::
            G5bTurnCueReason::AdversaryNoseOffOwnHalfSpace
        : extend_candidate
        ? LadyLuck::guidance::obfm::
            G5bTurnCueReason::AdversaryNoseLaggingOwnFlightPath
        : LadyLuck::guidance::obfm::G5bTurnCueReason::AdversaryNoseAimed;
}

G5bSafetyAdmissionReceipt SafetyAdmission(
    const DogfightGeometryFrame& frame,
    const G5bSafetyEvidence& safety,
    const bool completed_command_saturation_is_expected,
    const bool running) noexcept
{
    // Age-1 CIS/energy/GCAS fields are completion telemetry, not current
    // maneuver authority. Current finite kinematics and the actual hard-deck
    // margin own this admission. Optional stall/gamma
    // metadata may tighten a known boundary but never erase FCS authority.
    // Keeping optional previous-frame
    // evidence out of the gate prevents an absent diagnostic from cancelling
    // a valid same-frame EXTEND/ZOOM command.
    static_cast<void>(completed_command_saturation_is_expected);
    static_cast<void>(running);
    G5bSafetyAdmissionReceipt output{};
    output.evaluated = true;
    if (!safety.valid)
    {
        output.reason = G5bSafetyAdmissionReason::SafetyObservationMissing;
        return output;
    }
    if (!safety.hard_deck_source_present)
    {
        output.reason = G5bSafetyAdmissionReason::HardDeckSourceMissing;
        return output;
    }
    if (safety.flight_path_gamma_limit_source_present
        && (!std::isfinite(safety.flight_path_gamma_limit_rad)
            || safety.flight_path_gamma_limit_rad <= 0.0
            || safety.flight_path_gamma_limit_rad
                >= 0.5 * LadyLuck::constants::Pi))
    {
        output.reason =
            G5bSafetyAdmissionReason::FlightPathGammaLimitInvalid;
        return output;
    }
    if (!std::isfinite(safety.hard_deck_margin_m)
        || safety.hard_deck_margin_m <= 0.0)
    {
        output.reason =
            G5bSafetyAdmissionReason::HardDeckMarginNotPositive;
        return output;
    }
    if (safety.stall_source_present
        && (!std::isfinite(safety.stall_speed_1g_mps)
            || safety.stall_speed_1g_mps <= 0.0))
    {
        output.reason = G5bSafetyAdmissionReason::StallBoundaryInvalid;
        return output;
    }
    double own_speed = 0.0;
    if (!NumpyNorm3(frame.own.velocity_ned_mps, own_speed))
    {
        output.reason = G5bSafetyAdmissionReason::StallBoundaryInvalid;
        return output;
    }
    if (safety.stall_source_present
        && own_speed <= safety.stall_speed_1g_mps)
    {
        output.reason =
            G5bSafetyAdmissionReason::OneGStallMarginNotPositive;
        return output;
    }
    output.admitted = true;
    output.reason = G5bSafetyAdmissionReason::Admitted;
    return output;
}

G5bReleaseReason ReleaseReason(
    const G5bDelayedClimbObservation& observation) noexcept
{
    if (observation.phase == G5bDelayedClimbPhase::Extend)
    {
        if (!observation.wez_clear)
        {
            return G5bReleaseReason::ExtensionWezReentry;
        }
        if (observation.threat_recovery_margin_evaluated
            && observation.threat_recovery_margin_exhausted)
        {
            return G5bReleaseReason::
                ExtensionThreatRecoveryMarginExhausted;
        }
        if (!observation.running_safety.admitted)
        {
            return G5bReleaseReason::ExtensionRunningSafetyNotAdmitted;
        }
        // The tactical speed floor is vertical-phase evidence.  Its absence
        // withholds ZoomEntry, but it cannot invalidate the already-admitted
        // horizontal EXTEND command.
        return G5bReleaseReason::None;
    }
    if (observation.phase == G5bDelayedClimbPhase::ZoomClimb)
    {
        if (!observation.wez_clear)
        {
            return G5bReleaseReason::ZoomClimbWezReentry;
        }
        if (observation.threat_recovery_margin_evaluated
            && observation.threat_recovery_margin_exhausted)
        {
            return G5bReleaseReason::
                ZoomClimbThreatRecoveryMarginExhausted;
        }
        if (!observation.boom_zoom_eligibility.strict_outward_wez_complete)
        {
            return G5bReleaseReason::
                ZoomClimbStrictCompletionLineageLost;
        }
        if (!observation.boom_zoom_eligibility
                .estimator_chronology_consistent)
        {
            return G5bReleaseReason::
                ZoomClimbEstimatorChronologyLost;
        }
        if (!observation.boom_zoom_eligibility
                .enemy_fire_opportunity_absent)
        {
            return G5bReleaseReason::ZoomClimbEnemyFireOpportunity;
        }
        if (!observation.boom_zoom_eligibility
                .own_energy_advantage_resolved)
        {
            return G5bReleaseReason::ZoomClimbEnergyAdvantageLost;
        }
        if (!observation.boom_zoom_eligibility
                .opponent_vertical_follow_insufficient)
        {
            return G5bReleaseReason::
                ZoomClimbVerticalFollowCapability;
        }
        if (!observation.running_safety.admitted)
        {
            return G5bReleaseReason::ZoomClimbRunningSafetyNotAdmitted;
        }
        if (!observation.speed_floor_admitted)
        {
            return G5bReleaseReason::ZoomClimbSpeedFloorUnavailable;
        }
        if (observation.speed_floor_guard_reached
            && !observation.altitude_gain_observed)
        {
            return G5bReleaseReason::ZoomClimbNoMeasuredAltitudeGain;
        }
    }
    return G5bReleaseReason::None;
}

bool CompleteSelected(
    const G5bDelayedClimbObservation& observation) noexcept
{
    return observation.phase == G5bDelayedClimbPhase::ZoomClimb
        && observation.climb_observed
        && observation.altitude_gain_observed
        && (observation.target_altitude_reached
            || observation.speed_floor_guard_reached);
}

void MaximumPowerSpeedReference(
    const double altitude_m,
    double& output,
    Status& status) noexcept
{
    output = 0.0;
    status = Status{};
    if (!std::isfinite(altitude_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double temperature_k = altitude_m <= kTropopauseAltitudeM
        ? kSeaLevelTemperatureK - kTroposphereLapseKpm * altitude_m
        : kTropopauseTemperatureK;
    output = kCapabilityTableMachEdge * std::sqrt(
        kAirHeatCapacityRatio * kAirSpecificGasConstantJpkgK * temperature_k);
    if (!std::isfinite(output) || output <= 0.0)
    {
        output = 0.0;
        status.code = StatusCode::InvalidConfiguration;
    }
}

bool TaskInputsValid(
    const G16ProductionEvidenceReceipt& current_evidence,
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    const G5bSelectedBranch expected,
    const LadyLuck::guidance::obfm::G5bDelayedClimbSnapshot& snapshot) noexcept
{
    return snapshot.active
        && ValidEvidence(current_evidence)
        && observation.evaluated
        && selection.valid
        && selection.selected_branch == expected
        && selection.observed_phase == observation.phase
        && snapshot.phase == observation.phase
        && LadyLuck::SameControlFrameIdentity(
            current_evidence.frame_identity,
            observation.frame_identity)
        && LadyLuck::SameControlFrameIdentity(
            current_evidence.frame_identity,
            selection.frame_identity);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

G5bSafetyAdmissionReceipt EvaluateG5bSafetyAdmission(
    const DogfightGeometryFrame& frame,
    const G5bSafetyEvidence& safety,
    const bool completed_command_saturation_is_expected,
    const bool running) noexcept
{
    return SafetyAdmission(
        frame,
        safety,
        completed_command_saturation_is_expected,
        running);
}

G5bDelayedClimb::G5bDelayedClimb() noexcept
{
    Reset();
}

void G5bDelayedClimb::ClearState() noexcept
{
    snapshot_ = G5bDelayedClimbSnapshot{};
    cached_observation_valid_ = false;
    cached_observation_ = G5bDelayedClimbObservation{};
}

void G5bDelayedClimb::Reset() noexcept
{
    ClearState();
}

void G5bDelayedClimb::CopySnapshot(
    G5bDelayedClimbSnapshot& output) const noexcept
{
    output = snapshot_;
}

void G5bDelayedClimb::Enter(
    const committed::G16G5bCompletionHandoff& handoff,
    Status& status) noexcept
{
    status = Status{};
    if (snapshot_.active
        || !handoff.valid
        || !handoff.completed_this_sample
        || !IsValidControlFrameIdentity(handoff.frame_identity)
        || !ValidEvidence(handoff.production_evidence)
        || !SameControlFrameIdentity(
            handoff.frame_identity,
            handoff.production_evidence.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    Vector3 direction = handoff.production_evidence.frame.own.velocity_ned_mps;
    direction[2] = 0.0;
    double horizontal_norm = 0.0;
    if (!NumpyNorm3(direction, horizontal_norm))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (horizontal_norm <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    ClearState();
    snapshot_.active = true;
    snapshot_.phase = G5bDelayedClimbPhase::Extend;
    snapshot_.horizontal_direction_valid = true;
    snapshot_.horizontal_direction_ned = Scale(
        direction,
        1.0 / horizontal_norm);
    // G16 creates this handoff only for the typed strict outward-WEZ
    // Completed event.  The identity is retained as lineage evidence; it is
    // never interpreted as a new command owner.
    snapshot_.strict_outward_completion_lineage_valid = true;
    snapshot_.strict_outward_completion_identity = handoff.frame_identity;
    const auto& entry_range =
        handoff.production_evidence.enemy_range_interval;
    if (entry_range.valid
        && std::isfinite(entry_range.upper_m)
        && entry_range.upper_m >= 0.0)
    {
        // A later ZOOM needs measured separation, not merely passage of the
        // WEZ boundary. Current lower > entry upper is the interval proof.
        snapshot_.extend_start_range_upper_m.has_value = true;
        snapshot_.extend_start_range_upper_m.value = entry_range.upper_m;
    }
}

void G5bDelayedClimb::Observe(
    const committed::G16ProductionEvidenceReceipt& current_evidence,
    const G5bSafetyEvidence& safety,
    const G5bSpeedFloorEvidence& speed_floor,
    G5bDelayedClimbObservation& output,
    Status& status) noexcept
{
    output = G5bDelayedClimbObservation{};
    status = Status{};
    if (!snapshot_.active
        || (snapshot_.phase != G5bDelayedClimbPhase::Extend
            && snapshot_.phase != G5bDelayedClimbPhase::ZoomClimb)
        || !snapshot_.horizontal_direction_valid
        || !FiniteVector(snapshot_.horizontal_direction_ned)
        || !ValidEvidence(current_evidence))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (cached_observation_valid_
        && SameControlFrameIdentity(
            cached_observation_.frame_identity,
            current_evidence.frame_identity))
    {
        if (cached_observation_.phase != snapshot_.phase)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        output = cached_observation_;
        return;
    }

    const DogfightGeometryFrame& frame = current_evidence.frame;
    double own_speed = 0.0;
    if (!NumpyNorm3(frame.own.velocity_ned_mps, own_speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    double speed_error = 0.0;
    WorldVelocityErrorBound(frame.own, speed_error, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    const double own_speed_lower = std::nextafter(
        (std::max)(0.0, own_speed - speed_error),
        0.0);

    double altitude_error = 0.0;
    Float32WireVectorErrorBound(
        frame.own.position_ned_m,
        altitude_error,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    const double altitude = -frame.own.position_ned_m[2];
    const double infinity = (std::numeric_limits<double>::infinity)();
    const double altitude_lower = std::nextafter(
        altitude - altitude_error,
        -infinity);
    const double altitude_upper = std::nextafter(
        altitude + altitude_error,
        infinity);
    if (!std::isfinite(own_speed_lower)
        || !std::isfinite(altitude)
        || !std::isfinite(altitude_lower)
        || !std::isfinite(altitude_upper))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    bool opponent_turn_aimed = false;
    G5bTurnCueReason turn_cue_reason = G5bTurnCueReason::AdversaryNoseAimed;
    Status turn_cue_status{};
    OpponentTurnCue(
        frame,
        opponent_turn_aimed,
        turn_cue_reason,
        turn_cue_status);
    if (!turn_cue_status.ok())
    {
        // This cue is diagnostic only. Unobservable nose/LOS geometry must
        // keep the already-admitted horizontal EXTEND command alive and may
        // not become a structural failure of the G5b transaction.
        opponent_turn_aimed = false;
        turn_cue_reason = G5bTurnCueReason::Unobservable;
    }

    const G5bSafetyAdmissionReceipt entry_safety = SafetyAdmission(
        frame,
        safety,
        true,
        false);
    const G5bSafetyAdmissionReceipt running_safety = SafetyAdmission(
        frame,
        safety,
        false,
        true);
    G5bEnergyStandingEvaluation energy_standing{};
    EvaluateEnergyStanding(current_evidence, energy_standing);
    LadyLuck::guidance::ThreatRecoveryMarginReceipt threat_margin{};
    LadyLuck::guidance::EvaluateThreatRecoveryMargin(
        current_evidence.frame,
        current_evidence.own_turn_capability.admitted
            && current_evidence.own_turn_capability.physical_authority,
        current_evidence.own_turn_capability.capability_g,
        threat_margin);
    const bool floor_admitted = speed_floor.valid
        && speed_floor.sample.admitted()
        && std::isfinite(speed_floor.sample.floor_mps.value)
        && speed_floor.sample.floor_mps.value > 0.0;
    const double floor_value = floor_admitted
        ? speed_floor.sample.floor_mps.value
        : 0.0;

    const double vertical_down = frame.own.velocity_ned_mps[2];
    const double vertical_lower = std::nextafter(
        vertical_down - speed_error,
        -infinity);
    const double vertical_upper = std::nextafter(
        vertical_down + speed_error,
        infinity);
    if (!std::isfinite(vertical_lower) || !std::isfinite(vertical_upper))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const G5bVerticalPhase vertical_phase = vertical_upper < 0.0
        ? G5bVerticalPhase::Climbing
        : vertical_lower > 0.0
        ? G5bVerticalPhase::Descending
        : G5bVerticalPhase::UnresolvedZeroInterval;
    const bool climb_now = snapshot_.climb_observed
        || vertical_phase == G5bVerticalPhase::Climbing;
    const bool altitude_gain_observed =
        snapshot_.climb_start_altitude_upper_m.has_value
        && altitude_lower
            > snapshot_.climb_start_altitude_upper_m.value;
    const bool target_reached = snapshot_.target_altitude_m.has_value
        && altitude_lower >= snapshot_.target_altitude_m.value;
    const bool speed_floor_guard_reached = floor_admitted
        && own_speed_lower <= floor_value;
    G5bBoomZoomEligibilityReceipt boom_zoom_eligibility{};
    EvaluateBoomZoomEligibility(
        current_evidence,
        safety,
        speed_floor,
        snapshot_,
        entry_safety,
        opponent_turn_aimed,
        threat_margin,
        own_speed_lower,
        altitude_lower,
        altitude_error,
        boom_zoom_eligibility);
    const bool wez_clear = boom_zoom_eligibility.strict_current_wez_clear;
    const bool climb_entry_admitted =
        snapshot_.phase == G5bDelayedClimbPhase::Extend
        && boom_zoom_eligibility.zoom_admitted;

    G5bObservationReason reason =
        G5bObservationReason::AwaitingResolvedZoomAltitudeBudget;
    if (snapshot_.phase == G5bDelayedClimbPhase::ZoomClimb
        && climb_now
        && altitude_gain_observed
        && (target_reached || speed_floor_guard_reached))
    {
        reason = G5bObservationReason::MeasuredZoomComplete;
    }
    else if (snapshot_.phase == G5bDelayedClimbPhase::ZoomClimb
        && speed_floor_guard_reached
        && !altitude_gain_observed)
    {
        reason =
            G5bObservationReason::ZoomSpeedFloorWithoutMeasuredClimb;
    }
    else if (snapshot_.phase == G5bDelayedClimbPhase::ZoomClimb
        && running_safety.admitted
        && wez_clear)
    {
        reason = G5bObservationReason::ZoomClimbRunning;
    }
    else if (climb_entry_admitted)
    {
        reason = G5bObservationReason::DelayedClimbEntryAdmitted;
    }
    else if (!wez_clear)
    {
        reason = G5bObservationReason::AwaitingStrictWezClear;
    }
    else if (!opponent_turn_aimed)
    {
        reason = G5bObservationReason::AwaitingOpponentTurnCompletion;
    }
    else if (!entry_safety.admitted)
    {
        reason = G5bObservationReason::AwaitingEntrySafetyAdmission;
    }
    else if (!floor_admitted)
    {
        reason = G5bObservationReason::SpeedFloorUnavailable;
    }
    else if (own_speed_lower <= floor_value)
    {
        reason =
            G5bObservationReason::AwaitingResolvedSpeedAboveTacticalFloor;
    }
    else if (!energy_standing.evaluated
        || !energy_standing.advantage_proven)
    {
        reason = G5bObservationReason::AwaitingEnergyAdvantageProof;
    }
    else if (!threat_margin.evaluated)
    {
        reason = G5bObservationReason::AwaitingThreatRecoveryMargin;
    }
    else if (threat_margin.exhausted)
    {
        reason =
            G5bObservationReason::AwaitingPositiveThreatRecoveryMargin;
    }
    else if (!boom_zoom_eligibility.zoom_admitted)
    {
        reason = G5bObservationReason::AwaitingBoomZoomEligibility;
    }

    if (snapshot_.phase == G5bDelayedClimbPhase::ZoomClimb)
    {
        snapshot_.climb_observed = climb_now;
    }
    output.evaluated = true;
    output.frame_identity = current_evidence.frame_identity;
    output.phase = snapshot_.phase;
    output.wez_clear = wez_clear;
    output.energy_standing_evaluated = energy_standing.evaluated;
    output.delta_specific_energy_m =
        energy_standing.delta_specific_energy_m;
    output.energy_evidence_band_m = energy_standing.evidence_band_m;
    output.energy_advantage_proven =
        boom_zoom_eligibility.own_energy_advantage_resolved;
    output.threat_recovery_margin_evaluated = threat_margin.evaluated;
    output.closing_speed_mps = threat_margin.closing_speed_mps;
    output.time_to_enemy_wez_s = IntentOptionalValue<double>{
        threat_margin.time_to_enemy_wez_valid,
        threat_margin.time_to_enemy_wez_s};
    output.own_reversal_time_s = IntentOptionalValue<double>{
        threat_margin.own_reversal_time_valid,
        threat_margin.own_reversal_time_s};
    output.threat_recovery_margin_exhausted = threat_margin.exhausted;
    output.opponent_turn_aimed = opponent_turn_aimed;
    output.opponent_turn_cue_reason = turn_cue_reason;
    output.entry_safety = entry_safety;
    output.running_safety = running_safety;
    output.speed_floor_admitted = floor_admitted;
    output.own_speed_mps = own_speed;
    output.own_speed_lower_mps = own_speed_lower;
    output.tactical_speed_floor_mps.has_value = floor_admitted;
    output.tactical_speed_floor_mps.value = floor_value;
    output.altitude_m = altitude;
    output.altitude_lower_m = altitude_lower;
    output.altitude_upper_m = altitude_upper;
    output.vertical_phase = vertical_phase;
    output.climb_observed = climb_now;
    output.climb_start_altitude_upper_m =
        snapshot_.climb_start_altitude_upper_m;
    output.altitude_gain_observed = altitude_gain_observed;
    output.target_altitude_m = snapshot_.target_altitude_m;
    output.target_altitude_reached = target_reached;
    output.speed_floor_guard_reached = speed_floor_guard_reached;
    output.climb_entry_admitted = climb_entry_admitted;
    output.boom_zoom_eligibility = boom_zoom_eligibility;
    output.reason = reason;
    if (boom_zoom_eligibility.estimator_chronology_consistent)
    {
        snapshot_.last_observation_identity_valid = true;
        snapshot_.last_observation_identity = current_evidence.frame_identity;
    }
    cached_observation_ = output;
    cached_observation_valid_ = true;
}

void G5bDelayedClimb::Select(
    const G5bDelayedClimbObservation& observation,
    G5bDelayedClimbSelection& output,
    Status& status) const noexcept
{
    output = G5bDelayedClimbSelection{};
    status = Status{};
    if (!snapshot_.active
        || !cached_observation_valid_
        || !observation.evaluated
        || !SameControlFrameIdentity(
            observation.frame_identity,
            cached_observation_.frame_identity)
        || observation.phase != cached_observation_.phase
        || observation.phase != snapshot_.phase)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.valid = true;
    output.frame_identity = observation.frame_identity;
    output.observed_phase = observation.phase;
    output.release_reason = ReleaseReason(observation);
    if (output.release_reason != G5bReleaseReason::None)
    {
        output.selected_branch = G5bSelectedBranch::Release;
    }
    else if (CompleteSelected(observation))
    {
        output.selected_branch = G5bSelectedBranch::Complete;
    }
    else
    {
        if (observation.phase == G5bDelayedClimbPhase::Extend
            && observation.climb_entry_admitted)
        {
            output.selected_branch = G5bSelectedBranch::ZoomEntry;
            output.command_task = true;
        }
        else if (observation.phase == G5bDelayedClimbPhase::Extend)
        {
            output.selected_branch = G5bSelectedBranch::Extend;
            output.command_task = true;
        }
        else if (observation.phase == G5bDelayedClimbPhase::ZoomClimb)
        {
            output.selected_branch = G5bSelectedBranch::ZoomClimb;
            output.command_task = true;
        }
        else
        {
            output.selected_branch = G5bSelectedBranch::Invalid;
            output.valid = false;
            status.code = StatusCode::InvalidConfiguration;
        }
    }
}

void G5bDelayedClimb::BuildExtendTask(
    const committed::G16ProductionEvidenceReceipt& current_evidence,
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    ControlIntent& output,
    G5bDelayedClimbTaskReceipt& task,
    Status& status) const noexcept
{
    output.Clear();
    task = G5bDelayedClimbTaskReceipt{};
    status = Status{};
    if (!TaskInputsValid(
            current_evidence,
            observation,
            selection,
            G5bSelectedBranch::Extend,
            snapshot_)
        || !selection.command_task
        || !snapshot_.horizontal_direction_valid)
    {
        FailIntent(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const DogfightGeometryFrame& frame = current_evidence.frame;
    const double altitude_m = -frame.own.position_ned_m[2];
    double speed_reference_mps = 0.0;
    MaximumPowerSpeedReference(altitude_m, speed_reference_mps, status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
        return;
    }
    const double look_distance_m = frame.own_offense.phase.max_range_m;
    double current_speed_mps = 0.0;
    if (!NumpyNorm3(frame.own.velocity_ned_mps, current_speed_mps))
    {
        FailIntent(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (!std::isfinite(look_distance_m)
        || look_distance_m <= 0.0
        || current_speed_mps <= 0.0)
    {
        FailIntent(output, status, StatusCode::InvalidArgument);
        return;
    }

    output.frame_identity = current_evidence.frame_identity;
    output.aim_point_m = Add(
        frame.own.position_ned_m,
        Scale(snapshot_.horizontal_direction_ned, look_distance_m));
    output.desired_speed_mps = (std::max)(
        current_speed_mps,
        speed_reference_mps);
    output.desired_speed_rate_mps2 = 0.0;
    output.specific_energy_rate_bias_m2ps3 = 0.0;
    output.capture_range_des_m = look_distance_m;
    output.behavior_id =
        DoctrineBehaviorId::G5bDelayedClimbExtendMaxPower;
    output.mode_id = DoctrineModeId::ControlZone;
    output.route_kind = ControlRouteKind::AimPoint;
    output.writer_id = ControlIntentWriterG5bDelayedClimb;
    output.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
        return;
    }

    task.valid = true;
    task.frame_identity = current_evidence.frame_identity;
    task.branch = G5bSelectedBranch::Extend;
    task.command_ready = true;
}

void G5bDelayedClimb::BuildZoomIntent(
    const committed::G16ProductionEvidenceReceipt& current_evidence,
    const G5bSafetyEvidence& safety,
    const G5bDelayedClimbSnapshot& snapshot,
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    if (!snapshot.active
        || snapshot.phase != G5bDelayedClimbPhase::ZoomClimb
        || !snapshot.horizontal_direction_valid
        || !snapshot.target_altitude_m.has_value
        || !ValidEvidence(current_evidence)
        || !safety.valid)
    {
        FailIntent(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const DogfightGeometryFrame& frame = current_evidence.frame;
    const double target_altitude_m = snapshot.target_altitude_m.value;
    const double own_altitude_m = -frame.own.position_ned_m[2];
    double gamma_limit_rad = control::route5::Route5GuidanceConfig{}
        .gamma_cmd_limit_rad;
    if (safety.flight_path_gamma_limit_source_present)
    {
        gamma_limit_rad = std::fabs(
            safety.flight_path_gamma_limit_rad);
    }
    const double capture_range_m = frame.own_offense.phase.max_range_m;
    if (!std::isfinite(target_altitude_m)
        || target_altitude_m <= own_altitude_m
        || !std::isfinite(gamma_limit_rad)
        || gamma_limit_rad <= 0.0
        || gamma_limit_rad >= 0.5 * constants::Pi
        || !std::isfinite(capture_range_m)
        || capture_range_m <= 0.0)
    {
        FailIntent(output, status, StatusCode::InvalidArgument);
        return;
    }

    const double altitude_gain_m = target_altitude_m - own_altitude_m;
    const double horizontal_distance_m = (std::max)(
        capture_range_m,
        altitude_gain_m / std::tan(gamma_limit_rad));
    Vector3 aim_point = Add(
        frame.own.position_ned_m,
        Scale(snapshot.horizontal_direction_ned, horizontal_distance_m));
    aim_point[2] = -target_altitude_m;
    const Vector3 path = Subtract(aim_point, frame.own.position_ned_m);
    const double raw_gamma_rad = std::atan2(
        -path[2],
        std::hypot(path[0], path[1]));
    const double admitted_gamma_rad = (std::min)(
        gamma_limit_rad,
        (std::max)(-gamma_limit_rad, raw_gamma_rad));
    double current_speed_mps = 0.0;
    if (!NumpyNorm3(frame.own.velocity_ned_mps, current_speed_mps))
    {
        FailIntent(output, status, StatusCode::NonFiniteInput);
        return;
    }
    const double speed_rate_mps2 = -constants::StandardGravityMps2
        * std::sin(admitted_gamma_rad);
    if (!FiniteVector(aim_point)
        || !std::isfinite(horizontal_distance_m)
        || horizontal_distance_m <= 0.0
        || current_speed_mps <= 0.0
        || !std::isfinite(speed_rate_mps2))
    {
        FailIntent(output, status, StatusCode::NonFiniteInput);
        return;
    }

    output.frame_identity = current_evidence.frame_identity;
    output.aim_point_m = aim_point;
    output.desired_speed_mps = current_speed_mps;
    output.desired_speed_rate_mps2 = speed_rate_mps2;
    output.path_inversion_allowed.has_value = true;
    output.path_inversion_allowed.value = false;
    output.capture_range_des_m = capture_range_m;
    output.behavior_id = DoctrineBehaviorId::G5bDelayedClimbClimbUnload;
    output.mode_id = DoctrineModeId::Obfm;
    output.route_kind = ControlRouteKind::AimPoint;
    output.writer_id = ControlIntentWriterG5bDelayedClimb;
    output.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
    }
}

void G5bDelayedClimb::BuildZoomEntryTask(
    const committed::G16ProductionEvidenceReceipt& current_evidence,
    const G5bSafetyEvidence& safety,
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    ControlIntent& output,
    G5bDelayedClimbTaskReceipt& task,
    Status& status) noexcept
{
    output.Clear();
    task = G5bDelayedClimbTaskReceipt{};
    status = Status{};
    if (!TaskInputsValid(
            current_evidence,
            observation,
            selection,
            G5bSelectedBranch::ZoomEntry,
            snapshot_)
        || !selection.command_task
        || !observation.climb_entry_admitted
        || !observation.tactical_speed_floor_mps.has_value)
    {
        FailIntent(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const double floor_mps = observation.tactical_speed_floor_mps.value;
    const double zoom_gain_m =
        (observation.own_speed_lower_mps
                * observation.own_speed_lower_mps
            - floor_mps * floor_mps)
        / (2.0 * constants::StandardGravityMps2);
    if (!std::isfinite(zoom_gain_m) || zoom_gain_m <= 0.0)
    {
        FailIntent(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    G5bDelayedClimbSnapshot staged_snapshot = snapshot_;
    staged_snapshot.target_altitude_m.has_value = true;
    staged_snapshot.target_altitude_m.value =
        observation.altitude_m + zoom_gain_m;
    staged_snapshot.climb_start_altitude_upper_m.has_value = true;
    staged_snapshot.climb_start_altitude_upper_m.value =
        observation.altitude_upper_m;
    staged_snapshot.climb_observed = false;
    staged_snapshot.phase = G5bDelayedClimbPhase::ZoomClimb;
    BuildZoomIntent(
        current_evidence,
        safety,
        staged_snapshot,
        output,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    snapshot_ = staged_snapshot;

    task.valid = true;
    task.frame_identity = current_evidence.frame_identity;
    task.branch = G5bSelectedBranch::ZoomEntry;
    task.command_ready = true;
}

void G5bDelayedClimb::BuildZoomClimbTask(
    const committed::G16ProductionEvidenceReceipt& current_evidence,
    const G5bSafetyEvidence& safety,
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    ControlIntent& output,
    G5bDelayedClimbTaskReceipt& task,
    Status& status) const noexcept
{
    output.Clear();
    task = G5bDelayedClimbTaskReceipt{};
    status = Status{};
    if (!TaskInputsValid(
            current_evidence,
            observation,
            selection,
            G5bSelectedBranch::ZoomClimb,
            snapshot_)
        || !selection.command_task)
    {
        FailIntent(output, status, StatusCode::InvalidConfiguration);
        return;
    }
    BuildZoomIntent(current_evidence, safety, snapshot_, output, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    task.valid = true;
    task.frame_identity = current_evidence.frame_identity;
    task.branch = G5bSelectedBranch::ZoomClimb;
    task.command_ready = true;
}

void G5bDelayedClimb::CompleteTask(
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    G5bDelayedClimbTaskReceipt& task,
    Status& status) noexcept
{
    task = G5bDelayedClimbTaskReceipt{};
    status = Status{};
    if (!snapshot_.active
        || snapshot_.phase != G5bDelayedClimbPhase::ZoomClimb
        || !observation.evaluated
        || !selection.valid
        || selection.selected_branch != G5bSelectedBranch::Complete
        || !CompleteSelected(observation)
        || !SameControlFrameIdentity(
            observation.frame_identity,
            selection.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.phase = G5bDelayedClimbPhase::Complete;
    task.valid = true;
    task.frame_identity = observation.frame_identity;
    task.branch = G5bSelectedBranch::Complete;
    task.completed_this_sample = true;
}

void G5bDelayedClimb::ReleaseTask(
    const G5bDelayedClimbObservation& observation,
    const G5bDelayedClimbSelection& selection,
    G5bDelayedClimbTaskReceipt& task,
    Status& status) noexcept
{
    task = G5bDelayedClimbTaskReceipt{};
    status = Status{};
    const G5bReleaseReason release_reason = ReleaseReason(observation);
    if (!snapshot_.active
        || !observation.evaluated
        || !selection.valid
        || selection.selected_branch != G5bSelectedBranch::Release
        || selection.release_reason != release_reason
        || release_reason == G5bReleaseReason::None
        || snapshot_.phase != observation.phase
        || !SameControlFrameIdentity(
            observation.frame_identity,
            selection.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.phase = G5bDelayedClimbPhase::Release;
    task.valid = true;
    task.frame_identity = observation.frame_identity;
    task.branch = G5bSelectedBranch::Release;
    task.released_this_sample = true;
    task.release_reason = release_reason;
}

void G5bDelayedClimb::Halt(
    G5bDelayedClimbHaltReceipt& output) noexcept
{
    output = G5bDelayedClimbHaltReceipt{};
    output.valid = true;
    output.was_active = snapshot_.active;
    if (snapshot_.active)
    {
        output.terminal = snapshot_.phase == G5bDelayedClimbPhase::Complete
            || snapshot_.phase == G5bDelayedClimbPhase::Release;
        output.completed =
            snapshot_.phase == G5bDelayedClimbPhase::Complete;
        output.preempted = !output.terminal;
        output.reason = output.preempted
            ? G5bReleaseReason::TreePreempted
            : G5bReleaseReason::None;
    }
    ClearState();
}

void G5bDelayedClimb::Exit() noexcept
{
    ClearState();
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
