#include "LadyLuck/guidance/dbfm/DbfmEscapeAdmission.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/FiniteMathState.hpp"
#include "LadyLuck/geometry/WezGeometry.hpp"
#include "LadyLuck/geometry/WezRule.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using LadyLuck::ControlFrameIdentity;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::common::FiniteMathState;
using LadyLuck::guidance::dbfm::DbfmDefenseUrgencyReason;
using LadyLuck::guidance::dbfm::DbfmDefenseUrgencyReceipt;
using LadyLuck::guidance::dbfm::DbfmEscapeEligibilityReason;
using LadyLuck::guidance::dbfm::DbfmEscapeEligibilityReceipt;
using LadyLuck::guidance::dbfm::DbfmEscapeTurnCapabilityReceipt;
using LadyLuck::guidance::dbfm::DbfmOfficialScratchReason;
using LadyLuck::guidance::dbfm::DbfmOfficialScratchReceipt;
using LadyLuck::guidance::dbfm::DbfmScratchEntryForecastReceipt;

constexpr double Float32RelativeQuantum = 1.1920928955078125e-7;
// Frozen Python math.sqrt(3.0) * 0.0003048 result.
constexpr double BodyVelocityNormQuantumMps = 0.0005279290861469938;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool SafeSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = std::numeric_limits<double>::max();
    if ((right > 0.0 && left <= -maximum + right)
        || (right < 0.0 && left >= maximum + right))
    {
        return false;
    }
    output = left - right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
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
    const double maximum = std::numeric_limits<double>::max();
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
    const double left_magnitude = std::fabs(left);
    const double right_magnitude = std::fabs(right);
    const double maximum = std::numeric_limits<double>::max();
    if ((left_magnitude > 1.0
            && right_magnitude >= maximum / left_magnitude)
        || (right_magnitude > 1.0
            && left_magnitude >= maximum / right_magnitude))
    {
        return false;
    }
    output = left * right;
    if (!std::isfinite(output))
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
    const double numerator_magnitude = std::fabs(numerator);
    const double denominator_magnitude = std::fabs(denominator);
    const double maximum = std::numeric_limits<double>::max();
    if (denominator_magnitude < 1.0
        && numerator_magnitude >= maximum * denominator_magnitude)
    {
        return false;
    }
    output = numerator / denominator;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

FiniteMathState PythonNorm3(
    const Vector3& value,
    double& output) noexcept
{
    double x_squared = 0.0;
    double y_squared = 0.0;
    double z_squared = 0.0;
    double xz_squared = 0.0;
    double sum_squared = 0.0;
    if (!SafeMultiply(value[0], value[0], x_squared)
        || !SafeMultiply(value[1], value[1], y_squared)
        || !SafeMultiply(value[2], value[2], z_squared)
        || !SafeAdd(x_squared, z_squared, xz_squared)
        || !SafeAdd(xz_squared, y_squared, sum_squared))
    {
        output = 0.0;
        return FiniteMathState::ArithmeticUnavailable;
    }
    output = std::sqrt(sum_squared);
    return output == 0.0
        ? FiniteMathState::Degenerate
        : FiniteMathState::Available;
}

bool PythonDot3(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    double product_x = 0.0;
    double product_y = 0.0;
    double product_z = 0.0;
    double xz_sum = 0.0;
    if (!SafeMultiply(left[0], right[0], product_x)
        || !SafeMultiply(left[1], right[1], product_y)
        || !SafeMultiply(left[2], right[2], product_z)
        || !SafeAdd(product_x, product_z, xz_sum)
        || !SafeAdd(xz_sum, product_y, output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool NormalizeByAcceptedNorm(
    const Vector3& value,
    const double norm,
    Vector3& output) noexcept
{
    return norm > 0.0
        && std::isfinite(norm)
        && SafeDivide(value[0], norm, output[0])
        && SafeDivide(value[1], norm, output[1])
        && SafeDivide(value[2], norm, output[2]);
}

bool ActivationValid(
    const LadyLuck::guidance::dbfm::DbfmEscapeAdmissionActivation&
        activation) noexcept
{
    return activation.phase_graded_enabled
            == activation.exact_phase_graded_provenance
        && activation.entry_forecast_enabled
            == activation.exact_entry_forecast_provenance;
}

void ObserveOfficialScratch(
    const DogfightGeometryFrame& frame,
    DbfmOfficialScratchReceipt& output,
    Status& status) noexcept
{
    output = DbfmOfficialScratchReceipt{};
    output.evaluated = true;
    status = Status{};
    const double damage_rate = frame.enemy_offense.damage_rate;
    if (!std::isfinite(damage_rate))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (damage_rate < 0.0)
    {
        output.reason = DbfmOfficialScratchReason::DamageValueUnavailable;
        return;
    }
    if (damage_rate == 0.0)
    {
        output.reason = DbfmOfficialScratchReason::DamageNotPositive;
        return;
    }

    const LadyLuck::Result<LadyLuck::WezPhaseMatch> matched =
        LadyLuck::MatchWezPhase(
            frame.enemy_offense.range_m,
            frame.enemy_offense.ata_rad,
            frame.t_sec);
    if (matched.status.code != StatusCode::Ok)
    {
        status = matched.status;
        return;
    }
    if (!matched.value.matched)
    {
        output.reason = DbfmOfficialScratchReason::NoOfficialPhaseMatch;
        return;
    }
    if (matched.value.phase.id != LadyLuck::WezPhaseId::P3)
    {
        output.reason = DbfmOfficialScratchReason::HigherPriorityGunBand;
        return;
    }
    output.scratch_matched = true;
    output.reason = DbfmOfficialScratchReason::ScratchBandMatched;
}

void EvaluateDefenseUrgency(
    const DogfightGeometryFrame& frame,
    const DbfmEscapeTurnCapabilityReceipt& capability,
    DbfmDefenseUrgencyReceipt& output,
    Status& status) noexcept
{
    output = DbfmDefenseUrgencyReceipt{};
    output.evaluated = true;
    status = Status{};

    if (!capability.admitted)
    {
        output.reason = DbfmDefenseUrgencyReason::CapabilityUnavailable;
        return;
    }
    if (!capability.n_max_available
        || !LadyLuck::IsValidControlFrameIdentity(capability.frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            capability.frame_identity,
            frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!std::isfinite(capability.n_max_g))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (capability.n_max_g <= 1.0)
    {
        output.reason = DbfmDefenseUrgencyReason::CapabilityUnavailable;
        return;
    }

    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    double own_speed_mps = 0.0;
    const FiniteMathState own_speed_state =
        PythonNorm3(own_velocity, own_speed_mps);
    if (own_speed_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    if (own_speed_state == FiniteMathState::Degenerate)
    {
        output.reason = DbfmDefenseUrgencyReason::OwnSpeedUnavailable;
        return;
    }
    output.own_speed_available = true;
    output.own_speed_mps = own_speed_mps;

    const Vector3& opponent_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(opponent_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    double opponent_speed_mps = 0.0;
    const FiniteMathState opponent_speed_state =
        PythonNorm3(opponent_velocity, opponent_speed_mps);
    if (opponent_speed_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    if (opponent_speed_state == FiniteMathState::Degenerate)
    {
        output.reason = DbfmDefenseUrgencyReason::OpponentSpeedUnavailable;
        return;
    }
    output.opponent_speed_available = true;
    output.opponent_speed_mps = opponent_speed_mps;

    const double range_m = frame.enemy_offense.range_m;
    if (!std::isfinite(range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (range_m <= 0.0)
    {
        output.reason = DbfmDefenseUrgencyReason::RangeUnavailable;
        return;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& opponent_position = frame.opponent.position_ned_m;
    if (!FiniteVector(own_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!FiniteVector(opponent_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    Vector3 line_of_sight{};
    if (!SafeSubtract(
            opponent_position[0], own_position[0], line_of_sight[0])
        || !SafeSubtract(
            opponent_position[1], own_position[1], line_of_sight[1])
        || !SafeSubtract(
            opponent_position[2], own_position[2], line_of_sight[2]))
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    double separation_m = 0.0;
    const FiniteMathState separation_state =
        PythonNorm3(line_of_sight, separation_m);
    if (separation_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    if (separation_state == FiniteMathState::Degenerate)
    {
        output.reason = DbfmDefenseUrgencyReason::SeparationUnavailable;
        return;
    }

    Vector3 velocity_hat{};
    Vector3 los_hat{};
    if (!NormalizeByAcceptedNorm(
            own_velocity, own_speed_mps, velocity_hat)
        || !NormalizeByAcceptedNorm(
            line_of_sight, separation_m, los_hat))
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    double direction_cosine = 0.0;
    if (!PythonDot3(velocity_hat, los_hat, direction_cosine))
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    direction_cosine = std::max(-1.0, std::min(1.0, direction_cosine));
    const double required_turn_rad = std::acos(direction_cosine);

    double load_squared = 0.0;
    if (!SafeMultiply(
            capability.n_max_g, capability.n_max_g, load_squared)
        || load_squared <= 1.0)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    double load_minus_one = 0.0;
    if (!SafeSubtract(load_squared, 1.0, load_minus_one)
        || load_minus_one <= 0.0)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }
    const double load_radical = std::sqrt(load_minus_one);
    double speed_squared = 0.0;
    double radius_denominator = 0.0;
    double turn_radius_m = 0.0;
    double available_turn_rate_rad_s = 0.0;
    if (!SafeMultiply(own_speed_mps, own_speed_mps, speed_squared)
        || !SafeMultiply(
            LadyLuck::constants::StandardGravityMps2,
            load_radical,
            radius_denominator)
        || radius_denominator <= 0.0
        || !SafeDivide(
            speed_squared, radius_denominator, turn_radius_m)
        || turn_radius_m <= 0.0
        || !SafeDivide(
            own_speed_mps,
            turn_radius_m,
            available_turn_rate_rad_s)
        || available_turn_rate_rad_s <= 0.0)
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }

    double time_to_face_s = 0.0;
    double attacker_reach_time_s = 0.0;
    if (!SafeDivide(
            required_turn_rad,
            available_turn_rate_rad_s,
            time_to_face_s)
        || !SafeDivide(
            range_m,
            opponent_speed_mps,
            attacker_reach_time_s))
    {
        output.reason = DbfmDefenseUrgencyReason::ArithmeticUnavailable;
        return;
    }

    output.admitted = true;
    output.urgent_available = true;
    output.urgent = attacker_reach_time_s < time_to_face_s;
    output.required_turn_available = true;
    output.required_turn_rad = required_turn_rad;
    output.time_to_face_available = true;
    output.time_to_face_s = time_to_face_s;
    output.attacker_reach_time_available = true;
    output.attacker_reach_time_s = attacker_reach_time_s;
    output.reason = output.urgent
        ? DbfmDefenseUrgencyReason::AttackerInsideOwnTurnCircle
        : DbfmDefenseUrgencyReason::AttackerOutsideOwnTurnCircle;
}

void EvaluateEligibility(
    const DogfightGeometryFrame& frame,
    const DbfmEscapeTurnCapabilityReceipt& capability,
    DbfmEscapeEligibilityReceipt& output,
    Status& status) noexcept
{
    output = DbfmEscapeEligibilityReceipt{};
    output.evaluated = true;
    EvaluateDefenseUrgency(frame, capability, output.urgency, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!output.urgency.admitted || !output.urgency.urgent_available)
    {
        output.reason =
            DbfmEscapeEligibilityReason::CapabilityOrKinematicsUnavailable;
        return;
    }
    if (output.urgency.urgent)
    {
        output.reason = DbfmEscapeEligibilityReason::DefenseUrgent;
        return;
    }

    const double own_speed_mps = output.urgency.own_speed_mps;
    const double opponent_speed_mps = output.urgency.opponent_speed_mps;
    double own_relative_error = 0.0;
    double opponent_relative_error = 0.0;
    double own_error = 0.0;
    double opponent_error = 0.0;
    double evidence_band = 0.0;
    double speed_delta = 0.0;
    if (!SafeMultiply(
            std::fabs(own_speed_mps),
            Float32RelativeQuantum,
            own_relative_error)
        || !SafeMultiply(
            std::fabs(opponent_speed_mps),
            Float32RelativeQuantum,
            opponent_relative_error)
        || !SafeAdd(
            BodyVelocityNormQuantumMps,
            own_relative_error,
            own_error)
        || !SafeAdd(
            BodyVelocityNormQuantumMps,
            opponent_relative_error,
            opponent_error)
        || !SafeAdd(own_error, opponent_error, evidence_band)
        || !SafeSubtract(own_speed_mps, opponent_speed_mps, speed_delta))
    {
        output.reason =
            DbfmEscapeEligibilityReason::SpeedBandArithmeticUnavailable;
        return;
    }

    output.speed_evidence_available = true;
    output.own_speed_mps = own_speed_mps;
    output.opponent_speed_mps = opponent_speed_mps;
    output.speed_delta_mps = speed_delta;
    output.speed_evidence_band_mps = evidence_band;
    output.selected = speed_delta > evidence_band;
    output.reason = output.selected
        ? DbfmEscapeEligibilityReason::Selected
        : DbfmEscapeEligibilityReason::SpeedAdvantageNotProven;
}

bool HistoryCompatible(
    const ControlFrameIdentity& previous,
    const ControlFrameIdentity& current,
    const std::int32_t previous_own_plane_id,
    const std::int32_t previous_target_plane_id,
    const std::uint64_t previous_target_frame_index,
    const DogfightGeometryFrame& frame) noexcept
{
    return LadyLuck::IsValidControlFrameIdentity(previous)
        && LadyLuck::IsValidControlFrameIdentity(current)
        && previous.episode_epoch == current.episode_epoch
        && previous.frame_index < current.frame_index
        && previous_own_plane_id >= 0
        && previous_target_plane_id >= 0
        && previous_own_plane_id == frame.own_plane_id
        && previous_target_plane_id == frame.target_plane_id
        && previous_target_frame_index <= frame.target_frame_index;
}

void ClearUnadmittedForecastEvidence(
    DbfmScratchEntryForecastReceipt& output) noexcept
{
    output.admitted = false;
    output.entry_forming = false;
    output.cone_satisfied = false;
    output.cone_closing_proven = false;
    output.range_satisfied = false;
    output.range_closing_proven = false;
    output.time_to_cone_available = false;
    output.time_to_cone_s = 0.0;
    output.time_to_range_available = false;
    output.time_to_range_s = 0.0;
}
}

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{
void DbfmEscapeAdmissionEvaluator::Reset() noexcept
{
    history_valid_ = false;
    history_frame_identity_ = ControlFrameIdentity{};
    history_own_plane_id_ = -1;
    history_target_plane_id_ = -1;
    history_target_frame_index_ = 0U;
    previous_t_s_ = 0.0;
    previous_ata_rad_ = 0.0;
}

void DbfmEscapeAdmissionEvaluator::UpdateEntryForecast(
    const DogfightGeometryFrame& frame,
    DbfmScratchEntryForecastReceipt& output,
    Status& status) noexcept
{
    output = DbfmScratchEntryForecastReceipt{};
    output.frame_identity = frame.frame_identity;
    output.evaluated = true;
    status = Status{};

    const double t_s = frame.t_sec;
    const double ata_rad = frame.enemy_offense.ata_rad;
    const double range_m = frame.enemy_offense.range_m;
    if (!std::isfinite(t_s)
        || !std::isfinite(ata_rad)
        || !std::isfinite(range_m))
    {
        Reset();
        output.reason = DbfmScratchEntryForecastReason::FrameStateUnavailable;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (range_m <= 0.0)
    {
        Reset();
        output.reason = DbfmScratchEntryForecastReason::FrameStateUnavailable;
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps))
    {
        Reset();
        output.reason = DbfmScratchEntryForecastReason::FrameStateUnavailable;
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const bool compatible = history_valid_
        && HistoryCompatible(
            history_frame_identity_,
            frame.frame_identity,
            history_own_plane_id_,
            history_target_plane_id_,
            history_target_frame_index_,
            frame);
    const bool previous_available = compatible;
    const double previous_t_s = previous_t_s_;
    const double previous_ata_rad = previous_ata_rad_;
    output.causal_history_reset = history_valid_ && !compatible;

    history_valid_ = true;
    history_frame_identity_ = frame.frame_identity;
    history_own_plane_id_ = frame.own_plane_id;
    history_target_plane_id_ = frame.target_plane_id;
    history_target_frame_index_ = frame.target_frame_index;
    previous_t_s_ = t_s;
    previous_ata_rad_ = ata_rad;

    const Result<WezPhase> active = ActiveWezPhase(t_s);
    const Result<WezPhase> scratch =
        OfficialWezPhaseAt(OfficialWezPhaseCount - 1U);
    if (active.status.code != StatusCode::Ok
        || scratch.status.code != StatusCode::Ok)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.scratch_active = active.value.id == scratch.value.id;
    if (!output.scratch_active)
    {
        output.reason =
            DbfmScratchEntryForecastReason::ScratchPhaseNotActive;
        return;
    }

    Vector3 relative_position{};
    Vector3 relative_velocity{};
    if (!SafeSubtract(
            frame.opponent.position_ned_m[0],
            frame.own.position_ned_m[0],
            relative_position[0])
        || !SafeSubtract(
            frame.opponent.position_ned_m[1],
            frame.own.position_ned_m[1],
            relative_position[1])
        || !SafeSubtract(
            frame.opponent.position_ned_m[2],
            frame.own.position_ned_m[2],
            relative_position[2])
        || !SafeSubtract(
            frame.opponent.velocity_ned_mps[0],
            frame.own.velocity_ned_mps[0],
            relative_velocity[0])
        || !SafeSubtract(
            frame.opponent.velocity_ned_mps[1],
            frame.own.velocity_ned_mps[1],
            relative_velocity[1])
        || !SafeSubtract(
            frame.opponent.velocity_ned_mps[2],
            frame.own.velocity_ned_mps[2],
            relative_velocity[2]))
    {
        output.reason =
            DbfmScratchEntryForecastReason::ArithmeticUnavailable;
        return;
    }
    double separation_m = 0.0;
    const FiniteMathState separation_state =
        PythonNorm3(relative_position, separation_m);
    if (separation_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason =
            DbfmScratchEntryForecastReason::ArithmeticUnavailable;
        return;
    }
    if (separation_state == FiniteMathState::Degenerate)
    {
        output.reason =
            DbfmScratchEntryForecastReason::DegenerateSeparation;
        return;
    }
    double range_rate_numerator = 0.0;
    double range_rate_mps = 0.0;
    if (!PythonDot3(
            relative_position,
            relative_velocity,
            range_rate_numerator)
        || !SafeDivide(
            range_rate_numerator, separation_m, range_rate_mps))
    {
        output.reason =
            DbfmScratchEntryForecastReason::ArithmeticUnavailable;
        return;
    }

    output.range_satisfied = range_m <= scratch.value.max_range_m;
    output.range_closing_proven = range_rate_mps < 0.0;
    if (output.range_satisfied)
    {
        output.time_to_range_available = true;
        output.time_to_range_s = 0.0;
    }
    else if (output.range_closing_proven)
    {
        double range_gap_m = 0.0;
        if (!SafeSubtract(
                range_m, scratch.value.max_range_m, range_gap_m)
            || !SafeDivide(
                range_gap_m,
                -range_rate_mps,
                output.time_to_range_s))
        {
            ClearUnadmittedForecastEvidence(output);
            output.reason =
                DbfmScratchEntryForecastReason::ArithmeticUnavailable;
            return;
        }
        output.time_to_range_available = true;
    }

    output.cone_satisfied = ata_rad <= scratch.value.angle_rad;
    if (output.cone_satisfied)
    {
        output.time_to_cone_available = true;
        output.time_to_cone_s = 0.0;
    }
    else
    {
        if (!previous_available)
        {
            output.reason = DbfmScratchEntryForecastReason::
                ConeSweepMemoryNotEstablished;
            return;
        }
        double dt_s = 0.0;
        if (!SafeSubtract(t_s, previous_t_s, dt_s) || dt_s <= 0.0)
        {
            ClearUnadmittedForecastEvidence(output);
            output.reason =
                DbfmScratchEntryForecastReason::NonMonotonicTime;
            return;
        }
        double delta_ata_rad = 0.0;
        if (!SafeSubtract(ata_rad, previous_ata_rad, delta_ata_rad))
        {
            ClearUnadmittedForecastEvidence(output);
            output.reason =
                DbfmScratchEntryForecastReason::ArithmeticUnavailable;
            return;
        }
        const double evidence_scale = std::max(
            std::max(std::fabs(ata_rad), std::fabs(previous_ata_rad)),
            1.0);
        double evidence_band = 0.0;
        if (!SafeMultiply(
                evidence_scale,
                Float32RelativeQuantum,
                evidence_band))
        {
            ClearUnadmittedForecastEvidence(output);
            output.reason =
                DbfmScratchEntryForecastReason::ArithmeticUnavailable;
            return;
        }
        if (delta_ata_rad < -evidence_band)
        {
            double sweep_rate_rad_s = 0.0;
            double cone_gap_rad = 0.0;
            if (!SafeDivide(-delta_ata_rad, dt_s, sweep_rate_rad_s)
                || sweep_rate_rad_s <= 0.0
                || !SafeSubtract(
                    ata_rad, scratch.value.angle_rad, cone_gap_rad)
                || !SafeDivide(
                    cone_gap_rad,
                    sweep_rate_rad_s,
                    output.time_to_cone_s))
            {
                ClearUnadmittedForecastEvidence(output);
                output.reason =
                    DbfmScratchEntryForecastReason::ArithmeticUnavailable;
                return;
            }
            output.cone_closing_proven = true;
            output.time_to_cone_available = true;
        }
    }

    output.entry_forming =
        (output.cone_satisfied || output.cone_closing_proven)
        && (output.range_satisfied || output.range_closing_proven)
        && !(output.cone_satisfied && output.range_satisfied);
    output.admitted = true;
    output.reason = output.entry_forming
        ? DbfmScratchEntryForecastReason::EntryForming
        : DbfmScratchEntryForecastReason::EntryNotForming;
}

void DbfmEscapeAdmissionEvaluator::Evaluate(
    const DogfightGeometryFrame& frame,
    const bool dbfm_owner_selected,
    const bool escape_branch_reached,
    const DbfmEscapeAdmissionActivation& activation,
    const DbfmEscapeTurnCapabilityReceipt& capability,
    DbfmEscapeAdmissionReceipt& output,
    Status& status) noexcept
{
    output = DbfmEscapeAdmissionReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};

    if (!dbfm_owner_selected)
    {
        Reset();
        output.reason = DbfmEscapeAdmissionReason::NonOwner;
        return;
    }
    if (!escape_branch_reached)
    {
        output.reason = DbfmEscapeAdmissionReason::HigherPriorityBranch;
        return;
    }
    if (!ActivationValid(activation))
    {
        Reset();
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!activation.phase_graded_enabled)
    {
        output.reason = DbfmEscapeAdmissionReason::ProductionDisabled;
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        Reset();
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.evaluated = true;
    ObserveOfficialScratch(frame, output.official_scratch, status);
    if (status.code != StatusCode::Ok)
    {
        Reset();
        return;
    }
    if (output.official_scratch.scratch_matched)
    {
        EvaluateEligibility(
            frame,
            capability,
            output.damage_scratch_eligibility,
            status);
        if (status.code != StatusCode::Ok)
        {
            Reset();
            return;
        }
        if (output.damage_scratch_eligibility.selected)
        {
            output.selected = true;
            output.selection_source =
                DbfmEscapeSelectionSource::DamagePositiveScratch;
            output.reason = DbfmEscapeAdmissionReason::DamageScratchSelected;
            return;
        }
    }

    if (!activation.entry_forecast_enabled)
    {
        output.entry_forecast.reason =
            DbfmScratchEntryForecastReason::ProductionDisabled;
        output.reason = output.official_scratch.scratch_matched
            ? DbfmEscapeAdmissionReason::DamageScratchNotEligible
            : DbfmEscapeAdmissionReason::NotDamagePositiveScratch;
        return;
    }

    UpdateEntryForecast(frame, output.entry_forecast, status);
    if (status.code != StatusCode::Ok)
    {
        Reset();
        return;
    }
    if (!output.entry_forecast.admitted
        || !output.entry_forecast.entry_forming)
    {
        output.reason = DbfmEscapeAdmissionReason::EntryForecastNotForming;
        return;
    }

    EvaluateEligibility(
        frame,
        capability,
        output.entry_eligibility,
        status);
    if (status.code != StatusCode::Ok)
    {
        Reset();
        return;
    }
    if (!output.entry_eligibility.selected)
    {
        output.reason = DbfmEscapeAdmissionReason::EntryForecastNotEligible;
        return;
    }

    output.selected = true;
    output.selection_source =
        DbfmEscapeSelectionSource::PreDamageEntryForecast;
    output.reason = DbfmEscapeAdmissionReason::EntryForecastSelected;
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
