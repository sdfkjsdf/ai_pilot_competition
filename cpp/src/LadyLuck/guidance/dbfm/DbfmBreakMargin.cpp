#include "LadyLuck/guidance/dbfm/DbfmBreakMargin.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/geometry/WezGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::dbfm::DbfmBreakMarginActivation;
using LadyLuck::guidance::dbfm::DbfmBreakMarginObservation;
using LadyLuck::guidance::dbfm::DbfmBreakMarginReason;
using LadyLuck::guidance::dbfm::DbfmBreakMarginTurnCapabilityReceipt;
using LadyLuck::guidance::dbfm::DbfmBreakOfficialThreatObservation;
using LadyLuck::guidance::dbfm::DbfmBreakOfficialThreatReason;

enum class FiniteMathState : std::uint8_t
{
    Available = 0U,
    TooSmall = 1U,
    ArithmeticUnavailable = 2U
};

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
    // Divide the maximum only by a magnitude greater than one.  Besides
    // rejecting product overflow before multiplication, this keeps the guard
    // itself finite when the other operand is tiny.
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

// Live add/main NumPy 1.26.4/MKL three-element reduction, independently
// frozen by the DBFM Extend parity gate: (term0 + term2) + term1.
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
    return output < LadyLuck::constants::Tiny
        ? FiniteMathState::TooSmall
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
    return SafeMultiply(left[0], right[0], product_x)
        && SafeMultiply(left[1], right[1], product_y)
        && SafeMultiply(left[2], right[2], product_z)
        && SafeAdd(product_x, product_z, xz_sum)
        && SafeAdd(xz_sum, product_y, output);
}

bool NormalizeByAcceptedNorm(
    const Vector3& value,
    const double norm,
    Vector3& output) noexcept
{
    return norm >= LadyLuck::constants::Tiny
        && SafeDivide(value[0], norm, output[0])
        && SafeDivide(value[1], norm, output[1])
        && SafeDivide(value[2], norm, output[2]);
}

bool ActivationValid(const DbfmBreakMarginActivation& activation) noexcept
{
    return activation.break_margin_enabled
            == activation.exact_break_margin_provenance
        && activation.phase_graded_enabled
            == activation.exact_phase_graded_provenance;
}

void ObserveOfficialThreat(
    const DogfightGeometryFrame& frame,
    const DbfmBreakMarginActivation& activation,
    DbfmBreakOfficialThreatObservation& output,
    Status& status) noexcept
{
    output = DbfmBreakOfficialThreatObservation{};
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
        // A finite negative proxy is untrusted/missing evidence, not a reason
        // to terminate command production.  Python raises here; the port's
        // required totalized contract returns typed nonselection.
        output.reason =
            DbfmBreakOfficialThreatReason::DamageEvidenceUnavailable;
        return;
    }
    output.damage_evidence_available = true;
    output.official_gun_threat = damage_rate > 0.0;
    if (!output.official_gun_threat)
    {
        output.reason = DbfmBreakOfficialThreatReason::OfficialThreatClear;
        return;
    }
    if (!activation.phase_graded_enabled)
    {
        output.reason =
            DbfmBreakOfficialThreatReason::OfficialThreatSelected;
        return;
    }

    output.scratch_evaluated = true;
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
    output.official_scratch = matched.value.matched
        && matched.value.phase.id == LadyLuck::WezPhaseId::P3;
    if (!output.official_scratch)
    {
        output.reason =
            DbfmBreakOfficialThreatReason::OfficialThreatNotScratch;
        return;
    }

    output.phase_graded_demoted = true;
    output.official_gun_threat = false;
    output.reason = DbfmBreakOfficialThreatReason::
        OfficialScratchDemotedByPhaseGrading;
}

void EvaluateMargin(
    const DogfightGeometryFrame& frame,
    const DbfmBreakMarginTurnCapabilityReceipt& capability,
    DbfmBreakMarginObservation& output,
    Status& status) noexcept
{
    output = DbfmBreakMarginObservation{};
    output.evaluated = true;
    status = Status{};

    if (!capability.admitted)
    {
        output.reason = DbfmBreakMarginReason::CapabilityUnavailable;
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
        output.reason = DbfmBreakMarginReason::CapabilityUnavailable;
        return;
    }

    // Python materializes closing/velocity/speed before validating its
    // range -> active scoring range -> speed tuple.  This totalized port
    // preserves the tuple's semantic order and the valid-domain operation
    // tree, but deliberately lets an earlier finite unavailable value
    // short-circuit a later poisoned calculation instead of reproducing a
    // Python exception.
    const double range_m = frame.enemy_offense.range_m;
    if (!std::isfinite(range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (range_m < LadyLuck::constants::Tiny)
    {
        output.reason = DbfmBreakMarginReason::RangeUnavailable;
        return;
    }

    const double scoring_range_m = frame.enemy_offense.phase.max_range_m;
    if (!std::isfinite(scoring_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (scoring_range_m < LadyLuck::constants::Tiny)
    {
        output.reason = DbfmBreakMarginReason::ScoringRangeUnavailable;
        return;
    }

    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    double own_speed_mps = 0.0;
    const FiniteMathState speed_state =
        PythonNorm3(own_velocity, own_speed_mps);
    if (speed_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    if (speed_state == FiniteMathState::TooSmall)
    {
        output.reason = DbfmBreakMarginReason::OwnSpeedUnavailable;
        return;
    }
    output.own_speed_available = true;
    output.own_speed_mps = own_speed_mps;

    const double closing_mps = frame.closing_speed_mps;
    if (!std::isfinite(closing_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    double scoring_gap_m = 0.0;
    if (!SafeSubtract(range_m, scoring_range_m, scoring_gap_m))
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    output.scoring_gap_available = true;
    output.scoring_gap_m = scoring_gap_m;
    if (scoring_gap_m <= 0.0)
    {
        output.admitted = true;
        output.margin_break = true;
        output.time_to_score_available = true;
        output.time_to_score_s = 0.0;
        output.reason = DbfmBreakMarginReason::AttackerInsideScoringRange;
        return;
    }
    if (closing_mps < LadyLuck::constants::Tiny)
    {
        output.admitted = true;
        output.reason = DbfmBreakMarginReason::AttackerNotClosing;
        return;
    }

    // Python evaluates adversary position before own position in the LOS
    // subtraction expression.
    const Vector3& opponent_position = frame.opponent.position_ned_m;
    const Vector3& own_position = frame.own.position_ned_m;
    if (!FiniteVector(opponent_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!FiniteVector(own_position))
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
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    double separation_m = 0.0;
    const FiniteMathState separation_state =
        PythonNorm3(line_of_sight, separation_m);
    if (separation_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    if (separation_state == FiniteMathState::TooSmall)
    {
        output.reason = DbfmBreakMarginReason::SeparationUnavailable;
        return;
    }

    Vector3 velocity_hat{};
    Vector3 los_hat{};
    if (!NormalizeByAcceptedNorm(
            own_velocity, own_speed_mps, velocity_hat)
        || !NormalizeByAcceptedNorm(
            line_of_sight, separation_m, los_hat))
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    double direction_cosine = 0.0;
    if (!PythonDot3(velocity_hat, los_hat, direction_cosine))
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    direction_cosine = std::max(-1.0, std::min(1.0, direction_cosine));
    const double required_turn_rad = std::acos(direction_cosine);

    // Preserve Python's radius-then-rate operation tree exactly:
    // V*V / (g*sqrt(n*n - 1)), followed by V / radius.
    double load_squared = 0.0;
    double load_minus_one = 0.0;
    if (!SafeMultiply(
            capability.n_max_g, capability.n_max_g, load_squared)
        || !SafeSubtract(load_squared, 1.0, load_minus_one)
        || load_minus_one <= 0.0)
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }
    const double load_radical = std::sqrt(load_minus_one);
    double speed_squared = 0.0;
    double radius_denominator = 0.0;
    double turn_radius_m = 0.0;
    double turn_rate_rad_s = 0.0;
    if (!SafeMultiply(own_speed_mps, own_speed_mps, speed_squared)
        || !SafeMultiply(
            LadyLuck::constants::StandardGravityMps2,
            load_radical,
            radius_denominator)
        || radius_denominator < LadyLuck::constants::Tiny
        || !SafeDivide(
            speed_squared, radius_denominator, turn_radius_m)
        || turn_radius_m < LadyLuck::constants::Tiny
        || !SafeDivide(
            own_speed_mps, turn_radius_m, turn_rate_rad_s)
        || turn_rate_rad_s < LadyLuck::constants::Tiny)
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }

    double time_to_face_s = 0.0;
    double time_to_score_s = 0.0;
    if (!SafeDivide(
            required_turn_rad, turn_rate_rad_s, time_to_face_s)
        || !SafeDivide(scoring_gap_m, closing_mps, time_to_score_s))
    {
        output.reason = DbfmBreakMarginReason::ArithmeticUnavailable;
        return;
    }

    output.admitted = true;
    output.required_turn_available = true;
    output.required_turn_rad = required_turn_rad;
    output.available_turn_rate_available = true;
    output.available_turn_rate_rad_s = turn_rate_rad_s;
    output.time_to_score_available = true;
    output.time_to_score_s = time_to_score_s;
    output.time_to_face_available = true;
    output.time_to_face_s = time_to_face_s;
    output.margin_break = time_to_score_s < time_to_face_s;
    output.reason = output.margin_break
        ? DbfmBreakMarginReason::ScoringCrossingBeforeOwnTurn
        : DbfmBreakMarginReason::OwnTurnCompletesBeforeScoring;
}
}

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

void EvaluateDbfmBreakMargin(
    const DogfightGeometryFrame& frame,
    const bool dbfm_owner_selected,
    const bool higher_priority_root_gun_selected,
    const DbfmBreakMarginActivation& activation,
    const DbfmBreakMarginTurnCapabilityReceipt& capability,
    DbfmBreakMarginReceipt& output,
    Status& status) noexcept
{
    output = DbfmBreakMarginReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};

    if (!dbfm_owner_selected)
    {
        output.reason = DbfmBreakMarginDecisionReason::NonOwner;
        return;
    }
    if (higher_priority_root_gun_selected)
    {
        output.reason =
            DbfmBreakMarginDecisionReason::HigherPriorityRootGun;
        return;
    }
    if (!ActivationValid(activation))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.evaluated = true;
    ObserveOfficialThreat(frame, activation, output.official_threat, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!output.official_threat.damage_evidence_available)
    {
        output.reason =
            DbfmBreakMarginDecisionReason::OfficialEvidenceUnavailable;
        return;
    }
    if (output.official_threat.official_gun_threat)
    {
        output.break_selected = true;
        output.selection_source = DbfmBreakSelectionSource::OfficialGunThreat;
        output.reason =
            DbfmBreakMarginDecisionReason::OfficialGunThreatSelected;
        return;
    }
    if (!activation.break_margin_enabled)
    {
        output.reason = DbfmBreakMarginDecisionReason::BreakMarginDisabled;
        return;
    }

    EvaluateMargin(frame, capability, output.margin, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!output.margin.margin_break)
    {
        output.reason =
            DbfmBreakMarginDecisionReason::BreakMarginNotSelected;
        return;
    }

    output.break_selected = true;
    output.selection_source =
        DbfmBreakSelectionSource::PredictiveBreakMargin;
    output.reason = DbfmBreakMarginDecisionReason::BreakMarginSelected;
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
