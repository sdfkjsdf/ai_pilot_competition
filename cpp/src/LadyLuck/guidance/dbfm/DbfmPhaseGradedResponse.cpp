#include "LadyLuck/guidance/dbfm/DbfmPhaseGradedResponse.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/geometry/WezGeometry.hpp"

#include <cmath>
#include <limits>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
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
    if ((right > 0.0
            && left >= std::numeric_limits<double>::max() - right)
        || (right < 0.0
            && left <= -std::numeric_limits<double>::max() - right))
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
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if ((right < 0.0
            && left >= std::numeric_limits<double>::max() + right)
        || (right > 0.0
            && left <= -std::numeric_limits<double>::max() + right))
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
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    if ((absolute_left > 1.0
            && absolute_right
                >= std::numeric_limits<double>::max() / absolute_left)
        || (absolute_right > 1.0
            && absolute_left
                >= std::numeric_limits<double>::max() / absolute_right))
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

// Preserve NumPy's three-term reduction order used by add/main@45abc while
// making finite overflow an ordinary unavailable result.
bool PythonNorm3(
    const LadyLuck::Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double head_tail = 0.0;
    double squared = 0.0;
    if (!SafeMultiply(value[0], value[0], term0)
        || !SafeMultiply(value[1], value[1], term1)
        || !SafeMultiply(value[2], value[2], term2)
        || !SafeAdd(term0, term2, head_tail)
        || !SafeAdd(head_tail, term1, squared)
        || squared < 0.0)
    {
        return false;
    }
    output = std::sqrt(squared);
    return std::isfinite(output);
}

bool HorizontalUnit(
    const LadyLuck::Vector3& value,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    const LadyLuck::Vector3 horizontal{{value[0], value[1], 0.0}};
    double magnitude = 0.0;
    if (!PythonNorm3(horizontal, magnitude)
        || magnitude < LadyLuck::constants::Tiny)
    {
        return false;
    }
    output = LadyLuck::Vector3{{
        horizontal[0] / magnitude,
        horizontal[1] / magnitude,
        0.0}};
    return FiniteVector(output);
}

struct RearGeometry
{
    LadyLuck::Vector3 own_nose_hat{};
    LadyLuck::Vector3 attacker_los_hat{};
    bool available = false;
    bool strict_rear = false;
    std::int32_t toward_side_sign = 0;
};

bool PrepareRearGeometry(
    const LadyLuck::DogfightGeometryFrame& frame,
    const std::int32_t tie_side,
    RearGeometry& output) noexcept
{
    output = RearGeometry{};
    if (tie_side != -1 && tie_side != 1)
    {
        return false;
    }
    if (!FiniteVector(frame.own.nose_ned)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !HorizontalUnit(frame.own.nose_ned, output.own_nose_hat))
    {
        return false;
    }

    LadyLuck::Vector3 attacker_offset{};
    if (!SafeSubtract(
            frame.opponent.position_ned_m[0],
            frame.own.position_ned_m[0],
            attacker_offset[0])
        || !SafeSubtract(
            frame.opponent.position_ned_m[1],
            frame.own.position_ned_m[1],
            attacker_offset[1])
        || !SafeSubtract(
            frame.opponent.position_ned_m[2],
            frame.own.position_ned_m[2],
            attacker_offset[2])
        || !HorizontalUnit(attacker_offset, output.attacker_los_hat))
    {
        return false;
    }

    const double rear_projection =
        output.own_nose_hat[0] * output.attacker_los_hat[0]
        + output.own_nose_hat[1] * output.attacker_los_hat[1];
    const double cross =
        output.own_nose_hat[0] * output.attacker_los_hat[1]
        - output.own_nose_hat[1] * output.attacker_los_hat[0];
    if (!std::isfinite(rear_projection) || !std::isfinite(cross))
    {
        return false;
    }

    output.available = true;
    output.strict_rear = rear_projection < 0.0;
    output.toward_side_sign = cross > 0.0
        ? -1
        : (cross < 0.0 ? 1 : tie_side);
    return true;
}

bool OfficialScratch(
    const LadyLuck::DogfightGeometryFrame& frame,
    bool& matched) noexcept
{
    matched = false;
    const double damage_rate = frame.enemy_offense.damage_rate;
    const double range_m = frame.enemy_offense.range_m;
    const double ata_rad = frame.enemy_offense.ata_rad;
    if (!std::isfinite(damage_rate)
        || !std::isfinite(range_m)
        || !std::isfinite(ata_rad)
        || !std::isfinite(frame.t_sec))
    {
        return false;
    }
    if (damage_rate <= 0.0)
    {
        return true;
    }

    const LadyLuck::Result<LadyLuck::WezPhaseMatch> phase =
        LadyLuck::MatchWezPhase(range_m, ata_rad, frame.t_sec);
    if (phase.status.code != LadyLuck::StatusCode::Ok)
    {
        // A finite value outside the official function domain is ordinary
        // replacement unavailability; the valid Root BREAK remains selected.
        return true;
    }
    matched = phase.value.matched
        && phase.value.phase.id == LadyLuck::WezPhaseId::P3;
    return true;
}

bool BuildAimIntent(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::Vector3& raw_direction,
    const LadyLuck::DoctrineBehaviorId behavior,
    const std::uint32_t writer_id,
    LadyLuck::ControlIntent& output) noexcept
{
    output.Clear();
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(raw_direction))
    {
        return false;
    }

    LadyLuck::Vector3 direction{};
    double speed_mps = 0.0;
    const double range_m = frame.enemy_offense.range_m;
    const double capture_range_m = frame.enemy_offense.phase.max_range_m;
    if (!HorizontalUnit(raw_direction, direction)
        || !PythonNorm3(frame.own.velocity_ned_mps, speed_mps)
        || !std::isfinite(range_m)
        || !std::isfinite(capture_range_m)
        || range_m <= 0.0
        || capture_range_m <= 0.0
        || speed_mps <= 0.0)
    {
        return false;
    }

    double north_offset = 0.0;
    double east_offset = 0.0;
    double aim_north = 0.0;
    double aim_east = 0.0;
    if (!SafeMultiply(range_m, direction[0], north_offset)
        || !SafeMultiply(range_m, direction[1], east_offset)
        || !SafeAdd(
            frame.own.position_ned_m[0], north_offset, aim_north)
        || !SafeAdd(
            frame.own.position_ned_m[1], east_offset, aim_east))
    {
        return false;
    }

    LadyLuck::ControlIntent candidate{};
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = LadyLuck::Vector3{{
        aim_north,
        aim_east,
        frame.own.position_ned_m[2]}};
    candidate.desired_speed_mps = speed_mps;
    candidate.capture_range_des_m = capture_range_m;
    candidate.behavior_id = behavior;
    candidate.mode_id = LadyLuck::DoctrineModeId::Dbfm;
    candidate.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    candidate.writer_id = writer_id;
    LadyLuck::Status validation{};
    candidate.Validate(validation);
    if (!validation.ok())
    {
        return false;
    }
    output = candidate;
    return true;
}

bool BuildSidePreservingHardTurn(
    const LadyLuck::DogfightGeometryFrame& frame,
    const RearGeometry& geometry,
    const std::int32_t held_side,
    LadyLuck::ControlIntent& output) noexcept
{
    if (!geometry.available
        || !geometry.strict_rear
        || (held_side != -1 && held_side != 1)
        || geometry.toward_side_sign != held_side)
    {
        output.Clear();
        return false;
    }

    const LadyLuck::Vector3 lateral{{
        -geometry.attacker_los_hat[1],
        geometry.attacker_los_hat[0],
        0.0}};
    const LadyLuck::Vector3 break_direction{{
        static_cast<double>(held_side) * lateral[0],
        static_cast<double>(held_side) * lateral[1],
        0.0}};
    LadyLuck::Vector3 bisector{{
        geometry.own_nose_hat[0] + break_direction[0],
        geometry.own_nose_hat[1] + break_direction[1],
        0.0}};
    double bisector_norm = 0.0;
    if (!PythonNorm3(bisector, bisector_norm))
    {
        output.Clear();
        return false;
    }
    if (bisector_norm < LadyLuck::constants::Tiny)
    {
        bisector = break_direction;
    }
    return BuildAimIntent(
        frame,
        bisector,
        LadyLuck::DoctrineBehaviorId::DbfmHardTurn,
        LadyLuck::ControlIntentWriterDbfmHardTurn,
        output);
}
}

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

bool DbfmPhaseGradedArithmeticBoundarySafeForTesting() noexcept
{
    const double maximum = (std::numeric_limits<double>::max)();
    const double unit_2_971 = std::ldexp(1.0, 971);
    const double three_units_2_970 = 3.0 * std::ldexp(1.0, 970);
    const double positive_left = maximum - unit_2_971;
    const double negative_left = -maximum + unit_2_971;
    double output = 1.0;
    const bool positive_add_rejected = !SafeAdd(
        positive_left,
        three_units_2_970,
        output) && output == 0.0;
    output = 1.0;
    const bool negative_add_rejected = !SafeAdd(
        negative_left,
        -three_units_2_970,
        output) && output == 0.0;
    output = 1.0;
    const bool positive_subtract_rejected = !SafeSubtract(
        positive_left,
        -three_units_2_970,
        output) && output == 0.0;
    output = 1.0;
    const bool negative_subtract_rejected = !SafeSubtract(
        negative_left,
        three_units_2_970,
        output) && output == 0.0;
    const double multiplier = 1.5;
    const double multiplicand = maximum / multiplier;
    output = 1.0;
    const bool positive_multiply_rejected = !SafeMultiply(
        multiplier,
        multiplicand,
        output) && output == 0.0;
    output = 1.0;
    const bool negative_multiply_rejected = !SafeMultiply(
        -multiplier,
        multiplicand,
        output) && output == 0.0;
    return maximum - three_units_2_970 == positive_left
        && -maximum + three_units_2_970 == negative_left
        && positive_add_rejected
        && negative_add_rejected
        && positive_subtract_rejected
        && negative_subtract_rejected
        && positive_multiply_rejected
        && negative_multiply_rejected;
}

void DbfmPhaseGradedResponseProvider::Reset() noexcept
{
}

void DbfmPhaseGradedResponseProvider::Evaluate(
    const DogfightGeometryFrame& frame,
    const bool root_gun_owner_selected,
    const GunDefenseSnapshot& gun_episode,
    const ControlIntent& base_break,
    DbfmPhaseGradedResponseReceipt& receipt,
    ControlIntent& output,
    Status& status) noexcept
{
    receipt = DbfmPhaseGradedResponseReceipt{};
    receipt.frame_identity = frame.frame_identity;
    output = base_break;
    status = Status{};

    Status base_status{};
    base_break.Validate(base_status);
    if (!base_status.ok()
        || !IsValidControlFrameIdentity(frame.frame_identity)
        || !SameControlFrameIdentity(
            base_break.frame_identity,
            frame.frame_identity)
        || base_break.behavior_id
            != DoctrineBehaviorId::GunDefenseHorizontalBreak
        || base_break.mode_id != DoctrineModeId::Dbfm
        || base_break.writer_id
            != ControlIntentWriterGunDefenseHorizontalBreak)
    {
        output.Clear();
        receipt.reason =
            DbfmPhaseGradedResponseReason::BaseBreakContractInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    receipt.base_break_preserved = true;
    if (base_break.route_kind != ControlRouteKind::AimPoint)
    {
        receipt.reason = DbfmPhaseGradedResponseReason::
            BaseBreakRouteUnavailable;
        return;
    }
    if (!root_gun_owner_selected)
    {
        Reset();
        receipt.reason = DbfmPhaseGradedResponseReason::NonOwner;
        return;
    }
    receipt.evaluated = true;

    if (!gun_episode.toward_side_candidate_held
        || (gun_episode.side_sign != -1 && gun_episode.side_sign != 1))
    {
        receipt.reason =
            DbfmPhaseGradedResponseReason::HeldSideUnavailable;
        return;
    }

    bool official_scratch = false;
    if (!OfficialScratch(frame, official_scratch))
    {
        receipt.reason =
            DbfmPhaseGradedResponseReason::NotOfficialScratch;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    receipt.official_scratch = official_scratch;
    if (!official_scratch)
    {
        receipt.reason =
            DbfmPhaseGradedResponseReason::NotOfficialScratch;
        return;
    }

    RearGeometry geometry{};
    if (!PrepareRearGeometry(frame, gun_episode.side_sign, geometry))
    {
        receipt.reason =
            DbfmPhaseGradedResponseReason::RearGeometryUnavailable;
        return;
    }
    receipt.strict_rear_geometry_available = true;
    receipt.attacker_in_strict_rear = geometry.strict_rear;
    receipt.current_toward_side_available = true;
    receipt.current_toward_side_sign = geometry.toward_side_sign;
    if (!geometry.strict_rear)
    {
        receipt.reason =
            DbfmPhaseGradedResponseReason::AttackerNotStrictRear;
        return;
    }
    receipt.held_side_preserved =
        geometry.toward_side_sign == gun_episode.side_sign;
    if (!receipt.held_side_preserved)
    {
        receipt.reason = DbfmPhaseGradedResponseReason::
            HeldSideNoLongerTowardAttacker;
        return;
    }
    receipt.outer_admitted = true;

    ControlIntent candidate{};
    if (!BuildSidePreservingHardTurn(
            frame,
            geometry,
            gun_episode.side_sign,
            candidate))
    {
        receipt.reason = DbfmPhaseGradedResponseReason::
            HardTurnMaterializationUnavailable;
        return;
    }
    receipt.selected_branch = DbfmPhaseGradedResponseBranch::HardTurn;
    receipt.reason = DbfmPhaseGradedResponseReason::HardTurnSelected;

    output = candidate;
    receipt.replacement_available = true;
    receipt.base_break_preserved = false;
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
