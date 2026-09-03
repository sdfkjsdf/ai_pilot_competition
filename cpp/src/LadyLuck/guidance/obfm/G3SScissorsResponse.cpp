#include "LadyLuck/guidance/obfm/G3SScissorsResponse.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double OfficialMinimumGunConeRad =
    LadyLuck::constants::Pi / 180.0;
constexpr double CapabilityTableMachEdge = 2.0;

double WrapAngle(const double value) noexcept
{
    double wrapped = std::remainder(value, 2.0 * LadyLuck::constants::Pi);
    if (wrapped <= -LadyLuck::constants::Pi)
    {
        wrapped += 2.0 * LadyLuck::constants::Pi;
    }
    return wrapped;
}

std::int32_t Sign(const double value) noexcept
{
    return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0);
}

double Speed3(const LadyLuck::Vector3& velocity) noexcept
{
    return std::hypot(
        std::hypot(velocity[0], velocity[1]),
        velocity[2]);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

G3ScissorsOwner::G3ScissorsOwner() noexcept
{
    Reset();
}

void G3ScissorsOwner::Reset() noexcept
{
    state_ = G3ScissorsState{};
}

void G3ScissorsOwner::ResetObservation(
    const DogfightGeometryFrame& frame) noexcept
{
    state_ = G3ScissorsState{};
    state_.identity_ready = true;
    state_.episode_epoch = frame.frame_identity.episode_epoch;
    state_.target_plane_id = frame.target_plane_id;
    state_.frame_seen = true;
    state_.last_frame_index = frame.frame_identity.frame_index;
}

bool G3ScissorsOwner::ObserveReversal(
    const DogfightGeometryFrame& frame) noexcept
{
    const double horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    if (!(horizontal_speed_mps > 0.0))
    {
        return false;
    }

    const double course_rad = std::atan2(
        frame.opponent.velocity_ned_mps[1],
        frame.opponent.velocity_ned_mps[0]);
    if (!state_.previous_course_ready)
    {
        state_.previous_course_ready = true;
        state_.previous_course_rad = course_rad;
        return false;
    }

    const double delta_rad = WrapAngle(course_rad - state_.previous_course_rad);
    state_.previous_course_rad = course_rad;
    const std::int32_t turn_sign = Sign(delta_rad);
    if (turn_sign == 0)
    {
        return false;
    }

    if (state_.current_turn_sign == 0)
    {
        state_.current_turn_sign = turn_sign;
        state_.current_turn_sweep_rad = std::fabs(delta_rad);
        return false;
    }
    if (turn_sign == state_.current_turn_sign)
    {
        state_.current_turn_sweep_rad += std::fabs(delta_rad);
    }
    else
    {
        state_.previous_run_qualified =
            state_.current_turn_sweep_rad >= OfficialMinimumGunConeRad;
        state_.current_turn_sign = turn_sign;
        state_.current_turn_sweep_rad = std::fabs(delta_rad);
        state_.current_reversal_reported = false;
    }

    if (state_.previous_run_qualified
        && !state_.current_reversal_reported
        && state_.current_turn_sweep_rad >= OfficialMinimumGunConeRad)
    {
        state_.current_reversal_reported = true;
        return true;
    }
    return false;
}

bool G3ScissorsOwner::OwnAhead(
    const DogfightGeometryFrame& frame) const noexcept
{
    const double target_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    if (!(target_speed_mps > 0.0))
    {
        return false;
    }
    const double course_north =
        frame.opponent.velocity_ned_mps[0] / target_speed_mps;
    const double course_east =
        frame.opponent.velocity_ned_mps[1] / target_speed_mps;
    return (frame.own.position_ned_m[0]
                - frame.opponent.position_ned_m[0]) * course_north
        + (frame.own.position_ned_m[1]
                - frame.opponent.position_ned_m[1]) * course_east > 0.0;
}

bool G3ScissorsOwner::InEngagementBand(
    const DogfightGeometryFrame& frame) const noexcept
{
    return frame.own_offense.range_m
        <= frame.own_offense.phase.max_range_m;
}

bool G3ScissorsOwner::CanBuildGap(
    const DogfightGeometryFrame& frame) const noexcept
{
    const double own_horizontal_speed_mps = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    const double target_horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    return own_horizontal_speed_mps > target_horizontal_speed_mps
        && frame.closing_speed_mps > 0.0
        && InEngagementBand(frame);
}

void G3ScissorsOwner::BuildGapCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3ScissorsReceipt& receipt) const noexcept
{
    const double own_horizontal_speed_mps = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    const double target_horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    const double relative_north =
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0];
    const double relative_east =
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1];
    const double separation_m = std::hypot(relative_north, relative_east);

    command.Clear();
    command.frame_identity = frame.frame_identity;
    command.aim_point_m = frame.opponent.position_ned_m;
    if (own_horizontal_speed_mps > 0.0 && separation_m > 0.0)
    {
        const double course_north =
            frame.own.velocity_ned_mps[0] / own_horizontal_speed_mps;
        const double course_east =
            frame.own.velocity_ned_mps[1] / own_horizontal_speed_mps;
        const double los_north = relative_north / separation_m;
        const double los_east = relative_east / separation_m;
        const double side = course_north * los_east - course_east * los_north;
        const double away_north = side >= 0.0 ? course_east : -course_east;
        const double away_east = side >= 0.0 ? -course_north : course_north;
        const double speed_ratio = (std::min)(
            1.0,
            target_horizontal_speed_mps / own_horizontal_speed_mps);
        const double lateral_ratio = std::sqrt((std::max)(
            0.0,
            1.0 - speed_ratio * speed_ratio));
        command.aim_point_m[0] = frame.own.position_ned_m[0]
            + separation_m
                * (speed_ratio * los_north + lateral_ratio * away_north);
        command.aim_point_m[1] = frame.own.position_ned_m[1]
            + separation_m
                * (speed_ratio * los_east + lateral_ratio * away_east);
        command.aim_point_m[2] = frame.own.position_ned_m[2];
    }
    command.desired_speed_mps = Speed3(frame.own.velocity_ned_mps);
    command.desired_speed_rate_mps2 = 0.0;
    command.behavior_id = DoctrineBehaviorId::ObfmScissorsGapBuild;
    command.mode_id = DoctrineModeId::Obfm;
    command.route_kind = ControlRouteKind::AimPoint;
    command.writer_id = ControlIntentWriterG3Scissors;

    receipt.selected = true;
    receipt.phase = G3ScissorsPhase::GapBuild;
    receipt.writer_id = command.writer_id;
}

void G3ScissorsOwner::BuildEgressCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3ScissorsReceipt& receipt) const noexcept
{
    const double horizontal_speed_mps = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    const double separation_m = (std::max)(
        frame.own_offense.range_m,
        frame.own_offense.phase.max_range_m);
    const double own_altitude_m = -frame.own.position_ned_m[2];
    const double temperature_k = own_altitude_m <= 11000.0
        ? 288.15 - 0.0065 * own_altitude_m
        : 216.65;

    command.Clear();
    command.frame_identity = frame.frame_identity;
    command.aim_point_m = frame.own.position_ned_m;
    if (horizontal_speed_mps > 0.0)
    {
        command.aim_point_m[0] += separation_m
            * frame.own.velocity_ned_mps[0] / horizontal_speed_mps;
        command.aim_point_m[1] += separation_m
            * frame.own.velocity_ned_mps[1] / horizontal_speed_mps;
    }
    command.desired_speed_mps = CapabilityTableMachEdge
        * std::sqrt(1.4 * 287.05287 * temperature_k);
    command.desired_speed_rate_mps2 = 0.0;
    command.behavior_id = DoctrineBehaviorId::ObfmScissorsEgressAccel;
    command.mode_id = DoctrineModeId::Obfm;
    command.route_kind = ControlRouteKind::AimPoint;
    command.writer_id = ControlIntentWriterG3Scissors;

    receipt.selected = true;
    receipt.phase = G3ScissorsPhase::Egress;
    receipt.writer_id = command.writer_id;
}

void G3ScissorsOwner::Evaluate(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3ScissorsReceipt& receipt) noexcept
{
    command.Clear();
    receipt = G3ScissorsReceipt{};
    receipt.evaluated = true;

    if (!state_.identity_ready
        || state_.episode_epoch != frame.frame_identity.episode_epoch
        || state_.target_plane_id != frame.target_plane_id)
    {
        ResetObservation(frame);
    }
    else if (state_.frame_seen
        && state_.last_frame_index == frame.frame_identity.frame_index)
    {
        receipt.phase = state_.phase;
        if (state_.phase == G3ScissorsPhase::GapBuild)
        {
            BuildGapCommand(frame, command, receipt);
        }
        else if (state_.phase == G3ScissorsPhase::Egress)
        {
            BuildEgressCommand(frame, command, receipt);
        }
        return;
    }
    else
    {
        state_.frame_seen = true;
        state_.last_frame_index = frame.frame_identity.frame_index;
    }

    receipt.reversal_observed = ObserveReversal(frame);
    receipt.own_ahead = OwnAhead(frame);

    if (state_.phase == G3ScissorsPhase::GapBuild)
    {
        if (receipt.own_ahead || frame.closing_speed_mps <= 0.0)
        {
            state_.phase = G3ScissorsPhase::Egress;
            receipt.transitioned_to_egress = true;
            BuildEgressCommand(frame, command, receipt);
        }
        else
        {
            BuildGapCommand(frame, command, receipt);
        }
        return;
    }

    if (state_.phase == G3ScissorsPhase::Egress)
    {
        if (!InEngagementBand(frame) || frame.closing_speed_mps > 0.0)
        {
            receipt.released = true;
            receipt.release_reason = !InEngagementBand(frame)
                ? G3ScissorsReleaseReason::OutsideEngagementBand
                : G3ScissorsReleaseReason::EgressNoLongerOpening;
            state_.phase = G3ScissorsPhase::None;
            return;
        }
        BuildEgressCommand(frame, command, receipt);
        return;
    }

    if (receipt.reversal_observed && CanBuildGap(frame))
    {
        state_.phase = G3ScissorsPhase::GapBuild;
        receipt.entered = true;
        BuildGapCommand(frame, command, receipt);
    }
}

void G3ScissorsOwner::Halt() noexcept
{
    Reset();
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
