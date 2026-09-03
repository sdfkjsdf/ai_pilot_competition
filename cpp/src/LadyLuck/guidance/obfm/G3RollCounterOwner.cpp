#include "LadyLuck/guidance/obfm/G3RollCounterOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{

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

std::int32_t VerticalPhaseSign(
    const LadyLuck::Vector3& velocity_ned_mps) noexcept
{
    const double horizontal_speed_mps = std::hypot(
        velocity_ned_mps[0], velocity_ned_mps[1]);
    if (!(horizontal_speed_mps > 0.0))
    {
        return 0;
    }
    const double flight_path_angle_rad = std::atan2(
        -velocity_ned_mps[2], horizontal_speed_mps);
    const double official_minimum_gun_cone_rad =
        LadyLuck::constants::Pi / 180.0;
    if (std::fabs(flight_path_angle_rad)
        < official_minimum_gun_cone_rad)
    {
        return 0;
    }
    return Sign(velocity_ned_mps[2]);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

G3RollCounterOwner::G3RollCounterOwner() noexcept
{
    Reset();
}

void G3RollCounterOwner::Reset() noexcept
{
    state_ = G3RollCounterState{};
}

void G3RollCounterOwner::ResetObservation(
    const DogfightGeometryFrame& frame,
    const std::int32_t initial_vertical_sign) noexcept
{
    const bool active = state_.active;
    const std::int32_t active_phase_sign = state_.active_phase_sign;
    const G3RollCounterKind active_kind = state_.active_kind;
    state_ = G3RollCounterState{};
    state_.identity_ready = true;
    state_.episode_epoch = frame.frame_identity.episode_epoch;
    state_.target_plane_id = frame.target_plane_id;
    state_.frame_seen = true;
    state_.last_frame_index = frame.frame_identity.frame_index;
    state_.vertical_phase_sign = initial_vertical_sign;
    state_.vertical_phase_count = initial_vertical_sign == 0 ? 0U : 1U;
    state_.phase_start_altitude_m = -frame.opponent.position_ned_m[2];
    state_.active = active;
    state_.active_phase_sign = active_phase_sign;
    state_.active_kind = active_kind;
}

void G3RollCounterOwner::BuildCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3RollCounterReceipt& receipt) const noexcept
{
    command.Clear();
    const double own_speed_mps = std::hypot(
        std::hypot(frame.own.velocity_ned_mps[0],
                   frame.own.velocity_ned_mps[1]),
        frame.own.velocity_ned_mps[2]);
    const double opponent_speed_mps = std::hypot(
        std::hypot(frame.opponent.velocity_ned_mps[0],
                   frame.opponent.velocity_ned_mps[1]),
        frame.opponent.velocity_ned_mps[2]);
    const double own_altitude_m = -frame.own.position_ned_m[2];

    command.frame_identity = frame.frame_identity;
    command.aim_point_m[0] = frame.opponent.position_ned_m[0];
    command.aim_point_m[1] = frame.opponent.position_ned_m[1];
    command.desired_speed_rate_mps2 = 0.0;
    command.mode_id = DoctrineModeId::Obfm;
    command.route_kind = ControlRouteKind::AimPoint;

    const double excess_specific_energy_m = (std::max)(
        0.0,
        (own_speed_mps * own_speed_mps
            - opponent_speed_mps * opponent_speed_mps)
            / (2.0 * constants::StandardGravityMps2));
    command.aim_point_m[2] =
        -(own_altitude_m + excess_specific_energy_m);
    command.desired_speed_mps = opponent_speed_mps;
    command.behavior_id = DoctrineBehaviorId::BrCounterHold;
    command.writer_id = ControlIntentWriterG3CounterBarrel;

    receipt.selected = true;
    receipt.kind = state_.active_kind;
    receipt.writer_id = command.writer_id;
}

void G3RollCounterOwner::BuildRollingScissorsCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3CounterRollingScissorsReceipt& receipt) const noexcept
{
    command.Clear();
    const double opponent_horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    const double opponent_altitude_m = -frame.opponent.position_ned_m[2];

    command.frame_identity = frame.frame_identity;
    command.aim_point_m[0] = frame.opponent.position_ned_m[0];
    command.aim_point_m[1] = frame.opponent.position_ned_m[1];
    command.aim_point_m[2] = -(
        opponent_altitude_m + state_.last_vertical_excursion_m);
    command.desired_speed_mps = opponent_horizontal_speed_mps;
    command.desired_speed_rate_mps2 = 0.0;
    command.behavior_id = DoctrineBehaviorId::RScissorsHold;
    command.mode_id = DoctrineModeId::Obfm;
    command.route_kind = ControlRouteKind::AimPoint;
    command.writer_id = ControlIntentWriterG3CounterRollingScissors;

    receipt.selected = true;
    receipt.vertical_excursion_m = state_.last_vertical_excursion_m;
    receipt.writer_id = command.writer_id;
}

bool G3RollCounterOwner::OwnPushedAhead(
    const DogfightGeometryFrame& frame) const noexcept
{
    const double target_horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    if (!(target_horizontal_speed_mps > 0.0))
    {
        return false;
    }
    const double course_north =
        frame.opponent.velocity_ned_mps[0] / target_horizontal_speed_mps;
    const double course_east =
        frame.opponent.velocity_ned_mps[1] / target_horizontal_speed_mps;
    return (frame.own.position_ned_m[0]
                - frame.opponent.position_ned_m[0]) * course_north
        + (frame.own.position_ned_m[1]
                - frame.opponent.position_ned_m[1]) * course_east > 0.0;
}

void G3RollCounterOwner::Evaluate(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3RollCounterReceipt& receipt) noexcept
{
    command.Clear();
    receipt = G3RollCounterReceipt{};
    receipt.evaluated = true;

    const std::int32_t vertical_sign = VerticalPhaseSign(
        frame.opponent.velocity_ned_mps);
    if (!state_.identity_ready
        || state_.episode_epoch != frame.frame_identity.episode_epoch
        || state_.target_plane_id != frame.target_plane_id)
    {
        Reset();
        ResetObservation(frame, vertical_sign);
    }
    else if (state_.frame_seen
        && state_.last_frame_index == frame.frame_identity.frame_index)
    {
        receipt.kind = state_.active_kind;
        receipt.vertical_phase_count = state_.vertical_phase_count;
        receipt.horizontal_turn_bits = state_.horizontal_turn_bits;
        if (state_.active && frame.closing_speed_mps > 0.0)
        {
            BuildCommand(frame, command, receipt);
        }
        return;
    }
    else
    {
        state_.frame_seen = true;
        state_.last_frame_index = frame.frame_identity.frame_index;
    }

    const double horizontal_speed_mps = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    if (horizontal_speed_mps > 0.0)
    {
        const double course_rad = std::atan2(
            frame.opponent.velocity_ned_mps[1],
            frame.opponent.velocity_ned_mps[0]);
        if (state_.previous_course_ready)
        {
            const double course_delta_rad = WrapAngle(
                course_rad - state_.previous_course_rad);
            if (course_delta_rad > 0.0)
            {
                state_.horizontal_turn_bits |= 1U;
            }
            else if (course_delta_rad < 0.0)
            {
                state_.horizontal_turn_bits |= 2U;
            }
        }
        state_.previous_course_ready = true;
        state_.previous_course_rad = course_rad;
    }

    const bool new_vertical_phase = vertical_sign != 0
        && vertical_sign != state_.vertical_phase_sign;
    receipt.new_vertical_phase = new_vertical_phase;
    if (new_vertical_phase)
    {
        const double opponent_altitude_m =
            -frame.opponent.position_ned_m[2];
        state_.last_vertical_excursion_m = std::fabs(
            opponent_altitude_m - state_.phase_start_altitude_m);
        state_.phase_start_altitude_m = opponent_altitude_m;
        state_.vertical_phase_sign = vertical_sign;
        if (state_.rolling_active)
        {
            state_.rolling_release_pending = true;
        }
        if (state_.active)
        {
            state_.active = false;
            state_.active_phase_sign = 0;
            state_.active_kind = G3RollCounterKind::None;
            state_.rolling_handoff_ready = true;
            receipt.released = true;
            receipt.release_reason =
                G3RollCounterReleaseReason::VerticalPhaseChanged;
            receipt.vertical_phase_count = state_.vertical_phase_count;
            return;
        }
        if (state_.vertical_phase_count < 3U)
        {
            ++state_.vertical_phase_count;
        }
    }
    else if (state_.vertical_phase_sign == 0 && vertical_sign != 0)
    {
        state_.vertical_phase_sign = vertical_sign;
        state_.vertical_phase_count = 1U;
    }

    if (state_.active && frame.closing_speed_mps <= 0.0)
    {
        state_.active = false;
        state_.active_phase_sign = 0;
        state_.active_kind = G3RollCounterKind::None;
        ResetObservation(frame, vertical_sign);
        receipt.released = true;
        receipt.release_reason = G3RollCounterReleaseReason::RangeOpening;
        receipt.vertical_phase_count = state_.vertical_phase_count;
        return;
    }

    if (!state_.active
        && !state_.rolling_handoff_ready
        && !state_.rolling_active
        && !state_.rolling_release_pending
        && state_.vertical_phase_count >= 3U
        && state_.horizontal_turn_bits != 0U
        && frame.closing_speed_mps > 0.0)
    {
        state_.active = true;
        state_.active_phase_sign = state_.vertical_phase_sign;
        state_.active_kind = G3RollCounterKind::CounterBarrel;
    }

    receipt.kind = state_.active_kind;
    receipt.vertical_phase_count = state_.vertical_phase_count;
    receipt.horizontal_turn_bits = state_.horizontal_turn_bits;
    if (state_.active)
    {
        BuildCommand(frame, command, receipt);
    }
}

void G3RollCounterOwner::EvaluateCounterRollingScissors(
    const DogfightGeometryFrame& frame,
    ControlIntent& command,
    G3CounterRollingScissorsReceipt& receipt) noexcept
{
    command.Clear();
    receipt = G3CounterRollingScissorsReceipt{};
    receipt.evaluated = true;
    receipt.vertical_excursion_m = state_.last_vertical_excursion_m;

    if (state_.rolling_release_pending)
    {
        receipt.released = true;
        Reset();
        ResetObservation(
            frame,
            VerticalPhaseSign(frame.opponent.velocity_ned_mps));
        return;
    }
    receipt.own_pushed_ahead = OwnPushedAhead(frame);
    if (state_.rolling_active && receipt.own_pushed_ahead)
    {
        receipt.released = true;
        Reset();
        ResetObservation(
            frame,
            VerticalPhaseSign(frame.opponent.velocity_ned_mps));
        return;
    }
    if (!state_.rolling_active && state_.rolling_handoff_ready)
    {
        state_.rolling_handoff_ready = false;
        state_.rolling_active = true;
        state_.rolling_active_phase_sign = state_.vertical_phase_sign;
        receipt.handoff_consumed = true;
    }
    if (state_.rolling_active)
    {
        BuildRollingScissorsCommand(frame, command, receipt);
    }
}

void G3RollCounterOwner::HaltCounterBarrel() noexcept
{
    state_.active = false;
    state_.active_phase_sign = 0;
    state_.active_kind = G3RollCounterKind::None;
    if (!state_.rolling_active)
    {
        state_.rolling_handoff_ready = false;
    }
}

void G3RollCounterOwner::HaltCounterRollingScissors() noexcept
{
    Reset();
}

void G3RollCounterOwner::Halt() noexcept
{
    Reset();
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
