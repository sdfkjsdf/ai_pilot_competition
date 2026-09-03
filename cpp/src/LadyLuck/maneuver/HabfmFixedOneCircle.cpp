#include "LadyLuck/maneuver/HabfmFixedOneCircle.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

LadyLuck::Result<LadyLuck::Vector3> HorizontalUnit(
    const LadyLuck::Vector3& value) noexcept
{
    LadyLuck::Result<LadyLuck::Vector3> result{};
    if (!FiniteVector(value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    LadyLuck::Vector3 horizontal = value;
    horizontal[2] = 0.0;
    const double magnitude = VectorNorm(horizontal);
    if (!std::isfinite(magnitude))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    if (magnitude < LadyLuck::constants::Tiny)
    {
        result.status.code = LadyLuck::StatusCode::InvalidArgument;
        return result;
    }
    result.value = LadyLuck::Vector3{{
        horizontal[0] / magnitude,
        horizontal[1] / magnitude,
        horizontal[2] / magnitude}};
    return result;
}

LadyLuck::Result<double> HeadingRad(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Result<double> result{};
    const LadyLuck::Result<LadyLuck::Vector3> nose = HorizontalUnit(
        frame.own.nose_ned);
    if (!nose.ok())
    {
        result.status = nose.status;
        return result;
    }
    result.value = std::atan2(nose.value[1], nose.value[0]);
    if (!std::isfinite(result.value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return result;
}

LadyLuck::Result<std::int32_t> EntrySide(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Result<std::int32_t> result{};
    const LadyLuck::Result<LadyLuck::Vector3> own_nose = HorizontalUnit(
        frame.own.nose_ned);
    if (!own_nose.ok())
    {
        result.status = own_nose.status;
        return result;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    const LadyLuck::Vector3 line_of_sight{{
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0],
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1],
        frame.opponent.position_ned_m[2] - frame.own.position_ned_m[2]}};
    const LadyLuck::Result<LadyLuck::Vector3> los = HorizontalUnit(
        line_of_sight);
    if (!los.ok())
    {
        result.status = los.status;
        return result;
    }
    const double cross_down =
        own_nose.value[0] * los.value[1]
        - own_nose.value[1] * los.value[0];
    result.value = cross_down >= 0.0 ? 1 : -1;
    return result;
}

double WrappedDelta(
    const double current_rad,
    const double previous_rad) noexcept
{
    const double difference = current_rad - previous_rad;
    return std::atan2(std::sin(difference), std::cos(difference));
}
}

namespace LadyLuck
{
Result<TacticalCommand> BuildHabfmFixedOneCircleCommand(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign)
{
    Result<TacticalCommand> result{};
    HabfmFixedOneCircleReference reference{};
    BuildHabfmFixedOneCircleReference(
        frame,
        side_sign,
        reference,
        result.status);
    if (result.status.code != StatusCode::Ok)
    {
        return result;
    }

    TacticalCommand candidate{};
    candidate.aim_point_m = reference.aim_point_m;
    candidate.desired_speed_mps = reference.desired_speed_mps;
    candidate.capture_range_des_m = reference.capture_range_des_m;
    try
    {
        candidate.behavior_label = "ONE_CIRCLE";
        candidate.mode_label = "HABFM";
    }
    catch (...)
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }
    return MakeValidatedTacticalCommand(candidate);
}

HabfmFixedOneCirclePolicy::HabfmFixedOneCirclePolicy() noexcept
{
    Reset();
}

void HabfmFixedOneCirclePolicy::Reset() noexcept
{
    active_ = false;
    side_sign_ = 0;
    previous_heading_valid_ = false;
    previous_heading_rad_ = 0.0;
    progress_rad_ = 0.0;
    previous_closing_valid_ = false;
    previous_closing_ = false;
}

HabfmFixedOneCircleSnapshot HabfmFixedOneCirclePolicy::Snapshot() const noexcept
{
    HabfmFixedOneCircleSnapshot snapshot{};
    snapshot.active = active_;
    snapshot.side_sign.has_value = active_;
    snapshot.side_sign.value = side_sign_;
    snapshot.previous_heading_rad.has_value = previous_heading_valid_;
    snapshot.previous_heading_rad.value = previous_heading_rad_;
    snapshot.progress_rad = progress_rad_;
    snapshot.previous_closing.has_value = previous_closing_valid_;
    snapshot.previous_closing.value = previous_closing_;
    return snapshot;
}

Result<HabfmFixedOneCircleOutput>
HabfmFixedOneCirclePolicy::FailAndReset(const Status& status)
{
    Reset();
    Result<HabfmFixedOneCircleOutput> result{};
    result.status = status;
    if (result.status.ok())
    {
        result.status.code = StatusCode::InvalidConfiguration;
    }
    return result;
}

Result<HabfmFixedOneCircleOutput> HabfmFixedOneCirclePolicy::StepLeg(
    const DogfightGeometryFrame& frame,
    const std::uint64_t blackboard_neutral_cue_streak)
{
    const bool entered = !active_;
    if (entered)
    {
        const Result<std::int32_t> side = EntrySide(frame);
        if (!side.ok())
        {
            return FailAndReset(side.status);
        }
        const Result<double> entry_heading = HeadingRad(frame);
        if (!entry_heading.ok())
        {
            return FailAndReset(entry_heading.status);
        }
        active_ = true;
        side_sign_ = side.value;
        previous_heading_valid_ = true;
        previous_heading_rad_ = entry_heading.value;
        progress_rad_ = 0.0;
    }

    if (!active_
        || (side_sign_ != -1 && side_sign_ != 1)
        || !previous_heading_valid_)
    {
        Status failure{};
        failure.code = StatusCode::InvalidConfiguration;
        return FailAndReset(failure);
    }

    const Result<double> heading = HeadingRad(frame);
    if (!heading.ok())
    {
        return FailAndReset(heading.status);
    }
    const double delta = WrappedDelta(
        heading.value,
        previous_heading_rad_);
    progress_rad_ += static_cast<double>(side_sign_) * delta;
    previous_heading_rad_ = heading.value;

    const Result<HabfmCheckpointCueEvidence> cue =
        EvaluateCheckpointCue(frame);
    if (!cue.ok())
    {
        return FailAndReset(cue.status);
    }

    const double closing_speed_mps = frame.closing_speed_mps;
    if (!std::isfinite(closing_speed_mps))
    {
        Status failure{};
        failure.code = StatusCode::NonFiniteInput;
        return FailAndReset(failure);
    }
    const bool merge_pass = previous_closing_valid_
        && previous_closing_
        && closing_speed_mps < 0.0;
    if (closing_speed_mps != 0.0)
    {
        previous_closing_valid_ = true;
        previous_closing_ = closing_speed_mps > 0.0;
    }

    std::uint64_t completed_neutral_streak = blackboard_neutral_cue_streak;
    if (merge_pass)
    {
        if (cue.value.cue == HabfmCheckpointCueState::Neutral)
        {
            if (completed_neutral_streak
                == (std::numeric_limits<std::uint64_t>::max)())
            {
                Status failure{};
                failure.code = StatusCode::InvalidConfiguration;
                return FailAndReset(failure);
            }
            ++completed_neutral_streak;
        }
        else
        {
            completed_neutral_streak = 0U;
        }
    }

    Result<TacticalCommand> command{};
    if (!merge_pass)
    {
        command = BuildHabfmFixedOneCircleCommand(frame, side_sign_);
        if (!command.ok())
        {
            return FailAndReset(command.status);
        }
    }

    Result<HabfmFixedOneCircleOutput> result{};
    try
    {
        result.value.leg_status = merge_pass
            ? HabfmFixedOneCircleLegStatus::MergePass
            : HabfmFixedOneCircleLegStatus::Running;
        result.value.entered = entered;
        result.value.merge_pass = merge_pass;
        result.value.side_sign = side_sign_;
        result.value.turn_progress_rad = progress_rad_;
        result.value.mode_recheck = merge_pass;
        result.value.neutral_cue_streak = completed_neutral_streak;
        result.value.checkpoint_cue = cue.value;
        if (merge_pass)
        {
            result.value.command.has_value = false;
            result.value.transition_reason.has_value = true;
            result.value.transition_reason.value = "one_circle_merge_pass";
        }
        else
        {
            result.value.command.has_value = true;
            result.value.command.value = std::move(command.value);
            result.value.transition_reason.has_value = false;
        }
    }
    catch (...)
    {
        Status failure{};
        failure.code = StatusCode::InvalidConfiguration;
        return FailAndReset(failure);
    }

    if (merge_pass)
    {
        // BehaviorTree Task SUCCESS invokes on_exit in the same tick.  Return
        // the completed-leg output, but expose reset-seeded policy state.
        Reset();
    }
    return result;
}
}
