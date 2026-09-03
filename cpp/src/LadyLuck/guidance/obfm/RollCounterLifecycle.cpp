#include "LadyLuck/guidance/obfm/RollCounterLifecycle.hpp"

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void RollCounterLifecycle::Reset() noexcept
{
    receipt_ = RollCounterLifecycleReceipt{};
}

void RollCounterLifecycle::Update(
    const RollCounterLifecycleInput& input,
    RollCounterLifecycleReceipt& output,
    Status& status) noexcept
{
    status = Status{};
    RollCounterMode mode = receipt_.mode;
    RollCounterReleaseReason release_reason = receipt_.release_reason;
    const RollCounterMode previous_mode = mode;

    if (!input.observation_present || !input.observation_valid)
    {
        if (mode != RollCounterMode::None)
        {
            release_reason = RollCounterReleaseReason::StateInvalid;
        }
        mode = RollCounterMode::None;
    }
    else if (mode == RollCounterMode::BZoom)
    {
        if (input.rolling_scissors_enabled
            && input.rolling_signature_core
            && input.observation_s3_relative
            && input.maneuver_planes_separated
            && input.mutual_climb
            && input.mutual_reach
            && !input.standing_reversed
            && !input.own_pushed_ahead)
        {
            mode = RollCounterMode::RScissors;
        }
        else if (!input.observation_admitted || input.apex_reached)
        {
            release_reason =
                RollCounterReleaseReason::ApexOrAdmissionDrop;
            mode = RollCounterMode::None;
        }
    }
    else if (mode == RollCounterMode::RScissors)
    {
        if (input.own_pushed_ahead)
        {
            release_reason = RollCounterReleaseReason::ScissorsLost;
            mode = RollCounterMode::None;
        }
        else if (input.standing_reversed)
        {
            release_reason = RollCounterReleaseReason::StandingReversed;
            mode = RollCounterMode::None;
        }
        else if (!input.in_engagement_band)
        {
            release_reason = RollCounterReleaseReason::BandExit;
            mode = RollCounterMode::None;
        }
        else if (!input.rolling_signature_core)
        {
            release_reason = RollCounterReleaseReason::SignatureDropped;
            mode = RollCounterMode::RSuspended;
        }
        else if (!input.mutual_reach)
        {
            release_reason = RollCounterReleaseReason::MutualReachExit;
            mode = RollCounterMode::RSuspended;
        }
    }
    else if (mode == RollCounterMode::RSuspended)
    {
        if (input.own_pushed_ahead)
        {
            release_reason = RollCounterReleaseReason::ScissorsLost;
            mode = RollCounterMode::None;
        }
        else if (input.standing_reversed)
        {
            release_reason = RollCounterReleaseReason::StandingReversed;
            mode = RollCounterMode::None;
        }
        else if (!input.in_engagement_band)
        {
            release_reason = RollCounterReleaseReason::BandExit;
            mode = RollCounterMode::None;
        }
        else if (input.observation_admitted
            && !input.apex_reached
            && input.within_reach)
        {
            mode = RollCounterMode::BZoom;
        }
        else if (input.rolling_signature_core && input.mutual_reach)
        {
            mode = RollCounterMode::RScissors;
        }
    }
    else
    {
        if (input.observation_admitted
            && !input.apex_reached
            && input.in_engagement_band
            && input.within_reach)
        {
            mode = RollCounterMode::BZoom;
        }
        else if (input.rolling_scissors_enabled
            && input.rolling_signature_core
            && input.maneuver_planes_separated
            && input.own_vertical_activity
            && input.in_engagement_band
            && input.mutual_reach
            && !input.standing_reversed
            && !input.own_pushed_ahead)
        {
            mode = RollCounterMode::RScissors;
        }
    }

    receipt_.previous_mode = previous_mode;
    receipt_.mode = mode;
    receipt_.release_reason = release_reason;
    receipt_.barrel_roll_counter_engaged =
        mode == RollCounterMode::BZoom
        || mode == RollCounterMode::RScissors;
    output = receipt_;
}

void RollCounterLifecycle::CopyReceipt(
    RollCounterLifecycleReceipt& output) const noexcept
{
    output = receipt_;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
