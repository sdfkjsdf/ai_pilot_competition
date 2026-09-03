#pragma once

#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class RollCounterMode : std::uint8_t
{
    None = 0U,
    BZoom = 1U,
    RScissors = 2U,
    RSuspended = 3U
};

enum class RollCounterReleaseReason : std::uint8_t
{
    None = 0U,
    StateInvalid = 1U,
    ApexOrAdmissionDrop = 2U,
    ScissorsLost = 3U,
    StandingReversed = 4U,
    BandExit = 5U,
    SignatureDropped = 6U,
    MutualReachExit = 7U
};

struct RollCounterLifecycleInput
{
    bool observation_present = false;
    bool observation_valid = false;
    bool observation_admitted = false;
    bool observation_s3_relative = false;
    bool rolling_scissors_enabled = false;
    bool rolling_signature_core = false;
    bool maneuver_planes_separated = false;
    bool mutual_climb = false;
    bool mutual_reach = false;
    bool standing_reversed = false;
    bool own_pushed_ahead = false;
    bool apex_reached = true;
    bool in_engagement_band = false;
    bool within_reach = false;
    bool own_vertical_activity = false;
};

struct RollCounterLifecycleReceipt
{
    RollCounterMode previous_mode = RollCounterMode::None;
    RollCounterMode mode = RollCounterMode::None;
    RollCounterReleaseReason release_reason =
        RollCounterReleaseReason::None;
    bool barrel_roll_counter_engaged = false;
};

class RollCounterLifecycle final
{
public:
    void Reset() noexcept;
    void Update(
        const RollCounterLifecycleInput& input,
        RollCounterLifecycleReceipt& output,
        Status& status) noexcept;
    void CopyReceipt(RollCounterLifecycleReceipt& output) const noexcept;

private:
    RollCounterLifecycleReceipt receipt_{};
};

static_assert(
    std::is_trivially_copyable<RollCounterLifecycleInput>::value,
    "roll-counter lifecycle input must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RollCounterLifecycleReceipt>::value,
    "roll-counter lifecycle receipt must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
