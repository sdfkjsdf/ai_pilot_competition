#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/obfm/PursuitPlaneSeparationObserver.hpp"
#include "LadyLuck/guidance/obfm/RollDefenseObserver.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void EvaluateRollingScissorsSignatureCore(
    const RollDefenseObservation* observation,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsPlaneSeparated(
    const RollingScissorsPlaneSeparationReceipt* receipt,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsOwnVerticalActivity(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsOwnClimbing(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsMutualClimb(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsMutualReach(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsStandingReversed(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

void EvaluateRollingScissorsOwnPushedAhead(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

enum class RScissorsHoldSelectionReason : std::uint8_t
{
    SignatureCoreNotResolved = 0U,
    StateNotFinite = 1U,
    AdversaryHorizontalSpeedNotResolved = 2U,
    HoldMaterialized = 3U,
    ContractRejected = 4U
};

const char* RScissorsHoldSelectionReasonLabel(
    RScissorsHoldSelectionReason reason) noexcept;

const char* RScissorsHoldBehaviorLabel() noexcept;

struct RScissorsHoldOverlay
{
    bool valid = false;
    Vector3 aim_point_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
};

struct RScissorsHoldSelectionReceipt
{
    bool selected = false;
    RScissorsHoldSelectionReason reason =
        RScissorsHoldSelectionReason::SignatureCoreNotResolved;
    RScissorsHoldOverlay overlay{};
};

void MaterializeRScissorsHold(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    RScissorsHoldSelectionReceipt& output,
    Status& status) noexcept;

enum class RollingScissorsReleaseAction : std::uint8_t
{
    Maintain = 0U,
    Suspend = 1U,
    Release = 2U
};

enum class RollingScissorsReleaseReason : std::uint8_t
{
    Maintained = 0U,
    StateInvalid = 1U,
    ScissorsLost = 2U,
    StandingReversed = 3U,
    BandExit = 4U,
    SignatureDropped = 5U,
    MutualReachExit = 6U,
    ContractRejected = 7U
};

const char* RollingScissorsReleaseReasonLabel(
    RollingScissorsReleaseReason reason) noexcept;

// Exact R_SCISSORS-state release/suspension decision. The engagement-band
// verdict is supplied by its existing producer; this module does not recreate
// that separate Python contract.
struct RollingScissorsReleaseReceipt
{
    bool evaluated = false;
    RollingScissorsReleaseAction action =
        RollingScissorsReleaseAction::Release;
    RollingScissorsReleaseReason reason =
        RollingScissorsReleaseReason::StateInvalid;
    bool own_pushed_ahead = false;
    bool standing_reversed = false;
    bool engagement_band_resolved = false;
    bool signature_core = false;
    bool mutual_reach = false;
};

void EvaluateRollingScissorsRelease(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    bool in_engagement_band,
    RollingScissorsReleaseReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<RollingScissorsPlaneSeparationReceipt>::value,
    "rolling-scissors plane receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RScissorsHoldSelectionReceipt>::value,
    "rolling-scissors hold selection must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RollingScissorsReleaseReceipt>::value,
    "rolling-scissors release receipt must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
