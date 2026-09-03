#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

// Allocation-free production mapping of Python's
// SameIndexGeometryFrameEnvelope. `frame_identity` carries the admitted track
// lineage, source epoch, and sample index. A null pointer at the observer API
// preserves Python None.
struct SameIndexGeometryFrameEnvelope
{
    ControlFrameIdentity frame_identity{};
    double t_sec = 0.0;
};

struct SignedLateralInterval
{
    double nominal = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    std::int32_t resolved_sign = 0;
};

enum class RootGunTowardSideReason : std::uint8_t
{
    RootGunOwnerInactive = 0U,
    SameIndexGeometryFrameEnvelopeMissing = 1U,
    SameIndexGeometryFrameEnvelopeTypeInvalid = 2U,
    RootGunTowardSideGeometryContractRejected = 3U,
    HorizontalNoseUnobservableWithinBounds = 4U,
    AttackerNotResolvedInStrictRearHalfspace = 5U,
    AttackerSideUnresolvedWithinBounds = 6U,
    SelectedRootGunCommandMissing = 7U,
    SelectedRootGunCommandTypeInvalid = 8U,
    SelectedRootGunCommandDirectionUnobservable = 9U,
    SelectedRootGunCommandSideUnresolvedWithinBounds = 10U,
    RootGunTowardSideObservationPublished = 11U,
    RootGunTowardSideObserverContractRejected = 12U
};

const char* RootGunTowardSideReasonLabel(
    RootGunTowardSideReason reason) noexcept;

// This is an observation receipt. It never writes aircraft guidance or FCS
// references. Production Root Gun and prefire entry consume only the resolved
// `toward_side_sign` from this same API.
struct RootGunTowardSideShadowReceipt
{
    bool evaluated = false;
    RootGunTowardSideReason reason =
        RootGunTowardSideReason::RootGunOwnerInactive;

    bool rear_projection_interval_valid = false;
    SignedLateralInterval rear_projection_interval{};
    bool attacker_side_interval_valid = false;
    SignedLateralInterval attacker_side_interval{};
    bool toward_side_sign_valid = false;
    std::int32_t toward_side_sign = 0;

    bool selected_command_side_interval_valid = false;
    SignedLateralInterval selected_command_side_interval{};
    bool selected_command_side_sign_valid = false;
    std::int32_t selected_command_side_sign = 0;
    bool selected_command_matches_toward_valid = false;
    bool selected_command_matches_toward = false;

    // Exact Python authority fields. Approved consumers may read the receipt,
    // but the observer itself owns no direct command authority.
    bool target_simultaneity_authenticated = false;
    bool target_estimator_transaction_authenticated = false;
    bool position_upstream_quantization_authenticated = false;
    bool side_orientation_authority = false;
    bool side_selection_authority = false;
    bool reversal_authority = false;
    bool overshoot_timing_authority = false;
    bool tactical_command_authority = false;
    bool production_authority = false;
    std::uint32_t formal_credit = 0U;
};

void ObserveRootGunTowardSideShadow(
    const DogfightGeometryFrame& frame,
    const SameIndexGeometryFrameEnvelope* envelope,
    const ControlIntent* selected_command,
    bool root_gun_owner_active,
    RootGunTowardSideShadowReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<SameIndexGeometryFrameEnvelope>::value,
    "same-index envelope must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RootGunTowardSideShadowReceipt>::value,
    "Root Gun toward-side receipt must remain allocation-free");

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
