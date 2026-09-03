#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/g4/HighGBarrelEvidence.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

constexpr DoctrineBehaviorId DoctrineBehaviorG4WezShortestExit =
    DoctrineBehaviorId::G4WezShortestExit;

enum class HighGBarrelVariant : std::uint8_t
{
    None = 0U,
    Underneath = 1U,
    OverTheTop = 2U
};

enum class HighGBarrelPhase : std::uint8_t
{
    None = 0U,
    WezShortestExit = 1U
};

enum class HighGBarrelDecision : std::uint8_t
{
    Unavailable = 0U,
    RootPassthrough = 1U,
    ReleasePassthrough = 2U,
    Underneath = 3U,
    OverTheTop = 4U
};

enum class HighGBarrelReason : std::uint8_t
{
    Unavailable = 0U,
    EntryNotAdmitted = 1U,
    OfficialRootGunOwnerInactive = 2U,
    CausalAttackFormUnavailable = 3U,
    CausalContinuousTrackingNotEstablished = 4U,
    EntrySafetyRejected = 5U,
    PreviousControlFeedbackNotCausal = 6U,
    PreviousCommandNotRootGunDefense = 7U,
    PreviousLoadedResponseIncomplete = 8U,
    PreviousTotalLoadGovernorInconsistent = 9U,
    PreviousCoreLoadedResponseNotObserved = 10U,
    G13CoreExhaustionEvidenceMissing = 11U,
    G13ReverseOwnerSenior = 12U,
    BoundedBreakScissorsExhaustionNotEstablished = 13U,
    AttackerTrackingLossObserved = 14U,
    AttackerTrackingUnresolved = 15U,
    RunningSafetyRejected = 16U,
    CommittedControlFeedbackNotCausal = 17U,
    PreviousG4CommandNotCompleted = 18U,
    G13ContinuationEvidenceMissing = 19U,
    G13ContinuationSourceUnresolved = 20U,
    AttackerPassageObserved = 21U,
    AttackerPassageUnresolved = 22U,
    AttackerCounterTurnObserved = 23U,
    AttackerTurnCommitmentUnresolved = 24U,
    FarSteadyLosVeto = 25U,
    LosVetoUnresolved = 26U,
    LoadedRollCommandUnavailable = 27U,
    Admitted = 28U,
    CurrentThreeDimensionalGeometryUnavailable = 29U,
    CurrentPhysicalLoadUnavailable = 30U,
    ManeuverCompleted = 31U
};

struct HighGBarrelOwnerSnapshot
{
    bool engaged = false;
    HighGBarrelVariant variant = HighGBarrelVariant::None;
    HighGBarrelPhase phase = HighGBarrelPhase::None;
    double requested_load_magnitude_g = 0.0;
    HighGBarrelReason last_release_reason = HighGBarrelReason::Unavailable;
};

// Service-owned command-neutral selection receipt.  The provided BT must use
// its decision in visible Conditions; BuildCandidate is called only by the
// selected Underneath or Over-the-Top leaf Task.
struct HighGBarrelSelectionReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    HighGBarrelDecision decision = HighGBarrelDecision::Unavailable;
    HighGBarrelReason reason = HighGBarrelReason::Unavailable;
    bool engaged_before = false;
    bool entry_admitted = false;
    bool entered_if_published = false;
    bool released_if_published = false;
    HighGBarrelVariant selected_variant = HighGBarrelVariant::None;
    double requested_load_magnitude_g = 0.0;
    HighGBarrelVerticalExcessEvidence vertical_excess{};
};

struct HighGBarrelTaskReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool candidate_available = false;
    bool root_passthrough_required = false;
    bool entered_this_tick = false;
    HighGBarrelVariant variant = HighGBarrelVariant::None;
    HighGBarrelPhase phase = HighGBarrelPhase::None;
    HighGBarrelReason reason = HighGBarrelReason::Unavailable;
    Vector3 desired_bank_direction_ned{};
    Vector3 acceleration_ned_mps2{};
    double load_magnitude_g = 0.0;
    bool los_projection_valid = false;
    Vector3 los_direction_velocity_normal_ned{};
    double los_pull_alignment = 0.0;
};

class HighGBarrelOwner final
{
public:
    HighGBarrelOwner() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const runtime::TacticalCommandBuildInput& input,
        const ControlIntent& root_intent,
        bool root_gun_selected,
        const HighGBarrelExactEvidence& evidence,
        HighGBarrelSelectionReceipt& output,
        Status& status) const noexcept;
    void BuildCandidate(
        HighGBarrelVariant selected_variant,
        const runtime::TacticalCommandBuildInput& input,
        const ControlIntent& root_intent,
        const HighGBarrelExactEvidence& evidence,
        const HighGBarrelSelectionReceipt& selection,
        ControlIntent& output,
        HighGBarrelOwnerSnapshot& commit,
        HighGBarrelTaskReceipt& receipt,
        Status& status) const noexcept;
    void BuildReleaseCommit(
        const HighGBarrelSelectionReceipt& selection,
        HighGBarrelOwnerSnapshot& commit,
        Status& status) const noexcept;
    void CommitPublished(
        const HighGBarrelOwnerSnapshot& commit,
        Status& status) noexcept;
    void CopySnapshot(HighGBarrelOwnerSnapshot& output) const noexcept;

private:
    HighGBarrelOwnerSnapshot snapshot_{};
};

static_assert(
    std::is_trivially_copyable<HighGBarrelOwnerSnapshot>::value,
    "G4 lifecycle must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<HighGBarrelSelectionReceipt>::value,
    "G4 selection receipt must remain allocation-free.");

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
