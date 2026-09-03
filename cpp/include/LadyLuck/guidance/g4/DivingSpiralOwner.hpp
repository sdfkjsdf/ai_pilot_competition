#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/g13/G13FlatScissors.hpp"
#include "LadyLuck/guidance/g4/HighGBarrelEvidence.hpp"
#include "LadyLuck/guidance/obfm/G3ChaseDown.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

// Manual G4 Diving Spiral geometry.  These are the existing Python/manual
// boundaries, not tuned C++ gains: enter toward at most 40 degrees down and do
// not begin/continue the loaded roll until 30 degrees down is measured.
constexpr double DivingSpiralMinimumDiveRad =
    0.52359877559829887308;
constexpr double DivingSpiralEntryUpperDiveRad =
    0.69813170079773183077;

struct DivingSpiralActivation
{
    bool enabled = false;
    bool exact_provenance = false;
};

// User-authorized production request.  Runtime wiring remains responsible for
// preserving the visible PostRoot priority and exactly-one-writer contract.
constexpr DivingSpiralActivation DivingSpiralProductionActivation{
    true,
    true};

enum class DivingSpiralPhase : std::uint8_t
{
    None = 0U,
    DiveEntry = 1U,
    Spiral = 2U
};

enum class DivingSpiralDecision : std::uint8_t
{
    Unavailable = 0U,
    RootPassthrough = 1U,
    ReleasePassthrough = 2U,
    DiveEntry = 3U,
    Spiral = 4U
};

enum class DivingSpiralReason : std::uint8_t
{
    Unavailable = 0U,
    ProductionDisabled = 1U,
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
    PositiveClosureNotObserved = 14U,
    ManualEntryDiveExceedsFlightPathAuthority = 15U,
    RecoveryPullCapabilityUnavailable = 16U,
    InvertedDeepDiveRecoveryNotAdmitted = 17U,
    RunningSafetyRejected = 18U,
    CommittedControlFeedbackNotCausal = 19U,
    PreviousDivingSpiralCommandNotCompleted = 20U,
    G13ContinuationEvidenceMissing = 21U,
    G13ContinuationSourceUnresolved = 22U,
    AttackerPassageObserved = 23U,
    AttackerPassageUnresolved = 24U,
    AttackerCounterTurnObserved = 25U,
    FarSteadyLosVeto = 26U,
    LosVetoUnresolved = 27U,
    AttackerPursuitObservationUnresolved = 28U,
    AttackerDivingTurnNotSustained = 29U,
    CurrentPullCapabilityBelowCommittedLoad = 30U,
    ManualMinimumDeepDiveNotMaintained = 31U,
    DeepDiveEntryCommandUnavailable = 32U,
    SpiralLoadedRollCommandUnavailable = 33U,
    Admitted = 34U
};

// Exact age-1 lifecycle label supplied by the visible PostRoot runtime after a
// selected Diving Spiral Task has completed the control path.
struct DivingSpiralCompletedCommandEvidence
{
    bool valid = false;
    bool feedback_fresh = false;
    ControlFrameIdentity source_frame_identity{};
    double source_t_sec = 0.0;
    DivingSpiralPhase completed_phase = DivingSpiralPhase::None;
};

struct DivingSpiralOwnerSnapshot
{
    bool engaged = false;
    DivingSpiralPhase phase = DivingSpiralPhase::None;
    std::int32_t roll_direction_sign = 0;
    double requested_roll_rate_radps = 0.0;
    double requested_load_magnitude_g = 0.0;
    double entry_load_limit_g = 0.0;
    double entry_command_dive_angle_rad = 0.0;
    Vector3 commanded_velocity_ned_mps{};
    Vector3 commanded_bank_direction_ned{};
    double entry_elapsed_s = 0.0;
    DivingSpiralReason last_release_reason = DivingSpiralReason::Unavailable;
};

// Command-neutral Service receipt.  Only a selected leaf may materialize and
// publish the reference described here.
struct DivingSpiralSelectionReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    DivingSpiralDecision decision = DivingSpiralDecision::Unavailable;
    DivingSpiralReason reason = DivingSpiralReason::Unavailable;
    bool engaged_before = false;
    bool entry_admitted = false;
    bool entered_if_published = false;
    bool released_if_published = false;
    std::int32_t defender_turn_sign = 0;
    double requested_roll_rate_radps = 0.0;
    double requested_load_magnitude_g = 0.0;
    double entry_load_limit_g = 0.0;
    double requested_entry_dive_angle_rad = 0.0;
    double measured_dive_angle_rad = 0.0;
};

// Selected-Task receipt.  It explicitly separates the raw 3-D guidance
// request from FCS response.  The runtime copies these fields into one
// ControlIntent with the Diving Spiral writer identity.
struct DivingSpiralTaskReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool candidate_available = false;
    bool root_passthrough_required = false;
    bool entered_this_tick = false;
    DivingSpiralPhase phase = DivingSpiralPhase::None;
    DivingSpiralReason reason = DivingSpiralReason::Unavailable;
    Vector3 aim_point_ned_m{};
    Vector3 desired_bank_direction_ned{};
    Vector3 acceleration_ned_mps2{};
    bool roll_rate_reference_valid = false;
    double signed_roll_rate_reference_radps = 0.0;
    double load_magnitude_g = 0.0;
    double load_limit_g = 0.0;
    double measured_dive_angle_rad = 0.0;
    double entry_command_dive_angle_rad = 0.0;
    double entry_elapsed_s = 0.0;
};

class DivingSpiralOwner final
{
public:
    DivingSpiralOwner() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const runtime::TacticalCommandBuildInput& input,
        const ControlIntent& root_intent,
        bool root_gun_selected,
        const HighGBarrelExactEvidence& evidence,
        const guidance::g13::G13FlatScissorsObservation& g13_observation,
        const guidance::obfm::G3ChaseDownObservation* pursuit_observation,
        const DivingSpiralCompletedCommandEvidence& completed_command,
        const DivingSpiralActivation& activation,
        DivingSpiralSelectionReceipt& output,
        Status& status) const noexcept;
    void BuildCandidate(
        DivingSpiralPhase selected_phase,
        const runtime::TacticalCommandBuildInput& input,
        const ControlIntent& root_intent,
        const DivingSpiralSelectionReceipt& selection,
        ControlIntent& output,
        DivingSpiralOwnerSnapshot& commit,
        DivingSpiralTaskReceipt& receipt,
        Status& status) const noexcept;
    void BuildReleaseCommit(
        const DivingSpiralSelectionReceipt& selection,
        DivingSpiralOwnerSnapshot& commit,
        Status& status) const noexcept;
    void CommitPublished(
        const DivingSpiralOwnerSnapshot& commit,
        Status& status) noexcept;
    void CopySnapshot(DivingSpiralOwnerSnapshot& output) const noexcept;

private:
    DivingSpiralOwnerSnapshot snapshot_{};
};

static_assert(
    std::is_trivially_copyable<DivingSpiralOwnerSnapshot>::value,
    "Diving Spiral lifecycle must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<DivingSpiralSelectionReceipt>::value,
    "Diving Spiral selection must remain allocation-free.");

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
