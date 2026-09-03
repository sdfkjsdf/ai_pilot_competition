#pragma once

#include "LadyLuck/guidance/committed/G16CommittedOwner.hpp"
#include "LadyLuck/guidance/committed/G16HighPrevention.hpp"
#include "LadyLuck/guidance/doctrine/ModeDecision.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"
#include "LadyLuck/guidance/obfm/G5bDelayedClimb.hpp"
#include "LadyLuck/guidance/obfm/G3RollCounterOwner.hpp"
#include "LadyLuck/guidance/obfm/G3SScissorsResponse.hpp"
#include "LadyLuck/guidance/obfm/ObfmChaseUpGuard.hpp"
#include "LadyLuck/guidance/obfm/ObfmApexDisplacement.hpp"
#include "LadyLuck/guidance/obfm/ObfmEmployGuidance.hpp"
#include "LadyLuck/guidance/obfm/ObfmEntryLongitudinalReference.hpp"
#include "LadyLuck/guidance/obfm/ObfmEntryWindowAdmission.hpp"
#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"
#include "LadyLuck/guidance/obfm/ObfmLeadDiscipline.hpp"
#include "LadyLuck/guidance/obfm/ObfmLongitudinalReferenceProvider.hpp"
#include "LadyLuck/guidance/obfm/ObfmNearStationCarrot.hpp"
#include "LadyLuck/guidance/obfm/ObfmSpacingOwner.hpp"
#include "LadyLuck/guidance/obfm/ObfmStationHoldService.hpp"
#include "LadyLuck/guidance/obfm/TerminalAiming.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

// Fixed, already-observed inputs for the OBFM G16/G5b spine.  The safety and
// speed-floor receipts remain command-neutral inputs owned by their upstream
// Services; this owner does not recreate Root gun, prefire, or E-M authority.
struct StaticDoctrineObfmG16G5bInput
{
    bool valid = false;
    bool current_effect_employ_override = false;
    runtime::TacticalCommandBuildInput tactical_input{};
    guidance::doctrine::ModeDecision mode_decision{};
    guidance::obfm::G5bSafetyEvidence g5b_safety{};
    guidance::obfm::G5bSpeedFloorEvidence g5b_speed_floor{};
    guidance::obfm::ObfmLeadDisciplineInput previous_pursuit{};
};

// Only state that can be advanced by writer 6, 8, 5, or 7 candidate work is
// captured here.  G16 physical observation and production-evidence history are
// deliberately outside this transaction and therefore survive candidate abort.
struct StaticDoctrineObfmG16G5bOwnerState
{
    guidance::committed::G16CommittedOwner g16_committed{};
    guidance::committed::G16HighPreventionTransactionState g16_high{};
    guidance::obfm::G5bDelayedClimb g5b{};
    guidance::obfm::ObfmEntryWindowAdmission entry_window{};
    guidance::obfm::ObfmEntryLongitudinalReference entry_longitudinal{};
    guidance::obfm::ObfmSpacingOwner spacing{};
    bool spacing_task_active = false;
    guidance::obfm::G3RollCounterOwner g3_roll_counter{};
    guidance::obfm::G3ScissorsOwner g3_scissors{};
    guidance::obfm::ObfmApexDisplacement apex{};
    bool apex_task_active = false;
    ObfmLagGuidance lag{};
};

// Fixed diagnostic receipt for one raw-guidance transaction.  It contains no
// p/q/r, Nz, surface, thrust, estimator-truth, or aircraft-response authority.
struct StaticDoctrineObfmG16G5bSnapshot
{
    bool prepare_attempted = false;
    bool frame_ready = false;
    ControlFrameIdentity frame_identity{};
    guidance::doctrine::OptionalTacticalMode effective_mode{};
    StatusCode status_code = StatusCode::Ok;

    bool physical_observation_attempted = false;
    bool employ_observation_attempted = false;
    bool employ_observation_ready = false;
    guidance::obfm::ObfmEmployAdmissionReceipt employ_admission{};
    bool entry_established_turn_attempted = false;
    bool entry_established_turn_ready = false;
    guidance::obfm::ObfmEntryEstablishedTurnReceipt
        entry_established_turn{};
    StatusCode entry_established_turn_status = StatusCode::Ok;
    bool entry_observation_attempted = false;
    bool entry_observation_ready = false;
    guidance::obfm::ObfmEntryWindowObservationReceipt entry_observation{};
    StatusCode entry_observation_status = StatusCode::Ok;
    bool entry_service_attempted = false;
    guidance::obfm::ObfmEntrySetupServiceReceipt entry_service{};
    guidance::obfm::ObfmEntrySetupTaskReceipt entry_task{};
    guidance::obfm::ObfmEntryLongitudinalPreparation
        entry_longitudinal_preparation{};
    guidance::obfm::ObfmEntryLongitudinalReceipt entry_longitudinal{};
    guidance::obfm::ObfmNearStationCarrotReceipt entry_carrot{};
    guidance::obfm::ObfmEntrySetupHaltReceipt entry_halt{};
    bool entry_owner_active = false;
    bool precision_speed_ready = false;
    guidance::committed::G16PrecisionSpeedReceipt precision_speed{};
    ObfmStationHoldServiceReceipt precision_station{};
    ObfmLagGuidancePreparation precision_lag_preparation{};
    ObfmLongitudinalProviderReceipt precision_longitudinal{};
    ObfmBumplessSpeedReceipt precision_bumpless{};
    bool g16_evidence_attempted = false;
    bool g16_evidence_ready = false;
    guidance::committed::G16ProductionEvidenceReceipt g16_evidence{};
    StatusCode g16_evidence_status = StatusCode::Ok;
    bool g16_high_observation_attempted = false;
    bool g16_high_observation_ready = false;
    guidance::committed::G16HighObservationReceipt
        g16_high_observation{};
    StatusCode g16_high_observation_status = StatusCode::Ok;

    guidance::committed::G16CommittedOwnerReceipt g16_owner{};
    guidance::committed::G16CommittedSelection g16_selection{};
    bool g16_high_transition_attempted = false;
    bool g16_high_transition_ready = false;
    guidance::committed::G16HighPreventionReceipt g16_high_transition{};
    guidance::committed::G16HighSelection g16_high_selection{};
    guidance::committed::G16HighToLagHandoff g16_high_to_lag_commit{};

    bool lag_station_observation_attempted = false;
    bool lag_station_observation_ready = false;
    ObfmStationHoldServiceReceipt lag_station{};
    ObfmLagSpeedAuthority lag_speed_authority =
        ObfmLagSpeedAuthority::Unavailable;
    bool lag_speed_authority_ready = false;
    bool lag_base_ready = false;
    ObfmLagGuidanceCommit lag_commit{};
    bool lag_commit_ready = false;
    ControlIntent lag_base_intent{};
    guidance::obfm::ObfmLeadDisciplineReceipt lead_discipline{};
    bool lead_discipline_ready = false;
    guidance::obfm::TerminalTrackingReceipt terminal_tracking{};
    bool terminal_tracking_ready = false;
    bool effective_terminal_tracking = false;
    HabfmFrameEvidence chase_up_frame_evidence{};
    HabfmFrameEvidenceStatus chase_up_frame_evidence_status =
        HabfmFrameEvidenceStatus::FrameStateNotFinite;
    guidance::obfm::ObfmChaseUpGuardReceipt chase_up{};

    bool g5b_observation_attempted = false;
    bool g5b_observation_ready = false;
    bool g5b_terminal_fallthrough = false;
    bool g5b_mode_reevaluation_requested = false;
    guidance::obfm::G5bDelayedClimbObservation g5b_observation{};
    guidance::obfm::G5bDelayedClimbSelection g5b_selection{};
    guidance::obfm::G5bDelayedClimbTaskReceipt g5b_task{};
    guidance::obfm::G5bDelayedClimbHaltReceipt g5b_halt{};

    bool spacing_service_attempted = false;
    bool spacing_service_ready = false;
    bool spacing_owner_active = false;
    guidance::obfm::ObfmSpacingOwnerServiceReceipt spacing_service{};
    guidance::obfm::ObfmSpacingOwnerSelection spacing_selection{};
    guidance::obfm::ObfmSpacingOwnerTaskReceipt spacing_task{};
    guidance::obfm::ObfmSpacingOwnerHaltReceipt spacing_halt{};
    bool spacing_candidate_ready = false;

    guidance::obfm::G3RollCounterReceipt g3_roll_counter{};
    guidance::obfm::G3CounterRollingScissorsReceipt
        g3_counter_rolling_scissors{};
    guidance::obfm::G3ScissorsReceipt g3_scissors{};

    bool apex_observation_attempted = false;
    bool apex_observation_ready = false;
    bool apex_owner_active = false;
    guidance::obfm::ObfmApexDisplacementServiceReceipt apex_service{};
    guidance::obfm::ObfmApexDisplacementSelection apex_selection{};
    guidance::obfm::ObfmApexDisplacementTaskReceipt apex_task{};
    guidance::obfm::ObfmApexDisplacementHaltReceipt apex_halt{};

    bool candidate_stage_active = false;
    bool candidate_state_ready = false;
    bool deferred_commit_requested = false;
    bool integrated_intent_staged = false;
    bool prepared_transaction_ready = false;
    bool prepared_transaction_committed = false;
    bool prepared_transaction_aborted = false;
    std::uint32_t selected_writer_id = ControlIntentWriterNone;
    std::uint8_t selection_count = 0U;
    std::uint8_t candidate_count = 0U;
    ControlIntent staged_base_intent{};
    ControlIntent prepared_intent{};
};

// Fixed-storage facade for StaticDoctrineObfmG16G5bAdapter.  The adapter
// performs visible priority/lifecycle traversal; this class owns only the
// existing writer-6/8/5/7 calculations and their fixed staged state.
class StaticDoctrineObfmG16G5bOwner final
{
public:
    StaticDoctrineObfmG16G5bOwner() noexcept;

    void Reset() noexcept;
    void Prepare(
        const StaticDoctrineObfmG16G5bInput& input,
        StaticDoctrineObfmG16G5bSnapshot& output,
        Status& status) noexcept;
    void ValidatePrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) const noexcept;
    void CommitPrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) noexcept;
    void AbortPrepared() noexcept;
    void CopySnapshot(
        StaticDoctrineObfmG16G5bSnapshot& output) const noexcept;
    void CopyPreparedIntent(
        ControlIntent& output,
        Status& status) const noexcept;

    // Surface consumed by StaticDoctrineObfmG16G5bAdapter.
    void BeginFinalCommandCandidateStage(Status& status) noexcept;
    void CopyStagedBaseIntent(
        ControlIntent& output,
        Status& status) const noexcept;
    void RollbackFinalCommandCandidateState(Status& status) noexcept;
    void BeginDeferredFinalStateCommit(Status& status) noexcept;
    void PublishIntegratedFinalIntent(
        const ControlIntent& intent,
        Status& status) noexcept;
    void CommitPreparedFinalState(Status& status) noexcept;
    void AbortPreparedFinalState() noexcept;
    void CheckObfmWriterLocalRejection(
        bool& rejected,
        Status& status) const noexcept;

    void ObserveObfmG16HighPhysical(Status& status) noexcept;
    void ObserveObfmEmploy(Status& status) noexcept;
    void SelectObfmEmploy(bool& selected, Status& status) noexcept;
    void PublishObfmEmploy(Status& status) noexcept;
    void ObserveObfmEntryPhysical(Status& status) noexcept;
    void SelectObfmEntry(
        bool& selected,
        bool& completed,
        Status& status) noexcept;
    void PublishObfmEntry(Status& status) noexcept;
    void CompleteObfmEntry() noexcept;
    void HaltObfmEntry() noexcept;
    void EvaluateObfmG16HighCandidate(Status& status) noexcept;
    void SelectG16Committed(bool& selected, Status& status) noexcept;
    void PublishG16Committed(Status& status) noexcept;
    void HaltG16Committed() noexcept;
    void SelectObfmG16High(bool& selected, Status& status) noexcept;
    void PublishObfmG16High(Status& status) noexcept;

    void SelectObfmG16HighToLag(
        bool& selected,
        Status& status) noexcept;
    void ObserveObfmLagStation(Status& status) noexcept;
    void SelectObfmLagSpeedAuthority(
        ObfmLagSpeedAuthority authority,
        bool& selected,
        Status& status) noexcept;
    void PrepareObfmLagBase(Status& status) noexcept;
    void ObserveObfmLeadDiscipline(Status& status) noexcept;
    void ObserveObfmTerminalTracking(Status& status) noexcept;
    void SelectObfmTerminalTracking(
        bool& selected,
        Status& status) noexcept;
    void PublishObfmG16HighToLag(
        bool terminal_tracking_selected,
        Status& status) noexcept;
    // Ordinary lower OBFM leaf. It uses the same writer-5 speed authority,
    // LAG, lead-discipline, and terminal-tracking path without consuming the
    // G16 High-to-Lag lifecycle handoff.
    void PublishObfmLagFallback(Status& status) noexcept;

    void SelectObfmG5b(bool& selected, Status& status) const noexcept;
    void ObserveObfmG5b(Status& status) noexcept;
    void CheckObfmG5bNormalTerminalFallthrough(
        bool& fallthrough,
        Status& status) const noexcept;
    void SelectObfmG5bBranch(
        guidance::obfm::G5bSelectedBranch branch,
        bool& selected,
        Status& status) const noexcept;
    void PublishObfmG5b(
        guidance::obfm::G5bSelectedBranch branch,
        Status& status) noexcept;
    void CompleteObfmG5b(Status& status) noexcept;
    void ReleaseObfmG5b(Status& status) noexcept;
    void FailObfmG5bInvalid(Status& status) noexcept;
    void HaltObfmG5b(
        guidance::obfm::G5bSelectedBranch halted_branch) noexcept;
    void EvaluateObfmSpacing(
        bool& selected,
        bool& completed,
        bool& released,
        Status& status) noexcept;
    void ObserveObfmSpacingEmployPreemption(Status& status) noexcept;
    void HaltObfmSpacing() noexcept;
    void EvaluateObfmG3RollCounter(
        bool& selected,
        bool& released,
        Status& status) noexcept;
    void HaltObfmG3RollCounter() noexcept;
    void EvaluateObfmG3CounterRollingScissors(
        bool& selected,
        bool& released,
        Status& status) noexcept;
    void HaltObfmG3CounterRollingScissors() noexcept;
    void EvaluateObfmG3Scissors(
        bool& selected,
        bool& released,
        Status& status) noexcept;
    void HaltObfmG3Scissors() noexcept;
    void ObserveObfmApexPhysical(Status& status) noexcept;
    void EvaluateObfmApex(
        bool& selected,
        bool& released,
        Status& status) noexcept;
    void HaltObfmApex() noexcept;
    void ResolveTacticalModeChild(
        std::size_t& child_index,
        Status& status) const noexcept;

private:
    void ClearFrameState() noexcept;
    void ClearLagCandidateFrameState() noexcept;
    void CaptureOwnerState(
        StaticDoctrineObfmG16G5bOwnerState& output) const noexcept;
    void RestoreOwnerState(
        const StaticDoctrineObfmG16G5bOwnerState& input) noexcept;
    void SealCandidateState(Status& status) noexcept;
    void SelectWriter(std::uint32_t writer_id, Status& status) noexcept;
    void RejectSelectedCandidate(StatusCode code, Status& status) noexcept;
    void StageBaseIntent(
        std::uint32_t writer_id,
        const ControlIntent& intent,
        Status& status) noexcept;
    void BuildAndStageWriter(
        std::uint32_t writer_id,
        Status& status) noexcept;
    void BuildPrecisionSpeedReference(Status& status) noexcept;
    void SetEmptyPrecisionStation() noexcept;
    void SetPhaseCurrentSpeedEcho() noexcept;
    void BuildPublishedLagIntent(
        bool terminal_tracking_selected,
        const ControlIntent& upstream,
        ControlIntent& output,
        Status& status) noexcept;
    void PublishObfmLag(
        bool terminal_tracking_selected,
        Status& status) noexcept;
    void BuildObfmSpacingServiceInput(
        guidance::obfm::ObfmSpacingOwnerServiceInput& output) const noexcept;
    bool ObfmModeSelected() const noexcept;
    bool CurrentEffectEmploySelected() const noexcept;

    StaticDoctrineObfmG16G5bInput input_{};
    guidance::committed::G16ProductionEvidenceProvider
        g16_evidence_provider_{};
    guidance::obfm::ObfmEmployAdmissionProvider employ_admission_provider_{};
    guidance::obfm::ObfmEntryEstablishedTurnObserver
        entry_established_turn_observer_{};
    guidance::obfm::ObfmEntryWindowAdmission entry_window_{};
    guidance::obfm::ObfmEntryLongitudinalReference entry_longitudinal_{};
    guidance::committed::G16CommittedOwner g16_committed_owner_{};
    guidance::committed::G16HighPrevention g16_high_prevention_{};
    guidance::obfm::G5bDelayedClimb g5b_delayed_climb_{};
    guidance::obfm::ObfmSpacingOwner obfm_spacing_owner_{};
    bool obfm_spacing_task_active_ = false;
    guidance::obfm::G3RollCounterOwner g3_roll_counter_owner_{};
    guidance::obfm::G3ScissorsOwner g3_scissors_owner_{};
    guidance::obfm::ObfmApexDisplacement obfm_apex_displacement_{};
    bool obfm_apex_task_active_ = false;
    ObfmLagGuidance obfm_lag_guidance_{};
    ObfmStationHoldService obfm_station_hold_service_{};
    ObfmLongitudinalReferenceProvider obfm_longitudinal_provider_{};
    HabfmFrameEvidenceProvider frame_evidence_provider_{};

    StaticDoctrineObfmG16G5bOwnerState original_state_{};
    StaticDoctrineObfmG16G5bOwnerState candidate_state_{};
    bool original_state_ready_ = false;
    bool candidate_state_ready_ = false;
    bool candidate_stage_active_ = false;
    bool deferred_commit_requested_ = false;
    bool integrated_intent_staged_ = false;
    bool prepared_transaction_ready_ = false;
    ControlIntent staged_base_intent_{};
    bool staged_base_intent_ready_ = false;
    ControlIntent prepared_intent_{};
    std::uint32_t selected_writer_id_ = ControlIntentWriterNone;
    std::uint8_t selection_count_ = 0U;
    StaticDoctrineObfmG16G5bSnapshot snapshot_{};
};

static_assert(
    std::is_standard_layout<StaticDoctrineObfmG16G5bInput>::value,
    "Static OBFM G16/G5b input must have standard layout.");
static_assert(
    std::is_trivially_copyable<StaticDoctrineObfmG16G5bInput>::value,
    "Static OBFM G16/G5b input must remain fixed-storage.");
static_assert(
    std::is_standard_layout<StaticDoctrineObfmG16G5bOwnerState>::value,
    "Static OBFM G16/G5b owner state must have standard layout.");
static_assert(
    std::is_trivially_copyable<StaticDoctrineObfmG16G5bOwnerState>::value,
    "Static OBFM G16/G5b owner state must remain fixed-copy state.");
static_assert(
    std::is_standard_layout<StaticDoctrineObfmG16G5bSnapshot>::value,
    "Static OBFM G16/G5b snapshot must have standard layout.");
static_assert(
    std::is_trivially_copyable<StaticDoctrineObfmG16G5bSnapshot>::value,
    "Static OBFM G16/G5b snapshot must remain fixed-storage.");
static_assert(
    std::is_nothrow_copy_assignable<
        StaticDoctrineObfmG16G5bOwnerState>::value,
    "Static OBFM G16/G5b staged state copy must be nonthrowing.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
