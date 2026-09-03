#pragma once

#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"
#include "LadyLuck/guidance/habfm/HabfmEngageDecision.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"
#include "LadyLuck/guidance/habfm/HabfmMergeEvidenceProvider.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

enum class HabfmTerminalControlIntentReason : std::uint8_t
{
    NotEvaluated = 0U,
    CommandGeometryUnavailable = 1U,
    ActiveCoreSelected = 2U,
    SameFrameReentrySelected = 3U,
    MergeApproachFallbackSelected = 4U
};

// Fixed, current-frame inputs. HABFM consumes the already-governed command
// envelope directly; it does not require a Gun observer to re-prove the same
// load authority before producing a maneuver command.
struct HabfmTerminalControlIntentInput
{
    DogfightGeometryFrame frame{};
    control::route5::CommandEnvelope current_envelope{};
};

// Command-neutral observation bundle. No maneuver-persistent owner is advanced
// while this receipt is constructed.
struct HabfmTerminalControlIntentObservation
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool applicable = false;
    HabfmTerminalControlIntentReason reason =
        HabfmTerminalControlIntentReason::NotEvaluated;
    HabfmCommandGeometryReceipt command_geometry{};
    HabfmFrameEvidenceStatus frame_evidence_status =
        HabfmFrameEvidenceStatus::FrameStateNotFinite;
    HabfmFrameEvidence frame_evidence{};
    HabfmPreTaskObservations pre_task{};
};

// Complete maneuver-persistent HABFM state. Candidate construction advances a
// copy of this object; only CommitPublished may replace the live copy.
struct HabfmTerminalControlIntentState
{
    HabfmMergeEvidenceProvider merge_evidence_provider{};
    HabfmActiveControlCore active_core{};
    HabfmEngageDecisionLatch engage_decision_latch{};
    std::uint64_t neutral_cue_streak = 0U;
};

struct HabfmTerminalPreparedControlIntent
{
    ControlFrameIdentity frame_identity{};
    std::uint64_t captured_generation = 0U;
    bool evaluated = false;
    bool selected = false;
    bool next_state_ready = false;
    bool merge_approach_fallback_used = false;
    bool committed = false;
    std::uint32_t same_frame_reentry_count = 0U;
    HabfmTerminalControlIntentReason reason =
        HabfmTerminalControlIntentReason::NotEvaluated;
    HabfmActiveCoreInputs active_inputs{};
    HabfmEngageDecisionReceipt engage_decision{};
    HabfmActiveControlOutput active_output{};
    ControlIntent intent{};
    HabfmTerminalControlIntentState next_state{};
};

// Allocation-free terminal owner for the existing HABFM writer-4 spine. It
// owns no mode classification, FCS, Auto-GCAS, or wire publication.
class HabfmTerminalControlIntentOwner final
{
public:
    HabfmTerminalControlIntentOwner() noexcept;

    void Reset() noexcept;
    void HaltLeg(bool selection_ready) noexcept;

    void Observe(
        const HabfmTerminalControlIntentInput& input,
        HabfmTerminalControlIntentObservation& output,
        Status& status) const noexcept;

    void PrepareCandidate(
        const HabfmTerminalControlIntentInput& input,
        const HabfmTerminalControlIntentObservation& observation,
        HabfmTerminalPreparedControlIntent& output,
        Status& status) const noexcept;

    void ValidatePublished(
        const HabfmTerminalPreparedControlIntent& prepared,
        Status& status) const noexcept;

    void CommitPublished(
        HabfmTerminalPreparedControlIntent& prepared,
        Status& status) noexcept;

    void CopyState(HabfmTerminalControlIntentState& output) const noexcept;
    void RestoreState(
        const HabfmTerminalControlIntentState& input) noexcept;

private:
    void BuildActiveInputs(
        const HabfmTerminalControlIntentInput& input,
        const HabfmTerminalControlIntentObservation& observation,
        HabfmMergeEvidenceProvider& provider,
        const HabfmActiveControlCore& active_core,
        HabfmActiveCoreInputs& output,
        Status& status) const noexcept;

    void UpdateEngageDecision(
        const HabfmTerminalControlIntentInput& input,
        const HabfmTerminalControlIntentObservation& observation,
        HabfmEngageDecisionLatch& latch,
        const HabfmActiveCoreInputs& active_inputs,
        HabfmEngageDecisionReceipt& output,
        Status& status) const noexcept;

    HabfmFrameEvidenceProvider frame_evidence_provider_{};
    HabfmTerminalControlIntentState state_{};
    std::uint64_t generation_ = 0U;
};

static_assert(
    std::is_standard_layout<HabfmTerminalControlIntentInput>::value,
    "HABFM terminal input must have standard layout");
static_assert(
    std::is_trivially_copyable<HabfmTerminalControlIntentInput>::value,
    "HABFM terminal input must remain allocation-free");
static_assert(
    std::is_standard_layout<HabfmTerminalControlIntentObservation>::value,
    "HABFM terminal observation must have standard layout");
static_assert(
    std::is_trivially_copyable<
        HabfmTerminalControlIntentObservation>::value,
    "HABFM terminal observation must remain allocation-free");
static_assert(
    std::is_standard_layout<HabfmTerminalControlIntentState>::value,
    "HABFM terminal state must have standard layout");
static_assert(
    std::is_trivially_copyable<HabfmTerminalControlIntentState>::value,
    "HABFM terminal state must remain allocation-free");
static_assert(
    std::is_standard_layout<HabfmTerminalPreparedControlIntent>::value,
    "HABFM prepared candidate must have standard layout");
static_assert(
    std::is_trivially_copyable<HabfmTerminalPreparedControlIntent>::value,
    "HABFM prepared candidate must remain allocation-free");

} // namespace LadyLuck
