#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtLifecycle.hpp"
#include "LadyLuck/behavior_tree/static/StaticDoctrineDispatch.hpp"
#include "LadyLuck/guidance/obfm/G5bDelayedClimb.hpp"
#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"
#include "generated/LadyLuckV2TopologyCapacity.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace doctrine
{
class DoctrineBtRuntimeContext;
}
namespace static_bt
{

// This adapter owns the fixed official-EMPLOY/G16/G5b/SPACING/ENTRY shoulder
// of OBFM. The caller supplies those visible leaves and the lower LAG leaf.
enum class StaticDoctrineObfmG16G5bReason : BtReasonId
{
    None = 0U,
    TransactionAlreadyPending = 1U,
    FallbackDescriptorInvalid = 2U,
    CandidateStageUnavailable = 3U,
    StagedCandidateUnavailable = 4U,
    G5bInvalidPhase = 5U,
    TerminalWithoutCurrentFrameCandidate = 6U,
    DeferredCommitUnavailable = 7U,
    CommitNotPending = 8U,
    CommitPublicationFailed = 9U,
    PreparedStateCommitFailed = 10U,
    G5bActivePhaseUnknown = 11U
};

constexpr BtReasonId ToReasonId(
    const StaticDoctrineObfmG16G5bReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr std::uint32_t ObfmG16G5bObservationElementIndex = 1U;
constexpr std::uint32_t ObfmEmployElementIndex = 14U;
constexpr std::uint32_t ObfmG16CommittedElementIndex = 18U;
constexpr std::uint32_t ObfmG16HighElementIndex = 19U;
constexpr std::uint32_t ObfmHighToLagElementIndex = 20U;
constexpr std::uint32_t ObfmG5bElementIndex = 21U;
constexpr std::uint32_t ObfmG3CounterBarrelElementIndex = 22U;
constexpr std::uint32_t ObfmG3CounterRollingScissorsElementIndex = 23U;
constexpr std::uint32_t ObfmG3ScissorsElementIndex = 24U;
constexpr std::uint32_t ObfmApexElementIndex = 25U;
constexpr std::uint32_t ObfmEntryElementIndex = 26U;
constexpr std::uint32_t ObfmSpacingElementIndex = 27U;

constexpr BtNodeId ObfmG16G5bObservationNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG16G5bObservationElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmEmployNodeId =
    generated::LadyLuckV2XmlNodes[ObfmEmployElementIndex].static_node_id;
constexpr BtNodeId ObfmG16CommittedNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG16CommittedElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmG16HighNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG16HighElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmHighToLagTerminalNodeId =
    generated::LadyLuckV2XmlNodes[ObfmHighToLagElementIndex].static_node_id;
constexpr BtNodeId ObfmHighToLagNodeId =
    generated::LadyLuckV2XmlNodes[ObfmHighToLagElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmG5bCompleteNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG5bElementIndex].static_node_id;
constexpr BtNodeId ObfmG5bReleaseNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG5bElementIndex].static_node_id;
constexpr BtNodeId ObfmG3CounterBarrelNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG3CounterBarrelElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmG3CounterRollingScissorsNodeId =
    generated::LadyLuckV2XmlNodes[
        ObfmG3CounterRollingScissorsElementIndex].static_node_id;
constexpr BtNodeId ObfmG3ScissorsNodeId =
    generated::LadyLuckV2XmlNodes[ObfmG3ScissorsElementIndex]
        .static_node_id;
constexpr BtNodeId ObfmApexNodeId =
    generated::LadyLuckV2XmlNodes[ObfmApexElementIndex].static_node_id;
constexpr BtNodeId ObfmSpacingNodeId =
    generated::LadyLuckV2XmlNodes[ObfmSpacingElementIndex].static_node_id;
constexpr BtNodeId ObfmEntryNodeId =
    generated::LadyLuckV2XmlNodes[ObfmEntryElementIndex].static_node_id;
// EXTEND and ZOOM are internal states of the single XML Task_G5b leaf.  Their
// diagnostic IDs are deliberately outside the XML node range so lifecycle
// halt/transition records remain phase-specific without inventing more XML
// command publishers.
constexpr BtNodeId ObfmG5bZoomEntryNodeId =
    static_cast<BtNodeId>(generated::LadyLuckV2OracleNodeCapacity + 1U);
constexpr BtNodeId ObfmG5bExtendNodeId =
    static_cast<BtNodeId>(generated::LadyLuckV2OracleNodeCapacity + 2U);
constexpr BtNodeId ObfmG5bZoomClimbNodeId =
    static_cast<BtNodeId>(generated::LadyLuckV2OracleNodeCapacity + 3U);
constexpr BtNodeId ObfmG5bInvalidNodeId =
    static_cast<BtNodeId>(generated::LadyLuckV2OracleNodeCapacity + 4U);

constexpr BtStageId ObfmG16G5bStageId = 1U;
constexpr BtLifecycleOwnerId ObfmG16CommittedOwnerId =
    MakeBtLifecycleOwnerId(6U);
constexpr BtLifecycleOwnerId ObfmG16HighOwnerId =
    MakeBtLifecycleOwnerId(8U);
constexpr BtLifecycleOwnerId ObfmG5bOwnerId =
    MakeBtLifecycleOwnerId(7U);
constexpr BtLifecycleOwnerId ObfmG3CounterBarrelOwnerId =
    MakeBtLifecycleOwnerId(17U);
constexpr BtLifecycleOwnerId ObfmG3CounterRollingScissorsOwnerId =
    MakeBtLifecycleOwnerId(18U);
constexpr BtLifecycleOwnerId ObfmG3ScissorsOwnerId =
    MakeBtLifecycleOwnerId(16U);
constexpr BtLifecycleOwnerId ObfmApexOwnerId =
    MakeBtLifecycleOwnerId(21U);
constexpr BtLifecycleOwnerId ObfmSpacingOwnerId =
    MakeBtLifecycleOwnerId(28U);
constexpr BtLifecycleOwnerId ObfmEntryOwnerId =
    MakeBtLifecycleOwnerId(29U);

static_assert(ObfmG16G5bObservationNodeId == 2U,
              "Frozen G16 observation source identity changed");
static_assert(ObfmEmployNodeId == 15U,
              "Frozen current-effect EMPLOY source identity changed");
static_assert(ObfmG16CommittedNodeId == 19U,
              "Frozen G16 committed source identity changed");
static_assert(ObfmG16HighNodeId == 20U,
              "Frozen G16 High source identity changed");
static_assert(ObfmHighToLagTerminalNodeId == 21U,
              "Frozen terminal HighToLag source identity changed");
static_assert(ObfmHighToLagNodeId == 21U,
              "Frozen HighToLag source identity changed");
static_assert(ObfmG5bCompleteNodeId == 22U,
              "Frozen G5b COMPLETE source identity changed");
static_assert(ObfmG5bReleaseNodeId == 22U,
              "Frozen G5b RELEASE source identity changed");
static_assert(ObfmG3CounterBarrelNodeId == 23U,
              "Frozen G3 counter-barrel source identity changed");
static_assert(ObfmG3CounterRollingScissorsNodeId == 24U,
              "Frozen G3 counter-rolling-scissors identity changed");
static_assert(ObfmG3ScissorsNodeId == 25U,
              "Frozen G3 scissors gap/egress identity changed");
static_assert(ObfmApexNodeId == 26U,
              "Frozen OBFM Apex displacement identity changed");
static_assert(ObfmSpacingNodeId == 28U,
              "Frozen SPACING source identity changed");
static_assert(ObfmEntryNodeId == 27U,
              "Frozen current-turn ENTRY source identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmEmployElementIndex]
            .publication_writer_id == 10U,
    "OBFM EMPLOY writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmG16CommittedElementIndex]
            .publication_writer_id == 6U,
    "G16 committed writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmG16HighElementIndex]
            .publication_writer_id == 8U,
    "G16 High writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmHighToLagElementIndex]
            .publication_writer_id == 5U,
    "HighToLag writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmG5bElementIndex]
                .publication_writer_id == 7U &&
        generated::LadyLuckV2XmlNodes[ObfmG5bElementIndex]
                .lifecycle_writer_id == 7U,
    "G5b lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmG3CounterBarrelElementIndex]
                .publication_writer_id == 17U
        && generated::LadyLuckV2XmlNodes[ObfmG3CounterBarrelElementIndex]
                .lifecycle_writer_id == 17U,
    "G3 counter-barrel lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[
        ObfmG3CounterRollingScissorsElementIndex]
                .publication_writer_id == 18U
        && generated::LadyLuckV2XmlNodes[
            ObfmG3CounterRollingScissorsElementIndex]
                .lifecycle_writer_id == 18U,
    "G3 counter-rolling-scissors lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmG3ScissorsElementIndex]
                .publication_writer_id == 16U
        && generated::LadyLuckV2XmlNodes[ObfmG3ScissorsElementIndex]
                .lifecycle_writer_id == 16U,
    "G3 scissors lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmApexElementIndex]
                .publication_writer_id == 21U
        && generated::LadyLuckV2XmlNodes[ObfmApexElementIndex]
                .lifecycle_writer_id == 21U,
    "OBFM Apex lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmSpacingElementIndex]
                .publication_writer_id == 28U
        && generated::LadyLuckV2XmlNodes[ObfmSpacingElementIndex]
                .lifecycle_writer_id == 28U,
    "SPACING lifecycle/writer identity changed");
static_assert(
    generated::LadyLuckV2XmlNodes[ObfmEntryElementIndex]
                .publication_writer_id == 29U &&
        generated::LadyLuckV2XmlNodes[ObfmEntryElementIndex]
                .lifecycle_writer_id == 29U,
    "Current-turn ENTRY lifecycle/writer identity changed");

template <typename DoctrineContextT>
struct StaticDoctrineObfmLowerLeaf
{
    StaticBtChildDescriptor descriptor{};
    StaticBtCandidateCallback<DoctrineContextT> callback = nullptr;
};

enum class StaticDoctrineObfmModeRedispatch : std::uint8_t
{
    None = 0U,
    Habfm = 1U,
    Dbfm = 2U
};

template <typename DoctrineContextT>
struct StaticDoctrineObfmModeRedispatchLeaves
{
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> habfm{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> dbfm{};
};

struct StaticDoctrineObfmG16G5bResult
{
    StaticBtLifecycleSelection lifecycle{};
    StaticDoctrineObfmModeRedispatch mode_redispatch =
        StaticDoctrineObfmModeRedispatch::None;
    bool pending_commit = false;
    // A failure in an optional G16/G5b child may not erase a constructible
    // current-frame LAG terminal.  Preserve the original typed fault while
    // publishing that doctrine base; failure of the lower leaf itself remains
    // an honest command-neutral result.
    bool current_base_recovered = false;
    bool optional_fault_available = false;
    BtTickResult optional_fault{};
    std::uint8_t reserved[5]{};
};

template <typename DoctrineContextT>
struct StaticDoctrineObfmG16G5bEvaluationContext
{
    DoctrineContextT* doctrine = nullptr;
    ControlFrameIdentity frame{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> employ_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> g3_roll_counter_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT>
        g3_counter_rolling_scissors_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> g3_scissors_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> apex_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> spacing_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> entry_leaf{};
    StaticDoctrineObfmLowerLeaf<DoctrineContextT> lower_leaf{};
    StaticDoctrineObfmModeRedispatchLeaves<DoctrineContextT>
        redispatch_leaves{};
    StaticDoctrineObfmModeRedispatch mode_redispatch =
        StaticDoctrineObfmModeRedispatch::None;
    bool g16_candidate_state_ready = false;
    bool g16_high_owner_active_at_entry = false;
    bool entry_owner_active_at_entry = false;
    bool entry_handoff_to_lag_this_tick = false;

    bool high_to_lag_evaluated = false;
    bool high_to_lag_selected = false;
    bool high_to_lag_terminal_tracking = false;
    bool high_to_lag_candidate_ready = false;
    bool high_to_lag_fault_available = false;
    BtTickResult high_to_lag_fault{};
    ControlIntent high_to_lag_candidate{};

    bool g5b_evaluated = false;
    bool g5b_selected = false;
    bool g5b_normal_terminal_fallthrough = false;
    bool g5b_branch_resolved = false;
    guidance::obfm::G5bSelectedBranch g5b_branch =
        guidance::obfm::G5bSelectedBranch::Invalid;
    BtTickResult g5b_fault{};
    bool g5b_fault_available = false;
    bool g5b_action_applied = false;
    bool g5b_owner_active_at_entry = false;
    bool g5b_candidate_ready = false;
    ControlIntent g5b_candidate{};
};

template <typename DoctrineContextT>
constexpr BtTickResult MakeObfmG16G5bStatusResult(
    const StatusCode code,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    return MakeBtTickResult(
        MapDoctrineStatusReturnCode(code),
        node_id,
        stage_id,
        frame_index,
        ToReasonId(MapDoctrineStatusReason(code)));
}

constexpr BtTickResult MakeObfmG16G5bResult(
    const BtReturnCode code,
    const BtNodeId node_id,
    const std::uint64_t frame_index,
    const StaticDoctrineObfmG16G5bReason reason) noexcept
{
    return MakeBtTickResult(code,
                            node_id,
                            ObfmG16G5bStageId,
                            frame_index,
                            ToReasonId(reason));
}

constexpr BtTickResult MakeObfmG16G5bNotApplicable(
    const BtNodeId node_id,
    const std::uint64_t frame_index) noexcept
{
    return MakeObfmG16G5bResult(
        BtReturnCode::NotApplicable,
        node_id,
        frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
BtTickResult ResolveObfmWriterFailure(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const StatusCode writer_failure,
    const BtNodeId node_id) noexcept
{
    bool rejected = false;
    Status status{};
    context.doctrine->CheckObfmWriterLocalRejection(rejected, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    if (!rejected)
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            writer_failure,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }

    // Writer-local construction rejection is ordinary sibling fallthrough,
    // but only after selection/evaluation owner state is restored to the
    // transaction snapshot. Same-frame physical observation receipts remain.
    context.doctrine->RollbackFinalCommandCandidateState(status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    context.g16_candidate_state_ready = false;
    return MakeObfmG16G5bNotApplicable(
        node_id, context.frame.frame_index);
}

template <typename DoctrineContextT>
BtTickResult EnsureObfmG16CandidateState(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtNodeId node_id,
    bool& prepared_now) noexcept
{
    prepared_now = false;
    if (context.g16_candidate_state_ready)
    {
        return MakeObfmG16G5bResult(
            BtReturnCode::Completed,
            node_id,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::None);
    }
    Status status{};
    context.doctrine->EvaluateObfmG16HighCandidate(status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    context.g16_candidate_state_ready = true;
    prepared_now = true;
    return MakeObfmG16G5bResult(
        BtReturnCode::Completed,
        node_id,
        context.frame.frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
BtTickResult RollbackUnusedObfmCandidateState(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtNodeId node_id,
    const bool prepared_now) noexcept
{
    if (!prepared_now)
    {
        return MakeObfmG16G5bNotApplicable(
            node_id, context.frame.frame_index);
    }
    Status status{};
    context.doctrine->RollbackFinalCommandCandidateState(status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    context.g16_candidate_state_ready = false;
    return MakeObfmG16G5bNotApplicable(
        node_id, context.frame.frame_index);
}

template <typename DoctrineContextT>
BtTickResult CopyObfmG16G5bStagedCandidate(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtNodeId node_id,
    ControlIntent& output) noexcept
{
    Status status{};
    context.doctrine->CopyStagedBaseIntent(output, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            node_id,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    return MakeObfmG16G5bResult(
        BtReturnCode::Selected,
        node_id,
        context.frame.frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG16Committed(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    bool prepared_now = false;
    const BtTickResult prepared = EnsureObfmG16CandidateState(
        context, ObfmG16CommittedNodeId, prepared_now);
    if (IsBtReturnCodeError(prepared.code))
    {
        return prepared;
    }
    bool selected = false;
    Status status{};
    context.doctrine->SelectG16Committed(selected, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG16CommittedNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    if (!selected)
    {
        return RollbackUnusedObfmCandidateState(
            context, ObfmG16CommittedNodeId, prepared_now);
    }
    context.doctrine->PublishG16Committed(status);
    if (!status.ok())
    {
        return ResolveObfmWriterFailure(
            context, status.code, ObfmG16CommittedNodeId);
    }
    const BtTickResult copied = CopyObfmG16G5bStagedCandidate(
        context, ObfmG16CommittedNodeId, proposed);
    return IsBtReturnCodeError(copied.code)
        ? copied
        : MakeObfmG16G5bResult(
              BtReturnCode::Running,
              ObfmG16CommittedNodeId,
              context.frame.frame_index,
              StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG16High(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    bool prepared_now = false;
    const BtTickResult prepared = EnsureObfmG16CandidateState(
        context, ObfmG16HighNodeId, prepared_now);
    if (IsBtReturnCodeError(prepared.code))
    {
        return prepared;
    }
    bool selected = false;
    Status status{};
    context.doctrine->SelectObfmG16High(selected, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG16HighNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    if (!selected)
    {
        return RollbackUnusedObfmCandidateState(
            context, ObfmG16HighNodeId, prepared_now);
    }
    context.doctrine->PublishObfmG16High(status);
    if (!status.ok())
    {
        return ResolveObfmWriterFailure(
            context, status.code, ObfmG16HighNodeId);
    }
    const BtTickResult copied = CopyObfmG16G5bStagedCandidate(
        context, ObfmG16HighNodeId, proposed);
    return IsBtReturnCodeError(copied.code)
        ? copied
        : MakeObfmG16G5bResult(
              BtReturnCode::Running,
              ObfmG16HighNodeId,
              context.frame.frame_index,
              StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
void EvaluateObfmHighToLagOnce(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context)
    noexcept
{
    if (context.high_to_lag_evaluated)
    {
        return;
    }
    context.high_to_lag_evaluated = true;
    bool prepared_now = false;
    const BtTickResult prepared = EnsureObfmG16CandidateState(
        context, ObfmHighToLagTerminalNodeId, prepared_now);
    if (IsBtReturnCodeError(prepared.code))
    {
        context.high_to_lag_fault = prepared;
        context.high_to_lag_fault_available = true;
        return;
    }
    bool selected = false;
    Status status{};
    context.doctrine->SelectObfmG16HighToLag(selected, status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.high_to_lag_selected = selected;
    if (!selected)
    {
        const BtTickResult rollback = RollbackUnusedObfmCandidateState(
            context, ObfmHighToLagTerminalNodeId, prepared_now);
        if (IsBtReturnCodeError(rollback.code))
        {
            context.high_to_lag_fault = rollback;
            context.high_to_lag_fault_available = true;
        }
        return;
    }

    context.doctrine->ObserveObfmLagStation(status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    bool station_selected = false;
    context.doctrine->SelectObfmLagSpeedAuthority(
        ObfmLagSpeedAuthority::StationHold,
        station_selected,
        status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    if (!station_selected)
    {
        bool phase_selected = false;
        context.doctrine->SelectObfmLagSpeedAuthority(
            ObfmLagSpeedAuthority::PhaseLongitudinal,
            phase_selected,
            status);
        if (!status.ok() || !phase_selected)
        {
            context.high_to_lag_fault = status.ok()
                ? MakeObfmG16G5bResult(
                      BtReturnCode::InternalContractFault,
                      ObfmHighToLagTerminalNodeId,
                      context.frame.frame_index,
                      StaticDoctrineObfmG16G5bReason::StagedCandidateUnavailable)
                : MakeObfmG16G5bStatusResult<DoctrineContextT>(
                      status.code,
                      ObfmHighToLagTerminalNodeId,
                      ObfmG16G5bStageId,
                      context.frame.frame_index);
            context.high_to_lag_fault_available = true;
            return;
        }
    }

    context.doctrine->PrepareObfmLagBase(status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.doctrine->ObserveObfmLeadDiscipline(status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.doctrine->ObserveObfmTerminalTracking(status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.doctrine->SelectObfmTerminalTracking(
        context.high_to_lag_terminal_tracking,
        status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                ObfmHighToLagTerminalNodeId,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.doctrine->PublishObfmG16HighToLag(
        context.high_to_lag_terminal_tracking,
        status);
    const BtNodeId source_node = context.high_to_lag_terminal_tracking
        ? ObfmHighToLagTerminalNodeId
        : ObfmHighToLagNodeId;
    if (!status.ok())
    {
        const BtTickResult failure = ResolveObfmWriterFailure(
            context, status.code, source_node);
        if (failure.code == BtReturnCode::NotApplicable)
        {
            context.high_to_lag_selected = false;
            return;
        }
        context.high_to_lag_fault = failure;
        context.high_to_lag_fault_available = true;
        return;
    }
    context.doctrine->CopyStagedBaseIntent(
        context.high_to_lag_candidate,
        status);
    if (!status.ok())
    {
        context.high_to_lag_fault =
            MakeObfmG16G5bStatusResult<DoctrineContextT>(
                status.code,
                source_node,
                ObfmG16G5bStageId,
                context.frame.frame_index);
        context.high_to_lag_fault_available = true;
        return;
    }
    context.high_to_lag_candidate_ready = true;
}

template <typename DoctrineContextT, BtNodeId SourceNode, bool TerminalTracking>
BtTickResult EvaluateObfmHighToLagTerminalEvent(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent&) noexcept
{
    EvaluateObfmHighToLagOnce(context);
    if (context.high_to_lag_fault_available)
    {
        BtTickResult fault = context.high_to_lag_fault;
        fault.node_id = SourceNode;
        return fault;
    }
    if (!context.high_to_lag_selected ||
        context.high_to_lag_terminal_tracking != TerminalTracking)
    {
        return MakeObfmG16G5bNotApplicable(
            SourceNode, context.frame.frame_index);
    }
    if (!context.high_to_lag_candidate_ready)
    {
        return MakeObfmG16G5bResult(
            BtReturnCode::InternalContractFault,
            SourceNode,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::StagedCandidateUnavailable);
    }
    // The dynamic High service may produce a valid HighToLag handoff after a
    // prior command-neutral wait frame in which the static writer-8 owner was
    // already halted.  In that case writer 5 remains a valid current-frame
    // command, but there is no active owner-8 lifecycle to complete.
    if (!context.g16_high_owner_active_at_entry)
    {
        return MakeObfmG16G5bNotApplicable(
            SourceNode, context.frame.frame_index);
    }
    return MakeObfmG16G5bResult(
        BtReturnCode::Completed,
        SourceNode,
        context.frame.frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT, BtNodeId SourceNode, bool TerminalTracking>
BtTickResult EvaluateObfmHighToLagCandidate(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    EvaluateObfmHighToLagOnce(context);
    if (context.high_to_lag_fault_available)
    {
        BtTickResult fault = context.high_to_lag_fault;
        fault.node_id = SourceNode;
        return fault;
    }
    if (!context.high_to_lag_selected ||
        context.high_to_lag_terminal_tracking != TerminalTracking)
    {
        return MakeObfmG16G5bNotApplicable(
            SourceNode, context.frame.frame_index);
    }
    if (!context.high_to_lag_candidate_ready)
    {
        return MakeObfmG16G5bResult(
            BtReturnCode::InternalContractFault,
            SourceNode,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::StagedCandidateUnavailable);
    }
    proposed = context.high_to_lag_candidate;
    return MakeObfmG16G5bResult(
        BtReturnCode::Selected,
        SourceNode,
        context.frame.frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
void EvaluateObfmG5bOnce(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context)
    noexcept
{
    if (context.g5b_evaluated)
    {
        return;
    }
    context.g5b_evaluated = true;
    bool prepared_now = false;
    const BtTickResult prepared = EnsureObfmG16CandidateState(
        context, ObfmG5bCompleteNodeId, prepared_now);
    if (IsBtReturnCodeError(prepared.code))
    {
        context.g5b_fault = prepared;
        context.g5b_fault_available = true;
        return;
    }
    Status status{};
    context.doctrine->SelectObfmG5b(context.g5b_selected, status);
    if (!status.ok())
    {
        context.g5b_fault = MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG5bCompleteNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
        context.g5b_fault_available = true;
        return;
    }
    if (!context.g5b_selected)
    {
        const BtTickResult rollback = RollbackUnusedObfmCandidateState(
            context, ObfmG5bCompleteNodeId, prepared_now);
        if (IsBtReturnCodeError(rollback.code))
        {
            context.g5b_fault = rollback;
            context.g5b_fault_available = true;
        }
        return;
    }
    context.doctrine->ObserveObfmG5b(status);
    if (!status.ok())
    {
        context.g5b_fault = MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG5bCompleteNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
        context.g5b_fault_available = true;
        return;
    }
    bool normal_terminal_fallthrough = false;
    context.doctrine->CheckObfmG5bNormalTerminalFallthrough(
        normal_terminal_fallthrough,
        status);
    if (!status.ok())
    {
        context.g5b_fault = MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG5bCompleteNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
        context.g5b_fault_available = true;
        return;
    }
    if (normal_terminal_fallthrough)
    {
        context.g5b_selected = false;
        context.g5b_normal_terminal_fallthrough = true;
        const BtTickResult rollback = RollbackUnusedObfmCandidateState(
            context, ObfmG5bCompleteNodeId, prepared_now);
        if (IsBtReturnCodeError(rollback.code))
        {
            context.g5b_fault = rollback;
            context.g5b_fault_available = true;
        }
        return;
    }

    const std::array<guidance::obfm::G5bSelectedBranch, 5U> branches = {{
        guidance::obfm::G5bSelectedBranch::Complete,
        guidance::obfm::G5bSelectedBranch::Release,
        guidance::obfm::G5bSelectedBranch::ZoomEntry,
        guidance::obfm::G5bSelectedBranch::Extend,
        guidance::obfm::G5bSelectedBranch::ZoomClimb}};
    for (std::uint32_t index = 0U; index < branches.size(); ++index)
    {
        bool branch_selected = false;
        context.doctrine->SelectObfmG5bBranch(
            branches[index], branch_selected, status);
        if (!status.ok())
        {
            context.g5b_fault =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    ObfmG5bInvalidNodeId,
                    ObfmG16G5bStageId,
                    context.frame.frame_index);
            context.g5b_fault_available = true;
            return;
        }
        if (branch_selected)
        {
            context.g5b_branch = branches[index];
            context.g5b_branch_resolved = true;
            return;
        }
    }
}

template <guidance::obfm::G5bSelectedBranch Branch>
struct StaticDoctrineObfmG5bTerminalAction;

template <>
struct StaticDoctrineObfmG5bTerminalAction<
    guidance::obfm::G5bSelectedBranch::Complete>
{
    template <typename DoctrineContextT>
    static void Apply(
        StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
        Status& status) noexcept
    {
        context.doctrine->CompleteObfmG5b(status);
        if (!status.ok())
        {
            return;
        }
        std::size_t mode_child = 3U;
        context.doctrine->ResolveTacticalModeChild(mode_child, status);
        if (!status.ok())
        {
            return;
        }
        switch (mode_child)
        {
        case 0U:
            context.mode_redispatch = StaticDoctrineObfmModeRedispatch::None;
            return;
        case 1U:
            context.mode_redispatch = StaticDoctrineObfmModeRedispatch::Habfm;
            return;
        case 2U:
            context.mode_redispatch = StaticDoctrineObfmModeRedispatch::Dbfm;
            return;
        default:
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
    }
};

template <>
struct StaticDoctrineObfmG5bTerminalAction<
    guidance::obfm::G5bSelectedBranch::Release>
{
    template <typename DoctrineContextT>
    static void Apply(
        StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
        Status& status) noexcept
    {
        context.doctrine->ReleaseObfmG5b(status);
    }
};

template <typename DoctrineContextT,
          guidance::obfm::G5bSelectedBranch Branch,
          BtNodeId SourceNode,
          BtReturnCode TerminalCode>
BtTickResult EvaluateObfmG5bTerminalBranch(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent&) noexcept
{
    EvaluateObfmG5bOnce(context);
    if (context.g5b_fault_available)
    {
        BtTickResult fault = context.g5b_fault;
        fault.node_id = SourceNode;
        return fault;
    }
    if (!context.g5b_selected || !context.g5b_branch_resolved ||
        context.g5b_branch != Branch)
    {
        return MakeObfmG16G5bNotApplicable(
            SourceNode, context.frame.frame_index);
    }
    if (context.g5b_action_applied)
    {
        return MakeObfmG16G5bResult(
            BtReturnCode::InternalContractFault,
            SourceNode,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::G5bInvalidPhase);
    }
    context.g5b_action_applied = true;
    Status status{};
    StaticDoctrineObfmG5bTerminalAction<Branch>::Apply(
        context, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            SourceNode,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    if (context.g5b_owner_active_at_entry)
    {
        return MakeObfmG16G5bResult(
            TerminalCode,
            SourceNode,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::None);
    }
    // A first-tick RELEASE performs the real G5b cleanup but has no static
    // owner-7 lifecycle to terminate. Continue to a current-frame command
    // without manufacturing a terminal-owner event.
    return MakeObfmG16G5bNotApplicable(
        SourceNode, context.frame.frame_index);
}

template <typename DoctrineContextT, StaticDoctrineObfmModeRedispatch Mode>
struct StaticDoctrineObfmRedispatchLeafSelector;

template <typename DoctrineContextT>
struct StaticDoctrineObfmRedispatchLeafSelector<
    DoctrineContextT,
    StaticDoctrineObfmModeRedispatch::Habfm>
{
    static const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& Get(
        const StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>&
            context) noexcept
    {
        return context.redispatch_leaves.habfm;
    }
};

template <typename DoctrineContextT>
struct StaticDoctrineObfmRedispatchLeafSelector<
    DoctrineContextT,
    StaticDoctrineObfmModeRedispatch::Dbfm>
{
    static const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& Get(
        const StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>&
            context) noexcept
    {
        return context.redispatch_leaves.dbfm;
    }
};

template <typename DoctrineContextT, StaticDoctrineObfmModeRedispatch Mode>
BtTickResult EvaluateObfmModeRedispatch(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& leaf =
        StaticDoctrineObfmRedispatchLeafSelector<DoctrineContextT, Mode>::Get(
            context);
    if (context.mode_redispatch != Mode)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            leaf.descriptor.node_id,
            leaf.descriptor.stage_id,
            context.frame.frame_index,
            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
    }

    const BtTickResult selected = leaf.callback(
        *context.doctrine, established_base, proposed);
    if (selected.code != BtReturnCode::NotApplicable)
    {
        return selected;
    }
    // A completed G5b mode change cannot fall back into OBFM. The caller must
    // provide the current-frame HABFM/DBFM candidate in this same transaction.
    return MakeBtTickResult(
        BtReturnCode::MissingRequiredInput,
        leaf.descriptor.node_id,
        leaf.descriptor.stage_id,
        context.frame.frame_index,
        ToReasonId(
            StaticDoctrineObfmG16G5bReason::
                TerminalWithoutCurrentFrameCandidate));
}

template <typename DoctrineContextT,
          guidance::obfm::G5bSelectedBranch Branch,
          BtNodeId SourceNode>
BtTickResult EvaluateObfmG5bRunningBranch(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    EvaluateObfmG5bOnce(context);
    if (context.g5b_fault_available)
    {
        BtTickResult fault = context.g5b_fault;
        fault.node_id = SourceNode;
        return fault;
    }
    if (!context.g5b_selected || !context.g5b_branch_resolved ||
        context.g5b_branch != Branch)
    {
        return MakeObfmG16G5bNotApplicable(
            SourceNode, context.frame.frame_index);
    }
    if (context.g5b_action_applied)
    {
        return MakeObfmG16G5bResult(
            BtReturnCode::InternalContractFault,
            SourceNode,
            context.frame.frame_index,
            StaticDoctrineObfmG16G5bReason::G5bInvalidPhase);
    }
    context.g5b_action_applied = true;
    Status status{};
    context.doctrine->PublishObfmG5b(Branch, status);
    if (!status.ok())
    {
        return ResolveObfmWriterFailure(
            context, status.code, SourceNode);
    }
    context.doctrine->CopyStagedBaseIntent(
        context.g5b_candidate, status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            SourceNode,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    context.g5b_candidate_ready = true;
    proposed = context.g5b_candidate;
    return MakeObfmG16G5bResult(
        BtReturnCode::Running,
        SourceNode,
        context.frame.frame_index,
        StaticDoctrineObfmG16G5bReason::None);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG5bInvalid(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent*,
    ControlIntent&) noexcept
{
    EvaluateObfmG5bOnce(context);
    if (context.g5b_fault_available)
    {
        BtTickResult fault = context.g5b_fault;
        fault.node_id = ObfmG5bInvalidNodeId;
        return fault;
    }
    if (!context.g5b_selected || context.g5b_branch_resolved)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmG5bInvalidNodeId, context.frame.frame_index);
    }
    Status status{};
    context.doctrine->FailObfmG5bInvalid(status);
    if (!status.ok())
    {
        return MakeObfmG16G5bStatusResult<DoctrineContextT>(
            status.code,
            ObfmG5bInvalidNodeId,
            ObfmG16G5bStageId,
            context.frame.frame_index);
    }
    // An optional G5b phase that did not materialize is the legacy
    // ForceFailure fallthrough contract, not a malformed frame.  The cleanup
    // above is part of this candidate's prepared owner-state delta; continue
    // in the same transaction so the bounded lower OBFM leaf supplies the
    // current-frame command and both changes commit together.
    return MakeObfmG16G5bNotApplicable(
        ObfmG5bInvalidNodeId, context.frame.frame_index);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmEmployLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.employ_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmEmployNodeId, context.frame.frame_index);
    }
    return context.employ_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmEntryLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.entry_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmEntryNodeId, context.frame.frame_index);
    }
    const BtTickResult result = context.entry_leaf.callback(
        *context.doctrine, established_base, proposed);
    context.entry_handoff_to_lag_this_tick =
        context.entry_owner_active_at_entry
        && !IsCandidateProducingCode(result.code);
    return result;
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmSpacingLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    // When an active ENTRY owner completes or loses its current observation,
    // its same-tick command owner is the precision LAG terminal.  A newly
    // admitted SPACING episode must not intercept that lifecycle handoff.
    // SPACING remains available on ticks that did not begin in ENTRY.
    if (context.entry_handoff_to_lag_this_tick)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmSpacingNodeId, context.frame.frame_index);
    }
    if (context.spacing_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmSpacingNodeId, context.frame.frame_index);
    }
    return context.spacing_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG3RollCounterLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.g3_roll_counter_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmG3CounterBarrelNodeId, context.frame.frame_index);
    }
    return context.g3_roll_counter_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG3CounterRollingScissorsLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.g3_counter_rolling_scissors_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmG3CounterRollingScissorsNodeId,
            context.frame.frame_index);
    }
    return context.g3_counter_rolling_scissors_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmG3ScissorsLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.g3_scissors_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmG3ScissorsNodeId, context.frame.frame_index);
    }
    return context.g3_scissors_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmApexLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    if (context.apex_leaf.callback == nullptr)
    {
        return MakeObfmG16G5bNotApplicable(
            ObfmApexNodeId, context.frame.frame_index);
    }
    return context.apex_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult EvaluateObfmLowerLeaf(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const ControlIntent* established_base,
    ControlIntent& proposed) noexcept
{
    return context.lower_leaf.callback(
        *context.doctrine, established_base, proposed);
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG16Committed(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltG16Committed();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG3RollCounter(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmG3RollCounter();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG3CounterRollingScissors(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmG3CounterRollingScissors();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG3Scissors(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmG3Scissors();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmApex(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmApex();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG16High(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>&,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    // Parity with TaskObfmG16High::onHalted(): the reactive leaf halt is
    // command-neutral.  Root/mode observation owns the physical High reset.
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmG5b(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    guidance::obfm::G5bSelectedBranch branch =
        guidance::obfm::G5bSelectedBranch::Invalid;
    if (node_id == ObfmG5bZoomEntryNodeId)
    {
        branch = guidance::obfm::G5bSelectedBranch::ZoomEntry;
    }
    else if (node_id == ObfmG5bExtendNodeId)
    {
        branch = guidance::obfm::G5bSelectedBranch::Extend;
    }
    else if (node_id == ObfmG5bZoomClimbNodeId)
    {
        branch = guidance::obfm::G5bSelectedBranch::ZoomClimb;
    }
    if (branch == guidance::obfm::G5bSelectedBranch::Invalid)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            node_id,
            stage_id,
            frame_index,
            ToReasonId(
                StaticDoctrineObfmG16G5bReason::G5bActivePhaseUnknown));
    }
    context.doctrine->HaltObfmG5b(branch);
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmEntry(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmEntry();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

template <typename DoctrineContextT>
BtTickResult HaltObfmSpacing(
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>& context,
    const BtLifecycleOwnerId,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    context.doctrine->HaltObfmSpacing();
    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
}

constexpr std::size_t StaticDoctrineObfmG16G5bChildCount = 22U;
constexpr std::size_t StaticDoctrineObfmG16G5bHaltOwnerCount = 9U;

template <typename DoctrineContextT>
StaticBtChildTable<StaticDoctrineObfmG16G5bChildCount>
MakeObfmG16G5bChildTable(
    const StaticBtChildDescriptor employ_descriptor,
    const StaticBtChildDescriptor g3_roll_counter_descriptor,
    const StaticBtChildDescriptor g3_counter_rolling_scissors_descriptor,
    const StaticBtChildDescriptor g3_scissors_descriptor,
    const StaticBtChildDescriptor apex_descriptor,
    const StaticBtChildDescriptor spacing_descriptor,
    const StaticBtChildDescriptor entry_descriptor,
    const StaticBtChildDescriptor lower_descriptor,
    const StaticBtChildDescriptor habfm_redispatch_descriptor,
    const StaticBtChildDescriptor dbfm_redispatch_descriptor) noexcept
{
    StaticBtChildTable<StaticDoctrineObfmG16G5bChildCount> output{};
    output.children = {{
        StaticBtChildDescriptor{employ_descriptor.node_id,
                                employ_descriptor.stage_id,
                                0U,
                                employ_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{ObfmG16CommittedNodeId,
                                ObfmG16G5bStageId,
                                1U,
                                ObfmG16CommittedOwnerId},
        StaticBtChildDescriptor{ObfmG16HighNodeId,
                                ObfmG16G5bStageId,
                                2U,
                                ObfmG16HighOwnerId},
        StaticBtChildDescriptor{ObfmHighToLagTerminalNodeId,
                                ObfmG16G5bStageId,
                                3U,
                                ObfmG16HighOwnerId},
        StaticBtChildDescriptor{ObfmHighToLagNodeId,
                                ObfmG16G5bStageId,
                                4U,
                                ObfmG16HighOwnerId},
        StaticBtChildDescriptor{ObfmHighToLagTerminalNodeId,
                                ObfmG16G5bStageId,
                                5U,
                                BtLifecycleOwnerIdNone},
        StaticBtChildDescriptor{ObfmHighToLagNodeId,
                                ObfmG16G5bStageId,
                                6U,
                                BtLifecycleOwnerIdNone},
        StaticBtChildDescriptor{ObfmG5bCompleteNodeId,
                                ObfmG16G5bStageId,
                                7U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{ObfmG5bReleaseNodeId,
                                ObfmG16G5bStageId,
                                8U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{habfm_redispatch_descriptor.node_id,
                                habfm_redispatch_descriptor.stage_id,
                                9U,
                                habfm_redispatch_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{dbfm_redispatch_descriptor.node_id,
                                dbfm_redispatch_descriptor.stage_id,
                                10U,
                                dbfm_redispatch_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{ObfmG5bZoomEntryNodeId,
                                ObfmG16G5bStageId,
                                11U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{ObfmG5bExtendNodeId,
                                ObfmG16G5bStageId,
                                12U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{ObfmG5bZoomClimbNodeId,
                                ObfmG16G5bStageId,
                                13U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{ObfmG5bInvalidNodeId,
                                ObfmG16G5bStageId,
                                14U,
                                ObfmG5bOwnerId},
        StaticBtChildDescriptor{g3_roll_counter_descriptor.node_id,
                                g3_roll_counter_descriptor.stage_id,
                                15U,
                                g3_roll_counter_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{
                                g3_counter_rolling_scissors_descriptor.node_id,
                                g3_counter_rolling_scissors_descriptor.stage_id,
                                16U,
                                g3_counter_rolling_scissors_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{g3_scissors_descriptor.node_id,
                                g3_scissors_descriptor.stage_id,
                                17U,
                                g3_scissors_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{apex_descriptor.node_id,
                                apex_descriptor.stage_id,
                                18U,
                                apex_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{entry_descriptor.node_id,
                                entry_descriptor.stage_id,
                                19U,
                                entry_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{spacing_descriptor.node_id,
                                spacing_descriptor.stage_id,
                                20U,
                                spacing_descriptor.lifecycle_owner_id},
        StaticBtChildDescriptor{lower_descriptor.node_id,
                                lower_descriptor.stage_id,
                                21U,
                                lower_descriptor.lifecycle_owner_id}}};
    output.child_count =
        static_cast<std::uint32_t>(StaticDoctrineObfmG16G5bChildCount);
    return output;
}

template <typename DoctrineContextT>
StaticBtCallbackTable<
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>,
    StaticDoctrineObfmG16G5bChildCount>
MakeObfmG16G5bCallbackTable() noexcept
{
    typedef StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>
        EvaluationContext;
    StaticBtCallbackTable<EvaluationContext,
                          StaticDoctrineObfmG16G5bChildCount> output{};
    output.callbacks = {{
        &EvaluateObfmEmployLeaf<DoctrineContextT>,
        &EvaluateObfmG16Committed<DoctrineContextT>,
        &EvaluateObfmG16High<DoctrineContextT>,
        &EvaluateObfmHighToLagTerminalEvent<
            DoctrineContextT, ObfmHighToLagTerminalNodeId, true>,
        &EvaluateObfmHighToLagTerminalEvent<
            DoctrineContextT, ObfmHighToLagNodeId, false>,
        &EvaluateObfmHighToLagCandidate<
            DoctrineContextT, ObfmHighToLagTerminalNodeId, true>,
        &EvaluateObfmHighToLagCandidate<
            DoctrineContextT, ObfmHighToLagNodeId, false>,
        &EvaluateObfmG5bTerminalBranch<
            DoctrineContextT,
            guidance::obfm::G5bSelectedBranch::Complete,
            ObfmG5bCompleteNodeId,
            BtReturnCode::Completed>,
        &EvaluateObfmG5bTerminalBranch<
            DoctrineContextT,
            guidance::obfm::G5bSelectedBranch::Release,
            ObfmG5bReleaseNodeId,
            BtReturnCode::Released>,
        &EvaluateObfmModeRedispatch<
            DoctrineContextT, StaticDoctrineObfmModeRedispatch::Habfm>,
        &EvaluateObfmModeRedispatch<
            DoctrineContextT, StaticDoctrineObfmModeRedispatch::Dbfm>,
        &EvaluateObfmG5bRunningBranch<
            DoctrineContextT,
            guidance::obfm::G5bSelectedBranch::ZoomEntry,
            ObfmG5bZoomEntryNodeId>,
        &EvaluateObfmG5bRunningBranch<
            DoctrineContextT,
            guidance::obfm::G5bSelectedBranch::Extend,
            ObfmG5bExtendNodeId>,
        &EvaluateObfmG5bRunningBranch<
            DoctrineContextT,
            guidance::obfm::G5bSelectedBranch::ZoomClimb,
            ObfmG5bZoomClimbNodeId>,
        &EvaluateObfmG5bInvalid<DoctrineContextT>,
        &EvaluateObfmG3RollCounterLeaf<DoctrineContextT>,
        &EvaluateObfmG3CounterRollingScissorsLeaf<DoctrineContextT>,
        &EvaluateObfmG3ScissorsLeaf<DoctrineContextT>,
        &EvaluateObfmApexLeaf<DoctrineContextT>,
        &EvaluateObfmEntryLeaf<DoctrineContextT>,
        &EvaluateObfmSpacingLeaf<DoctrineContextT>,
        &EvaluateObfmLowerLeaf<DoctrineContextT>}};
    output.callback_count =
        static_cast<std::uint32_t>(StaticDoctrineObfmG16G5bChildCount);
    return output;
}

template <typename DoctrineContextT>
StaticBtHaltBindingTable<StaticDoctrineObfmG16G5bHaltOwnerCount>
MakeObfmG16G5bHaltBindings() noexcept
{
    StaticBtHaltBindingTable<StaticDoctrineObfmG16G5bHaltOwnerCount> output{};
    output.bindings = {{
        StaticBtHaltBinding{ObfmG16CommittedOwnerId, 0U},
        StaticBtHaltBinding{ObfmG16HighOwnerId, 1U},
        StaticBtHaltBinding{ObfmG5bOwnerId, 2U},
        StaticBtHaltBinding{ObfmG3CounterBarrelOwnerId, 3U},
        StaticBtHaltBinding{ObfmG3CounterRollingScissorsOwnerId, 4U},
        StaticBtHaltBinding{ObfmG3ScissorsOwnerId, 5U},
        StaticBtHaltBinding{ObfmApexOwnerId, 6U},
        StaticBtHaltBinding{ObfmSpacingOwnerId, 7U},
        StaticBtHaltBinding{ObfmEntryOwnerId, 8U}}};
    output.binding_count =
        static_cast<std::uint32_t>(StaticDoctrineObfmG16G5bHaltOwnerCount);
    return output;
}

template <typename DoctrineContextT>
StaticBtHaltCallbackTable<
    StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>,
    StaticDoctrineObfmG16G5bHaltOwnerCount>
MakeObfmG16G5bHaltCallbacks() noexcept
{
    typedef StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>
        EvaluationContext;
    StaticBtHaltCallbackTable<EvaluationContext,
                              StaticDoctrineObfmG16G5bHaltOwnerCount> output{};
    output.callbacks = {{
        &HaltObfmG16Committed<DoctrineContextT>,
        &HaltObfmG16High<DoctrineContextT>,
        &HaltObfmG5b<DoctrineContextT>,
        &HaltObfmG3RollCounter<DoctrineContextT>,
        &HaltObfmG3CounterRollingScissors<DoctrineContextT>,
        &HaltObfmG3Scissors<DoctrineContextT>,
        &HaltObfmApex<DoctrineContextT>,
        &HaltObfmSpacing<DoctrineContextT>,
        &HaltObfmEntry<DoctrineContextT>}};
    output.callback_count =
        static_cast<std::uint32_t>(StaticDoctrineObfmG16G5bHaltOwnerCount);
    return output;
}

template <typename DoctrineContextT>
class StaticDoctrineObfmG16G5bAdapter final
{
public:
    StaticDoctrineObfmG16G5bResult Evaluate(
        DoctrineContextT& doctrine,
        const ControlFrameIdentity& current_frame_identity,
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& lower_leaf,
        const StaticDoctrineObfmModeRedispatchLeaves<DoctrineContextT>&
            redispatch_leaves,
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& employ_leaf =
            StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>&
            g3_roll_counter_leaf =
                StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>&
            g3_counter_rolling_scissors_leaf =
                StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>&
            g3_scissors_leaf =
                StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& spacing_leaf =
            StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& entry_leaf =
            StaticDoctrineObfmLowerLeaf<DoctrineContextT>{},
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& apex_leaf =
            StaticDoctrineObfmLowerLeaf<DoctrineContextT>{}) noexcept
    {
        StaticDoctrineObfmG16G5bResult output{};
        if (pending_commit_)
        {
            output.lifecycle.selection.result = MakeObfmG16G5bResult(
                BtReturnCode::InternalContractFault,
                ObfmG16G5bObservationNodeId,
                current_frame_identity.frame_index,
                StaticDoctrineObfmG16G5bReason::TransactionAlreadyPending);
            return output;
        }
        const bool employ_leaf_present = employ_leaf.callback != nullptr;
        const bool g3_roll_counter_leaf_present =
            g3_roll_counter_leaf.callback != nullptr;
        const bool g3_counter_rolling_scissors_leaf_present =
            g3_counter_rolling_scissors_leaf.callback != nullptr;
        const bool g3_scissors_leaf_present =
            g3_scissors_leaf.callback != nullptr;
        const bool apex_leaf_present = apex_leaf.callback != nullptr;
        const bool spacing_leaf_present = spacing_leaf.callback != nullptr;
        const bool entry_leaf_present = entry_leaf.callback != nullptr;
        if ((employ_leaf_present
                && (employ_leaf.descriptor.node_id == BtNodeIdInvalid
                    || employ_leaf.descriptor.stage_id == BtStageIdInvalid))
            || lower_leaf.callback == nullptr
            || (g3_roll_counter_leaf_present
                && (g3_roll_counter_leaf.descriptor.node_id
                        == BtNodeIdInvalid
                    || g3_roll_counter_leaf.descriptor.stage_id
                        == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        g3_roll_counter_leaf.descriptor.lifecycle_owner_id)))
            || (g3_counter_rolling_scissors_leaf_present
                && (g3_counter_rolling_scissors_leaf.descriptor.node_id
                        == BtNodeIdInvalid
                    || g3_counter_rolling_scissors_leaf.descriptor.stage_id
                        == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        g3_counter_rolling_scissors_leaf.descriptor
                            .lifecycle_owner_id)))
            || (g3_scissors_leaf_present
                && (g3_scissors_leaf.descriptor.node_id == BtNodeIdInvalid
                    || g3_scissors_leaf.descriptor.stage_id == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        g3_scissors_leaf.descriptor.lifecycle_owner_id)))
            || (apex_leaf_present
                && (apex_leaf.descriptor.node_id == BtNodeIdInvalid
                    || apex_leaf.descriptor.stage_id == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        apex_leaf.descriptor.lifecycle_owner_id)))
            || (spacing_leaf_present
                && (spacing_leaf.descriptor.node_id == BtNodeIdInvalid
                    || spacing_leaf.descriptor.stage_id == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        spacing_leaf.descriptor.lifecycle_owner_id)))
            || (entry_leaf_present
                && (entry_leaf.descriptor.node_id == BtNodeIdInvalid
                    || entry_leaf.descriptor.stage_id == BtStageIdInvalid
                    || !HasBtLifecycleOwnerId(
                        entry_leaf.descriptor.lifecycle_owner_id)))
            || lower_leaf.descriptor.node_id == BtNodeIdInvalid
            || lower_leaf.descriptor.stage_id == BtStageIdInvalid
            || redispatch_leaves.habfm.callback == nullptr
            || redispatch_leaves.habfm.descriptor.node_id == BtNodeIdInvalid
            || redispatch_leaves.habfm.descriptor.stage_id == BtStageIdInvalid
            || redispatch_leaves.dbfm.callback == nullptr
            || redispatch_leaves.dbfm.descriptor.node_id == BtNodeIdInvalid
            || redispatch_leaves.dbfm.descriptor.stage_id == BtStageIdInvalid)
        {
            output.lifecycle.selection.result = MakeObfmG16G5bResult(
                BtReturnCode::InvalidTopology,
                ObfmG16G5bObservationNodeId,
                current_frame_identity.frame_index,
                StaticDoctrineObfmG16G5bReason::FallbackDescriptorInvalid);
            return output;
        }

        // Frozen XML parity is intentional: the command-neutral physical G16
        // observation/reset precedes the candidate-state transaction.
        Status status{};
        doctrine.ObserveObfmG16HighPhysical(status);
        if (!status.ok())
        {
            output.lifecycle.selection.result =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    ObfmG16G5bObservationNodeId,
                    ObfmG16G5bStageId,
                    current_frame_identity.frame_index);
            return output;
        }
        if (apex_leaf_present)
        {
            doctrine.ObserveObfmApexPhysical(status);
            if (!status.ok())
            {
                // Apex is an optional post-High geometry owner.  A malformed
                // optional candidate must not erase the constructible LAG
                // command for the accepted current frame.
                doctrine.HaltObfmApex();
                status = Status{};
            }
        }
        if (employ_leaf_present)
        {
            doctrine.ObserveObfmEmploy(status);
            if (!status.ok())
            {
                output.lifecycle.selection.result =
                    MakeObfmG16G5bStatusResult<DoctrineContextT>(
                        status.code,
                        ObfmG16G5bObservationNodeId,
                        ObfmG16G5bStageId,
                        current_frame_identity.frame_index);
                return output;
            }
        }
        if (entry_leaf_present)
        {
            doctrine.ObserveObfmEntryPhysical(status);
            if (!status.ok())
            {
                output.lifecycle.selection.result =
                    MakeObfmG16G5bStatusResult<DoctrineContextT>(
                        status.code,
                        ObfmG16G5bObservationNodeId,
                        ObfmG16G5bStageId,
                        current_frame_identity.frame_index);
                return output;
            }
        }

        doctrine.BeginFinalCommandCandidateStage(status);
        if (!status.ok())
        {
            output.lifecycle.selection.result =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    ObfmG16G5bObservationNodeId,
                    ObfmG16G5bStageId,
                    current_frame_identity.frame_index);
            return output;
        }
        // Preserve the SPACING post-hit token while EMPLOY retains command
        // priority. This observation is command-neutral and participates in
        // the same candidate-state transaction as the selected EMPLOY leaf.
        if (employ_leaf_present)
        {
            doctrine.ObserveObfmSpacingEmployPreemption(status);
            if (!status.ok())
            {
                doctrine.AbortPreparedFinalState();
                output.lifecycle.selection.result =
                    MakeObfmG16G5bStatusResult<DoctrineContextT>(
                        status.code,
                        ObfmSpacingNodeId,
                        ObfmG16G5bStageId,
                        current_frame_identity.frame_index);
                return output;
            }
        }
        doctrine.EvaluateObfmG16HighCandidate(status);
        if (!status.ok())
        {
            doctrine.AbortPreparedFinalState();
            const BtTickResult optional_fault =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    ObfmG16G5bObservationNodeId,
                    ObfmG16G5bStageId,
                    current_frame_identity.frame_index);
            return EvaluateCurrentFrameLower(
                doctrine,
                current_frame_identity,
                lower_leaf,
                optional_fault);
        }

        typedef StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>
            EvaluationContext;
        EvaluationContext evaluation{};
        evaluation.doctrine = &doctrine;
        evaluation.frame = current_frame_identity;
        evaluation.employ_leaf = employ_leaf;
        evaluation.g3_roll_counter_leaf = g3_roll_counter_leaf;
        evaluation.g3_counter_rolling_scissors_leaf =
            g3_counter_rolling_scissors_leaf;
        evaluation.g3_scissors_leaf = g3_scissors_leaf;
        evaluation.apex_leaf = apex_leaf;
        evaluation.spacing_leaf = spacing_leaf;
        evaluation.entry_leaf = entry_leaf;
        if (!employ_leaf_present)
        {
            evaluation.employ_leaf.descriptor = StaticBtChildDescriptor{
                ObfmEmployNodeId,
                ObfmG16G5bStageId,
                0U,
                BtLifecycleOwnerIdNone};
        }
        if (!entry_leaf_present)
        {
            evaluation.entry_leaf.descriptor = StaticBtChildDescriptor{
                ObfmEntryNodeId,
                ObfmG16G5bStageId,
                0U,
                BtLifecycleOwnerIdNone};
        }
        if (!g3_roll_counter_leaf_present)
        {
            evaluation.g3_roll_counter_leaf.descriptor =
                StaticBtChildDescriptor{
                    ObfmG3CounterBarrelNodeId,
                    ObfmG16G5bStageId,
                    0U,
                    BtLifecycleOwnerIdNone};
        }
        if (!g3_counter_rolling_scissors_leaf_present)
        {
            evaluation.g3_counter_rolling_scissors_leaf.descriptor =
                StaticBtChildDescriptor{
                    ObfmG3CounterRollingScissorsNodeId,
                    ObfmG16G5bStageId,
                    0U,
                    BtLifecycleOwnerIdNone};
        }
        if (!g3_scissors_leaf_present)
        {
            evaluation.g3_scissors_leaf.descriptor =
                StaticBtChildDescriptor{
                    ObfmG3ScissorsNodeId,
                    ObfmG16G5bStageId,
                    0U,
                    BtLifecycleOwnerIdNone};
        }
        if (!spacing_leaf_present)
        {
            evaluation.spacing_leaf.descriptor = StaticBtChildDescriptor{
                ObfmSpacingNodeId,
                ObfmG16G5bStageId,
                0U,
                BtLifecycleOwnerIdNone};
        }
        if (!apex_leaf_present)
        {
            evaluation.apex_leaf.descriptor = StaticBtChildDescriptor{
                ObfmApexNodeId,
                ObfmG16G5bStageId,
                0U,
                BtLifecycleOwnerIdNone};
        }
        evaluation.lower_leaf = lower_leaf;
        evaluation.redispatch_leaves = redispatch_leaves;
        evaluation.g16_candidate_state_ready = true;

        StaticBtLifecycleState working_state = lifecycle_state_;
        const auto halt_bindings =
            MakeObfmG16G5bHaltBindings<DoctrineContextT>();
        const auto halt_callbacks =
            MakeObfmG16G5bHaltCallbacks<DoctrineContextT>();
        if (working_state.episode_epoch !=
            current_frame_identity.episode_epoch)
        {
            const StaticBtLifecycleTransitionReceipt reset =
                ResetStaticBtLifecycleForEpoch(
                    evaluation,
                    working_state,
                    halt_bindings,
                    halt_callbacks,
                    current_frame_identity.episode_epoch,
                    current_frame_identity.frame_index,
                    ObfmG16G5bObservationNodeId,
                    ObfmG16G5bStageId);
            if (IsBtReturnCodeError(reset.result.code))
            {
                doctrine.AbortPreparedFinalState();
                output.lifecycle.selection.result = reset.result;
                output.lifecycle.transition = reset;
                return output;
            }
        }
        evaluation.g5b_owner_active_at_entry =
            SameActiveLifecycleOwner(working_state, ObfmG5bOwnerId);
        evaluation.g16_high_owner_active_at_entry =
            SameActiveLifecycleOwner(working_state, ObfmG16HighOwnerId);
        evaluation.entry_owner_active_at_entry =
            SameActiveLifecycleOwner(working_state, ObfmEntryOwnerId);

        const auto child_table =
            MakeObfmG16G5bChildTable<DoctrineContextT>(
                evaluation.employ_leaf.descriptor,
                evaluation.g3_roll_counter_leaf.descriptor,
                evaluation.g3_counter_rolling_scissors_leaf.descriptor,
                evaluation.g3_scissors_leaf.descriptor,
                evaluation.apex_leaf.descriptor,
                evaluation.spacing_leaf.descriptor,
                evaluation.entry_leaf.descriptor,
                lower_leaf.descriptor,
                redispatch_leaves.habfm.descriptor,
                redispatch_leaves.dbfm.descriptor);
        const auto callback_table =
            MakeObfmG16G5bCallbackTable<DoctrineContextT>();
        output.lifecycle = EvaluateStaticBtLifecycleChildren(
            evaluation,
            working_state,
            child_table,
            callback_table,
            halt_bindings,
            halt_callbacks,
            nullptr,
            current_frame_identity,
            current_frame_identity.frame_index);
        output.mode_redispatch = evaluation.mode_redispatch;
        if (IsBtReturnCodeError(output.lifecycle.selection.result.code))
        {
            const BtTickResult optional_fault =
                output.lifecycle.selection.result;
            doctrine.AbortPreparedFinalState();
            // A terminal redispatch belongs to the newly classified current
            // mode.  Do not resurrect OBFM LAG when that HABFM/DBFM terminal
            // cannot construct its own current-frame command.
            if (evaluation.mode_redispatch ==
                    StaticDoctrineObfmModeRedispatch::None &&
                optional_fault.node_id != lower_leaf.descriptor.node_id)
            {
                return EvaluateCurrentFrameLower(
                    doctrine,
                    current_frame_identity,
                    lower_leaf,
                    optional_fault);
            }
            return output;
        }
        if (!output.lifecycle.selection.candidate_available)
        {
            doctrine.AbortPreparedFinalState();
            const StaticBtLifecycleState previous = lifecycle_state_;
            const BtNodeId fault_node =
                output.lifecycle.selection.lifecycle_event_available
                    ? output.lifecycle.selection.lifecycle_event.node_id
                    : lower_leaf.descriptor.node_id;
            const BtTickResult fault = MakeObfmG16G5bResult(
                BtReturnCode::MissingRequiredInput,
                fault_node,
                current_frame_identity.frame_index,
                StaticDoctrineObfmG16G5bReason::
                    TerminalWithoutCurrentFrameCandidate);
            output.lifecycle.selection.result = fault;
            output.lifecycle.transition = MakeLifecycleReceipt(
                fault,
                previous.active_owner_id,
                previous.active_node_id,
                previous.active_stage_id,
                previous.active_owner_id,
                previous.active_node_id,
                previous.active_stage_id,
                StaticBtLifecycleTransitionKind::FaultNoMutation,
                false,
                false);
            return output;
        }

        doctrine.BeginDeferredFinalStateCommit(status);
        if (!status.ok())
        {
            const BtTickResult optional_fault =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    output.lifecycle.selection.result.node_id,
                    output.lifecycle.selection.result.stage_id,
                    current_frame_identity.frame_index);
            doctrine.AbortPreparedFinalState();
            if (evaluation.mode_redispatch ==
                    StaticDoctrineObfmModeRedispatch::None &&
                optional_fault.node_id != lower_leaf.descriptor.node_id)
            {
                return EvaluateCurrentFrameLower(
                    doctrine,
                    current_frame_identity,
                    lower_leaf,
                    optional_fault);
            }
            output.lifecycle.selection.candidate_available = false;
            output.lifecycle.selection.result = optional_fault;
            return output;
        }

        pending_lifecycle_state_ = working_state;
        pending_candidate_ = output.lifecycle.selection.candidate;
        pending_selected_leaf_ = output.lifecycle.selection.result;
        pending_commit_ = true;
        output.pending_commit = true;
        return output;
    }

    BtTickResult Commit(DoctrineContextT& doctrine) noexcept
    {
        if (!pending_commit_)
        {
            return MakeObfmG16G5bResult(
                BtReturnCode::InternalContractFault,
                ObfmG16G5bObservationNodeId,
                0U,
                StaticDoctrineObfmG16G5bReason::CommitNotPending);
        }
        Status status{};
        doctrine.PublishIntegratedFinalIntent(pending_candidate_, status);
        if (!status.ok())
        {
            const BtTickResult fault =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    pending_selected_leaf_.node_id,
                    pending_selected_leaf_.stage_id,
                    pending_selected_leaf_.frame_index);
            doctrine.AbortPreparedFinalState();
            ClearPending();
            return fault;
        }
        doctrine.CommitPreparedFinalState(status);
        if (!status.ok())
        {
            const BtTickResult fault =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    pending_selected_leaf_.node_id,
                    pending_selected_leaf_.stage_id,
                    pending_selected_leaf_.frame_index);
            ClearPending();
            return fault;
        }
        lifecycle_state_ = pending_lifecycle_state_;
        const BtTickResult result = MakeBtTickResult(
            BtReturnCode::Completed,
            pending_selected_leaf_.node_id,
            pending_selected_leaf_.stage_id,
            pending_selected_leaf_.frame_index,
            ToReasonId(StaticDoctrineObfmG16G5bReason::None));
        ClearPending();
        return result;
    }

    void Abort(DoctrineContextT& doctrine) noexcept
    {
        if (pending_commit_)
        {
            doctrine.AbortPreparedFinalState();
            ClearPending();
        }
    }

    const StaticBtLifecycleState& LifecycleState() const noexcept
    {
        return lifecycle_state_;
    }

    bool PendingCommit() const noexcept
    {
        return pending_commit_;
    }

private:
    StaticDoctrineObfmG16G5bResult EvaluateCurrentFrameLower(
        DoctrineContextT& doctrine,
        const ControlFrameIdentity& current_frame_identity,
        const StaticDoctrineObfmLowerLeaf<DoctrineContextT>& lower_leaf,
        const BtTickResult& optional_fault) noexcept
    {
        StaticDoctrineObfmG16G5bResult output{};
        output.current_base_recovered = true;
        output.optional_fault_available = true;
        output.optional_fault = optional_fault;

        Status status{};
        doctrine.BeginFinalCommandCandidateStage(status);
        if (!status.ok())
        {
            output.lifecycle.selection.result =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    lower_leaf.descriptor.node_id,
                    lower_leaf.descriptor.stage_id,
                    current_frame_identity.frame_index);
            return output;
        }

        typedef StaticDoctrineObfmG16G5bEvaluationContext<DoctrineContextT>
            EvaluationContext;
        EvaluationContext evaluation{};
        evaluation.doctrine = &doctrine;
        evaluation.frame = current_frame_identity;
        evaluation.lower_leaf = lower_leaf;

        StaticBtLifecycleState working_state = lifecycle_state_;
        const auto halt_bindings =
            MakeObfmG16G5bHaltBindings<DoctrineContextT>();
        const auto halt_callbacks =
            MakeObfmG16G5bHaltCallbacks<DoctrineContextT>();
        if (working_state.episode_epoch !=
            current_frame_identity.episode_epoch)
        {
            const StaticBtLifecycleTransitionReceipt reset =
                ResetStaticBtLifecycleForEpoch(
                    evaluation,
                    working_state,
                    halt_bindings,
                    halt_callbacks,
                    current_frame_identity.episode_epoch,
                    current_frame_identity.frame_index,
                    ObfmG16G5bObservationNodeId,
                    ObfmG16G5bStageId);
            if (IsBtReturnCodeError(reset.result.code))
            {
                doctrine.AbortPreparedFinalState();
                output.lifecycle.selection.result = reset.result;
                output.lifecycle.transition = reset;
                return output;
            }
        }

        StaticBtChildTable<1U> children{};
        children.children[0] = lower_leaf.descriptor;
        children.child_count = 1U;
        StaticBtCallbackTable<EvaluationContext, 1U> callbacks{};
        callbacks.callbacks[0] =
            &EvaluateObfmLowerLeaf<DoctrineContextT>;
        callbacks.callback_count = 1U;
        output.lifecycle = EvaluateStaticBtLifecycleChildren(
            evaluation,
            working_state,
            children,
            callbacks,
            halt_bindings,
            halt_callbacks,
            nullptr,
            current_frame_identity,
            current_frame_identity.frame_index);
        if (IsBtReturnCodeError(output.lifecycle.selection.result.code)
            || !output.lifecycle.selection.candidate_available)
        {
            doctrine.AbortPreparedFinalState();
            return output;
        }

        doctrine.BeginDeferredFinalStateCommit(status);
        if (!status.ok())
        {
            doctrine.AbortPreparedFinalState();
            output.lifecycle.selection.candidate_available = false;
            output.lifecycle.selection.result =
                MakeObfmG16G5bStatusResult<DoctrineContextT>(
                    status.code,
                    lower_leaf.descriptor.node_id,
                    lower_leaf.descriptor.stage_id,
                    current_frame_identity.frame_index);
            return output;
        }

        pending_lifecycle_state_ = working_state;
        pending_candidate_ = output.lifecycle.selection.candidate;
        pending_selected_leaf_ = output.lifecycle.selection.result;
        pending_commit_ = true;
        output.pending_commit = true;
        return output;
    }

    void ClearPending() noexcept
    {
        pending_lifecycle_state_ = StaticBtLifecycleState{};
        pending_candidate_.Clear();
        pending_selected_leaf_ = BtTickResult{};
        pending_commit_ = false;
    }

    StaticBtLifecycleState lifecycle_state_{};
    StaticBtLifecycleState pending_lifecycle_state_{};
    ControlIntent pending_candidate_{};
    BtTickResult pending_selected_leaf_{};
    bool pending_commit_ = false;
};

using DoctrineObfmG16G5bAdapter =
    StaticDoctrineObfmG16G5bAdapter<
        doctrine::DoctrineBtRuntimeContext>;

static_assert(
    std::is_standard_layout<StaticDoctrineObfmG16G5bResult>::value,
    "OBFM G16/G5b result must remain standard-layout");
static_assert(
    std::is_trivially_copyable<StaticDoctrineObfmG16G5bResult>::value,
    "OBFM G16/G5b result must remain trivially copyable");
static_assert(sizeof(StaticDoctrineObfmG16G5bResult) == 624U,
              "OBFM G16/G5b result x64 size changed");
static_assert(alignof(StaticDoctrineObfmG16G5bResult) == 8U,
              "OBFM G16/G5b result x64 alignment changed");
static_assert(offsetof(StaticDoctrineObfmG16G5bResult, lifecycle) == 0U,
              "OBFM G16/G5b lifecycle offset changed");
static_assert(
    offsetof(StaticDoctrineObfmG16G5bResult, mode_redispatch) == 576U,
    "OBFM G16/G5b mode redispatch offset changed");
static_assert(
    offsetof(StaticDoctrineObfmG16G5bResult, pending_commit) == 577U,
    "OBFM G16/G5b pending flag offset changed");
static_assert(
    offsetof(StaticDoctrineObfmG16G5bResult, current_base_recovered) == 578U,
    "OBFM G16/G5b recovery flag offset changed");
static_assert(
    offsetof(StaticDoctrineObfmG16G5bResult, optional_fault_available) == 579U,
    "OBFM G16/G5b optional fault flag offset changed");
static_assert(
    offsetof(StaticDoctrineObfmG16G5bResult, optional_fault) == 584U,
    "OBFM G16/G5b optional fault offset changed");
static_assert(offsetof(StaticDoctrineObfmG16G5bResult, reserved) == 616U,
              "OBFM G16/G5b reserve offset changed");
static_assert(
    std::is_same<
        std::underlying_type<StaticDoctrineObfmModeRedispatch>::type,
        std::uint8_t>::value,
    "OBFM mode redispatch must retain fixed-width storage");
static_assert(std::is_standard_layout<DoctrineObfmG16G5bAdapter>::value,
              "OBFM G16/G5b adapter state must remain standard-layout");
static_assert(std::is_trivially_copyable<DoctrineObfmG16G5bAdapter>::value,
              "OBFM G16/G5b adapter state must remain trivially copyable");
static_assert(sizeof(DoctrineObfmG16G5bAdapter) == 520U,
              "OBFM G16/G5b adapter state x64 size changed");
static_assert(alignof(DoctrineObfmG16G5bAdapter) == 8U,
              "OBFM G16/G5b adapter state x64 alignment changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
