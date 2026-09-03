#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtResult.hpp"
#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class StaticBtAtomicReason : BtReasonId
{
    None = 0U,
    EmptyBaseTable = 1U,
    ChildCountExceedsCapacity = 2U,
    CallbackCountExceedsCapacity = 3U,
    CallbackIndexOutOfRange = 4U,
    NullCallback = 5U,
    CallbackIdentityMismatch = 6U,
    CurrentFrameInvalid = 7U,
    CandidateFrameMismatch = 8U,
    CandidateNonFinite = 9U,
    CandidateContractInvalid = 10U,
    CandidateWriterMissing = 11U,
    CommitCallbackMissing = 12U,
    CommitIdentityMismatch = 13U,
    CommitAlreadyCompleted = 14U,
    MultipleLifecycleEvents = 15U,
    RunningLifecycleOwnerMissing = 16U,
    TerminalLifecycleOwnerMissing = 17U,
    StatelessLifecycleOwnerPresent = 18U
};

struct StaticBtChildDescriptor
{
    BtNodeId node_id = BtNodeIdInvalid;
    BtStageId stage_id = BtStageIdInvalid;
    std::uint32_t callback_index = 0U;
    BtLifecycleOwnerId lifecycle_owner_id{};
};

template <typename ContextT>
using StaticBtCandidateCallback = BtTickResult (*)(
    ContextT& context,
    const ControlIntent* established_base,
    ControlIntent& proposed_candidate);

template <typename ContextT>
using StaticBtCommitCallback = BtTickResult (*)(
    ContextT& context,
    const ControlIntent& final_candidate,
    const BtTickResult& selected_leaf);

template <typename ContextT, std::size_t Capacity>
struct StaticBtCallbackTable
{
    std::array<StaticBtCandidateCallback<ContextT>, Capacity> callbacks{};
    std::uint32_t callback_count = 0U;
};

template <std::size_t Capacity>
struct StaticBtChildTable
{
    std::array<StaticBtChildDescriptor, Capacity> children{};
    std::uint32_t child_count = 0U;
};

struct StaticBtCandidateSelection
{
    ControlIntent candidate{};
    BtTickResult result{};
    BtTickResult lifecycle_event{};
    BtLifecycleOwnerId selected_lifecycle_owner_id{};
    BtLifecycleOwnerId lifecycle_event_owner_id{};
    bool candidate_available = false;
    bool lifecycle_event_available = false;
    std::uint8_t reserved[6]{};
};

struct StaticBtAtomicResult
{
    ControlIntent final_candidate{};
    BtTickResult outcome{};
    BtTickResult response_fault{};
    BtTickResult selected_leaf{};
    BtTickResult lifecycle_event{};
    BtLifecycleOwnerId selected_lifecycle_owner_id{};
    BtLifecycleOwnerId lifecycle_event_owner_id{};
    bool candidate_available = false;
    bool response_fault_available = false;
    bool lifecycle_event_available = false;
    bool committed = false;
    std::uint8_t reserved[4]{};
};

constexpr BtReasonId ToReasonId(const StaticBtAtomicReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr bool IsNormalStopCode(const BtReturnCode code) noexcept
{
    return code == BtReturnCode::Selected ||
           code == BtReturnCode::Running ||
           code == BtReturnCode::Completed ||
           code == BtReturnCode::Released;
}

constexpr bool IsCandidateProducingCode(const BtReturnCode code) noexcept
{
    return code == BtReturnCode::Selected ||
           code == BtReturnCode::Running;
}

constexpr bool HasExactChildIdentity(
    const BtTickResult& result,
    const StaticBtChildDescriptor& child,
    const std::uint64_t frame_index) noexcept
{
    return result.node_id == child.node_id &&
           result.stage_id == child.stage_id &&
           result.frame_index == frame_index;
}

BtTickResult ValidateStaticBtControlIntentCandidate(
    const ControlIntent& candidate,
    const ControlFrameIdentity& current_frame_identity,
    BtNodeId node_id,
    BtStageId stage_id,
    std::uint64_t frame_index) noexcept;

template <typename ContextT,
          std::size_t ChildCapacity,
          std::size_t CallbackCapacity>
StaticBtCandidateSelection EvaluateStaticBtCandidateChildren(
    ContextT& context,
    const StaticBtChildTable<ChildCapacity>& child_table,
    const StaticBtCallbackTable<ContextT, CallbackCapacity>& callback_table,
    const ControlIntent* established_base,
    const ControlFrameIdentity& current_frame_identity,
    const std::uint64_t frame_index) noexcept
{
    StaticBtCandidateSelection output{};
    if (child_table.child_count > ChildCapacity)
    {
        output.result = MakeBtTickResult(
            BtReturnCode::CapacityExceeded,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticBtAtomicReason::ChildCountExceedsCapacity));
        return output;
    }
    if (callback_table.callback_count > CallbackCapacity)
    {
        output.result = MakeBtTickResult(
            BtReturnCode::CapacityExceeded,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CallbackCountExceedsCapacity));
        return output;
    }
    if (child_table.child_count == 0U)
    {
        output.result = MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticBtAtomicReason::EmptyBaseTable));
        return output;
    }

    for (std::uint32_t i = 0U; i < child_table.child_count; ++i)
    {
        const StaticBtChildDescriptor& child = child_table.children[i];
        if (child.callback_index >= callback_table.callback_count)
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InvalidTopology,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(StaticBtAtomicReason::CallbackIndexOutOfRange));
            return output;
        }

        const StaticBtCandidateCallback<ContextT> callback =
            callback_table.callbacks[child.callback_index];
        if (callback == nullptr)
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InvalidTopology,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(StaticBtAtomicReason::NullCallback));
            return output;
        }

        ControlIntent proposed{};
        const BtTickResult child_result =
            callback(context, established_base, proposed);

        // Typed faults are returned byte-for-byte.  They are not converted to
        // NotApplicable and cannot advance the selector.
        if (IsBtReturnCodeError(child_result.code))
        {
            output.result = child_result;
            return output;
        }
        if (!HasExactChildIdentity(child_result, child, frame_index))
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InternalContractFault,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(StaticBtAtomicReason::CallbackIdentityMismatch));
            return output;
        }

        if (child_result.code == BtReturnCode::NotApplicable)
        {
            output.result = child_result;
            continue;
        }
        if (IsBtReturnCodeTerminal(child_result.code))
        {
            // Completed/Released are command-neutral lifecycle observations.
            // They must not validate or publish the callback's unused
            // ControlIntent and must not prevent a lower-priority child from
            // producing the current-frame command.
            if (!HasBtLifecycleOwnerId(child.lifecycle_owner_id))
            {
                output.result = MakeBtTickResult(
                    BtReturnCode::InternalContractFault,
                    child.node_id,
                    child.stage_id,
                    frame_index,
                    ToReasonId(
                        StaticBtAtomicReason::TerminalLifecycleOwnerMissing));
                return output;
            }
            if (output.lifecycle_event_available)
            {
                output.result = MakeBtTickResult(
                    BtReturnCode::InternalContractFault,
                    child.node_id,
                    child.stage_id,
                    frame_index,
                    ToReasonId(
                        StaticBtAtomicReason::MultipleLifecycleEvents));
                return output;
            }
            output.lifecycle_event = child_result;
            output.lifecycle_event_owner_id = child.lifecycle_owner_id;
            output.lifecycle_event_available = true;
            output.result = child_result;
            continue;
        }
        if (!IsCandidateProducingCode(child_result.code))
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InternalContractFault,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(StaticBtAtomicReason::CallbackIdentityMismatch));
            return output;
        }
        if (child_result.code == BtReturnCode::Running &&
            !HasBtLifecycleOwnerId(child.lifecycle_owner_id))
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InternalContractFault,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(
                    StaticBtAtomicReason::RunningLifecycleOwnerMissing));
            return output;
        }
        if (child_result.code == BtReturnCode::Selected &&
            HasBtLifecycleOwnerId(child.lifecycle_owner_id))
        {
            output.result = MakeBtTickResult(
                BtReturnCode::InternalContractFault,
                child.node_id,
                child.stage_id,
                frame_index,
                ToReasonId(
                    StaticBtAtomicReason::StatelessLifecycleOwnerPresent));
            return output;
        }

        const BtTickResult validation =
            ValidateStaticBtControlIntentCandidate(proposed,
                                                   current_frame_identity,
                                                   child.node_id,
                                                   child.stage_id,
                                                   frame_index);
        if (IsBtReturnCodeError(validation.code))
        {
            output.result = validation;
            return output;
        }

        output.candidate = proposed;
        output.result = child_result;
        output.selected_lifecycle_owner_id = child.lifecycle_owner_id;
        output.candidate_available = true;
        return output;
    }
    return output;
}

template <typename ContextT,
          std::size_t BaseChildCapacity,
          std::size_t BaseCallbackCapacity,
          std::size_t ResponseChildCapacity,
          std::size_t ResponseCallbackCapacity>
StaticBtAtomicResult EvaluateStaticBtCandidates(
    ContextT& context,
    const StaticBtChildTable<BaseChildCapacity>& base_children,
    const StaticBtCallbackTable<ContextT, BaseCallbackCapacity>& base_callbacks,
    const StaticBtChildTable<ResponseChildCapacity>& response_children,
    const StaticBtCallbackTable<ContextT, ResponseCallbackCapacity>& response_callbacks,
    const ControlFrameIdentity& current_frame_identity,
    const std::uint64_t frame_index) noexcept
{
    StaticBtAtomicResult output{};
    const StaticBtCandidateSelection base =
        EvaluateStaticBtCandidateChildren(context,
                                          base_children,
                                          base_callbacks,
                                          nullptr,
                                          current_frame_identity,
                                          frame_index);
    output.outcome = base.result;
    output.lifecycle_event = base.lifecycle_event;
    output.lifecycle_event_owner_id = base.lifecycle_event_owner_id;
    output.lifecycle_event_available = base.lifecycle_event_available;
    if (!base.candidate_available)
    {
        return output;
    }

    output.final_candidate = base.candidate;
    output.candidate_available = true;
    output.selected_leaf = base.result;
    output.selected_lifecycle_owner_id =
        base.selected_lifecycle_owner_id;

    if (response_children.child_count != 0U)
    {
        const StaticBtCandidateSelection response =
            EvaluateStaticBtCandidateChildren(context,
                                              response_children,
                                              response_callbacks,
                                              &base.candidate,
                                              current_frame_identity,
                                              frame_index);
        if (response.lifecycle_event_available)
        {
            if (output.lifecycle_event_available)
            {
                output.response_fault = MakeBtTickResult(
                    BtReturnCode::InternalContractFault,
                    response.lifecycle_event.node_id,
                    response.lifecycle_event.stage_id,
                    frame_index,
                    ToReasonId(
                        StaticBtAtomicReason::MultipleLifecycleEvents));
                output.response_fault_available = true;
                output.outcome = output.response_fault;
                return output;
            }
            output.lifecycle_event = response.lifecycle_event;
            output.lifecycle_event_owner_id =
                response.lifecycle_event_owner_id;
            output.lifecycle_event_available = true;
        }
        if (IsBtReturnCodeError(response.result.code))
        {
            // The already validated base remains the sole publish candidate.
            // The exact response fault is retained independently and remains
            // the externally visible outcome after a successful base commit.
            output.response_fault = response.result;
            output.response_fault_available = true;
            output.outcome = response.result;
        }
        else if (response.candidate_available)
        {
            output.final_candidate = response.candidate;
            output.outcome = response.result;
            output.selected_leaf = response.result;
            output.selected_lifecycle_owner_id =
                response.selected_lifecycle_owner_id;
        }
        // All-response NotApplicable intentionally retains the established
        // base candidate and its selection receipt.
    }

    return output;
}

template <typename ContextT>
void CommitStaticBtCandidate(
    ContextT& context,
    StaticBtAtomicResult& output,
    const std::uint64_t frame_index,
    const StaticBtCommitCallback<ContextT> commit_callback) noexcept
{
    if (!output.candidate_available)
    {
        return;
    }
    if (output.committed)
    {
        output.outcome = MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            output.selected_leaf.node_id,
            output.selected_leaf.stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CommitAlreadyCompleted));
        return;
    }

    if (commit_callback == nullptr)
    {
        output.outcome = MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            output.selected_leaf.node_id,
            output.selected_leaf.stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CommitCallbackMissing));
        return;
    }

    const BtTickResult commit_result =
        commit_callback(context, output.final_candidate, output.selected_leaf);
    if (IsBtReturnCodeError(commit_result.code))
    {
        output.outcome = commit_result;
        return;
    }
    if (!HasExactChildIdentity(
            commit_result,
            StaticBtChildDescriptor{
                output.selected_leaf.node_id,
                output.selected_leaf.stage_id,
                0U},
            frame_index))
    {
        output.outcome = MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            output.selected_leaf.node_id,
            output.selected_leaf.stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CommitIdentityMismatch));
        return;
    }
    if (!IsNormalStopCode(commit_result.code))
    {
        output.outcome = MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            output.selected_leaf.node_id,
            output.selected_leaf.stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CommitIdentityMismatch));
        return;
    }

    output.committed = true;
}

template <typename ContextT,
          std::size_t BaseChildCapacity,
          std::size_t BaseCallbackCapacity,
          std::size_t ResponseChildCapacity,
          std::size_t ResponseCallbackCapacity>
StaticBtAtomicResult EvaluateAndCommitStaticBtCandidates(
    ContextT& context,
    const StaticBtChildTable<BaseChildCapacity>& base_children,
    const StaticBtCallbackTable<ContextT, BaseCallbackCapacity>& base_callbacks,
    const StaticBtChildTable<ResponseChildCapacity>& response_children,
    const StaticBtCallbackTable<ContextT, ResponseCallbackCapacity>& response_callbacks,
    const ControlFrameIdentity& current_frame_identity,
    const std::uint64_t frame_index,
    const StaticBtCommitCallback<ContextT> commit_callback) noexcept
{
    StaticBtAtomicResult output = EvaluateStaticBtCandidates(
        context,
        base_children,
        base_callbacks,
        response_children,
        response_callbacks,
        current_frame_identity,
        frame_index);
    CommitStaticBtCandidate(
        context,
        output,
        frame_index,
        commit_callback);
    return output;
}

static_assert(sizeof(bool) == 1U, "Static BT ABI requires one-byte bool");
static_assert(sizeof(ControlIntent) == 432U,
              "ControlIntent x64 ABI size changed");
static_assert(alignof(ControlIntent) == 8U,
              "ControlIntent x64 ABI alignment changed");
static_assert(std::is_standard_layout<StaticBtChildDescriptor>::value,
              "Child descriptor must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtChildDescriptor>::value,
              "Child descriptor must remain trivially copyable");
static_assert(sizeof(StaticBtChildDescriptor) == 16U,
              "Child descriptor x64 ABI size changed");
static_assert(alignof(StaticBtChildDescriptor) == 4U,
              "Child descriptor x64 ABI alignment changed");
static_assert(offsetof(StaticBtChildDescriptor, node_id) == 0U,
              "Child descriptor node_id offset changed");
static_assert(offsetof(StaticBtChildDescriptor, stage_id) == 4U,
              "Child descriptor stage_id offset changed");
static_assert(offsetof(StaticBtChildDescriptor, callback_index) == 8U,
              "Child descriptor callback_index offset changed");
static_assert(offsetof(StaticBtChildDescriptor, lifecycle_owner_id) == 12U,
              "Child descriptor lifecycle owner offset changed");
static_assert(std::is_standard_layout<StaticBtCandidateSelection>::value,
              "Candidate selection must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtCandidateSelection>::value,
              "Candidate selection must remain trivially copyable");
static_assert(sizeof(StaticBtCandidateSelection) == 512U,
              "Candidate selection x64 ABI size changed");
static_assert(alignof(StaticBtCandidateSelection) == 8U,
              "Candidate selection x64 ABI alignment changed");
static_assert(offsetof(StaticBtCandidateSelection, candidate) == 0U,
              "Candidate selection candidate offset changed");
static_assert(offsetof(StaticBtCandidateSelection, result) == 432U,
              "Candidate selection result offset changed");
static_assert(offsetof(StaticBtCandidateSelection, lifecycle_event) == 464U,
              "Candidate selection lifecycle event offset changed");
static_assert(offsetof(StaticBtCandidateSelection,
                       selected_lifecycle_owner_id) == 496U,
              "Candidate selection owner offset changed");
static_assert(offsetof(StaticBtCandidateSelection,
                       lifecycle_event_owner_id) == 500U,
              "Candidate selection event owner offset changed");
static_assert(offsetof(StaticBtCandidateSelection, candidate_available) == 504U,
              "Candidate selection availability offset changed");
static_assert(offsetof(StaticBtCandidateSelection,
                       lifecycle_event_available) == 505U,
              "Candidate selection lifecycle availability offset changed");
static_assert(offsetof(StaticBtCandidateSelection, reserved) == 506U,
              "Candidate selection reserve offset changed");
static_assert(std::is_standard_layout<StaticBtAtomicResult>::value,
              "Atomic result must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtAtomicResult>::value,
              "Atomic result must remain trivially copyable");
static_assert(sizeof(StaticBtAtomicResult) == 576U,
              "Atomic result x64 ABI size changed");
static_assert(alignof(StaticBtAtomicResult) == 8U,
              "Atomic result x64 ABI alignment changed");
static_assert(offsetof(StaticBtAtomicResult, final_candidate) == 0U,
              "Atomic result candidate offset changed");
static_assert(offsetof(StaticBtAtomicResult, outcome) == 432U,
              "Atomic result outcome offset changed");
static_assert(offsetof(StaticBtAtomicResult, response_fault) == 464U,
              "Atomic result response fault offset changed");
static_assert(offsetof(StaticBtAtomicResult, selected_leaf) == 496U,
              "Atomic result selected leaf offset changed");
static_assert(offsetof(StaticBtAtomicResult, lifecycle_event) == 528U,
              "Atomic result lifecycle event offset changed");
static_assert(offsetof(StaticBtAtomicResult,
                       selected_lifecycle_owner_id) == 560U,
              "Atomic result selected owner offset changed");
static_assert(offsetof(StaticBtAtomicResult,
                       lifecycle_event_owner_id) == 564U,
              "Atomic result event owner offset changed");
static_assert(offsetof(StaticBtAtomicResult, candidate_available) == 568U,
              "Atomic result candidate availability offset changed");
static_assert(offsetof(StaticBtAtomicResult, response_fault_available) == 569U,
              "Atomic result fault availability offset changed");
static_assert(offsetof(StaticBtAtomicResult,
                       lifecycle_event_available) == 570U,
              "Atomic result lifecycle availability offset changed");
static_assert(offsetof(StaticBtAtomicResult, committed) == 571U,
              "Atomic result committed offset changed");
static_assert(offsetof(StaticBtAtomicResult, reserved) == 572U,
              "Atomic result reserve offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
