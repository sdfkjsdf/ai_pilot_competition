#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtAtomicEvaluator.hpp"

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

enum class StaticBtLifecycleTransitionKind : std::uint8_t
{
    None = 0U,
    Activated = 1U,
    Hold = 2U,
    Switched = 3U,
    StatelessSelected = 4U,
    Completed = 5U,
    Released = 6U,
    AllNotApplicable = 7U,
    EpochReset = 8U,
    FaultNoMutation = 9U
};

enum class StaticBtLifecycleReason : BtReasonId
{
    None = 0U,
    Activated = 1U,
    Hold = 2U,
    Switched = 3U,
    StatelessSelected = 4U,
    Completed = 5U,
    Released = 6U,
    AllNotApplicable = 7U,
    EpochReset = 8U,
    EpochUnchanged = 9U,
    EpochMismatch = 10U,
    HaltBindingCountExceedsCapacity = 11U,
    HaltCallbackCountExceedsCapacity = 12U,
    DuplicateHaltBinding = 13U,
    HaltBindingMissing = 14U,
    HaltCallbackIndexOutOfRange = 15U,
    HaltCallbackNull = 16U,
    HaltCallbackIdentityMismatch = 17U,
    LifecycleEventActiveMismatch = 18U
};

struct StaticBtLifecycleState
{
    std::uint64_t episode_epoch = 0U;
    BtLifecycleOwnerId active_owner_id{};
    BtNodeId active_node_id = BtNodeIdInvalid;
    BtStageId active_stage_id = BtStageIdInvalid;
    bool active = false;
    std::uint8_t reserved[3]{};
};

struct StaticBtHaltBinding
{
    BtLifecycleOwnerId owner_id{};
    std::uint32_t callback_index = 0U;
};

template <typename ContextT>
using StaticBtHaltCallback = BtTickResult (*)(
    ContextT& context,
    BtLifecycleOwnerId active_owner_id,
    BtNodeId active_node_id,
    BtStageId active_stage_id,
    std::uint64_t frame_index);

template <std::size_t Capacity>
struct StaticBtHaltBindingTable
{
    std::array<StaticBtHaltBinding, Capacity> bindings{};
    std::uint32_t binding_count = 0U;
};

template <typename ContextT, std::size_t Capacity>
struct StaticBtHaltCallbackTable
{
    std::array<StaticBtHaltCallback<ContextT>, Capacity> callbacks{};
    std::uint32_t callback_count = 0U;
};

struct StaticBtLifecycleTransitionReceipt
{
    BtTickResult result{};
    BtLifecycleOwnerId previous_owner_id{};
    BtNodeId previous_node_id = BtNodeIdInvalid;
    BtStageId previous_stage_id = BtStageIdInvalid;
    BtLifecycleOwnerId next_owner_id{};
    BtNodeId next_node_id = BtNodeIdInvalid;
    BtStageId next_stage_id = BtStageIdInvalid;
    StaticBtLifecycleTransitionKind transition =
        StaticBtLifecycleTransitionKind::None;
    bool halt_called = false;
    bool state_changed = false;
    std::uint8_t reserved[1]{};
};

struct StaticBtLifecycleSelection
{
    StaticBtCandidateSelection selection{};
    StaticBtLifecycleTransitionReceipt transition{};
};

constexpr BtReasonId ToReasonId(const StaticBtLifecycleReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr StaticBtLifecycleTransitionReceipt MakeLifecycleReceipt(
    const BtTickResult result,
    const BtLifecycleOwnerId previous_owner_id,
    const BtNodeId previous_node_id,
    const BtStageId previous_stage_id,
    const BtLifecycleOwnerId next_owner_id,
    const BtNodeId next_node_id,
    const BtStageId next_stage_id,
    const StaticBtLifecycleTransitionKind transition,
    const bool halt_called,
    const bool state_changed) noexcept
{
    return StaticBtLifecycleTransitionReceipt{result,
                                              previous_owner_id,
                                              previous_node_id,
                                              previous_stage_id,
                                              next_owner_id,
                                              next_node_id,
                                              next_stage_id,
                                              transition,
                                              halt_called,
                                              state_changed,
                                              {0U}};
}

constexpr bool SameActiveLifecycleOwner(
    const StaticBtLifecycleState& state,
    const BtLifecycleOwnerId owner_id) noexcept
{
    return state.active &&
           SameBtLifecycleOwnerId(state.active_owner_id, owner_id);
}

template <typename ContextT,
          std::size_t BindingCapacity,
          std::size_t CallbackCapacity>
BtTickResult HaltStaticBtActiveNode(
    ContextT& context,
    const StaticBtLifecycleState& state,
    const StaticBtHaltBindingTable<BindingCapacity>& binding_table,
    const StaticBtHaltCallbackTable<ContextT, CallbackCapacity>& callback_table,
    const std::uint64_t frame_index) noexcept
{
    if (binding_table.binding_count > BindingCapacity)
    {
        return MakeBtTickResult(
            BtReturnCode::CapacityExceeded,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(
                StaticBtLifecycleReason::HaltBindingCountExceedsCapacity));
    }
    if (callback_table.callback_count > CallbackCapacity)
    {
        return MakeBtTickResult(
            BtReturnCode::CapacityExceeded,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(
                StaticBtLifecycleReason::HaltCallbackCountExceedsCapacity));
    }

    std::uint32_t match_count = 0U;
    std::uint32_t matched_callback_index = 0U;
    for (std::uint32_t i = 0U; i < binding_table.binding_count; ++i)
    {
        const StaticBtHaltBinding& binding = binding_table.bindings[i];
        if (SameBtLifecycleOwnerId(binding.owner_id,
                                   state.active_owner_id))
        {
            ++match_count;
            matched_callback_index = binding.callback_index;
        }
    }
    if (match_count == 0U)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(StaticBtLifecycleReason::HaltBindingMissing));
    }
    if (match_count != 1U)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(StaticBtLifecycleReason::DuplicateHaltBinding));
    }
    if (matched_callback_index >= callback_table.callback_count)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(
                StaticBtLifecycleReason::HaltCallbackIndexOutOfRange));
    }

    const StaticBtHaltCallback<ContextT> callback =
        callback_table.callbacks[matched_callback_index];
    if (callback == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(StaticBtLifecycleReason::HaltCallbackNull));
    }

    const BtTickResult halt_result = callback(context,
                                              state.active_owner_id,
                                              state.active_node_id,
                                              state.active_stage_id,
                                              frame_index);
    if (IsBtReturnCodeError(halt_result.code))
    {
        return halt_result;
    }
    if (halt_result.code != BtReturnCode::Completed ||
        halt_result.node_id != state.active_node_id ||
        halt_result.stage_id != state.active_stage_id ||
        halt_result.frame_index != frame_index)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            state.active_node_id,
            state.active_stage_id,
            frame_index,
            ToReasonId(StaticBtLifecycleReason::HaltCallbackIdentityMismatch));
    }
    return halt_result;
}

template <typename ContextT,
          std::size_t BindingCapacity,
          std::size_t CallbackCapacity>
StaticBtLifecycleTransitionReceipt ResetStaticBtLifecycleForEpoch(
    ContextT& context,
    StaticBtLifecycleState& state,
    const StaticBtHaltBindingTable<BindingCapacity>& binding_table,
    const StaticBtHaltCallbackTable<ContextT, CallbackCapacity>& callback_table,
    const std::uint64_t new_episode_epoch,
    const std::uint64_t frame_index,
    const BtNodeId reset_node_id,
    const BtStageId reset_stage_id) noexcept
{
    const StaticBtLifecycleState previous = state;
    if (state.episode_epoch == new_episode_epoch)
    {
        return MakeLifecycleReceipt(
            MakeBtTickResult(
                BtReturnCode::Completed,
                reset_node_id,
                reset_stage_id,
                frame_index,
                ToReasonId(StaticBtLifecycleReason::EpochUnchanged)),
            previous.active_owner_id,
            previous.active_node_id,
            previous.active_stage_id,
            previous.active_owner_id,
            previous.active_node_id,
            previous.active_stage_id,
            StaticBtLifecycleTransitionKind::None,
            false,
            false);
    }

    bool halt_called = false;
    if (previous.active)
    {
        const BtTickResult halt = HaltStaticBtActiveNode(context,
                                                        previous,
                                                        binding_table,
                                                        callback_table,
                                                        frame_index);
        if (IsBtReturnCodeError(halt.code))
        {
            return MakeLifecycleReceipt(
                halt,
                previous.active_owner_id,
                previous.active_node_id,
                previous.active_stage_id,
                previous.active_owner_id,
                previous.active_node_id,
                previous.active_stage_id,
                StaticBtLifecycleTransitionKind::FaultNoMutation,
                false,
                false);
        }
        halt_called = true;
    }

    state.episode_epoch = new_episode_epoch;
    state.active_owner_id = BtLifecycleOwnerIdNone;
    state.active_node_id = BtNodeIdInvalid;
    state.active_stage_id = BtStageIdInvalid;
    state.active = false;
    const BtNodeId receipt_node =
        previous.active ? previous.active_node_id : reset_node_id;
    const BtStageId receipt_stage =
        previous.active ? previous.active_stage_id : reset_stage_id;
    return MakeLifecycleReceipt(
        MakeBtTickResult(BtReturnCode::Completed,
                         receipt_node,
                         receipt_stage,
                         frame_index,
                         ToReasonId(StaticBtLifecycleReason::EpochReset)),
        previous.active_owner_id,
        previous.active_node_id,
        previous.active_stage_id,
        BtLifecycleOwnerIdNone,
        BtNodeIdInvalid,
        BtStageIdInvalid,
        StaticBtLifecycleTransitionKind::EpochReset,
        halt_called,
        true);
}

template <typename ContextT,
          std::size_t ChildCapacity,
          std::size_t CandidateCallbackCapacity,
          std::size_t HaltBindingCapacity,
          std::size_t HaltCallbackCapacity>
StaticBtLifecycleSelection EvaluateStaticBtLifecycleChildren(
    ContextT& context,
    StaticBtLifecycleState& state,
    const StaticBtChildTable<ChildCapacity>& child_table,
    const StaticBtCallbackTable<ContextT, CandidateCallbackCapacity>& candidate_callbacks,
    const StaticBtHaltBindingTable<HaltBindingCapacity>& halt_bindings,
    const StaticBtHaltCallbackTable<ContextT, HaltCallbackCapacity>& halt_callbacks,
    const ControlIntent* established_base,
    const ControlFrameIdentity& current_frame_identity,
    const std::uint64_t frame_index) noexcept
{
    StaticBtLifecycleSelection output{};
    const StaticBtLifecycleState previous = state;
    output.selection = EvaluateStaticBtCandidateChildren(context,
                                                         child_table,
                                                         candidate_callbacks,
                                                         established_base,
                                                         current_frame_identity,
                                                         frame_index);

    if (IsBtReturnCodeError(output.selection.result.code))
    {
        output.transition = MakeLifecycleReceipt(
            output.selection.result,
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
    if (state.episode_epoch != current_frame_identity.episode_epoch)
    {
        output.selection.candidate_available = false;
        output.selection.lifecycle_event_available = false;
        output.selection.result = MakeBtTickResult(
            BtReturnCode::InvalidInput,
            previous.active_node_id,
            previous.active_stage_id,
            frame_index,
            ToReasonId(StaticBtLifecycleReason::EpochMismatch));
        output.transition = MakeLifecycleReceipt(
            output.selection.result,
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

    const bool has_lifecycle_event =
        output.selection.lifecycle_event_available;
    if (has_lifecycle_event &&
        (!previous.active ||
         !SameActiveLifecycleOwner(
             previous,
             output.selection.lifecycle_event_owner_id)))
    {
        output.selection.candidate_available = false;
        output.selection.result = MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            output.selection.lifecycle_event.node_id,
            output.selection.lifecycle_event.stage_id,
            frame_index,
            ToReasonId(
                StaticBtLifecycleReason::LifecycleEventActiveMismatch));
        output.transition = MakeLifecycleReceipt(
            output.selection.result,
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

    const BtReturnCode selected_code = output.selection.result.code;
    const bool all_not_applicable =
        !output.selection.candidate_available &&
        !has_lifecycle_event &&
        selected_code == BtReturnCode::NotApplicable;
    const bool selected_running =
        output.selection.candidate_available &&
        selected_code == BtReturnCode::Running;
    const bool selected_stateless =
        output.selection.candidate_available &&
        selected_code == BtReturnCode::Selected;
    const bool same_active = selected_running &&
        SameActiveLifecycleOwner(
            previous,
            output.selection.selected_lifecycle_owner_id);

    bool halt_required = false;
    if (previous.active && !has_lifecycle_event)
    {
        if (all_not_applicable || selected_stateless)
        {
            halt_required = true;
        }
        else if (selected_running && !same_active)
        {
            halt_required = true;
        }
    }

    bool halt_called = false;
    if (halt_required)
    {
        const BtTickResult halt = HaltStaticBtActiveNode(context,
                                                        previous,
                                                        halt_bindings,
                                                        halt_callbacks,
                                                        frame_index);
        if (IsBtReturnCodeError(halt.code))
        {
            output.selection.candidate_available = false;
            output.selection.result = halt;
            output.transition = MakeLifecycleReceipt(
                halt,
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
        halt_called = true;
    }

    StaticBtLifecycleState next = previous;
    StaticBtLifecycleTransitionKind transition_kind =
        StaticBtLifecycleTransitionKind::None;
    StaticBtLifecycleReason transition_reason =
        StaticBtLifecycleReason::None;
    BtLifecycleOwnerId next_owner_id = previous.active_owner_id;
    BtNodeId next_node_id = previous.active_node_id;
    BtStageId next_stage_id = previous.active_stage_id;

    if (has_lifecycle_event)
    {
        next.active_owner_id = BtLifecycleOwnerIdNone;
        next.active_node_id = BtNodeIdInvalid;
        next.active_stage_id = BtStageIdInvalid;
        next.active = false;
        next_owner_id = BtLifecycleOwnerIdNone;
        next_node_id = BtNodeIdInvalid;
        next_stage_id = BtStageIdInvalid;
        if (selected_running)
        {
            next.active_owner_id =
                output.selection.selected_lifecycle_owner_id;
            next.active_node_id = output.selection.result.node_id;
            next.active_stage_id = output.selection.result.stage_id;
            next.active = true;
            next_owner_id = next.active_owner_id;
            next_node_id = next.active_node_id;
            next_stage_id = next.active_stage_id;
        }
        if (output.selection.lifecycle_event.code ==
            BtReturnCode::Completed)
        {
            transition_kind = StaticBtLifecycleTransitionKind::Completed;
            transition_reason = StaticBtLifecycleReason::Completed;
        }
        else
        {
            transition_kind = StaticBtLifecycleTransitionKind::Released;
            transition_reason = StaticBtLifecycleReason::Released;
        }
    }
    else if (all_not_applicable)
    {
        next.active_owner_id = BtLifecycleOwnerIdNone;
        next.active_node_id = BtNodeIdInvalid;
        next.active_stage_id = BtStageIdInvalid;
        next.active = false;
        next_owner_id = BtLifecycleOwnerIdNone;
        next_node_id = BtNodeIdInvalid;
        next_stage_id = BtStageIdInvalid;
        transition_kind = StaticBtLifecycleTransitionKind::AllNotApplicable;
        transition_reason = StaticBtLifecycleReason::AllNotApplicable;
    }
    else if (selected_code == BtReturnCode::Running)
    {
        next.active_owner_id =
            output.selection.selected_lifecycle_owner_id;
        next.active_node_id = output.selection.result.node_id;
        next.active_stage_id = output.selection.result.stage_id;
        next.active = true;
        next_owner_id = next.active_owner_id;
        next_node_id = next.active_node_id;
        next_stage_id = next.active_stage_id;
        if (same_active)
        {
            transition_kind = StaticBtLifecycleTransitionKind::Hold;
            transition_reason = StaticBtLifecycleReason::Hold;
        }
        else if (previous.active)
        {
            transition_kind = StaticBtLifecycleTransitionKind::Switched;
            transition_reason = StaticBtLifecycleReason::Switched;
        }
        else
        {
            transition_kind = StaticBtLifecycleTransitionKind::Activated;
            transition_reason = StaticBtLifecycleReason::Activated;
        }
    }
    else if (selected_stateless)
    {
        next.active_owner_id = BtLifecycleOwnerIdNone;
        next.active_node_id = BtNodeIdInvalid;
        next.active_stage_id = BtStageIdInvalid;
        next.active = false;
        next_owner_id = BtLifecycleOwnerIdNone;
        next_node_id = BtNodeIdInvalid;
        next_stage_id = BtStageIdInvalid;
        transition_kind = StaticBtLifecycleTransitionKind::StatelessSelected;
        transition_reason = StaticBtLifecycleReason::StatelessSelected;
    }
    state = next;
    const bool state_changed =
        previous.episode_epoch != next.episode_epoch ||
        !SameBtLifecycleOwnerId(previous.active_owner_id,
                                next.active_owner_id) ||
        previous.active_node_id != next.active_node_id ||
        previous.active_stage_id != next.active_stage_id ||
        previous.active != next.active;
    const BtNodeId transition_node = has_lifecycle_event
        ? output.selection.lifecycle_event.node_id
        : all_not_applicable && previous.active
            ? previous.active_node_id
            : output.selection.result.node_id;
    const BtStageId transition_stage = has_lifecycle_event
        ? output.selection.lifecycle_event.stage_id
        : all_not_applicable && previous.active
            ? previous.active_stage_id
            : output.selection.result.stage_id;
    const BtReturnCode transition_code = has_lifecycle_event
        ? output.selection.lifecycle_event.code
        : selected_code;
    output.transition = MakeLifecycleReceipt(
        MakeBtTickResult(transition_code,
                         transition_node,
                         transition_stage,
                         frame_index,
                         ToReasonId(transition_reason)),
        previous.active_owner_id,
        previous.active_node_id,
        previous.active_stage_id,
        next_owner_id,
        next_node_id,
        next_stage_id,
        transition_kind,
        halt_called,
        state_changed);
    return output;
}

static_assert(std::is_standard_layout<StaticBtLifecycleState>::value,
              "Lifecycle state must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtLifecycleState>::value,
              "Lifecycle state must remain trivially copyable");
static_assert(sizeof(StaticBtLifecycleState) == 24U,
              "Lifecycle state x64 ABI size changed");
static_assert(alignof(StaticBtLifecycleState) == 8U,
              "Lifecycle state x64 ABI alignment changed");
static_assert(offsetof(StaticBtLifecycleState, episode_epoch) == 0U,
              "Lifecycle epoch offset changed");
static_assert(offsetof(StaticBtLifecycleState, active_owner_id) == 8U,
              "Lifecycle owner offset changed");
static_assert(offsetof(StaticBtLifecycleState, active_node_id) == 12U,
              "Lifecycle node offset changed");
static_assert(offsetof(StaticBtLifecycleState, active_stage_id) == 16U,
              "Lifecycle stage offset changed");
static_assert(offsetof(StaticBtLifecycleState, active) == 20U,
              "Lifecycle active offset changed");
static_assert(offsetof(StaticBtLifecycleState, reserved) == 21U,
              "Lifecycle reserve offset changed");
static_assert(std::is_standard_layout<StaticBtHaltBinding>::value,
              "Halt binding must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtHaltBinding>::value,
              "Halt binding must remain trivially copyable");
static_assert(sizeof(StaticBtHaltBinding) == 8U,
              "Halt binding x64 ABI size changed");
static_assert(alignof(StaticBtHaltBinding) == 4U,
              "Halt binding x64 ABI alignment changed");
static_assert(offsetof(StaticBtHaltBinding, owner_id) == 0U,
              "Halt binding owner offset changed");
static_assert(offsetof(StaticBtHaltBinding, callback_index) == 4U,
              "Halt binding callback offset changed");
static_assert(
    std::is_standard_layout<StaticBtLifecycleTransitionReceipt>::value,
    "Lifecycle receipt must remain standard-layout");
static_assert(
    std::is_trivially_copyable<StaticBtLifecycleTransitionReceipt>::value,
    "Lifecycle receipt must remain trivially copyable");
static_assert(sizeof(StaticBtLifecycleTransitionReceipt) == 64U,
              "Lifecycle receipt x64 ABI size changed");
static_assert(alignof(StaticBtLifecycleTransitionReceipt) == 8U,
              "Lifecycle receipt x64 ABI alignment changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, result) == 0U,
              "Lifecycle receipt result offset changed");
static_assert(
    offsetof(StaticBtLifecycleTransitionReceipt, previous_owner_id) == 32U,
    "Lifecycle receipt previous owner offset changed");
static_assert(
    offsetof(StaticBtLifecycleTransitionReceipt, previous_node_id) == 36U,
    "Lifecycle receipt previous node offset changed");
static_assert(
    offsetof(StaticBtLifecycleTransitionReceipt, previous_stage_id) == 40U,
    "Lifecycle receipt previous stage offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, next_owner_id) == 44U,
              "Lifecycle receipt next owner offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, next_node_id) == 48U,
              "Lifecycle receipt next node offset changed");
static_assert(
    offsetof(StaticBtLifecycleTransitionReceipt, next_stage_id) == 52U,
    "Lifecycle receipt next stage offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, transition) == 56U,
              "Lifecycle receipt transition offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, halt_called) == 57U,
              "Lifecycle receipt halt offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, state_changed) == 58U,
              "Lifecycle receipt state offset changed");
static_assert(offsetof(StaticBtLifecycleTransitionReceipt, reserved) == 59U,
              "Lifecycle receipt reserve offset changed");
static_assert(std::is_standard_layout<StaticBtLifecycleSelection>::value,
              "Lifecycle selection must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticBtLifecycleSelection>::value,
              "Lifecycle selection must remain trivially copyable");
static_assert(sizeof(StaticBtLifecycleSelection) == 576U,
              "Lifecycle selection x64 ABI size changed");
static_assert(alignof(StaticBtLifecycleSelection) == 8U,
              "Lifecycle selection x64 ABI alignment changed");
static_assert(offsetof(StaticBtLifecycleSelection, selection) == 0U,
              "Lifecycle selection candidate offset changed");
static_assert(offsetof(StaticBtLifecycleSelection, transition) == 512U,
              "Lifecycle selection receipt offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
