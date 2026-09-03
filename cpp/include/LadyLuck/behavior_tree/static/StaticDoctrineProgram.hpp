#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtAtomicEvaluator.hpp"
#include "LadyLuck/behavior_tree/static/StaticDoctrineDispatch.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

// The flat production root has exactly five terminal route owners.  This is
// the cardinality of StaticDoctrineRoute, not a spare runtime capacity.
constexpr std::size_t StaticDoctrineRouteCount = 5U;

enum class StaticDoctrineProgramReason : BtReasonId
{
    None = 0U,
    RouteTableTooSmall = 1U,
    RouteTableTooLarge = 2U,
    DispatchRouteInvalid = 3U
};

struct StaticDoctrineProgramResult
{
    StaticDoctrineDispatchResult dispatch{};
    StaticBtAtomicResult transaction{};
    std::uint32_t route_index = 0U;
    bool route_index_valid = false;
    std::uint8_t reserved[3]{};
};

constexpr BtReasonId ToReasonId(
    const StaticDoctrineProgramReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr bool StaticDoctrineRouteIndex(
    const StaticDoctrineRoute route,
    std::uint32_t& output) noexcept
{
    switch (route)
    {
    case StaticDoctrineRoute::AutoGcas:
        output = 0U;
        return true;
    case StaticDoctrineRoute::ImmediateGunDefense:
        output = 1U;
        return true;
    case StaticDoctrineRoute::Obfm:
        output = 2U;
        return true;
    case StaticDoctrineRoute::Habfm:
        output = 3U;
        return true;
    case StaticDoctrineRoute::Dbfm:
        output = 4U;
        return true;
    default:
        output = 0U;
        return false;
    }
}

// One static root transaction:
//   observe/classify -> choose exactly one senior route -> evaluate that
//   route's complete candidate -> validate -> commit exactly once.
//
// Each route callback owns its internal maneuver selector.  This adapter adds
// no tactical gate, gain, threshold, guidance reference or FCS command.  The
// route descriptor table is supplied by the generated topology so node/stage
// identities are not invented here.
template <typename ContextT>
StaticDoctrineProgramResult EvaluateStaticDoctrineProgram(
    ContextT& context,
    const StaticBtChildTable<StaticDoctrineRouteCount>& route_children,
    const StaticBtCallbackTable<ContextT, StaticDoctrineRouteCount>&
        route_callbacks,
    const ControlFrameIdentity& current_frame_identity,
    const std::uint64_t frame_index,
    const StaticBtCommitCallback<ContextT> commit_callback) noexcept
{
    StaticDoctrineProgramResult output{};
    output.dispatch = DispatchStaticDoctrine(context, frame_index);
    if (!output.dispatch.route_valid ||
        IsBtReturnCodeError(output.dispatch.result.code))
    {
        output.transaction.outcome = output.dispatch.result;
        return output;
    }

    if (route_children.child_count < StaticDoctrineRouteCount)
    {
        output.transaction.outcome = MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticDoctrineProgramReason::RouteTableTooSmall));
        return output;
    }
    if (route_children.child_count > StaticDoctrineRouteCount)
    {
        output.transaction.outcome = MakeBtTickResult(
            BtReturnCode::CapacityExceeded,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticDoctrineProgramReason::RouteTableTooLarge));
        return output;
    }

    std::uint32_t route_index = 0U;
    if (!StaticDoctrineRouteIndex(output.dispatch.route, route_index))
    {
        output.transaction.outcome = MakeBtTickResult(
            BtReturnCode::InvalidTopology,
            BtNodeIdInvalid,
            BtStageIdInvalid,
            frame_index,
            ToReasonId(StaticDoctrineProgramReason::DispatchRouteInvalid));
        return output;
    }
    output.route_index = route_index;
    output.route_index_valid = true;

    StaticBtChildTable<1U> selected_route{};
    selected_route.children[0] = route_children.children[route_index];
    selected_route.child_count = 1U;
    StaticBtChildTable<1U> no_response{};
    StaticBtCallbackTable<ContextT, 1U> no_response_callbacks{};
    output.transaction = EvaluateAndCommitStaticBtCandidates(
        context,
        selected_route,
        route_callbacks,
        no_response,
        no_response_callbacks,
        current_frame_identity,
        frame_index,
        commit_callback);
    return output;
}

static_assert(StaticDoctrineRouteCount == 5U,
              "Static doctrine route cardinality changed");
static_assert(std::is_standard_layout<StaticDoctrineProgramResult>::value,
              "Static doctrine program result must remain standard-layout");
static_assert(
    std::is_trivially_copyable<StaticDoctrineProgramResult>::value,
    "Static doctrine program result must remain trivially copyable");
static_assert(sizeof(StaticDoctrineProgramResult) == 624U,
              "Static doctrine program result x64 ABI size changed");
static_assert(alignof(StaticDoctrineProgramResult) == 8U,
              "Static doctrine program result x64 ABI alignment changed");
static_assert(offsetof(StaticDoctrineProgramResult, dispatch) == 0U,
              "Static doctrine dispatch offset changed");
static_assert(offsetof(StaticDoctrineProgramResult, transaction) == 40U,
              "Static doctrine transaction offset changed");
static_assert(offsetof(StaticDoctrineProgramResult, route_index) == 616U,
              "Static doctrine route index offset changed");
static_assert(offsetof(StaticDoctrineProgramResult, route_index_valid) == 620U,
              "Static doctrine route validity offset changed");
static_assert(offsetof(StaticDoctrineProgramResult, reserved) == 621U,
              "Static doctrine reserve offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
