#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtResult.hpp"
#include "LadyLuck/contracts/Status.hpp"

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

// There is deliberately no None or Unresolved route.  A successful dispatch
// always names exactly one production branch.
enum class StaticDoctrineRoute : std::uint8_t
{
    AutoGcas = 1U,
    ImmediateGunDefense = 2U,
    Obfm = 3U,
    Habfm = 4U,
    Dbfm = 5U
};

enum class StaticDoctrineDispatchNode : BtNodeId
{
    Adapter = 1U,
    RootObservation = 2U,
    RootSafety = 3U,
    ImmediateDefense = 4U,
    TacticalMode = 5U
};

enum class StaticDoctrineDispatchStage : BtStageId
{
    Observation = 1U,
    Safety = 2U,
    ImmediateDefense = 3U,
    TacticalMode = 4U,
    PriorityResolution = 5U
};

enum class StaticDoctrineDispatchReason : BtReasonId
{
    None = 0U,
    AutoGcasSelected = 1U,
    ImmediateGunDefenseSelected = 2U,
    ObfmChildZeroSelected = 3U,
    HabfmChildOneSelected = 4U,
    DbfmChildTwoSelected = 5U,
    TacticalModeChildOutOfRange = 6U,

    StatusSeeded = 20U,
    StatusObservationInvalid = 21U,
    StatusFrameGap = 22U,
    StatusInvalidArgument = 23U,
    StatusNonFiniteInput = 24U,
    StatusInvalidDt = 25U,
    StatusAmbiguousRotation = 26U,
    StatusInvalidConfiguration = 27U,
    StatusUnknown = 28U
};

struct StaticDoctrineDispatchResult
{
    BtTickResult result{};
    StaticDoctrineRoute route = StaticDoctrineRoute::Habfm;
    std::uint8_t tactical_mode_child_index = 0xFFU;
    bool route_valid = false;
    std::uint8_t reserved[5]{};
};

constexpr BtNodeId ToNodeId(const StaticDoctrineDispatchNode node) noexcept
{
    return static_cast<BtNodeId>(node);
}

constexpr BtStageId ToStageId(const StaticDoctrineDispatchStage stage) noexcept
{
    return static_cast<BtStageId>(stage);
}

constexpr BtReasonId ToReasonId(
    const StaticDoctrineDispatchReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr StaticDoctrineDispatchResult MakeStaticDoctrineFailure(
    const BtReturnCode code,
    const StaticDoctrineDispatchNode node,
    const StaticDoctrineDispatchStage stage,
    const std::uint64_t frame_index,
    const StaticDoctrineDispatchReason reason) noexcept
{
    return StaticDoctrineDispatchResult{
        MakeBtTickResult(code,
                         ToNodeId(node),
                         ToStageId(stage),
                         frame_index,
                         ToReasonId(reason)),
        StaticDoctrineRoute::Habfm,
        0xFFU,
        false,
        {0U, 0U, 0U, 0U, 0U}};
}

constexpr StaticDoctrineDispatchReason MapDoctrineStatusReason(
    const StatusCode code) noexcept
{
    switch (code)
    {
    case StatusCode::Ok:
        return StaticDoctrineDispatchReason::None;
    case StatusCode::Seeded:
        return StaticDoctrineDispatchReason::StatusSeeded;
    case StatusCode::ObservationInvalid:
        return StaticDoctrineDispatchReason::StatusObservationInvalid;
    case StatusCode::FrameGap:
        return StaticDoctrineDispatchReason::StatusFrameGap;
    case StatusCode::InvalidArgument:
        return StaticDoctrineDispatchReason::StatusInvalidArgument;
    case StatusCode::NonFiniteInput:
        return StaticDoctrineDispatchReason::StatusNonFiniteInput;
    case StatusCode::InvalidDt:
        return StaticDoctrineDispatchReason::StatusInvalidDt;
    case StatusCode::AmbiguousRotation:
        return StaticDoctrineDispatchReason::StatusAmbiguousRotation;
    case StatusCode::InvalidConfiguration:
        return StaticDoctrineDispatchReason::StatusInvalidConfiguration;
    default:
        return StaticDoctrineDispatchReason::StatusUnknown;
    }
}

constexpr BtReturnCode MapDoctrineStatusReturnCode(
    const StatusCode code) noexcept
{
    return code == StatusCode::InvalidConfiguration
               ? BtReturnCode::InternalContractFault
           : code == StatusCode::Ok
               ? BtReturnCode::Completed
               : BtReturnCode::InvalidInput;
}

constexpr BtTickResult MapDoctrineStatus(
    const StatusCode code,
    const StaticDoctrineDispatchNode node,
    const StaticDoctrineDispatchStage stage,
    const std::uint64_t frame_index) noexcept
{
    return MakeBtTickResult(MapDoctrineStatusReturnCode(code),
                            ToNodeId(node),
                            ToStageId(stage),
                            frame_index,
                            ToReasonId(MapDoctrineStatusReason(code)));
}

// Pure priority resolver used by the adapter and by the exhaustive 2x2x3
// focused cells.  Safety and gun booleans are already-observed facts; this
// function introduces no tactical admission rule.
constexpr StaticDoctrineDispatchResult ResolveStaticDoctrinePriority(
    const bool safety_selected,
    const bool immediate_defense_selected,
    const std::size_t tactical_mode_child_index,
    const std::uint64_t frame_index) noexcept
{
    if (safety_selected)
    {
        return StaticDoctrineDispatchResult{
            MakeBtTickResult(
                BtReturnCode::Selected,
                ToNodeId(StaticDoctrineDispatchNode::RootSafety),
                ToStageId(StaticDoctrineDispatchStage::Safety),
                frame_index,
                ToReasonId(StaticDoctrineDispatchReason::AutoGcasSelected)),
            StaticDoctrineRoute::AutoGcas,
            0xFFU,
            true,
            {0U, 0U, 0U, 0U, 0U}};
    }
    if (immediate_defense_selected)
    {
        return StaticDoctrineDispatchResult{
            MakeBtTickResult(
                BtReturnCode::Selected,
                ToNodeId(StaticDoctrineDispatchNode::ImmediateDefense),
                ToStageId(StaticDoctrineDispatchStage::ImmediateDefense),
                frame_index,
                ToReasonId(
                    StaticDoctrineDispatchReason::ImmediateGunDefenseSelected)),
            StaticDoctrineRoute::ImmediateGunDefense,
            0xFFU,
            true,
            {0U, 0U, 0U, 0U, 0U}};
    }

    switch (tactical_mode_child_index)
    {
    case 0U:
        return StaticDoctrineDispatchResult{
            MakeBtTickResult(
                BtReturnCode::Selected,
                ToNodeId(StaticDoctrineDispatchNode::TacticalMode),
                ToStageId(StaticDoctrineDispatchStage::TacticalMode),
                frame_index,
                ToReasonId(
                    StaticDoctrineDispatchReason::ObfmChildZeroSelected)),
            StaticDoctrineRoute::Obfm,
            0U,
            true,
            {0U, 0U, 0U, 0U, 0U}};
    case 1U:
        return StaticDoctrineDispatchResult{
            MakeBtTickResult(
                BtReturnCode::Selected,
                ToNodeId(StaticDoctrineDispatchNode::TacticalMode),
                ToStageId(StaticDoctrineDispatchStage::TacticalMode),
                frame_index,
                ToReasonId(
                    StaticDoctrineDispatchReason::HabfmChildOneSelected)),
            StaticDoctrineRoute::Habfm,
            1U,
            true,
            {0U, 0U, 0U, 0U, 0U}};
    case 2U:
        return StaticDoctrineDispatchResult{
            MakeBtTickResult(
                BtReturnCode::Selected,
                ToNodeId(StaticDoctrineDispatchNode::TacticalMode),
                ToStageId(StaticDoctrineDispatchStage::TacticalMode),
                frame_index,
                ToReasonId(
                    StaticDoctrineDispatchReason::DbfmChildTwoSelected)),
            StaticDoctrineRoute::Dbfm,
            2U,
            true,
            {0U, 0U, 0U, 0U, 0U}};
    default:
        return MakeStaticDoctrineFailure(
            BtReturnCode::InvalidTopology,
            StaticDoctrineDispatchNode::TacticalMode,
            StaticDoctrineDispatchStage::TacticalMode,
            frame_index,
            StaticDoctrineDispatchReason::TacticalModeChildOutOfRange);
    }
}

// Structural adapter for DoctrineBtRuntimeContext's existing command-neutral
// observation/selection methods.  Keeping this template here prevents the
// static seam from importing the legacy dynamic TacticalCommand header graph.
template <typename DoctrineContextT>
StaticDoctrineDispatchResult DispatchStaticDoctrine(
    DoctrineContextT& context,
    const std::uint64_t frame_index) noexcept
{
    Status status{};
    context.ObserveRootTacticalState(status);
    if (status.code != StatusCode::Ok)
    {
        const BtTickResult fault = MapDoctrineStatus(
            status.code,
            StaticDoctrineDispatchNode::RootObservation,
            StaticDoctrineDispatchStage::Observation,
            frame_index);
        return MakeStaticDoctrineFailure(
            fault.code,
            StaticDoctrineDispatchNode::RootObservation,
            StaticDoctrineDispatchStage::Observation,
            frame_index,
            MapDoctrineStatusReason(status.code));
    }

    bool safety_selected = false;
    context.CheckRootSafetyRequired(safety_selected, status);
    if (status.code != StatusCode::Ok)
    {
        const BtTickResult fault = MapDoctrineStatus(
            status.code,
            StaticDoctrineDispatchNode::RootSafety,
            StaticDoctrineDispatchStage::Safety,
            frame_index);
        return MakeStaticDoctrineFailure(
            fault.code,
            StaticDoctrineDispatchNode::RootSafety,
            StaticDoctrineDispatchStage::Safety,
            frame_index,
            MapDoctrineStatusReason(status.code));
    }
    if (safety_selected)
    {
        return ResolveStaticDoctrinePriority(true, false, 0U, frame_index);
    }

    bool immediate_defense_selected = false;
    context.CheckImmediateDefenseRequired(immediate_defense_selected, status);
    if (status.code != StatusCode::Ok)
    {
        const BtTickResult fault = MapDoctrineStatus(
            status.code,
            StaticDoctrineDispatchNode::ImmediateDefense,
            StaticDoctrineDispatchStage::ImmediateDefense,
            frame_index);
        return MakeStaticDoctrineFailure(
            fault.code,
            StaticDoctrineDispatchNode::ImmediateDefense,
            StaticDoctrineDispatchStage::ImmediateDefense,
            frame_index,
            MapDoctrineStatusReason(status.code));
    }
    if (immediate_defense_selected)
    {
        return ResolveStaticDoctrinePriority(false, true, 0U, frame_index);
    }

    std::size_t tactical_mode_child_index = 3U;
    context.ResolveTacticalModeChild(tactical_mode_child_index, status);
    if (status.code != StatusCode::Ok)
    {
        const BtTickResult fault = MapDoctrineStatus(
            status.code,
            StaticDoctrineDispatchNode::TacticalMode,
            StaticDoctrineDispatchStage::TacticalMode,
            frame_index);
        return MakeStaticDoctrineFailure(
            fault.code,
            StaticDoctrineDispatchNode::TacticalMode,
            StaticDoctrineDispatchStage::TacticalMode,
            frame_index,
            MapDoctrineStatusReason(status.code));
    }
    return ResolveStaticDoctrinePriority(false,
                                         false,
                                         tactical_mode_child_index,
                                         frame_index);
}

static_assert(
    std::is_same<std::underlying_type<StaticDoctrineRoute>::type,
                 std::uint8_t>::value,
    "Static doctrine route must retain its fixed-width ABI type");
static_assert(std::is_standard_layout<StaticDoctrineDispatchResult>::value,
              "Static doctrine result must remain standard-layout");
static_assert(std::is_trivially_copyable<StaticDoctrineDispatchResult>::value,
              "Static doctrine result must remain trivially copyable");
static_assert(sizeof(StaticDoctrineDispatchResult) == 40U,
              "Static doctrine result x64 ABI size changed");
static_assert(alignof(StaticDoctrineDispatchResult) == 8U,
              "Static doctrine result x64 ABI alignment changed");
static_assert(offsetof(StaticDoctrineDispatchResult, result) == 0U,
              "Static doctrine result receipt offset changed");
static_assert(offsetof(StaticDoctrineDispatchResult, route) == 32U,
              "Static doctrine result route offset changed");
static_assert(
    offsetof(StaticDoctrineDispatchResult, tactical_mode_child_index) == 33U,
    "Static doctrine result child offset changed");
static_assert(offsetof(StaticDoctrineDispatchResult, route_valid) == 34U,
              "Static doctrine result valid offset changed");
static_assert(offsetof(StaticDoctrineDispatchResult, reserved) == 35U,
              "Static doctrine result reserve offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
