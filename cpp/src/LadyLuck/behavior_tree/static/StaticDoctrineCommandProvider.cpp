#include "LadyLuck/behavior_tree/static/StaticDoctrineCommandProvider.hpp"

#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeedControlIntent.hpp"

#include <cmath>

namespace
{

void RecordTerminal(
    LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderSnapshot& snapshot,
    const LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCandidateDisposition disposition,
    const LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderReason reason,
    const LadyLuck::StatusCode status_code,
    LadyLuck::Status& status) noexcept
{
    snapshot.candidate_disposition = disposition;
    snapshot.reason = reason;
    snapshot.provider_status_code = status_code;
    status.code = status_code;
}

LadyLuck::DbfmCornerSpeedControlEvidence BuildSpeedEvidence(
    const LadyLuck::HabfmFrameEvidence& evidence) noexcept
{
    LadyLuck::DbfmCornerSpeedControlEvidence output{};
    output.instantaneous_upper_mps.has_value =
        evidence.own_corner_interval.upper_mps.has_value;
    output.instantaneous_upper_mps.value =
        evidence.own_corner_interval.upper_mps.value;
    output.instantaneous_admitted =
        evidence.own_corner_interval.admitted();
    output.sustained_upper_mps.has_value =
        evidence.own_sustained_corner_interval.upper_mps.has_value;
    output.sustained_upper_mps.value =
        evidence.own_sustained_corner_interval.upper_mps.value;
    output.sustained_admitted =
        evidence.own_sustained_corner_interval.admitted();
    return output;
}

bool BuildDbfmBreakIntent(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::HorizontalBreakReferenceReceipt& reference,
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status) noexcept
{
    output.Clear();
    status = LadyLuck::Status{};
    if (!reference.command_available)
    {
        return false;
    }
    if (!LadyLuck::SameControlFrameIdentity(
            reference.frame_identity, frame.frame_identity)
        || !std::isfinite(reference.aim_point_m[0])
        || !std::isfinite(reference.aim_point_m[1])
        || !std::isfinite(reference.aim_point_m[2])
        || !std::isfinite(reference.desired_speed_mps)
        || !std::isfinite(reference.capture_range_des_m))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return false;
    }
    if (reference.desired_speed_mps <= 0.0
        || reference.capture_range_des_m <= 0.0)
    {
        return false;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = reference.aim_point_m;
    output.desired_speed_mps = reference.desired_speed_mps;
    output.capture_range_des_m = reference.capture_range_des_m;
    output.behavior_id = LadyLuck::DoctrineBehaviorId::DbfmBreak;
    output.mode_id = LadyLuck::DoctrineModeId::Dbfm;
    output.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    output.writer_id = LadyLuck::ControlIntentWriterDbfmBreak;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
        return false;
    }
    return true;
}

bool BuildDbfmAltitudeSeparatedIntent(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::guidance::dbfm::DbfmAltitudeSeparatedReceipt& reference,
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status) noexcept
{
    output.Clear();
    status = LadyLuck::Status{};
    if (!reference.candidate_available)
    {
        return false;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = reference.aim_point_ned_m;
    output.desired_speed_mps = reference.desired_speed_mps;
    output.capture_range_des_m = reference.capture_range_des_m;
    output.behavior_id = LadyLuck::DoctrineBehaviorId::DbfmAltitudeSeparated;
    output.mode_id = LadyLuck::DoctrineModeId::Dbfm;
    output.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    output.writer_id = LadyLuck::ControlIntentWriterDbfmAltitudeSeparated;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
        return false;
    }
    return true;
}

void CopySafetyGunSnapshot(
    const LadyLuck::behavior_tree::static_bt::
        StaticSafetyGunStagedOwner& owner,
    LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderSnapshot& snapshot) noexcept
{
    owner.CopySnapshot(snapshot.safety_gun);
    snapshot.safety_current_required =
        snapshot.safety_gun.safety_current_required;
    snapshot.safety_continuation_required =
        snapshot.safety_gun.safety_feedback_latched;
    snapshot.safety_selected = snapshot.safety_gun.disposition
        == LadyLuck::behavior_tree::static_bt::
            StaticSafetyGunDisposition::AutoGcasPrepared;
    snapshot.gun_evidence = snapshot.safety_gun.root_gun_evidence;
    snapshot.gun_admission = snapshot.safety_gun.gun_admission;
    snapshot.gun_selected = snapshot.safety_gun.disposition
        == LadyLuck::behavior_tree::static_bt::
            StaticSafetyGunDisposition::GunBreakPrepared;
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

namespace
{

constexpr std::uint32_t StaticObfmEmployElementIndex = 14U;
constexpr std::uint32_t StaticObfmG3CounterBarrelElementIndex = 22U;
constexpr std::uint32_t StaticObfmG3CounterRollingScissorsElementIndex = 23U;
constexpr std::uint32_t StaticObfmG3ScissorsElementIndex = 24U;
constexpr std::uint32_t StaticObfmApexElementIndex = 25U;
constexpr std::uint32_t StaticObfmEntryElementIndex = 26U;
constexpr std::uint32_t StaticObfmSpacingElementIndex = 27U;
constexpr std::uint32_t StaticObfmLagElementIndex = 28U;
constexpr std::uint32_t StaticHabfmAvoidElementIndex = 32U;
constexpr std::uint32_t StaticHabfmG10ElementIndex = 33U;
constexpr std::uint32_t StaticHabfmTerminalElementIndex = 34U;
constexpr std::uint32_t StaticDbfmBreakElementIndex = 38U;
constexpr std::uint32_t StaticDbfmAltitudeSeparatedElementIndex = 39U;
constexpr std::uint32_t StaticDbfmTerminalElementIndex = 40U;
constexpr BtNodeId StaticObfmEmployNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmEmployElementIndex]
        .static_node_id;
constexpr BtNodeId StaticObfmLagNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmLagElementIndex].static_node_id;
constexpr BtNodeId StaticObfmG3CounterBarrelNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmG3CounterBarrelElementIndex]
        .static_node_id;
constexpr BtNodeId StaticObfmG3CounterRollingScissorsNodeId =
    generated::LadyLuckV2XmlNodes[
        StaticObfmG3CounterRollingScissorsElementIndex].static_node_id;
constexpr BtNodeId StaticObfmG3ScissorsNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmG3ScissorsElementIndex]
        .static_node_id;
constexpr BtNodeId StaticObfmApexNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmApexElementIndex]
        .static_node_id;
constexpr BtNodeId StaticObfmEntryNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmEntryElementIndex].static_node_id;
constexpr BtNodeId StaticObfmSpacingNodeId =
    generated::LadyLuckV2XmlNodes[StaticObfmSpacingElementIndex]
        .static_node_id;
constexpr BtNodeId StaticHabfmTerminalNodeId =
    generated::LadyLuckV2XmlNodes[StaticHabfmTerminalElementIndex]
        .static_node_id;
constexpr BtNodeId StaticHabfmG10NodeId =
    generated::LadyLuckV2XmlNodes[StaticHabfmG10ElementIndex]
        .static_node_id;
constexpr BtNodeId StaticHabfmAvoidNodeId =
    generated::LadyLuckV2XmlNodes[StaticHabfmAvoidElementIndex]
        .static_node_id;
constexpr BtNodeId StaticDbfmTerminalNodeId =
    generated::LadyLuckV2XmlNodes[StaticDbfmTerminalElementIndex]
        .static_node_id;
constexpr BtNodeId StaticDbfmBreakNodeId =
    generated::LadyLuckV2XmlNodes[StaticDbfmBreakElementIndex]
        .static_node_id;
constexpr BtNodeId StaticDbfmAltitudeSeparatedNodeId =
    generated::LadyLuckV2XmlNodes[StaticDbfmAltitudeSeparatedElementIndex]
        .static_node_id;

struct StaticDoctrineDbfmSelectionContext
{
    const DogfightGeometryFrame* frame = nullptr;
    StaticDoctrineCommandProviderSnapshot* snapshot = nullptr;
    ControlIntent break_candidate{};
    bool break_candidate_available = false;
    ControlIntent altitude_separated_candidate{};
    bool altitude_separated_candidate_available = false;
};

struct StaticDoctrineHabfmSelectionContext
{
    const StaticHabfmG10Prepared* prepared = nullptr;
};

BtTickResult EvaluateStaticHabfmAvoid(
    StaticDoctrineHabfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.prepared != nullptr
        ? context.prepared->frame_identity.frame_index
        : 0U;
    if (context.prepared == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticHabfmAvoidNodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    if (context.prepared->disposition
        != StaticHabfmG10Disposition::HabfmAvoidPassSelected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticHabfmAvoidNodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    proposed = context.prepared->selected_intent;
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticHabfmAvoidNodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticHabfmG10(
    StaticDoctrineHabfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.prepared != nullptr
        ? context.prepared->frame_identity.frame_index
        : 0U;
    if (context.prepared == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticHabfmG10NodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    if (context.prepared->owner.released_this_tick)
    {
        return MakeBtTickResult(
            BtReturnCode::Released,
            StaticHabfmG10NodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    if (context.prepared->disposition
        != StaticHabfmG10Disposition::G10SecondUseSelected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticHabfmG10NodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    proposed = context.prepared->selected_intent;
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticHabfmG10NodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticHabfmBase(
    StaticDoctrineHabfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.prepared != nullptr
        ? context.prepared->frame_identity.frame_index
        : 0U;
    if (context.prepared == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticHabfmTerminalNodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    proposed = context.prepared->selected_intent;
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticHabfmTerminalNodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticDbfmBreak(
    StaticDoctrineDbfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.frame != nullptr
        ? context.frame->frame_identity.frame_index
        : 0U;
    if (context.frame == nullptr || context.snapshot == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticDbfmBreakNodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    context.snapshot->dbfm_break_candidate_available =
        context.break_candidate_available;
    if (!context.break_candidate_available)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticDbfmBreakNodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    proposed = context.break_candidate;
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticDbfmBreakNodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticDbfmHardTurn(
    StaticDoctrineDbfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.frame != nullptr
        ? context.frame->frame_identity.frame_index
        : 0U;
    if (context.frame == nullptr || context.snapshot == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticDbfmTerminalNodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }

    Status geometry_status{};
    ObserveDbfmHardTurnGeometry(
        *context.frame,
        context.snapshot->dbfm_geometry,
        geometry_status);
    context.snapshot->dbfm_geometry_status_code = geometry_status.code;
    if (!geometry_status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(geometry_status.code),
            StaticDbfmTerminalNodeId,
            1U,
            frame_index,
            ToReasonId(MapDoctrineStatusReason(geometry_status.code)));
    }
    if (!context.snapshot->dbfm_geometry.valid
        || !context.snapshot->dbfm_geometry.command_geometry_available)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticDbfmTerminalNodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }

    bool command_available = false;
    Status command_status{};
    BuildDbfmHardTurnCommand(
        *context.frame,
        context.snapshot->dbfm_geometry,
        proposed,
        command_available,
        command_status);
    context.snapshot->raw_candidate_status_code = command_status.code;
    context.snapshot->dbfm_hard_turn_candidate_available =
        command_status.ok() && command_available;
    if (!command_status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(command_status.code),
            StaticDbfmTerminalNodeId,
            1U,
            frame_index,
            ToReasonId(MapDoctrineStatusReason(command_status.code)));
    }
    if (!command_available)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticDbfmTerminalNodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticDbfmTerminalNodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticDbfmAltitudeSeparated(
    StaticDoctrineDbfmSelectionContext& context,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    const std::uint64_t frame_index = context.frame != nullptr
        ? context.frame->frame_identity.frame_index
        : 0U;
    if (context.frame == nullptr || context.snapshot == nullptr)
    {
        return MakeBtTickResult(
            BtReturnCode::InternalContractFault,
            StaticDbfmAltitudeSeparatedNodeId,
            1U,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    context.snapshot->dbfm_altitude_separated_candidate_available =
        context.altitude_separated_candidate_available;
    if (!context.altitude_separated_candidate_available)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticDbfmAltitudeSeparatedNodeId,
            1U,
            frame_index,
            BtReasonIdNone);
    }
    proposed = context.altitude_separated_candidate;
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticDbfmAltitudeSeparatedNodeId,
        1U,
        frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmLagLower(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    owner.PublishObfmLagFallback(status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmLagNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmLagNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticObfmLagNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmG3CounterBarrel(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool released = false;
    owner.EvaluateObfmG3RollCounter(selected, released, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3CounterBarrelNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (released)
    {
        return MakeBtTickResult(
            BtReturnCode::Released,
            StaticObfmG3CounterBarrelNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmG3CounterBarrelNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3CounterBarrelNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmG3CounterBarrelNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmG3CounterRollingScissors(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool released = false;
    owner.EvaluateObfmG3CounterRollingScissors(
        selected, released, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3CounterRollingScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (released)
    {
        return MakeBtTickResult(
            BtReturnCode::Released,
            StaticObfmG3CounterRollingScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmG3CounterRollingScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3CounterRollingScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmG3CounterRollingScissorsNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmG3Scissors(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool released = false;
    owner.EvaluateObfmG3Scissors(selected, released, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3ScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (released)
    {
        return MakeBtTickResult(
            BtReturnCode::Released,
            StaticObfmG3ScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmG3ScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmG3ScissorsNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmG3ScissorsNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmApex(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool released = false;
    owner.EvaluateObfmApex(selected, released, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmApexNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (released)
    {
        return MakeBtTickResult(
            BtReturnCode::Released,
            StaticObfmApexNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmApexNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmApexNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmApexNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmEmploy(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    owner.SelectObfmEmploy(selected, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEmployNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmEmployNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.PublishObfmEmploy(status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEmployNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEmployNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Selected,
        StaticObfmEmployNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmEntry(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool completed = false;
    owner.SelectObfmEntry(selected, completed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (completed)
    {
        owner.CompleteObfmEntry();
        return MakeBtTickResult(
            BtReturnCode::Completed,
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (!selected)
    {
        return MakeBtTickResult(
            BtReturnCode::NotApplicable,
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.PublishObfmEntry(status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmEntryNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

BtTickResult EvaluateStaticObfmSpacing(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent& proposed) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    Status status{};
    bool selected = false;
    bool completed = false;
    bool released = false;
    owner.EvaluateObfmSpacing(selected, completed, released, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmSpacingNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    if (completed)
    {
        return MakeBtTickResult(
            BtReturnCode::Completed,
            StaticObfmSpacingNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    if (released || !selected)
    {
        return MakeBtTickResult(
            released ? BtReturnCode::Released : BtReturnCode::NotApplicable,
            StaticObfmSpacingNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            BtReasonIdNone);
    }
    owner.CopyStagedBaseIntent(proposed, status);
    if (!status.ok())
    {
        return MakeBtTickResult(
            MapDoctrineStatusReturnCode(status.code),
            StaticObfmSpacingNodeId,
            ObfmG16G5bStageId,
            snapshot.frame_identity.frame_index,
            ToReasonId(MapDoctrineStatusReason(status.code)));
    }
    return MakeBtTickResult(
        BtReturnCode::Running,
        StaticObfmSpacingNodeId,
        ObfmG16G5bStageId,
        proposed.frame_identity.frame_index,
        BtReasonIdNone);
}

template <BtNodeId NodeId>
BtTickResult RejectUnexpectedObfmRedispatch(
    StaticDoctrineObfmG16G5bOwner& owner,
    const ControlIntent*,
    ControlIntent&) noexcept
{
    StaticDoctrineObfmG16G5bSnapshot snapshot{};
    owner.CopySnapshot(snapshot);
    return MakeBtTickResult(
        BtReturnCode::NotApplicable,
        NodeId,
        ObfmG16G5bStageId,
        snapshot.frame_identity.frame_index,
        BtReasonIdNone);
}

StaticDoctrineCommandProviderReason ObfmSelectedReason(
    const std::uint32_t writer_id) noexcept
{
    switch (writer_id)
    {
    case guidance::obfm::ControlIntentWriterObfmEmploy:
        return StaticDoctrineCommandProviderReason::ObfmEmploySelected;
    case ControlIntentWriterG16Committed:
        return StaticDoctrineCommandProviderReason::
            ObfmG16CommittedSelected;
    case ControlIntentWriterG16HighPrevention:
        return StaticDoctrineCommandProviderReason::ObfmG16HighSelected;
    case guidance::obfm::ControlIntentWriterG5bDelayedClimb:
        return StaticDoctrineCommandProviderReason::ObfmG5bSelected;
    case ControlIntentWriterG3CounterBarrel:
        return StaticDoctrineCommandProviderReason::
            ObfmG3CounterBarrelSelected;
    case ControlIntentWriterG3CounterRollingScissors:
        return StaticDoctrineCommandProviderReason::
            ObfmG3CounterRollingScissorsSelected;
    case ControlIntentWriterG3Scissors:
        return StaticDoctrineCommandProviderReason::
            ObfmG3ScissorsSelected;
    case ControlIntentWriterObfmApexDisplacement:
        return StaticDoctrineCommandProviderReason::ObfmApexSelected;
    case ControlIntentWriterObfmSpacing:
        return StaticDoctrineCommandProviderReason::ObfmSpacingSelected;
    case ControlIntentWriterObfmEntrySetup:
        return StaticDoctrineCommandProviderReason::ObfmEntrySelected;
    case ControlIntentWriterObfmLag:
    default:
        return StaticDoctrineCommandProviderReason::ObfmLagSelected;
    }
}

StaticDoctrineCommandProviderReason ImmediateGunSelectedReason(
    const std::uint32_t writer_id) noexcept
{
    switch (writer_id)
    {
    case ControlIntentWriterG4HighGBarrel:
        return StaticDoctrineCommandProviderReason::
            ImmediateGunG4HighGSelected;
    case ControlIntentWriterOfficialGunSnapshotPlaneChange:
        return StaticDoctrineCommandProviderReason::
            ImmediateGunSnapshotSelected;
    case ControlIntentWriterDbfmHardTurn:
        return StaticDoctrineCommandProviderReason::
            ImmediateGunPhaseHardTurnSelected;
    case ControlIntentWriterGunDefenseHorizontalBreak:
    default:
        return StaticDoctrineCommandProviderReason::
            ImmediateGunDefenseSelected;
    }
}

void CopyObfmCompatibilitySnapshot(
    const StaticDoctrineObfmG16G5bSnapshot& owner,
    StaticDoctrineCommandProviderSnapshot& provider) noexcept
{
    provider.obfm_current_speed_echo_ready = owner.precision_speed_ready;
    provider.obfm_current_speed_echo_mps =
        owner.precision_speed.desired_speed_mps;
    provider.obfm_speed_authority = owner.lag_speed_authority;
    provider.obfm_station_hold = owner.lag_station_observation_ready
        ? owner.lag_station
        : owner.precision_station;
    provider.obfm_lag_preparation = owner.precision_lag_preparation;
    provider.obfm_longitudinal = owner.precision_longitudinal;
    provider.obfm_bumpless = owner.precision_bumpless;
    provider.obfm_lag_commit = owner.lag_commit;
}

static_assert(StaticObfmEmployNodeId == 15U,
              "Frozen current-effect EMPLOY node identity changed.");
static_assert(StaticObfmG3CounterBarrelNodeId == 23U,
              "Frozen G3 counter-barrel node identity changed.");
static_assert(StaticObfmG3CounterRollingScissorsNodeId == 24U,
              "Frozen G3 counter-rolling-scissors identity changed.");
static_assert(StaticObfmG3ScissorsNodeId == 25U,
              "Frozen G3 scissors gap/egress identity changed.");
static_assert(StaticObfmApexNodeId == 26U,
              "Frozen OBFM Apex displacement identity changed.");
static_assert(StaticObfmSpacingNodeId == 28U,
              "Frozen OBFM SPACING node identity changed.");
static_assert(StaticObfmEntryNodeId == 27U,
              "Frozen current-turn OBFM ENTRY node identity changed.");
static_assert(StaticObfmLagNodeId == 29U,
              "Frozen ordinary OBFM LAG node identity changed.");
static_assert(StaticHabfmAvoidNodeId == 33U,
              "Frozen HABFM avoid-pass node identity changed.");
static_assert(StaticHabfmG10NodeId == 34U,
              "Frozen HABFM G10 node identity changed.");
static_assert(StaticHabfmTerminalNodeId == 35U,
              "Frozen HABFM terminal node identity changed.");
static_assert(StaticDbfmBreakNodeId == 39U,
              "Frozen DBFM BREAK node identity changed.");
static_assert(StaticDbfmAltitudeSeparatedNodeId == 40U,
              "Frozen DBFM altitude-separated node identity changed.");
static_assert(StaticDbfmTerminalNodeId == 41U,
              "Frozen DBFM terminal node identity changed.");

} // namespace

StaticDoctrineCommandProvider::StaticDoctrineCommandProvider() noexcept
{
    Reset();
}

void StaticDoctrineCommandProvider::Reset() noexcept
{
    frame_evidence_provider_.Reset();
    habfm_g10_owner_.Reset();
    obfm_owner_.Reset();
    obfm_adapter_ = StaticDoctrineObfmG16G5bAdapter<
        StaticDoctrineObfmG16G5bOwner>{};
    safety_gun_owner_.Reset();
    immediate_gun_response_owner_.Reset();
    habfm_g10_prepared_ = StaticHabfmG10Prepared{};
    ClearPreparedTransaction();
    snapshot_ = StaticDoctrineCommandProviderSnapshot{};
}

void StaticDoctrineCommandProvider::Build(
    const runtime::TacticalCommandBuildInput& input,
    ControlIntent& output,
    Status& status) noexcept
{
    BuildInternal(input, nullptr, output, status);
}

void StaticDoctrineCommandProvider::BuildWithProjection(
    const runtime::TacticalCommandBuildInput& input,
    runtime::ICurrentCisV4EnergyProjectionPort& projection_port,
    ControlIntent& output,
    Status& status) noexcept
{
    BuildInternal(input, &projection_port, output, status);
}

void StaticDoctrineCommandProvider::ClearPreparedTransaction() noexcept
{
    prepared_transaction_ = StaticDoctrinePreparedTransaction{};
}

void StaticDoctrineCommandProvider::StagePreparedTransaction(
    const StaticDoctrinePreparedOwner owner,
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t writer_id) noexcept
{
    ClearPreparedTransaction();
    prepared_transaction_.valid = true;
    prepared_transaction_.owner = owner;
    prepared_transaction_.frame_identity = frame_identity;
    prepared_transaction_.writer_id = writer_id;
    snapshot_.prepared_transaction_ready = true;
    snapshot_.prepared_transaction_committed = false;
    snapshot_.prepared_frame_identity = frame_identity;
    snapshot_.prepared_writer_id = writer_id;
}

void StaticDoctrineCommandProvider::CommitPrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t writer_id,
    Status& status) noexcept
{
    status = Status{};
    if (!prepared_transaction_.valid
        || !SameControlFrameIdentity(
            prepared_transaction_.frame_identity,
            frame_identity)
        || prepared_transaction_.writer_id != writer_id)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    Status tactical_validation_status{};
    switch (prepared_transaction_.owner)
    {
    case StaticDoctrinePreparedOwner::SafetyGun:
        if (writer_id != ControlIntentWriterAutoGcasRecovery
            && writer_id
                != ControlIntentWriterGunDefenseHorizontalBreak
            && writer_id
                != ControlIntentWriterOfficialGunSnapshotPlaneChange
            && writer_id != ControlIntentWriterG4HighGBarrel
            && writer_id != ControlIntentWriterDbfmHardTurn)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        break;
    case StaticDoctrinePreparedOwner::ObfmG16G5b:
        obfm_owner_.ValidatePrepared(
            frame_identity, writer_id, tactical_validation_status);
        break;
    case StaticDoctrinePreparedOwner::Habfm:
        if (writer_id != ControlIntentWriterHabfm
            && writer_id != ControlIntentWriterHabfmAvoidPass
            && writer_id != ControlIntentWriterG10SecondUse)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        habfm_g10_owner_.ValidatePublished(
            habfm_g10_prepared_,
            writer_id,
            tactical_validation_status);
        break;
    case StaticDoctrinePreparedOwner::Dbfm:
        if (writer_id != ControlIntentWriterDbfmHardTurn
            && writer_id != ControlIntentWriterDbfmBreak
            && writer_id != ControlIntentWriterDbfmAltitudeSeparated)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        break;
    default:
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    if (!tactical_validation_status.ok())
    {
        status = tactical_validation_status;
        return;
    }

    Status safety_gun_validation_status{};
    safety_gun_owner_.ValidatePrepared(
        frame_identity,
        writer_id,
        safety_gun_validation_status);
    if (!safety_gun_validation_status.ok())
    {
        status = safety_gun_validation_status;
        return;
    }

    if (snapshot_.gun_response.prepare_attempted)
    {
        Status gun_response_validation_status{};
        immediate_gun_response_owner_.ValidatePrepared(
            frame_identity,
            writer_id,
            gun_response_validation_status);
        if (!gun_response_validation_status.ok())
        {
            status = gun_response_validation_status;
            return;
        }
    }

    // All owner-private same-frame, writer, and generation checks have passed.
    // Provider finalization is serialized, so the following owner mutations
    // cannot invalidate one another between validation and commit.
    switch (prepared_transaction_.owner)
    {
    case StaticDoctrinePreparedOwner::SafetyGun:
        break;
    case StaticDoctrinePreparedOwner::ObfmG16G5b:
        obfm_owner_.CommitPrepared(frame_identity, writer_id, status);
        if (status.ok())
        {
            obfm_owner_.CopySnapshot(snapshot_.obfm_owner);
            CopyObfmCompatibilitySnapshot(snapshot_.obfm_owner, snapshot_);
            snapshot_.obfm_lag_commit_applied =
                writer_id == ControlIntentWriterObfmLag;
        }
        break;
    case StaticDoctrinePreparedOwner::Habfm:
        habfm_g10_owner_.CommitPublished(
            habfm_g10_prepared_,
            writer_id,
            status);
        if (status.ok())
        {
            snapshot_.habfm_prepared =
                habfm_g10_prepared_.habfm_base;
            snapshot_.habfm_commit_applied = true;
        }
        break;
    case StaticDoctrinePreparedOwner::Dbfm:
        break;
    default:
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!status.ok())
    {
        return;
    }

    if (snapshot_.gun_response.prepare_attempted)
    {
        Status gun_response_status{};
        immediate_gun_response_owner_.CommitPrepared(
            frame_identity,
            writer_id,
            gun_response_status);
        if (!gun_response_status.ok())
        {
            status = gun_response_status;
            return;
        }
        immediate_gun_response_owner_.CopySnapshot(snapshot_.gun_response);
        snapshot_.gun_snapshot = snapshot_.gun_response.snapshot;
    }

    Status safety_gun_status{};
    safety_gun_owner_.CommitPrepared(
        frame_identity,
        writer_id,
        safety_gun_status);
    if (!safety_gun_status.ok())
    {
        status = safety_gun_status;
        return;
    }
    CopySafetyGunSnapshot(safety_gun_owner_, snapshot_);

    snapshot_.prepared_transaction_ready = false;
    snapshot_.prepared_transaction_committed = true;
    ClearPreparedTransaction();
}

void StaticDoctrineCommandProvider::AbortPrepared() noexcept
{
    obfm_adapter_.Abort(obfm_owner_);
    obfm_owner_.AbortPrepared();
    safety_gun_owner_.AbortPrepared();
    immediate_gun_response_owner_.AbortPrepared();
    habfm_g10_prepared_ = StaticHabfmG10Prepared{};
    CopySafetyGunSnapshot(safety_gun_owner_, snapshot_);
    immediate_gun_response_owner_.CopySnapshot(snapshot_.gun_response);
    snapshot_.gun_snapshot = snapshot_.gun_response.snapshot;
    ClearPreparedTransaction();
    snapshot_.prepared_transaction_ready = false;
    snapshot_.prepared_transaction_committed = false;
    snapshot_.prepared_frame_identity = ControlFrameIdentity{};
    snapshot_.prepared_writer_id = ControlIntentWriterNone;
}

void StaticDoctrineCommandProvider::CopySnapshot(
    StaticDoctrineCommandProviderSnapshot& output) const noexcept
{
    output = snapshot_;
}

void StaticDoctrineCommandProvider::CopySnapshot(
    StaticDoctrineCommandProviderSnapshot& output,
    Status& status) const noexcept
{
    output = snapshot_;
    status = Status{};
}

void StaticDoctrineCommandProvider::BuildObfm(
    const runtime::TacticalCommandBuildInput& input,
    const StaticDoctrineObfmG16G5bInput& obfm_input,
    const bool include_current_effect_employ,
    ControlIntent& output,
    Status& status) noexcept
{
    Status owner_status{};
    obfm_owner_.Prepare(obfm_input, snapshot_.obfm_owner, owner_status);
    if (!owner_status.ok())
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmLifecycleFault,
            owner_status.code,
            status);
        return;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner> lower{};
    lower.descriptor = StaticBtChildDescriptor{
        StaticObfmLagNodeId,
        ObfmG16G5bStageId,
        0U,
        BtLifecycleOwnerIdNone};
    lower.callback = &EvaluateStaticObfmLagLower;

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner> employ{};
    if (include_current_effect_employ)
    {
        employ.descriptor = StaticBtChildDescriptor{
            StaticObfmEmployNodeId,
            ObfmG16G5bStageId,
            0U,
            BtLifecycleOwnerIdNone};
        employ.callback = &EvaluateStaticObfmEmploy;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner> entry{};
    if (!include_current_effect_employ)
    {
        entry.descriptor = StaticBtChildDescriptor{
            StaticObfmEntryNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmEntryOwnerId};
        entry.callback = &EvaluateStaticObfmEntry;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner>
        g3_counter_barrel{};
    if (!include_current_effect_employ)
    {
        g3_counter_barrel.descriptor = StaticBtChildDescriptor{
            StaticObfmG3CounterBarrelNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmG3CounterBarrelOwnerId};
        g3_counter_barrel.callback =
            &EvaluateStaticObfmG3CounterBarrel;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner>
        g3_counter_rolling_scissors{};
    if (!include_current_effect_employ)
    {
        g3_counter_rolling_scissors.descriptor = StaticBtChildDescriptor{
            StaticObfmG3CounterRollingScissorsNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmG3CounterRollingScissorsOwnerId};
        g3_counter_rolling_scissors.callback =
            &EvaluateStaticObfmG3CounterRollingScissors;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner>
        g3_scissors{};
    if (!include_current_effect_employ)
    {
        g3_scissors.descriptor = StaticBtChildDescriptor{
            StaticObfmG3ScissorsNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmG3ScissorsOwnerId};
        g3_scissors.callback = &EvaluateStaticObfmG3Scissors;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner> apex{};
    if (!include_current_effect_employ)
    {
        apex.descriptor = StaticBtChildDescriptor{
            StaticObfmApexNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmApexOwnerId};
        apex.callback = &EvaluateStaticObfmApex;
    }

    StaticDoctrineObfmLowerLeaf<StaticDoctrineObfmG16G5bOwner> spacing{};
    if (!include_current_effect_employ)
    {
        spacing.descriptor = StaticBtChildDescriptor{
            StaticObfmSpacingNodeId,
            ObfmG16G5bStageId,
            0U,
            ObfmSpacingOwnerId};
        spacing.callback = &EvaluateStaticObfmSpacing;
    }

    StaticDoctrineObfmModeRedispatchLeaves<
        StaticDoctrineObfmG16G5bOwner> redispatch{};
    redispatch.habfm.descriptor = StaticBtChildDescriptor{
        StaticHabfmTerminalNodeId,
        ObfmG16G5bStageId,
        0U,
        BtLifecycleOwnerIdNone};
    redispatch.habfm.callback =
        &RejectUnexpectedObfmRedispatch<StaticHabfmTerminalNodeId>;
    redispatch.dbfm.descriptor = StaticBtChildDescriptor{
        StaticDbfmTerminalNodeId,
        ObfmG16G5bStageId,
        0U,
        BtLifecycleOwnerIdNone};
    redispatch.dbfm.callback =
        &RejectUnexpectedObfmRedispatch<StaticDbfmTerminalNodeId>;

    snapshot_.obfm_result = obfm_adapter_.Evaluate(
        obfm_owner_,
        input.frame.frame_identity,
        lower,
        redispatch,
        employ,
        g3_counter_barrel,
        g3_counter_rolling_scissors,
        g3_scissors,
        spacing,
        entry,
        apex);
    obfm_owner_.CopySnapshot(snapshot_.obfm_owner);
    CopyObfmCompatibilitySnapshot(snapshot_.obfm_owner, snapshot_);
    if (IsBtReturnCodeError(
            snapshot_.obfm_result.lifecycle.selection.result.code)
        || !snapshot_.obfm_result.lifecycle.selection.candidate_available
        || !snapshot_.obfm_result.pending_commit)
    {
        obfm_adapter_.Abort(obfm_owner_);
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmLifecycleFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    snapshot_.obfm_adapter_commit = obfm_adapter_.Commit(obfm_owner_);
    obfm_owner_.CopySnapshot(snapshot_.obfm_owner);
    CopyObfmCompatibilitySnapshot(snapshot_.obfm_owner, snapshot_);
    if (IsBtReturnCodeError(snapshot_.obfm_adapter_commit.code))
    {
        obfm_owner_.AbortPrepared();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmPreparedStateFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    obfm_owner_.CopyPreparedIntent(output, owner_status);
    if (!owner_status.ok())
    {
        obfm_owner_.AbortPrepared();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmPreparedStateFault,
            owner_status.code,
            status);
        return;
    }
    const bool expected_writer =
        output.writer_id
            == guidance::obfm::ControlIntentWriterObfmEmploy
        || output.writer_id == ControlIntentWriterG16Committed
        || output.writer_id == ControlIntentWriterG16HighPrevention
        || output.writer_id == ControlIntentWriterObfmLag
        || output.writer_id == ControlIntentWriterG3CounterBarrel
        || output.writer_id
            == ControlIntentWriterG3CounterRollingScissors
        || output.writer_id == ControlIntentWriterG3Scissors
        || output.writer_id == ControlIntentWriterObfmApexDisplacement
        || output.writer_id == ControlIntentWriterObfmSpacing
        || output.writer_id == ControlIntentWriterObfmEntrySetup
        || output.writer_id
            == guidance::obfm::ControlIntentWriterG5bDelayedClimb;
    Status selected_status{};
    output.Validate(selected_status);
    if (!selected_status.ok()
        || !expected_writer
        || !SameControlFrameIdentity(
            output.frame_identity, input.frame.frame_identity))
    {
        obfm_owner_.AbortPrepared();
        output.Clear();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmWriterContractFault,
            selected_status.ok()
                ? StatusCode::InvalidConfiguration
                : selected_status.code,
            status);
        return;
    }

    snapshot_.raw_candidate = output;
    snapshot_.selected_candidate = output;
    snapshot_.raw_candidate_status_code = StatusCode::Ok;
    snapshot_.selected_candidate_status_code = StatusCode::Ok;
    snapshot_.candidate_count = 1U;
    snapshot_.selected_candidate_count = 1U;
    snapshot_.candidate_disposition =
        StaticDoctrineCandidateDisposition::Selected;
    snapshot_.reason = snapshot_.obfm_result.current_base_recovered
        ? StaticDoctrineCommandProviderReason::
            ObfmLagSelectedAfterTacticalFault
        : ObfmSelectedReason(output.writer_id);
    StagePreparedTransaction(
        StaticDoctrinePreparedOwner::ObfmG16G5b,
        output.frame_identity,
        output.writer_id);
    snapshot_.provider_status_code = StatusCode::Ok;
    status = Status{};
}

void StaticDoctrineCommandProvider::BuildHabfm(
    const runtime::TacticalCommandBuildInput& input,
    runtime::ICurrentCisV4EnergyProjectionPort* const projection_port,
    ControlIntent& output,
    Status& status) noexcept
{
    HabfmTerminalControlIntentInput habfm_input{};
    habfm_input.frame = input.frame;
    habfm_input.current_envelope = input.current_envelope;
    Status observation_status{};
    habfm_g10_owner_.Observe(
        habfm_input,
        snapshot_.habfm_observation,
        observation_status);
    snapshot_.habfm_observation_status_code = observation_status.code;
    if (!observation_status.ok())
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::HabfmObservationFault,
            observation_status.code,
            status);
        return;
    }
    if (!snapshot_.habfm_observation.evaluated
        || !snapshot_.habfm_observation.applicable
        || !SameControlFrameIdentity(
            snapshot_.habfm_observation.frame_identity,
            input.frame.frame_identity))
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::NotApplicable,
            StaticDoctrineCommandProviderReason::HabfmNotApplicable,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    Status preparation_status{};
    habfm_g10_owner_.Prepare(
        input,
        projection_port,
        snapshot_.habfm_observation,
        habfm_g10_prepared_,
        preparation_status);
    snapshot_.habfm_prepared =
        habfm_g10_prepared_.habfm_base;
    snapshot_.habfm_g10_disposition =
        habfm_g10_prepared_.disposition;
    snapshot_.habfm_g10_optional_status_code =
        habfm_g10_prepared_.optional_g10_status_code;
    snapshot_.habfm_avoid_overlay = habfm_g10_prepared_.avoid_overlay;
    snapshot_.habfm_avoid_optional_status_code =
        habfm_g10_prepared_.optional_avoid_status_code;
    snapshot_.habfm_h09_storage = habfm_g10_prepared_.h09_storage;
    snapshot_.habfm_h09_projection = habfm_g10_prepared_.h09_projection;
    snapshot_.habfm_h09_allocation = habfm_g10_prepared_.h09_allocation;
    snapshot_.habfm_h09_optional_status_code =
        habfm_g10_prepared_.optional_h09_status_code;
    snapshot_.habfm_h09_projection_attempted =
        habfm_g10_prepared_.h09_projection_attempted;
    snapshot_.habfm_h09_active = habfm_g10_prepared_.h09_active;
    snapshot_.projection_port_used =
        habfm_g10_prepared_.h09_projection_attempted;
    snapshot_.habfm_preparation_status_code = preparation_status.code;
    if (!preparation_status.ok()
        || !habfm_g10_prepared_.next_state_ready)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::HabfmPreparationFault,
            preparation_status.ok()
                ? StatusCode::InvalidConfiguration
                : preparation_status.code,
            status);
        return;
    }

    StaticDoctrineHabfmSelectionContext habfm_context{};
    habfm_context.prepared = &habfm_g10_prepared_;
    const StaticBtChildTable<3U> habfm_children{{{
        StaticBtChildDescriptor{
            StaticHabfmAvoidNodeId,
            1U,
            0U,
            BtLifecycleOwnerIdNone},
        StaticBtChildDescriptor{
            StaticHabfmG10NodeId,
            1U,
            1U,
            MakeBtLifecycleOwnerId(ControlIntentWriterG10SecondUse)},
        StaticBtChildDescriptor{
            StaticHabfmTerminalNodeId,
            1U,
            2U,
            BtLifecycleOwnerIdNone}}}, 3U};
    const StaticBtCallbackTable<StaticDoctrineHabfmSelectionContext, 3U>
        habfm_callbacks{{{
            &EvaluateStaticHabfmAvoid,
            &EvaluateStaticHabfmG10,
            &EvaluateStaticHabfmBase}}, 3U};
    const StaticBtCandidateSelection habfm_selection =
        EvaluateStaticBtCandidateChildren(
            habfm_context,
            habfm_children,
            habfm_callbacks,
            nullptr,
            input.frame.frame_identity,
            input.frame.frame_identity.frame_index);
    snapshot_.habfm_selector_result = habfm_selection.result;
    if (!habfm_selection.candidate_available)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::HabfmPreparationFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    snapshot_.raw_candidate = habfm_selection.candidate;
    Status raw_status{};
    snapshot_.raw_candidate.Validate(raw_status);
    snapshot_.raw_candidate_status_code = raw_status.code;
    if (!raw_status.ok()
        || (snapshot_.raw_candidate.writer_id != ControlIntentWriterHabfm
            && snapshot_.raw_candidate.writer_id
                != ControlIntentWriterHabfmAvoidPass
            && snapshot_.raw_candidate.writer_id
                != ControlIntentWriterG10SecondUse)
        || !SameControlFrameIdentity(
            snapshot_.raw_candidate.frame_identity,
            input.frame.frame_identity))
    {
        snapshot_.raw_candidate.Clear();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::HabfmWriterContractFault,
            raw_status.ok()
                ? StatusCode::InvalidConfiguration
                : raw_status.code,
            status);
        return;
    }
    snapshot_.candidate_count = 1U;

    snapshot_.selected_candidate = snapshot_.raw_candidate;
    Status selected_status{};
    snapshot_.selected_candidate.Validate(selected_status);
    snapshot_.selected_candidate_status_code = selected_status.code;
    if (!selected_status.ok()
        || (snapshot_.selected_candidate.writer_id
                != ControlIntentWriterHabfm
            && snapshot_.selected_candidate.writer_id
                != ControlIntentWriterHabfmAvoidPass
            && snapshot_.selected_candidate.writer_id
                != ControlIntentWriterG10SecondUse)
        || !SameControlFrameIdentity(
            snapshot_.selected_candidate.frame_identity,
            input.frame.frame_identity))
    {
        snapshot_.selected_candidate.Clear();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::HabfmWriterContractFault,
            selected_status.ok()
                ? StatusCode::InvalidConfiguration
                : selected_status.code,
            status);
        return;
    }

    snapshot_.selected_candidate_count = 1U;
    snapshot_.candidate_disposition =
        StaticDoctrineCandidateDisposition::Selected;
    snapshot_.reason = snapshot_.selected_candidate.writer_id
            == ControlIntentWriterG10SecondUse
        ? StaticDoctrineCommandProviderReason::HabfmG10SecondUseSelected
        : snapshot_.selected_candidate.writer_id
                == ControlIntentWriterHabfmAvoidPass
            ? StaticDoctrineCommandProviderReason::HabfmAvoidPassSelected
            : StaticDoctrineCommandProviderReason::HabfmSelected;
    output = snapshot_.selected_candidate;
    StagePreparedTransaction(
        StaticDoctrinePreparedOwner::Habfm,
        output.frame_identity,
        output.writer_id);
    snapshot_.provider_status_code = StatusCode::Ok;
    status = Status{};
}

void StaticDoctrineCommandProvider::BuildInternal(
    const runtime::TacticalCommandBuildInput& input,
    runtime::ICurrentCisV4EnergyProjectionPort* const projection_port,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status.code = StatusCode::InvalidConfiguration;
    AbortPrepared();
    snapshot_ = StaticDoctrineCommandProviderSnapshot{};
    snapshot_.build_attempted = true;
    snapshot_.projection_port_supplied = projection_port != nullptr;
    snapshot_.projection_port_used = false;
    snapshot_.frame_identity = input.frame.frame_identity;

    if (!input.valid)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::InputNotAdmitted,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }
    if (!IsValidControlFrameIdentity(input.frame.frame_identity))
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::InputNotAdmitted,
            StatusCode::InvalidArgument,
            status);
        return;
    }
    snapshot_.input_admitted = true;

    ControlIntent safety_gun_candidate{};
    Status safety_gun_status{};
    safety_gun_owner_.Prepare(
        input,
        safety_gun_candidate,
        snapshot_.safety_gun,
        safety_gun_status);
    CopySafetyGunSnapshot(safety_gun_owner_, snapshot_);
    if (!safety_gun_status.ok()
        || snapshot_.safety_gun.disposition
            == StaticSafetyGunDisposition::Fault)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::
                SafetyGunPreparationFault,
            safety_gun_status.ok()
                ? StatusCode::InvalidConfiguration
                : safety_gun_status.code,
            status);
        return;
    }

    if (snapshot_.safety_gun.disposition
            == StaticSafetyGunDisposition::AutoGcasPrepared
        || snapshot_.safety_gun.disposition
            == StaticSafetyGunDisposition::GunBreakPrepared)
    {
        const bool auto_gcas_selected =
            snapshot_.safety_gun.disposition
            == StaticSafetyGunDisposition::AutoGcasPrepared;
        ControlIntent selected_safety_gun_candidate = safety_gun_candidate;
        if (!auto_gcas_selected)
        {
            StaticImmediateGunResponseStagedInput response_input{};
            response_input.tactical_input = input;
            response_input.writer2_same_frame_admitted =
                snapshot_.safety_gun.disposition
                == StaticSafetyGunDisposition::GunBreakPrepared;
            response_input.safety_gun_frame_identity =
                snapshot_.safety_gun.frame_identity;
            response_input.entry_side_sign_valid =
                snapshot_.safety_gun.gun_admission.entry_side_sign_valid;
            response_input.entry_side_sign =
                snapshot_.safety_gun.gun_admission.entry_side_sign;
            response_input.root_gun_evidence =
                snapshot_.safety_gun.root_gun_evidence;
            response_input.base_break = safety_gun_candidate;
            Status gun_response_status{};
            immediate_gun_response_owner_.Prepare(
                response_input,
                selected_safety_gun_candidate,
                snapshot_.gun_response,
                gun_response_status);
            immediate_gun_response_owner_.CopySnapshot(
                snapshot_.gun_response);
            snapshot_.gun_snapshot = snapshot_.gun_response.snapshot;
            if (!gun_response_status.ok())
            {
                RecordTerminal(
                    snapshot_,
                    StaticDoctrineCandidateDisposition::Fault,
                    StaticDoctrineCommandProviderReason::
                        ImmediateGunResponsePreparationFault,
                    gun_response_status.code,
                    status);
                return;
            }
        }

        const std::uint32_t expected_writer_id = auto_gcas_selected
            ? ControlIntentWriterAutoGcasRecovery
            : selected_safety_gun_candidate.writer_id;
        snapshot_.raw_candidate = selected_safety_gun_candidate;
        Status raw_status{};
        snapshot_.raw_candidate.Validate(raw_status);
        snapshot_.raw_candidate_status_code = raw_status.code;
        if (!raw_status.ok()
            || snapshot_.raw_candidate.writer_id != expected_writer_id
            || !SameControlFrameIdentity(
                snapshot_.raw_candidate.frame_identity,
                input.frame.frame_identity))
        {
            snapshot_.raw_candidate.Clear();
            RecordTerminal(
                snapshot_,
                StaticDoctrineCandidateDisposition::Fault,
                StaticDoctrineCommandProviderReason::
                    SafetyGunPreparationFault,
                raw_status.ok()
                    ? StatusCode::InvalidConfiguration
                    : raw_status.code,
                status);
            return;
        }
        snapshot_.candidate_count = 1U;

        snapshot_.selected_candidate = snapshot_.raw_candidate;
        Status selected_status{};
        snapshot_.selected_candidate.Validate(selected_status);
        snapshot_.selected_candidate_status_code = selected_status.code;
        if (!selected_status.ok()
            || snapshot_.selected_candidate.writer_id
                != expected_writer_id
            || !SameControlFrameIdentity(
                snapshot_.selected_candidate.frame_identity,
                input.frame.frame_identity))
        {
            snapshot_.selected_candidate.Clear();
            RecordTerminal(
                snapshot_,
                StaticDoctrineCandidateDisposition::Fault,
                StaticDoctrineCommandProviderReason::
                    SafetyGunPreparationFault,
                selected_status.ok()
                    ? StatusCode::InvalidConfiguration
                    : selected_status.code,
                status);
            return;
        }

        snapshot_.selected_candidate_count = 1U;
        snapshot_.candidate_disposition =
            StaticDoctrineCandidateDisposition::Selected;
        snapshot_.reason = snapshot_.safety_selected
            ? StaticDoctrineCommandProviderReason::AutoGcasSelected
            : ImmediateGunSelectedReason(expected_writer_id);
        snapshot_.provider_status_code = StatusCode::Ok;
        output = snapshot_.selected_candidate;
        StagePreparedTransaction(
            StaticDoctrinePreparedOwner::SafetyGun,
            output.frame_identity,
            output.writer_id);
        status = Status{};
        return;
    }
    if (snapshot_.safety_gun.disposition
        != StaticSafetyGunDisposition::NotApplicable)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::
                SafetyGunPreparationFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    StaticDoctrineObfmG16G5bInputSources obfm_sources{};
    obfm_sources.tactical_input = input;
    obfm_sources.safety_gun_receipt_available =
        snapshot_.safety_gun.prepare_attempted;
    obfm_sources.safety_gun = snapshot_.safety_gun;
    // No independent previous-pursuit producer exists in this static provider.
    // Typed absence preserves ordinary writer 5 instead of fabricating history.
    StaticDoctrineObfmG16G5bInput obfm_input{};
    Status input_builder_status{};
    obfm_input_builder_.Build(
        obfm_sources,
        obfm_input,
        snapshot_.obfm_input_builder,
        input_builder_status);
    snapshot_.root_receipt =
        snapshot_.obfm_input_builder.root_observation;
    snapshot_.root_status_code =
        snapshot_.obfm_input_builder.root_status_code;
    if (!input_builder_status.ok()
        || !snapshot_.obfm_input_builder.output_ready)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::ObfmInputBuildFault,
            input_builder_status.ok()
                ? StatusCode::InvalidConfiguration
                : input_builder_status.code,
            status);
        return;
    }

    snapshot_.root_classification_available = true;
    snapshot_.classified_mode =
        obfm_input.mode_decision.mode.value;
    if (std::isfinite(input.frame.own_offense.damage_rate)
        && input.frame.own_offense.damage_rate > 0.0)
    {
        StaticDoctrineObfmG16G5bInput current_effect_input = obfm_input;
        current_effect_input.current_effect_employ_override = true;
        snapshot_.classified_route_implementation =
            StaticDoctrineRouteImplementation::Implemented;
        BuildObfm(
            input,
            current_effect_input,
            true,
            output,
            status);
        if (status.ok()
            && output.writer_id
                == guidance::obfm::ControlIntentWriterObfmEmploy)
        {
            snapshot_.reason = StaticDoctrineCommandProviderReason::
                CurrentEffectEmploySelected;
        }
        return;
    }
    switch (snapshot_.classified_mode)
    {
    case guidance::doctrine::TacticalMode::Obfm:
        snapshot_.classified_route_implementation =
            StaticDoctrineRouteImplementation::Implemented;
        BuildObfm(input, obfm_input, false, output, status);
        return;
    case guidance::doctrine::TacticalMode::Habfm:
        snapshot_.classified_route_implementation =
            StaticDoctrineRouteImplementation::Implemented;
        BuildHabfm(input, projection_port, output, status);
        return;
    case guidance::doctrine::TacticalMode::Dbfm:
        snapshot_.classified_route_implementation =
            StaticDoctrineRouteImplementation::Implemented;
        break;
    default:
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::TacticalModeOutOfRange,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    frame_evidence_provider_.Build(
        input.frame,
        snapshot_.dbfm_frame_evidence,
        snapshot_.dbfm_frame_evidence_status);
    if (snapshot_.dbfm_frame_evidence_status
        != HabfmFrameEvidenceStatus::Built)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::
                DbfmFrameEvidenceUnavailable,
            StatusCode::NonFiniteInput,
            status);
        return;
    }
    const DbfmCornerSpeedControlEvidence speed_evidence =
        BuildSpeedEvidence(snapshot_.dbfm_frame_evidence);

    snapshot_.dbfm_break_capability.frame_identity =
        input.frame.frame_identity;
    const bool dbfm_physical_capability_available =
        snapshot_.gun_evidence.valid
        && snapshot_.gun_evidence.capability_admitted
        && snapshot_.gun_evidence.capability.n_channel_trusted
        && snapshot_.gun_evidence.capability.n_inst_g.has_value
        && std::isfinite(
            snapshot_.gun_evidence.capability.n_inst_g.value)
        && snapshot_.gun_evidence.capability.n_inst_g.value > 1.0;
    snapshot_.dbfm_break_capability.admitted =
        dbfm_physical_capability_available;
    snapshot_.dbfm_break_capability.n_max_available =
        dbfm_physical_capability_available;
    snapshot_.dbfm_break_capability.n_max_g =
        dbfm_physical_capability_available
            ? snapshot_.gun_evidence.capability.n_inst_g.value
            : 0.0;
    snapshot_.dbfm_break_capability.physical_authority =
        dbfm_physical_capability_available;
    snapshot_.dbfm_break_capability.fixed_command_bound = false;

    Status break_margin_status{};
    guidance::dbfm::EvaluateDbfmBreakMargin(
        input.frame,
        true,
        false,
        guidance::dbfm::DbfmBreakMarginProductionActivation,
        snapshot_.dbfm_break_capability,
        snapshot_.dbfm_break_margin,
        break_margin_status);
    snapshot_.dbfm_break_margin_status_code = break_margin_status.code;

    bool break_ready = false;
    ControlIntent raw_break{};
    if (break_margin_status.ok()
        && snapshot_.dbfm_break_margin.break_selected)
    {
        Status break_reference_status{};
        BuildHorizontalBreakReference(
            input.frame,
            safety_gun_owner_.NextGunSideSign(),
            snapshot_.dbfm_break_reference,
            break_reference_status);
        snapshot_.dbfm_break_reference_status_code =
            break_reference_status.code;
        if (break_reference_status.ok())
        {
            Status break_intent_status{};
            break_ready = BuildDbfmBreakIntent(
                input.frame,
                snapshot_.dbfm_break_reference,
                raw_break,
                break_intent_status);
            if (!break_intent_status.ok())
            {
                snapshot_.dbfm_break_reference_status_code =
                    break_intent_status.code;
                break_ready = false;
                raw_break.Clear();
            }
        }
    }

    ControlIntent raw_altitude_separated{};
    bool altitude_separated_ready = false;
    Status altitude_separated_status{};
    guidance::dbfm::EvaluateDbfmAltitudeSeparated(
        input.frame,
        true,
        guidance::dbfm::DbfmAltitudeSeparatedProductionActivation,
        snapshot_.dbfm_frame_evidence.own_sustained_corner_interval,
        snapshot_.dbfm_altitude_separated,
        altitude_separated_status);
    snapshot_.dbfm_altitude_separated_status_code =
        altitude_separated_status.code;
    if (altitude_separated_status.ok()
        && snapshot_.dbfm_altitude_separated.candidate_available)
    {
        Status altitude_intent_status{};
        altitude_separated_ready = BuildDbfmAltitudeSeparatedIntent(
            input.frame,
            snapshot_.dbfm_altitude_separated,
            raw_altitude_separated,
            altitude_intent_status);
        if (altitude_separated_ready)
        {
            guidance::dbfm::DbfmEscapeEnergyBaseReference base{};
            base.valid = true;
            base.frame_identity = raw_altitude_separated.frame_identity;
            base.aim_point_ned_m = raw_altitude_separated.aim_point_m;
            base.desired_speed_mps =
                raw_altitude_separated.desired_speed_mps;
            base.desired_speed_rate_mps2 =
                raw_altitude_separated.desired_speed_rate_mps2;
            Status escape_energy_status{};
            guidance::dbfm::ApplyDbfmEscapeEnergy(
                input.frame,
                guidance::dbfm::DbfmEscapeEnergyBehavior::AltitudeSeparated,
                guidance::dbfm::DbfmEscapeEnergyProductionActivation,
                base,
                snapshot_.gun_evidence.capability,
                input.current_safety,
                snapshot_.dbfm_escape_energy,
                escape_energy_status);
            snapshot_.dbfm_escape_energy_status_code =
                escape_energy_status.code;
            if (escape_energy_status.ok()
                && snapshot_.dbfm_escape_energy.candidate_available)
            {
                ControlIntent shaped = raw_altitude_separated;
                shaped.aim_point_m =
                    snapshot_.dbfm_escape_energy.aim_point_ned_m;
                shaped.desired_speed_mps =
                    snapshot_.dbfm_escape_energy.desired_speed_mps;
                shaped.desired_speed_rate_mps2 =
                    snapshot_.dbfm_escape_energy.desired_speed_rate_mps2;
                Status shaped_status{};
                shaped.Validate(shaped_status);
                if (shaped_status.ok())
                {
                    raw_altitude_separated = shaped;
                }
                else
                {
                    // Energy shaping is optional.  Preserve the already-valid
                    // horizontal-away candidate on any shaping-only failure.
                    snapshot_.dbfm_escape_energy_status_code =
                        shaped_status.code;
                }
            }
        }
        else
        {
            snapshot_.dbfm_altitude_separated_status_code =
                altitude_intent_status.code;
        }
    }

    StaticDoctrineDbfmSelectionContext dbfm_context{};
    dbfm_context.frame = &input.frame;
    dbfm_context.snapshot = &snapshot_;
    dbfm_context.break_candidate = raw_break;
    dbfm_context.break_candidate_available = break_ready;
    dbfm_context.altitude_separated_candidate = raw_altitude_separated;
    dbfm_context.altitude_separated_candidate_available =
        altitude_separated_ready;
    const StaticBtChildTable<3U> dbfm_children{{{
        StaticBtChildDescriptor{StaticDbfmBreakNodeId, 1U, 0U,
                                BtLifecycleOwnerIdNone},
        StaticBtChildDescriptor{StaticDbfmAltitudeSeparatedNodeId, 1U, 1U,
                                BtLifecycleOwnerIdNone},
        StaticBtChildDescriptor{StaticDbfmTerminalNodeId, 1U, 2U,
                                BtLifecycleOwnerIdNone}}}, 3U};
    const StaticBtCallbackTable<StaticDoctrineDbfmSelectionContext, 3U>
        dbfm_callbacks{{{
            &EvaluateStaticDbfmBreak,
            &EvaluateStaticDbfmAltitudeSeparated,
            &EvaluateStaticDbfmHardTurn}}, 3U};
    const StaticBtCandidateSelection dbfm_selection =
        EvaluateStaticBtCandidateChildren(
            dbfm_context,
            dbfm_children,
            dbfm_callbacks,
            nullptr,
            input.frame.frame_identity,
            input.frame.frame_identity.frame_index);
    snapshot_.dbfm_selector_result = dbfm_selection.result;
    if (!dbfm_selection.candidate_available)
    {
        if (IsBtReturnCodeError(dbfm_selection.result.code))
        {
            const bool geometry_fault =
                snapshot_.dbfm_geometry_status_code != StatusCode::Ok;
            const StatusCode failure_status = geometry_fault
                ? snapshot_.dbfm_geometry_status_code
                : (snapshot_.raw_candidate_status_code != StatusCode::Ok
                    ? snapshot_.raw_candidate_status_code
                    : StatusCode::InvalidConfiguration);
            RecordTerminal(
                snapshot_,
                StaticDoctrineCandidateDisposition::Fault,
                geometry_fault
                    ? StaticDoctrineCommandProviderReason::
                        DbfmGeometryObservationFault
                    : StaticDoctrineCommandProviderReason::
                        DbfmHardTurnBuildFault,
                failure_status,
                status);
        }
        else
        {
            RecordTerminal(
                snapshot_,
                StaticDoctrineCandidateDisposition::NotApplicable,
                snapshot_.dbfm_geometry.valid
                    ? StaticDoctrineCommandProviderReason::
                        DbfmHardTurnNotApplicable
                    : StaticDoctrineCommandProviderReason::
                        DbfmGeometryNotApplicable,
                StatusCode::InvalidConfiguration,
                status);
        }
        return;
    }
    snapshot_.raw_candidate = dbfm_selection.candidate;
    const bool break_selected = snapshot_.raw_candidate.writer_id
        == ControlIntentWriterDbfmBreak;
    const bool altitude_separated_selected =
        snapshot_.raw_candidate.writer_id
            == ControlIntentWriterDbfmAltitudeSeparated;
    const bool hard_turn_selected = snapshot_.raw_candidate.writer_id
        == ControlIntentWriterDbfmHardTurn;
    if (!(break_selected || altitude_separated_selected || hard_turn_selected)
        || !SameControlFrameIdentity(
            snapshot_.raw_candidate.frame_identity,
            input.frame.frame_identity))
    {
        snapshot_.raw_candidate.Clear();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::DbfmWriterContractFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }
    snapshot_.candidate_count = 1U;

    Status selected_status{};
    ApplyDbfmDefenseSpeed(
        snapshot_.raw_candidate,
        speed_evidence,
        snapshot_.selected_candidate,
        selected_status);
    snapshot_.selected_candidate_status_code = selected_status.code;
    if (selected_status.code != StatusCode::Ok)
    {
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::DbfmDefenseSpeedFault,
            selected_status.code,
            status);
        return;
    }
    if (break_selected)
    {
        DbfmBreakLoadKinematics own{};
        own.position_ned_m = input.frame.own.position_ned_m;
        own.velocity_body_mps = input.frame.own.velocity_body_mps;
        own.velocity_world_ned_mps = input.frame.own.velocity_ned_mps;
        own.rpy_rad = input.frame.own.rpy_rad;
        DbfmBreakLoadEvidence evidence{};
        evidence.capability_admitted =
            snapshot_.dbfm_break_capability.admitted;
        if (evidence.capability_admitted)
        {
            evidence.instantaneous_load_limit_g.has_value = true;
            evidence.instantaneous_load_limit_g.value =
                snapshot_.dbfm_break_capability.n_max_g;
        }
        const DbfmBreakLoadConfig config{true, true, false};
        ControlIntent break_load_candidate{};
        Status break_load_status{};
        ApplyDbfmBreakLoad(
            snapshot_.selected_candidate,
            own,
            evidence,
            config,
            break_load_candidate,
            break_load_status);
        snapshot_.dbfm_break_load_status_code = break_load_status.code;
        if (break_load_status.ok())
        {
            snapshot_.selected_candidate = break_load_candidate;
            snapshot_.dbfm_break_load_legacy_fallback =
                break_load_candidate.route_kind
                    == ControlRouteKind::AimPoint;
        }
        else
        {
            // The BREAK aim/speed reference remains a valid tactical command
            // when the optional load-vector representation loses direction
            // in a collinear geometry.  Preserve it instead of creating a
            // command gap or substituting an unrelated maneuver.
            snapshot_.dbfm_break_load_legacy_fallback = true;
        }
    }
    if (snapshot_.selected_candidate.writer_id
            != snapshot_.raw_candidate.writer_id
        || !SameControlFrameIdentity(
            snapshot_.selected_candidate.frame_identity,
            input.frame.frame_identity))
    {
        snapshot_.selected_candidate.Clear();
        RecordTerminal(
            snapshot_,
            StaticDoctrineCandidateDisposition::Fault,
            StaticDoctrineCommandProviderReason::DbfmWriterContractFault,
            StatusCode::InvalidConfiguration,
            status);
        return;
    }

    snapshot_.selected_candidate_count = 1U;
    snapshot_.candidate_disposition =
        StaticDoctrineCandidateDisposition::Selected;
    snapshot_.reason = break_selected
        ? StaticDoctrineCommandProviderReason::DbfmBreakSelected
        : (altitude_separated_selected
            ? StaticDoctrineCommandProviderReason::
                DbfmAltitudeSeparatedSelected
            : StaticDoctrineCommandProviderReason::DbfmHardTurnSelected);
    snapshot_.provider_status_code = StatusCode::Ok;
    output = snapshot_.selected_candidate;
    StagePreparedTransaction(
        StaticDoctrinePreparedOwner::Dbfm,
        output.frame_identity,
        output.writer_id);
    status = Status{};
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
