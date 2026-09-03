#include "LadyLuck/guidance/dbfm/DbfmOrdinaryMode.hpp"

#include "LadyLuck/guidance/dbfm/DbfmHardTurnControlIntent.hpp"

namespace
{

bool ValidAdmission(
    const LadyLuck::guidance::dbfm::DbfmOrdinaryBranchAdmission& value)
    noexcept
{
    using LadyLuck::guidance::dbfm::DbfmOrdinaryUnavailability;
    if (value.evidence_available)
    {
        return value.unavailability == DbfmOrdinaryUnavailability::None;
    }
    return !value.selected
        && value.unavailability != DbfmOrdinaryUnavailability::None;
}

LadyLuck::guidance::dbfm::DbfmOrdinaryBranch SelectBranch(
    const LadyLuck::guidance::dbfm::DbfmOrdinaryAdmissionReceipt& receipt)
    noexcept
{
    using LadyLuck::guidance::dbfm::DbfmOrdinaryBranch;
    if (receipt.break_branch.selected)
    {
        return DbfmOrdinaryBranch::Break;
    }
    if (receipt.altitude_separated_branch.selected)
    {
        return DbfmOrdinaryBranch::AltitudeSeparated;
    }
    if (receipt.escape_branch.selected)
    {
        return DbfmOrdinaryBranch::Escape;
    }
    if (receipt.extend_branch.selected)
    {
        return DbfmOrdinaryBranch::Extend;
    }
    return DbfmOrdinaryBranch::HardTurn;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

DbfmOrdinaryModeWriter::DbfmOrdinaryModeWriter() noexcept
{
    Reset();
}

void DbfmOrdinaryModeWriter::Reset() noexcept
{
    active_ = false;
    active_branch_ = DbfmOrdinaryBranch::None;
    last_frame_identity_ = ControlFrameIdentity{};
}

void DbfmOrdinaryModeWriter::ObserveCurrentProductionAdmissions(
    const DogfightGeometryFrame& frame,
    DbfmOrdinaryAdmissionReceipt& output,
    Status& status) const noexcept
{
    output = DbfmOrdinaryAdmissionReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.valid = true;
    output.break_branch.unavailability =
        DbfmOrdinaryUnavailability::RootGunOwner;
    output.altitude_separated_branch.unavailability =
        DbfmOrdinaryUnavailability::ProductionConfigDisabled;
    output.escape_branch.unavailability =
        DbfmOrdinaryUnavailability::ProductionConfigDisabled;
    output.extend_branch.unavailability =
        DbfmOrdinaryUnavailability::ProductionConfigDisabled;
}

void DbfmOrdinaryModeWriter::Build(
    const DogfightGeometryFrame& frame,
    const DoctrineModeId selected_mode,
    const DbfmOrdinaryAdmissionReceipt& admissions,
    const DbfmOrdinarySpeedReceipt& speed_receipt,
    const DbfmHardTurnGeometryReceipt& hard_turn_geometry,
    ControlIntent& output,
    DbfmOrdinarySelectionReceipt& selection,
    Status& status) noexcept
{
    output.Clear();
    selection = DbfmOrdinarySelectionReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        Reset();
        status.code = StatusCode::InvalidArgument;
        return;
    }

    selection.frame_identity = frame.frame_identity;
    selection.valid = true;
    if (selected_mode != DoctrineModeId::Dbfm)
    {
        // Python mode_state resets state and the root Sequence publishes no
        // DBFM command.  Never substitute the previous command or a Hold.
        Reset();
        return;
    }
    selection.mode_selected = true;

    if (!admissions.valid
        || !speed_receipt.valid
        || !SameControlFrameIdentity(
            admissions.frame_identity,
            frame.frame_identity)
        || !SameControlFrameIdentity(
            speed_receipt.frame_identity,
            frame.frame_identity)
        || !ValidAdmission(admissions.break_branch)
        || !ValidAdmission(admissions.altitude_separated_branch)
        || !ValidAdmission(admissions.escape_branch)
        || !ValidAdmission(admissions.extend_branch))
    {
        active_ = false;
        active_branch_ = DbfmOrdinaryBranch::None;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    const DbfmOrdinaryBranch selected = SelectBranch(admissions);
    selection.selected_branch = selected;
    selection.selection_count = 1U;
    if (selected != DbfmOrdinaryBranch::HardTurn)
    {
        // Priority is preserved, but this bounded production writer has no
        // authority to fabricate the withheld branch command.  Do not fall
        // through to HARD_TURN after a higher branch is actually admitted.
        active_ = false;
        active_branch_ = DbfmOrdinaryBranch::None;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ControlIntent raw{};
    bool hard_turn_ready = false;
    BuildDbfmHardTurnCommand(
        frame,
        hard_turn_geometry,
        raw,
        hard_turn_ready,
        status);
    if (status.code != StatusCode::Ok || !hard_turn_ready)
    {
        active_ = false;
        active_branch_ = DbfmOrdinaryBranch::None;
        if (status.code == StatusCode::Ok)
        {
            // A visible BT Condition must have rejected a normal unavailable
            // geometry before selecting this writer.  Reaching the Task with
            // command_ready=false is therefore a wiring contradiction.
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }

    ControlIntent shaped{};
    ApplyDbfmDefenseSpeed(
        raw,
        speed_receipt.evidence,
        shaped,
        status);
    if (status.code != StatusCode::Ok)
    {
        active_ = false;
        active_branch_ = DbfmOrdinaryBranch::None;
        return;
    }

    active_ = true;
    active_branch_ = DbfmOrdinaryBranch::HardTurn;
    last_frame_identity_ = frame.frame_identity;
    output = shaped;
    selection.command_ready = true;
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
