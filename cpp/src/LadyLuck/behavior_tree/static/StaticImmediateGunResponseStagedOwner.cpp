#include "LadyLuck/behavior_tree/static/StaticImmediateGunResponseStagedOwner.hpp"
#include "generated/LadyLuckV2TopologyCapacity.hpp"

#include <cmath>

namespace
{

constexpr std::uint32_t ImmediateGunSelectorElementIndex = 9U;
constexpr std::uint32_t ImmediateGunHighGElementIndex = 10U;
constexpr std::uint32_t ImmediateGunSnapshotElementIndex = 11U;
constexpr std::uint32_t ImmediateGunHardTurnElementIndex = 12U;
constexpr std::uint32_t ImmediateGunBaseBreakElementIndex = 13U;

constexpr LadyLuck::behavior_tree::static_bt::BtNodeId NodeIdAt(
    const std::uint32_t element_index) noexcept
{
    return LadyLuck::behavior_tree::generated::
        LadyLuckV2XmlNodes[element_index].static_node_id;
}

constexpr LadyLuck::behavior_tree::static_bt::BtStageId StageIdAt(
    const std::uint32_t element_index) noexcept
{
    return LadyLuck::behavior_tree::generated::
        LadyLuckV2XmlNodes[element_index].static_stage_id;
}

void RecordSelectorResult(
    LadyLuck::behavior_tree::static_bt::
        StaticImmediateGunResponsePreparedReceipt& snapshot,
    const std::uint32_t element_index,
    const LadyLuck::behavior_tree::static_bt::BtReturnCode code,
    const LadyLuck::behavior_tree::static_bt::BtReasonId reason) noexcept
{
    snapshot.selector_result =
        LadyLuck::behavior_tree::static_bt::MakeBtTickResult(
            code,
            NodeIdAt(element_index),
            StageIdAt(element_index),
            snapshot.frame_identity.frame_index,
            reason);
}

static_assert(NodeIdAt(ImmediateGunSelectorElementIndex) == 10U,
              "ImmediateGun selector identity changed");
static_assert(NodeIdAt(ImmediateGunHighGElementIndex) == 11U,
              "ImmediateGun High-G identity changed");
static_assert(NodeIdAt(ImmediateGunSnapshotElementIndex) == 12U,
              "ImmediateGun Snapshot identity changed");
static_assert(NodeIdAt(ImmediateGunHardTurnElementIndex) == 13U,
              "ImmediateGun HardTurn identity changed");
static_assert(NodeIdAt(ImmediateGunBaseBreakElementIndex) == 14U,
              "ImmediateGun base BREAK identity changed");

bool FinalWriterAllowed(const std::uint32_t writer_id) noexcept
{
    return writer_id
            == LadyLuck::ControlIntentWriterGunDefenseHorizontalBreak
        || writer_id == LadyLuck::ControlIntentWriterG4HighGBarrel
        || writer_id
            == LadyLuck::ControlIntentWriterOfficialGunSnapshotPlaneChange
        || writer_id == LadyLuck::ControlIntentWriterDbfmHardTurn;
}

bool ValidateBaseBreak(
    const LadyLuck::behavior_tree::static_bt::
        StaticImmediateGunResponseStagedInput& input,
    LadyLuck::Status& status) noexcept
{
    status = LadyLuck::Status{};
    const auto& tactical = input.tactical_input;
    input.base_break.Validate(status);
    if (!status.sample_valid())
    {
        return false;
    }
    if (!tactical.valid
        || !LadyLuck::IsValidControlFrameIdentity(
            tactical.frame.frame_identity)
        || !input.writer2_same_frame_admitted
        || !LadyLuck::IsValidControlFrameIdentity(
            input.safety_gun_frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            tactical.frame.frame_identity,
            input.safety_gun_frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            tactical.frame.frame_identity,
            input.base_break.frame_identity)
        || (input.base_break.route_kind
                != LadyLuck::ControlRouteKind::AimPoint
            && input.base_break.route_kind
                != LadyLuck::ControlRouteKind::DirectNedAcceleration)
        || input.base_break.behavior_id
            != LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak
        || input.base_break.mode_id != LadyLuck::DoctrineModeId::Dbfm
        || input.base_break.writer_id
            != LadyLuck::ControlIntentWriterGunDefenseHorizontalBreak
        || (input.entry_side_sign_valid
            && input.entry_side_sign != -1
            && input.entry_side_sign != 1)
        || (input.root_gun_evidence.valid
            && (!LadyLuck::IsValidControlFrameIdentity(
                    input.root_gun_evidence.frame_identity)
                || !LadyLuck::SameControlFrameIdentity(
                    tactical.frame.frame_identity,
                    input.root_gun_evidence.frame_identity))))
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return false;
    }
    return true;
}

bool ValidateG4Candidate(
    const LadyLuck::ControlIntent& candidate,
    const LadyLuck::ControlFrameIdentity& frame_identity,
    const std::uint32_t writer_id,
    LadyLuck::Status& status) noexcept
{
    candidate.Validate(status);
    if (!status.sample_valid())
    {
        return false;
    }
    if (!LadyLuck::SameControlFrameIdentity(
            candidate.frame_identity,
            frame_identity)
        || candidate.writer_id != writer_id
        || candidate.mode_id != LadyLuck::DoctrineModeId::Dbfm)
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return false;
    }
    if (writer_id == LadyLuck::ControlIntentWriterG4HighGBarrel)
    {
        const bool behavior_valid = candidate.behavior_id
            == LadyLuck::DoctrineBehaviorId::G4WezShortestExit;
        if (!behavior_valid)
        {
            status.code = LadyLuck::StatusCode::InvalidConfiguration;
            return false;
        }
    }
    return true;
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

StaticImmediateGunResponseStagedOwner::
    StaticImmediateGunResponseStagedOwner() noexcept
{
    Reset();
}

void StaticImmediateGunResponseStagedOwner::Reset() noexcept
{
    committed_high_g_owner_.Reset();
    committed_snapshot_owner_.Reset();
    committed_phase_graded_owner_.Reset();

    staged_high_g_owner_ = committed_high_g_owner_;
    staged_snapshot_owner_ = committed_snapshot_owner_;
    staged_phase_graded_owner_ = committed_phase_graded_owner_;

    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
    staged_generation_ = 0U;
    ++generation_;
    snapshot_ = StaticImmediateGunResponsePreparedReceipt{};
}

void StaticImmediateGunResponseStagedOwner::DiscardStagedState() noexcept
{
    staged_high_g_owner_ = committed_high_g_owner_;
    staged_snapshot_owner_ = committed_snapshot_owner_;
    staged_phase_graded_owner_ = committed_phase_graded_owner_;
    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
    staged_generation_ = 0U;
}

void StaticImmediateGunResponseStagedOwner::RetainBaseBreak(
    const StaticImmediateGunResponseReason reason,
    const StatusCode diagnostic_status_code,
    const ControlIntent& base_break,
    ControlIntent& output,
    StaticImmediateGunResponsePreparedReceipt& receipt,
    Status& status) noexcept
{
    output = base_break;
    staged_writer_id_ = ControlIntentWriterGunDefenseHorizontalBreak;
    snapshot_.disposition =
        StaticImmediateGunResponseDisposition::BaseBreakRetained;
    snapshot_.reason = reason;
    snapshot_.prepared_writer_id = staged_writer_id_;
    snapshot_.candidate_count = 1U;
    snapshot_.state_staged = staged_ready_;
    snapshot_.state_committed = false;
    snapshot_.state_aborted = false;
    snapshot_.optional_response_fault_contained =
        diagnostic_status_code != StatusCode::Ok;
    snapshot_.diagnostic_status_code = diagnostic_status_code;
    RecordSelectorResult(
        snapshot_,
        ImmediateGunBaseBreakElementIndex,
        BtReturnCode::Selected,
        static_cast<BtReasonId>(reason));
    receipt = snapshot_;
    status = Status{};
}

void StaticImmediateGunResponseStagedOwner::RejectInput(
    const StaticImmediateGunResponseReason reason,
    const StatusCode code,
    ControlIntent& output,
    StaticImmediateGunResponsePreparedReceipt& receipt,
    Status& status) noexcept
{
    output.Clear();
    snapshot_.disposition =
        StaticImmediateGunResponseDisposition::InputContractFault;
    snapshot_.reason = reason;
    snapshot_.prepared_writer_id = ControlIntentWriterNone;
    snapshot_.candidate_count = 0U;
    snapshot_.state_staged = false;
    snapshot_.state_committed = false;
    snapshot_.state_aborted = staged_ready_;
    snapshot_.diagnostic_status_code = code;
    RecordSelectorResult(
        snapshot_,
        ImmediateGunSelectorElementIndex,
        BtReturnCode::InvalidInput,
        static_cast<BtReasonId>(reason));
    DiscardStagedState();
    receipt = snapshot_;
    status.code = code;
}

void StaticImmediateGunResponseStagedOwner::Prepare(
    const StaticImmediateGunResponseStagedInput& input,
    ControlIntent& output,
    StaticImmediateGunResponsePreparedReceipt& receipt,
    Status& status) noexcept
{
    output.Clear();
    receipt = StaticImmediateGunResponsePreparedReceipt{};
    status = Status{};
    DiscardStagedState();
    snapshot_ = StaticImmediateGunResponsePreparedReceipt{};
    snapshot_.prepare_attempted = true;
    snapshot_.frame_identity = input.tactical_input.frame.frame_identity;

    Status base_status{};
    if (!ValidateBaseBreak(input, base_status))
    {
        RejectInput(
            StaticImmediateGunResponseReason::BaseBreakContractFault,
            base_status.code,
            output,
            receipt,
            status);
        return;
    }
    snapshot_.base_writer2_same_frame_admitted = true;

    staged_high_g_owner_ = committed_high_g_owner_;
    staged_snapshot_owner_ = committed_snapshot_owner_;
    staged_phase_graded_owner_ = committed_phase_graded_owner_;
    staged_ready_ = true;
    staged_frame_identity_ = input.tactical_input.frame.frame_identity;
    staged_writer_id_ = ControlIntentWriterGunDefenseHorizontalBreak;
    staged_generation_ = generation_;
    snapshot_.captured_generation = staged_generation_;
    snapshot_.state_staged = true;

    // Writer 14 is one BT leaf and is evaluated exactly once. It consumes the
    // already-admitted same-frame Gun owner, official WEZ geometry, current
    // aircraft state, and the downstream load envelope. Retired G13/corner
    // evidence cannot create a second hidden writer-14 candidate.
    guidance::g4::HighGBarrelExactEvidence direct_wez_evidence{};
    Status direct_wez_status{};
    staged_high_g_owner_.Observe(
        input.tactical_input,
        input.base_break,
        true,
        direct_wez_evidence,
        snapshot_.g4_high_g_selection,
        direct_wez_status);
    if (!direct_wez_status.ok()
        || !snapshot_.g4_high_g_selection.valid)
    {
        RetainBaseBreak(
            StaticImmediateGunResponseReason::
                OptionalResponseFaultContained,
            direct_wez_status.ok()
                ? StatusCode::InvalidConfiguration
                : direct_wez_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    if (snapshot_.g4_high_g_selection.decision
        == guidance::g4::HighGBarrelDecision::ReleasePassthrough)
    {
        guidance::g4::HighGBarrelOwnerSnapshot release{};
        staged_high_g_owner_.BuildReleaseCommit(
            snapshot_.g4_high_g_selection,
            release,
            direct_wez_status);
        if (direct_wez_status.ok())
        {
            staged_high_g_owner_.CommitPublished(
                release,
                direct_wez_status);
        }
        if (!direct_wez_status.ok())
        {
            RetainBaseBreak(
                StaticImmediateGunResponseReason::
                    OptionalResponseFaultContained,
                direct_wez_status.code,
                input.base_break,
                output,
                receipt,
                status);
            return;
        }
        snapshot_.g4_high_g_release_staged = true;
    }
    else if (snapshot_.g4_high_g_selection.decision
            == guidance::g4::HighGBarrelDecision::Underneath
        || snapshot_.g4_high_g_selection.decision
            == guidance::g4::HighGBarrelDecision::OverTheTop)
    {
        const guidance::g4::HighGBarrelVariant variant =
            snapshot_.g4_high_g_selection.selected_variant;
        ControlIntent candidate{};
        guidance::g4::HighGBarrelOwnerSnapshot commit{};
        staged_high_g_owner_.BuildCandidate(
            variant,
            input.tactical_input,
            input.base_break,
            direct_wez_evidence,
            snapshot_.g4_high_g_selection,
            candidate,
            commit,
            snapshot_.g4_high_g_task,
            direct_wez_status);
        if (!direct_wez_status.ok() || !snapshot_.g4_high_g_task.valid)
        {
            RetainBaseBreak(
                StaticImmediateGunResponseReason::
                    OptionalResponseFaultContained,
                direct_wez_status.ok()
                    ? StatusCode::InvalidConfiguration
                    : direct_wez_status.code,
                input.base_break,
                output,
                receipt,
                status);
            return;
        }
        if (snapshot_.g4_high_g_task.root_passthrough_required)
        {
            staged_high_g_owner_.CommitPublished(
                commit,
                direct_wez_status);
            if (!direct_wez_status.ok())
            {
                RetainBaseBreak(
                    StaticImmediateGunResponseReason::
                        OptionalResponseFaultContained,
                    direct_wez_status.code,
                    input.base_break,
                    output,
                    receipt,
                    status);
                return;
            }
        }
        else if (snapshot_.g4_high_g_task.candidate_available)
        {
            Status candidate_status{};
            if (!ValidateG4Candidate(
                    candidate,
                    input.tactical_input.frame.frame_identity,
                    ControlIntentWriterG4HighGBarrel,
                    candidate_status))
            {
                RetainBaseBreak(
                    StaticImmediateGunResponseReason::
                        OptionalResponseFaultContained,
                    candidate_status.code,
                    input.base_break,
                    output,
                    receipt,
                    status);
                return;
            }
            staged_high_g_owner_.CommitPublished(
                commit,
                direct_wez_status);
            if (!direct_wez_status.ok())
            {
                RetainBaseBreak(
                    StaticImmediateGunResponseReason::
                        OptionalResponseFaultContained,
                    direct_wez_status.code,
                    input.base_break,
                    output,
                    receipt,
                    status);
                return;
            }
            output = candidate;
            staged_writer_id_ = ControlIntentWriterG4HighGBarrel;
            snapshot_.disposition =
                StaticImmediateGunResponseDisposition::G4HighGPrepared;
            snapshot_.reason =
                StaticImmediateGunResponseReason::G4HighGSelected;
            snapshot_.prepared_writer_id = staged_writer_id_;
            snapshot_.candidate_count = 1U;
            snapshot_.diagnostic_status_code = StatusCode::Ok;
            RecordSelectorResult(
                snapshot_,
                ImmediateGunHighGElementIndex,
                BtReturnCode::Selected,
                static_cast<BtReasonId>(snapshot_.reason));
            receipt = snapshot_;
            status = Status{};
            return;
        }
        else
        {
            RetainBaseBreak(
                StaticImmediateGunResponseReason::
                    OptionalResponseFaultContained,
                StatusCode::InvalidConfiguration,
                input.base_break,
                output,
                receipt,
                status);
            return;
        }
    }

    snapshot_.snapshot_attempted = true;
    StaticGunSnapshotStagedInput snapshot_input{};
    snapshot_input.frame = input.tactical_input.frame;
    snapshot_input.current_envelope = input.tactical_input.current_envelope;
    snapshot_input.base_break = input.base_break;
    ControlIntent snapshot_candidate{};
    Status snapshot_status{};
    staged_snapshot_owner_.Prepare(
        snapshot_input,
        snapshot_candidate,
        snapshot_.snapshot,
        snapshot_status);
    snapshot_.snapshot_transaction_staged = snapshot_status.ok()
        && snapshot_.snapshot.state_staged;
    if (!snapshot_status.ok())
    {
        staged_snapshot_owner_.AbortPrepared();
        snapshot_.snapshot_transaction_staged = false;
        RetainBaseBreak(
            StaticImmediateGunResponseReason::
                OptionalResponseFaultContained,
            snapshot_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }
    if (snapshot_candidate.writer_id
        == ControlIntentWriterOfficialGunSnapshotPlaneChange)
    {
        output = snapshot_candidate;
        staged_writer_id_ =
            ControlIntentWriterOfficialGunSnapshotPlaneChange;
        snapshot_.disposition =
            StaticImmediateGunResponseDisposition::SnapshotPrepared;
        snapshot_.reason =
            StaticImmediateGunResponseReason::SnapshotSelected;
        snapshot_.prepared_writer_id = staged_writer_id_;
        snapshot_.candidate_count = 1U;
        snapshot_.diagnostic_status_code = StatusCode::Ok;
        RecordSelectorResult(
            snapshot_,
            ImmediateGunSnapshotElementIndex,
            BtReturnCode::Selected,
            static_cast<BtReasonId>(snapshot_.reason));
        receipt = snapshot_;
        status = Status{};
        return;
    }
    if (snapshot_candidate.writer_id
            != ControlIntentWriterGunDefenseHorizontalBreak
        || snapshot_.snapshot.disposition
            == StaticGunSnapshotDisposition::InputContractFault
        || snapshot_.snapshot.disposition
            == StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault)
    {
        if (snapshot_candidate.writer_id
                != ControlIntentWriterGunDefenseHorizontalBreak
            && snapshot_candidate.writer_id
                != ControlIntentWriterOfficialGunSnapshotPlaneChange)
        {
            staged_snapshot_owner_.AbortPrepared();
            snapshot_.snapshot_transaction_staged = false;
        }
        RetainBaseBreak(
            StaticImmediateGunResponseReason::
                OptionalResponseFaultContained,
            snapshot_.snapshot.diagnostic_status_code == StatusCode::Ok
                ? StatusCode::InvalidConfiguration
                : snapshot_.snapshot.diagnostic_status_code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    snapshot_.phase_graded_attempted = true;
    snapshot_.phase_gun_episode.active = true;
    snapshot_.phase_gun_episode.side_sign = input.entry_side_sign_valid
        ? input.entry_side_sign
        : 1;
    snapshot_.phase_gun_episode.toward_side_candidate_held =
        input.entry_side_sign_valid;

    if (!input.entry_side_sign_valid)
    {
        RetainBaseBreak(
            StaticImmediateGunResponseReason::BaseBreakNotApplicable,
            StatusCode::Ok,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    ControlIntent phase_candidate{};
    Status phase_status{};
    staged_phase_graded_owner_.Evaluate(
        input.tactical_input.frame,
        true,
        snapshot_.phase_gun_episode,
        input.base_break,
        snapshot_.phase_graded,
        phase_candidate,
        phase_status);
    snapshot_.phase_graded_status_code = phase_status.code;
    if (!phase_status.ok())
    {
        RetainBaseBreak(
            StaticImmediateGunResponseReason::
                OptionalResponseFaultContained,
            phase_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }
    if (!snapshot_.phase_graded.replacement_available)
    {
        RetainBaseBreak(
            StaticImmediateGunResponseReason::BaseBreakNotApplicable,
            StatusCode::Ok,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    Status phase_candidate_status{};
    phase_candidate.Validate(phase_candidate_status);
    const bool hard_turn_selected = snapshot_.phase_graded.selected_branch
            == guidance::dbfm::DbfmPhaseGradedResponseBranch::HardTurn
        && phase_candidate.writer_id == ControlIntentWriterDbfmHardTurn;
    if (!phase_candidate_status.sample_valid()
        || !SameControlFrameIdentity(
            phase_candidate.frame_identity,
            input.tactical_input.frame.frame_identity)
        || !hard_turn_selected)
    {
        RetainBaseBreak(
            StaticImmediateGunResponseReason::
                OptionalResponseFaultContained,
            phase_candidate_status.sample_valid()
                ? StatusCode::InvalidConfiguration
                : phase_candidate_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    output = phase_candidate;
    staged_writer_id_ = phase_candidate.writer_id;
    snapshot_.disposition = StaticImmediateGunResponseDisposition::
        PhaseGradedHardTurnPrepared;
    snapshot_.reason =
        StaticImmediateGunResponseReason::PhaseGradedHardTurnSelected;
    snapshot_.prepared_writer_id = staged_writer_id_;
    snapshot_.candidate_count = 1U;
    snapshot_.diagnostic_status_code = StatusCode::Ok;
    RecordSelectorResult(
        snapshot_,
        ImmediateGunHardTurnElementIndex,
        BtReturnCode::Selected,
        static_cast<BtReasonId>(snapshot_.reason));
    receipt = snapshot_;
    status = Status{};
}

void StaticImmediateGunResponseStagedOwner::ValidatePrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) const noexcept
{
    status = Status{};
    if (!staged_ready_
        || !snapshot_.state_staged
        || snapshot_.state_committed
        || snapshot_.state_aborted
        || snapshot_.candidate_count != 1U
        || staged_generation_ != generation_
        || snapshot_.captured_generation != staged_generation_
        || !IsValidControlFrameIdentity(frame_identity)
        || !SameControlFrameIdentity(
            staged_frame_identity_,
            frame_identity)
        || !SameControlFrameIdentity(
            snapshot_.frame_identity,
            frame_identity)
        || staged_writer_id_ != published_writer_id
        || snapshot_.prepared_writer_id != published_writer_id
        || !FinalWriterAllowed(published_writer_id))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (snapshot_.snapshot_transaction_staged)
    {
        staged_snapshot_owner_.ValidatePrepared(
            frame_identity,
            published_writer_id,
            status);
    }
}

void StaticImmediateGunResponseStagedOwner::CommitPrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) noexcept
{
    ValidatePrepared(frame_identity, published_writer_id, status);
    if (!status.ok())
    {
        return;
    }

    if (snapshot_.snapshot_transaction_staged)
    {
        staged_snapshot_owner_.CommitPrepared(
            frame_identity,
            published_writer_id,
            status);
        if (!status.ok())
        {
            return;
        }
        staged_snapshot_owner_.CopySnapshot(snapshot_.snapshot);
    }

    committed_high_g_owner_ = staged_high_g_owner_;
    committed_snapshot_owner_ = staged_snapshot_owner_;
    committed_phase_graded_owner_ = staged_phase_graded_owner_;
    snapshot_.state_committed = true;
    snapshot_.state_aborted = false;
    ++generation_;
    DiscardStagedState();
}

void StaticImmediateGunResponseStagedOwner::AbortPrepared() noexcept
{
    if (staged_ready_)
    {
        if (snapshot_.snapshot_transaction_staged)
        {
            staged_snapshot_owner_.AbortPrepared();
            staged_snapshot_owner_.CopySnapshot(snapshot_.snapshot);
        }
        snapshot_.state_committed = false;
        snapshot_.state_aborted = true;
    }
    DiscardStagedState();
}

void StaticImmediateGunResponseStagedOwner::CopySnapshot(
    StaticImmediateGunResponsePreparedReceipt& output) const noexcept
{
    output = snapshot_;
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
