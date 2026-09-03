#include "LadyLuck/behavior_tree/static/StaticSafetyGunStagedOwner.hpp"

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

StaticSafetyGunStagedOwner::StaticSafetyGunStagedOwner() noexcept
{
    Reset();
}

void StaticSafetyGunStagedOwner::Reset() noexcept
{
    committed_writers_.Reset();
    staged_writers_.Reset();
    committed_prefire_observer_.Reset();
    staged_prefire_observer_.Reset();
    committed_prefire_consumer_.Reset();
    staged_prefire_consumer_.Reset();
    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
    snapshot_ = StaticSafetyGunPreparedReceipt{};
}

void StaticSafetyGunStagedOwner::DiscardStagedState() noexcept
{
    staged_writers_ = committed_writers_;
    staged_prefire_observer_ = committed_prefire_observer_;
    staged_prefire_consumer_ = committed_prefire_consumer_;
    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
}

void StaticSafetyGunStagedOwner::RejectPrepared(
    const StatusCode code,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    snapshot_.disposition = StaticSafetyGunDisposition::Fault;
    snapshot_.prepared_writer_id = ControlIntentWriterNone;
    snapshot_.candidate_count = 0U;
    snapshot_.command_candidate_staged = false;
    snapshot_.state_committed = false;
    snapshot_.state_aborted = staged_ready_;
    snapshot_.status_code = code;
    DiscardStagedState();
    status.code = code;
}

void StaticSafetyGunStagedOwner::Prepare(
    const runtime::TacticalCommandBuildInput& input,
    ControlIntent& output,
    StaticSafetyGunPreparedReceipt& receipt,
    Status& status) noexcept
{
    output.Clear();
    receipt = StaticSafetyGunPreparedReceipt{};
    status = Status{};

    // A caller that abandoned an earlier frame cannot leak its staged writer
    // state into the current observation. The committed state remains intact.
    DiscardStagedState();
    snapshot_ = StaticSafetyGunPreparedReceipt{};
    snapshot_.prepare_attempted = true;
    snapshot_.frame_identity = input.frame.frame_identity;

    if (!input.valid
        || !IsValidControlFrameIdentity(input.frame.frame_identity))
    {
        RejectPrepared(StatusCode::InvalidArgument, output, status);
        receipt = snapshot_;
        return;
    }

    staged_writers_ = committed_writers_;
    staged_prefire_observer_ = committed_prefire_observer_;
    staged_prefire_consumer_ = committed_prefire_consumer_;
    staged_ready_ = true;
    staged_frame_identity_ = input.frame.frame_identity;

    // Exact current dynamic-provider predicate: no new terrain threshold,
    // dwell, feedback freshness gate, or inferred recovery authority.
    snapshot_.safety_current_required =
        input.current_safety.valid
        && input.current_safety.entry_available
        && input.current_safety.entry_recoverable
        && input.current_safety.entry_should_activate;
    snapshot_.safety_feedback_latched =
        input.current_safety.valid
        && input.current_safety.entry_available
        && input.current_safety.entry_recoverable
        && input.previous_control_feedback.valid
        && input.previous_control_feedback.auto_gcas_active;
    snapshot_.safety_required =
        snapshot_.safety_current_required
        || snapshot_.safety_feedback_latched;

    Status evidence_status{};
    staged_writers_.ObserveRootGunPreTaskEvidence(
        input.frame,
        snapshot_.root_gun_evidence,
        evidence_status);
    snapshot_.root_gun_evidence_status = evidence_status.code;
    const bool root_gun_evidence_ready =
        evidence_status.code == StatusCode::Ok
        && snapshot_.root_gun_evidence.valid
        && SameControlFrameIdentity(
            snapshot_.root_gun_evidence.frame_identity,
            input.frame.frame_identity);
    if (!root_gun_evidence_ready)
    {
        if (snapshot_.root_gun_evidence_status == StatusCode::Ok)
        {
            snapshot_.root_gun_evidence_status =
                StatusCode::InvalidConfiguration;
        }
        if (snapshot_.safety_required)
        {
            // Root Gun evidence is command-neutral for a selected Safety
            // override. Preserve the last committed observation state and
            // continue with writer 1.
            staged_writers_ = committed_writers_;
            snapshot_.optional_evidence_fault = true;
        }
        else
        {
            RejectPrepared(
                evidence_status.code == StatusCode::Ok
                    ? StatusCode::InvalidConfiguration
                    : evidence_status.code,
                output,
                status);
            receipt = snapshot_;
            return;
        }
    }

    if (root_gun_evidence_ready)
    {
        snapshot_.prefire_observation_attempted = true;
        guidance::prefire::PrefireOptionalDouble capability_n{};
        capability_n.has_value =
            snapshot_.root_gun_evidence.capability_admitted;
        capability_n.value = capability_n.has_value
            ? snapshot_.root_gun_evidence.capability.n_inst_g.value
            : 0.0;
        Status prefire_observation_status{};
        staged_prefire_observer_.Update(
            input.frame,
            capability_n,
            snapshot_.root_gun_evidence.capability_admitted,
            snapshot_.prefire_threat_shadow,
            prefire_observation_status);
        snapshot_.prefire_observation_status =
            prefire_observation_status.code;
        if (prefire_observation_status.ok())
        {
            snapshot_.prefire_observation_ready = true;
        }
        else
        {
            // Predictive evidence is optional command-neutral evidence.
            // Retain a current official Gun or Safety base command while the
            // exact observer-owned fault/reset state remains staged.
            snapshot_.optional_evidence_fault = true;
        }
    }

    if (snapshot_.safety_required)
    {
        // Existing Safety preemption semantics: clear the staged Gun episode,
        // clear the staged predictive consumer lifecycle, then reuse the
        // current-receipt writer-1 builder unchanged. The observer's causal
        // sample history remains independent, matching the dynamic owner.
        staged_writers_.ResetGunThreatEpisode();
        staged_prefire_consumer_.Reset();
        snapshot_.prefire_safety_veto = true;
        Status candidate_status{};
        staged_writers_.BuildRootAutoGcasRecovery(
            input.frame,
            input.current_safety,
            output,
            candidate_status);
        if (candidate_status.code != StatusCode::Ok
            || output.writer_id != ControlIntentWriterAutoGcasRecovery
            || !SameControlFrameIdentity(
                output.frame_identity,
                input.frame.frame_identity))
        {
            RejectPrepared(
                candidate_status.code == StatusCode::Ok
                    ? StatusCode::InvalidConfiguration
                    : candidate_status.code,
                output,
                status);
            receipt = snapshot_;
            return;
        }

        Status validation_status{};
        output.Validate(validation_status);
        if (validation_status.code != StatusCode::Ok)
        {
            RejectPrepared(
                validation_status.code,
                output,
                status);
            receipt = snapshot_;
            return;
        }

        staged_writer_id_ = ControlIntentWriterAutoGcasRecovery;
        snapshot_.disposition =
            StaticSafetyGunDisposition::AutoGcasPrepared;
        snapshot_.prepared_writer_id = staged_writer_id_;
        snapshot_.candidate_count = 1U;
        snapshot_.observation_state_staged = true;
        snapshot_.command_candidate_staged = true;
        snapshot_.status_code = StatusCode::Ok;
        receipt = snapshot_;
        return;
    }

    const bool current_prefire_safety_veto =
        input.current_safety.valid
        && (input.current_safety.entry_should_activate
            || input.current_safety.entry_boundary_breached);
    const bool feedback_prefire_safety_veto =
        input.feedback_freshness
            == runtime::TacticalFeedbackFreshness::Fresh
        && input.previous_control_feedback.valid
        && input.previous_control_feedback.auto_gcas_active;
    snapshot_.prefire_safety_veto =
        snapshot_.safety_required
        || current_prefire_safety_veto
        || feedback_prefire_safety_veto;

    snapshot_.gun_admission.valid = true;
    snapshot_.gun_admission.frame_identity = input.frame.frame_identity;
    snapshot_.gun_admission.official_damage_active =
        snapshot_.root_gun_evidence.official_gun_threat;
    snapshot_.gun_admission.entry_side_sign_valid = false;
    snapshot_.gun_admission.entry_side_sign = 1;

    if (snapshot_.prefire_safety_veto)
    {
        // Exact dynamic veto: a current boundary/terrain demand or fresh
        // active Auto-GCAS feedback releases Gun and predictive continuation.
        staged_writers_.ResetGunThreatEpisode();
        staged_prefire_consumer_.Reset();
        snapshot_.observation_state_staged = true;
        snapshot_.disposition = StaticSafetyGunDisposition::NotApplicable;
        snapshot_.status_code = StatusCode::Ok;
        receipt = snapshot_;
        return;
    }

    guidance::prefire::SameIndexGeometryFrameEnvelope envelope{};
    const guidance::prefire::SameIndexGeometryFrameEnvelope*
        envelope_ptr = nullptr;
    if (input.frame.target_same_index)
    {
        envelope.frame_identity = input.frame.frame_identity;
        envelope.t_sec = input.frame.t_sec;
        envelope_ptr = &envelope;
    }

    if (snapshot_.prefire_observation_ready)
    {
        guidance::prefire::RootPrefireControlPathEvidence control{};
        control.safety_observation_available = input.current_safety.valid;
        control.feedback_freshness = input.feedback_freshness;
        control.previous_control_feedback_available =
            input.previous_control_feedback.valid;
        control.command_backend_id =
            input.previous_control_feedback.command_backend_id;
        snapshot_.prefire_consumer_attempted = true;
        Status prefire_consumer_status{};
        staged_prefire_consumer_.Update(
            input.frame,
            snapshot_.prefire_threat_shadow,
            envelope_ptr,
            snapshot_.gun_admission.official_damage_active,
            control,
            snapshot_.prefire_consumer,
            prefire_consumer_status);
        snapshot_.prefire_consumer_status = prefire_consumer_status.code;
        if (prefire_consumer_status.ok())
        {
            snapshot_.prefire_consumer_ready = true;
        }
        else
        {
            // Preserve the observer's accepted causal sample and apply the
            // dynamic owner's exact consumer-fault reset without erasing a
            // same-frame official base Gun command.
            staged_prefire_consumer_.Reset();
            snapshot_.prefire_consumer =
                guidance::prefire::RootPrefireThreatConsumerDecision{};
            snapshot_.optional_evidence_fault = true;
        }
    }

    snapshot_.gun_admission.predictive_prefire_active =
        snapshot_.prefire_consumer_ready
        && snapshot_.prefire_consumer.receipt.active;
    snapshot_.gun_admission.predictive_check_extend_hold =
        snapshot_.prefire_consumer_ready
        && snapshot_.prefire_consumer.receipt.check_extend_hold;
    snapshot_.gun_admission.immediate_defense_required =
        snapshot_.gun_admission.official_damage_active
        || snapshot_.gun_admission.predictive_prefire_active
        || snapshot_.gun_admission.predictive_check_extend_hold;

    const bool gun_threat_active =
        snapshot_.gun_admission.official_damage_active
        || snapshot_.gun_admission.predictive_prefire_active;
    if (gun_threat_active)
    {
        snapshot_.toward_side_observation_attempted = true;
        Status toward_status{};
        guidance::prefire::ObserveRootGunTowardSideShadow(
            input.frame,
            envelope_ptr,
            nullptr,
            true,
            snapshot_.toward_side,
            toward_status);
        snapshot_.toward_side_observation_status = toward_status.code;
        if (toward_status.ok())
        {
            snapshot_.toward_side_observation_ready = true;
            snapshot_.gun_admission.entry_side_sign_valid =
                snapshot_.toward_side.toward_side_sign_valid;
            snapshot_.gun_admission.entry_side_sign =
                snapshot_.toward_side.toward_side_sign;
        }
        else
        {
            // Direction evidence is optional. Root writer 2 retains its exact
            // existing alternating entry-side fallback on this fault.
            snapshot_.optional_evidence_fault = true;
        }
    }

    Status admitted_status{};
    staged_writers_.ObserveAdmittedGunThreat(
        snapshot_.gun_admission.immediate_defense_required,
        admitted_status);
    if (admitted_status.code != StatusCode::Ok)
    {
        RejectPrepared(admitted_status.code, output, status);
        receipt = snapshot_;
        return;
    }
    snapshot_.observation_state_staged = true;

    if (!snapshot_.gun_admission.immediate_defense_required)
    {
        // The selected lower tactical writer will finalize this command-neutral
        // no-threat observation after its own current wire succeeds.
        snapshot_.disposition = StaticSafetyGunDisposition::NotApplicable;
        snapshot_.status_code = StatusCode::Ok;
        receipt = snapshot_;
        return;
    }

    Status candidate_status{};
    staged_writers_.BuildGunDefense(
        input.frame,
        snapshot_.root_gun_evidence,
        true,
        snapshot_.gun_admission.entry_side_sign_valid,
        snapshot_.gun_admission.entry_side_sign,
        output,
        candidate_status);
    if (candidate_status.code != StatusCode::Ok
        || output.writer_id
            != ControlIntentWriterGunDefenseHorizontalBreak
        || !SameControlFrameIdentity(
            output.frame_identity,
            input.frame.frame_identity))
    {
        RejectPrepared(
            candidate_status.code == StatusCode::Ok
                ? StatusCode::InvalidConfiguration
                : candidate_status.code,
            output,
            status);
        receipt = snapshot_;
        return;
    }

    Status validation_status{};
    output.Validate(validation_status);
    if (validation_status.code != StatusCode::Ok)
    {
        RejectPrepared(validation_status.code, output, status);
        receipt = snapshot_;
        return;
    }

    staged_writer_id_ = ControlIntentWriterGunDefenseHorizontalBreak;
    snapshot_.disposition = StaticSafetyGunDisposition::GunBreakPrepared;
    snapshot_.prepared_writer_id = staged_writer_id_;
    snapshot_.candidate_count = 1U;
    snapshot_.command_candidate_staged = true;
    snapshot_.status_code = StatusCode::Ok;
    receipt = snapshot_;
}

void StaticSafetyGunStagedOwner::ValidatePrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) const noexcept
{
    status = Status{};
    bool candidate_writer_matches = false;
    if (staged_writer_id_ == ControlIntentWriterNone)
    {
        candidate_writer_matches =
            published_writer_id != ControlIntentWriterNone
            && published_writer_id
                != ControlIntentWriterAutoGcasRecovery
            && published_writer_id
                != ControlIntentWriterGunDefenseHorizontalBreak
            && published_writer_id
                != ControlIntentWriterOfficialGunSnapshotPlaneChange;
    }
    else if (staged_writer_id_
        == ControlIntentWriterGunDefenseHorizontalBreak)
    {
        candidate_writer_matches =
            published_writer_id
                == ControlIntentWriterGunDefenseHorizontalBreak
            || published_writer_id
                == ControlIntentWriterG4HighGBarrel
            || published_writer_id
                == ControlIntentWriterOfficialGunSnapshotPlaneChange
            || published_writer_id == ControlIntentWriterDbfmHardTurn;
    }
    else
    {
        candidate_writer_matches = published_writer_id == staged_writer_id_;
    }
    if (!staged_ready_
        || !IsValidControlFrameIdentity(frame_identity)
        || !SameControlFrameIdentity(
            staged_frame_identity_,
            frame_identity)
        || !candidate_writer_matches)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
}

void StaticSafetyGunStagedOwner::CommitPrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) noexcept
{
    ValidatePrepared(frame_identity, published_writer_id, status);
    if (!status.ok())
    {
        snapshot_.status_code = status.code;
        return;
    }

    if (staged_writer_id_
        == ControlIntentWriterGunDefenseHorizontalBreak)
    {
        staged_writers_.CommitPublishedGunDirection(frame_identity);
    }
    committed_writers_ = staged_writers_;
    committed_prefire_observer_ = staged_prefire_observer_;
    committed_prefire_consumer_ = staged_prefire_consumer_;
    snapshot_.state_committed = true;
    snapshot_.state_aborted = false;
    snapshot_.status_code = StatusCode::Ok;
    DiscardStagedState();
}

void StaticSafetyGunStagedOwner::AbortPrepared() noexcept
{
    if (staged_ready_)
    {
        snapshot_.state_aborted = true;
        snapshot_.state_committed = false;
    }
    DiscardStagedState();
}

void StaticSafetyGunStagedOwner::CopySnapshot(
    StaticSafetyGunPreparedReceipt& output) const noexcept
{
    output = snapshot_;
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
