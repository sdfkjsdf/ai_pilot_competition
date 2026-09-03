#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bOwner.hpp"
#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteMatrix(const LadyLuck::Matrix3RowMajor& value) noexcept
{
    for (const double element : value)
    {
        if (!std::isfinite(element))
        {
            return false;
        }
    }
    return true;
}

bool IsTacticalMode(
    const LadyLuck::guidance::doctrine::TacticalMode mode) noexcept
{
    return mode == LadyLuck::guidance::doctrine::TacticalMode::Obfm
        || mode == LadyLuck::guidance::doctrine::TacticalMode::Habfm
        || mode == LadyLuck::guidance::doctrine::TacticalMode::Dbfm;
}

LadyLuck::DoctrineBehaviorId SpacingBehaviorId(
    const LadyLuck::guidance::obfm::ObfmSpacingOwnerPhase phase) noexcept
{
    using LadyLuck::DoctrineBehaviorId;
    using LadyLuck::guidance::obfm::ObfmSpacingOwnerPhase;
    switch (phase)
    {
    case ObfmSpacingOwnerPhase::PathEnergyExchange:
        return DoctrineBehaviorId::ObfmSpacingArrestPathEnergyExchange;
    case ObfmSpacingOwnerPhase::PostHitRminArrest:
        return DoctrineBehaviorId::ObfmSpacingArrestPostHitRminArrest;
    case ObfmSpacingOwnerPhase::LevelRecovery:
        return DoctrineBehaviorId::ObfmSpacingArrestLevelRecovery;
    case ObfmSpacingOwnerPhase::WezReacquire:
        return DoctrineBehaviorId::ObfmSpacingArrestWezReacquire;
    case ObfmSpacingOwnerPhase::Inactive:
    default:
        return DoctrineBehaviorId::Invalid;
    }
}

void BuildSpacingIntent(
    const LadyLuck::ControlFrameIdentity& frame_identity,
    const LadyLuck::guidance::obfm::ObfmSpacingOwnerTaskReceipt& task,
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status) noexcept
{
    output.Clear();
    status = LadyLuck::Status{};
    const LadyLuck::DoctrineBehaviorId behavior =
        SpacingBehaviorId(task.phase);
    if (!task.candidate_valid
        || task.candidate_count != 1U
        || behavior == LadyLuck::DoctrineBehaviorId::Invalid)
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return;
    }
    output.frame_identity = frame_identity;
    output.aim_point_m = task.candidate.aim_point_ned_m;
    output.desired_speed_mps = task.candidate.desired_speed_mps;
    output.desired_speed_rate_mps2 =
        task.candidate.desired_speed_rate_mps2;
    output.specific_energy_rate_bias_m2ps3 =
        task.candidate.specific_energy_rate_bias_m2ps3;
    output.path_inversion_allowed.has_value = true;
    output.path_inversion_allowed.value =
        task.candidate.path_inversion_allowed;
    output.capture_range_des_m = task.candidate.capture_range_des_m;
    output.behavior_id = behavior;
    output.mode_id = LadyLuck::DoctrineModeId::Obfm;
    output.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    output.writer_id = LadyLuck::ControlIntentWriterObfmSpacing;
    output.Validate(status);
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

StaticDoctrineObfmG16G5bOwner::StaticDoctrineObfmG16G5bOwner() noexcept
{
    Reset();
}

void StaticDoctrineObfmG16G5bOwner::Reset() noexcept
{
    g16_evidence_provider_.Reset();
    g16_committed_owner_.Reset();
    g16_high_prevention_.Reset();
    g5b_delayed_climb_.Reset();
    obfm_spacing_owner_.ResetEpisode();
    obfm_spacing_task_active_ = false;
    g3_roll_counter_owner_.Reset();
    g3_scissors_owner_.Reset();
    obfm_apex_displacement_.ResetEpisode();
    obfm_apex_task_active_ = false;
    entry_established_turn_observer_.Reset();
    entry_window_.ResetEpisode();
    entry_longitudinal_.ResetEpisode();
    obfm_lag_guidance_.Reset();
    frame_evidence_provider_.Reset();
    input_ = StaticDoctrineObfmG16G5bInput{};
    original_state_ = StaticDoctrineObfmG16G5bOwnerState{};
    candidate_state_ = StaticDoctrineObfmG16G5bOwnerState{};
    ClearFrameState();
}

void StaticDoctrineObfmG16G5bOwner::ClearFrameState() noexcept
{
    original_state_ready_ = false;
    candidate_state_ready_ = false;
    candidate_stage_active_ = false;
    deferred_commit_requested_ = false;
    integrated_intent_staged_ = false;
    prepared_transaction_ready_ = false;
    staged_base_intent_.Clear();
    staged_base_intent_ready_ = false;
    prepared_intent_.Clear();
    selected_writer_id_ = ControlIntentWriterNone;
    selection_count_ = 0U;
    snapshot_ = StaticDoctrineObfmG16G5bSnapshot{};
}

void StaticDoctrineObfmG16G5bOwner::ClearLagCandidateFrameState() noexcept
{
    snapshot_.lag_station_observation_attempted = false;
    snapshot_.lag_station_observation_ready = false;
    snapshot_.lag_station = ObfmStationHoldServiceReceipt{};
    snapshot_.lag_speed_authority = ObfmLagSpeedAuthority::Unavailable;
    snapshot_.lag_speed_authority_ready = false;
    snapshot_.lag_base_ready = false;
    snapshot_.lag_commit = ObfmLagGuidanceCommit{};
    snapshot_.lag_commit_ready = false;
    snapshot_.lag_base_intent.Clear();
    snapshot_.lead_discipline =
        guidance::obfm::ObfmLeadDisciplineReceipt{};
    snapshot_.lead_discipline_ready = false;
    snapshot_.terminal_tracking =
        guidance::obfm::TerminalTrackingReceipt{};
    snapshot_.terminal_tracking_ready = false;
    snapshot_.effective_terminal_tracking = false;
    snapshot_.chase_up_frame_evidence = HabfmFrameEvidence{};
    snapshot_.chase_up_frame_evidence_status =
        HabfmFrameEvidenceStatus::FrameStateNotFinite;
    snapshot_.chase_up = guidance::obfm::ObfmChaseUpGuardReceipt{};
}

void StaticDoctrineObfmG16G5bOwner::Prepare(
    const StaticDoctrineObfmG16G5bInput& input,
    StaticDoctrineObfmG16G5bSnapshot& output,
    Status& status) noexcept
{
    AbortPrepared();
    input_ = input;
    snapshot_ = StaticDoctrineObfmG16G5bSnapshot{};
    snapshot_.prepare_attempted = true;
    snapshot_.frame_identity = input.tactical_input.frame.frame_identity;
    status = Status{};

    if (!input.valid
        || !input.tactical_input.valid
        || !IsValidControlFrameIdentity(
            input.tactical_input.frame.frame_identity)
        || !input.mode_decision.valid
        || !input.mode_decision.mode.has_value
        || !IsTacticalMode(input.mode_decision.mode.value))
    {
        snapshot_.status_code = StatusCode::InvalidArgument;
        status.code = snapshot_.status_code;
        output = snapshot_;
        return;
    }

    snapshot_.frame_ready = true;
    snapshot_.effective_mode = input.mode_decision.mode;
    snapshot_.status_code = StatusCode::Ok;

    // Preserve the legacy parent-preemption semantics.  Physical observation
    // history remains intact; only private High execution is halted here.
    if (input.mode_decision.bypass_reason
            != guidance::doctrine::ModeDecisionBypassReason::None
        || input.mode_decision.mode.value
            != guidance::doctrine::TacticalMode::Obfm)
    {
        g16_high_prevention_.HaltExecutionPreservingObservation();
    }
    if (input.mode_decision.mode.value
        != guidance::doctrine::TacticalMode::Obfm)
    {
        g16_committed_owner_.Reset();
    }

    output = snapshot_;
}

void StaticDoctrineObfmG16G5bOwner::CaptureOwnerState(
    StaticDoctrineObfmG16G5bOwnerState& output) const noexcept
{
    output.g16_committed = g16_committed_owner_;
    g16_high_prevention_.CaptureTransactionState(output.g16_high);
    output.g5b = g5b_delayed_climb_;
    output.entry_window = entry_window_;
    output.entry_longitudinal = entry_longitudinal_;
    output.spacing = obfm_spacing_owner_;
    output.spacing_task_active = obfm_spacing_task_active_;
    output.g3_roll_counter = g3_roll_counter_owner_;
    output.g3_scissors = g3_scissors_owner_;
    output.apex = obfm_apex_displacement_;
    output.apex_task_active = obfm_apex_task_active_;
    output.lag = obfm_lag_guidance_;
}

void StaticDoctrineObfmG16G5bOwner::RestoreOwnerState(
    const StaticDoctrineObfmG16G5bOwnerState& input) noexcept
{
    g16_committed_owner_ = input.g16_committed;
    g16_high_prevention_.RestoreTransactionState(input.g16_high);
    g5b_delayed_climb_ = input.g5b;
    entry_window_ = input.entry_window;
    entry_longitudinal_ = input.entry_longitudinal;
    obfm_spacing_owner_ = input.spacing;
    obfm_spacing_task_active_ = input.spacing_task_active;
    g3_roll_counter_owner_ = input.g3_roll_counter;
    g3_scissors_owner_ = input.g3_scissors;
    obfm_apex_displacement_ = input.apex;
    obfm_apex_task_active_ = input.apex_task_active;
    obfm_lag_guidance_ = input.lag;
}

bool StaticDoctrineObfmG16G5bOwner::ObfmModeSelected() const noexcept
{
    return snapshot_.frame_ready
        && snapshot_.effective_mode.has_value
        && snapshot_.effective_mode.value
            == guidance::doctrine::TacticalMode::Obfm;
}

bool StaticDoctrineObfmG16G5bOwner::CurrentEffectEmploySelected() const noexcept
{
    return snapshot_.frame_ready
        && input_.current_effect_employ_override
        && std::isfinite(input_.tactical_input.frame.own_offense.damage_rate)
        && input_.tactical_input.frame.own_offense.damage_rate > 0.0;
}

void StaticDoctrineObfmG16G5bOwner::BeginFinalCommandCandidateStage(
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.frame_ready
        || (!ObfmModeSelected() && !CurrentEffectEmploySelected())
        || candidate_stage_active_
        || prepared_transaction_ready_
        || staged_base_intent_ready_
        || selected_writer_id_ != ControlIntentWriterNone
        || selection_count_ != 0U)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }

    CaptureOwnerState(original_state_);
    original_state_ready_ = true;
    candidate_state_ready_ = false;
    candidate_stage_active_ = true;
    deferred_commit_requested_ = false;
    integrated_intent_staged_ = false;
    snapshot_.candidate_stage_active = true;
    snapshot_.candidate_state_ready = false;
    snapshot_.deferred_commit_requested = false;
    snapshot_.integrated_intent_staged = false;
}

void StaticDoctrineObfmG16G5bOwner::CopyStagedBaseIntent(
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    if (!snapshot_.frame_ready
        || !candidate_stage_active_
        || !staged_base_intent_ready_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output = staged_base_intent_;
    output.Validate(status);
    if (!status.ok()
        || !SameControlFrameIdentity(
            output.frame_identity,
            snapshot_.frame_identity))
    {
        output.Clear();
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
    }
}

void StaticDoctrineObfmG16G5bOwner::RollbackFinalCommandCandidateState(
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.frame_ready
        || !candidate_stage_active_
        || !original_state_ready_
        || staged_base_intent_ready_
        || candidate_state_ready_
        || deferred_commit_requested_
        || integrated_intent_staged_
        || prepared_transaction_ready_
        || selected_writer_id_ != ControlIntentWriterNone
        || selection_count_ != 0U)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    RestoreOwnerState(original_state_);
    // Candidate-local LAG receipts are produced while probing HighToLag.
    // If that optional sibling rejects its writer, persistent owner state is
    // rolled back above and these per-frame construction receipts must be
    // cleared as well before ordinary LAG is evaluated in the same selector
    // tick.  Physical observations and precision-speed inputs remain intact.
    ClearLagCandidateFrameState();
}

void StaticDoctrineObfmG16G5bOwner::SealCandidateState(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !staged_base_intent_ready_
        || !original_state_ready_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (candidate_state_ready_)
    {
        return;
    }
    CaptureOwnerState(candidate_state_);
    RestoreOwnerState(original_state_);
    candidate_state_ready_ = true;
    snapshot_.candidate_state_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::BeginDeferredFinalStateCommit(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !staged_base_intent_ready_
        || !original_state_ready_
        || deferred_commit_requested_
        || integrated_intent_staged_
        || prepared_transaction_ready_)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    SealCandidateState(status);
    if (status.ok())
    {
        deferred_commit_requested_ = true;
        snapshot_.deferred_commit_requested = true;
    }
}

void StaticDoctrineObfmG16G5bOwner::PublishIntegratedFinalIntent(
    const ControlIntent& intent,
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.frame_ready
        || !candidate_stage_active_
        || !staged_base_intent_ready_
        || integrated_intent_staged_
        || prepared_transaction_ready_
        || intent.writer_id == ControlIntentWriterNone
        || !SameControlFrameIdentity(
            intent.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    intent.Validate(status);
    if (!status.ok())
    {
        snapshot_.status_code = status.code;
        return;
    }
    SealCandidateState(status);
    if (!status.ok())
    {
        snapshot_.status_code = status.code;
        return;
    }
    prepared_intent_ = intent;
    integrated_intent_staged_ = true;
    snapshot_.integrated_intent_staged = true;
    snapshot_.prepared_intent = intent;
    snapshot_.candidate_count = 1U;
}

void StaticDoctrineObfmG16G5bOwner::CommitPreparedFinalState(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !original_state_ready_
        || !candidate_state_ready_
        || !integrated_intent_staged_
        || prepared_transaction_ready_)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }

    // The adapter has completed raw-guidance selection.  Persistent maneuver
    // state remains in candidate_state_ until the downstream finite wire is
    // acknowledged through CommitPrepared().
    prepared_transaction_ready_ = true;
    candidate_stage_active_ = false;
    staged_base_intent_ready_ = false;
    staged_base_intent_.Clear();
    original_state_ready_ = false;
    deferred_commit_requested_ = false;
    snapshot_.candidate_stage_active = false;
    snapshot_.prepared_transaction_ready = true;
    snapshot_.prepared_transaction_committed = false;
    snapshot_.prepared_transaction_aborted = false;
    snapshot_.selected_writer_id = prepared_intent_.writer_id;
    snapshot_.selection_count = 1U;
    snapshot_.status_code = StatusCode::Ok;
}

void StaticDoctrineObfmG16G5bOwner::AbortPreparedFinalState() noexcept
{
    if (prepared_transaction_ready_)
    {
        return;
    }
    if (original_state_ready_)
    {
        RestoreOwnerState(original_state_);
    }
    candidate_stage_active_ = false;
    original_state_ready_ = false;
    candidate_state_ready_ = false;
    deferred_commit_requested_ = false;
    integrated_intent_staged_ = false;
    staged_base_intent_ready_ = false;
    staged_base_intent_.Clear();
    prepared_intent_.Clear();
    selected_writer_id_ = ControlIntentWriterNone;
    selection_count_ = 0U;
    snapshot_.candidate_stage_active = false;
    snapshot_.candidate_state_ready = false;
    snapshot_.deferred_commit_requested = false;
    snapshot_.integrated_intent_staged = false;
    snapshot_.prepared_transaction_aborted = true;
    snapshot_.selected_writer_id = ControlIntentWriterNone;
    snapshot_.selection_count = 0U;
    snapshot_.candidate_count = 0U;
    snapshot_.staged_base_intent.Clear();
    snapshot_.prepared_intent.Clear();
    ClearLagCandidateFrameState();
}

void StaticDoctrineObfmG16G5bOwner::ValidatePrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) const noexcept
{
    status = Status{};
    if (!prepared_transaction_ready_
        || !candidate_state_ready_
        || !integrated_intent_staged_
        || published_writer_id == ControlIntentWriterNone
        || prepared_intent_.writer_id != published_writer_id
        || !SameControlFrameIdentity(
            frame_identity,
            snapshot_.frame_identity)
        || !SameControlFrameIdentity(
            prepared_intent_.frame_identity,
            frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    prepared_intent_.Validate(status);
}

void StaticDoctrineObfmG16G5bOwner::CommitPrepared(
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
    RestoreOwnerState(candidate_state_);
    prepared_transaction_ready_ = false;
    candidate_state_ready_ = false;
    integrated_intent_staged_ = false;
    snapshot_.prepared_transaction_ready = false;
    snapshot_.prepared_transaction_committed = true;
    snapshot_.prepared_transaction_aborted = false;
    snapshot_.status_code = StatusCode::Ok;
}

void StaticDoctrineObfmG16G5bOwner::AbortPrepared() noexcept
{
    if (candidate_stage_active_)
    {
        AbortPreparedFinalState();
    }
    if (prepared_transaction_ready_)
    {
        prepared_transaction_ready_ = false;
        candidate_state_ready_ = false;
        integrated_intent_staged_ = false;
        snapshot_.prepared_transaction_ready = false;
        snapshot_.prepared_transaction_committed = false;
        snapshot_.prepared_transaction_aborted = true;
    }
    prepared_intent_.Clear();
}

void StaticDoctrineObfmG16G5bOwner::CopySnapshot(
    StaticDoctrineObfmG16G5bSnapshot& output) const noexcept
{
    output = snapshot_;
}

void StaticDoctrineObfmG16G5bOwner::CopyPreparedIntent(
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    if (!prepared_transaction_ready_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output = prepared_intent_;
    output.Validate(status);
}

void StaticDoctrineObfmG16G5bOwner::CheckObfmWriterLocalRejection(
    bool& rejected,
    Status& status) const noexcept
{
    rejected = false;
    status = Status{};
    if (!snapshot_.frame_ready
        || !candidate_stage_active_
        || !SameControlFrameIdentity(
            snapshot_.frame_identity,
            input_.tactical_input.frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    rejected = selection_count_ == 0U
        && selected_writer_id_ == ControlIntentWriterNone
        && !staged_base_intent_ready_;
}

void StaticDoctrineObfmG16G5bOwner::SetEmptyPrecisionStation() noexcept
{
    snapshot_.precision_station = ObfmStationHoldServiceReceipt{};
    snapshot_.precision_station.reference.frame_identity =
        snapshot_.frame_identity;
    snapshot_.precision_station.reference.evaluated = true;
}

void StaticDoctrineObfmG16G5bOwner::SetPhaseCurrentSpeedEcho() noexcept
{
    snapshot_.precision_longitudinal = ObfmLongitudinalProviderReceipt{};
    snapshot_.precision_longitudinal.status =
        ObfmLongitudinalProviderStatus::ExactCausalCommandAdmissionUnavailable;
    ObfmLagLongitudinalReference& reference =
        snapshot_.precision_longitudinal.reference;
    reference.frame_identity = snapshot_.frame_identity;
    reference.evaluated = true;
    reference.source_authoritative = true;
    reference.same_reference_episode =
        snapshot_.precision_lag_preparation.same_reference_episode;
}

void StaticDoctrineObfmG16G5bOwner::BuildPrecisionSpeedReference(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.precision_speed_ready = false;
    snapshot_.precision_speed =
        guidance::committed::G16PrecisionSpeedReceipt{};
    snapshot_.precision_station = ObfmStationHoldServiceReceipt{};
    snapshot_.precision_lag_preparation = ObfmLagGuidancePreparation{};
    snapshot_.precision_longitudinal = ObfmLongitudinalProviderReceipt{};
    snapshot_.precision_bumpless = ObfmBumplessSpeedReceipt{};
    if (!snapshot_.frame_ready)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    const DogfightGeometryFrame& frame = input_.tactical_input.frame;
    const double own_speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    if (!std::isfinite(own_speed_mps) || own_speed_mps <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    guidance::committed::G16PrecisionSpeedReceipt& output =
        snapshot_.precision_speed;
    output.evaluated = true;
    output.frame_identity = frame.frame_identity;
    output.source = guidance::committed::
        G16PrecisionSpeedSource::CurrentSpeedEcho;
    output.desired_speed_mps = own_speed_mps;
    output.desired_speed_rate_mps2 = 0.0;
    snapshot_.precision_speed_ready = true;

    ObfmStationHoldServiceInput station_input{};
    station_input.frame_identity = frame.frame_identity;
    station_input.station_hold_enabled = true;
    station_input.station_hold_owner_split_enabled = false;
    station_input.range_m = frame.own_offense.range_m;
    station_input.official_min_range_m = frame.own_offense.phase.min_range_m;
    station_input.official_max_range_m = frame.own_offense.phase.max_range_m;
    station_input.target_velocity_ned_mps = frame.opponent.velocity_ned_mps;
    station_input.stall_speed_1g_mps.has_value =
        input_.tactical_input.current_envelope.stall_speed_valid;
    station_input.stall_speed_1g_mps.value =
        input_.tactical_input.current_envelope.stall_speed_mps;
    Status station_status{};
    obfm_station_hold_service_.Evaluate(
        station_input,
        snapshot_.precision_station,
        station_status);
    if (!station_status.ok()
        || !snapshot_.precision_station.reference.evaluated
        || !SameControlFrameIdentity(
            snapshot_.precision_station.reference.frame_identity,
            frame.frame_identity))
    {
        SetEmptyPrecisionStation();
    }

    OptionalFrameIndex safety_frame_index{};
    if (input_.tactical_input.current_safety.valid)
    {
        safety_frame_index.has_value = true;
        safety_frame_index.value = frame.frame_identity.frame_index;
    }
    Status preparation_status{};
    obfm_lag_guidance_.Prepare(
        frame,
        safety_frame_index,
        snapshot_.precision_lag_preparation,
        preparation_status);
    if (!preparation_status.ok()
        || !snapshot_.precision_lag_preparation.valid
        || !SameControlFrameIdentity(
            snapshot_.precision_lag_preparation.frame_identity,
            frame.frame_identity))
    {
        return;
    }

    bool station_selected = false;
    double raw_speed_mps = 0.0;
    double raw_speed_rate_mps2 = 0.0;
    const ObfmLagStationHoldReference& station_reference =
        snapshot_.precision_station.reference;
    if (station_reference.desired_speed_mps.has_value)
    {
        station_selected = true;
        raw_speed_mps = station_reference.desired_speed_mps.value;
    }
    else
    {
        Status phase_status{};
        obfm_longitudinal_provider_.Evaluate(
            snapshot_.precision_lag_preparation,
            input_.tactical_input,
            input_.tactical_input.obfm_longitudinal_authority,
            snapshot_.precision_longitudinal,
            phase_status);
        if (!phase_status.ok()
            || !snapshot_.precision_longitudinal.reference.evaluated
            || !SameControlFrameIdentity(
                snapshot_.precision_longitudinal.reference.frame_identity,
                frame.frame_identity))
        {
            SetPhaseCurrentSpeedEcho();
            return;
        }
        const ObfmLagLongitudinalReference& phase_reference =
            snapshot_.precision_longitudinal.reference;
        if (phase_reference.admitted
            && phase_reference.desired_speed_mps.has_value
            && phase_reference.desired_speed_rate_mps2.has_value)
        {
            raw_speed_mps = phase_reference.desired_speed_mps.value;
            raw_speed_rate_mps2 =
                phase_reference.desired_speed_rate_mps2.value;
        }
        else
        {
            return;
        }
    }

    ObfmBumplessSpeedInput shape_input{};
    shape_input.frame_identity = frame.frame_identity;
    shape_input.raw_reference_admitted = true;
    shape_input.raw_desired_speed_mps = raw_speed_mps;
    shape_input.raw_desired_speed_rate_mps2 = raw_speed_rate_mps2;
    shape_input.prior_published_speed_valid =
        snapshot_.precision_lag_preparation.same_reference_episode;
    shape_input.prior_published_speed_mps =
        snapshot_.precision_lag_preparation.previous_speed_command_mps;
    shape_input.dt_s = snapshot_.precision_lag_preparation.dt_s;
    shape_input.rate_authority = input_.tactical_input
        .current_longitudinal_evidence.tecs_configuration;
    Status shape_status{};
    ShapeObfmBumplessSpeedReference(
        shape_input,
        snapshot_.precision_bumpless,
        shape_status);
    if (!shape_status.ok() || !snapshot_.precision_bumpless.admitted)
    {
        if (station_selected)
        {
            snapshot_.precision_station.reference.desired_speed_mps =
                IntentOptionalValue<double>{};
            snapshot_.precision_station.reference.desired_speed_rate_mps2 =
                IntentOptionalValue<double>{};
        }
        SetPhaseCurrentSpeedEcho();
        return;
    }

    output.admitted = true;
    output.source = station_selected
        ? guidance::committed::G16PrecisionSpeedSource::StationHold
        : guidance::committed::G16PrecisionSpeedSource::PhaseLongitudinal;
    output.desired_speed_mps =
        snapshot_.precision_bumpless.desired_speed_mps;
    output.desired_speed_rate_mps2 = station_selected
        ? raw_speed_rate_mps2
        : snapshot_.precision_bumpless.desired_speed_rate_mps2;
    if (station_selected)
    {
        ObfmLagStationHoldReference& reference =
            snapshot_.precision_station.reference;
        reference.desired_speed_mps.has_value = true;
        reference.desired_speed_mps.value = output.desired_speed_mps;
        reference.desired_speed_rate_mps2.has_value = true;
        reference.desired_speed_rate_mps2.value =
            output.desired_speed_rate_mps2;
    }
    else
    {
        ObfmLagLongitudinalReference& reference =
            snapshot_.precision_longitudinal.reference;
        reference.admitted = true;
        reference.desired_speed_mps.has_value = true;
        reference.desired_speed_mps.value = output.desired_speed_mps;
        reference.desired_speed_rate_mps2.has_value = true;
        reference.desired_speed_rate_mps2.value =
            output.desired_speed_rate_mps2;
    }
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmG16HighPhysical(
    Status& status) noexcept
{
    status = Status{};
    if (!ObfmModeSelected() && !CurrentEffectEmploySelected())
    {
        return;
    }
    snapshot_.physical_observation_attempted = true;

    Status local_status{};
    BuildPrecisionSpeedReference(local_status);
    snapshot_.g16_evidence_status = local_status.code;
    if (CurrentEffectEmploySelected())
    {
        status = local_status;
        return;
    }
    if (local_status.ok() && snapshot_.precision_speed_ready)
    {
        snapshot_.g16_evidence_attempted = true;
        g16_evidence_provider_.Observe(
            input_.tactical_input,
            snapshot_.g16_evidence,
            local_status);
        snapshot_.g16_evidence_status = local_status.code;
        snapshot_.g16_evidence_ready = local_status.ok()
            && snapshot_.g16_evidence.valid;
    }

    snapshot_.g16_high_observation_attempted = true;
    local_status = Status{};
    g16_high_prevention_.ObserveKinematics(
        input_.tactical_input,
        snapshot_.g16_high_observation,
        local_status);
    snapshot_.g16_high_observation_status = local_status.code;
    snapshot_.g16_high_observation_ready = local_status.ok()
        && snapshot_.g16_high_observation.valid
        && SameControlFrameIdentity(
            snapshot_.g16_high_observation.frame_identity,
            snapshot_.frame_identity);
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmEmploy(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.employ_observation_attempted = true;
    snapshot_.employ_observation_ready = false;
    snapshot_.employ_admission =
        guidance::obfm::ObfmEmployAdmissionReceipt{};
    if (!ObfmModeSelected() && !CurrentEffectEmploySelected())
    {
        return;
    }
    employ_admission_provider_.Observe(
        input_.tactical_input.frame,
        snapshot_.employ_admission,
        status);
    snapshot_.employ_observation_ready = status.ok()
        && snapshot_.employ_admission.valid
        && SameControlFrameIdentity(
            snapshot_.employ_admission.frame_identity,
            snapshot_.frame_identity);
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmEmploy(
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!candidate_stage_active_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!snapshot_.employ_observation_attempted
        || !snapshot_.employ_observation_ready
        || !snapshot_.employ_admission.admitted)
    {
        return;
    }
    SelectWriter(
        guidance::obfm::ControlIntentWriterObfmEmploy,
        status);
    selected = status.ok();
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmEmploy(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !snapshot_.employ_observation_ready
        || !snapshot_.employ_admission.admitted
        || !snapshot_.precision_speed_ready
        || selection_count_ != 1U
        || selected_writer_id_
            != guidance::obfm::ControlIntentWriterObfmEmploy)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    guidance::obfm::ObfmEmployGuidanceInput employ_input{};
    employ_input.selected = true;
    employ_input.station_hold = snapshot_.precision_station.reference;
    guidance::obfm::ObfmEmployGuidanceCandidate employ_candidate{};
    guidance::obfm::BuildObfmEmployGuidanceCandidate(
        input_.tactical_input.frame,
        employ_input,
        employ_candidate,
        status);
    if (!status.ok() || !employ_candidate.valid)
    {
        RejectSelectedCandidate(
            status.ok() ? StatusCode::InvalidConfiguration : status.code,
            status);
        return;
    }

    ControlIntent base{};
    base.Clear();
    base.frame_identity = employ_candidate.frame_identity;
    base.aim_point_m = employ_candidate.aim_point_ned_m;
    base.desired_speed_mps = employ_candidate.desired_speed_mps;
    base.desired_speed_rate_mps2 =
        employ_candidate.desired_speed_rate_mps2;
    base.capture_range_des_m = employ_candidate.capture_range_des_m;
    base.behavior_id = DoctrineBehaviorId::Employ;
    base.mode_id = DoctrineModeId::Obfm;
    base.route_kind = ControlRouteKind::AimPoint;
    base.writer_id = guidance::obfm::ControlIntentWriterObfmEmploy;

    guidance::obfm::TerminalTrackingReceipt terminal{};
    guidance::obfm::EvaluateTerminalTracking(
        input_.tactical_input.frame,
        true,
        true,
        terminal,
        status);
    if (!status.ok() || !terminal.evaluated || !terminal.admitted)
    {
        RejectSelectedCandidate(
            status.ok() ? StatusCode::InvalidConfiguration : status.code,
            status);
        return;
    }

    ControlIntent final_intent{};
    guidance::obfm::ApplyTerminalTracking(
        base,
        terminal,
        final_intent,
        status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    StageBaseIntent(
        guidance::obfm::ControlIntentWriterObfmEmploy,
        final_intent,
        status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
    }
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmEntryPhysical(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.entry_established_turn_attempted = true;
    snapshot_.entry_established_turn_ready = false;
    snapshot_.entry_established_turn =
        guidance::obfm::ObfmEntryEstablishedTurnReceipt{};
    snapshot_.entry_established_turn_status = StatusCode::Ok;
    snapshot_.entry_observation_attempted = false;
    snapshot_.entry_observation_ready = false;
    snapshot_.entry_observation =
        guidance::obfm::ObfmEntryWindowObservationReceipt{};
    snapshot_.entry_observation_status = StatusCode::Ok;
    if (!ObfmModeSelected())
    {
        return;
    }

    guidance::obfm::ObfmEntryWindowObservationInput observation_input{};
    observation_input.frame_evidence_available =
        input_.tactical_input.frame.target_same_index;
    observation_input.dt_s = input_.tactical_input.frame.tau_sec;

    Status local_status{};
    entry_established_turn_observer_.Observe(
        input_.tactical_input.frame,
        observation_input,
        snapshot_.entry_established_turn,
        local_status);
    snapshot_.entry_established_turn_status = local_status.code;
    snapshot_.entry_established_turn_ready = local_status.ok()
        && snapshot_.entry_established_turn.evaluated
        && SameControlFrameIdentity(
            snapshot_.entry_established_turn.frame_identity,
            snapshot_.frame_identity);
    if (!snapshot_.entry_established_turn_ready)
    {
        // ENTRY is optional. Malformed or discontinuous local observation
        // leaves the lower precision-LAG command constructible this tick.
        return;
    }

    snapshot_.entry_observation_attempted = true;
    local_status = Status{};
    entry_window_.ObserveFromEstablishedTurn(
        input_.tactical_input.frame,
        observation_input,
        snapshot_.entry_established_turn,
        snapshot_.entry_observation,
        local_status);
    snapshot_.entry_observation_status = local_status.code;
    snapshot_.entry_observation_ready = local_status.ok()
        && snapshot_.entry_observation.evaluated
        && SameControlFrameIdentity(
            snapshot_.entry_observation.frame_identity,
            snapshot_.frame_identity);
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmEntry(
    bool& selected,
    bool& completed,
    Status& status) noexcept
{
    selected = false;
    completed = false;
    status = Status{};
    snapshot_.entry_service_attempted = true;
    snapshot_.entry_service =
        guidance::obfm::ObfmEntrySetupServiceReceipt{};
    snapshot_.entry_task = guidance::obfm::ObfmEntrySetupTaskReceipt{};
    snapshot_.entry_owner_active = entry_window_.owner_active();
    if (!candidate_stage_active_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!snapshot_.entry_observation_attempted
        || !snapshot_.entry_observation_ready)
    {
        return;
    }

    guidance::obfm::ObfmEntrySetupServiceInput service_input{};
    service_input.selector_service_reached = true;
    service_input.feature_enabled = true;
    service_input.spacing_owner_enabled = true;
    service_input.safety_evidence_available =
        input_.tactical_input.current_safety.valid;
    service_input.safety_admitted =
        input_.tactical_input.current_safety.valid
        && !input_.tactical_input.current_safety.entry_should_activate
        && !input_.tactical_input.current_safety.entry_boundary_breached;
    service_input.spacing_handoff_deferred_current_energy = false;
    entry_window_.EvaluateService(
        service_input,
        snapshot_.entry_observation,
        snapshot_.entry_service,
        status);
    if (!status.ok())
    {
        return;
    }
    completed = snapshot_.entry_service.completed_this_tick;
    if (completed || !snapshot_.entry_service.selected_result)
    {
        return;
    }

    if (!entry_window_.owner_active())
    {
        entry_window_.EnterOwner(snapshot_.entry_service, status);
        if (!status.ok())
        {
            return;
        }
        entry_longitudinal_.EnterOwner(snapshot_.entry_service, status);
        if (!status.ok())
        {
            return;
        }
    }
    else if (!entry_longitudinal_.owner_active())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    entry_window_.TickOwner(
        snapshot_.entry_service,
        snapshot_.entry_observation,
        snapshot_.entry_task,
        status);
    if (!status.ok()
        || !snapshot_.entry_task.producer_ready
        || snapshot_.entry_task.producer_count != 1U)
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }

    SelectWriter(ControlIntentWriterObfmEntrySetup, status);
    selected = status.ok();
    snapshot_.entry_owner_active = entry_window_.owner_active();
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmEntry(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmEntrySetup
        || !entry_window_.owner_active()
        || !entry_longitudinal_.owner_active())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    entry_longitudinal_.Prepare(
        true,
        snapshot_.entry_service,
        snapshot_.entry_observation,
        input_.tactical_input,
        snapshot_.entry_longitudinal_preparation,
        snapshot_.entry_longitudinal,
        status);
    if (!status.ok()
        || !snapshot_.entry_longitudinal.producer_ready
        || snapshot_.entry_longitudinal.producer_count != 1U
        || !snapshot_.entry_longitudinal.command.valid
        || !snapshot_.entry_longitudinal_preparation.valid)
    {
        RejectSelectedCandidate(
            status.ok() ? StatusCode::ObservationInvalid : status.code,
            status);
        return;
    }

    const guidance::obfm::ObfmEntrySetupCommandCandidate& command =
        snapshot_.entry_longitudinal.command;
    ControlIntent base{};
    base.Clear();
    base.frame_identity = snapshot_.frame_identity;
    base.aim_point_m = command.aim_point_ned_m;
    base.aim_point_velocity_mps.has_value = true;
    base.aim_point_velocity_mps.value = command.aim_point_velocity_ned_mps;
    base.desired_speed_mps = command.desired_speed_mps;
    base.desired_speed_rate_mps2 = command.desired_speed_rate_mps2;
    base.specific_energy_rate_bias_m2ps3 =
        command.specific_energy_rate_bias_m2ps3;
    base.path_inversion_allowed.has_value = true;
    base.path_inversion_allowed.value = command.path_inversion_allowed;
    base.capture_range_des_m = command.capture_range_des_m;
    base.behavior_id =
        DoctrineBehaviorId::ObfmEntrySetupCurrentTurnEntryWindow;
    base.mode_id = DoctrineModeId::Obfm;
    base.route_kind = ControlRouteKind::AimPoint;
    base.writer_id = ControlIntentWriterObfmEntrySetup;

    guidance::obfm::ObfmNearStationCarrotInput carrot_input{};
    carrot_input.owner_selected = true;
    carrot_input.target_track_velocity_available = true;
    carrot_input.own_position_ned_m =
        input_.tactical_input.frame.own.position_ned_m;
    carrot_input.target_track_velocity_ned_mps =
        input_.tactical_input.frame.opponent.velocity_ned_mps;
    ControlIntent final_intent{};
    guidance::obfm::ShapeObfmNearStationCarrotAimPoint(
        base,
        carrot_input,
        snapshot_.entry_carrot,
        final_intent,
        status);
    if (!status.ok())
    {
        // This shaper is optional; the already valid physical ENTRY reference
        // remains the owner command if optional carrot arithmetic is absent.
        final_intent = base;
        status = Status{};
    }
    final_intent.Validate(status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }

    entry_longitudinal_.CommitPublished(
        snapshot_.entry_longitudinal_preparation,
        true,
        final_intent.desired_speed_mps,
        status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    StageBaseIntent(ControlIntentWriterObfmEntrySetup, final_intent, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
    }
}

void StaticDoctrineObfmG16G5bOwner::CompleteObfmEntry() noexcept
{
    entry_window_.HaltOwner(
        guidance::obfm::ObfmEntrySetupHaltCause::CompletedThisTick,
        snapshot_.entry_halt);
    entry_longitudinal_.HaltOwner();
    snapshot_.entry_owner_active = false;
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmEntry() noexcept
{
    guidance::obfm::ObfmEntrySetupHaltCause cause =
        guidance::obfm::ObfmEntrySetupHaltCause::OtherPreemption;
    if (snapshot_.employ_observation_ready
        && snapshot_.employ_admission.admitted)
    {
        cause = guidance::obfm::ObfmEntrySetupHaltCause::OfficialEmployActive;
    }
    entry_window_.HaltOwner(cause, snapshot_.entry_halt);
    entry_longitudinal_.HaltOwner();
    snapshot_.entry_owner_active = false;
}

void StaticDoctrineObfmG16G5bOwner::EvaluateObfmG16HighCandidate(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    if (!snapshot_.precision_speed_ready
        || !snapshot_.g16_evidence_ready
        || !snapshot_.g16_high_observation_attempted)
    {
        return;
    }

    Status local_status{};
    g16_committed_owner_.Observe(
        snapshot_.g16_evidence,
        snapshot_.g16_owner,
        local_status);
    if (!local_status.ok() || !snapshot_.g16_owner.valid)
    {
        snapshot_.g16_owner =
            guidance::committed::G16CommittedOwnerReceipt{};
    }
    else if (snapshot_.g16_owner.event
        == guidance::committed::
            G16CommitEvent::CompletedAlreadyOutsideMaintained)
    {
        g16_committed_owner_.Reset();
        snapshot_.g16_owner =
            guidance::committed::G16CommittedOwnerReceipt{};
    }

    snapshot_.g16_high_transition_attempted = true;
    if (!snapshot_.g16_high_observation_ready)
    {
        return;
    }
    local_status = Status{};
    g16_high_prevention_.Evaluate(
        input_.tactical_input,
        snapshot_.g16_evidence,
        snapshot_.precision_speed,
        snapshot_.g16_high_transition,
        local_status);
    if (!local_status.ok()
        || !snapshot_.g16_high_transition.valid
        || !SameControlFrameIdentity(
            snapshot_.g16_high_transition.frame_identity,
            snapshot_.frame_identity))
    {
        snapshot_.g16_high_transition =
            guidance::committed::G16HighPreventionReceipt{};
        return;
    }
    g16_high_prevention_.CopySelection(
        snapshot_.frame_identity,
        snapshot_.g16_high_selection,
        local_status);
    if (!local_status.ok()
        || !snapshot_.g16_high_selection.valid
        || !SameControlFrameIdentity(
            snapshot_.g16_high_selection.frame_identity,
            snapshot_.frame_identity))
    {
        snapshot_.g16_high_selection =
            guidance::committed::G16HighSelection{};
        return;
    }
    snapshot_.g16_high_transition_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::SelectWriter(
    const std::uint32_t writer_id,
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || writer_id == ControlIntentWriterNone
        || selection_count_ != 0U
        || selected_writer_id_ != ControlIntentWriterNone)
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    selected_writer_id_ = writer_id;
    selection_count_ = 1U;
    snapshot_.selected_writer_id = writer_id;
    snapshot_.selection_count = 1U;
}

void StaticDoctrineObfmG16G5bOwner::RejectSelectedCandidate(
    const StatusCode code,
    Status& status) noexcept
{
    selected_writer_id_ = ControlIntentWriterNone;
    selection_count_ = 0U;
    staged_base_intent_ready_ = false;
    staged_base_intent_.Clear();
    snapshot_.selected_writer_id = ControlIntentWriterNone;
    snapshot_.selection_count = 0U;
    snapshot_.candidate_count = 0U;
    snapshot_.staged_base_intent.Clear();
    status.code = code == StatusCode::Ok
        ? StatusCode::InvalidConfiguration
        : code;
}

void StaticDoctrineObfmG16G5bOwner::StageBaseIntent(
    const std::uint32_t writer_id,
    const ControlIntent& intent,
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || staged_base_intent_ready_
        || writer_id == ControlIntentWriterNone
        || intent.writer_id != writer_id
        || selection_count_ != 1U
        || selected_writer_id_ != writer_id
        || !SameControlFrameIdentity(
            intent.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        snapshot_.status_code = status.code;
        return;
    }
    intent.Validate(status);
    if (!status.ok())
    {
        return;
    }
    staged_base_intent_ = intent;
    staged_base_intent_ready_ = true;
    selected_writer_id_ = ControlIntentWriterNone;
    selection_count_ = 0U;
    snapshot_.selected_writer_id = ControlIntentWriterNone;
    snapshot_.selection_count = 0U;
    snapshot_.candidate_count = 1U;
    snapshot_.staged_base_intent = intent;
}

void StaticDoctrineObfmG16G5bOwner::SelectG16Committed(
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!candidate_stage_active_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!snapshot_.g16_evidence_attempted
        || !snapshot_.g16_evidence_ready
        || !snapshot_.g16_owner.valid
        || !SameControlFrameIdentity(
            snapshot_.g16_owner.frame_identity,
            snapshot_.frame_identity))
    {
        return;
    }
    g16_committed_owner_.CopySelection(
        snapshot_.frame_identity,
        snapshot_.g16_selection,
        status);
    if (!status.ok())
    {
        return;
    }
    if (!snapshot_.g16_selection.selected)
    {
        return;
    }
    if (!snapshot_.g16_selection.command_ready
        || snapshot_.g16_selection.writer_id
            != ControlIntentWriterG16Committed)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    SelectWriter(ControlIntentWriterG16Committed, status);
    selected = status.ok();
}

void StaticDoctrineObfmG16G5bOwner::PublishG16Committed(
    Status& status) noexcept
{
    BuildAndStageWriter(ControlIntentWriterG16Committed, status);
}

void StaticDoctrineObfmG16G5bOwner::HaltG16Committed() noexcept
{
    g16_committed_owner_.HaltExecutionPreservingLifecycle();
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmG16High(
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.g16_high_transition_ready
        || !snapshot_.g16_high_selection.valid
        || !snapshot_.g16_high_selection.command_selected)
    {
        return;
    }
    if (snapshot_.g16_high_selection.high_to_lag_selected
        || snapshot_.g16_high_selection.g16_e_handoff
        || snapshot_.g16_high_selection.writer_id
            != ControlIntentWriterG16HighPrevention)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    SelectWriter(ControlIntentWriterG16HighPrevention, status);
    selected = status.ok();
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmG16High(
    Status& status) noexcept
{
    BuildAndStageWriter(ControlIntentWriterG16HighPrevention, status);
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmG16HighToLag(
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.g16_high_transition_ready
        || !snapshot_.g16_high_selection.valid
        || !snapshot_.g16_high_selection.high_to_lag_selected)
    {
        return;
    }
    if (snapshot_.g16_high_selection.command_selected
        || snapshot_.g16_high_selection.g16_e_handoff
        || snapshot_.g16_high_selection.writer_id != ControlIntentWriterNone
        || !snapshot_.g16_high_transition.high_to_lag.valid
        || snapshot_.g16_high_transition.high_to_lag.consumed)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    SelectWriter(ControlIntentWriterObfmLag, status);
    selected = status.ok();
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmLagStation(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.lag_station_observation_attempted = true;
    if (!ObfmModeSelected())
    {
        if (snapshot_.g5b_mode_reevaluation_requested)
        {
            return;
        }
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    // Several higher-priority OBFM leaves may inspect the same command-neutral
    // station receipt before falling through to LAG in this frame.  The
    // receipt is immutable after Prepare(), so a second consumer reuses it;
    // duplicate observation is not a control-contract failure.
    if (snapshot_.lag_station_observation_ready)
    {
        return;
    }
    if (!snapshot_.precision_speed_ready)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.lag_station = snapshot_.precision_station;
    if (!snapshot_.lag_station.reference.evaluated
        || !SameControlFrameIdentity(
            snapshot_.lag_station.reference.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.lag_station_observation_ready = true;
    snapshot_.lag_speed_authority =
        snapshot_.lag_station.reference.desired_speed_mps.has_value
            ? ObfmLagSpeedAuthority::StationHold
            : ObfmLagSpeedAuthority::PhaseLongitudinal;
    snapshot_.lag_speed_authority_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmLagSpeedAuthority(
    const ObfmLagSpeedAuthority authority,
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.lag_station_observation_ready
        || !snapshot_.lag_speed_authority_ready
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmLag
        || snapshot_.lag_speed_authority
            == ObfmLagSpeedAuthority::Unavailable
        || authority == ObfmLagSpeedAuthority::Unavailable)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    selected = snapshot_.lag_speed_authority == authority;
}

void StaticDoctrineObfmG16G5bOwner::PrepareObfmLagBase(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !snapshot_.lag_station_observation_ready
        || !snapshot_.lag_speed_authority_ready
        || snapshot_.lag_speed_authority
            == ObfmLagSpeedAuthority::Unavailable
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmLag
        || snapshot_.lag_base_ready
        || !snapshot_.precision_speed_ready
        || !snapshot_.precision_lag_preparation.valid
        || !SameControlFrameIdentity(
            snapshot_.precision_lag_preparation.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ObfmLagGuidanceInput lag_input{};
    lag_input.ordinary_fallback_selected = true;
    lag_input.behavior_id = DoctrineBehaviorId::Lag;
    lag_input.writer_id = ControlIntentWriterObfmLag;
    lag_input.speed_authority = snapshot_.lag_speed_authority;
    if (snapshot_.lag_speed_authority
        == ObfmLagSpeedAuthority::PhaseLongitudinal)
    {
        if (!snapshot_.precision_longitudinal.reference.evaluated
            || !SameControlFrameIdentity(
                snapshot_.precision_longitudinal.reference.frame_identity,
                snapshot_.frame_identity))
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        lag_input.longitudinal =
            snapshot_.precision_longitudinal.reference;
    }
    else
    {
        if (!snapshot_.lag_station.reference.evaluated
            || !snapshot_.lag_station.reference.desired_speed_mps.has_value)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        lag_input.station_hold = snapshot_.lag_station.reference;
    }

    obfm_lag_guidance_.BuildCandidate(
        input_.tactical_input.frame,
        snapshot_.precision_lag_preparation,
        lag_input,
        snapshot_.lag_base_intent,
        snapshot_.lag_commit,
        status);
    if (!status.ok())
    {
        return;
    }
    snapshot_.lag_commit_ready = snapshot_.lag_commit.valid;
    snapshot_.lag_base_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmLeadDiscipline(
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.lag_base_ready
        || snapshot_.lead_discipline_ready
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmLag)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    guidance::obfm::EvaluateObfmLeadDiscipline(
        input_.previous_pursuit,
        snapshot_.lead_discipline);
    if (snapshot_.lead_discipline.decision_count != 1U
        || snapshot_.lead_discipline.previous_observation_present
            != input_.previous_pursuit.previous_observation_present
        || snapshot_.lead_discipline.preserve_terminal_tracking
            == snapshot_.lead_discipline.withhold_terminal_pull)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.lead_discipline_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmTerminalTracking(
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.lag_base_ready
        || !snapshot_.lead_discipline_ready
        || snapshot_.terminal_tracking_ready
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmLag)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    guidance::obfm::EvaluateTerminalTracking(
        input_.tactical_input.frame,
        true,
        true,
        snapshot_.terminal_tracking,
        status);
    if (!status.ok())
    {
        const DogfightGeometryFrame& frame = input_.tactical_input.frame;
        const bool finite_optional_geometry =
            std::isfinite(frame.own_offense.range_m)
            && std::isfinite(frame.own_offense.phase.max_range_m)
            && FiniteVector(frame.opponent.position_ned_m)
            && FiniteVector(frame.opponent.velocity_ned_mps)
            && FiniteVector(frame.own.position_ned_m)
            && FiniteVector(frame.own.velocity_ned_mps)
            && FiniteVector(frame.own.down_ned)
            && FiniteVector(frame.own.rpy_rad)
            && FiniteMatrix(frame.own.dcm_body_to_ned);
        if (status.code == StatusCode::NonFiniteInput
            && finite_optional_geometry)
        {
            // Terminal pull is an optional overlay. Finite arithmetic loss
            // keeps the already-built writer-5 LAG command unchanged.
            snapshot_.terminal_tracking =
                guidance::obfm::TerminalTrackingReceipt{};
            snapshot_.terminal_tracking.frame_identity =
                snapshot_.frame_identity;
            snapshot_.terminal_tracking.evaluated = true;
            status = Status{};
        }
        else
        {
            return;
        }
    }
    snapshot_.terminal_tracking_ready = true;
    snapshot_.effective_terminal_tracking =
        snapshot_.terminal_tracking.admitted
        && snapshot_.lead_discipline.preserve_terminal_tracking;
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmTerminalTracking(
    bool& selected,
    Status& status) noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.lag_base_ready
        || !snapshot_.terminal_tracking_ready
        || !snapshot_.terminal_tracking.evaluated)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    selected = snapshot_.effective_terminal_tracking;
}

void StaticDoctrineObfmG16G5bOwner::BuildPublishedLagIntent(
    const bool terminal_tracking_selected,
    const ControlIntent& upstream,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    const bool expected_terminal = snapshot_.terminal_tracking.admitted
        && snapshot_.lead_discipline.preserve_terminal_tracking;
    if (!snapshot_.lead_discipline_ready
        || !snapshot_.terminal_tracking_ready
        || !snapshot_.terminal_tracking.evaluated
        || snapshot_.effective_terminal_tracking != expected_terminal
        || terminal_tracking_selected != expected_terminal)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!terminal_tracking_selected)
    {
        output = upstream;
    }
    else
    {
        guidance::obfm::ApplyTerminalTracking(
            upstream,
            snapshot_.terminal_tracking,
            output,
            status);
        if (!status.ok())
        {
            status = Status{};
            output = upstream;
        }
    }

    frame_evidence_provider_.Build(
        input_.tactical_input.frame,
        snapshot_.chase_up_frame_evidence,
        snapshot_.chase_up_frame_evidence_status);
    if (snapshot_.chase_up_frame_evidence_status
        != HabfmFrameEvidenceStatus::Built)
    {
        status = Status{};
        return;
    }
    guidance::obfm::EvaluateObfmChaseUpGuard(
        input_.tactical_input.frame,
        guidance::obfm::ObfmChaseUpBehavior::Lag,
        output,
        snapshot_.chase_up_frame_evidence.own_sustained_corner_interval,
        snapshot_.chase_up,
        status);
    if (!status.ok() || !snapshot_.chase_up.valid)
    {
        status = Status{};
        snapshot_.chase_up = guidance::obfm::ObfmChaseUpGuardReceipt{};
        return;
    }
    output = snapshot_.chase_up.candidate;
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmLag(
    const bool terminal_tracking_selected,
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || !snapshot_.lag_station_observation_ready
        || !snapshot_.lag_base_ready
        || !snapshot_.terminal_tracking_ready
        || selection_count_ != 1U
        || selected_writer_id_ != ControlIntentWriterObfmLag)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    ControlIntent final_intent{};
    BuildPublishedLagIntent(
        terminal_tracking_selected,
        snapshot_.lag_base_intent,
        final_intent,
        status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    StageBaseIntent(ControlIntentWriterObfmLag, final_intent, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    if (!snapshot_.lag_commit_ready)
    {
        RejectSelectedCandidate(StatusCode::InvalidConfiguration, status);
        return;
    }
    obfm_lag_guidance_.CommitPublished(snapshot_.lag_commit, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    snapshot_.lag_commit_ready = false;
    snapshot_.lag_commit = ObfmLagGuidanceCommit{};
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmG16HighToLag(
    const bool terminal_tracking_selected,
    Status& status) noexcept
{
    PublishObfmLag(terminal_tracking_selected, status);
    if (!status.ok())
    {
        return;
    }
    Status consume_status{};
    g16_high_prevention_.ConsumeHighToLagHandoff(
        snapshot_.frame_identity,
        snapshot_.g16_high_to_lag_commit,
        consume_status);
    if (!consume_status.ok()
        || !snapshot_.g16_high_to_lag_commit.valid
        || !snapshot_.g16_high_to_lag_commit.consumed
        || !SameControlFrameIdentity(
            snapshot_.g16_high_to_lag_commit.frame_identity,
            snapshot_.frame_identity))
    {
        RejectSelectedCandidate(
            consume_status.ok()
                ? StatusCode::InvalidConfiguration
                : consume_status.code,
            status);
    }
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmLagFallback(
    Status& status) noexcept
{
    status = Status{};
    SelectWriter(ControlIntentWriterObfmLag, status);
    if (!status.ok())
    {
        return;
    }

    ObserveObfmLagStation(status);
    if (!status.ok())
    {
        return;
    }

    bool speed_authority_selected = false;
    SelectObfmLagSpeedAuthority(
        ObfmLagSpeedAuthority::StationHold,
        speed_authority_selected,
        status);
    if (!status.ok())
    {
        return;
    }
    if (!speed_authority_selected)
    {
        SelectObfmLagSpeedAuthority(
            ObfmLagSpeedAuthority::PhaseLongitudinal,
            speed_authority_selected,
            status);
    }
    if (!status.ok() || !speed_authority_selected)
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }

    PrepareObfmLagBase(status);
    if (!status.ok())
    {
        return;
    }
    ObserveObfmLeadDiscipline(status);
    if (!status.ok())
    {
        return;
    }
    ObserveObfmTerminalTracking(status);
    if (!status.ok())
    {
        return;
    }

    bool terminal_tracking_selected = false;
    SelectObfmTerminalTracking(terminal_tracking_selected, status);
    if (!status.ok())
    {
        return;
    }
    PublishObfmLag(terminal_tracking_selected, status);
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmG5b(
    bool& selected,
    Status& status) const noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.frame_ready || !ObfmModeSelected())
    {
        return;
    }
    guidance::obfm::G5bDelayedClimbSnapshot state{};
    g5b_delayed_climb_.CopySnapshot(state);
    const guidance::committed::G16G5bCompletionHandoff& handoff =
        snapshot_.g16_owner.g5b_handoff;
    const bool current_completion = handoff.valid
        && handoff.completed_this_sample
        && SameControlFrameIdentity(
            handoff.frame_identity,
            snapshot_.frame_identity)
        && handoff.production_evidence.valid
        && SameControlFrameIdentity(
            handoff.production_evidence.frame_identity,
            snapshot_.frame_identity);
    selected = state.active || current_completion;
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmG5b(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.g5b_observation_attempted = true;
    bool selected = false;
    SelectObfmG5b(selected, status);
    if (!status.ok() || !selected || snapshot_.g5b_observation_ready)
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }

    guidance::obfm::G5bDelayedClimbSnapshot state{};
    g5b_delayed_climb_.CopySnapshot(state);
    if (!state.active)
    {
        g5b_delayed_climb_.Enter(
            snapshot_.g16_owner.g5b_handoff,
            status);
        if (!status.ok())
        {
            return;
        }
    }
    if (!snapshot_.g16_evidence_ready
        || !SameControlFrameIdentity(
            snapshot_.g16_evidence.frame_identity,
            snapshot_.frame_identity))
    {
        g5b_delayed_climb_.Halt(snapshot_.g5b_halt);
        g16_committed_owner_.Reset();
        g16_high_prevention_.HaltExecutionPreservingObservation();
        snapshot_.g5b_terminal_fallthrough = true;
        return;
    }
    if (!input_.g5b_safety.valid
        || !SameControlFrameIdentity(
            input_.g5b_safety.frame_identity,
            snapshot_.frame_identity)
        || !input_.g5b_speed_floor.valid
        || !SameControlFrameIdentity(
            input_.g5b_speed_floor.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    g5b_delayed_climb_.Observe(
        snapshot_.g16_evidence,
        input_.g5b_safety,
        input_.g5b_speed_floor,
        snapshot_.g5b_observation,
        status);
    if (!status.ok())
    {
        return;
    }
    g5b_delayed_climb_.Select(
        snapshot_.g5b_observation,
        snapshot_.g5b_selection,
        status);
    if (!status.ok()
        || !snapshot_.g5b_selection.valid
        || !SameControlFrameIdentity(
            snapshot_.g5b_selection.frame_identity,
            snapshot_.frame_identity))
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }
    snapshot_.g5b_observation_ready = true;
}

void StaticDoctrineObfmG16G5bOwner::CheckObfmG5bNormalTerminalFallthrough(
    bool& fallthrough,
    Status& status) const noexcept
{
    fallthrough = false;
    status = Status{};
    if (!snapshot_.frame_ready
        || !SameControlFrameIdentity(
            snapshot_.frame_identity,
            input_.tactical_input.frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    fallthrough = snapshot_.g5b_terminal_fallthrough
        && !snapshot_.g5b_observation_ready;
}

void StaticDoctrineObfmG16G5bOwner::SelectObfmG5bBranch(
    const guidance::obfm::G5bSelectedBranch branch,
    bool& selected,
    Status& status) const noexcept
{
    selected = false;
    status = Status{};
    if (!snapshot_.g5b_observation_ready
        || !snapshot_.g5b_selection.valid
        || branch == guidance::obfm::G5bSelectedBranch::Invalid)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    selected = snapshot_.g5b_selection.selected_branch == branch;
}

void StaticDoctrineObfmG16G5bOwner::PublishObfmG5b(
    const guidance::obfm::G5bSelectedBranch branch,
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.g5b_observation_ready
        || !snapshot_.g5b_selection.valid
        || !snapshot_.g5b_selection.command_task
        || snapshot_.g5b_selection.selected_branch != branch
        || (branch != guidance::obfm::G5bSelectedBranch::Extend
            && branch != guidance::obfm::G5bSelectedBranch::ZoomEntry
            && branch != guidance::obfm::G5bSelectedBranch::ZoomClimb))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    SelectWriter(
        guidance::obfm::ControlIntentWriterG5bDelayedClimb,
        status);
    if (status.ok())
    {
        BuildAndStageWriter(
            guidance::obfm::ControlIntentWriterG5bDelayedClimb,
            status);
    }
}

void StaticDoctrineObfmG16G5bOwner::CompleteObfmG5b(
    Status& status) noexcept
{
    status = Status{};
    g5b_delayed_climb_.CompleteTask(
        snapshot_.g5b_observation,
        snapshot_.g5b_selection,
        snapshot_.g5b_task,
        status);
    if (!status.ok())
    {
        return;
    }
    g5b_delayed_climb_.Halt(snapshot_.g5b_halt);
    if (!snapshot_.g5b_halt.valid
        || !snapshot_.g5b_halt.was_active
        || !snapshot_.g5b_halt.terminal
        || !snapshot_.g5b_halt.completed
        || snapshot_.g5b_halt.preempted
        || !input_.mode_decision.valid
        || !input_.mode_decision.mode.has_value)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    g16_committed_owner_.Reset();
    g16_high_prevention_.HaltExecutionPreservingObservation();
    snapshot_.effective_mode = input_.mode_decision.mode;
    snapshot_.g5b_mode_reevaluation_requested = true;
    snapshot_.g5b_terminal_fallthrough = true;
}

void StaticDoctrineObfmG16G5bOwner::ReleaseObfmG5b(
    Status& status) noexcept
{
    status = Status{};
    g5b_delayed_climb_.ReleaseTask(
        snapshot_.g5b_observation,
        snapshot_.g5b_selection,
        snapshot_.g5b_task,
        status);
    if (!status.ok())
    {
        return;
    }
    g5b_delayed_climb_.Halt(snapshot_.g5b_halt);
    snapshot_.g5b_terminal_fallthrough = true;
}

void StaticDoctrineObfmG16G5bOwner::FailObfmG5bInvalid(
    Status& status) noexcept
{
    status = Status{};
    if (!snapshot_.g5b_terminal_fallthrough)
    {
        g5b_delayed_climb_.Halt(snapshot_.g5b_halt);
        snapshot_.g5b_terminal_fallthrough = true;
    }
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmG5b(
    const guidance::obfm::G5bSelectedBranch halted_branch) noexcept
{
    const guidance::obfm::G5bSelectedBranch selected_branch =
        snapshot_.g5b_selection.selected_branch;
    const bool selected_inner_command =
        selected_branch == guidance::obfm::G5bSelectedBranch::ZoomEntry
        || selected_branch == guidance::obfm::G5bSelectedBranch::Extend
        || selected_branch == guidance::obfm::G5bSelectedBranch::ZoomClimb;
    const bool inner_phase_handoff = snapshot_.g5b_observation_ready
        && snapshot_.g5b_selection.valid
        && snapshot_.g5b_selection.command_task
        && selected_inner_command
        && selected_branch != halted_branch;
    if (!snapshot_.g5b_terminal_fallthrough && !inner_phase_handoff)
    {
        g5b_delayed_climb_.Halt(snapshot_.g5b_halt);
        if (snapshot_.g5b_halt.valid
            && snapshot_.g5b_halt.was_active
            && snapshot_.g5b_halt.preempted)
        {
            g16_committed_owner_.Reset();
            g16_high_prevention_.HaltExecutionPreservingObservation();
        }
    }
}

void StaticDoctrineObfmG16G5bOwner::BuildObfmSpacingServiceInput(
    guidance::obfm::ObfmSpacingOwnerServiceInput& output) const noexcept
{
    output = guidance::obfm::ObfmSpacingOwnerServiceInput{};
    output.selector_branch_reached = true;
    output.frame_evidence_declared_ready = snapshot_.frame_ready;
    output.feature_enabled = true;
    output.current_energy_projection_required = false;

    const guidance::obfm::G5bSafetyAdmissionReceipt strict =
        guidance::obfm::EvaluateG5bSafetyAdmission(
            input_.tactical_input.frame,
            input_.g5b_safety,
            false,
            false);
    const guidance::obfm::G5bSafetyAdmissionReceipt running =
        guidance::obfm::EvaluateG5bSafetyAdmission(
            input_.tactical_input.frame,
            input_.g5b_safety,
            false,
            true);
    output.safety.frame_identity = snapshot_.frame_identity;
    output.safety.strict_entry_evaluated = strict.evaluated;
    output.safety.strict_entry_admitted = strict.admitted;
    output.safety.running_fault_only_evaluated = running.evaluated;
    output.safety.running_fault_only_admitted = running.admitted;

    const auto& longitudinal =
        input_.tactical_input.current_longitudinal_evidence;
    output.flight_path_gamma_limit_available = longitudinal.valid
        && longitudinal.flight_path_gamma_limit_valid
        && std::isfinite(longitudinal.flight_path_gamma_limit_rad)
        && longitudinal.flight_path_gamma_limit_rad > 0.0
        && longitudinal.flight_path_gamma_limit_rad
            < 0.5 * constants::Pi;
    output.flight_path_gamma_limit_rad =
        output.flight_path_gamma_limit_available
            ? longitudinal.flight_path_gamma_limit_rad
            : 0.0;
    output.previous_energy_authority =
        input_.tactical_input.obfm_longitudinal_authority.previous_energy;
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmSpacingEmployPreemption(
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!obfm_spacing_task_active_ || !CurrentEffectEmploySelected())
    {
        return;
    }
    snapshot_.spacing_service_attempted = true;
    guidance::obfm::ObfmSpacingOwnerServiceInput service_input{};
    BuildObfmSpacingServiceInput(service_input);
    obfm_spacing_owner_.ObserveService(
        input_.tactical_input.frame,
        service_input,
        snapshot_.spacing_service,
        status);
    if (!status.ok())
    {
        return;
    }
    snapshot_.spacing_service_ready =
        snapshot_.spacing_service.service_evaluated
        && snapshot_.spacing_service.selection_finalized
        && SameControlFrameIdentity(
            snapshot_.spacing_service.frame_identity,
            snapshot_.frame_identity);
    snapshot_.spacing_owner_active = obfm_spacing_task_active_;
}

void StaticDoctrineObfmG16G5bOwner::EvaluateObfmSpacing(
    bool& selected,
    bool& completed,
    bool& released,
    Status& status) noexcept
{
    selected = false;
    completed = false;
    released = false;
    status = Status{};
    snapshot_.spacing_service_attempted = true;
    snapshot_.spacing_owner_active = obfm_spacing_task_active_;
    if (!candidate_stage_active_ || !ObfmModeSelected())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    guidance::obfm::ObfmSpacingOwnerServiceInput service_input{};
    BuildObfmSpacingServiceInput(service_input);

    obfm_spacing_owner_.ObserveService(
        input_.tactical_input.frame,
        service_input,
        snapshot_.spacing_service,
        status);
    if (!status.ok())
    {
        return;
    }
    snapshot_.spacing_service_ready =
        snapshot_.spacing_service.service_evaluated
        && snapshot_.spacing_service.selection_finalized
        && SameControlFrameIdentity(
            snapshot_.spacing_service.frame_identity,
            snapshot_.frame_identity);
    if (!snapshot_.spacing_service_ready)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!snapshot_.spacing_service.selected_result)
    {
        snapshot_.spacing_owner_active = obfm_spacing_task_active_;
        return;
    }

    obfm_spacing_owner_.EvaluateDecorator(
        true,
        snapshot_.spacing_service,
        snapshot_.spacing_selection,
        status);
    if (!status.ok())
    {
        return;
    }
    if (!snapshot_.spacing_selection.selected)
    {
        return;
    }
    if (snapshot_.spacing_selection.selection_count != 1U)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!obfm_spacing_task_active_)
    {
        obfm_spacing_owner_.EnterTask(
            snapshot_.spacing_service,
            snapshot_.spacing_selection,
            status);
        if (!status.ok())
        {
            return;
        }
    }

    guidance::obfm::ObfmSpacingOwnerTaskInput task_input{};
    task_input.flight_path_gamma_limit_available =
        service_input.flight_path_gamma_limit_available;
    task_input.flight_path_gamma_limit_rad =
        service_input.flight_path_gamma_limit_rad;
    task_input.previous_energy_authority =
        service_input.previous_energy_authority;
    obfm_spacing_owner_.TickTask(
        input_.tactical_input.frame,
        snapshot_.spacing_service,
        task_input,
        snapshot_.spacing_task,
        status);
    if (!status.ok())
    {
        return;
    }
    if (snapshot_.spacing_task.task_completed)
    {
        completed = true;
        return;
    }
    if (snapshot_.spacing_task.release_required)
    {
        released = true;
        return;
    }
    if (!snapshot_.spacing_task.candidate_valid)
    {
        return;
    }
    if (!snapshot_.spacing_task.task_active
        || snapshot_.spacing_task.candidate_count != 1U
        || !SameControlFrameIdentity(
            snapshot_.spacing_task.frame_identity,
            snapshot_.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    SelectWriter(ControlIntentWriterObfmSpacing, status);
    if (!status.ok())
    {
        return;
    }
    ControlIntent intent{};
    BuildSpacingIntent(
        snapshot_.frame_identity,
        snapshot_.spacing_task,
        intent,
        status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    StageBaseIntent(ControlIntentWriterObfmSpacing, intent, status);
    if (!status.ok())
    {
        return;
    }
    obfm_spacing_task_active_ = true;
    snapshot_.spacing_owner_active = true;
    snapshot_.spacing_candidate_ready = true;
    selected = true;
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmSpacing() noexcept
{
    if (!obfm_spacing_task_active_)
    {
        return;
    }
    const bool official_employ_preemption =
        snapshot_.spacing_service.reason
            == guidance::obfm::ObfmSpacingOwnerReason::
                OfficialEmployAvailable;
    Status status{};
    obfm_spacing_owner_.HaltTask(
        official_employ_preemption,
        snapshot_.spacing_halt,
        status);
    if (status.ok())
    {
        obfm_spacing_task_active_ = false;
        snapshot_.spacing_owner_active = false;
    }
    else
    {
        snapshot_.status_code = status.code;
    }
}

void StaticDoctrineObfmG16G5bOwner::EvaluateObfmG3RollCounter(
    bool& selected,
    bool& released,
    Status& status) noexcept
{
    selected = false;
    released = false;
    status = Status{};
    snapshot_.g3_roll_counter = guidance::obfm::G3RollCounterReceipt{};
    if (!candidate_stage_active_ || !ObfmModeSelected())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ControlIntent command{};
    g3_roll_counter_owner_.Evaluate(
        input_.tactical_input.frame,
        command,
        snapshot_.g3_roll_counter);
    released = snapshot_.g3_roll_counter.released;
    if (!snapshot_.g3_roll_counter.selected)
    {
        return;
    }

    Status command_status{};
    command.Validate(command_status);
    if (!command_status.ok()
        || command.writer_id != ControlIntentWriterG3CounterBarrel
        || !SameControlFrameIdentity(
            command.frame_identity,
            snapshot_.frame_identity))
    {
        // This optional tactical candidate must never erase the current-frame
        // lower OBFM command.  Drop only the G3 event and continue the selector.
        g3_roll_counter_owner_.Halt();
        snapshot_.g3_roll_counter = guidance::obfm::G3RollCounterReceipt{};
        return;
    }

    SelectWriter(command.writer_id, status);
    if (!status.ok())
    {
        return;
    }
    StageBaseIntent(command.writer_id, command, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    selected = true;
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmG3RollCounter() noexcept
{
    g3_roll_counter_owner_.HaltCounterBarrel();
    snapshot_.g3_roll_counter = guidance::obfm::G3RollCounterReceipt{};
}

void StaticDoctrineObfmG16G5bOwner::
EvaluateObfmG3CounterRollingScissors(
    bool& selected,
    bool& released,
    Status& status) noexcept
{
    selected = false;
    released = false;
    status = Status{};
    snapshot_.g3_counter_rolling_scissors =
        guidance::obfm::G3CounterRollingScissorsReceipt{};
    if (!candidate_stage_active_ || !ObfmModeSelected())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ControlIntent command{};
    g3_roll_counter_owner_.EvaluateCounterRollingScissors(
        input_.tactical_input.frame,
        command,
        snapshot_.g3_counter_rolling_scissors);
    released = snapshot_.g3_counter_rolling_scissors.released;
    if (!snapshot_.g3_counter_rolling_scissors.selected)
    {
        return;
    }

    Status command_status{};
    command.Validate(command_status);
    if (!command_status.ok()
        || command.writer_id
            != ControlIntentWriterG3CounterRollingScissors
        || !SameControlFrameIdentity(
            command.frame_identity,
            snapshot_.frame_identity))
    {
        g3_roll_counter_owner_.HaltCounterRollingScissors();
        snapshot_.g3_counter_rolling_scissors =
            guidance::obfm::G3CounterRollingScissorsReceipt{};
        return;
    }

    SelectWriter(command.writer_id, status);
    if (!status.ok())
    {
        return;
    }
    StageBaseIntent(command.writer_id, command, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    selected = true;
}

void StaticDoctrineObfmG16G5bOwner::
HaltObfmG3CounterRollingScissors() noexcept
{
    g3_roll_counter_owner_.HaltCounterRollingScissors();
    snapshot_.g3_counter_rolling_scissors =
        guidance::obfm::G3CounterRollingScissorsReceipt{};
}

void StaticDoctrineObfmG16G5bOwner::EvaluateObfmG3Scissors(
    bool& selected,
    bool& released,
    Status& status) noexcept
{
    selected = false;
    released = false;
    status = Status{};
    snapshot_.g3_scissors = guidance::obfm::G3ScissorsReceipt{};
    if (!candidate_stage_active_ || !ObfmModeSelected())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ControlIntent command{};
    g3_scissors_owner_.Evaluate(
        input_.tactical_input.frame,
        command,
        snapshot_.g3_scissors);
    released = snapshot_.g3_scissors.released;
    if (!snapshot_.g3_scissors.selected)
    {
        return;
    }

    Status command_status{};
    command.Validate(command_status);
    if (!command_status.ok()
        || command.writer_id != ControlIntentWriterG3Scissors
        || !SameControlFrameIdentity(
            command.frame_identity,
            snapshot_.frame_identity))
    {
        g3_scissors_owner_.Halt();
        snapshot_.g3_scissors = guidance::obfm::G3ScissorsReceipt{};
        return;
    }

    SelectWriter(command.writer_id, status);
    if (!status.ok())
    {
        return;
    }
    StageBaseIntent(command.writer_id, command, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    selected = true;
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmG3Scissors() noexcept
{
    g3_scissors_owner_.Halt();
    snapshot_.g3_scissors = guidance::obfm::G3ScissorsReceipt{};
}

void StaticDoctrineObfmG16G5bOwner::ObserveObfmApexPhysical(
    Status& status) noexcept
{
    status = Status{};
    snapshot_.apex_observation_attempted = true;
    snapshot_.apex_observation_ready = false;
    snapshot_.apex_service =
        guidance::obfm::ObfmApexDisplacementServiceReceipt{};
    if (!ObfmModeSelected())
    {
        return;
    }

    const runtime::TacticalCommandBuildInput& tactical =
        input_.tactical_input;
    const control::route5::CommandEnvelope& envelope =
        tactical.current_envelope;
    guidance::obfm::ObfmApexDisplacementServiceInput service_input{};
    service_input.selector_service_reached = true;
    service_input.frame_evidence_declared_ready =
        tactical.frame.target_same_index;
    service_input.feature_enabled = true;
    if (tactical.previous_control_feedback.valid)
    {
        if (tactical.previous_control_feedback.writer_id
            == ControlIntentWriterG16HighPrevention)
        {
            service_input.climb_owner =
                guidance::obfm::ObfmApexClimbOwner::G16High;
        }
        else if (tactical.previous_control_feedback.writer_id
                 == ControlIntentWriterObfmSpacing)
        {
            service_input.climb_owner =
                guidance::obfm::ObfmApexClimbOwner::SpacingArrest;
        }
    }

    // Direct-entry reachability consumes the already-admitted physical Nz
    // envelope.  Fixed containment bounds remain available to downstream FCS
    // clipping but are not promoted to measured turn capability here.
    double turn_capability_n_g = envelope.nz_feasible_roll_g;
    if (!std::isfinite(turn_capability_n_g)
        || !(turn_capability_n_g > 1.0))
    {
        turn_capability_n_g = envelope.nz_feasible_g;
    }
    service_input.turn_capability_available = envelope.valid
        && tactical.current_physical_envelope_available
        && std::isfinite(turn_capability_n_g)
        && turn_capability_n_g > 1.0;
    service_input.turn_capability_n_g = turn_capability_n_g;
    service_input.lag_point_available =
        snapshot_.precision_lag_preparation.valid
        && FiniteVector(
            snapshot_.precision_lag_preparation.current_reference_point_ned_m);
    service_input.lag_point_ned_m =
        snapshot_.precision_lag_preparation.current_reference_point_ned_m;
    service_input.preferred_side_available =
        snapshot_.g16_evidence_ready
        && snapshot_.g16_evidence.selected_egress_side_resolved;
    service_input.preferred_side_sign =
        snapshot_.g16_evidence.selected_egress_side_sign;

    obfm_apex_displacement_.ObserveService(
        tactical.frame,
        service_input,
        snapshot_.apex_service,
        status);
    snapshot_.apex_observation_ready = status.ok()
        && snapshot_.apex_service.service_evaluated
        && SameControlFrameIdentity(
            snapshot_.apex_service.frame_identity,
            snapshot_.frame_identity);
    snapshot_.apex_owner_active = obfm_apex_task_active_;
}

void StaticDoctrineObfmG16G5bOwner::EvaluateObfmApex(
    bool& selected,
    bool& released,
    Status& status) noexcept
{
    selected = false;
    released = false;
    status = Status{};
    snapshot_.apex_selection =
        guidance::obfm::ObfmApexDisplacementSelection{};
    snapshot_.apex_task = guidance::obfm::ObfmApexDisplacementTaskReceipt{};
    if (!candidate_stage_active_ || !ObfmModeSelected())
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!snapshot_.apex_observation_attempted
        || !snapshot_.apex_observation_ready)
    {
        if (obfm_apex_task_active_)
        {
            obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
            obfm_apex_task_active_ = false;
            snapshot_.apex_owner_active = false;
            released = true;
        }
        return;
    }

    Status local_status{};
    obfm_apex_displacement_.EvaluateDecorator(
        true,
        snapshot_.apex_service,
        snapshot_.apex_selection,
        local_status);
    if (!local_status.ok() || !snapshot_.apex_selection.selected)
    {
        if (obfm_apex_task_active_)
        {
            obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
            obfm_apex_task_active_ = false;
            snapshot_.apex_owner_active = false;
            released = true;
        }
        return;
    }

    if (!snapshot_.precision_speed_ready)
    {
        if (obfm_apex_task_active_)
        {
            obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
            obfm_apex_task_active_ = false;
            snapshot_.apex_owner_active = false;
            released = true;
        }
        return;
    }
    if (!obfm_apex_task_active_)
    {
        obfm_apex_displacement_.EnterTask(
            snapshot_.apex_service,
            snapshot_.apex_selection,
            local_status);
        if (!local_status.ok())
        {
            return;
        }
        obfm_apex_task_active_ = true;
    }

    const double lateral_accel =
        snapshot_.apex_service.reference.available_lateral_accel_mps2;
    const double total_load_n_g = std::sqrt(
        1.0 + lateral_accel * lateral_accel
            / (constants::StandardGravityMps2
               * constants::StandardGravityMps2));
    guidance::obfm::ObfmApexDisplacementTaskInput task_input{};
    task_input.safety_sample_available =
        input_.tactical_input.current_safety.valid
        && !input_.tactical_input.current_safety.entry_should_activate
        && !input_.tactical_input.current_safety.entry_boundary_breached;
    task_input.desired_speed_mps =
        snapshot_.precision_speed.desired_speed_mps;
    task_input.desired_speed_rate_mps2 =
        snapshot_.precision_speed.desired_speed_rate_mps2;
    task_input.total_load_factor_limit_g = total_load_n_g;
    task_input.capture_range_des_m =
        input_.tactical_input.frame.own_offense.phase.max_range_m;
    obfm_apex_displacement_.TickTask(
        input_.tactical_input.frame,
        snapshot_.apex_service,
        task_input,
        snapshot_.apex_task,
        local_status);
    if (!local_status.ok()
        || !snapshot_.apex_task.candidate_valid
        || snapshot_.apex_task.candidate_count != 1U)
    {
        obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
        obfm_apex_task_active_ = false;
        snapshot_.apex_owner_active = false;
        released = true;
        return;
    }

    const guidance::obfm::ObfmApexDisplacementCommand& command =
        snapshot_.apex_task.candidate;
    ControlIntent intent{};
    intent.Clear();
    intent.frame_identity = snapshot_.frame_identity;
    intent.aim_point_m = command.aim_point_ned_m;
    intent.desired_speed_mps = command.desired_speed_mps;
    intent.desired_speed_rate_mps2 = command.desired_speed_rate_mps2;
    intent.path_inversion_allowed.has_value = true;
    intent.path_inversion_allowed.value = command.path_inversion_allowed;
    intent.capture_range_des_m = command.capture_range_des_m;
    intent.total_load_factor_limit_g.has_value = true;
    intent.total_load_factor_limit_g.value =
        command.total_load_factor_limit_g;
    intent.behavior_id = DoctrineBehaviorId::ObfmApexDisplacement;
    intent.mode_id = DoctrineModeId::Obfm;
    intent.route_kind = ControlRouteKind::AimPoint;
    intent.writer_id = ControlIntentWriterObfmApexDisplacement;
    intent.Validate(local_status);
    if (!local_status.ok())
    {
        obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
        obfm_apex_task_active_ = false;
        snapshot_.apex_owner_active = false;
        released = true;
        return;
    }

    SelectWriter(ControlIntentWriterObfmApexDisplacement, status);
    if (!status.ok())
    {
        return;
    }
    StageBaseIntent(
        ControlIntentWriterObfmApexDisplacement, intent, status);
    if (!status.ok())
    {
        RejectSelectedCandidate(status.code, status);
        return;
    }
    selected = true;
    snapshot_.apex_owner_active = true;
}

void StaticDoctrineObfmG16G5bOwner::HaltObfmApex() noexcept
{
    obfm_apex_displacement_.HaltTask(snapshot_.apex_halt);
    obfm_apex_task_active_ = false;
    snapshot_.apex_owner_active = false;
}

void StaticDoctrineObfmG16G5bOwner::ResolveTacticalModeChild(
    std::size_t& child_index,
    Status& status) const noexcept
{
    child_index = 3U;
    status = Status{};
    if (!snapshot_.frame_ready || !snapshot_.effective_mode.has_value)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    switch (snapshot_.effective_mode.value)
    {
    case guidance::doctrine::TacticalMode::Obfm:
        child_index = 0U;
        return;
    case guidance::doctrine::TacticalMode::Habfm:
        child_index = 1U;
        return;
    case guidance::doctrine::TacticalMode::Dbfm:
        child_index = 2U;
        return;
    default:
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
}

void StaticDoctrineObfmG16G5bOwner::BuildAndStageWriter(
    const std::uint32_t writer_id,
    Status& status) noexcept
{
    status = Status{};
    if (!candidate_stage_active_
        || selection_count_ != 1U
        || selected_writer_id_ != writer_id)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    ControlIntent intent{};
    Status writer_status{};
    if (writer_id == ControlIntentWriterG16Committed)
    {
        g16_committed_owner_.BuildCandidate(
            snapshot_.g16_evidence,
            intent,
            writer_status);
    }
    else if (writer_id == ControlIntentWriterG16HighPrevention)
    {
        g16_high_prevention_.BuildCandidate(
            snapshot_.frame_identity,
            intent,
            writer_status);
    }
    else if (writer_id
        == guidance::obfm::ControlIntentWriterG5bDelayedClimb)
    {
        switch (snapshot_.g5b_selection.selected_branch)
        {
        case guidance::obfm::G5bSelectedBranch::Extend:
            g5b_delayed_climb_.BuildExtendTask(
                snapshot_.g16_evidence,
                snapshot_.g5b_observation,
                snapshot_.g5b_selection,
                intent,
                snapshot_.g5b_task,
                writer_status);
            break;
        case guidance::obfm::G5bSelectedBranch::ZoomEntry:
            g5b_delayed_climb_.BuildZoomEntryTask(
                snapshot_.g16_evidence,
                input_.g5b_safety,
                snapshot_.g5b_observation,
                snapshot_.g5b_selection,
                intent,
                snapshot_.g5b_task,
                writer_status);
            break;
        case guidance::obfm::G5bSelectedBranch::ZoomClimb:
            g5b_delayed_climb_.BuildZoomClimbTask(
                snapshot_.g16_evidence,
                input_.g5b_safety,
                snapshot_.g5b_observation,
                snapshot_.g5b_selection,
                intent,
                snapshot_.g5b_task,
                writer_status);
            break;
        default:
            writer_status.code = StatusCode::InvalidConfiguration;
            break;
        }
        if (writer_status.ok()
            && (!snapshot_.g5b_task.valid
                || !snapshot_.g5b_task.command_ready
                || snapshot_.g5b_task.branch
                    != snapshot_.g5b_selection.selected_branch
                || !SameControlFrameIdentity(
                    snapshot_.g5b_task.frame_identity,
                    snapshot_.frame_identity)))
        {
            writer_status.code = StatusCode::InvalidConfiguration;
            intent.Clear();
        }
    }
    else
    {
        writer_status.code = StatusCode::InvalidArgument;
    }

    if (writer_status.code != StatusCode::Ok)
    {
        RejectSelectedCandidate(writer_status.code, status);
        return;
    }
    if (intent.writer_id != writer_id
        || !SameControlFrameIdentity(
            intent.frame_identity,
            snapshot_.frame_identity))
    {
        RejectSelectedCandidate(StatusCode::InvalidConfiguration, status);
        return;
    }
    intent.Validate(writer_status);
    if (!writer_status.ok())
    {
        RejectSelectedCandidate(writer_status.code, status);
        return;
    }
    StageBaseIntent(writer_id, intent, status);
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
