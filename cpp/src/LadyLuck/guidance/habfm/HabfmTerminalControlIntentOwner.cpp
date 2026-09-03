#include "LadyLuck/guidance/habfm/HabfmTerminalControlIntentOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

constexpr double kOfficialCrashFloorM = 1000.0 * 0.3048;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

void HabfmEntrySide(
    const LadyLuck::DogfightGeometryFrame& frame,
    std::int32_t& output,
    LadyLuck::Status& status) noexcept
{
    output = 0;
    status = LadyLuck::Status{};
    if (!FiniteVector(frame.own.nose_ned)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }

    LadyLuck::Vector3 nose = frame.own.nose_ned;
    const LadyLuck::Vector3 line_of_sight_3d{{
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0],
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1],
        frame.opponent.position_ned_m[2] - frame.own.position_ned_m[2]}};
    LadyLuck::Vector3 line_of_sight = line_of_sight_3d;
    nose[2] = 0.0;
    line_of_sight[2] = 0.0;
    const double nose_magnitude = VectorNorm(nose);
    const double los_magnitude = VectorNorm(line_of_sight);
    if (!std::isfinite(nose_magnitude) || !std::isfinite(los_magnitude))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (nose_magnitude >= LadyLuck::constants::Tiny
        && los_magnitude >= LadyLuck::constants::Tiny)
    {
        nose[0] /= nose_magnitude;
        nose[1] /= nose_magnitude;
        line_of_sight[0] /= los_magnitude;
        line_of_sight[1] /= los_magnitude;
        const double cross_down =
            nose[0] * line_of_sight[1] - nose[1] * line_of_sight[0];
        output = cross_down >= 0.0 ? 1 : -1;
        return;
    }

    if (!FiniteVector(frame.own.down_ned))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    LadyLuck::Vector3 effective_nose = frame.own.nose_ned;
    double nose_3d_magnitude = VectorNorm(effective_nose);
    const double los_3d_magnitude = VectorNorm(line_of_sight_3d);
    if (!std::isfinite(nose_3d_magnitude)
        || !std::isfinite(los_3d_magnitude))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (nose_3d_magnitude < LadyLuck::constants::Tiny)
    {
        effective_nose = frame.own.velocity_ned_mps;
        nose_3d_magnitude = VectorNorm(effective_nose);
        if (!std::isfinite(nose_3d_magnitude))
        {
            status.code = LadyLuck::StatusCode::NonFiniteInput;
            return;
        }
    }
    if (los_3d_magnitude < LadyLuck::constants::Tiny)
    {
        // Coincident finite positions have no defined left/right geometry.
        // A stable deterministic side preserves a current-frame command and
        // the active lifecycle will retain it on subsequent ticks.
        output = 1;
        return;
    }
    if (nose_3d_magnitude < LadyLuck::constants::Tiny)
    {
        output = 1;
        return;
    }
    const LadyLuck::Vector3 cross{{
        effective_nose[1] * line_of_sight_3d[2]
            - effective_nose[2] * line_of_sight_3d[1],
        effective_nose[2] * line_of_sight_3d[0]
            - effective_nose[0] * line_of_sight_3d[2],
        effective_nose[0] * line_of_sight_3d[1]
            - effective_nose[1] * line_of_sight_3d[0]}};
    const double cross_about_down =
        cross[0] * frame.own.down_ned[0]
        + cross[1] * frame.own.down_ned[1]
        + cross[2] * frame.own.down_ned[2];
    if (!std::isfinite(cross_about_down))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    output = cross_about_down >= 0.0 ? 1 : -1;
}

bool SameFrame(
    const LadyLuck::ControlFrameIdentity& left,
    const LadyLuck::ControlFrameIdentity& right) noexcept
{
    return LadyLuck::SameControlFrameIdentity(left, right);
}

} // namespace

namespace LadyLuck
{

HabfmTerminalControlIntentOwner::HabfmTerminalControlIntentOwner() noexcept
{
    Reset();
}

void HabfmTerminalControlIntentOwner::Reset() noexcept
{
    frame_evidence_provider_.Reset();
    state_ = HabfmTerminalControlIntentState{};
    state_.merge_evidence_provider.Reset();
    state_.active_core.ResetEpisode();
    state_.engage_decision_latch.Reset();
    state_.neutral_cue_streak = 0U;
    ++generation_;
}

void HabfmTerminalControlIntentOwner::HaltLeg(
    const bool selection_ready) noexcept
{
    if (selection_ready)
    {
        return;
    }
    state_.active_core.ResetLeg();
    state_.merge_evidence_provider.Reset();
    state_.neutral_cue_streak = 0U;
    ++generation_;
}

void HabfmTerminalControlIntentOwner::CopyState(
    HabfmTerminalControlIntentState& output) const noexcept
{
    output = state_;
}

void HabfmTerminalControlIntentOwner::RestoreState(
    const HabfmTerminalControlIntentState& input) noexcept
{
    state_ = input;
    ++generation_;
}

void HabfmTerminalControlIntentOwner::Observe(
    const HabfmTerminalControlIntentInput& input,
    HabfmTerminalControlIntentObservation& output,
    Status& status) const noexcept
{
    output = HabfmTerminalControlIntentObservation{};
    status = Status{};
    output.frame_identity = input.frame.frame_identity;
    output.evaluated = true;

    EvaluateHabfmCommandGeometry(
        input.frame,
        output.command_geometry,
        status);
    if (!status.ok() || !output.command_geometry.valid)
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }
    if (!output.command_geometry.available)
    {
        output.reason =
            HabfmTerminalControlIntentReason::CommandGeometryUnavailable;
        return;
    }

    frame_evidence_provider_.Build(
        input.frame,
        output.frame_evidence,
        output.frame_evidence_status);
    if (output.frame_evidence_status != HabfmFrameEvidenceStatus::Built)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    HabfmObservationInputs observation_inputs{};
    observation_inputs.adversary_corner_speed_lower_mps.has_value =
        output.frame_evidence.adversary_corner_interval.lower_mps.has_value;
    observation_inputs.adversary_corner_speed_lower_mps.value =
        output.frame_evidence.adversary_corner_interval.lower_mps.value;
    observation_inputs.adversary_corner_speed_upper_mps.has_value =
        output.frame_evidence.adversary_corner_interval.upper_mps.has_value;
    observation_inputs.adversary_corner_speed_upper_mps.value =
        output.frame_evidence.adversary_corner_interval.upper_mps.value;
    observation_inputs.adversary_corner_interval_admitted =
        output.frame_evidence.adversary_corner_interval.admitted();
    observation_inputs.own_corner_speed_upper_mps.has_value =
        output.frame_evidence.own_corner_interval.upper_mps.has_value;
    observation_inputs.own_corner_speed_upper_mps.value =
        output.frame_evidence.own_corner_interval.upper_mps.value;
    observation_inputs.own_corner_interval_admitted =
        output.frame_evidence.own_corner_interval.admitted();

    const Result<HabfmPreTaskObservations> observations =
        EvaluateHabfmPreTaskObservations(input.frame, observation_inputs);
    if (!observations.ok())
    {
        status = observations.status;
        return;
    }
    output.pre_task = observations.value;
    output.applicable = true;
}

void HabfmTerminalControlIntentOwner::BuildActiveInputs(
    const HabfmTerminalControlIntentInput& input,
    const HabfmTerminalControlIntentObservation& observation,
    HabfmMergeEvidenceProvider& provider,
    const HabfmActiveControlCore& active_core,
    HabfmActiveCoreInputs& output,
    Status& status) const noexcept
{
    status = Status{};
    output = HabfmActiveCoreInputs{};
    HabfmMergeEvidenceProviderInputs provider_input{};
    const control::route5::CommandEnvelope& envelope =
        input.current_envelope;
    // The accepted control pipeline already produced this finite envelope.
    // HABFM consumes the bound; it does not create a second admission gate.
    // A value at or below one g merely leaves the lead-turn event unarmed;
    // Merge Approach still publishes writer 4 on the current frame.
    provider_input.capability_n_max_g.has_value = true;
    provider_input.capability_n_max_g.value = envelope.nz_feasible_g;
    provider_input.capability_n_max_admitted = true;
    provider_input.merge_intent = observation.pre_task.merge_intent;
    provider_input.merge_speed_floor.admitted =
        observation.frame_evidence.g17_speed_floor.admitted();
    provider_input.merge_speed_floor.floor_speed_mps =
        observation.frame_evidence.g17_speed_floor.floor_mps.value;
    provider_input.three_dimensional_merge_required =
        observation.command_geometry.three_dimensional_merge_required;

    const HabfmActiveControlCoreSnapshot snapshot = active_core.Snapshot();
    if (!provider_input.three_dimensional_merge_required
        && snapshot.side_sign.has_value
        && (snapshot.side_sign.value == -1 || snapshot.side_sign.value == 1))
    {
        provider_input.frontal_pass_fallback_side_sign =
            snapshot.side_sign.value;
    }
    else if (!provider_input.three_dimensional_merge_required)
    {
        HabfmEntrySide(
            input.frame,
            provider_input.frontal_pass_fallback_side_sign,
            status);
        if (!status.ok())
        {
            return;
        }
    }

    provider_input.vertical_excess_present = true;
    provider_input.vertical_excess = observation.pre_task.vertical_excess;
    provider_input.corner_speed_mps.has_value =
        observation.frame_evidence.own_corner_interval.upper_mps.has_value;
    provider_input.corner_speed_mps.value =
        observation.frame_evidence.own_corner_interval.upper_mps.value;
    provider_input.corner_admitted =
        observation.frame_evidence.own_corner_interval.admitted();
    provider_input.turn_radius_m.has_value =
        observation.frame_evidence.own_sustained_turn_point
            .turn_radius_m.has_value;
    provider_input.turn_radius_m.value =
        observation.frame_evidence.own_sustained_turn_point
            .turn_radius_m.value;
    provider_input.turn_radius_admitted =
        observation.frame_evidence.own_sustained_turn_point.admitted();
    provider_input.hard_deck_margin_m.has_value =
        observation.frame_evidence.own_altitude_m.has_value;
    provider_input.hard_deck_margin_m.value =
        observation.frame_evidence.own_altitude_m.value
        - kOfficialCrashFloorM;
    provider_input.vertical_room_gate.enabled = true;
    provider_input.vertical_room_gate.provenance_present = true;
    provider_input.vertical_room_gate.provenance_matches = true;

    provider.Build(input.frame, provider_input, output, status);
    if (status.ok())
    {
        output.far_flee_approach_enabled = true;
    }
}

void HabfmTerminalControlIntentOwner::UpdateEngageDecision(
    const HabfmTerminalControlIntentInput& input,
    const HabfmTerminalControlIntentObservation& observation,
    HabfmEngageDecisionLatch& latch,
    const HabfmActiveCoreInputs& active_inputs,
    HabfmEngageDecisionReceipt& output,
    Status& status) const noexcept
{
    status = Status{};
    HabfmEngageDecisionInput decision_input{};
    decision_input.frame_identity = input.frame.frame_identity;
    decision_input.gate_enabled = true;
    decision_input.adversary_range_m = input.frame.own_offense.range_m;
    decision_input.reengage_range_m =
        input.frame.own_offense.phase.max_range_m;
    decision_input.closing_speed_mps = input.frame.closing_speed_mps;
    const Result<HabfmMergeProfileSelection> selection =
        SelectHabfmMergeProfile(input.frame);
    if (!selection.ok())
    {
        status = selection.status;
        return;
    }
    decision_input.merge_selection_present = true;
    decision_input.merge_selection = selection.value;
    decision_input.frontal_pass = active_inputs.frontal_pass;
    decision_input.speed_floor = observation.frame_evidence.g17_speed_floor;
    latch.Update(decision_input, output, status);
    if (status.ok()
        && (!output.valid
            || !SameFrame(
                output.frame_identity,
                input.frame.frame_identity)))
    {
        status.code = StatusCode::InvalidConfiguration;
    }
}

void HabfmTerminalControlIntentOwner::PrepareCandidate(
    const HabfmTerminalControlIntentInput& input,
    const HabfmTerminalControlIntentObservation& observation,
    HabfmTerminalPreparedControlIntent& output,
    Status& status) const noexcept
{
    output = HabfmTerminalPreparedControlIntent{};
    status = Status{};
    output.frame_identity = input.frame.frame_identity;
    output.captured_generation = generation_;
    output.evaluated = true;

    if (!observation.evaluated
        || !observation.applicable
        || !SameFrame(observation.frame_identity, input.frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.next_state = state_;
    BuildActiveInputs(
        input,
        observation,
        output.next_state.merge_evidence_provider,
        output.next_state.active_core,
        output.active_inputs,
        status);
    if (!status.ok())
    {
        return;
    }
    UpdateEngageDecision(
        input,
        observation,
        output.next_state.engage_decision_latch,
        output.active_inputs,
        output.engage_decision,
        status);
    if (!status.ok())
    {
        return;
    }

    output.next_state.active_core.StepControlIntent(
        input.frame,
        output.active_inputs,
        output.active_output,
        status,
        output.next_state.neutral_cue_streak);
    if (status.ok()
        && output.active_output.leg_status
            == HabfmActiveCoreLegStatus::MergePass
        && output.active_output.merge_pass
        && output.active_output.mode_recheck
        && !output.active_output.intent_present)
    {
        output.next_state.neutral_cue_streak =
            output.active_output.neutral_cue_streak;
        output.same_frame_reentry_count = 1U;
        output.next_state.merge_evidence_provider.Reset();
        BuildActiveInputs(
            input,
            observation,
            output.next_state.merge_evidence_provider,
            output.next_state.active_core,
            output.active_inputs,
            status);
        if (status.ok())
        {
            UpdateEngageDecision(
                input,
                observation,
                output.next_state.engage_decision_latch,
                output.active_inputs,
                output.engage_decision,
                status);
        }
        if (status.ok())
        {
            output.next_state.active_core.StepControlIntent(
                input.frame,
                output.active_inputs,
                output.active_output,
                status,
                output.next_state.neutral_cue_streak);
        }
    }

    if (!status.ok()
        || output.active_output.leg_status
            != HabfmActiveCoreLegStatus::Running
        || !output.active_output.intent_present
        || output.active_output.branch == HabfmActiveBranch::None)
    {
        status = Status{};
        output.active_output = HabfmActiveControlOutput{};
        if (observation.command_geometry.three_dimensional_merge_required)
        {
            BuildHabfmThreeDimensionalMergeIntent(
                input.frame,
                output.active_inputs.merge_speed_floor,
                output.active_inputs.merge_vertical_room,
                HabfmOptionalScalar{},
                output.active_output.intent,
                status);
        }
        else
        {
            BuildHabfmMergeApproachIntent(
                input.frame,
                output.active_inputs.merge_speed_floor,
                output.active_inputs.merge_separation_policy.admitted
                    && output.active_inputs.merge_separation_policy.compress,
                output.active_inputs.frontal_pass,
                output.active_inputs.merge_vertical_room,
                output.active_output.intent,
                status);
        }
        if (status.ok())
        {
            output.active_output.leg_status =
                HabfmActiveCoreLegStatus::Running;
            output.active_output.branch = HabfmActiveBranch::MergeApproach;
            output.active_output.intent_present = true;
            output.active_output.neutral_cue_streak =
                output.next_state.neutral_cue_streak;
            output.merge_approach_fallback_used = true;
        }
    }
    if (!status.ok()
        || output.active_output.leg_status
            != HabfmActiveCoreLegStatus::Running
        || !output.active_output.intent_present
        || output.active_output.branch == HabfmActiveBranch::None)
    {
        if (status.ok())
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }

    output.next_state.neutral_cue_streak =
        output.active_output.neutral_cue_streak;
    output.intent = output.active_output.intent;
    Status intent_status{};
    output.intent.Validate(intent_status);
    if (!intent_status.ok()
        || output.intent.writer_id != ControlIntentWriterHabfm
        || !SameFrame(
            output.intent.frame_identity,
            input.frame.frame_identity))
    {
        output.intent.Clear();
        status.code = intent_status.ok()
            ? StatusCode::InvalidConfiguration
            : intent_status.code;
        return;
    }

    output.selected = true;
    output.next_state_ready = true;
    if (output.merge_approach_fallback_used)
    {
        output.reason = HabfmTerminalControlIntentReason::
            MergeApproachFallbackSelected;
    }
    else if (output.same_frame_reentry_count == 1U)
    {
        output.reason = HabfmTerminalControlIntentReason::
            SameFrameReentrySelected;
    }
    else
    {
        output.reason =
            HabfmTerminalControlIntentReason::ActiveCoreSelected;
    }
}

void HabfmTerminalControlIntentOwner::ValidatePublished(
    const HabfmTerminalPreparedControlIntent& prepared,
    Status& status) const noexcept
{
    status = Status{};
    if (!prepared.evaluated
        || !prepared.selected
        || !prepared.next_state_ready
        || prepared.committed
        || prepared.captured_generation != generation_
        || prepared.intent.writer_id != ControlIntentWriterHabfm
        || !SameFrame(
            prepared.intent.frame_identity,
            prepared.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
}

void HabfmTerminalControlIntentOwner::CommitPublished(
    HabfmTerminalPreparedControlIntent& prepared,
    Status& status) noexcept
{
    ValidatePublished(prepared, status);
    if (!status.ok())
    {
        return;
    }
    state_ = prepared.next_state;
    prepared.committed = true;
    ++generation_;
}

} // namespace LadyLuck
