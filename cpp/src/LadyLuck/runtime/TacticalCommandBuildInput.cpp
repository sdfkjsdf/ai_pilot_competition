#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cmath>

namespace
{
void ObserveLongitudinalAuthority(
    const LadyLuck::ControlFrameIdentity& current_frame_identity,
    const LadyLuck::runtime::
        TacticalCurrentLongitudinalAuthorityEvidence& current,
    const LadyLuck::runtime::TacticalAge1ControlFeedback& previous,
    LadyLuck::ObfmLongitudinalAuthorityReceipt& output,
    LadyLuck::Status& status) noexcept
{
    const auto& completed = previous.completed_energy_authority;
    LadyLuck::ObfmLongitudinalAuthorityInput input{};
    input.current_frame_identity = current_frame_identity;
    input.previous_energy.source_frame_identity =
        completed.source_frame_identity;
    input.previous_energy.cis_v4_backend =
        previous.command_backend_id
            == LadyLuck::runtime::TacticalControlBackendId::TecsCisV4;
    input.previous_energy.energy_receipt_available = completed.valid;
    input.previous_energy.continuous_total_energy_controller =
        completed.continuous_total_energy_controller;
    input.previous_energy.rate_measurement_valid =
        completed.rate_measurement_valid;
    input.previous_energy.controller_configuration_available =
        completed.controller_configuration_available;
    input.previous_energy.energy_error_gain_per_s =
        completed.energy_error_gain_per_s;
    input.previous_energy.energy_integral_gain_per_s2 =
        completed.energy_integral_gain_per_s2;
    input.previous_energy.energy_rate_feedback_gain =
        completed.energy_rate_feedback_gain;
    input.previous_energy.total_energy_error_m2ps2 =
        completed.total_energy_error_m2ps2;
    input.previous_energy.energy_integral_error_m2ps =
        completed.energy_integral_error_m2ps;
    input.previous_energy.specific_energy_rate_measured_m2ps3 =
        completed.specific_energy_rate_measured_m2ps3;
    input.previous_energy.speed_mps = completed.speed_mps;
    input.previous_energy.minimum_speed_mps = completed.minimum_speed_mps;
    input.previous_energy.mass_kg = completed.mass_kg;
    input.previous_energy.drag_estimate_n = completed.drag_estimate_n;
    input.previous_energy.thrust_velocity_projection =
        completed.thrust_velocity_projection;
    input.previous_energy.thrust_min_n = completed.thrust_min_n;
    input.previous_energy.thrust_max_n = completed.thrust_max_n;
    input.speed_rate_bounds_available = current.tecs_configuration.valid;
    input.speed_rate_bounds_source_valid =
        current.valid
        && current.tecs_configuration.continuous_total_energy_controller;
    input.speed_rate_lower_mps2 =
        current.tecs_configuration.speed_command_rate_min_mps2;
    input.speed_rate_upper_mps2 =
        current.tecs_configuration.speed_command_rate_max_mps2;
    input.flight_path_gamma_limit_rad =
        current.flight_path_gamma_limit_rad;

    LadyLuck::ObfmLongitudinalAuthority authority{};
    authority.Observe(input, output, status);
}

void SetFeedbackMissing(
    LadyLuck::runtime::TacticalCommandBuildInput& output) noexcept
{
    output.previous_control_feedback =
        LadyLuck::runtime::TacticalAge1ControlFeedback{};
    output.feedback_freshness =
        LadyLuck::runtime::TacticalFeedbackFreshness::Missing;
    output.valid = true;
}

} // namespace

namespace LadyLuck
{
namespace runtime
{

void TacticalCommandBuildInputBuilder::Build(
    const DogfightGeometryFrame& frame,
    const EstimatorOutputV6& accepted_estimator,
    const control::route5::CommandEnvelope& current_envelope,
    const safety::AutoGcasEntryReceipt& current_safety,
    const TacticalCurrentLongitudinalAuthorityEvidence&
        current_longitudinal_evidence,
    const TacticalAge1ControlFeedback& previous_control_feedback,
    TacticalCommandBuildInput& output,
    Status& status) const noexcept
{
    output = TacticalCommandBuildInput{};
    status = Status{};
    // The ControlCore constructs this immutable current-frame bundle from
    // successful producers.  Do not unpack and re-prove those receipts here.
    output.frame = frame;
    output.accepted_estimator = accepted_estimator;
    output.current_envelope = current_envelope;
    output.current_command_envelope_available =
        current_envelope.valid
        && current_envelope.enabled
        && current_envelope.command_containment_authority
        && control::route5::CommandEnvelopeSourceProvidesBounds(
            current_envelope.source)
        && std::isfinite(current_envelope.nz_feasible_g)
        && std::isfinite(current_envelope.nz_min_g)
        && std::isfinite(current_envelope.p_max_radps)
        && current_envelope.nz_feasible_g > 0.0
        && current_envelope.nz_min_g <= current_envelope.nz_feasible_g
        && current_envelope.p_max_radps >= 0.0;
    output.current_physical_envelope_available =
        control::route5::IsPhysicalNzCommandEnvelopeSource(
            current_envelope.source);
    output.current_safety = current_safety;
    output.current_longitudinal_evidence =
        current_longitudinal_evidence;

    if (!previous_control_feedback.valid)
    {
        SetFeedbackMissing(output);
        return;
    }

    // Previous feedback is internal immutable state, not a new external
    // payload.  Only chronology determines whether it can be used as age-1
    // evidence; malformed or unrelated history degrades to Missing.
    if (previous_control_feedback.command_backend_id
            != TacticalControlBackendId::TecsCisV4
        || previous_control_feedback.source_kind
            != TacticalFeedbackSourceKind::AcceptedEstimatorFrame
        || !previous_control_feedback.source_frame_index_valid
        || !IsValidControlFrameIdentity(
            previous_control_feedback.source_frame_identity))
    {
        SetFeedbackMissing(output);
        return;
    }
    if (previous_control_feedback.source_frame_identity.episode_epoch
        < frame.frame_identity.episode_epoch)
    {
        // A gap-resync starts a new estimator episode.  The completed sample
        // remains historical telemetry but cannot be age-1 control evidence
        // in the new episode.
        SetFeedbackMissing(output);
        return;
    }
    if (previous_control_feedback.source_frame_identity.episode_epoch
            != frame.frame_identity.episode_epoch
        || previous_control_feedback.source_frame_identity.frame_index
            >= frame.frame_identity.frame_index)
    {
        SetFeedbackMissing(output);
        return;
    }

    output.previous_control_feedback = previous_control_feedback;
    output.feedback_frame_age = frame.frame_identity.frame_index
        - previous_control_feedback.source_frame_index;
    output.feedback_freshness = output.feedback_frame_age == 1U
        ? TacticalFeedbackFreshness::Fresh
        : TacticalFeedbackFreshness::Stale;
    if (output.feedback_freshness == TacticalFeedbackFreshness::Fresh)
    {
        ObserveLongitudinalAuthority(
            frame.frame_identity,
            current_longitudinal_evidence,
            previous_control_feedback,
            output.obfm_longitudinal_authority,
            status);
        if (!status.ok())
        {
            output.obfm_longitudinal_authority =
                ObfmLongitudinalAuthorityReceipt{};
            status = Status{};
        }
    }
    output.valid = true;
}

void TacticalCommandBuildInputBuilder::PrepareFeedback(
    const ControlFrameIdentity& source_frame_identity,
    const ControlIntent& tactical,
    const EstimatorOutputV6& estimate,
    const control::route5::CommandEnvelope& envelope,
    const control::tecs_cis::BodyRateLoadEnergyCommand& reference,
    const control::tecs_cis::TecsCisOutput& control,
    const TacticalCompletedTotalLoadReceipt& completed_total_load,
    TacticalAge1ControlFeedback& output,
    Status& status) const noexcept
{
    output = TacticalAge1ControlFeedback{};
    status = Status{};
    // All inputs are successful outputs of the current ControlCore pipeline.
    // Assemble age-1 telemetry without an additional rejection seam.
    const auto& completed = control.completed_energy_authority;

    output.source_kind = TacticalFeedbackSourceKind::AcceptedEstimatorFrame;
    output.source_frame_index_valid = true;
    output.source_frame_index = source_frame_identity.frame_index;
    output.source_frame_identity = source_frame_identity;
    output.command_backend_id = TacticalControlBackendId::TecsCisV4;
    output.writer_id = tactical.writer_id;
    output.behavior_id = tactical.behavior_id;
    output.mode_id = tactical.mode_id;
    output.cis_integrity_valid = true;
    output.cis_clipped = control.diagnostics.command_clipped;
    output.cis_fallback = false;
    output.energy_lower_saturated =
        control.diagnostics.lower_thrust_saturated;
    output.energy_upper_saturated =
        control.diagnostics.upper_thrust_saturated;
    output.energy_rate_measurement_valid =
        control.diagnostics.rate_measurement_valid;
    output.specific_energy_rate_measured_m2ps3 =
        control.diagnostics.specific_energy_rate_measured_m2ps3;
    output.completed_energy_authority = completed;
    output.completed_total_load = completed_total_load;
    output.energy_rate_authority_valid =
        completed.valid && completed.rate_measurement_valid;
    output.energy_authority_mass_kg = completed.mass_kg;
    output.nz_cmd_governed_valid = true;
    output.nz_cmd_governed_g = reference.nz_cmd_g;
    output.nz_measured_valid = estimate.nz_valid
        && estimate.nz_source != EstimatorSource::Uninitialized;
    output.nz_measured_g = estimate.nz;
    output.nz_feasible_valid =
        control::route5::IsPhysicalNzCommandEnvelopeSource(envelope.source);
    output.nz_feasible_g = envelope.nz_feasible_g;
}

void TacticalCommandBuildInputBuilder::CompleteFeedback(
    const safety::AutoGcasReceipt& auto_gcas,
    const double source_decision_time_s,
    const control::tecs_cis::NormalizedControlCommand&
        transmitted_wire_command,
    TacticalAge1ControlFeedback& output,
    Status& status) const noexcept
{
    // The selected control backend has already admitted the transmitted
    // command and any Auto-GCAS lifecycle update before this telemetry step.
    status = Status{};
    output.source_decision_time_s = source_decision_time_s;
    output.transmitted_command = transmitted_wire_command;
    output.auto_gcas_active = auto_gcas.override_active;
    output.auto_gcas_phase = auto_gcas.phase;
    output.auto_gcas_fault = false;
    output.auto_gcas_state_valid = auto_gcas.entry_available
        && auto_gcas.entry_recoverable;
    output.valid = true;
}

} // namespace runtime
} // namespace LadyLuck
