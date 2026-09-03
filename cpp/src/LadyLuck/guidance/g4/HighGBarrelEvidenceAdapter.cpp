#include "LadyLuck/guidance/g4/HighGBarrelEvidenceAdapter.hpp"

#include <cmath>

namespace
{

bool FiniteTotalLoad(
    const LadyLuck::runtime::TacticalCompletedTotalLoadReceipt& value) noexcept
{
    return std::isfinite(value.raw_total_load_g)
        && std::isfinite(value.governed_total_load_g)
        && std::isfinite(value.physical_limit_g);
}

bool ValidPublishedTotalLoad(
    const LadyLuck::runtime::TacticalCompletedTotalLoadReceipt& value) noexcept
{
    return value.source
            != LadyLuck::runtime::TacticalCompletedTotalLoadSource::Unavailable
        && value.raw_total_load_g > 0.0
        && value.governed_total_load_g > 0.0
        && value.physical_limit_g > 0.0;
}

bool RootGunDefenseBehavior(
    const LadyLuck::DoctrineBehaviorId value) noexcept
{
    return value == LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak
        || value == LadyLuck::DoctrineBehaviorId::PrefireSnapshotPlaneChange
        || value
            == LadyLuck::DoctrineBehaviorId::OfficialGunSnapshotPlaneChange;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

void HighGBarrelEvidenceAdapter::BuildSafety(
    const runtime::TacticalCommandBuildInput& input,
    const bool hard_deck_source_valid,
    const double hard_deck_margin_m,
    HighGBarrelSafetyEvidence& output,
    Status& status) const noexcept
{
    output = HighGBarrelSafetyEvidence{};
    status = Status{};
    const auto& envelope = input.current_envelope;
    const auto& longitudinal = input.current_longitudinal_evidence;
    const double gamma_limit_rad = longitudinal.flight_path_gamma_limit_rad;

    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    output.state_sample_t_sec = input.frame.t_sec;
    output.envelope_valid = runtime::CurrentCommandEnvelopeAvailable(input);
    output.hard_deck_source_valid = hard_deck_source_valid
        && std::isfinite(hard_deck_margin_m);
    output.hard_deck_margin_m = std::isfinite(hard_deck_margin_m)
        ? hard_deck_margin_m
        : 0.0;
    output.stall_source_valid = envelope.stall_speed_valid
        && envelope.stall_speed_source
            != control::route5::StallSpeedBoundarySource::Unavailable
        && std::isfinite(envelope.stall_speed_mps)
        && envelope.stall_speed_mps > 0.0;
    output.stall_speed_1g_mps = std::isfinite(envelope.stall_speed_mps)
        ? envelope.stall_speed_mps
        : 0.0;
    output.flight_path_gamma_limit_source_valid =
        longitudinal.flight_path_gamma_limit_valid
        && std::isfinite(gamma_limit_rad)
        && gamma_limit_rad > 0.0;
    output.flight_path_gamma_limit_rad = std::isfinite(gamma_limit_rad)
        ? gamma_limit_rad
        : 0.0;
}

void HighGBarrelEvidenceAdapter::BuildLoadedResponse(
    const runtime::TacticalCommandBuildInput& input,
    const runtime::TacticalCompletedTotalLoadReceipt& total_load,
    HighGBarrelLoadedResponseEvidence& output,
    Status& status) const noexcept
{
    output = HighGBarrelLoadedResponseEvidence{};
    status = Status{};
    const auto& feedback = input.previous_control_feedback;

    output.valid = feedback.valid
        && feedback.source_kind
            == runtime::TacticalFeedbackSourceKind::AcceptedEstimatorFrame
        && feedback.command_backend_id
            == runtime::TacticalControlBackendId::TecsCisV4
        && feedback.cis_integrity_valid;
    output.source_frame_identity = feedback.source_frame_identity;
    output.source_t_sec = feedback.source_frame_identity.source_time_s;
    output.feedback_fresh = input.feedback_freshness
        == runtime::TacticalFeedbackFreshness::Fresh;
    output.backend_is_cis_v4 = feedback.command_backend_id
        == runtime::TacticalControlBackendId::TecsCisV4;
    output.previous_behavior_id = feedback.behavior_id;
    output.previous_command_is_root_gun_defense =
        RootGunDefenseBehavior(feedback.behavior_id);
    // cis_integrity_valid is the C++ completed-backend receipt that no NaN
    // guard or fallback substitution occurred.  Invalid integrity keeps this
    // whole evidence sample unavailable instead of inventing a false guard.
    output.cis_nan_guard = false;
    output.cis_fallback = feedback.cis_fallback;
    output.energy_rate_measurement_valid =
        feedback.energy_rate_measurement_valid;
    output.auto_gcas_active = feedback.auto_gcas_active;
    output.auto_gcas_state_valid = feedback.auto_gcas_state_valid;
    output.nz_cmd_governed_valid = feedback.nz_cmd_governed_valid;
    output.nz_cmd_governed_g = feedback.nz_cmd_governed_g;
    output.nz_measured_valid = feedback.nz_measured_valid;
    output.nz_measured_g = feedback.nz_measured_g;
    if (total_load.valid
        && FiniteTotalLoad(total_load)
        && ValidPublishedTotalLoad(total_load)
        && feedback.valid
        && SameControlFrameIdentity(
            total_load.source_frame_identity,
            feedback.source_frame_identity))
    {
        output.total_load_cmd_raw_g = total_load.raw_total_load_g;
        output.total_load_cmd_governed_g = total_load.governed_total_load_g;
        output.total_load_limit_g = total_load.physical_limit_g;
        output.total_load_source =
            HighGBarrelTotalLoadSource::CompletedCisV4LoadVector;
    }
}

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
