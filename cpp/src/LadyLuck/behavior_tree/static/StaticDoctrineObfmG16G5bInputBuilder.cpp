#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bInputBuilder.hpp"

#include "LadyLuck/guidance/doctrine/TacticalSpeedFloorObserver.hpp"

#include <cmath>

namespace
{

constexpr double OfficialCrashFloorM = 1000.0 * 0.3048;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteRootObservationFrame(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    return FiniteVector(frame.own.position_ned_m)
        && FiniteVector(frame.own.velocity_ned_mps)
        && FiniteVector(frame.opponent.position_ned_m)
        && FiniteVector(frame.opponent.velocity_ned_mps)
        && std::isfinite(frame.own_offense.ata_rad)
        && std::isfinite(frame.enemy_offense.ata_rad);
}

bool TacticalModeValue(
    const LadyLuck::guidance::doctrine::TacticalMode mode) noexcept
{
    return mode == LadyLuck::guidance::doctrine::TacticalMode::Obfm
        || mode == LadyLuck::guidance::doctrine::TacticalMode::Habfm
        || mode == LadyLuck::guidance::doctrine::TacticalMode::Dbfm;
}

LadyLuck::guidance::obfm::G5bFeedbackFreshness FeedbackFreshness(
    const LadyLuck::runtime::TacticalFeedbackFreshness freshness) noexcept
{
    switch (freshness)
    {
    case LadyLuck::runtime::TacticalFeedbackFreshness::Fresh:
        return LadyLuck::guidance::obfm::G5bFeedbackFreshness::Fresh;
    case LadyLuck::runtime::TacticalFeedbackFreshness::Stale:
        return LadyLuck::guidance::obfm::G5bFeedbackFreshness::Stale;
    case LadyLuck::runtime::TacticalFeedbackFreshness::Missing:
    default:
        return LadyLuck::guidance::obfm::G5bFeedbackFreshness::Missing;
    }
}

LadyLuck::guidance::obfm::G5bControlBackend ControlBackend(
    const LadyLuck::runtime::TacticalControlBackendId backend) noexcept
{
    return backend == LadyLuck::runtime::TacticalControlBackendId::TecsCisV4
        ? LadyLuck::guidance::obfm::G5bControlBackend::TecsCisV4
        : LadyLuck::guidance::obfm::G5bControlBackend::Unavailable;
}

void BuildModeDecision(
    const LadyLuck::guidance::doctrine::
        BilateralDoctrineTurnCircleReceipt& root,
    const bool safety_receipt_current,
    const LadyLuck::behavior_tree::static_bt::
        StaticSafetyGunPreparedReceipt& safety_gun,
    LadyLuck::guidance::doctrine::ModeDecision& output) noexcept
{
    output = LadyLuck::guidance::doctrine::ModeDecision{};
    const LadyLuck::guidance::doctrine::TacticalMode raw_mode =
        TacticalModeValue(root.candidate_mode)
        ? root.candidate_mode
        : LadyLuck::guidance::doctrine::TacticalMode::Habfm;
    output.valid = true;
    output.raw_mode = raw_mode;
    output.mode.has_value = true;
    output.mode.value = raw_mode;
    output.admitted = true;
    output.bypass_reason =
        LadyLuck::guidance::doctrine::ModeDecisionBypassReason::None;

    if (safety_receipt_current && safety_gun.safety_required)
    {
        output.bypass_reason = LadyLuck::guidance::doctrine::
            ModeDecisionBypassReason::SafetyRequired;
        return;
    }
    const bool gun_receipt_current = safety_receipt_current
        && safety_gun.gun_admission.valid
        && LadyLuck::SameControlFrameIdentity(
            safety_gun.gun_admission.frame_identity,
            safety_gun.frame_identity);
    if (gun_receipt_current
        && safety_gun.gun_admission.immediate_defense_required)
    {
        output.bypass_reason = LadyLuck::guidance::doctrine::
            ModeDecisionBypassReason::ActualGunThreat;
    }
}

void BuildG5bSafety(
    const LadyLuck::runtime::TacticalCommandBuildInput& input,
    const bool safety_receipt_current,
    const LadyLuck::behavior_tree::static_bt::
        StaticSafetyGunPreparedReceipt& safety_gun,
    LadyLuck::guidance::obfm::G5bSafetyEvidence& output) noexcept
{
    output = LadyLuck::guidance::obfm::G5bSafetyEvidence{};
    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    output.state_sample_t_sec = input.frame.t_sec;

    const LadyLuck::control::route5::CommandEnvelope& envelope =
        input.current_envelope;
    const LadyLuck::runtime::TacticalCurrentLongitudinalAuthorityEvidence&
        longitudinal = input.current_longitudinal_evidence;
    const double altitude_m = -input.frame.own.position_ned_m[2];
    output.envelope_valid =
        LadyLuck::runtime::CurrentCommandEnvelopeAvailable(input)
        && std::isfinite(altitude_m)
        && std::isfinite(envelope.nz_feasible_g)
        && envelope.nz_feasible_g > 0.0;
    output.hard_deck_source_present = true;
    output.hard_deck_margin_m = altitude_m - OfficialCrashFloorM;
    output.stall_source_present = envelope.stall_speed_valid
        && envelope.stall_speed_source
            != LadyLuck::control::route5::StallSpeedBoundarySource::
                Unavailable;
    output.stall_speed_1g_mps = envelope.stall_speed_mps;
    output.flight_path_gamma_limit_source_present =
        longitudinal.flight_path_gamma_limit_valid;
    output.flight_path_gamma_limit_rad =
        longitudinal.flight_path_gamma_limit_rad;

    const LadyLuck::runtime::TacticalAge1ControlFeedback& feedback =
        input.previous_control_feedback;
    output.previous_control_feedback_present = feedback.valid;
    output.feedback_freshness = FeedbackFreshness(input.feedback_freshness);
    output.command_backend = ControlBackend(feedback.command_backend_id);
    const bool tecs_feedback = feedback.valid
        && feedback.command_backend_id
            == LadyLuck::runtime::TacticalControlBackendId::TecsCisV4;
    output.cis_nan_guard_present = tecs_feedback;
    output.cis_nan_guard = tecs_feedback && !feedback.cis_integrity_valid;
    output.cis_clipped_present = tecs_feedback;
    output.cis_clipped = feedback.cis_clipped;
    output.cis_fallback_present = tecs_feedback;
    output.cis_fallback = feedback.cis_fallback;
    output.energy_lower_saturated_present = tecs_feedback;
    output.energy_lower_saturated = feedback.energy_lower_saturated;
    output.energy_upper_saturated_present = tecs_feedback;
    output.energy_upper_saturated = feedback.energy_upper_saturated;
    output.energy_rate_measurement_valid_present = tecs_feedback;
    output.energy_rate_measurement_valid =
        feedback.energy_rate_measurement_valid;
    output.auto_gcas_active = feedback.auto_gcas_active;
    output.auto_gcas_state_valid = feedback.auto_gcas_state_valid;

    output.boom_zoom_evidence_valid = true;
    output.boom_zoom_frame_identity = input.frame.frame_identity;
    output.energy_comparison_authority = LadyLuck::guidance::obfm::
        G5bEnergyComparisonAuthority::CompetitionSameF16TypeAndMass;
    output.same_f16_type_and_mass_assumed = true;
    output.official_enemy_gun_receipt_current = safety_receipt_current
        && safety_gun.root_gun_evidence.valid
        && LadyLuck::SameControlFrameIdentity(
            safety_gun.root_gun_evidence.frame_identity,
            input.frame.frame_identity);
    output.predictive_prefire_receipt_current = safety_receipt_current
        && safety_gun.prefire_observation_attempted
        && safety_gun.prefire_observation_ready
        && safety_gun.prefire_threat_shadow.evaluated
        && safety_gun.prefire_consumer_attempted
        && safety_gun.prefire_consumer_ready;
    output.enemy_fire_opportunity_active =
        (output.official_enemy_gun_receipt_current
            && safety_gun.root_gun_evidence.official_gun_threat)
        || (safety_receipt_current
            && safety_gun.prefire_consumer_ready
            && safety_gun.prefire_consumer.receipt.active);
    output.predictive_prefire_absence_resolved =
        output.predictive_prefire_receipt_current
        && !output.enemy_fire_opportunity_active
        && (safety_gun.prefire_threat_shadow.admitted
            || safety_gun.prefire_threat_shadow.reason
                == LadyLuck::guidance::prefire::
                    RootPrefireThreatShadowReason::
                        NoElapsedNonScratchPhase);
    output.enemy_fire_opportunity_evaluated =
        output.official_enemy_gun_receipt_current;
}

void BuildSpeedFloor(
    const LadyLuck::runtime::TacticalCommandBuildInput& input,
    LadyLuck::guidance::obfm::G5bSpeedFloorEvidence& output) noexcept
{
    output = LadyLuck::guidance::obfm::G5bSpeedFloorEvidence{};
    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    LadyLuck::guidance::doctrine::SampleTacticalSpeedFloor(
        -input.frame.own.position_ned_m[2],
        output.sample);
    output.opponent_valid = true;
    LadyLuck::guidance::doctrine::SampleTacticalSpeedFloor(
        -input.frame.opponent.position_ned_m[2],
        output.opponent_sample);
}

void BindPreviousPursuit(
    const LadyLuck::behavior_tree::static_bt::
        StaticDoctrineObfmG16G5bInputSources& sources,
    LadyLuck::guidance::obfm::ObfmLeadDisciplineInput& output,
    bool& consumed) noexcept
{
    output = LadyLuck::guidance::obfm::ObfmLeadDisciplineInput{};
    consumed = sources.previous_pursuit_receipt_available
        && sources.previous_pursuit.completed_observation_present
        && !sources.previous_pursuit.producer_contract_contradiction
        && sources.previous_pursuit.lead_discipline_input_for_next_frame
            .previous_observation_present;
    if (consumed)
    {
        output = sources.previous_pursuit
            .lead_discipline_input_for_next_frame;
    }
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

void StaticDoctrineObfmG16G5bInputBuilder::Build(
    const StaticDoctrineObfmG16G5bInputSources& sources,
    StaticDoctrineObfmG16G5bInput& output,
    StaticDoctrineObfmG16G5bInputBuilderReceipt& receipt,
    Status& status) const noexcept
{
    output = StaticDoctrineObfmG16G5bInput{};
    receipt = StaticDoctrineObfmG16G5bInputBuilderReceipt{};
    status = Status{};
    receipt.build_attempted = true;
    receipt.frame_identity = sources.tactical_input.frame.frame_identity;

    if (!sources.tactical_input.valid
        || !IsValidControlFrameIdentity(
            sources.tactical_input.frame.frame_identity))
    {
        receipt.status_code = StatusCode::InvalidArgument;
        status.code = receipt.status_code;
        return;
    }

    receipt.root_observation_attempted = true;
    Status root_status{};
    root_authority_.Observe(
        sources.tactical_input.frame,
        receipt.root_observation,
        root_status);
    receipt.root_status_code = root_status.code;
    if (root_status.code != StatusCode::Ok)
    {
        if (!FiniteRootObservationFrame(sources.tactical_input.frame))
        {
            receipt.status_code = root_status.code;
            status.code = receipt.status_code;
            return;
        }
        // Finite but unresolved positional capability is the established
        // total-classifier HABFM result. It is not a control-path fault.
        receipt.root_observation = guidance::doctrine::
            BilateralDoctrineTurnCircleReceipt{};
        receipt.root_observation.valid = true;
        receipt.root_observation.evaluated = true;
        receipt.root_observation.candidate_mode =
            guidance::doctrine::TacticalMode::Habfm;
        receipt.root_observation.dominance_status = guidance::doctrine::
            BilateralTurnCircleDominanceStatus::HabfmFallback;
        receipt.root_observation.tactical_mode_authority = true;
        receipt.root_observation.production_authority = true;
        receipt.root_finite_habfm_fallback = true;
    }
    if (!receipt.root_observation.valid
        || !receipt.root_observation.evaluated
        || !receipt.root_observation.tactical_mode_authority
        || !receipt.root_observation.production_authority)
    {
        receipt.status_code = StatusCode::InvalidConfiguration;
        status.code = receipt.status_code;
        return;
    }
    receipt.root_observation_ready = true;

    receipt.safety_gun_receipt_current =
        sources.safety_gun_receipt_available
        && sources.safety_gun.prepare_attempted
        && SameControlFrameIdentity(
            sources.safety_gun.frame_identity,
            sources.tactical_input.frame.frame_identity);
    BuildModeDecision(
        receipt.root_observation,
        receipt.safety_gun_receipt_current,
        sources.safety_gun,
        output.mode_decision);
    receipt.mode_decision_ready = true;

    BuildG5bSafety(
        sources.tactical_input,
        receipt.safety_gun_receipt_current,
        sources.safety_gun,
        output.g5b_safety);
    receipt.official_gun_receipt_current =
        output.g5b_safety.official_enemy_gun_receipt_current;
    receipt.predictive_prefire_receipt_current =
        output.g5b_safety.predictive_prefire_receipt_current;
    receipt.g5b_safety_ready = true;

    BuildSpeedFloor(sources.tactical_input, output.g5b_speed_floor);
    receipt.own_speed_floor_sampled = true;
    receipt.opponent_speed_floor_sampled = true;

    BindPreviousPursuit(
        sources,
        output.previous_pursuit,
        receipt.previous_pursuit_consumed);
    if (sources.previous_pursuit_receipt_available)
    {
        receipt.previous_pursuit_frame_identity =
            sources.previous_pursuit.frame_identity;
    }

    output.valid = true;
    output.tactical_input = sources.tactical_input;
    receipt.output_ready = true;
    receipt.status_code = StatusCode::Ok;
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
