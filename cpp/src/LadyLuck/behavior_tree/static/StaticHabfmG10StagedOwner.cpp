#include "LadyLuck/behavior_tree/static/StaticHabfmG10StagedOwner.hpp"

#include "LadyLuck/control/route5/Route5Guidance.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::ControlIntent;
using LadyLuck::ControlIntentWriterG10SecondUse;
using LadyLuck::DoctrineBehaviorId;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::SameControlFrameIdentity;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::behavior_tree::static_bt::StaticHabfmG10State;
using LadyLuck::guidance::g10::G10FlightPathGammaLimitReceipt;
using LadyLuck::guidance::g10::G10PursuitOvershootForecastReason;
using LadyLuck::guidance::g10::G10PursuitOvershootForecastReceipt;
using LadyLuck::guidance::g10::G10PursuitOvershootForecastStatus;
using LadyLuck::guidance::g10::G10SecondUseCommandLabel;
using LadyLuck::guidance::g10::G10SecondUseOwnerInput;
using LadyLuck::guidance::g10::G10SecondUseOwnerReason;
using LadyLuck::guidance::g10::G10SecondUseCausalSupplyInput;
using LadyLuck::guidance::obfm::AdversaryReversalObservation;
using LadyLuck::guidance::obfm::PursuitOvershootForecast;
using LadyLuck::guidance::obfm::PursuitOvershootForecastReason;
using LadyLuck::guidance::obfm::PursuitOvershootForecastStatus;
using LadyLuck::runtime::TacticalCommandBuildInput;

void ResetG10ManeuverState(StaticHabfmG10State& state) noexcept
{
    state.owner.Reset();
    state.last_descending_publication_available = false;
    state.last_descending_publication_identity = ControlFrameIdentity{};
}

bool SafeSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right)) return false;
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right < 0.0 && left >= maximum + right)
        || (right > 0.0 && left <= -maximum + right))
    {
        return false;
    }
    output = left - right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right)) return false;
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((absolute_left > 1.0
            && absolute_right >= maximum / absolute_left)
        || (absolute_right > 1.0
            && absolute_left >= maximum / absolute_right))
    {
        return false;
    }
    output = left * right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

void MapForecast(
    const PursuitOvershootForecast& source,
    G10PursuitOvershootForecastReceipt& target) noexcept
{
    target = G10PursuitOvershootForecastReceipt{};
    switch (source.status)
    {
    case PursuitOvershootForecastStatus::Forced:
        target.status = G10PursuitOvershootForecastStatus::Forced;
        break;
    case PursuitOvershootForecastStatus::NotForced:
        target.status = G10PursuitOvershootForecastStatus::NotForced;
        break;
    case PursuitOvershootForecastStatus::Unresolved:
    default:
        target.status = G10PursuitOvershootForecastStatus::Unresolved;
        break;
    }
    target.reason = source.reason == PursuitOvershootForecastReason::OwnAlreadyAhead
        ? G10PursuitOvershootForecastReason::OwnAlreadyAhead
        : G10PursuitOvershootForecastReason::Other;
}

bool BuildG10Candidate(
    const TacticalCommandBuildInput& tactical_input,
    const ControlIntent& habfm_base,
    StaticHabfmG10State& state,
    LadyLuck::behavior_tree::static_bt::StaticHabfmG10Prepared& prepared,
    Status& status) noexcept
{
    status = Status{};
    const DogfightGeometryFrame& frame = tactical_input.frame;

    G10PursuitOvershootForecastReceipt forecast{};
    const G10PursuitOvershootForecastReceipt* forecast_pointer = nullptr;
    if (state.completed_forecast_available)
    {
        MapForecast(state.completed_forecast, forecast);
        forecast_pointer = &forecast;
    }

    G10FlightPathGammaLimitReceipt gamma{};
    const double observed_gamma = tactical_input.current_longitudinal_evidence
        .flight_path_gamma_limit_rad;
    const bool observed_gamma_available = tactical_input
            .current_longitudinal_evidence.flight_path_gamma_limit_valid
        && std::isfinite(observed_gamma)
        && observed_gamma > 0.0;
    const double gamma_limit = observed_gamma_available
        ? observed_gamma
        : LadyLuck::control::route5::Route5GuidanceConfig{}
            .gamma_cmd_limit_rad;
    const G10FlightPathGammaLimitReceipt* gamma_pointer = nullptr;
    if (std::isfinite(gamma_limit) && gamma_limit > 0.0)
    {
        gamma.available = true;
        gamma.source_nonempty = true;
        gamma.value_rad = gamma_limit;
        gamma_pointer = &gamma;
    }

    const bool completed_k_minus_1_descending =
        state.last_descending_publication_available
        && state.last_descending_publication_identity.episode_epoch
            == frame.frame_identity.episode_epoch
        && state.last_descending_publication_identity.frame_index
            < (std::numeric_limits<std::uint64_t>::max)()
        && state.last_descending_publication_identity.frame_index + 1U
            == frame.frame_identity.frame_index;
    G10SecondUseCausalSupplyInput causal{};
    causal.completed_k_minus_1_descending_lag_publication =
        completed_k_minus_1_descending;
    state.supply_provider.Update(
        frame,
        forecast_pointer,
        gamma_pointer,
        causal,
        prepared.supply,
        status);
    if (!status.ok())
    {
        return false;
    }
    if (!prepared.supply.valid)
    {
        ResetG10ManeuverState(state);
        return false;
    }

    const PursuitOvershootForecast* completed_forecast =
        state.completed_forecast_available
            ? &state.completed_forecast
            : nullptr;
    const AdversaryReversalObservation* completed_reversal =
        state.completed_reversal_available
            ? &state.completed_reversal
            : nullptr;
    state.admission_provider.Update(
        frame,
        true,
        completed_forecast,
        completed_reversal,
        prepared.supply.overshoot_realized_t_sec,
        false,
        prepared.admission,
        status);
    if (!status.ok())
    {
        return false;
    }
    if (!prepared.admission.valid)
    {
        return false;
    }

    G10SecondUseOwnerInput owner_input{};
    owner_input.gate_enabled = true;
    owner_input.root_command_available = true;
    owner_input.root_command.intent = habfm_base;
    owner_input.root_command.label = G10SecondUseCommandLabel::Upstream;
    owner_input.bridge = prepared.admission.bridge;
    owner_input.supply = prepared.supply;
    owner_input.sample_dt_s = tactical_input.accepted_estimator.sample_dt_s;

    const double current_nz = tactical_input.current_envelope.nz_feasible_g;
    const bool current_nz_available =
        LadyLuck::runtime::CurrentCommandEnvelopeAvailable(tactical_input)
        && std::isfinite(current_nz)
        && current_nz > 0.0;
    owner_input.runtime.nz_feasible_source_nonempty = current_nz_available;
    owner_input.runtime.nz_feasible_g.has_value = current_nz_available;
    owner_input.runtime.nz_feasible_g.value = current_nz;

    state.owner.Update(frame, owner_input, prepared.owner, status);
    if (!status.ok() || !prepared.owner.valid)
    {
        if (status.ok()) status.code = StatusCode::InvalidConfiguration;
        return false;
    }

    if (prepared.owner.released_this_tick)
    {
        ResetG10ManeuverState(state);
    }
    if (prepared.owner.reason
            != G10SecondUseOwnerReason::SecondUseCommandPublished
        || !prepared.owner.engaged)
    {
        return false;
    }

    prepared.selected_intent = prepared.owner.command.intent;
    switch (prepared.owner.command.label)
    {
    case G10SecondUseCommandLabel::PitchUp:
        prepared.selected_intent.behavior_id =
            DoctrineBehaviorId::G10SecondUsePitchUp;
        break;
    case G10SecondUseCommandLabel::PositiveLoadedWinding:
        prepared.selected_intent.behavior_id =
            DoctrineBehaviorId::G10SecondUsePositiveLoadedWinding;
        break;
    case G10SecondUseCommandLabel::DescendingLag:
        prepared.selected_intent.behavior_id =
            DoctrineBehaviorId::G10SecondUseDescendingLag;
        break;
    case G10SecondUseCommandLabel::Upstream:
    default:
        status.code = StatusCode::InvalidConfiguration;
        return false;
    }
    prepared.selected_intent.writer_id = ControlIntentWriterG10SecondUse;
    prepared.selected_intent.Validate(status);
    return status.ok();
}

bool ApplyH09AltitudeStorage(
    const TacticalCommandBuildInput& tactical_input,
    const LadyLuck::HabfmTerminalControlIntentObservation& observation,
    LadyLuck::runtime::ICurrentCisV4EnergyProjectionPort& projection_port,
    LadyLuck::behavior_tree::static_bt::StaticHabfmG10Prepared& prepared,
    Status& status) noexcept
{
    using LadyLuck::ControlIntentWriterHabfm;
    using LadyLuck::ControlRouteKind;
    using LadyLuck::HabfmActiveBranch;
    using LadyLuck::HabfmEngageDecisionState;
    using LadyLuck::constants::StandardGravityMps2;
    using LadyLuck::guidance::habfm::HabfmH09ResidualClimbInput;
    using LadyLuck::guidance::habfm::HabfmH09StorageReason;

    status = Status{};
    prepared.h09_storage =
        LadyLuck::guidance::habfm::HabfmH09AltitudeStorageAdmission{};
    prepared.h09_projection =
        LadyLuck::runtime::CurrentCisV4EnergyProjectionReceipt{};
    prepared.h09_allocation =
        LadyLuck::guidance::habfm::HabfmH09ResidualClimbAllocation{};
    prepared.h09_projection_attempted = false;
    prepared.h09_active = false;

    LadyLuck::guidance::habfm::EvaluateHabfmH09AltitudeStorage(
        tactical_input,
        observation.frame_evidence,
        prepared.habfm_base.active_output,
        prepared.h09_storage,
        status);
    if (!status.ok() || !prepared.h09_storage.admitted)
    {
        status = Status{};
        return false;
    }

    const auto reject = [&prepared](
        const HabfmH09StorageReason reason) noexcept
    {
        prepared.h09_storage.admitted = false;
        prepared.h09_storage.reason = reason;
    };

    const LadyLuck::control::route5::CommandEnvelope& envelope =
        tactical_input.current_envelope;
    if (!LadyLuck::runtime::CurrentCommandEnvelopeAvailable(tactical_input)
        || !std::isfinite(envelope.nz_feasible_g)
        || envelope.nz_feasible_g <= 0.0)
    {
        reject(HabfmH09StorageReason::CurrentPhysicalAuthorityUnavailable);
        return false;
    }
    if (!tactical_input.accepted_estimator.mass_valid
        || !std::isfinite(tactical_input.accepted_estimator.mass)
        || tactical_input.accepted_estimator.mass <= 0.0)
    {
        reject(HabfmH09StorageReason::CurrentMassUnavailable);
        return false;
    }
    if (!prepared.habfm_base.active_output.turn_side_sign.has_value
        || (prepared.habfm_base.active_output.turn_side_sign.value != -1
            && prepared.habfm_base.active_output.turn_side_sign.value != 1))
    {
        reject(HabfmH09StorageReason::TurnSideUnavailable);
        return false;
    }
    if (prepared.selected_intent.route_kind != ControlRouteKind::AimPoint
        || prepared.selected_intent.writer_id != ControlIntentWriterHabfm
        || prepared.selected_intent.behavior_id
            != DoctrineBehaviorId::HabfmTwoCircle
        || prepared.habfm_base.active_output.branch
            != HabfmActiveBranch::TwoCircle)
    {
        reject(HabfmH09StorageReason::FinalIntentUnavailable);
        return false;
    }

    const auto build_candidate = [&prepared, &tactical_input](
        const double energy_rate_m2ps3,
        ControlIntent& candidate) noexcept -> bool
    {
        candidate = prepared.selected_intent;
        const double speed = prepared.h09_storage.own_speed_mps;
        if (!std::isfinite(energy_rate_m2ps3)
            || energy_rate_m2ps3 <= 0.0
            || !std::isfinite(speed)
            || speed <= 0.0)
        {
            return false;
        }
        const double climb_rate = energy_rate_m2ps3
            / LadyLuck::constants::StandardGravityMps2;
        if (!std::isfinite(climb_rate)
            || climb_rate <= 0.0
            || climb_rate >= speed)
        {
            return false;
        }
        const double gamma_cmd = std::asin(climb_rate / speed);
        const double tangent = std::tan(gamma_cmd);
        double down_offset_m = 0.0;
        double aim_down_m = 0.0;
        if (!std::isfinite(gamma_cmd)
            || !std::isfinite(tangent)
            || !std::isfinite(tactical_input.frame.own_offense.range_m)
            || tactical_input.frame.own_offense.range_m <= 0.0
            || !SafeMultiply(
                tangent,
                tactical_input.frame.own_offense.range_m,
                down_offset_m)
            || !SafeSubtract(
                tactical_input.frame.own.position_ned_m[2],
                down_offset_m,
                aim_down_m))
        {
            return false;
        }
        candidate.aim_point_m[2] = aim_down_m;
        candidate.desired_speed_mps =
            prepared.h09_storage.sustained_speed_mps;
        candidate.desired_speed_rate_mps2 = 0.0;
        Status candidate_status{};
        candidate.Validate(candidate_status);
        return candidate_status.ok();
    };

    double finalized_rate = prepared.h09_storage.admitted_energy_rate_m2ps3;
    ControlIntent projected{};
    LadyLuck::runtime::CurrentCisV4EnergyProjectionReceipt projection{};
    Status projection_status{};
    prepared.h09_projection_attempted = true;
    projection_port.Project(
        prepared.selected_intent, projected, projection, projection_status);
    prepared.h09_projection = projection;
    if (!projection_status.ok()
        || !projection.evaluated
        || !projection.admitted
        || !projection.raw_rate_measurement_valid
        || !std::isfinite(
            projection.raw_specific_energy_rate_measured_m2ps3)
        || projection.raw_specific_energy_rate_measured_m2ps3 <= 0.0)
    {
        reject(HabfmH09StorageReason::CurrentProjectionUnavailable);
        return false;
    }
    finalized_rate = (std::min)(
        finalized_rate,
        projection.raw_specific_energy_rate_measured_m2ps3);
    prepared.h09_storage.measured_energy_rate_m2ps3 =
        projection.raw_specific_energy_rate_measured_m2ps3;
    ControlIntent provisional{};
    if (!build_candidate(finalized_rate, provisional))
    {
        reject(HabfmH09StorageReason::FinalIntentUnavailable);
        return false;
    }
    const double speed = prepared.h09_storage.own_speed_mps;
    const double climb_rate = finalized_rate / StandardGravityMps2;
    const double gamma_cmd = std::asin(climb_rate / speed);
    const double horizontal_speed = std::hypot(
        tactical_input.frame.own.velocity_ned_mps[0],
        tactical_input.frame.own.velocity_ned_mps[1]);
    const double gamma = std::atan2(
        -tactical_input.frame.own.velocity_ned_mps[2],
        horizontal_speed);
    double gamma_error = 0.0;
    double gamma_rate_raw = 0.0;
    if (!std::isfinite(projection.projected_thrust_cmd_limited_n)
        || !std::isfinite(projection.route_k_gamma_per_s)
        || projection.route_k_gamma_per_s < 0.0
        || !std::isfinite(projection.route_gamma_rate_limit_radps)
        || projection.route_gamma_rate_limit_radps < 0.0
        || !std::isfinite(gamma_cmd)
        || !std::isfinite(gamma)
        || !SafeSubtract(gamma_cmd, gamma, gamma_error)
        || !SafeMultiply(
            projection.route_k_gamma_per_s,
            gamma_error,
            gamma_rate_raw))
    {
        reject(HabfmH09StorageReason::CurrentThrustPreviewUnavailable);
        return false;
    }
    const double gamma_rate = (std::max)(
        -projection.route_gamma_rate_limit_radps,
        (std::min)(
            gamma_rate_raw,
            projection.route_gamma_rate_limit_radps));

    HabfmH09ResidualClimbInput allocation_input{};
    allocation_input.frame_identity = tactical_input.frame.frame_identity;
    allocation_input.velocity_ned_mps =
        tactical_input.frame.own.velocity_ned_mps;
    allocation_input.body_forward_ned = tactical_input.frame.own.nose_ned;
    allocation_input.side_sign =
        prepared.habfm_base.active_output.turn_side_sign.value;
    allocation_input.sustained_speed_mps =
        prepared.h09_storage.sustained_speed_mps;
    allocation_input.sustained_course_rate_radps =
        prepared.h09_storage.sustained_turn_rate_radps;
    allocation_input.speed_reference_rate_mps2 = 0.0;
    allocation_input.residual_energy_rate_m2ps3 = finalized_rate;
    allocation_input.tecs_thrust_command_n =
        projection.projected_thrust_cmd_limited_n;
    allocation_input.mass_kg = tactical_input.accepted_estimator.mass;
    allocation_input.instantaneous_load_limit_g = envelope.nz_feasible_g;
    allocation_input.flight_path_rate_command_radps = gamma_rate;
    LadyLuck::guidance::habfm::AllocateHabfmH09ResidualClimb(
        allocation_input, prepared.h09_allocation, status);
    if (!status.ok() || !prepared.h09_allocation.admitted)
    {
        status = Status{};
        reject(prepared.h09_allocation.reason);
        return false;
    }

    ControlIntent candidate = provisional;
    candidate.route_kind = ControlRouteKind::DirectLoadVectorAcceleration;
    candidate.direct_load_vector_acceleration_ned_mps2.has_value = true;
    candidate.direct_load_vector_acceleration_ned_mps2.value =
        prepared.h09_allocation.adapter_inertial_acceleration_ned_mps2;
    candidate.total_load_factor_limit_g.has_value = true;
    candidate.total_load_factor_limit_g.value = envelope.nz_feasible_g;
    Status candidate_status{};
    candidate.Validate(candidate_status);
    if (!candidate_status.ok())
    {
        reject(HabfmH09StorageReason::FinalIntentUnavailable);
        return false;
    }

    prepared.h09_storage.admitted_energy_rate_m2ps3 = finalized_rate;
    prepared.h09_storage.climb_rate_mps = climb_rate;
    prepared.h09_storage.climb_gamma_cmd_rad = gamma_cmd;
    prepared.selected_intent = candidate;
    prepared.h09_active = true;
    return true;
}

void ObserveNextFrameEvidence(
    const DogfightGeometryFrame& frame,
    StaticHabfmG10State& state,
    StatusCode& optional_status) noexcept
{
    LadyLuck::guidance::obfm::PursuitOvershootForecaster forecaster{};
    PursuitOvershootForecast forecast{};
    Status forecast_status{};
    forecaster.Update(frame, forecast, forecast_status);
    if (forecast_status.ok())
    {
        state.completed_forecast = forecast;
        state.completed_forecast_available = forecast.valid;
    }
    else if (optional_status == StatusCode::Ok)
    {
        optional_status = forecast_status.code;
    }

    AdversaryReversalObservation reversal{};
    Status reversal_status{};
    state.reversal_observer.Update(frame, reversal, reversal_status);
    if (reversal_status.ok())
    {
        state.completed_reversal = reversal;
        state.completed_reversal_available = reversal.valid;
    }
    else if (optional_status == StatusCode::Ok)
    {
        optional_status = reversal_status.code;
    }
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

StaticHabfmG10StagedOwner::StaticHabfmG10StagedOwner() noexcept
{
    Reset();
}

void StaticHabfmG10StagedOwner::Reset() noexcept
{
    habfm_owner_.Reset();
    g10_state_ = StaticHabfmG10State{};
    generation_ = 0U;
}

void StaticHabfmG10StagedOwner::Observe(
    const HabfmTerminalControlIntentInput& input,
    HabfmTerminalControlIntentObservation& output,
    Status& status) const noexcept
{
    habfm_owner_.Observe(input, output, status);
}

void StaticHabfmG10StagedOwner::Prepare(
    const runtime::TacticalCommandBuildInput& tactical_input,
    runtime::ICurrentCisV4EnergyProjectionPort* const projection_port,
    const HabfmTerminalControlIntentObservation& observation,
    StaticHabfmG10Prepared& output,
    Status& status) const noexcept
{
    output = StaticHabfmG10Prepared{};
    output.prepare_attempted = true;
    output.frame_identity = tactical_input.frame.frame_identity;
    output.captured_generation = generation_;
    status = Status{};

    HabfmTerminalControlIntentInput habfm_input{};
    habfm_input.frame = tactical_input.frame;
    habfm_input.current_envelope = tactical_input.current_envelope;
    habfm_owner_.PrepareCandidate(
        habfm_input, observation, output.habfm_base, status);
    if (!status.ok()
        || !output.habfm_base.selected
        || !output.habfm_base.next_state_ready)
    {
        if (status.ok()) status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.selected_intent = output.habfm_base.intent;
    output.disposition = StaticHabfmG10Disposition::HabfmBaseSelected;
    output.next_state = g10_state_;

    HabfmAvoidPassOverlayInput avoid_input{};
    avoid_input.frame = tactical_input.frame;
    avoid_input.upstream_intent = output.habfm_base.intent;
    avoid_input.decision = output.habfm_base.engage_decision;
    avoid_input.modifier_writer_id = ControlIntentWriterHabfmAvoidPass;
    Status avoid_status{};
    BuildHabfmAvoidPassOverlay(
        avoid_input, output.avoid_overlay, avoid_status);
    const bool avoid_selected = avoid_status.ok()
        && output.avoid_overlay.valid
        && output.avoid_overlay.modified;
    if (!avoid_status.ok())
    {
        output.optional_avoid_status_code = avoid_status.code;
    }
    if (avoid_selected)
    {
        output.selected_intent = output.avoid_overlay.candidate;
        output.disposition =
            StaticHabfmG10Disposition::HabfmAvoidPassSelected;
    }

    StaticHabfmG10State g10_candidate = g10_state_;
    if (avoid_selected)
    {
        ResetG10ManeuverState(g10_candidate);
    }
    Status g10_status{};
    const bool g10_selected = avoid_selected
        ? false
        : BuildG10Candidate(
            tactical_input,
            output.habfm_base.intent,
            g10_candidate,
            output,
            g10_status);
    if (!g10_status.ok())
    {
        output.optional_g10_status_code = g10_status.code;
        g10_candidate = g10_state_;
        output.selected_intent = output.habfm_base.intent;
        output.disposition =
            StaticHabfmG10Disposition::HabfmBaseSelected;
    }
    else if (g10_selected)
    {
        output.disposition = StaticHabfmG10Disposition::G10SecondUseSelected;
    }

    if (!avoid_selected && !g10_selected && projection_port != nullptr)
    {
        Status h09_status{};
        static_cast<void>(ApplyH09AltitudeStorage(
            tactical_input,
            observation,
            *projection_port,
            output,
            h09_status));
        if (!h09_status.ok()
            && output.optional_h09_status_code == StatusCode::Ok)
        {
            output.optional_h09_status_code = h09_status.code;
        }
    }

    ObserveNextFrameEvidence(
        tactical_input.frame,
        g10_candidate,
        output.optional_g10_status_code);
    output.next_state = g10_candidate;
    output.next_state_ready = true;
    status = Status{};
}

void StaticHabfmG10StagedOwner::ValidatePublished(
    const StaticHabfmG10Prepared& prepared,
    const std::uint32_t writer_id,
    Status& status) const noexcept
{
    status = Status{};
    if (!prepared.prepare_attempted
        || !prepared.next_state_ready
        || prepared.committed
        || prepared.captured_generation != generation_
        || !SameControlFrameIdentity(
            prepared.frame_identity, prepared.selected_intent.frame_identity)
        || prepared.selected_intent.writer_id != writer_id)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    prepared.selected_intent.Validate(status);
    if (!status.ok()) return;

    if (writer_id == ControlIntentWriterHabfm
        || writer_id == ControlIntentWriterHabfmAvoidPass)
    {
        habfm_owner_.ValidatePublished(prepared.habfm_base, status);
        if (status.ok()
            && writer_id == ControlIntentWriterHabfmAvoidPass
            && prepared.disposition
                != StaticHabfmG10Disposition::HabfmAvoidPassSelected)
        {
            status.code = StatusCode::InvalidConfiguration;
        }
        return;
    }
    if (writer_id != ControlIntentWriterG10SecondUse
        || prepared.disposition
            != StaticHabfmG10Disposition::G10SecondUseSelected)
    {
        status.code = StatusCode::InvalidConfiguration;
    }
}

void StaticHabfmG10StagedOwner::CommitPublished(
    StaticHabfmG10Prepared& prepared,
    const std::uint32_t writer_id,
    Status& status) noexcept
{
    ValidatePublished(prepared, writer_id, status);
    if (!status.ok()) return;

    if (writer_id == ControlIntentWriterHabfm
        || writer_id == ControlIntentWriterHabfmAvoidPass)
    {
        habfm_owner_.CommitPublished(prepared.habfm_base, status);
        if (!status.ok()) return;
    }

    g10_state_ = prepared.next_state;
    const bool descending = writer_id == ControlIntentWriterG10SecondUse
        && prepared.selected_intent.behavior_id
            == DoctrineBehaviorId::G10SecondUseDescendingLag;
    g10_state_.last_descending_publication_available = descending;
    g10_state_.last_descending_publication_identity = descending
        ? prepared.frame_identity
        : ControlFrameIdentity{};
    ++generation_;
    prepared.committed = true;
    status = Status{};
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
