#include "LadyLuck/guidance/prefire/RootPrefireThreatObservation.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::WezPhase;
using LadyLuck::guidance::prefire::PrefireOptionalDouble;
using LadyLuck::guidance::prefire::RootPrefireControlAdmissionReason;
using LadyLuck::guidance::prefire::RootPrefireControlPathEvidence;
using LadyLuck::guidance::prefire::RootPrefireThreatConsumerReason;
using LadyLuck::guidance::prefire::RootPrefireThreatShadowReason;
using LadyLuck::guidance::prefire::RootPrefireThreatShadowReceipt;

constexpr double kFloat32Epsilon =
    static_cast<double>(std::numeric_limits<float>::epsilon());

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
}

bool OfficialEnemyGunThreat(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    const double damage = frame.enemy_offense.damage_rate;
    if (!std::isfinite(damage))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (damage < 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    output = damage > 0.0;
    return true;
}

double AvailableTurnRateRadS(
    const double n_max_g,
    const double speed_mps) noexcept
{
    return LadyLuck::constants::StandardGravityMps2
        * std::sqrt(n_max_g * n_max_g - 1.0)
        / speed_mps;
}

void FailShadow(
    RootPrefireThreatShadowReceipt& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = RootPrefireThreatShadowReceipt{};
    output.reason = RootPrefireThreatShadowReason::
        PrefireThreatObserverContractRejected;
    status.code = code;
}

struct ExtendCueObservation
{
    bool available = false;
    bool candidate = false;
};

bool ObserveExtendCue(
    const DogfightGeometryFrame& frame,
    ExtendCueObservation& output,
    Status& status) noexcept
{
    output = ExtendCueObservation{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_nose = frame.opponent.nose_ned;
    if (!FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_nose))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    const Vector3 los = Subtract(own_position, adversary_position);
    const double los_norm = NumpyNorm3(los);
    const double speed = NumpyNorm3(own_velocity);
    const double nose_norm = NumpyNorm3(adversary_nose);
    if (!std::isfinite(los_norm)
        || !std::isfinite(speed)
        || !std::isfinite(nose_norm))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (los_norm < LadyLuck::constants::Tiny
        || speed < LadyLuck::constants::Tiny
        || nose_norm < LadyLuck::constants::Tiny)
    {
        // These are finite but do not define the normalized directions used
        // by the CHECK_EXTEND observation.  Absence of that optional evidence
        // is not a contradiction in the already-admitted BREAK lifecycle.
        return true;
    }
    const Vector3 los_hat = Scale(los, 1.0 / los_norm);
    const Vector3 nose_hat = Scale(adversary_nose, 1.0 / nose_norm);
    const Vector3 velocity_hat = Scale(own_velocity, 1.0 / speed);
    const double forward_projection = Dot3(nose_hat, los_hat);
    const bool nose_forward = forward_projection > 0.0;
    const Vector3 nose_perpendicular = Subtract(
        nose_hat,
        Scale(los_hat, forward_projection));
    const double lag_sign = Dot3(nose_perpendicular, velocity_hat);
    output.available = true;
    output.candidate = !nose_forward || lag_sign < 0.0;
    return true;
}

void ControlPathAdmission(
    const RootPrefireControlPathEvidence& evidence,
    bool& admitted,
    RootPrefireControlAdmissionReason& reason) noexcept
{
    (void)evidence;
    // The response is selected from current same-index threat geometry and is
    // executed by the current FCS path.  The Root caller withholds this
    // consumer for an actual terrain-recovery/Auto-GCAS demand; the mere
    // absence of an otherwise optional safety receipt is not a second entry
    // veto. Age-1 backend telemetry remains diagnostic and is not causal
    // authority for this frame.
    admitted = true;
    reason = RootPrefireControlAdmissionReason::PrefireControlPathAdmitted;
}

RootPrefireThreatConsumerReason ConsumerReasonFromControl(
    const RootPrefireControlAdmissionReason reason) noexcept
{
    switch (reason)
    {
    case RootPrefireControlAdmissionReason::PrefireControlFeedbackNotFresh:
        return RootPrefireThreatConsumerReason::
            PrefireControlFeedbackNotFresh;
    case RootPrefireControlAdmissionReason::PrefireControlBackendUntrusted:
        return RootPrefireThreatConsumerReason::
            PrefireControlBackendUntrusted;
    case RootPrefireControlAdmissionReason::PrefireControlPathAdmitted:
        return RootPrefireThreatConsumerReason::
            PrefireBreakEnteredControlPathAdmitted;
    case RootPrefireControlAdmissionReason::PrefireControlFeedbackUnavailable:
    default:
        return RootPrefireThreatConsumerReason::
            PrefireControlFeedbackUnavailable;
    }
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

const char* RootPrefireMarginReasonLabel(
    const RootPrefireMarginReason reason) noexcept
{
    switch (reason)
    {
    case RootPrefireMarginReason::OfficialRangeEntryNotProven:
        return "official_range_entry_not_proven";
    case RootPrefireMarginReason::ConeSweepMemoryNotEstablished:
        return "cone_sweep_memory_not_established";
    case RootPrefireMarginReason::AttackerConeEntryNotProven:
        return "attacker_cone_entry_not_proven";
    case RootPrefireMarginReason::SolutionEntryBeforeOwnTurn:
        return "solution_entry_before_own_turn";
    case RootPrefireMarginReason::OwnTurnCompletesBeforeSolutionEntry:
        return "own_turn_completes_before_solution_entry";
    case RootPrefireMarginReason::CapabilityEvidenceNotAdmitted:
    default:
        return "capability_evidence_not_admitted";
    }
}

const char* RootPrefireThreatShadowReasonLabel(
    const RootPrefireThreatShadowReason reason) noexcept
{
    switch (reason)
    {
    case RootPrefireThreatShadowReason::OfficialGunThreatAlreadyActive:
        return "official_gun_threat_already_active";
    case RootPrefireThreatShadowReason::NoElapsedNonScratchPhase:
        return "no_elapsed_non_scratch_phase";
    case RootPrefireThreatShadowReason::CapabilityEvidenceNotAdmitted:
        return "capability_evidence_not_admitted";
    case RootPrefireThreatShadowReason::NoPrefireBreakCandidate:
        return "no_prefire_break_candidate";
    case RootPrefireThreatShadowReason::PrefireBreakCandidateObserved:
        return "prefire_break_candidate_observed";
    case RootPrefireThreatShadowReason::FiniteKinematicsUnavailable:
        return "finite_kinematics_unavailable";
    case RootPrefireThreatShadowReason::PrefireThreatObserverContractRejected:
    default:
        return "prefire_threat_observer_contract_rejected";
    }
}

const char* RootPrefireControlAdmissionReasonLabel(
    const RootPrefireControlAdmissionReason reason) noexcept
{
    switch (reason)
    {
    case RootPrefireControlAdmissionReason::PrefireControlFeedbackNotFresh:
        return "prefire_control_feedback_not_fresh";
    case RootPrefireControlAdmissionReason::PrefireControlBackendUntrusted:
        return "prefire_control_backend_untrusted";
    case RootPrefireControlAdmissionReason::PrefireControlPathAdmitted:
        return "prefire_control_path_admitted";
    case RootPrefireControlAdmissionReason::PrefireControlFeedbackUnavailable:
    default:
        return "prefire_control_feedback_unavailable";
    }
}

const char* RootPrefireThreatConsumerReasonLabel(
    const RootPrefireThreatConsumerReason reason) noexcept
{
    switch (reason)
    {
    case RootPrefireThreatConsumerReason::CheckExtendRelease:
        return "check_extend_release";
    case RootPrefireThreatConsumerReason::PrefireMarginCleared:
        return "prefire_margin_cleared";
    case RootPrefireThreatConsumerReason::PrefireControlFeedbackUnavailable:
        return "prefire_control_feedback_unavailable";
    case RootPrefireThreatConsumerReason::PrefireControlFeedbackNotFresh:
        return "prefire_control_feedback_not_fresh";
    case RootPrefireThreatConsumerReason::PrefireControlBackendUntrusted:
        return "prefire_control_backend_untrusted";
    case RootPrefireThreatConsumerReason::PrefireEntrySideNotAdmitted:
        return "prefire_entry_side_not_admitted";
    case RootPrefireThreatConsumerReason::PrefireBreakEnteredControlPathAdmitted:
        return "prefire_break_entered:prefire_control_path_admitted";
    case RootPrefireThreatConsumerReason::OfficialThreatActive:
        return "official_threat_active";
    case RootPrefireThreatConsumerReason::PrefireBreakContinued:
        return "prefire_break_continued";
    case RootPrefireThreatConsumerReason::CheckExtendObservationRejected:
        return "check_extend_observation_rejected";
    case RootPrefireThreatConsumerReason::CheckExtendHold:
        return "check_extend_hold";
    case RootPrefireThreatConsumerReason::PrefireCandidateNotRearmed:
        return "prefire_candidate_not_rearmed";
    case RootPrefireThreatConsumerReason::PrefireThreatConsumerContractRejected:
        return "prefire_threat_consumer_contract_rejected";
    case RootPrefireThreatConsumerReason::NoPrefireBreakDemand:
    default:
        return "no_prefire_break_demand";
    }
}

RootPrefireThreatObserver::RootPrefireThreatObserver() noexcept
{
    Reset();
}

void RootPrefireThreatObserver::Reset() noexcept
{
    previous_valid_ = false;
    previous_t_s_ = 0.0;
    previous_ata_rad_ = 0.0;
}

void RootPrefireThreatObserver::Update(
    const DogfightGeometryFrame& frame,
    const PrefireOptionalDouble& capability_n_max_g,
    const bool capability_n_max_admitted,
    RootPrefireThreatShadowReceipt& output,
    Status& status) noexcept
{
    output = RootPrefireThreatShadowReceipt{};
    status = Status{};

    const double t_sec = frame.t_sec;
    const double ata_rad = std::fabs(frame.enemy_offense.ata_rad);
    const double range_m = frame.enemy_offense.range_m;
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!std::isfinite(t_sec)
        || t_sec < 0.0
        || !std::isfinite(ata_rad)
        || ata_rad < 0.0
        || ata_rad > constants::Pi
        || !std::isfinite(range_m)
        || range_m <= 0.0
        || !FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity))
    {
        Reset();
        FailShadow(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const bool had_previous = previous_valid_;
    const double previous_t = previous_t_s_;
    const double previous_ata = previous_ata_rad_;
    previous_valid_ = true;
    previous_t_s_ = t_sec;
    previous_ata_rad_ = ata_rad;

    bool dt_valid = false;
    double dt_s = 0.0;
    double ata_delta_rad = 0.0;
    if (had_previous)
    {
        const double candidate_dt = t_sec - previous_t;
        if (std::isfinite(candidate_dt) && candidate_dt > 0.0)
        {
            dt_valid = true;
            dt_s = candidate_dt;
            ata_delta_rad = ata_rad - previous_ata;
        }
    }

    bool official_threat = false;
    if (!OfficialEnemyGunThreat(frame, official_threat, status))
    {
        FailShadow(output, status, status.code);
        return;
    }
    if (official_threat)
    {
        output.reason = RootPrefireThreatShadowReason::
            OfficialGunThreatAlreadyActive;
        return;
    }

    std::array<WezPhase, RootPrefireNonScratchPhaseCount> elapsed{};
    std::size_t elapsed_count = 0U;
    for (std::size_t index = 0U;
         index < RootPrefireNonScratchPhaseCount;
         ++index)
    {
        const Result<WezPhase> phase = OfficialWezPhaseAt(index);
        if (phase.status.code != StatusCode::Ok)
        {
            FailShadow(output, status, phase.status.code);
            return;
        }
        if (t_sec >= phase.value.start_sec)
        {
            elapsed[elapsed_count++] = phase.value;
        }
    }
    if (elapsed_count == 0U)
    {
        output.evaluated = true;
        output.reason = RootPrefireThreatShadowReason::
            NoElapsedNonScratchPhase;
        return;
    }

    const bool capability_ok = capability_n_max_admitted
        && capability_n_max_g.has_value
        && std::isfinite(capability_n_max_g.value)
        && capability_n_max_g.value > 1.0;
    const Vector3 relative_position = Subtract(
        adversary_position,
        own_position);
    const double separation_m = NumpyNorm3(relative_position);
    const double own_speed_mps = NumpyNorm3(own_velocity);
    if (!std::isfinite(separation_m)
        || !std::isfinite(own_speed_mps))
    {
        FailShadow(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (separation_m <= 0.0 || own_speed_mps <= 0.0)
    {
        // Python raises ValueError here.  In the production BT this observer
        // is command-neutral and runs before owner selection, so a finite
        // geometric singularity is an ordinary unavailable observation, not
        // a frame-contract failure.  Preserve the already accepted causal
        // (time, ATA) sample and publish no pre-fire authority.
        output.reason = RootPrefireThreatShadowReason::
            FiniteKinematicsUnavailable;
        return;
    }
    const Vector3 relative_velocity = Subtract(
        adversary_velocity,
        own_velocity);
    const double range_rate_mps =
        Dot3(relative_position, relative_velocity) / separation_m;
    const Vector3 los_hat = Scale(relative_position, 1.0 / separation_m);
    const Vector3 own_velocity_hat = Scale(
        own_velocity,
        1.0 / own_speed_mps);
    const double required_turn_rad = std::acos((std::max)(
        -1.0,
        (std::min)(1.0, Dot3(own_velocity_hat, los_hat))));
    PrefireOptionalDouble time_to_face_s{};
    if (capability_ok)
    {
        time_to_face_s.has_value = true;
        time_to_face_s.value = required_turn_rad
            / AvailableTurnRateRadS(
                capability_n_max_g.value,
                own_speed_mps);
    }

    output.phase_observation_count = elapsed_count;
    for (std::size_t index = 0U; index < elapsed_count; ++index)
    {
        const WezPhase& phase = elapsed[index];
        RootPrefirePhaseObservation& observation =
            output.phase_observations[index];
        observation.phase = phase;

        observation.range_satisfied =
            phase.min_range_m <= range_m
            && range_m <= phase.max_range_m;
        if (observation.range_satisfied)
        {
            observation.time_to_range_s.has_value = true;
            observation.time_to_range_s.value = 0.0;
        }
        else if (range_m > phase.max_range_m && range_rate_mps < 0.0)
        {
            observation.range_closing_proven = true;
            observation.time_to_range_s.has_value = true;
            observation.time_to_range_s.value =
                (range_m - phase.max_range_m) / -range_rate_mps;
        }
        else if (range_m < phase.min_range_m && range_rate_mps > 0.0)
        {
            observation.range_closing_proven = true;
            observation.time_to_range_s.has_value = true;
            observation.time_to_range_s.value =
                (phase.min_range_m - range_m) / range_rate_mps;
        }

        observation.cone_satisfied = ata_rad <= phase.angle_rad;
        if (observation.cone_satisfied)
        {
            observation.time_to_cone_s.has_value = true;
            observation.time_to_cone_s.value = 0.0;
        }
        else if (dt_valid)
        {
            const double evidence_band = (std::max)(
                (std::max)(std::fabs(ata_rad), std::fabs(previous_ata)),
                1.0)
                * kFloat32Epsilon;
            if (ata_delta_rad < -evidence_band)
            {
                observation.cone_closing_proven = true;
                const double cone_rate_rad_s = -ata_delta_rad / dt_s;
                observation.time_to_cone_s.has_value = true;
                observation.time_to_cone_s.value =
                    (ata_rad - phase.angle_rad) / cone_rate_rad_s;
            }
        }

        if (observation.time_to_range_s.has_value
            && observation.time_to_cone_s.has_value)
        {
            observation.time_to_solution_s.has_value = true;
            observation.time_to_solution_s.value = (std::max)(
                observation.time_to_range_s.value,
                observation.time_to_cone_s.value);
        }
        const bool margin_break = capability_ok
            && observation.time_to_solution_s.has_value
            && time_to_face_s.has_value
            && observation.time_to_solution_s.value
                < time_to_face_s.value;
        observation.margin.admitted = capability_ok;
        observation.margin.margin_break = margin_break;
        observation.margin.scoring_gap_m =
            range_m - phase.max_range_m;
        observation.margin.time_to_score_s =
            observation.time_to_solution_s;
        observation.margin.time_to_face_s = time_to_face_s;
        if (!capability_ok)
        {
            observation.margin.reason = RootPrefireMarginReason::
                CapabilityEvidenceNotAdmitted;
        }
        else if (!observation.time_to_range_s.has_value)
        {
            observation.margin.reason = RootPrefireMarginReason::
                OfficialRangeEntryNotProven;
        }
        else if (!observation.time_to_cone_s.has_value)
        {
            observation.margin.reason = had_previous
                ? RootPrefireMarginReason::AttackerConeEntryNotProven
                : RootPrefireMarginReason::ConeSweepMemoryNotEstablished;
        }
        else
        {
            observation.margin.reason = margin_break
                ? RootPrefireMarginReason::SolutionEntryBeforeOwnTurn
                : RootPrefireMarginReason::
                    OwnTurnCompletesBeforeSolutionEntry;
        }
    }

    output.evaluated = true;
    if (!capability_ok)
    {
        output.reason = RootPrefireThreatShadowReason::
            CapabilityEvidenceNotAdmitted;
        return;
    }

    bool candidate_valid = false;
    std::size_t candidate_index = 0U;
    for (std::size_t index = 0U; index < elapsed_count; ++index)
    {
        const RootPrefirePhaseObservation& item =
            output.phase_observations[index];
        if (!item.margin.margin_break)
        {
            continue;
        }
        if (!candidate_valid)
        {
            candidate_valid = true;
            candidate_index = index;
            continue;
        }
        const RootPrefirePhaseObservation& current =
            output.phase_observations[candidate_index];
        const double item_time = item.time_to_solution_s.has_value
            ? item.time_to_solution_s.value
            : (std::numeric_limits<double>::infinity)();
        const double current_time = current.time_to_solution_s.has_value
            ? current.time_to_solution_s.value
            : (std::numeric_limits<double>::infinity)();
        if (item_time < current_time
            || (item_time == current_time
                && item.phase.index < current.phase.index))
        {
            candidate_index = index;
        }
    }
    output.admitted = true;
    if (!candidate_valid)
    {
        output.reason = RootPrefireThreatShadowReason::
            NoPrefireBreakCandidate;
        return;
    }

    output.prefire_break_candidate = true;
    output.reason = RootPrefireThreatShadowReason::
        PrefireBreakCandidateObserved;
    output.candidate_phase_valid = true;
    output.candidate_phase =
        output.phase_observations[candidate_index].phase.id;
}

RootPrefireThreatConsumer::RootPrefireThreatConsumer() noexcept
{
    Reset();
}

void RootPrefireThreatConsumer::Reset() noexcept
{
    break_active_ = false;
    official_seen_ = false;
    check_extend_hold_ = false;
    entry_armed_ = true;
}

void RootPrefireThreatConsumer::Update(
    const DogfightGeometryFrame& frame,
    const RootPrefireThreatShadowReceipt& shadow,
    const SameIndexGeometryFrameEnvelope* const envelope,
    const bool official_threat,
    const RootPrefireControlPathEvidence& control_evidence,
    RootPrefireThreatConsumerDecision& output,
    Status& status) noexcept
{
    output = RootPrefireThreatConsumerDecision{};
    status = Status{};
    if (shadow.phase_observation_count > RootPrefireNonScratchPhaseCount
        || (shadow.prefire_break_candidate
            && (!shadow.admitted || !shadow.candidate_phase_valid)))
    {
        output.receipt.reason = RootPrefireThreatConsumerReason::
            PrefireThreatConsumerContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }

    bool break_active = break_active_;
    bool official_seen = official_seen_;
    bool check_extend_hold = check_extend_hold_;
    bool entry_armed = entry_armed_;
    const bool candidate =
        shadow.admitted && shadow.prefire_break_candidate;
    bool entered = false;
    bool cleared = false;
    RootPrefireThreatConsumerReason reason =
        RootPrefireThreatConsumerReason::NoPrefireBreakDemand;

    ExtendCueObservation extend{};
    if (check_extend_hold
        || (break_active && official_seen && !official_threat))
    {
        if (!ObserveExtendCue(frame, extend, status))
        {
            output.receipt.reason = RootPrefireThreatConsumerReason::
                PrefireThreatConsumerContractRejected;
            return;
        }
    }

    if (check_extend_hold)
    {
        if (official_threat)
        {
            check_extend_hold = false;
        }
        else if (extend.available && !extend.candidate)
        {
            check_extend_hold = false;
        }
    }

    if (break_active && official_threat)
    {
        official_seen = true;
    }

    if (break_active && !official_threat)
    {
        const bool check_extend_release = official_seen
            && extend.available
            && extend.candidate;
        const bool margin_clear = !candidate;
        if (check_extend_release || margin_clear)
        {
            break_active = false;
            official_seen = false;
            check_extend_hold = check_extend_release;
            cleared = true;
            reason = check_extend_release
                ? RootPrefireThreatConsumerReason::CheckExtendRelease
                : RootPrefireThreatConsumerReason::PrefireMarginCleared;
        }
    }

    if (!break_active
        && !check_extend_hold
        && !official_threat
        && !candidate)
    {
        entry_armed = true;
    }

    const bool demand = candidate
        && !official_threat
        && entry_armed;
    if (!break_active && !check_extend_hold && demand)
    {
        bool entry_admitted = false;
        RootPrefireControlAdmissionReason control_reason =
            RootPrefireControlAdmissionReason::
                PrefireControlFeedbackUnavailable;
        ControlPathAdmission(
            control_evidence,
            entry_admitted,
            control_reason);
        RootPrefireThreatConsumerReason entry_failure_reason =
            ConsumerReasonFromControl(control_reason);
        if (entry_admitted)
        {
            output.entry_side_observation_attempted = true;
            ObserveRootGunTowardSideShadow(
                frame,
                envelope,
                nullptr,
                true,
                output.entry_side_observation,
                status);
            if (status.code != StatusCode::Ok)
            {
                output.receipt.reason = RootPrefireThreatConsumerReason::
                    PrefireThreatConsumerContractRejected;
                return;
            }
            if (output.entry_side_observation.toward_side_sign_valid
                && output.entry_side_observation.toward_side_sign != -1
                && output.entry_side_observation.toward_side_sign != 1)
            {
                output.receipt.reason = RootPrefireThreatConsumerReason::
                    PrefireThreatConsumerContractRejected;
                status.code = StatusCode::InvalidConfiguration;
                return;
            }
            // Threat admission and escape-side selection are independent.
            // Exact rear-centerline geometry legitimately has no unique
            // toward-side sign; it must still start defensive ownership in
            // this frame.  The downstream Root Gun/writer-14 builders already
            // own deterministic direction fallback when this optional sign
            // is absent.
        }
        if (entry_admitted)
        {
            break_active = true;
            official_seen = false;
            entry_armed = false;
            entered = true;
            cleared = false;
            reason = RootPrefireThreatConsumerReason::
                PrefireBreakEnteredControlPathAdmitted;
        }
        else
        {
            reason = entry_failure_reason;
        }
    }

    if (break_active && !entered)
    {
        reason = official_threat
            ? RootPrefireThreatConsumerReason::OfficialThreatActive
            : RootPrefireThreatConsumerReason::PrefireBreakContinued;
    }
    else if (check_extend_hold && !cleared)
    {
        reason = extend.available
            ? RootPrefireThreatConsumerReason::CheckExtendHold
            : RootPrefireThreatConsumerReason::
                CheckExtendObservationRejected;
    }
    else if (candidate
        && !official_threat
        && !entry_armed
        && !entered
        && !cleared)
    {
        reason = RootPrefireThreatConsumerReason::
            PrefireCandidateNotRearmed;
    }

    break_active_ = break_active;
    official_seen_ = official_seen;
    check_extend_hold_ = check_extend_hold;
    entry_armed_ = entry_armed;

    output.receipt.active = break_active;
    output.receipt.entered = entered;
    output.receipt.cleared = cleared;
    output.receipt.official_threat = official_threat;
    output.receipt.prefire_candidate = candidate;
    output.receipt.official_threat_seen = official_seen;
    output.receipt.check_extend_hold = check_extend_hold;
    output.receipt.reason = reason;
}

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
