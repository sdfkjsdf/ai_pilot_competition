#include "LadyLuck/guidance/obfm/ObfmEntryLongitudinalReference.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{
namespace
{

bool Finite(const double value) noexcept
{
    return std::isfinite(value) != 0;
}

bool FiniteVector(const Vector3& value) noexcept
{
    return Finite(value[0]) && Finite(value[1]) && Finite(value[2]);
}

bool CheckedAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    return CheckedAdd(left, -right, output);
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double left_absolute = std::fabs(left);
    const double right_absolute = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((left_absolute > 1.0
            && right_absolute >= maximum / left_absolute)
        || (right_absolute > 1.0
            && left_absolute >= maximum / right_absolute))
    {
        return false;
    }
    output = left * right;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedDivide(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(numerator) || !Finite(denominator) || denominator == 0.0)
    {
        return false;
    }
    const double numerator_absolute = std::fabs(numerator);
    const double denominator_absolute = std::fabs(denominator);
    if (numerator_absolute != 0.0
        && denominator_absolute < 1.0
        && numerator_absolute
            >= (std::numeric_limits<double>::max)()
                * denominator_absolute)
    {
        return false;
    }
    output = numerator / denominator;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedVectorSubtract(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedSubtract(left[0], right[0], output[0])
        && CheckedSubtract(left[1], right[1], output[1])
        && CheckedSubtract(left[2], right[2], output[2]);
}

bool CheckedVectorAdd(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedAdd(left[0], right[0], output[0])
        && CheckedAdd(left[1], right[1], output[1])
        && CheckedAdd(left[2], right[2], output[2]);
}

bool CheckedScale(
    const Vector3& value,
    const double scale,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedMultiply(value[0], scale, output[0])
        && CheckedMultiply(value[1], scale, output[1])
        && CheckedMultiply(value[2], scale, output[2]);
}

bool CheckedDot(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    output = 0.0;
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double first_sum = 0.0;
    return CheckedMultiply(left[0], right[0], term0)
        && CheckedMultiply(left[1], right[1], term1)
        && CheckedMultiply(left[2], right[2], term2)
        && CheckedAdd(term0, term1, first_sum)
        && CheckedAdd(first_sum, term2, output);
}

bool VectorNorm(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    output = std::hypot(std::hypot(value[0], value[1]), value[2]);
    return Finite(output);
}

bool HorizontalNorm(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    output = std::hypot(value[0], value[1]);
    return Finite(output);
}

bool ConsecutiveReferenceEpisode(
    const ControlFrameIdentity& current,
    const double current_time_s,
    const ObfmEntryLongitudinalSnapshot& snapshot) noexcept
{
    if (!IsValidControlFrameIdentity(current)
        || !Finite(current_time_s)
        || !snapshot.previous_frame_identity_valid
        || !IsValidControlFrameIdentity(snapshot.previous_frame_identity)
        || !snapshot.previous_time_valid
        || !Finite(snapshot.previous_time_s)
        || !snapshot.previous_reference_point_valid
        || !FiniteVector(snapshot.previous_reference_point_ned_m)
        || !snapshot.previous_own_position_valid
        || !FiniteVector(snapshot.previous_own_position_ned_m)
        || !snapshot.previous_own_velocity_valid
        || !FiniteVector(snapshot.previous_own_velocity_ned_mps)
        || !snapshot.previous_speed_command_valid
        || !Finite(snapshot.previous_speed_command_mps)
        || snapshot.previous_speed_command_mps <= 0.0
        || current.episode_epoch
            != snapshot.previous_frame_identity.episode_epoch
        || current.frame_index == 0U
        || snapshot.previous_frame_identity.frame_index
            != current.frame_index - 1U
        || current_time_s <= snapshot.previous_time_s)
    {
        return false;
    }
    return true;
}

void SelectEchoMode(
    const bool previous_reference_admitted,
    const ObfmEntryLongitudinalReason reason,
    ObfmEntryLongitudinalReceipt& output) noexcept
{
    // Every call site is downstream of ConsecutiveReferenceEpisode().  The
    // command candidate has therefore already been replaced by the last
    // actually-published episode speed.  Retain the historical boolean only
    // as receipt lineage; it must not decide whether a reject re-echoes the
    // current measured (possibly bled) speed.
    static_cast<void>(previous_reference_admitted);
    output.reason = reason;
    output.application_mode = ObfmEntryLongitudinalApplicationMode::
        ActiveEpisodeLatchedSpeedHold;
}

void SetCommandCandidate(
    const ObfmEntryWindowObservationReceipt& observation,
    const double desired_speed_mps,
    const double desired_speed_rate_mps2,
    const double official_max_range_m,
    ObfmEntryLongitudinalReceipt& output) noexcept
{
    output.command.valid = true;
    output.command.aim_point_ned_m =
        observation.geometry.entry_point_ned_m;
    output.command.aim_point_velocity_ned_mps =
        observation.entry_point_velocity_ned_mps;
    output.command.desired_speed_mps = desired_speed_mps;
    output.command.desired_speed_rate_mps2 = desired_speed_rate_mps2;
    output.command.specific_energy_rate_bias_m2ps3 = 0.0;
    output.command.path_inversion_allowed = false;
    output.command.capture_range_des_m = official_max_range_m;
    output.producer_ready = true;
    output.producer_count = 1U;
}

bool BuildPointSpeed(
    const ObfmEntryWindowObservationReceipt& observation,
    const ObfmEntryLongitudinalSnapshot& snapshot,
    const double dt_s,
    const double official_max_range_m,
    ObfmEntryPointSpeedReceipt& output,
    ObfmEntryLongitudinalReason& reason) noexcept
{
    output = ObfmEntryPointSpeedReceipt{};
    output.evaluated = true;
    reason = ObfmEntryLongitudinalReason::PointSpeedGeometryUnavailable;

    const ObfmEntryWindowGeometry& geometry = observation.geometry;
    double own_speed = 0.0;
    double target_speed = 0.0;
    if (!VectorNorm(geometry.own_velocity_ned_mps, own_speed)
        || own_speed <= constants::Tiny)
    {
        reason = ObfmEntryLongitudinalReason::CurrentSpeedUnavailable;
        return false;
    }
    output.current_speed_mps = own_speed;
    if (!VectorNorm(
            geometry.circle.target_velocity_ned_mps,
            target_speed)
        || target_speed <= constants::Tiny)
    {
        reason = ObfmEntryLongitudinalReason::TargetSpeedNotPositive;
        return false;
    }
    output.target_speed_mps = target_speed;
    if (!Finite(geometry.circle.speed_mps)
        || target_speed != geometry.circle.speed_mps)
    {
        reason = ObfmEntryLongitudinalReason::
            TargetCircleSpeedLineageMismatch;
        return false;
    }
    if (!Finite(dt_s) || dt_s <= 0.0
        || !Finite(official_max_range_m)
        || official_max_range_m <= 0.0)
    {
        reason = ObfmEntryLongitudinalReason::
            GuidanceTimeLineageMismatch;
        return false;
    }

    ObfmEntryWindowReason transport_reason =
        ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
    if (!BuildObfmEntryTargetOnlyTransportedPoint(
            geometry.circle,
            snapshot.previous_own_position_ned_m,
            snapshot.previous_own_velocity_ned_mps,
            output.transported_reference_point_ned_m,
            transport_reason))
    {
        reason = ObfmEntryLongitudinalReason::TransportedPointUnavailable;
        return false;
    }

    Vector3 reference_delta{};
    Vector3 reference_velocity{};
    Vector3 capture_error{};
    Vector3 structural_capture{};
    Vector3 required_velocity{};
    Vector3 path_direction{};
    Vector3 along_velocity{};
    Vector3 perpendicular{};
    double inverse_dt = 0.0;
    double structural_rate = 0.0;
    double inverse_own_speed = 0.0;
    double raw_speed = 0.0;
    double perpendicular_speed = 0.0;
    if (!CheckedVectorSubtract(
            output.transported_reference_point_ned_m,
            snapshot.previous_reference_point_ned_m,
            reference_delta)
        || !CheckedDivide(1.0, dt_s, inverse_dt)
        || !CheckedScale(reference_delta, inverse_dt, reference_velocity)
        || !CheckedVectorSubtract(
            geometry.entry_point_ned_m,
            geometry.own_position_ned_m,
            capture_error)
        || !CheckedDivide(target_speed, official_max_range_m, structural_rate)
        || structural_rate <= 0.0
        || !CheckedScale(capture_error, structural_rate, structural_capture)
        || !CheckedVectorAdd(
            reference_velocity,
            structural_capture,
            required_velocity)
        || !CheckedDivide(1.0, own_speed, inverse_own_speed)
        || !CheckedScale(
            geometry.own_velocity_ned_mps,
            inverse_own_speed,
            path_direction)
        || !CheckedDot(path_direction, required_velocity, raw_speed)
        || !CheckedScale(path_direction, raw_speed, along_velocity)
        || !CheckedVectorSubtract(
            required_velocity,
            along_velocity,
            perpendicular)
        || !VectorNorm(perpendicular, perpendicular_speed))
    {
        reason = ObfmEntryLongitudinalReason::
            PointSpeedGeometryUnavailable;
        return false;
    }

    output.raw_speed_mps = raw_speed;
    output.structural_rate_per_s = structural_rate;
    output.reference_velocity_ned_mps = reference_velocity;
    output.capture_error_ned_m = capture_error;
    output.required_velocity_ned_mps = required_velocity;
    output.perpendicular_velocity_ned_mps = perpendicular;
    output.perpendicular_speed_mps = perpendicular_speed;
    if (raw_speed <= 0.0)
    {
        reason = ObfmEntryLongitudinalReason::
            ReferenceNotAcquirableAlongCurrentPath;
        return false;
    }
    output.admitted = true;
    return true;
}

ObfmEntryLongitudinalReason AdmissionReason(
    const ObfmLongitudinalAdmissionStatus status) noexcept
{
    switch (status)
    {
    case ObfmLongitudinalAdmissionStatus::ReferenceAvailable:
        return ObfmEntryLongitudinalReason::LongitudinalReferenceAdmitted;
    case ObfmLongitudinalAdmissionStatus::C1SourceNotAdmitted:
    case ObfmLongitudinalAdmissionStatus::
        TerminalSpeedOutsideQualifiedInterval:
        return ObfmEntryLongitudinalReason::CanonicalNzfeasC1Unavailable;
    case ObfmLongitudinalAdmissionStatus::TransientLoadBridgeNotAdmitted:
        return ObfmEntryLongitudinalReason::TransientLoadBridgeUnavailable;
    case ObfmLongitudinalAdmissionStatus::CausalCommandNotAdmitted:
    case ObfmLongitudinalAdmissionStatus::InputNonfinite:
        return ObfmEntryLongitudinalReason::CausalCommandUnavailable;
    }
    return ObfmEntryLongitudinalReason::CausalCommandUnavailable;
}

} // namespace

const char* ObfmEntryLongitudinalReasonLabel(
    const ObfmEntryLongitudinalReason reason) noexcept
{
    switch (reason)
    {
    case ObfmEntryLongitudinalReason::Reset:
        return "reset";
    case ObfmEntryLongitudinalReason::NonOwnerSkip:
        return "non_owner_skip";
    case ObfmEntryLongitudinalReason::FeatureDisabled:
        return "feature_disabled";
    case ObfmEntryLongitudinalReason::EntryObservationUnavailable:
        return "entry_observation_unavailable";
    case ObfmEntryLongitudinalReason::FrameIdentityUnavailable:
        return "frame_identity_unavailable";
    case ObfmEntryLongitudinalReason::CurrentSpeedUnavailable:
        return "current_speed_unavailable";
    case ObfmEntryLongitudinalReason::OfficialMaximumRangeUnavailable:
        return "official_maximum_range_unavailable";
    case ObfmEntryLongitudinalReason::
        ReferenceEpisodePrimeOrDiscontinuous:
        return "reference_episode_prime_or_discontinuous";
    case ObfmEntryLongitudinalReason::TransportedPointUnavailable:
        return "transported_point_unavailable";
    case ObfmEntryLongitudinalReason::TargetCircleSpeedLineageMismatch:
        return "target_circle_speed_lineage_mismatch";
    case ObfmEntryLongitudinalReason::TargetSpeedNotPositive:
        return "target_speed_not_positive";
    case ObfmEntryLongitudinalReason::PointSpeedGeometryUnavailable:
        return "point_speed_geometry_unavailable";
    case ObfmEntryLongitudinalReason::
        ReferenceNotAcquirableAlongCurrentPath:
        return "reference_not_acquirable_along_current_path";
    case ObfmEntryLongitudinalReason::GuidanceTimeLineageMismatch:
        return "guidance_time_lineage_mismatch";
    case ObfmEntryLongitudinalReason::CurrentEnvelopeInvalid:
        return "current_envelope_invalid";
    case ObfmEntryLongitudinalReason::CurrentStateTimeMismatch:
        return "current_state_time_mismatch";
    case ObfmEntryLongitudinalReason::ControlFeedbackMissing:
        return "control_feedback_missing";
    case ObfmEntryLongitudinalReason::ControlFeedbackStale:
        return "control_feedback_stale";
    case ObfmEntryLongitudinalReason::PreviousFeedbackTimeMismatch:
        return "previous_feedback_time_mismatch";
    case ObfmEntryLongitudinalReason::PreviousBackendNotCisV4:
        return "previous_backend_not_cis_v4";
    case ObfmEntryLongitudinalReason::PreviousCisIntegrityNotClean:
        return "previous_cis_integrity_not_clean";
    case ObfmEntryLongitudinalReason::
        PreviousAutoGcasInterventionOrFault:
        return "previous_auto_gcas_intervention_or_fault";
    case ObfmEntryLongitudinalReason::PreviousEnergyMeasurementInvalid:
        return "previous_energy_measurement_invalid";
    case ObfmEntryLongitudinalReason::PreviousEnergyAuthorityInvalid:
        return "previous_energy_authority_invalid";
    case ObfmEntryLongitudinalReason::PreviousGovernedLoadUnavailable:
        return "previous_governed_load_unavailable";
    case ObfmEntryLongitudinalReason::PreviousMeasuredLoadUnavailable:
        return "previous_measured_load_unavailable";
    case ObfmEntryLongitudinalReason::BackendSpeedRateBoundsUnavailable:
        return "backend_speed_rate_bounds_unavailable";
    case ObfmEntryLongitudinalReason::ReferenceHorizontalRangeDegenerate:
        return "reference_horizontal_range_degenerate";
    case ObfmEntryLongitudinalReason::FlightPathGammaLimitUnavailable:
        return "flight_path_gamma_limit_unavailable";
    case ObfmEntryLongitudinalReason::CanonicalNzfeasC1Unavailable:
        return "canonical_nzfeas_c1_unavailable";
    case ObfmEntryLongitudinalReason::TransientLoadBridgeUnavailable:
        return "transient_load_bridge_unavailable";
    case ObfmEntryLongitudinalReason::CausalCommandUnavailable:
        return "causal_command_unavailable";
    case ObfmEntryLongitudinalReason::LongitudinalReferenceAdmitted:
        return "longitudinal_reference_admitted";
    case ObfmEntryLongitudinalReason::CommitUnavailable:
        return "commit_unavailable";
    }
    return "unknown";
}

ObfmEntryLongitudinalReference::
    ObfmEntryLongitudinalReference() noexcept
{
    ResetEpisode();
}

void ObfmEntryLongitudinalReference::AdvanceLifecycle() noexcept
{
    if (lifecycle_generation_
        == (std::numeric_limits<std::uint64_t>::max)())
    {
        lifecycle_generation_ = 1U;
    }
    else
    {
        ++lifecycle_generation_;
    }
}

void ObfmEntryLongitudinalReference::ClearHistory() noexcept
{
    snapshot_ = ObfmEntryLongitudinalSnapshot{};
    commit_count_ = 0U;
    AdvanceLifecycle();
}

void ObfmEntryLongitudinalReference::ResetEpisode() noexcept
{
    owner_active_ = false;
    ClearHistory();
}

void ObfmEntryLongitudinalReference::EnterOwner(
    const ObfmEntrySetupServiceReceipt& service,
    Status& status) noexcept
{
    status = Status{};
    ClearHistory();
    owner_active_ = service.service_evaluated
        && service.enabled_result
        && service.selected_result;
    if (!owner_active_)
    {
        status.code = StatusCode::ObservationInvalid;
    }
}

void ObfmEntryLongitudinalReference::HaltOwner() noexcept
{
    owner_active_ = false;
    ClearHistory();
}

void ObfmEntryLongitudinalReference::Prepare(
    const bool feature_enabled,
    const ObfmEntrySetupServiceReceipt& service,
    const ObfmEntryWindowObservationReceipt& observation,
    const runtime::TacticalCommandBuildInput& tactical_input,
    ObfmEntryLongitudinalPreparation& preparation,
    ObfmEntryLongitudinalReceipt& output,
    Status& status) noexcept
{
    preparation = ObfmEntryLongitudinalPreparation{};
    output = ObfmEntryLongitudinalReceipt{};
    output.evaluated = true;
    output.frame_identity = observation.frame_identity;
    status = Status{};

    if (!owner_active_ || !service.selected_result)
    {
        output.non_owner_skip = true;
        output.reason = ObfmEntryLongitudinalReason::NonOwnerSkip;
        return;
    }
    if (!feature_enabled)
    {
        output.base_fallback_required = true;
        output.reason = ObfmEntryLongitudinalReason::FeatureDisabled;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    if (!observation.admitted
        || !observation.geometry_available
        || !observation.entry_point_velocity_available
        || !FiniteVector(observation.geometry.entry_point_ned_m)
        || !FiniteVector(observation.entry_point_velocity_ned_mps))
    {
        output.base_fallback_required = true;
        output.reason =
            ObfmEntryLongitudinalReason::EntryObservationUnavailable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    if (!tactical_input.valid
        || !IsValidControlFrameIdentity(observation.frame_identity)
        || !SameControlFrameIdentity(
            service.frame_identity,
            observation.frame_identity)
        || !SameControlFrameIdentity(
            observation.frame_identity,
            tactical_input.frame.frame_identity)
        || !Finite(tactical_input.frame.t_sec))
    {
        output.base_fallback_required = true;
        output.reason = ObfmEntryLongitudinalReason::
            FrameIdentityUnavailable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    double current_speed = 0.0;
    if (!VectorNorm(
            observation.geometry.own_velocity_ned_mps,
            current_speed)
        || current_speed <= constants::Tiny)
    {
        output.base_fallback_required = true;
        output.reason = ObfmEntryLongitudinalReason::
            CurrentSpeedUnavailable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    const double official_max_range_m =
        tactical_input.frame.own_offense.phase.max_range_m;
    if (!Finite(official_max_range_m) || official_max_range_m <= 0.0)
    {
        output.base_fallback_required = true;
        output.reason = ObfmEntryLongitudinalReason::
            OfficialMaximumRangeUnavailable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    preparation.valid = true;
    preparation.frame_identity = observation.frame_identity;
    preparation.lifecycle_generation = lifecycle_generation_;
    preparation.base_commit_count = commit_count_;
    preparation.current_reference_point_ned_m =
        observation.geometry.entry_point_ned_m;
    preparation.current_own_position_ned_m =
        observation.geometry.own_position_ned_m;
    preparation.current_own_velocity_ned_mps =
        observation.geometry.own_velocity_ned_mps;
    preparation.current_time_s = tactical_input.frame.t_sec;
    SetCommandCandidate(
        observation,
        current_speed,
        0.0,
        official_max_range_m,
        output);

    const bool same_episode = ConsecutiveReferenceEpisode(
        preparation.frame_identity,
        preparation.current_time_s,
        snapshot_);
    preparation.same_reference_episode = same_episode;
    if (!same_episode)
    {
        output.application_mode = ObfmEntryLongitudinalApplicationMode::
            PrimeCurrentSpeedEcho;
        output.reason = ObfmEntryLongitudinalReason::
            ReferenceEpisodePrimeOrDiscontinuous;
        return;
    }

    // REQ-OBFM-10: once an Entry command has actually published, an active
    // non-admitted sample holds that transactional command exactly.  Only a
    // newly admitted reference below may move speed; the first owner sample
    // returned above remains the existing measured-speed fail-close.
    output.command.desired_speed_mps =
        snapshot_.previous_speed_command_mps;
    output.command.desired_speed_rate_mps2 = 0.0;

    double dt_s = 0.0;
    if (!CheckedSubtract(
            preparation.current_time_s,
            snapshot_.previous_time_s,
            dt_s)
        || dt_s <= 0.0)
    {
        SelectEchoMode(
            snapshot_.previous_reference_admitted,
            ObfmEntryLongitudinalReason::GuidanceTimeLineageMismatch,
            output);
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    ObfmEntryLongitudinalReason point_reason =
        ObfmEntryLongitudinalReason::PointSpeedGeometryUnavailable;
    if (!BuildPointSpeed(
            observation,
            snapshot_,
            dt_s,
            official_max_range_m,
            output.point,
            point_reason))
    {
        SelectEchoMode(
            snapshot_.previous_reference_admitted,
            point_reason,
            output);
        return;
    }

    // The current moving entry-point geometry owns v_cmd.  Age-1 energy and
    // load receipts remain tracking telemetry; absence or ambiguity in those
    // optional observations must not suppress the entry-leg speed reference.
    preparation.reference_admitted = true;
    output.admission.admitted = true;
    output.admission.status =
        ObfmLongitudinalAdmissionStatus::ReferenceAvailable;
    output.admission.desired_speed_mps = output.point.raw_speed_mps;
    output.admission.desired_speed_rate_mps2 = 0.0;
    SetCommandCandidate(
        observation,
        output.point.raw_speed_mps,
        0.0,
        official_max_range_m,
        output);
    output.application_mode =
        ObfmEntryLongitudinalApplicationMode::AdmittedReference;
    output.reason =
        ObfmEntryLongitudinalReason::LongitudinalReferenceAdmitted;
}

void ObfmEntryLongitudinalReference::CommitPublished(
    const ObfmEntryLongitudinalPreparation& preparation,
    const bool entry_command_published,
    const double published_desired_speed_mps,
    Status& status) noexcept
{
    status = Status{};
    if (!entry_command_published)
    {
        return;
    }
    if (!owner_active_
        || !preparation.valid
        || !IsValidControlFrameIdentity(preparation.frame_identity)
        || preparation.lifecycle_generation != lifecycle_generation_
        || preparation.base_commit_count != commit_count_
        || !FiniteVector(preparation.current_reference_point_ned_m)
        || !FiniteVector(preparation.current_own_position_ned_m)
        || !FiniteVector(preparation.current_own_velocity_ned_mps)
        || !Finite(preparation.current_time_s)
        || !Finite(published_desired_speed_mps)
        || published_desired_speed_mps <= 0.0)
    {
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    snapshot_.previous_reference_point_valid = true;
    snapshot_.previous_reference_point_ned_m =
        preparation.current_reference_point_ned_m;
    snapshot_.previous_own_position_valid = true;
    snapshot_.previous_own_position_ned_m =
        preparation.current_own_position_ned_m;
    snapshot_.previous_own_velocity_valid = true;
    snapshot_.previous_own_velocity_ned_mps =
        preparation.current_own_velocity_ned_mps;
    snapshot_.previous_speed_command_valid = true;
    snapshot_.previous_speed_command_mps = published_desired_speed_mps;
    snapshot_.previous_frame_identity_valid = true;
    snapshot_.previous_frame_identity = preparation.frame_identity;
    snapshot_.previous_time_valid = true;
    snapshot_.previous_time_s = preparation.current_time_s;
    snapshot_.previous_reference_admitted =
        preparation.same_reference_episode
        && preparation.reference_admitted;

    if (commit_count_
        == (std::numeric_limits<std::uint64_t>::max)())
    {
        commit_count_ = 0U;
        AdvanceLifecycle();
    }
    else
    {
        ++commit_count_;
    }
}

void ObfmEntryLongitudinalReference::CopySnapshot(
    ObfmEntryLongitudinalSnapshot& output) const noexcept
{
    output = snapshot_;
}

bool ObfmEntryLongitudinalReference::owner_active() const noexcept
{
    return owner_active_;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
