#include "LadyLuck/guidance/g4/HighGBarrelEvidenceProvider.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g13::G13FlatScissorsAdmissionReason;
using LadyLuck::guidance::g13::G13FlatScissorsAdmissionStatus;
using LadyLuck::guidance::g13::G13FlatScissorsObservation;
using LadyLuck::guidance::g13::G13FlatScissorsScopeGrade;
using LadyLuck::guidance::g13::G13Truth;
using LadyLuck::guidance::g4::HighGBarrelAttackForm;
using LadyLuck::guidance::g4::HighGBarrelAttackFormEvidence;
using LadyLuck::guidance::g4::HighGBarrelCornerIntervalEvidence;
using LadyLuck::guidance::g4::HighGBarrelG13AdmissionReason;
using LadyLuck::guidance::g4::HighGBarrelG13AdmissionStatus;
using LadyLuck::guidance::g4::HighGBarrelG13Evidence;
using LadyLuck::guidance::g4::HighGBarrelG13ScopeGrade;
using LadyLuck::guidance::g4::HighGBarrelLoadedResponseEvidence;
using LadyLuck::guidance::g4::HighGBarrelSafetyEvidence;
using LadyLuck::guidance::g4::HighGBarrelVerticalExcessEvidence;
using LadyLuck::guidance::g4::HighGBarrelVerticalExcessState;
using LadyLuck::guidance::prefire::GunAttackForm;
using LadyLuck::guidance::prefire::GunAttackFormObservation;

constexpr double kBodyVelocityQuantumMps =
    0.001 * LadyLuck::constants::FeetToMeters;
constexpr double kVerticalSpeedErrorMps =
    1.7320508075688772935 * kBodyVelocityQuantumMps;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

HighGBarrelAttackForm MapAttackForm(const GunAttackForm value) noexcept
{
    switch (value)
    {
    case GunAttackForm::Tracking:
        return HighGBarrelAttackForm::Tracking;
    case GunAttackForm::Snapshot:
        return HighGBarrelAttackForm::Snapshot;
    case GunAttackForm::Indeterminate:
    default:
        return HighGBarrelAttackForm::Indeterminate;
    }
}

HighGBarrelG13AdmissionStatus MapG13Status(
    const G13FlatScissorsAdmissionStatus value) noexcept
{
    switch (value)
    {
    case G13FlatScissorsAdmissionStatus::Hold:
        return HighGBarrelG13AdmissionStatus::Hold;
    case G13FlatScissorsAdmissionStatus::ReverseEvaluate:
        return HighGBarrelG13AdmissionStatus::ReverseEvaluate;
    case G13FlatScissorsAdmissionStatus::NoAuthority:
    default:
        return HighGBarrelG13AdmissionStatus::NoAuthority;
    }
}

HighGBarrelG13AdmissionReason MapG13Reason(
    const G13FlatScissorsAdmissionReason value) noexcept
{
    return value == G13FlatScissorsAdmissionReason::FpoOrderRefuted
        ? HighGBarrelG13AdmissionReason::FpoOrderRefuted
        : HighGBarrelG13AdmissionReason::Unavailable;
}

HighGBarrelG13ScopeGrade MapG13Scope(
    const G13FlatScissorsScopeGrade value) noexcept
{
    switch (value)
    {
    case G13FlatScissorsScopeGrade::Admitted:
        return HighGBarrelG13ScopeGrade::Admitted;
    case G13FlatScissorsScopeGrade::Refuted:
        return HighGBarrelG13ScopeGrade::Refuted;
    case G13FlatScissorsScopeGrade::NotObservable:
    default:
        return HighGBarrelG13ScopeGrade::NotObservable;
    }
}

void MapTruth(
    const G13Truth source,
    bool& present,
    bool& value) noexcept
{
    present = source != G13Truth::Unresolved;
    value = source == G13Truth::True;
}

void MapStrictPositiveInterval(
    const LadyLuck::guidance::g13::G13SignedInterval& source,
    bool& present,
    bool& value) noexcept
{
    present = false;
    value = false;
    if (!source.valid
        || !std::isfinite(source.lower)
        || !std::isfinite(source.upper)
        || source.lower > source.upper)
    {
        return;
    }
    if (source.lower > 0.0)
    {
        present = true;
        value = true;
    }
    else if (source.upper < 0.0)
    {
        present = true;
        value = false;
    }
}

void MapG13(
    const G13FlatScissorsObservation& source,
    const bool sample_valid,
    const bool response_engaged,
    HighGBarrelG13Evidence& output) noexcept
{
    output = HighGBarrelG13Evidence{};
    output.valid = source.valid && sample_valid;
    output.source_identity_valid = source.source_identity.valid;
    output.status = MapG13Status(source.admission_status);
    output.reason = MapG13Reason(source.admission_reason);
    output.scope_grade = MapG13Scope(source.scope_grade);
    MapTruth(
        source.bounded_source_valid,
        output.bounded_source_valid_present,
        output.bounded_source_valid);
    MapTruth(
        source.fpo_before_defender_body_39_observed,
        output.fpo_before_defender_body_39_present,
        output.fpo_before_defender_body_39_observed);
    MapStrictPositiveInterval(
        source.defender_body_39_margin_m,
        output.defender_body_39_strict_positive_present,
        output.defender_body_39_strict_positive_observed);
    MapTruth(
        source.attacker_pre_passage_observed,
        output.attacker_pre_passage_present,
        output.attacker_pre_passage_observed);
    MapTruth(
        source.attacker_original_turn_committed,
        output.attacker_original_turn_committed_present,
        output.attacker_original_turn_committed);
    MapTruth(
        source.far_los_steady_veto,
        output.far_los_steady_veto_present,
        output.far_los_steady_veto);
    output.defender_turn_sign = source.defender_turn_sign_valid
        ? source.defender_turn_sign
        : 0;
    output.g13_response_engaged = response_engaged;
}

void EvaluateVerticalExcess(
    const DogfightGeometryFrame& frame,
    const HighGBarrelCornerIntervalEvidence& interval,
    HighGBarrelVerticalExcessEvidence& output,
    Status& status) noexcept
{
    output = HighGBarrelVerticalExcessEvidence{};
    status = Status{};
    if (!FiniteVector(frame.own.velocity_ned_mps)
        || !std::isfinite(interval.upper_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double speed = Norm3(frame.own.velocity_ned_mps);
    if (!std::isfinite(speed) || speed < 0.0)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.valid = true;
    output.own_speed_mps = speed;
    output.speed_error_bound_mps = kVerticalSpeedErrorMps;
    output.evidence_admitted = interval.valid
        && interval.admitted
        && interval.upper_mps > 0.0;
    if (!output.evidence_admitted)
    {
        output.state = HighGBarrelVerticalExcessState::WithinResolution;
        return;
    }

    output.corner_speed_valid = true;
    output.corner_speed_mps = interval.upper_mps;
    output.excess_valid = true;
    output.excess_mps = speed - interval.upper_mps;
    if (output.excess_mps > kVerticalSpeedErrorMps)
    {
        output.state = HighGBarrelVerticalExcessState::AboveCornerProven;
    }
    else if (output.excess_mps < -kVerticalSpeedErrorMps)
    {
        output.state = HighGBarrelVerticalExcessState::BelowCornerProven;
    }
    else
    {
        output.state = HighGBarrelVerticalExcessState::WithinResolution;
    }
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

void HighGBarrelEvidenceProvider::Reset() noexcept
{
    attack_form_observer_.Reset();
    previous_tracking_t_sec_valid_ = false;
    previous_tracking_t_sec_ = 0.0;
    cached_ = false;
    cached_frame_identity_ = ControlFrameIdentity{};
    cached_output_ = HighGBarrelExactEvidence{};
    cached_status_code_ = StatusCode::InvalidConfiguration;
}

void HighGBarrelEvidenceProvider::Observe(
    const runtime::TacticalCommandBuildInput& input,
    const guidance::g13::G13FlatScissorsObservation& g13_observation,
    const bool g13_sample_valid,
    const bool g13_response_engaged,
    const HighGBarrelSafetyEvidence& safety,
    const HighGBarrelLoadedResponseEvidence& loaded_response,
    const HighGBarrelCornerIntervalEvidence& corner_interval,
    HighGBarrelExactEvidence& output,
    Status& status) noexcept
{
    output = HighGBarrelExactEvidence{};
    status = Status{};
    if (cached_
        && SameControlFrameIdentity(
            cached_frame_identity_,
            input.frame.frame_identity))
    {
        output = cached_output_;
        status.code = cached_status_code_;
        return;
    }

    HighGBarrelExactEvidence candidate{};
    candidate.frame_identity = input.frame.frame_identity;
    candidate.safety = safety;
    candidate.loaded_response = loaded_response;
    candidate.own_corner_interval = corner_interval;
    EvaluateVerticalExcess(
        input.frame,
        corner_interval,
        candidate.vertical_excess,
        status);
    if (!status.ok())
    {
        Reset();
        output = HighGBarrelExactEvidence{};
        return;
    }

    const Vector3 relative_position{{
        input.frame.opponent.position_ned_m[0]
            - input.frame.own.position_ned_m[0],
        input.frame.opponent.position_ned_m[1]
            - input.frame.own.position_ned_m[1],
        input.frame.opponent.position_ned_m[2]
            - input.frame.own.position_ned_m[2]}};
    if (!FiniteVector(relative_position))
    {
        Reset();
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double range_squared = Dot3(relative_position, relative_position);
    if (!std::isfinite(range_squared))
    {
        Reset();
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    if (range_squared == 0.0)
    {
        attack_form_observer_.Reset();
        previous_tracking_t_sec_valid_ = false;
        previous_tracking_t_sec_ = 0.0;
    }
    else
    {
        GunAttackFormObservation attack{};
        attack_form_observer_.Update(input.frame, attack, status);
        if (!status.ok())
        {
            Reset();
            return;
        }
        candidate.attack_form.valid = attack.valid;
        candidate.attack_form.form = MapAttackForm(attack.attack_form);
        candidate.attack_form.continuous_aim_solution =
            attack.continuous_aim_solution;
        const bool tracking_now = attack.valid
            && attack.attack_form == GunAttackForm::Tracking
            && attack.continuous_aim_solution;
        candidate.attack_form.tracking_retained_from_previous_sample =
            tracking_now
            && previous_tracking_t_sec_valid_
            && input.frame.t_sec > previous_tracking_t_sec_;
        previous_tracking_t_sec_valid_ = tracking_now;
        previous_tracking_t_sec_ = tracking_now
            ? input.frame.t_sec
            : 0.0;
    }

    MapG13(
        g13_observation,
        g13_sample_valid,
        g13_response_engaged,
        candidate.g13);
    candidate.valid = true;

    cached_ = true;
    cached_frame_identity_ = input.frame.frame_identity;
    cached_output_ = candidate;
    cached_status_code_ = StatusCode::Ok;
    output = candidate;
    status = Status{};
}

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
