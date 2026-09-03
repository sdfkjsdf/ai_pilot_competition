#include "LadyLuck/guidance/g4/DivingSpiralOwner.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlIntent;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g4::DivingSpiralActivation;
using LadyLuck::guidance::g4::DivingSpiralCompletedCommandEvidence;
using LadyLuck::guidance::g4::DivingSpiralDecision;
using LadyLuck::guidance::g4::DivingSpiralEntryUpperDiveRad;
using LadyLuck::guidance::g4::DivingSpiralMinimumDiveRad;
using LadyLuck::guidance::g4::DivingSpiralOwnerSnapshot;
using LadyLuck::guidance::g4::DivingSpiralPhase;
using LadyLuck::guidance::g4::DivingSpiralReason;
using LadyLuck::guidance::g4::DivingSpiralSelectionReceipt;
using LadyLuck::guidance::g4::DivingSpiralTaskReceipt;
using LadyLuck::guidance::g4::HighGBarrelExactEvidence;
using LadyLuck::guidance::g13::G13FlatScissorsObservation;
using LadyLuck::guidance::obfm::G3ChaseDownObservation;
using LadyLuck::runtime::TacticalCommandBuildInput;

// Exact caller-certified roll authority used by the existing Python fixture
// and by the already-production G4 High-G owner.  This is not a selector
// threshold or a maneuver-completion clock.
constexpr double kRollAuthorityRadps = 2.0;

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

Vector3 Cross3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

double ScaledTolerance(
    const double first,
    const double second = 0.0,
    const double third = 0.0,
    const double fourth = 0.0) noexcept
{
    const double scale = (std::max)(
        1.0,
        (std::max)(
            std::fabs(first),
            (std::max)(
                std::fabs(second),
                (std::max)(std::fabs(third), std::fabs(fourth)))));
    return 64.0 * (std::numeric_limits<double>::epsilon)() * scale;
}

bool Unit3(const Vector3& input, Vector3& output) noexcept
{
    output = Vector3{};
    if (!FiniteVector(input))
    {
        return false;
    }
    const double magnitude = Norm3(input);
    if (!std::isfinite(magnitude)
        || magnitude <= (std::numeric_limits<double>::epsilon)())
    {
        return false;
    }
    output = Vector3{{
        input[0] / magnitude,
        input[1] / magnitude,
        input[2] / magnitude}};
    return FiniteVector(output);
}

bool VelocityNormalDirection(
    const Vector3& direction,
    const Vector3& velocity_hat,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 velocity{};
    if (!Unit3(velocity_hat, velocity) || !FiniteVector(direction))
    {
        return false;
    }
    const double parallel = Dot3(direction, velocity);
    const Vector3 projected{{
        direction[0] - parallel * velocity[0],
        direction[1] - parallel * velocity[1],
        direction[2] - parallel * velocity[2]}};
    const double magnitude = Norm3(projected);
    if (!std::isfinite(magnitude)
        || magnitude <= ScaledTolerance(magnitude))
    {
        return false;
    }
    output = Vector3{{
        projected[0] / magnitude,
        projected[1] / magnitude,
        projected[2] / magnitude}};
    return FiniteVector(output);
}

bool ParallelTransportNormalDirection(
    const Vector3& previous_velocity,
    const Vector3& current_velocity,
    const Vector3& previous_direction,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 previous_velocity_hat{};
    Vector3 current_velocity_hat{};
    Vector3 direction{};
    if (!Unit3(previous_velocity, previous_velocity_hat)
        || !Unit3(current_velocity, current_velocity_hat)
        || !VelocityNormalDirection(
            previous_direction,
            previous_velocity_hat,
            direction))
    {
        return false;
    }

    Vector3 axis = Cross3(previous_velocity_hat, current_velocity_hat);
    const double sine_angle = Norm3(axis);
    const double cosine_angle = (std::max)(
        -1.0,
        (std::min)(1.0, Dot3(previous_velocity_hat, current_velocity_hat)));
    Vector3 transported{};
    if (sine_angle <= ScaledTolerance(sine_angle))
    {
        if (cosine_angle < 0.0)
        {
            return false;
        }
        transported = direction;
    }
    else
    {
        axis = Vector3{{
            axis[0] / sine_angle,
            axis[1] / sine_angle,
            axis[2] / sine_angle}};
        const Vector3 cross = Cross3(axis, direction);
        const double axial = Dot3(axis, direction);
        transported = Vector3{{
            cosine_angle * direction[0]
                + sine_angle * cross[0]
                + (1.0 - cosine_angle) * axial * axis[0],
            cosine_angle * direction[1]
                + sine_angle * cross[1]
                + (1.0 - cosine_angle) * axial * axis[1],
            cosine_angle * direction[2]
                + sine_angle * cross[2]
                + (1.0 - cosine_angle) * axial * axis[2]}};
    }
    return VelocityNormalDirection(
        transported,
        current_velocity_hat,
        output);
}

bool RotateAboutUnitAxis(
    const Vector3& input,
    const Vector3& axis_input,
    const double angle_rad,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 axis{};
    if (!FiniteVector(input)
        || !Unit3(axis_input, axis)
        || !std::isfinite(angle_rad))
    {
        return false;
    }
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    const Vector3 cross = Cross3(axis, input);
    const double axial = Dot3(axis, input);
    const Vector3 rotated{{
        cosine * input[0] + sine * cross[0]
            + (1.0 - cosine) * axial * axis[0],
        cosine * input[1] + sine * cross[1]
            + (1.0 - cosine) * axial * axis[1],
        cosine * input[2] + sine * cross[2]
            + (1.0 - cosine) * axial * axis[2]}};
    return Unit3(rotated, output);
}

bool CurrentPhysicalSafetyAndLoad(
    const TacticalCommandBuildInput& input,
    double& governed_load_g,
    double& load_limit_g) noexcept
{
    governed_load_g = 0.0;
    load_limit_g = 0.0;
    const double own_speed = Norm3(input.frame.own.velocity_ned_mps);
    if (!input.current_safety.valid
        || !input.current_safety.entry_available
        || input.current_safety.entry_should_activate
        || input.current_safety.entry_boundary_breached
        || !input.current_physical_envelope_available
        || !input.current_envelope.physical_authority
        || !std::isfinite(input.current_envelope.nz_feasible_g)
        || input.current_envelope.nz_feasible_g <= 1.0
        || !std::isfinite(own_speed)
        || own_speed <= LadyLuck::constants::Tiny)
    {
        return false;
    }
    governed_load_g = input.current_envelope.nz_feasible_g;
    load_limit_g = input.current_envelope.nz_feasible_g;
    return true;
}

bool CurrentThreeDimensionalGeometry(
    const TacticalCommandBuildInput& input,
    std::int32_t& turn_sign) noexcept
{
    turn_sign = 0;
    Vector3 velocity_hat{};
    Vector3 positive_load{};
    Vector3 los_hat{};
    const Vector3 body_up{{
        -input.frame.own.down_ned[0],
        -input.frame.own.down_ned[1],
        -input.frame.own.down_ned[2]}};
    const Vector3 los{{
        input.frame.opponent.position_ned_m[0]
            - input.frame.own.position_ned_m[0],
        input.frame.opponent.position_ned_m[1]
            - input.frame.own.position_ned_m[1],
        input.frame.opponent.position_ned_m[2]
            - input.frame.own.position_ned_m[2]}};
    if (!Unit3(input.frame.own.velocity_ned_mps, velocity_hat)
        || !VelocityNormalDirection(body_up, velocity_hat, positive_load)
        || !Unit3(los, los_hat))
    {
        return false;
    }
    const double signed_rotation = Dot3(
        Cross3(positive_load, los_hat), velocity_hat);
    if (!std::isfinite(signed_rotation))
    {
        return false;
    }
    turn_sign = signed_rotation >= 0.0 ? 1 : -1;
    return true;
}

void EncodeClimbRate(
    const double target_climb_rate_mps,
    LadyLuck::PlaneState& state) noexcept
{
    const double bank_rad = state.rpy_rad[0];
    const double sine_bank = std::sin(bank_rad);
    const double cosine_bank = std::cos(bank_rad);
    state.rpy_rad[1] = 0.0;
    state.velocity_body_mps = Vector3{{0.0, 0.0, 0.0}};
    if (std::fabs(cosine_bank) >= std::fabs(sine_bank))
    {
        state.velocity_body_mps[2] =
            -target_climb_rate_mps / cosine_bank;
    }
    else
    {
        state.velocity_body_mps[1] =
            -target_climb_rate_mps / sine_bank;
    }
}

bool RecoveryAllowsInvertedDeepDive(
    const TacticalCommandBuildInput& input,
    double& pull_capability_g) noexcept
{
    pull_capability_g = 0.0;
    const double speed_mps = Norm3(input.frame.own.velocity_ned_mps);
    const double altitude_m = -input.frame.own.position_ned_m[2];
    const double measured_climb_rate_mps =
        input.current_safety.climb_rate_mps;
    if (!input.current_safety.valid
        || !input.current_physical_envelope_available
        || !input.current_envelope.physical_authority
        || !std::isfinite(input.current_envelope.nz_feasible_g)
        || input.current_envelope.nz_feasible_g <= 1.0
        || !std::isfinite(speed_mps)
        || speed_mps <= 0.0
        || !std::isfinite(altitude_m)
        || !std::isfinite(measured_climb_rate_mps))
    {
        return false;
    }
    pull_capability_g = input.current_envelope.nz_feasible_g;

    const double candidate_climb_rate_mps =
        -speed_mps * std::sin(DivingSpiralEntryUpperDiveRad);
    const double target_climb_rate_mps = (std::min)(
        candidate_climb_rate_mps,
        measured_climb_rate_mps);

    LadyLuck::safety::AutoGcasEntryInput candidate =
        input.current_safety.evaluated_input;
    candidate.ownship.position_ned_m[2] = -altitude_m;
    candidate.ownship.rpy_rad[0] = LadyLuck::constants::Pi;
    candidate.ownship.speed_mps = speed_mps;
    candidate.available_nz_g = pull_capability_g;
    candidate.available_nz_valid = true;
    EncodeClimbRate(target_climb_rate_mps, candidate.ownship);

    const LadyLuck::safety::AutoGcas predictor{};
    LadyLuck::safety::AutoGcasEntryReceipt candidate_receipt{};
    Status candidate_status{};
    predictor.EvaluateEntry(
        candidate,
        candidate_receipt,
        candidate_status);
    return candidate_status.ok()
        && candidate_receipt.valid
        && candidate_receipt.entry_available
        && !candidate_receipt.entry_should_activate;
}

bool MeasuredDiveAngle(
    const TacticalCommandBuildInput& input,
    double& angle_rad) noexcept
{
    angle_rad = 0.0;
    const Vector3& velocity = input.frame.own.velocity_ned_mps;
    if (!FiniteVector(velocity))
    {
        return false;
    }
    const double horizontal = std::hypot(velocity[0], velocity[1]);
    const double speed = Norm3(velocity);
    if (!std::isfinite(horizontal)
        || !std::isfinite(speed)
        || speed <= 0.0)
    {
        return false;
    }
    angle_rad = std::atan2(velocity[2], horizontal);
    return std::isfinite(angle_rad);
}

DivingSpiralReason ClassifyCommonEntry(
    const TacticalCommandBuildInput& input,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence&,
    std::int32_t& defender_turn_sign,
    double& governed_load_g,
    double& load_limit_g) noexcept
{
    defender_turn_sign = 0;
    governed_load_g = 0.0;
    load_limit_g = 0.0;
    if (!root_gun_selected)
    {
        return DivingSpiralReason::OfficialRootGunOwnerInactive;
    }
    if (!CurrentPhysicalSafetyAndLoad(
            input, governed_load_g, load_limit_g))
    {
        return DivingSpiralReason::EntrySafetyRejected;
    }
    if (!CurrentThreeDimensionalGeometry(input, defender_turn_sign))
    {
        return DivingSpiralReason::CausalAttackFormUnavailable;
    }
    return DivingSpiralReason::Admitted;
}

DivingSpiralReason ClassifyContinuation(
    const TacticalCommandBuildInput& input,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence&,
    const G13FlatScissorsObservation&,
    const G3ChaseDownObservation*,
    const DivingSpiralCompletedCommandEvidence&,
    const DivingSpiralOwnerSnapshot& snapshot,
    double& measured_dive_angle_rad,
    double& current_load_g) noexcept
{
    measured_dive_angle_rad = 0.0;
    current_load_g = 0.0;
    if (!root_gun_selected)
    {
        return DivingSpiralReason::OfficialRootGunOwnerInactive;
    }
    double current_load_limit_g = 0.0;
    if (!CurrentPhysicalSafetyAndLoad(
            input, current_load_g, current_load_limit_g))
    {
        return DivingSpiralReason::RunningSafetyRejected;
    }
    std::int32_t current_turn_sign = 0;
    if (!CurrentThreeDimensionalGeometry(input, current_turn_sign))
    {
        return DivingSpiralReason::RunningSafetyRejected;
    }
    (void)current_turn_sign;
    (void)current_load_limit_g;
    if (!MeasuredDiveAngle(input, measured_dive_angle_rad))
    {
        return DivingSpiralReason::RunningSafetyRejected;
    }
    if (snapshot.phase != DivingSpiralPhase::Spiral
        && snapshot.phase != DivingSpiralPhase::DiveEntry)
    {
        return DivingSpiralReason::PreviousDivingSpiralCommandNotCompleted;
    }

    double pull_capability_g = 0.0;
    if (!RecoveryAllowsInvertedDeepDive(input, pull_capability_g))
    {
        return input.current_physical_envelope_available
            ? DivingSpiralReason::InvertedDeepDiveRecoveryNotAdmitted
            : DivingSpiralReason::RecoveryPullCapabilityUnavailable;
    }
    if (pull_capability_g < current_load_g)
    {
        return DivingSpiralReason::
            CurrentPullCapabilityBelowCommittedLoad;
    }
    if (snapshot.phase == DivingSpiralPhase::Spiral
        && measured_dive_angle_rad < DivingSpiralMinimumDiveRad)
    {
        return DivingSpiralReason::ManualMinimumDeepDiveNotMaintained;
    }
    return DivingSpiralReason::Admitted;
}

void ClearLateralOwners(ControlIntent& output) noexcept
{
    output.direct_p_cmd_radps = LadyLuck::IntentOptionalValue<double>{};
    output.direct_nz_cmd_g = LadyLuck::IntentOptionalValue<double>{};
    output.direct_beta_cmd_rad = LadyLuck::IntentOptionalValue<double>{};
    output.direct_acceleration_ned_mps2 =
        LadyLuck::IntentOptionalValue<Vector3>{};
    output.direct_acceleration_roll_rate_reference_radps =
        LadyLuck::IntentOptionalValue<double>{};
    output.direct_acceleration_tracking_enabled = false;
    output.direct_acceleration_tracking_observation_only = false;
    output.direct_acceleration_magnitude_tracking_enabled = false;
    output.direct_acceleration_loaded_roll_enabled = false;
    output.direct_acceleration_load_component_compensation_enabled = false;
    output.direct_acceleration_yaw_coordination_enabled = false;
    output.direct_acceleration_roll_priority_yaw_enabled = false;
    output.direct_bank_cmd_rad = LadyLuck::IntentOptionalValue<double>{};
    output.direct_turn_rate_cmd_radps = LadyLuck::IntentOptionalValue<double>{};
    output.direct_load_vector_acceleration_ned_mps2 =
        LadyLuck::IntentOptionalValue<Vector3>{};
}

bool SnapshotFinite(const DivingSpiralOwnerSnapshot& value) noexcept
{
    return std::isfinite(value.requested_roll_rate_radps)
        && std::isfinite(value.requested_load_magnitude_g)
        && std::isfinite(value.entry_load_limit_g)
        && std::isfinite(value.entry_command_dive_angle_rad)
        && FiniteVector(value.commanded_velocity_ned_mps)
        && FiniteVector(value.commanded_bank_direction_ned)
        && std::isfinite(value.entry_elapsed_s);
}

void RootPassthrough(
    const TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    const DivingSpiralReason reason,
    ControlIntent& output,
    DivingSpiralOwnerSnapshot& commit,
    DivingSpiralTaskReceipt& receipt) noexcept
{
    commit = DivingSpiralOwnerSnapshot{};
    commit.last_release_reason = reason;
    output = root_intent;
    receipt = DivingSpiralTaskReceipt{};
    receipt.valid = true;
    receipt.frame_identity = input.frame.frame_identity;
    receipt.root_passthrough_required = true;
    receipt.reason = reason;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

void DivingSpiralOwner::Reset() noexcept
{
    snapshot_ = DivingSpiralOwnerSnapshot{};
}

void DivingSpiralOwner::Observe(
    const runtime::TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence& evidence,
    const guidance::g13::G13FlatScissorsObservation& g13_observation,
    const guidance::obfm::G3ChaseDownObservation* pursuit_observation,
    const DivingSpiralCompletedCommandEvidence& completed_command,
    const DivingSpiralActivation& activation,
    DivingSpiralSelectionReceipt& output,
    Status& status) const noexcept
{
    output = DivingSpiralSelectionReceipt{};
    status = Status{};
    Status root_status{};
    root_intent.Validate(root_status);
    if (!input.valid
        || !root_status.ok()
        || !IsValidControlFrameIdentity(input.frame.frame_identity)
        || (evidence.valid
            && !SameControlFrameIdentity(
                input.frame.frame_identity,
                evidence.frame_identity))
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            root_intent.frame_identity)
        || !SnapshotFinite(snapshot_)
        || activation.enabled != activation.exact_provenance)
    {
        status.code = root_status.ok()
            ? StatusCode::InvalidConfiguration
            : root_status.code;
        return;
    }

    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    output.engaged_before = snapshot_.engaged;
    if (!activation.enabled)
    {
        output.decision = DivingSpiralDecision::RootPassthrough;
        output.reason = DivingSpiralReason::ProductionDisabled;
        return;
    }

    if (snapshot_.engaged)
    {
        double measured_dive = 0.0;
        double current_load_g = 0.0;
        const DivingSpiralReason reason = ClassifyContinuation(
            input,
            root_gun_selected,
            evidence,
            g13_observation,
            pursuit_observation,
            completed_command,
            snapshot_,
            measured_dive,
            current_load_g);
        output.reason = reason;
        output.measured_dive_angle_rad = measured_dive;
        if (reason != DivingSpiralReason::Admitted)
        {
            output.decision = DivingSpiralDecision::ReleasePassthrough;
            output.released_if_published = true;
            return;
        }
        output.defender_turn_sign = snapshot_.roll_direction_sign;
        output.requested_roll_rate_radps =
            snapshot_.requested_roll_rate_radps;
        output.requested_load_magnitude_g = current_load_g;
        output.entry_load_limit_g = current_load_g;
        output.requested_entry_dive_angle_rad =
            snapshot_.entry_command_dive_angle_rad;
        output.decision = snapshot_.phase == DivingSpiralPhase::Spiral
                || measured_dive >= DivingSpiralMinimumDiveRad
            ? DivingSpiralDecision::Spiral
            : DivingSpiralDecision::DiveEntry;
        return;
    }

    std::int32_t defender_turn_sign = 0;
    double governed_load_g = 0.0;
    double load_limit_g = 0.0;
    DivingSpiralReason reason = ClassifyCommonEntry(
        input,
        root_gun_selected,
        evidence,
        defender_turn_sign,
        governed_load_g,
        load_limit_g);
    double measured_dive = 0.0;
    if (reason == DivingSpiralReason::Admitted
        && !MeasuredDiveAngle(input, measured_dive))
    {
        reason = DivingSpiralReason::EntrySafetyRejected;
    }
    double gamma_authority = DivingSpiralEntryUpperDiveRad;
    if (evidence.safety.flight_path_gamma_limit_source_valid
        && std::isfinite(evidence.safety.flight_path_gamma_limit_rad))
    {
        gamma_authority = evidence.safety.flight_path_gamma_limit_rad;
    }
    if (reason == DivingSpiralReason::Admitted
        && gamma_authority < DivingSpiralMinimumDiveRad)
    {
        reason = DivingSpiralReason::
            ManualEntryDiveExceedsFlightPathAuthority;
    }
    double pull_capability_g = 0.0;
    if (reason == DivingSpiralReason::Admitted
        && !RecoveryAllowsInvertedDeepDive(input, pull_capability_g))
    {
        reason = input.current_physical_envelope_available
            ? DivingSpiralReason::InvertedDeepDiveRecoveryNotAdmitted
            : DivingSpiralReason::RecoveryPullCapabilityUnavailable;
    }
    output.reason = reason;
    output.measured_dive_angle_rad = measured_dive;
    if (reason != DivingSpiralReason::Admitted)
    {
        output.decision = DivingSpiralDecision::RootPassthrough;
        return;
    }

    output.entry_admitted = true;
    output.entered_if_published = true;
    output.decision = DivingSpiralDecision::DiveEntry;
    output.defender_turn_sign = defender_turn_sign;
    output.requested_roll_rate_radps = kRollAuthorityRadps;
    output.requested_load_magnitude_g = governed_load_g;
    output.entry_load_limit_g = load_limit_g;
    output.requested_entry_dive_angle_rad = (std::min)(
        DivingSpiralEntryUpperDiveRad,
        gamma_authority);
}

void DivingSpiralOwner::BuildCandidate(
    const DivingSpiralPhase selected_phase,
    const runtime::TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    const DivingSpiralSelectionReceipt& selection,
    ControlIntent& output,
    DivingSpiralOwnerSnapshot& commit,
    DivingSpiralTaskReceipt& receipt,
    Status& status) const noexcept
{
    output.Clear();
    commit = snapshot_;
    receipt = DivingSpiralTaskReceipt{};
    status = Status{};
    Status root_status{};
    root_intent.Validate(root_status);
    const DivingSpiralDecision expected =
        selected_phase == DivingSpiralPhase::DiveEntry
        ? DivingSpiralDecision::DiveEntry
        : DivingSpiralDecision::Spiral;
    if (!root_status.ok()
        || !input.valid
        || !selection.valid
        || (selected_phase != DivingSpiralPhase::DiveEntry
            && selected_phase != DivingSpiralPhase::Spiral)
        || selection.decision != expected
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            selection.frame_identity)
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            root_intent.frame_identity)
        || !std::isfinite(input.accepted_estimator.sample_dt_s)
        || input.accepted_estimator.sample_dt_s <= 0.0)
    {
        status.code = root_status.ok()
            ? StatusCode::InvalidConfiguration
            : root_status.code;
        return;
    }

    bool entered = false;
    if (!commit.engaged)
    {
        if (selected_phase != DivingSpiralPhase::DiveEntry
            || !selection.entry_admitted
            || (selection.defender_turn_sign != -1
                && selection.defender_turn_sign != 1)
            || selection.requested_roll_rate_radps != kRollAuthorityRadps
            || !std::isfinite(selection.requested_load_magnitude_g)
            || selection.requested_load_magnitude_g <= 1.0
            || !std::isfinite(selection.entry_load_limit_g)
            || selection.entry_load_limit_g
                < selection.requested_load_magnitude_g
            || !std::isfinite(selection.requested_entry_dive_angle_rad)
            || selection.requested_entry_dive_angle_rad
                < DivingSpiralMinimumDiveRad
            || selection.requested_entry_dive_angle_rad
                > DivingSpiralEntryUpperDiveRad
            || !FiniteVector(input.frame.own.velocity_ned_mps)
            || !FiniteVector(input.frame.own.down_ned))
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        commit.engaged = true;
        commit.phase = DivingSpiralPhase::DiveEntry;
        commit.roll_direction_sign = selection.defender_turn_sign;
        commit.requested_roll_rate_radps =
            selection.requested_roll_rate_radps;
        commit.requested_load_magnitude_g =
            selection.requested_load_magnitude_g;
        commit.entry_load_limit_g = selection.entry_load_limit_g;
        commit.entry_command_dive_angle_rad =
            selection.requested_entry_dive_angle_rad;
        commit.commanded_velocity_ned_mps =
            input.frame.own.velocity_ned_mps;
        commit.commanded_bank_direction_ned = Vector3{{
            -input.frame.own.down_ned[0],
            -input.frame.own.down_ned[1],
            -input.frame.own.down_ned[2]}};
        commit.entry_elapsed_s = 0.0;
        commit.last_release_reason = DivingSpiralReason::Unavailable;
        entered = true;
    }
    if (!commit.engaged
        || (commit.roll_direction_sign != -1
            && commit.roll_direction_sign != 1)
        || !SnapshotFinite(commit)
        || commit.requested_roll_rate_radps <= 0.0
        || commit.requested_load_magnitude_g <= 1.0
        || commit.entry_load_limit_g
            < commit.requested_load_magnitude_g
        || commit.entry_command_dive_angle_rad
            < DivingSpiralMinimumDiveRad
        || commit.entry_command_dive_angle_rad
            > DivingSpiralEntryUpperDiveRad)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!entered)
    {
        if (!std::isfinite(selection.requested_load_magnitude_g)
            || selection.requested_load_magnitude_g <= 1.0
            || !std::isfinite(selection.entry_load_limit_g)
            || selection.entry_load_limit_g
                < selection.requested_load_magnitude_g)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        commit.requested_load_magnitude_g =
            selection.requested_load_magnitude_g;
        commit.entry_load_limit_g = selection.entry_load_limit_g;
    }

    if (selected_phase == DivingSpiralPhase::DiveEntry)
    {
        const Vector3 root_path{{
            root_intent.aim_point_m[0]
                - input.frame.own.position_ned_m[0],
            root_intent.aim_point_m[1]
                - input.frame.own.position_ned_m[1],
            root_intent.aim_point_m[2]
                - input.frame.own.position_ned_m[2]}};
        Vector3 horizontal{{root_path[0], root_path[1], 0.0}};
        double horizontal_range = Norm3(horizontal);
        if (std::isfinite(horizontal_range)
            && horizontal_range <= constants::Tiny)
        {
            const double path_range = Norm3(root_path);
            const Vector3 candidates[] = {
                input.frame.own.nose_ned,
                input.frame.own.right_ned,
                input.frame.own.velocity_ned_mps};
            for (const Vector3& candidate : candidates)
            {
                const double candidate_horizontal = std::hypot(
                    candidate[0], candidate[1]);
                if (FiniteVector(candidate)
                    && std::isfinite(candidate_horizontal)
                    && candidate_horizontal > constants::Tiny
                    && std::isfinite(path_range)
                    && path_range > constants::Tiny)
                {
                    horizontal = Vector3{{
                        path_range * candidate[0] / candidate_horizontal,
                        path_range * candidate[1] / candidate_horizontal,
                        0.0}};
                    horizontal_range = path_range;
                    break;
                }
            }
        }
        Vector3 velocity_hat{};
        Vector3 desired_path_hat{};
        Vector3 desired_bank{};
        Vector3 dive_path{{
            horizontal[0],
            horizontal[1],
            horizontal_range
                * std::tan(commit.entry_command_dive_angle_rad)}};
        const bool reference_available = FiniteVector(root_path)
            && std::isfinite(horizontal_range)
            && horizontal_range > 0.0
            && FiniteVector(dive_path)
            && Unit3(input.frame.own.velocity_ned_mps, velocity_hat)
            && Unit3(dive_path, desired_path_hat)
            && VelocityNormalDirection(
                desired_path_hat,
                velocity_hat,
                desired_bank);
        if (!reference_available)
        {
            RootPassthrough(
                input,
                root_intent,
                DivingSpiralReason::DeepDiveEntryCommandUnavailable,
                output,
                commit,
                receipt);
            return;
        }
        const double force_magnitude =
            commit.requested_load_magnitude_g
            * constants::StandardGravityMps2;
        const Vector3 acceleration{{
            force_magnitude * desired_bank[0],
            force_magnitude * desired_bank[1],
            constants::StandardGravityMps2
                + force_magnitude * desired_bank[2]}};
        if (!FiniteVector(acceleration))
        {
            RootPassthrough(
                input,
                root_intent,
                DivingSpiralReason::DeepDiveEntryCommandUnavailable,
                output,
                commit,
                receipt);
            return;
        }

        output = root_intent;
        ClearLateralOwners(output);
        output.aim_point_m = Vector3{{
            input.frame.own.position_ned_m[0] + dive_path[0],
            input.frame.own.position_ned_m[1] + dive_path[1],
            input.frame.own.position_ned_m[2] + dive_path[2]}};
        output.aim_point_velocity_mps = IntentOptionalValue<Vector3>{};
        output.total_load_factor_limit_g.has_value = true;
        output.total_load_factor_limit_g.value =
            commit.requested_load_magnitude_g;
        output.path_inversion_allowed.has_value = true;
        output.path_inversion_allowed.value = true;
        output.direct_acceleration_ned_mps2.has_value = true;
        output.direct_acceleration_ned_mps2.value = acceleration;
        output.route_kind = ControlRouteKind::DirectNedAcceleration;
        output.behavior_id =
            DoctrineBehaviorId::G4DivingSpiralDiveEntry;
        output.writer_id = ControlIntentWriterG4DivingSpiral;
        output.Validate(status);
        if (!status.ok())
        {
            output.Clear();
            commit = snapshot_;
            return;
        }
        commit.entry_elapsed_s += input.accepted_estimator.sample_dt_s;
        receipt.valid = true;
        receipt.frame_identity = input.frame.frame_identity;
        receipt.candidate_available = true;
        receipt.entered_this_tick = entered;
        receipt.phase = DivingSpiralPhase::DiveEntry;
        receipt.reason = DivingSpiralReason::Admitted;
        receipt.aim_point_ned_m = output.aim_point_m;
        receipt.desired_bank_direction_ned = desired_bank;
        receipt.acceleration_ned_mps2 = acceleration;
        receipt.load_magnitude_g = commit.requested_load_magnitude_g;
        receipt.load_limit_g = commit.requested_load_magnitude_g;
        receipt.measured_dive_angle_rad =
            selection.measured_dive_angle_rad;
        receipt.entry_command_dive_angle_rad =
            commit.entry_command_dive_angle_rad;
        receipt.entry_elapsed_s = commit.entry_elapsed_s;
        return;
    }

    if (commit.phase != DivingSpiralPhase::DiveEntry
        && commit.phase != DivingSpiralPhase::Spiral)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    Vector3 transported{};
    Vector3 velocity_hat{};
    Vector3 desired_bank{};
    const double roll_increment = commit.requested_roll_rate_radps
        * input.accepted_estimator.sample_dt_s;
    const bool reference_available = std::isfinite(roll_increment)
        && roll_increment > 0.0
        && roll_increment < constants::Pi
        && Unit3(input.frame.own.velocity_ned_mps, velocity_hat)
        && ParallelTransportNormalDirection(
            commit.commanded_velocity_ned_mps,
            input.frame.own.velocity_ned_mps,
            commit.commanded_bank_direction_ned,
            transported)
        && RotateAboutUnitAxis(
            transported,
            velocity_hat,
            static_cast<double>(commit.roll_direction_sign)
                * roll_increment,
            desired_bank);
    if (!reference_available)
    {
        RootPassthrough(
            input,
            root_intent,
            DivingSpiralReason::SpiralLoadedRollCommandUnavailable,
            output,
            commit,
            receipt);
        return;
    }
    const double signed_roll_rate =
        static_cast<double>(commit.roll_direction_sign)
        * commit.requested_roll_rate_radps;
    const double force_magnitude = commit.requested_load_magnitude_g
        * constants::StandardGravityMps2;
    const Vector3 acceleration{{
        force_magnitude * desired_bank[0],
        force_magnitude * desired_bank[1],
        constants::StandardGravityMps2
            + force_magnitude * desired_bank[2]}};
    const Vector3 specific_force{{
        acceleration[0],
        acceleration[1],
        acceleration[2] - constants::StandardGravityMps2}};
    const double parallel_force = Dot3(specific_force, velocity_hat);
    if (!FiniteVector(acceleration)
        || !std::isfinite(parallel_force)
        || std::fabs(parallel_force) > ScaledTolerance(
            parallel_force,
            Norm3(specific_force)))
    {
        RootPassthrough(
            input,
            root_intent,
            DivingSpiralReason::SpiralLoadedRollCommandUnavailable,
            output,
            commit,
            receipt);
        return;
    }

    output = root_intent;
    ClearLateralOwners(output);
    output.total_load_factor_limit_g.has_value = true;
    output.total_load_factor_limit_g.value = commit.entry_load_limit_g;
    output.path_inversion_allowed.has_value = true;
    output.path_inversion_allowed.value = true;
    output.direct_acceleration_ned_mps2.has_value = true;
    output.direct_acceleration_ned_mps2.value = acceleration;
    output.direct_acceleration_roll_rate_reference_radps.has_value = true;
    output.direct_acceleration_roll_rate_reference_radps.value =
        signed_roll_rate;
    output.route_kind = ControlRouteKind::DirectNedAcceleration;
    output.behavior_id = DoctrineBehaviorId::G4DivingSpiralLoadedRoll;
    output.writer_id = ControlIntentWriterG4DivingSpiral;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
        commit = snapshot_;
        return;
    }

    commit.phase = DivingSpiralPhase::Spiral;
    commit.commanded_velocity_ned_mps =
        input.frame.own.velocity_ned_mps;
    commit.commanded_bank_direction_ned = desired_bank;
    receipt.valid = true;
    receipt.frame_identity = input.frame.frame_identity;
    receipt.candidate_available = true;
    receipt.entered_this_tick =
        snapshot_.phase != DivingSpiralPhase::Spiral;
    receipt.phase = DivingSpiralPhase::Spiral;
    receipt.reason = DivingSpiralReason::Admitted;
    receipt.aim_point_ned_m = output.aim_point_m;
    receipt.desired_bank_direction_ned = desired_bank;
    receipt.acceleration_ned_mps2 = acceleration;
    receipt.roll_rate_reference_valid = true;
    receipt.signed_roll_rate_reference_radps = signed_roll_rate;
    receipt.load_magnitude_g = commit.requested_load_magnitude_g;
    receipt.load_limit_g = commit.entry_load_limit_g;
    receipt.measured_dive_angle_rad =
        selection.measured_dive_angle_rad;
    receipt.entry_command_dive_angle_rad =
        commit.entry_command_dive_angle_rad;
    receipt.entry_elapsed_s = commit.entry_elapsed_s;
}

void DivingSpiralOwner::BuildReleaseCommit(
    const DivingSpiralSelectionReceipt& selection,
    DivingSpiralOwnerSnapshot& commit,
    Status& status) const noexcept
{
    commit = snapshot_;
    status = Status{};
    if (!selection.valid
        || selection.decision
            != DivingSpiralDecision::ReleasePassthrough
        || !snapshot_.engaged)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    commit = DivingSpiralOwnerSnapshot{};
    commit.last_release_reason = selection.reason;
}

void DivingSpiralOwner::CommitPublished(
    const DivingSpiralOwnerSnapshot& commit,
    Status& status) noexcept
{
    status = Status{};
    if (!SnapshotFinite(commit)
        || commit.entry_elapsed_s < 0.0
        || (commit.engaged
            && ((commit.phase != DivingSpiralPhase::DiveEntry
                    && commit.phase != DivingSpiralPhase::Spiral)
                || (commit.roll_direction_sign != -1
                    && commit.roll_direction_sign != 1)
                || commit.requested_roll_rate_radps <= 0.0
                || commit.requested_load_magnitude_g <= 1.0
                || commit.entry_load_limit_g
                    < commit.requested_load_magnitude_g
                || commit.entry_command_dive_angle_rad
                    < DivingSpiralMinimumDiveRad
                || commit.entry_command_dive_angle_rad
                    > DivingSpiralEntryUpperDiveRad)))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_ = commit;
}

void DivingSpiralOwner::CopySnapshot(
    DivingSpiralOwnerSnapshot& output) const noexcept
{
    output = snapshot_;
}

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
