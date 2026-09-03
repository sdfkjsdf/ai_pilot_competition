#include "LadyLuck/control/direct_ned/ForceVectorTracking.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{
constexpr double MinimumDtS = 0.5 / 60.0;
constexpr double MaximumDtS = 1.5 / 60.0;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(
    const LadyLuck::Vector3& first,
    const LadyLuck::Vector3& second) noexcept
{
    return first[0] * second[0]
        + first[1] * second[1]
        + first[2] * second[2];
}

double Norm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(Dot(value, value));
}

LadyLuck::Vector3 Add(
    const LadyLuck::Vector3& first,
    const LadyLuck::Vector3& second) noexcept
{
    return LadyLuck::Vector3{{
        first[0] + second[0],
        first[1] + second[1],
        first[2] + second[2]}};
}

LadyLuck::Vector3 Subtract(
    const LadyLuck::Vector3& first,
    const LadyLuck::Vector3& second) noexcept
{
    return LadyLuck::Vector3{{
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2]}};
}

LadyLuck::Vector3 Scale(
    const LadyLuck::Vector3& value,
    const double scale) noexcept
{
    return LadyLuck::Vector3{{
        value[0] * scale,
        value[1] * scale,
        value[2] * scale}};
}

LadyLuck::Vector3 Cross(
    const LadyLuck::Vector3& first,
    const LadyLuck::Vector3& second) noexcept
{
    return LadyLuck::Vector3{{
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0]}};
}

double Wrap(const double angle_rad) noexcept
{
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

void SetFailure(
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    status.code = code;
}

void SetOptional(
    LadyLuck::control::direct_ned::ForceTrackingOptionalScalar& output,
    const double value) noexcept
{
    output.has_value = true;
    output.value = value;
}

LadyLuck::control::direct_ned::ForceVectorTrackingGate MapDirectionGate(
    const LadyLuck::control::direct_ned::ForceDirectionGate gate) noexcept
{
    using DirectionGate =
        LadyLuck::control::direct_ned::ForceDirectionGate;
    using TrackingGate =
        LadyLuck::control::direct_ned::ForceVectorTrackingGate;
    switch (gate)
    {
    case DirectionGate::Init:
        return TrackingGate::DirectionInit;
    case DirectionGate::FrameGap:
        return TrackingGate::DirectionFrameGap;
    case DirectionGate::ExcludedForceInvalid:
        return TrackingGate::DirectionExcludedForceInvalid;
    case DirectionGate::VelocityDirectionUndefined:
        return TrackingGate::DirectionVelocityDirectionUndefined;
    case DirectionGate::ObservationInvalid:
        return TrackingGate::DirectionObservationInvalid;
    case DirectionGate::Update:
    default:
        return TrackingGate::DirectionObservationInvalid;
    }
}
}

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{

void CausalForceDirection::Reset() noexcept
{
    has_previous_velocity_ = false;
    previous_velocity_ned_mps_ = Vector3{};
}

void CausalForceDirection::CopySnapshot(
    CausalForceDirectionSnapshot& output) const noexcept
{
    output = CausalForceDirectionSnapshot{};
    output.has_previous_velocity = has_previous_velocity_;
    output.previous_velocity_ned_mps = previous_velocity_ned_mps_;
}

void CausalForceDirection::Update(
    const ForceDirectionInput& input,
    ForceDirectionOutput& output,
    Status& status) noexcept
{
    output = ForceDirectionOutput{};
    status = Status{};
    output.sample_dt_s = std::isfinite(input.dt_s) ? input.dt_s : 0.0;

    if (!FiniteVector(input.velocity_ned_mps))
    {
        Reset();
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    if (!std::isfinite(input.dt_s))
    {
        Reset();
        SetFailure(status, StatusCode::InvalidDt);
        return;
    }
    if (!has_previous_velocity_)
    {
        has_previous_velocity_ = true;
        previous_velocity_ned_mps_ = input.velocity_ned_mps;
        output.gate = ForceDirectionGate::Init;
        return;
    }

    const Vector3 previous_velocity = previous_velocity_ned_mps_;
    previous_velocity_ned_mps_ = input.velocity_ned_mps;
    if (input.dt_s < MinimumDtS || input.dt_s > MaximumDtS)
    {
        output.gate = ForceDirectionGate::FrameGap;
        return;
    }
    if (!input.excluded_force_valid)
    {
        output.gate = ForceDirectionGate::ExcludedForceInvalid;
        return;
    }

    const double speed_mps = Norm(input.velocity_ned_mps);
    if (speed_mps <= constants::Tiny)
    {
        output.gate = ForceDirectionGate::VelocityDirectionUndefined;
        return;
    }

    const Vector3 acceleration_ned_mps2 = Scale(
        Subtract(input.velocity_ned_mps, previous_velocity),
        1.0 / input.dt_s);
    const Vector3 total_specific_force_ned_mps2{{
        acceleration_ned_mps2[0],
        acceleration_ned_mps2[1],
        acceleration_ned_mps2[2] - constants::StandardGravityMps2}};
    const Vector3 velocity_direction_ned = Scale(
        input.velocity_ned_mps,
        1.0 / speed_mps);
    if (!FiniteVector(total_specific_force_ned_mps2)
        || (input.excluded_force_valid
            && !FiniteVector(input.excluded_specific_force_ned_mps2))
        || !FiniteVector(velocity_direction_ned)
        || !FiniteVector(input.positive_lateral_direction_ned)
        || !FiniteVector(input.positive_support_direction_ned)
        || !std::isfinite(input.target_force_bank_rad)
        || !std::isfinite(input.observed_kinematic_bank_rad))
    {
        Reset();
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }

    // These axes are optional force-observation evidence, not command
    // authority. Reproject and normalize finite current-frame axes locally so
    // harmless roundoff cannot revoke the already prepared base acceleration.
    const Vector3 lateral_projected = Subtract(
        input.positive_lateral_direction_ned,
        Scale(
            velocity_direction_ned,
            Dot(
                input.positive_lateral_direction_ned,
                velocity_direction_ned)));
    const double lateral_norm = Norm(lateral_projected);
    if (!std::isfinite(lateral_norm) || lateral_norm <= constants::Tiny)
    {
        output.gate = ForceDirectionGate::ObservationInvalid;
        return;
    }
    const Vector3 positive_lateral_direction_ned = Scale(
        lateral_projected,
        1.0 / lateral_norm);
    Vector3 support_projected = Subtract(
        input.positive_support_direction_ned,
        Scale(
            velocity_direction_ned,
            Dot(
                input.positive_support_direction_ned,
                velocity_direction_ned)));
    support_projected = Subtract(
        support_projected,
        Scale(
            positive_lateral_direction_ned,
            Dot(support_projected, positive_lateral_direction_ned)));
    const double support_norm = Norm(support_projected);
    if (!std::isfinite(support_norm) || support_norm <= constants::Tiny)
    {
        output.gate = ForceDirectionGate::ObservationInvalid;
        return;
    }
    const Vector3 positive_support_direction_ned = Scale(
        support_projected,
        1.0 / support_norm);

    VelocityNormalForceDirectionObservation observation{};
    observation.total_specific_force_ned_g = Scale(
        total_specific_force_ned_mps2,
        1.0 / constants::StandardGravityMps2);
    observation.excluded_specific_force_ned_g = Scale(
        input.excluded_specific_force_ned_mps2,
        1.0 / constants::StandardGravityMps2);
    observation.observed_specific_force_ned_g = Subtract(
        observation.total_specific_force_ned_g,
        observation.excluded_specific_force_ned_g);
    observation.velocity_parallel_g = Dot(
        observation.observed_specific_force_ned_g,
        velocity_direction_ned);
    observation.velocity_normal_specific_force_ned_g = Subtract(
        observation.observed_specific_force_ned_g,
        Scale(velocity_direction_ned, observation.velocity_parallel_g));
    observation.magnitude_g = Norm(
        observation.velocity_normal_specific_force_ned_g);
    if (!FiniteVector(observation.total_specific_force_ned_g)
        || !FiniteVector(observation.excluded_specific_force_ned_g)
        || !FiniteVector(observation.observed_specific_force_ned_g)
        || !FiniteVector(observation.velocity_normal_specific_force_ned_g)
        || !std::isfinite(observation.velocity_parallel_g)
        || !std::isfinite(observation.magnitude_g))
    {
        Reset();
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    if (observation.magnitude_g <= constants::Tiny)
    {
        output.gate = ForceDirectionGate::ObservationInvalid;
        return;
    }

    observation.direction_ned = Scale(
        observation.velocity_normal_specific_force_ned_g,
        1.0 / observation.magnitude_g);
    observation.lateral_g = Dot(
        observation.velocity_normal_specific_force_ned_g,
        positive_lateral_direction_ned);
    observation.support_g = Dot(
        observation.velocity_normal_specific_force_ned_g,
        positive_support_direction_ned);
    observation.bank_rad = std::atan2(
        observation.lateral_g,
        observation.support_g);
    if (!FiniteVector(observation.direction_ned)
        || !std::isfinite(observation.lateral_g)
        || !std::isfinite(observation.support_g)
        || !std::isfinite(observation.bank_rad))
    {
        Reset();
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }

    ForceDirectionBiasCompensation compensation{};
    compensation.target_force_bank_rad = Wrap(input.target_force_bank_rad);
    compensation.observed_kinematic_bank_rad = Wrap(
        input.observed_kinematic_bank_rad);
    compensation.observed_force_bank_rad = Wrap(observation.bank_rad);
    compensation.force_to_kinematic_bias_rad = Wrap(
        compensation.observed_force_bank_rad
        - compensation.observed_kinematic_bank_rad);
    compensation.force_direction_error_rad = Wrap(
        compensation.target_force_bank_rad
        - compensation.observed_force_bank_rad);
    compensation.compensated_kinematic_reference_rad = Wrap(
        compensation.target_force_bank_rad
        - compensation.force_to_kinematic_bias_rad);
    const double compensation_values[] = {
        compensation.target_force_bank_rad,
        compensation.observed_kinematic_bank_rad,
        compensation.observed_force_bank_rad,
        compensation.force_to_kinematic_bias_rad,
        compensation.force_direction_error_rad,
        compensation.compensated_kinematic_reference_rad};
    for (const double value : compensation_values)
    {
        if (!std::isfinite(value))
        {
            Reset();
            SetFailure(status, StatusCode::NonFiniteInput);
            return;
        }
    }

    output.valid = true;
    output.gate = ForceDirectionGate::Update;
    output.observation_valid = true;
    output.observation = observation;
    output.compensation_valid = true;
    output.compensation = compensation;
}

void ForceVectorTracking::Reset() noexcept
{
    direction_.Reset();
    has_previous_target_force_magnitude_ = false;
    previous_target_force_magnitude_g_ = 0.0;
    has_previous_force_magnitude_error_ = false;
    previous_force_magnitude_error_g_ = 0.0;
}

void ForceVectorTracking::CopySnapshot(
    ForceVectorTrackingSnapshot& output) const noexcept
{
    output = ForceVectorTrackingSnapshot{};
    direction_.CopySnapshot(output.direction);
    output.has_previous_target_force_magnitude =
        has_previous_target_force_magnitude_;
    output.previous_target_force_magnitude_g =
        previous_target_force_magnitude_g_;
    output.has_previous_force_magnitude_error =
        has_previous_force_magnitude_error_;
    output.previous_force_magnitude_error_g =
        previous_force_magnitude_error_g_;
}

void ForceVectorTracking::Step(
    const ForceVectorTrackingInput& input,
    ForceVectorTrackingOutput& output,
    Status& status) noexcept
{
    output = ForceVectorTrackingOutput{};
    status = Status{};

    // Python validates this first and does not reset existing causal history on
    // this particular caller-contract failure.
    if (!FiniteVector(input.requested_acceleration_ned_mps2))
    {
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    output.valid = true;
    output.requested_acceleration_ned_mps2 =
        input.requested_acceleration_ned_mps2;
    output.applied_acceleration_ned_mps2 =
        input.requested_acceleration_ned_mps2;

    if (!input.magnitude_tracking_enabled)
    {
        has_previous_force_magnitude_error_ = false;
        previous_force_magnitude_error_g_ = 0.0;
    }
    if (!FiniteVector(input.velocity_ned_mps)
        || !std::isfinite(input.observed_kinematic_bank_rad))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    if (!std::isfinite(input.dt_s))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::InvalidDt);
        return;
    }

    const double speed_mps = Norm(input.velocity_ned_mps);
    if (speed_mps <= constants::Tiny)
    {
        Reset();
        output.gate = ForceVectorTrackingGate::VelocityDirectionUndefined;
        return;
    }
    const Vector3 velocity_hat = Scale(
        input.velocity_ned_mps,
        1.0 / speed_mps);
    const Vector3 down_hat{{0.0, 0.0, 1.0}};
    Vector3 lateral_hat = Cross(down_hat, velocity_hat);
    const double lateral_norm = Norm(lateral_hat);
    if (lateral_norm <= constants::Tiny)
    {
        Reset();
        output.gate = ForceVectorTrackingGate::LateralAxisUndefined;
        return;
    }
    lateral_hat = Scale(lateral_hat, 1.0 / lateral_norm);
    Vector3 support_hat = Scale(Cross(velocity_hat, lateral_hat), -1.0);
    const double support_norm = Norm(support_hat);
    if (support_norm <= constants::Tiny)
    {
        Reset();
        output.gate = ForceVectorTrackingGate::SupportAxisUndefined;
        return;
    }
    support_hat = Scale(support_hat, 1.0 / support_norm);
    if (!FiniteVector(velocity_hat)
        || !FiniteVector(lateral_hat)
        || !FiniteVector(support_hat))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }

    const Vector3 gravity_ned_mps2{{
        0.0, 0.0, constants::StandardGravityMps2}};
    const Vector3 requested_specific_force = Subtract(
        input.requested_acceleration_ned_mps2,
        gravity_ned_mps2);
    const double parallel_force_mps2 = Dot(
        requested_specific_force,
        velocity_hat);
    const Vector3 normal_force_mps2 = Subtract(
        requested_specific_force,
        Scale(velocity_hat, parallel_force_mps2));
    const double target_magnitude_g = Norm(normal_force_mps2)
        / constants::StandardGravityMps2;
    if (!FiniteVector(requested_specific_force)
        || !std::isfinite(parallel_force_mps2)
        || !FiniteVector(normal_force_mps2)
        || !std::isfinite(target_magnitude_g))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    if (target_magnitude_g <= constants::Tiny)
    {
        Reset();
        output.gate = ForceVectorTrackingGate::TargetForceDirectionUndefined;
        return;
    }

    const double target_lateral_g = Dot(normal_force_mps2, lateral_hat)
        / constants::StandardGravityMps2;
    const double target_support_g = Dot(normal_force_mps2, support_hat)
        / constants::StandardGravityMps2;
    const double target_bank_rad = std::atan2(
        target_lateral_g,
        target_support_g);
    if (!std::isfinite(target_lateral_g)
        || !std::isfinite(target_support_g)
        || !std::isfinite(target_bank_rad))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }
    if (target_support_g <= 0.0
        || std::fabs(target_lateral_g) <= constants::Tiny)
    {
        Reset();
        output.gate =
            ForceVectorTrackingGate::TargetOutsidePositiveLiftTurnDomain;
        SetOptional(output.target_force_bank_rad, target_bank_rad);
        SetOptional(output.target_force_magnitude_g, target_magnitude_g);
        return;
    }

    const bool had_previous_target =
        has_previous_target_force_magnitude_;
    const double previous_target_magnitude_g =
        previous_target_force_magnitude_g_;
    has_previous_target_force_magnitude_ = true;
    previous_target_force_magnitude_g_ = target_magnitude_g;

    if (input.excluded_force_valid
        && !FiniteVector(input.excluded_specific_force_ned_mps2))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }

    ForceDirectionInput direction_input{};
    direction_input.velocity_ned_mps = input.velocity_ned_mps;
    direction_input.dt_s = input.dt_s;
    direction_input.excluded_specific_force_ned_mps2 =
        input.excluded_specific_force_ned_mps2;
    direction_input.positive_lateral_direction_ned = lateral_hat;
    direction_input.positive_support_direction_ned = support_hat;
    direction_input.target_force_bank_rad = target_bank_rad;
    direction_input.observed_kinematic_bank_rad =
        input.observed_kinematic_bank_rad;
    direction_input.excluded_force_valid = input.excluded_force_valid;
    ForceDirectionOutput direction_output{};
    Status direction_status{};
    direction_.Update(direction_input, direction_output, direction_status);
    if (!direction_status.ok())
    {
        Reset();
        output.valid = false;
        status = direction_status;
        return;
    }

    SetOptional(output.target_force_bank_rad, target_bank_rad);
    SetOptional(output.target_force_magnitude_g, target_magnitude_g);
    if (had_previous_target)
    {
        SetOptional(
            output.previous_target_force_magnitude_g,
            previous_target_magnitude_g);
    }
    output.observation_source = direction_output.source;
    if (!direction_output.valid
        || !direction_output.observation_valid
        || !direction_output.compensation_valid)
    {
        has_previous_force_magnitude_error_ = false;
        previous_force_magnitude_error_g_ = 0.0;
        output.gate = MapDirectionGate(direction_output.gate);
        output.magnitude_tracking_gate = input.magnitude_tracking_enabled
            ? ForceMagnitudeTrackingGate::DirectionObservationUnavailable
            : ForceMagnitudeTrackingGate::NotRequested;
        return;
    }

    const double observed_bank_rad =
        direction_output.observation.bank_rad;
    const double observed_magnitude_g =
        direction_output.observation.magnitude_g;
    const double corrected_bank_rad = direction_output.compensation
        .compensated_kinematic_reference_rad;
    const double direction_error_rad = direction_output.compensation
        .force_direction_error_rad;
    SetOptional(output.observed_force_bank_rad, observed_bank_rad);
    SetOptional(output.observed_force_magnitude_g, observed_magnitude_g);
    SetOptional(output.corrected_kinematic_bank_rad, corrected_bank_rad);
    SetOptional(output.force_direction_error_rad, direction_error_rad);
    if (std::cos(observed_bank_rad) <= 0.0
        || observed_bank_rad * target_bank_rad <= 0.0)
    {
        has_previous_force_magnitude_error_ = false;
        previous_force_magnitude_error_g_ = 0.0;
        output.gate =
            ForceVectorTrackingGate::ObservedForceOutsideTargetTurnDomain;
        output.magnitude_tracking_gate = input.magnitude_tracking_enabled
            ? ForceMagnitudeTrackingGate::DirectionDomainRejected
            : ForceMagnitudeTrackingGate::NotRequested;
        return;
    }
    if (std::cos(corrected_bank_rad) <= 0.0
        || corrected_bank_rad * target_bank_rad <= 0.0)
    {
        has_previous_force_magnitude_error_ = false;
        previous_force_magnitude_error_g_ = 0.0;
        output.gate = ForceVectorTrackingGate::
            CompensatedReferenceOutsideTargetTurnDomain;
        output.magnitude_tracking_gate = input.magnitude_tracking_enabled
            ? ForceMagnitudeTrackingGate::DirectionDomainRejected
            : ForceMagnitudeTrackingGate::NotRequested;
        return;
    }

    double corrected_magnitude_g = target_magnitude_g;
    double force_magnitude_error_g = 0.0;
    bool force_magnitude_error_valid = false;
    if (input.magnitude_tracking_enabled)
    {
        if (!had_previous_target)
        {
            output.magnitude_tracking_gate =
                ForceMagnitudeTrackingGate::PreviousTargetUnavailable;
        }
        else
        {
            force_magnitude_error_g = previous_target_magnitude_g
                - observed_magnitude_g;
            if (!std::isfinite(force_magnitude_error_g))
            {
                Reset();
                output.valid = false;
                SetFailure(status, StatusCode::NonFiniteInput);
                return;
            }
            force_magnitude_error_valid = true;
            const bool had_previous_error =
                has_previous_force_magnitude_error_;
            const double previous_error_g =
                previous_force_magnitude_error_g_;
            has_previous_force_magnitude_error_ = true;
            previous_force_magnitude_error_g_ = force_magnitude_error_g;
            if (force_magnitude_error_g <= 0.0)
            {
                output.magnitude_tracking_gate =
                    ForceMagnitudeTrackingGate::NonpositiveDeficit;
            }
            else if (!had_previous_error)
            {
                output.magnitude_tracking_gate =
                    ForceMagnitudeTrackingGate::PreviousErrorUnavailable;
            }
            else if (force_magnitude_error_g < previous_error_g)
            {
                output.magnitude_tracking_gate =
                    ForceMagnitudeTrackingGate::DeficitClosing;
            }
            else
            {
                const double candidate_magnitude_g = target_magnitude_g
                    + force_magnitude_error_g;
                if (!std::isfinite(candidate_magnitude_g))
                {
                    Reset();
                    output.valid = false;
                    SetFailure(status, StatusCode::NonFiniteInput);
                    return;
                }
                if (candidate_magnitude_g > constants::Tiny)
                {
                    corrected_magnitude_g = candidate_magnitude_g;
                    output.magnitude_tracking_applied = true;
                    output.magnitude_tracking_gate =
                        ForceMagnitudeTrackingGate::Active;
                    SetOptional(
                        output.compensated_force_magnitude_reference_g,
                        corrected_magnitude_g);
                }
                else
                {
                    output.magnitude_tracking_gate = ForceMagnitudeTrackingGate::
                        NonpositiveCompensatedMagnitude;
                }
            }
        }
    }
    if (force_magnitude_error_valid)
    {
        SetOptional(
            output.force_magnitude_error_g,
            force_magnitude_error_g);
    }

    const Vector3 corrected_normal_force_mps2 = Scale(
        Add(
            Scale(lateral_hat, std::sin(corrected_bank_rad)),
            Scale(support_hat, std::cos(corrected_bank_rad))),
        corrected_magnitude_g * constants::StandardGravityMps2);
    const Vector3 corrected_specific_force_mps2 = Add(
        Scale(velocity_hat, parallel_force_mps2),
        corrected_normal_force_mps2);
    const Vector3 applied_acceleration_ned_mps2 = Add(
        gravity_ned_mps2,
        corrected_specific_force_mps2);
    if (!FiniteVector(corrected_normal_force_mps2)
        || !FiniteVector(corrected_specific_force_mps2)
        || !FiniteVector(applied_acceleration_ned_mps2))
    {
        Reset();
        output.valid = false;
        SetFailure(status, StatusCode::NonFiniteInput);
        return;
    }

    output.applied_acceleration_ned_mps2 =
        applied_acceleration_ned_mps2;
    output.tracking_applied = true;
    output.gate = ForceVectorTrackingGate::Active;
}

} // namespace direct_ned
} // namespace control
} // namespace LadyLuck
