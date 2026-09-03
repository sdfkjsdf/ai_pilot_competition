#include "LadyLuck/guidance/habfm/HabfmH09AltitudeStorage.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

bool SafeMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if (left == 0.0 || right == 0.0)
    {
        return true;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    const double absolute_right = std::fabs(right);
    if (absolute_right > 1.0
        && std::fabs(left) > maximum / absolute_right)
    {
        return false;
    }
    output = left * right;
    return std::isfinite(output);
}

bool SafeAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
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
    return std::isfinite(output);
}

bool SafeScaleVector(
    const LadyLuck::Vector3& value,
    const double scale,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    return SafeMultiply(value[0], scale, output[0])
        && SafeMultiply(value[1], scale, output[1])
        && SafeMultiply(value[2], scale, output[2]);
}

bool SafeAddVectors(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    return SafeAdd(left[0], right[0], output[0])
        && SafeAdd(left[1], right[1], output[1])
        && SafeAdd(left[2], right[2], output[2]);
}

bool SafeSubtractVectors(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right,
    LadyLuck::Vector3& output) noexcept
{
    const LadyLuck::Vector3 negative{{
        -right[0], -right[1], -right[2]}};
    return SafeAddVectors(left, negative, output);
}

bool SafeDot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right,
    double& output) noexcept
{
    double p0 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double first = 0.0;
    return SafeMultiply(left[0], right[0], p0)
        && SafeMultiply(left[1], right[1], p1)
        && SafeMultiply(left[2], right[2], p2)
        && SafeAdd(p0, p1, first)
        && SafeAdd(first, p2, output);
}

void Reject(
    LadyLuck::guidance::habfm::HabfmH09AltitudeStorageAdmission& output,
    const LadyLuck::guidance::habfm::HabfmH09StorageReason reason) noexcept
{
    output.admitted = false;
    output.reason = reason;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace habfm
{

void EvaluateHabfmH09AltitudeStorage(
    const runtime::TacticalCommandBuildInput& tactical,
    const HabfmFrameEvidence& evidence,
    const HabfmActiveControlOutput& active,
    HabfmH09AltitudeStorageAdmission& output,
    Status& status) noexcept
{
    output = HabfmH09AltitudeStorageAdmission{};
    status = Status{};
    output.frame_identity = tactical.frame.frame_identity;
    output.evaluated = true;

    if (active.branch != HabfmActiveBranch::TwoCircle)
    {
        Reject(output, HabfmH09StorageReason::NotTwoCircle);
        return;
    }
    const SustainedTurnOperatingPoint& point =
        evidence.own_sustained_turn_point;
    if (!point.admitted()
        || !std::isfinite(point.speed_mps.value)
        || !std::isfinite(point.turn_rate_radps.value)
        || point.speed_mps.value <= 0.0
        || point.turn_rate_radps.value <= 0.0)
    {
        Reject(
            output,
            HabfmH09StorageReason::SustainedOperatingPointUnavailable);
        return;
    }
    output.sustained_speed_mps = point.speed_mps.value;
    output.sustained_turn_rate_radps = point.turn_rate_radps.value;

    if (!FiniteVector(tactical.frame.own.velocity_ned_mps))
    {
        Reject(output, HabfmH09StorageReason::OwnSpeedUnavailable);
        return;
    }
    output.own_speed_mps = VectorNorm(tactical.frame.own.velocity_ned_mps);
    if (!std::isfinite(output.own_speed_mps)
        || output.own_speed_mps <= 0.0)
    {
        Reject(output, HabfmH09StorageReason::OwnSpeedUnavailable);
        return;
    }
    if (output.own_speed_mps < output.sustained_speed_mps)
    {
        Reject(
            output,
            HabfmH09StorageReason::SpeedBelowSustainedReference);
        return;
    }

    // Auto-GCAS and gun defense are higher-priority tree owners.  Rechecking
    // their evidence here made a normal TWO_CIRCLE energy modifier another
    // safety selector.  H09 owns only the physical question left at this
    // point: is speed above the sustained operating point and can the current
    // flight-path channel represent a positive storage request?
    const auto& longitudinal = tactical.current_longitudinal_evidence;
    const double gamma_limit = longitudinal.flight_path_gamma_limit_rad;
    if (!longitudinal.valid
        || !longitudinal.tecs_configuration.valid
        || !longitudinal.flight_path_gamma_limit_valid
        || !std::isfinite(gamma_limit)
        || gamma_limit <= 0.0
        || gamma_limit >= 0.5 * constants::Pi)
    {
        Reject(
            output,
            HabfmH09StorageReason::TecsEnergyAuthorityUnavailable);
        return;
    }
    double vertical_speed_limit_mps = 0.0;
    if (!SafeMultiply(
            output.own_speed_mps,
            std::sin(gamma_limit),
            vertical_speed_limit_mps)
        || !SafeMultiply(
            constants::StandardGravityMps2,
            vertical_speed_limit_mps,
            output.admitted_energy_rate_m2ps3))
    {
        Reject(
            output,
            HabfmH09StorageReason::PositiveTecsReferenceNotAuthorized);
        return;
    }
    output.authority_upper_reference_m2ps3 =
        output.admitted_energy_rate_m2ps3;
    if (!std::isfinite(output.admitted_energy_rate_m2ps3)
        || output.admitted_energy_rate_m2ps3 <= 0.0)
    {
        Reject(
            output,
            HabfmH09StorageReason::AdmittedStorageRateNotPositive);
        return;
    }
    output.speed_reference_rate_mps2 = 0.0;
    output.climb_rate_mps = output.admitted_energy_rate_m2ps3
        / constants::StandardGravityMps2;
    if (!std::isfinite(output.climb_rate_mps)
        || output.climb_rate_mps <= 0.0
        || output.climb_rate_mps >= output.own_speed_mps)
    {
        Reject(
            output,
            HabfmH09StorageReason::ClimbRateNotKinematicallyRepresentable);
        return;
    }
    output.climb_gamma_cmd_rad = std::asin(
        output.climb_rate_mps / output.own_speed_mps);
    if (!std::isfinite(output.climb_gamma_cmd_rad))
    {
        Reject(
            output,
            HabfmH09StorageReason::ClimbPathAngleOutsideCurrentLimit);
        return;
    }
    // The energy-rate proposal was constructed from this exact gamma limit.
    // asin(sin(limit)) may nevertheless reconstruct one representable value
    // above `limit`; project that round-off back onto the originating physical
    // boundary instead of turning an admissible boundary command into a
    // tactical non-admission.
    output.climb_gamma_cmd_rad = (std::min)(
        output.climb_gamma_cmd_rad, gamma_limit);

    output.admitted = true;
    output.reason = HabfmH09StorageReason::Admitted;
}

void AllocateHabfmH09ResidualClimb(
    const HabfmH09ResidualClimbInput& input,
    HabfmH09ResidualClimbAllocation& output,
    Status& status) noexcept
{
    output = HabfmH09ResidualClimbAllocation{};
    status = Status{};
    output.frame_identity = input.frame_identity;
    output.evaluated = true;
    output.instantaneous_load_limit_g = input.instantaneous_load_limit_g;

    const double speed = VectorNorm(input.velocity_ned_mps);
    const double forward_norm = VectorNorm(input.body_forward_ned);
    const double finite_values[] = {
        speed,
        forward_norm,
        input.sustained_speed_mps,
        input.sustained_course_rate_radps,
        input.speed_reference_rate_mps2,
        input.residual_energy_rate_m2ps3,
        input.tecs_thrust_command_n,
        input.mass_kg,
        input.instantaneous_load_limit_g,
        input.flight_path_rate_command_radps};
    for (const double value : finite_values)
    {
        if (!std::isfinite(value))
        {
            output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
            return;
        }
    }
    if (!IsValidControlFrameIdentity(input.frame_identity)
        || !FiniteVector(input.velocity_ned_mps)
        || !FiniteVector(input.body_forward_ned)
        || (input.side_sign != -1 && input.side_sign != 1)
        || speed <= 0.0
        || forward_norm <= 0.0
        || input.sustained_speed_mps <= 0.0
        || input.sustained_course_rate_radps <= 0.0
        || input.residual_energy_rate_m2ps3 <= 0.0
        || input.mass_kg <= 0.0
        || input.instantaneous_load_limit_g <= 0.0)
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }

    double kinetic_rate = 0.0;
    if (!SafeMultiply(
            input.sustained_speed_mps,
            input.speed_reference_rate_mps2,
            kinetic_rate))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    const double potential_rate =
        input.residual_energy_rate_m2ps3 - kinetic_rate;
    if (!std::isfinite(potential_rate) || potential_rate <= 0.0)
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    output.climb_rate_mps = potential_rate
        / constants::StandardGravityMps2;
    if (!std::isfinite(output.climb_rate_mps)
        || output.climb_rate_mps <= 0.0
        || output.climb_rate_mps >= speed)
    {
        output.reason =
            HabfmH09StorageReason::ClimbRateNotKinematicallyRepresentable;
        return;
    }
    output.climb_gamma_cmd_rad = std::asin(output.climb_rate_mps / speed);

    const double horizontal_speed = std::hypot(
        input.velocity_ned_mps[0], input.velocity_ned_mps[1]);
    const double gamma = std::atan2(
        -input.velocity_ned_mps[2], horizontal_speed);
    const double chi = std::atan2(
        input.velocity_ned_mps[1], input.velocity_ned_mps[0]);
    const double cosine_gamma = std::cos(gamma);
    const double sine_gamma = std::sin(gamma);
    const double cosine_chi = std::cos(chi);
    const double sine_chi = std::sin(chi);
    if (!std::isfinite(output.climb_gamma_cmd_rad)
        || !std::isfinite(gamma)
        || !std::isfinite(chi)
        || !std::isfinite(cosine_gamma)
        || !std::isfinite(sine_gamma)
        || !std::isfinite(cosine_chi)
        || !std::isfinite(sine_chi))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }

    double course_scale = 0.0;
    double course_acceleration = 0.0;
    if (!SafeMultiply(
            speed,
            (std::max)(cosine_gamma, 0.1),
            course_scale)
        || !SafeMultiply(
            course_scale,
            input.sustained_course_rate_radps,
            course_acceleration))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    output.course_acceleration_mps2 = course_acceleration;

    const Vector3 velocity_hat{{
        input.velocity_ned_mps[0] / speed,
        input.velocity_ned_mps[1] / speed,
        input.velocity_ned_mps[2] / speed}};
    const Vector3 forward_hat{{
        input.body_forward_ned[0] / forward_norm,
        input.body_forward_ned[1] / forward_norm,
        input.body_forward_ned[2] / forward_norm}};
    const Vector3 wind_y_ned{{
        -sine_chi, cosine_chi, 0.0}};
    const Vector3 wind_z_ned{{
        cosine_chi * sine_gamma,
        sine_chi * sine_gamma,
        cosine_gamma}};

    Vector3 course_component{};
    Vector3 path_component{};
    const double signed_course_acceleration =
        static_cast<double>(input.side_sign) * course_acceleration;
    double path_acceleration = 0.0;
    if (!SafeMultiply(
            speed,
            input.flight_path_rate_command_radps,
            path_acceleration)
        || !SafeScaleVector(
            wind_y_ned,
            signed_course_acceleration,
            course_component)
        || !SafeScaleVector(
            wind_z_ned,
            -path_acceleration,
            path_component)
        || !SafeAddVectors(
            course_component,
            path_component,
            output.required_inertial_acceleration_ned_mps2))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }

    const double thrust_acceleration =
        input.tecs_thrust_command_n / input.mass_kg;
    if (!std::isfinite(thrust_acceleration)
        || !SafeScaleVector(
            forward_hat,
            thrust_acceleration,
            output.thrust_acceleration_ned_mps2)
        || !SafeSubtractVectors(
            output.required_inertial_acceleration_ned_mps2,
            output.thrust_acceleration_ned_mps2,
            output.adapter_inertial_acceleration_ned_mps2))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }

    const Vector3 gravity_ned{{
        0.0, 0.0, constants::StandardGravityMps2}};
    Vector3 aerodynamic_full{};
    if (!SafeSubtractVectors(
            output.adapter_inertial_acceleration_ned_mps2,
            gravity_ned,
            aerodynamic_full))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    double parallel = 0.0;
    Vector3 parallel_vector{};
    if (!SafeDot(aerodynamic_full, velocity_hat, parallel)
        || !SafeScaleVector(velocity_hat, parallel, parallel_vector)
        || !SafeSubtractVectors(
            aerodynamic_full,
            parallel_vector,
            output.aerodynamic_specific_force_ned_mps2))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    const double aerodynamic_force = VectorNorm(
        output.aerodynamic_specific_force_ned_mps2);
    output.aerodynamic_load_factor_g = aerodynamic_force
        / constants::StandardGravityMps2;
    if (!std::isfinite(output.aerodynamic_load_factor_g))
    {
        output.reason = HabfmH09StorageReason::CombinedVectorUnavailable;
        return;
    }
    if (output.aerodynamic_load_factor_g
        > input.instantaneous_load_limit_g)
    {
        output.reason =
            HabfmH09StorageReason::CombinedLoadExceedsCurrentLimit;
        return;
    }

    output.admitted = true;
    output.reason = HabfmH09StorageReason::Admitted;
}

} // namespace habfm
} // namespace guidance
} // namespace LadyLuck
