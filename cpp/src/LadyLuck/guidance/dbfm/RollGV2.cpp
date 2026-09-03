#include "LadyLuck/guidance/dbfm/RollGV2.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

LadyLuck::Vector3 Cross(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}
}

namespace LadyLuck
{
Result<RollGV2Result> BuildRollGV2Reference(
    const RollGV2Input& input) noexcept
{
    Result<RollGV2Result> result{};
    if (!FiniteVector(input.inertial_acceleration_ned_mps2)
        || !FiniteVector(input.velocity_ned_mps)
        || !std::isfinite(input.roll_actual_rad)
        || !std::isfinite(input.pitch_rad)
        || !std::isfinite(input.alpha_rad)
        || !std::isfinite(input.roll_gain_per_s)
        || !std::isfinite(input.max_roll_rate_radps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    if (input.roll_gain_per_s <= 0.0 || input.max_roll_rate_radps <= 0.0)
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }

    const double speed = VectorNorm(input.velocity_ned_mps);
    if (!std::isfinite(speed))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    if (speed <= constants::Epsilon)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    const Vector3 x_velocity{{
        input.velocity_ned_mps[0] / speed,
        input.velocity_ned_mps[1] / speed,
        input.velocity_ned_mps[2] / speed}};
    const Vector3 down{{0.0, 0.0, 1.0}};
    Vector3 y_velocity = Cross(down, x_velocity);
    const double y_norm = VectorNorm(y_velocity);
    if (!std::isfinite(y_norm) || y_norm <= constants::Epsilon)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    for (double& value : y_velocity)
    {
        value /= y_norm;
    }
    const Vector3 z_velocity = Cross(x_velocity, y_velocity);
    const Vector3 specific_force{{
        input.inertial_acceleration_ned_mps2[0],
        input.inertial_acceleration_ned_mps2[1],
        input.inertial_acceleration_ned_mps2[2]
            - constants::StandardGravityMps2}};
    const double parallel = Dot(specific_force, x_velocity);
    const Vector3 force_perpendicular{{
        specific_force[0] - parallel * x_velocity[0],
        specific_force[1] - parallel * x_velocity[1],
        specific_force[2] - parallel * x_velocity[2]}};

    result.value.g_cmd = VectorNorm(force_perpendicular)
        / constants::StandardGravityMps2;
    result.value.bank_command_rad = std::atan2(
        Dot(force_perpendicular, y_velocity),
        -Dot(force_perpendicular, z_velocity));
    result.value.bank_error_rad = std::atan2(
        std::sin(result.value.bank_command_rad - input.roll_actual_rad),
        std::cos(result.value.bank_command_rad - input.roll_actual_rad));
    result.value.roll_rate_command_radps = std::max(
        -input.max_roll_rate_radps,
        std::min(
            input.max_roll_rate_radps,
            input.roll_gain_per_s * result.value.bank_error_rad));
    result.value.nz_command_g = 1.0
        + (result.value.g_cmd - 1.0)
            * std::max(0.0, std::cos(result.value.bank_error_rad));
    const double gain = speed * std::cos(input.alpha_rad)
        / constants::StandardGravityMps2;
    if (!std::isfinite(gain) || std::fabs(gain) <= constants::Epsilon)
    {
        result.value = RollGV2Result{};
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    const double gravity_projection = std::cos(input.pitch_rad)
        * std::cos(input.roll_actual_rad);
    result.value.pitch_rate_command_radps =
        (result.value.nz_command_g - gravity_projection) / gain;
    const double values[] = {
        result.value.g_cmd,
        result.value.bank_command_rad,
        result.value.bank_error_rad,
        result.value.roll_rate_command_radps,
        result.value.nz_command_g,
        result.value.pitch_rate_command_radps};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            result.value = RollGV2Result{};
            result.status.code = StatusCode::NonFiniteInput;
            return result;
        }
    }
    return result;
}
}
