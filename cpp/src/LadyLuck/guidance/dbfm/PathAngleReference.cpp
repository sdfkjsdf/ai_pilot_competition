#include "LadyLuck/guidance/dbfm/PathAngleReference.hpp"

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

bool ValidLaw(const LadyLuck::PathAngleReferenceLaw& law) noexcept
{
    const double values[] = {
        law.course_gain_per_s,
        law.path_gain_per_s,
        law.course_rate_limit_radps,
        law.path_rate_limit_radps,
        law.path_command_limit_rad};
    for (const double value : values)
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            return false;
        }
    }
    return true;
}

double Clamp(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::max(lower, std::min(upper, value));
}

double PythonWrap(const double angle) noexcept
{
    const double two_pi = 2.0 * LadyLuck::Pi;
    double wrapped = std::fmod(angle + LadyLuck::Pi, two_pi);
    if (wrapped < 0.0)
    {
        wrapped += two_pi;
    }
    return wrapped - LadyLuck::Pi;
}

LadyLuck::Matrix3RowMajor DirectDcmNedToBody(
    const LadyLuck::Vector3& rpy) noexcept
{
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double yaw = rpy[2];
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const LadyLuck::Matrix3RowMajor tx{{
        1.0, 0.0, 0.0,
        0.0, cr, sr,
        0.0, -sr, cr}};
    const LadyLuck::Matrix3RowMajor ty{{
        cp, 0.0, -sp,
        0.0, 1.0, 0.0,
        sp, 0.0, cp}};
    const LadyLuck::Matrix3RowMajor tz{{
        cy, sy, 0.0,
        -sy, cy, 0.0,
        0.0, 0.0, 1.0}};
    return LadyLuck::MatrixProduct(LadyLuck::MatrixProduct(tx, ty), tz);
}

LadyLuck::Matrix3RowMajor WindToNed(
    const double course,
    const double path) noexcept
{
    const double cc = std::cos(course);
    const double sc = std::sin(course);
    const double cg = std::cos(path);
    const double sg = std::sin(path);
    return LadyLuck::Matrix3RowMajor{{
        cc * cg, -sc, cc * sg,
        sc * cg, cc, sc * sg,
        -sg, 0.0, cg}};
}
}

namespace LadyLuck
{
Result<Vector3> BuildPathAngleAccelerationReferenceNed(
    const PathAngleReferenceInput& input,
    const PathAngleReferenceLaw& law) noexcept
{
    Result<Vector3> result{};
    if (!FiniteVector(input.aim_point_ned_m)
        || !FiniteVector(input.position_ned_m)
        || !FiniteVector(input.velocity_body_mps)
        || !FiniteVector(input.rpy_rad))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    if (!ValidLaw(law))
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }

    const Matrix3RowMajor ned_to_body = DirectDcmNedToBody(input.rpy_rad);
    const Vector3 velocity_ned = TransposeMatrixVectorProduct(
        ned_to_body,
        input.velocity_body_mps);
    const double speed_raw = VectorNorm(velocity_ned);
    if (!std::isfinite(speed_raw))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const double speed = std::max(speed_raw, constants::Epsilon);
    const double course = std::atan2(velocity_ned[1], velocity_ned[0]);
    const double horizontal_speed = std::hypot(
        velocity_ned[0],
        velocity_ned[1]);
    const double path = std::atan2(-velocity_ned[2], horizontal_speed);

    const Vector3 displacement{{
        input.aim_point_ned_m[0] - input.position_ned_m[0],
        input.aim_point_ned_m[1] - input.position_ned_m[1],
        input.aim_point_ned_m[2] - input.position_ned_m[2]}};
    const double horizontal_range = std::hypot(
        displacement[0],
        displacement[1]);
    const double course_command = horizontal_range < constants::Epsilon
        ? course
        : std::atan2(displacement[1], displacement[0]);
    const double path_command = Clamp(
        std::atan2(
            -displacement[2],
            std::max(horizontal_range, constants::Epsilon)),
        -law.path_command_limit_rad,
        law.path_command_limit_rad);
    const double course_rate = Clamp(
        law.course_gain_per_s * PythonWrap(course_command - course),
        -law.course_rate_limit_radps,
        law.course_rate_limit_radps);
    const double path_rate = Clamp(
        law.path_gain_per_s * (path_command - path),
        -law.path_rate_limit_radps,
        law.path_rate_limit_radps);

    const Matrix3RowMajor wind_to_ned = WindToNed(course, path);
    const Matrix3RowMajor ned_to_wind = MatrixTranspose(wind_to_ned);
    const Vector3 gravity_ned_g{{0.0, 0.0, 1.0}};
    const Vector3 gravity_wind_g = MatrixVectorProduct(
        ned_to_wind,
        gravity_ned_g);
    const Vector3 inertial_wind_g{{
        0.0,
        speed * std::max(std::cos(path), 0.1) * course_rate
            / constants::StandardGravityMps2,
        -speed * path_rate / constants::StandardGravityMps2}};
    const Vector3 specific_wind_yz_g{{
        0.0,
        inertial_wind_g[1] - gravity_wind_g[1],
        inertial_wind_g[2] - gravity_wind_g[2]}};
    const Vector3 specific_ned_g = MatrixVectorProduct(
        wind_to_ned,
        specific_wind_yz_g);
    result.value = Vector3{{
        constants::StandardGravityMps2 * specific_ned_g[0],
        constants::StandardGravityMps2 * specific_ned_g[1],
        constants::StandardGravityMps2
            + constants::StandardGravityMps2 * specific_ned_g[2]}};
    if (!FiniteVector(result.value))
    {
        result.value = Vector3{};
        result.status.code = StatusCode::NonFiniteInput;
    }
    return result;
}
}
