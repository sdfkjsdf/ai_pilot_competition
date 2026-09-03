#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>

namespace
{
bool Finite3(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool Finite4(const LadyLuck::QuaternionWxyz& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2])
        && std::isfinite(value[3]);
}

bool Finite9(const LadyLuck::Matrix3RowMajor& value) noexcept
{
    for (const double element : value)
    {
        if (!std::isfinite(element))
        {
            return false;
        }
    }
    return true;
}
}

namespace LadyLuck
{
double WrapRadians(const double angle_rad) noexcept
{
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double VectorNorm(const Vector3& vector) noexcept
{
    return std::sqrt(
        vector[0] * vector[0]
        + vector[1] * vector[1]
        + vector[2] * vector[2]);
}

Vector3 MatrixVectorProduct(const Matrix3RowMajor& matrix, const Vector3& vector) noexcept
{
    return Vector3{{
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]}};
}

Vector3 TransposeMatrixVectorProduct(const Matrix3RowMajor& matrix, const Vector3& vector) noexcept
{
    return Vector3{{
        matrix[0] * vector[0] + matrix[3] * vector[1] + matrix[6] * vector[2],
        matrix[1] * vector[0] + matrix[4] * vector[1] + matrix[7] * vector[2],
        matrix[2] * vector[0] + matrix[5] * vector[1] + matrix[8] * vector[2]}};
}

Matrix3RowMajor MatrixProduct(const Matrix3RowMajor& left, const Matrix3RowMajor& right) noexcept
{
    Matrix3RowMajor output{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            output[row * 3U + column] =
                left[row * 3U] * right[column]
                + left[row * 3U + 1U] * right[3U + column]
                + left[row * 3U + 2U] * right[6U + column];
        }
    }
    return output;
}

Matrix3RowMajor MatrixTranspose(const Matrix3RowMajor& matrix) noexcept
{
    return Matrix3RowMajor{{
        matrix[0], matrix[3], matrix[6],
        matrix[1], matrix[4], matrix[7],
        matrix[2], matrix[5], matrix[8]}};
}

Result<QuaternionWxyz> Euler321ToQuaternion(const Vector3& rpy_rad) noexcept
{
    Result<QuaternionWxyz> result{};
    if (!Finite3(rpy_rad))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    const double cr = std::cos(rpy_rad[0] / 2.0);
    const double sr = std::sin(rpy_rad[0] / 2.0);
    const double cp = std::cos(rpy_rad[1] / 2.0);
    const double sp = std::sin(rpy_rad[1] / 2.0);
    const double cy = std::cos(rpy_rad[2] / 2.0);
    const double sy = std::sin(rpy_rad[2] / 2.0);
    result.value = QuaternionWxyz{{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy}};
    const double norm = std::sqrt(
        result.value[0] * result.value[0]
        + result.value[1] * result.value[1]
        + result.value[2] * result.value[2]
        + result.value[3] * result.value[3]);
    if (!std::isfinite(norm) || norm <= 0.0)
    {
        result.status.code = StatusCode::InvalidArgument;
        result.value = QuaternionWxyz{};
        return result;
    }
    for (double& element : result.value)
    {
        element /= norm;
    }
    return result;
}

bool QuaternionToEuler321(
    const std::array<double, 4>& quaternion_wxyz,
    std::array<double, 3>& rpy_rad) noexcept
{
    rpy_rad = std::array<double, 3>{};
    if (!Finite4(quaternion_wxyz))
    {
        return false;
    }
    const double w = quaternion_wxyz[0];
    const double x = quaternion_wxyz[1];
    const double y = quaternion_wxyz[2];
    const double z = quaternion_wxyz[3];
    rpy_rad[0] = std::atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y));
    rpy_rad[1] = std::asin(std::max(
        -1.0,
        std::min(1.0, 2.0 * (w * y - z * x))));
    rpy_rad[2] = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
    return Finite3(rpy_rad);
}

bool QuaternionToDcmNedToBody(
    const std::array<double, 4>& quaternion_wxyz,
    std::array<double, 9>& dcm_ned_to_body) noexcept
{
    dcm_ned_to_body = std::array<double, 9>{};
    if (!Finite4(quaternion_wxyz))
    {
        return false;
    }
    const double w = quaternion_wxyz[0];
    const double x = quaternion_wxyz[1];
    const double y = quaternion_wxyz[2];
    const double z = quaternion_wxyz[3];
    dcm_ned_to_body = Matrix3RowMajor{{
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
        2.0 * (x * y - w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + w * x),
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y)}};
    return Finite9(dcm_ned_to_body);
}

Result<Matrix3RowMajor> RpyToDcmNedToBody(const Vector3& rpy_rad) noexcept
{
    Result<Matrix3RowMajor> result{};
    const Result<QuaternionWxyz> quaternion = Euler321ToQuaternion(rpy_rad);
    if (!quaternion.ok())
    {
        result.status = quaternion.status;
        return result;
    }
    if (!QuaternionToDcmNedToBody(quaternion.value, result.value))
    {
        result.status.code = StatusCode::NonFiniteInput;
    }
    return result;
}

Result<Vector3> RotationLogVee(const Matrix3RowMajor& rotation) noexcept
{
    Result<Vector3> result{};
    if (!Finite9(rotation))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const double trace = rotation[0] + rotation[4] + rotation[8];
    const double cosine = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    const double angle = std::acos(cosine);
    const Vector3 skew_vee{{
        0.5 * (rotation[7] - rotation[5]),
        0.5 * (rotation[2] - rotation[6]),
        0.5 * (rotation[3] - rotation[1])}};
    if (angle < 1.0e-6)
    {
        result.value = skew_vee;
        return result;
    }
    const double sine = std::sin(angle);
    if (std::fabs(sine) < 1.0e-6)
    {
        result.status.code = StatusCode::AmbiguousRotation;
        return result;
    }
    result.value = Vector3{{
        (angle / sine) * skew_vee[0],
        (angle / sine) * skew_vee[1],
        (angle / sine) * skew_vee[2]}};
    return result;
}
}
