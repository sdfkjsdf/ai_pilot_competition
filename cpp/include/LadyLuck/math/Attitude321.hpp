#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"

#include <array>

namespace LadyLuck
{
constexpr double Pi = 3.141592653589793238462643383279502884;
constexpr double DegreesToRadians = Pi / 180.0;
constexpr double RadiansToDegrees = 180.0 / Pi;

double WrapRadians(double angle_rad) noexcept;
double VectorNorm(const Vector3& vector) noexcept;
Vector3 MatrixVectorProduct(const Matrix3RowMajor& matrix, const Vector3& vector) noexcept;
Vector3 TransposeMatrixVectorProduct(const Matrix3RowMajor& matrix, const Vector3& vector) noexcept;
Matrix3RowMajor MatrixProduct(const Matrix3RowMajor& left, const Matrix3RowMajor& right) noexcept;
Matrix3RowMajor MatrixTranspose(const Matrix3RowMajor& matrix) noexcept;

Result<QuaternionWxyz> Euler321ToQuaternion(const Vector3& rpy_rad) noexcept;
Result<Matrix3RowMajor> RpyToDcmNedToBody(const Vector3& rpy_rad) noexcept;
Result<Vector3> RotationLogVee(const Matrix3RowMajor& rotation) noexcept;

// Plant-facing allocation-free helpers. Their operation order mirrors the
// Python attitude.py functions used at add/main ed757e27.
bool QuaternionToEuler321(
    const std::array<double, 4>& quaternion_wxyz,
    std::array<double, 3>& rpy_rad) noexcept;
bool QuaternionToDcmNedToBody(
    const std::array<double, 4>& quaternion_wxyz,
    std::array<double, 9>& dcm_ned_to_body) noexcept;
}
