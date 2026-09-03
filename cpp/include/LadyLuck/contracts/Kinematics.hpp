#pragma once

#include "LadyLuck/contracts/Status.hpp"

#include <array>
#include <cstdint>

namespace LadyLuck
{
using Vector3 = std::array<double, 3>;
using Vector4 = std::array<double, 4>;
// Element (row, column) is stored at row * 3 + column.
using Matrix3RowMajor = std::array<double, 9>;
// Scalar-first quaternion ordering [w, x, y, z].
using QuaternionWxyz = std::array<double, 4>;

struct KinematicObservation
{
    std::uint64_t frame_index = 0U;
    std::int32_t plane_id = -1;
    std::int32_t force_side = 0;
    std::array<float, 3> position_neu_m{};
    std::array<float, 3> rpy_deg{};
    std::array<float, 3> velocity_body_mps{};
};

struct PlaneState
{
    std::uint64_t frame_index = 0U;
    std::int32_t plane_id = -1;
    std::int32_t force_side = 0;
    Vector3 position_ned_m{};
    Vector3 rpy_rad{};
    Vector3 velocity_body_mps{};
    double speed_mps = 0.0;
    double alpha_rad = 0.0;
    double beta_rad = 0.0;
};

Result<PlaneState> ConvertKinematicObservation(
    const KinematicObservation& input) noexcept;
}
