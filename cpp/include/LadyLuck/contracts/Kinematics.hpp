#pragma once

#include "LadyLuck/contracts/ScalarTypes.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <array>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
using Vector3 = std::array<Float64, 3>;
using Vector4 = std::array<Float64, 4>;
// Element (row, column) is stored at row * 3 + column.
using Matrix3RowMajor = std::array<Float64, 9>;
// Scalar-first quaternion ordering [w, x, y, z].
using QuaternionWxyz = std::array<Float64, 4>;

static_assert(sizeof(Vector3) == 3U * sizeof(Float64), "");
static_assert(std::is_standard_layout<Vector3>::value, "");
static_assert(std::is_trivially_copyable<Vector3>::value, "");

struct KinematicObservation
{
    std::uint64_t frame_index = 0U;
    std::int32_t plane_id = -1;
    std::int32_t force_side = 0;
    std::array<Float32, 3> position_neu_m{};
    std::array<Float32, 3> rpy_deg{};
    std::array<Float32, 3> velocity_body_mps{};
};

struct PlaneState
{
    std::uint64_t frame_index = 0U;
    std::int32_t plane_id = -1;
    std::int32_t force_side = 0;
    Vector3 position_ned_m{};
    Vector3 rpy_rad{};
    Vector3 velocity_body_mps{};
    Float64 speed_mps = 0.0;
    Float64 alpha_rad = 0.0;
    Float64 beta_rad = 0.0;
};

Result<PlaneState> ConvertKinematicObservation(
    const KinematicObservation& input) noexcept;
}
