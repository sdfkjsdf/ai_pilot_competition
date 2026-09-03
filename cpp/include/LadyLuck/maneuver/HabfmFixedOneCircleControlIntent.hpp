#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
// Allocation-free numerical reference shared by the production ControlIntent
// materializer and the legacy TacticalCommand parity adapter. This is raw
// lateral aim/speed guidance only; downstream control owns p/q/r, Nz,
// surfaces, thrust, and the measured aircraft response.
struct HabfmFixedOneCircleReference
{
    Vector3 aim_point_m{};
    double desired_speed_mps = 0.0;
    double capture_range_des_m = 0.0;
};

void BuildHabfmFixedOneCircleReference(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign,
    HabfmFixedOneCircleReference& output,
    Status& status) noexcept;

void BuildHabfmFixedOneCircleIntent(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign,
    ControlIntent& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<HabfmFixedOneCircleReference>::value,
    "HABFM ONE_CIRCLE reference must remain allocation-free.");
}
