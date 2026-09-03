#pragma once

#include <array>

namespace LadyLuck
{
namespace plant
{

// Internal estimator state representation. Quaternion ordering is [w, x, y, z],
// position is NED, velocity and omega are body-axis quantities.
struct PlantState
{
    std::array<double, 3> position_ned_m{{0.0, 0.0, 0.0}};
    std::array<double, 4> quaternion_wxyz{{1.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> velocity_body_mps{{0.0, 0.0, 0.0}};
    std::array<double, 3> omega_body_rad_s{{0.0, 0.0, 0.0}};
};

} // namespace plant
} // namespace LadyLuck
