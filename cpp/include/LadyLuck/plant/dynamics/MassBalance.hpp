#pragma once

#include "LadyLuck/plant/types/PlantStatus.hpp"

#include <array>

namespace LadyLuck
{
namespace plant
{
namespace dynamics
{

PlantResult<double> ComputeLoadedMassKg(
    const std::array<double, 4>& tanks_lb) noexcept;

} // namespace dynamics
} // namespace plant
} // namespace LadyLuck
