#pragma once

#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

// Exact allocation-free representation of d90 TacticalSpeedFloorObserver's
// immutable 25-cell altitude cache. The caller controls branch ordering.
void SampleTacticalSpeedFloor(
    double altitude_m,
    TacticalSpeedFloorSample& output) noexcept;

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
