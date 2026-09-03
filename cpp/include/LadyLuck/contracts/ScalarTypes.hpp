#pragma once

#include <limits>

namespace LadyLuck
{

using Float32 = float;
using Float64 = double;

static_assert(sizeof(Float32) == 4U, "Float32 must be 32-bit.");
static_assert(sizeof(Float64) == 8U, "Float64 must be 64-bit.");
static_assert(
    std::numeric_limits<Float32>::is_iec559,
    "Float32 must use IEC 559 / IEEE-754.");
static_assert(
    std::numeric_limits<Float64>::is_iec559,
    "Float64 must use IEC 559 / IEEE-754.");

} // namespace LadyLuck
