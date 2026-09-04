#pragma once

#include "LadyLuck/contracts/ScalarTypes.hpp"

#include <cmath>
#include <type_traits>

namespace LadyLuck
{
namespace common
{

struct CompensatedDouble
{
    Float64 hi = 0.0;
    Float64 lo = 0.0;
};

inline CompensatedDouble FastSum(
    const Float64 left,
    const Float64 right) noexcept
{
    const Float64 sum = left + right;
    return CompensatedDouble{sum, (left - sum) + right};
}

inline CompensatedDouble ExactProduct(
    const Float64 left,
    const Float64 right) noexcept
{
    const Float64 product = left * right;
    return CompensatedDouble{
        product,
        std::fma(left, right, -product)};
}

static_assert(
    std::is_standard_layout<CompensatedDouble>::value,
    "CompensatedDouble must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<CompensatedDouble>::value,
    "CompensatedDouble must remain allocation-free.");

} // namespace common
} // namespace LadyLuck
