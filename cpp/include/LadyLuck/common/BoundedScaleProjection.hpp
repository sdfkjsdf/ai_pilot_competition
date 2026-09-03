#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace LadyLuck
{
namespace common
{

// Find the largest non-negative binary64 scale, no greater than
// initial_scale, whose evaluated magnitude is strictly below bound.
//
// Non-negative finite IEEE-754 encodings have the same ordering as their
// uint64 bit patterns.  An integer bisection therefore searches the complete
// representable interval in at most 64 evaluations; it cannot acquire the
// unbounded latency of repeated nextafter(..., 0.0) steps.
//
// Evaluate(scale, magnitude) must be noexcept, return false only when its
// arithmetic result is not finite, and have a nondecreasing magnitude over
// this non-negative scale interval.  This helper owns no tactical threshold:
// the caller supplies the physical strict bound.
template <typename Evaluate>
bool LargestRepresentableScaleBelowBound(
    const double initial_scale,
    const double bound,
    Evaluate evaluate,
    double& admitted_scale,
    double& admitted_magnitude) noexcept
{
    admitted_scale = 0.0;
    admitted_magnitude = 0.0;
    if (!std::isfinite(initial_scale)
        || initial_scale < 0.0
        || initial_scale > 1.0
        || !std::isfinite(bound)
        || bound <= 0.0)
    {
        return false;
    }

    const double normalized_initial_scale = initial_scale == 0.0
        ? 0.0
        : initial_scale;
    double observed = 0.0;
    if (!evaluate(normalized_initial_scale, observed)
        || !std::isfinite(observed)
        || observed < 0.0)
    {
        return false;
    }
    if (observed < bound)
    {
        admitted_scale = normalized_initial_scale;
        admitted_magnitude = observed;
        return true;
    }

    double zero_observed = 0.0;
    if (!evaluate(0.0, zero_observed)
        || !std::isfinite(zero_observed)
        || zero_observed < 0.0
        || !(zero_observed < bound))
    {
        return false;
    }

    std::uint64_t lower_bits = 0U;
    std::uint64_t upper_bits = 0U;
    std::memcpy(
        &upper_bits,
        &normalized_initial_scale,
        sizeof(upper_bits));
    double lower_observed = zero_observed;

    for (std::size_t iteration = 0U;
        iteration < std::numeric_limits<std::uint64_t>::digits
            && upper_bits - lower_bits > 1U;
        ++iteration)
    {
        const std::uint64_t middle_bits = lower_bits
            + (upper_bits - lower_bits) / 2U;
        double middle_scale = 0.0;
        std::memcpy(&middle_scale, &middle_bits, sizeof(middle_scale));
        double middle_observed = 0.0;
        if (!evaluate(middle_scale, middle_observed)
            || !std::isfinite(middle_observed)
            || middle_observed < 0.0)
        {
            return false;
        }
        if (middle_observed < bound)
        {
            lower_bits = middle_bits;
            lower_observed = middle_observed;
        }
        else
        {
            upper_bits = middle_bits;
        }
    }

    std::memcpy(&admitted_scale, &lower_bits, sizeof(admitted_scale));
    admitted_magnitude = lower_observed;
    return true;
}

} // namespace common
} // namespace LadyLuck
