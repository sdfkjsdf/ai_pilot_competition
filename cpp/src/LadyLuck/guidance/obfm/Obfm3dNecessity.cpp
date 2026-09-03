#include "LadyLuck/guidance/obfm/Obfm3dNecessity.hpp"

#include <cmath>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* Obfm3dNecessityReasonLabel(
    const Obfm3dNecessityReason reason) noexcept
{
    switch (reason)
    {
    case Obfm3dNecessityReason::InputNotObservable:
        return "input_not_observable";
    case Obfm3dNecessityReason::NoPositiveClosure:
        return "no_positive_closure";
    case Obfm3dNecessityReason::InPlaneArrestSufficient:
        return "in_plane_arrest_sufficient";
    case Obfm3dNecessityReason::InPlaneArrestInsufficient:
        return "in_plane_arrest_insufficient";
    default:
        return "unknown";
    }
}

void EvaluateObfm3dNecessity(
    const Obfm3dNecessityInput& input,
    Obfm3dNecessityReceipt& output) noexcept
{
    output = Obfm3dNecessityReceipt{};
    output.range_m = input.range_m;
    output.closing_speed_mps = input.closing_speed_mps;
    output.available_deceleration_mps2 =
        input.available_deceleration_mps2;

    if (!std::isfinite(input.range_m)
        || !std::isfinite(input.closing_speed_mps)
        || !std::isfinite(input.available_deceleration_mps2)
        || input.range_m <= 0.0
        || input.available_deceleration_mps2 <= 0.0)
    {
        output.reason = Obfm3dNecessityReason::InputNotObservable;
        return;
    }

    output.observable = true;
    if (input.closing_speed_mps <= 0.0)
    {
        output.in_plane_arrest_sufficient = true;
        output.reason = Obfm3dNecessityReason::NoPositiveClosure;
        return;
    }

    output.positive_closure = true;

    // The displayed arrestable speed is diagnostic only.  Admission compares
    // the original stopping-distance products after exponent normalization,
    // so neither overflow nor sqrt-product rounding changes the exact
    // equality rule.
    output.arrestable_closing_speed_mps =
        std::sqrt(2.0)
        * std::sqrt(input.available_deceleration_mps2)
        * std::sqrt(input.range_m);

    // Minimum physical statement only: the in-plane channel is insufficient
    // when the deceleration required to arrest closure inside the measured
    // range exceeds the admitted deceleration.  Dividing by range before the
    // final multiplication avoids the former exponent-by-exponent comparator
    // without changing the stopping-distance meaning.
    const double required_deceleration_mps2 =
        0.5 * input.closing_speed_mps
        * (input.closing_speed_mps / input.range_m);
    output.three_dimensional_needed =
        required_deceleration_mps2
        > input.available_deceleration_mps2;
    output.in_plane_arrest_sufficient =
        !output.three_dimensional_needed;
    output.reason = output.three_dimensional_needed
        ? Obfm3dNecessityReason::InPlaneArrestInsufficient
        : Obfm3dNecessityReason::InPlaneArrestSufficient;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
