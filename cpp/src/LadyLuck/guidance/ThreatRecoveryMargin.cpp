#include "LadyLuck/guidance/ThreatRecoveryMargin.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::Vector3;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool SafeAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    if (!std::isfinite(right))
    {
        output = 0.0;
        return false;
    }
    return SafeAdd(left, -right, output);
}

bool SafeMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if (left == 0.0 || right == 0.0)
    {
        return true;
    }

    const double left_magnitude = std::fabs(left);
    const double right_magnitude = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((left_magnitude > 1.0
            && right_magnitude >= maximum / left_magnitude)
        || (right_magnitude > 1.0
            && left_magnitude >= maximum / right_magnitude))
    {
        return false;
    }
    const int exponent_sum = std::ilogb(left_magnitude)
        + std::ilogb(right_magnitude);
    if (exponent_sum <= std::numeric_limits<double>::min_exponent - 1)
    {
        return false;
    }

    output = left * right;
    if (!std::isfinite(output)
        || (output != 0.0 && std::fpclassify(output) == FP_SUBNORMAL))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeDivide(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(numerator)
        || !std::isfinite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    if (numerator == 0.0)
    {
        return true;
    }

    const int exponent_difference = std::ilogb(std::fabs(numerator))
        - std::ilogb(std::fabs(denominator));
    if (exponent_difference
            >= std::numeric_limits<double>::max_exponent - 1
        || exponent_difference
            <= std::numeric_limits<double>::min_exponent)
    {
        return false;
    }

    output = numerator / denominator;
    if (!std::isfinite(output)
        || (output != 0.0 && std::fpclassify(output) == FP_SUBNORMAL))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeScaledNorm3(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const double scale = (std::max)(
        std::fabs(value[0]),
        (std::max)(std::fabs(value[1]), std::fabs(value[2])));
    if (scale == 0.0)
    {
        return true;
    }
    if (std::fpclassify(scale) == FP_SUBNORMAL)
    {
        return false;
    }

    Vector3 scaled{};
    if (!SafeDivide(value[0], scale, scaled[0])
        || !SafeDivide(value[1], scale, scaled[1])
        || !SafeDivide(value[2], scale, scaled[2]))
    {
        return false;
    }
    double square_x = 0.0;
    double square_y = 0.0;
    double square_z = 0.0;
    double sum_xy = 0.0;
    double sum = 0.0;
    if (!SafeMultiply(scaled[0], scaled[0], square_x)
        || !SafeMultiply(scaled[1], scaled[1], square_y)
        || !SafeMultiply(scaled[2], scaled[2], square_z)
        || !SafeAdd(square_x, square_y, sum_xy)
        || !SafeAdd(sum_xy, square_z, sum))
    {
        return false;
    }
    const double scaled_norm = std::sqrt(sum);
    return SafeMultiply(scale, scaled_norm, output);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{

void EvaluateThreatRecoveryMargin(
    const DogfightGeometryFrame& frame,
    const bool own_turn_capability_admitted,
    const double own_turn_capability_g,
    ThreatRecoveryMarginReceipt& output) noexcept
{
    output = ThreatRecoveryMarginReceipt{};
    const double closing_speed_mps = frame.closing_speed_mps;
    const double enemy_range_m = frame.enemy_offense.range_m;
    const double enemy_rmax_m = frame.enemy_offense.phase.max_range_m;
    double own_speed_mps = 0.0;
    if (!std::isfinite(closing_speed_mps)
        || !std::isfinite(enemy_range_m)
        || enemy_range_m < 0.0
        || !std::isfinite(enemy_rmax_m)
        || enemy_rmax_m <= 0.0
        || !own_turn_capability_admitted
        || !std::isfinite(own_turn_capability_g)
        || own_turn_capability_g <= 1.0
        || !SafeScaledNorm3(frame.own.velocity_ned_mps, own_speed_mps)
        || own_speed_mps <= 0.0)
    {
        return;
    }

    double n_minus_one = 0.0;
    double n_plus_one = 0.0;
    double load_radical_squared = 0.0;
    if (!SafeSubtract(own_turn_capability_g, 1.0, n_minus_one)
        || !SafeAdd(own_turn_capability_g, 1.0, n_plus_one)
        || !SafeMultiply(
            n_minus_one,
            n_plus_one,
            load_radical_squared)
        || load_radical_squared <= 0.0)
    {
        return;
    }
    const double load_radical = std::sqrt(load_radical_squared);
    double speed_per_load = 0.0;
    double reversal_scale_s2pm = 0.0;
    double own_reversal_time_s = 0.0;
    if (!SafeDivide(own_speed_mps, load_radical, speed_per_load)
        || !SafeDivide(
            constants::Pi,
            constants::StandardGravityMps2,
            reversal_scale_s2pm)
        || !SafeMultiply(
            reversal_scale_s2pm,
            speed_per_load,
            own_reversal_time_s)
        || own_reversal_time_s <= 0.0)
    {
        return;
    }

    output.closing_speed_mps = closing_speed_mps;
    output.own_reversal_time_valid = true;
    output.own_reversal_time_s = own_reversal_time_s;
    if (closing_speed_mps <= 0.0)
    {
        output.evaluated = true;
        return;
    }

    double range_gap_m = 0.0;
    if (!SafeSubtract(enemy_range_m, enemy_rmax_m, range_gap_m))
    {
        output = ThreatRecoveryMarginReceipt{};
        return;
    }
    double time_to_enemy_wez_s = 0.0;
    if (range_gap_m > 0.0
        && !SafeDivide(
            range_gap_m,
            closing_speed_mps,
            time_to_enemy_wez_s))
    {
        output = ThreatRecoveryMarginReceipt{};
        return;
    }

    output.time_to_enemy_wez_valid = true;
    output.time_to_enemy_wez_s = time_to_enemy_wez_s;
    output.exhausted = time_to_enemy_wez_s <= own_reversal_time_s;
    output.evaluated = true;
}

} // namespace guidance
} // namespace LadyLuck
