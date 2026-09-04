#include "LadyLuck/guidance/prefire/GunAttackFormObservation.hpp"

#include "LadyLuck/common/CompensatedDouble.hpp"
#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::AircraftGeometryKinematics;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::WezPhase;
using LadyLuck::common::CompensatedDouble;
using LadyLuck::common::ExactProduct;
using LadyLuck::common::FastSum;
using LadyLuck::guidance::prefire::GunAttackForm;
using LadyLuck::guidance::prefire::GunAttackFormObservation;
using LadyLuck::guidance::prefire::GunAttackFormReason;
using LadyLuck::guidance::prefire::GunAttackGeometryObservation;
using LadyLuck::guidance::prefire::GunAttackOptionalBool;
using LadyLuck::guidance::prefire::GunAttackOptionalDouble;

constexpr double kBattleServerRpyQuantumRad =
    LadyLuck::constants::Pi / 180.0 / 1000.0;
constexpr double kBattleServerBodyVelocityQuantumMps =
    0.001 * LadyLuck::constants::FeetToMeters;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool CheckedAdd(
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
    return std::isfinite(output);
}

bool CheckedSubtract(
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
    if ((right < 0.0 && left >= maximum + right)
        || (right > 0.0 && left <= -maximum + right))
    {
        return false;
    }
    output = left - right;
    return std::isfinite(output);
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((absolute_left > 1.0
            && absolute_right >= maximum / absolute_left)
        || (absolute_right > 1.0
            && absolute_left >= maximum / absolute_right))
    {
        return false;
    }
    output = left * right;
    return std::isfinite(output);
}

bool CheckedDivide(
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
        output = numerator / denominator;
        return true;
    }
    const double absolute_numerator = std::fabs(numerator);
    const double absolute_denominator = std::fabs(denominator);
    const double maximum = (std::numeric_limits<double>::max)();
    if (absolute_denominator < 1.0
        && absolute_numerator >= maximum * absolute_denominator)
    {
        return false;
    }
    output = numerator / denominator;
    return std::isfinite(output);
}

bool SafeSubtract(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 candidate{};
    if (!FiniteVector(left)
        || !FiniteVector(right)
        || !CheckedSubtract(left[0], right[0], candidate[0])
        || !CheckedSubtract(left[1], right[1], candidate[1])
        || !CheckedSubtract(left[2], right[2], candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeScale(
    const Vector3& value,
    const double scalar,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 candidate{};
    if (!FiniteVector(value)
        || !std::isfinite(scalar)
        || !CheckedMultiply(value[0], scalar, candidate[0])
        || !CheckedMultiply(value[1], scalar, candidate[1])
        || !CheckedMultiply(value[2], scalar, candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeDot3(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    output = 0.0;
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double inner = 0.0;
    return FiniteVector(left)
        && FiniteVector(right)
        && CheckedMultiply(left[0], right[0], term0)
        && CheckedMultiply(left[1], right[1], term1)
        && CheckedMultiply(left[2], right[2], term2)
        && CheckedAdd(term1, term2, inner)
        && CheckedAdd(term0, inner, output);
}

bool SafeCross(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 candidate{};
    double first = 0.0;
    double second = 0.0;
    if (!FiniteVector(left)
        || !FiniteVector(right)
        || !CheckedMultiply(left[1], right[2], first)
        || !CheckedMultiply(left[2], right[1], second)
        || !CheckedSubtract(first, second, candidate[0])
        || !CheckedMultiply(left[2], right[0], first)
        || !CheckedMultiply(left[0], right[2], second)
        || !CheckedSubtract(first, second, candidate[1])
        || !CheckedMultiply(left[0], right[1], first)
        || !CheckedMultiply(left[1], right[0], second)
        || !CheckedSubtract(first, second, candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeNumpyNorm3(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    double square = 0.0;
    if (!SafeDot3(value, value, square) || square < 0.0)
    {
        return false;
    }
    output = std::sqrt(square);
    return std::isfinite(output);
}

bool NextFiniteTowardMaximum(
    const double value,
    double& output) noexcept
{
    output = 0.0;
    const double maximum = (std::numeric_limits<double>::max)();
    if (!std::isfinite(value) || value >= maximum)
    {
        return false;
    }
    output = std::nextafter(value, maximum);
    return std::isfinite(output);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    // Frozen NumPy 1.26.4 three-element dot association used by the other
    // exact d90 ports in this tree.
    return left[0] * right[0]
        + (left[1] * right[1]
            + left[2] * right[2]);
}

// Exact allocation-free n=3 specialization of CPython 3.12 vector_norm(),
// which is the implementation behind d90's three-argument math.hypot calls.
double MathHypot3(const Vector3& value) noexcept
{
    Vector3 coordinates{{
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])}};
    const double maximum = (std::max)(
        coordinates[0],
        (std::max)(coordinates[1], coordinates[2]));
    if (!FiniteVector(coordinates))
    {
        return 0.0;
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    const double minimum_normal =
        (std::numeric_limits<double>::min)();
    if (maximum_exponent < -1023)
    {
        coordinates[0] /= minimum_normal;
        coordinates[1] /= minimum_normal;
        coordinates[2] /= minimum_normal;
        return minimum_normal * MathHypot3(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const CompensatedDouble product = ExactProduct(scaled, scaled);
        const CompensatedDouble sum = FastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const CompensatedDouble negative_square = ExactProduct(
        -result,
        result);
    const CompensatedDouble corrected_sum = FastSum(
        compensated_sum,
        negative_square.hi);
    compensated_sum = corrected_sum.hi;
    fraction_one += negative_square.lo;
    fraction_two += corrected_sum.lo;
    const double correction =
        compensated_sum - 1.0 + (fraction_one + fraction_two);
    result += correction / (2.0 * result);
    return result / scale;
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
}

Vector3 Negate(const Vector3& value) noexcept
{
    return Vector3{{-value[0], -value[1], -value[2]}};
}

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double ClampUnit(const double value) noexcept
{
    return (std::max)(-1.0, (std::min)(1.0, value));
}

bool UnitWithThreshold(
    const Vector3& value,
    const double threshold,
    Vector3& output) noexcept
{
    output = Vector3{};
    double magnitude = 0.0;
    double inverse_magnitude = 0.0;
    if (!FiniteVector(value)
        || !SafeNumpyNorm3(value, magnitude)
        || magnitude <= threshold
        || !CheckedDivide(1.0, magnitude, inverse_magnitude)
        || !SafeScale(value, inverse_magnitude, output))
    {
        return false;
    }
    return FiniteVector(output);
}

bool Float32HalfCell(
    const Vector3& value,
    Vector3& half_cell) noexcept
{
    half_cell = Vector3{};
    if (!FiniteVector(value))
    {
        return false;
    }
    const float float_maximum = (std::numeric_limits<float>::max)();
    const double double_float_maximum =
        static_cast<double>(float_maximum);
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (std::fabs(value[index]) > double_float_maximum)
        {
            return false;
        }
        const float represented_f32 = static_cast<float>(value[index]);
        if (std::fabs(represented_f32) >= float_maximum)
        {
            return false;
        }
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(
            std::nextafter(represented_f32, float_maximum));
        const double lower = static_cast<double>(
            std::nextafter(represented_f32, -float_maximum));
        if (!std::isfinite(represented)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return false;
        }
        half_cell[index] = 0.5 * (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }
    return FiniteVector(half_cell);
}

bool BodyVelocityNormErrorBoundMps(
    const Vector3& body_velocity,
    double& output) noexcept
{
    Vector3 half_cell{};
    double half_cell_norm = 0.0;
    const double quantization_bound =
        std::sqrt(3.0) * kBattleServerBodyVelocityQuantumMps;
    if (!Float32HalfCell(body_velocity, half_cell)
        || !SafeNumpyNorm3(half_cell, half_cell_norm)
        || !CheckedAdd(quantization_bound, half_cell_norm, output))
    {
        return false;
    }
    return std::isfinite(output) && output >= 0.0;
}

bool BodyVelocityDirectionResolutionRad(
    const Vector3& body_velocity,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(body_velocity))
    {
        return false;
    }
    double speed = 0.0;
    if (!SafeNumpyNorm3(body_velocity, speed) || speed <= 0.0)
    {
        return false;
    }
    Vector3 half_cell{};
    if (!Float32HalfCell(body_velocity, half_cell))
    {
        return false;
    }
    double half_cell_norm = 0.0;
    double error_bound = 0.0;
    double true_speed_lower = 0.0;
    const double quantization_bound =
        std::sqrt(3.0) * kBattleServerBodyVelocityQuantumMps;
    if (!SafeNumpyNorm3(half_cell, half_cell_norm)
        || !CheckedAdd(quantization_bound, half_cell_norm, error_bound)
        || !CheckedSubtract(speed, error_bound, true_speed_lower)
        || true_speed_lower <= 0.0)
    {
        return false;
    }
    double raw_ratio = 0.0;
    if (!CheckedDivide(error_bound, true_speed_lower, raw_ratio))
    {
        return false;
    }
    const double ratio = (std::min)(1.0, raw_ratio);
    output = std::asin(ratio);
    return std::isfinite(output);
}

enum class ProxyOutcome : std::uint8_t
{
    Observed = 0U,
    NotObservable = 1U,
    Error = 2U
};

struct ManeuverPlaneProxy
{
    Vector3 flight_direction_ned{};
    Vector3 lift_axis_ned{};
    Vector3 plane_normal_ned{};
    double flight_direction_resolution_rad = 0.0;
    double lift_axis_resolution_rad = 0.0;
    double plane_normal_resolution_rad = 0.0;
};

ProxyOutcome AxisDirectionResolutionRad(
    const AircraftGeometryKinematics& aircraft,
    double& output) noexcept
{
    output = 0.0;
    const Vector3& body_velocity = aircraft.velocity_body_mps;
    if (!FiniteVector(body_velocity))
    {
        return ProxyOutcome::Error;
    }
    double body_speed = 0.0;
    double body_error = 0.0;
    if (!SafeNumpyNorm3(body_velocity, body_speed)
        || !BodyVelocityNormErrorBoundMps(body_velocity, body_error))
    {
        return ProxyOutcome::Error;
    }
    double body_speed_lower = 0.0;
    if (!CheckedSubtract(body_speed, body_error, body_speed_lower))
    {
        return ProxyOutcome::Error;
    }
    if (body_speed_lower <= 0.0)
    {
        return ProxyOutcome::NotObservable;
    }
    double body_bound = 0.0;
    if (!BodyVelocityDirectionResolutionRad(body_velocity, body_bound))
    {
        return ProxyOutcome::Error;
    }
    const double attitude_bound = 3.0 * kBattleServerRpyQuantumRad;
    return CheckedAdd(body_bound, attitude_bound, output)
        ? ProxyOutcome::Observed
        : ProxyOutcome::Error;
}

ProxyOutcome CrossDirectionResolutionRad(
    const Vector3& first,
    const Vector3& second,
    const double first_bound_rad,
    const double second_bound_rad,
    double& output) noexcept
{
    output = 0.0;
    Vector3 cross{};
    double cross_magnitude = 0.0;
    double first_chord = 0.0;
    double second_chord = 0.0;
    double chord_error = 0.0;
    if (!std::isfinite(first_bound_rad)
        || !std::isfinite(second_bound_rad)
        || !SafeCross(first, second, cross)
        || !SafeNumpyNorm3(cross, cross_magnitude)
        || !CheckedMultiply(
            2.0,
            std::sin(first_bound_rad / 2.0),
            first_chord)
        || !CheckedMultiply(
            2.0,
            std::sin(second_bound_rad / 2.0),
            second_chord)
        || !CheckedAdd(first_chord, second_chord, chord_error))
    {
        return ProxyOutcome::Error;
    }
    double lower_magnitude = 0.0;
    if (!CheckedSubtract(cross_magnitude, chord_error, lower_magnitude))
    {
        return ProxyOutcome::Error;
    }
    if (lower_magnitude <= 0.0)
    {
        return ProxyOutcome::NotObservable;
    }
    double raw_ratio = 0.0;
    if (!CheckedDivide(chord_error, lower_magnitude, raw_ratio))
    {
        return ProxyOutcome::Error;
    }
    const double ratio = (std::min)(1.0, raw_ratio);
    output = 2.0 * std::asin(ratio);
    return std::isfinite(output)
        ? ProxyOutcome::Observed
        : ProxyOutcome::Error;
}

ProxyOutcome AttitudeManeuverPlaneProxy(
    const AircraftGeometryKinematics& aircraft,
    ManeuverPlaneProxy& output) noexcept
{
    output = ManeuverPlaneProxy{};
    if (!FiniteVector(aircraft.velocity_ned_mps)
        || !FiniteVector(aircraft.down_ned))
    {
        return ProxyOutcome::Error;
    }
    double velocity_norm = 0.0;
    const Vector3 body_up = Negate(aircraft.down_ned);
    double body_up_norm = 0.0;
    if (!SafeNumpyNorm3(aircraft.velocity_ned_mps, velocity_norm)
        || !SafeNumpyNorm3(body_up, body_up_norm))
    {
        return ProxyOutcome::Error;
    }
    if (velocity_norm <= 0.0 || body_up_norm <= 0.0)
    {
        return ProxyOutcome::NotObservable;
    }
    double inverse_velocity = 0.0;
    double inverse_body_up = 0.0;
    Vector3 velocity_hat{};
    Vector3 body_up_hat{};
    if (!CheckedDivide(1.0, velocity_norm, inverse_velocity)
        || !CheckedDivide(1.0, body_up_norm, inverse_body_up)
        || !SafeScale(
            aircraft.velocity_ned_mps,
            inverse_velocity,
            velocity_hat)
        || !SafeScale(body_up, inverse_body_up, body_up_hat))
    {
        return ProxyOutcome::Error;
    }

    double flight_bound = 0.0;
    const ProxyOutcome flight_outcome = AxisDirectionResolutionRad(
        aircraft,
        flight_bound);
    if (flight_outcome != ProxyOutcome::Observed)
    {
        return flight_outcome;
    }
    const double body_up_bound = 3.0 * kBattleServerRpyQuantumRad;
    const Vector3 raw_normal = Cross(velocity_hat, body_up_hat);
    Vector3 normal_hat{};
    if (!UnitWithThreshold(raw_normal, 0.0, normal_hat))
    {
        return ProxyOutcome::NotObservable;
    }
    double normal_bound = 0.0;
    const ProxyOutcome normal_outcome = CrossDirectionResolutionRad(
        velocity_hat,
        body_up_hat,
        flight_bound,
        body_up_bound,
        normal_bound);
    if (normal_outcome != ProxyOutcome::Observed)
    {
        return normal_outcome;
    }

    Vector3 lift_axis{};
    if (!UnitWithThreshold(
            Cross(normal_hat, velocity_hat),
            0.0,
            lift_axis))
    {
        return ProxyOutcome::NotObservable;
    }
    if (Dot3(lift_axis, body_up_hat) < 0.0)
    {
        lift_axis = Negate(lift_axis);
        normal_hat = Negate(normal_hat);
    }
    const double lift_bound = (std::min)(
        LadyLuck::constants::Pi,
        normal_bound + flight_bound);
    if (!std::isfinite(lift_bound))
    {
        return ProxyOutcome::Error;
    }
    output.flight_direction_ned = velocity_hat;
    output.lift_axis_ned = lift_axis;
    output.plane_normal_ned = normal_hat;
    output.flight_direction_resolution_rad = flight_bound;
    output.lift_axis_resolution_rad = lift_bound;
    output.plane_normal_resolution_rad = normal_bound;
    return ProxyOutcome::Observed;
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const float float_maximum = (std::numeric_limits<float>::max)();
    const double double_float_maximum =
        static_cast<double>(float_maximum);
    Vector3 residual{};
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (std::fabs(value[index]) > double_float_maximum)
        {
            return false;
        }
        const float represented_f32 = static_cast<float>(value[index]);
        if (std::fabs(represented_f32) >= float_maximum)
        {
            return false;
        }
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(
            std::nextafter(represented_f32, float_maximum));
        const double lower = static_cast<double>(
            std::nextafter(represented_f32, -float_maximum));
        if (!std::isfinite(represented)
            || !std::isfinite(upper)
            || !std::isfinite(lower)
            || !CheckedSubtract(
                value[index],
                represented,
                residual[index]))
        {
            return false;
        }
        double upper_distance = 0.0;
        double lower_distance = 0.0;
        if (!CheckedSubtract(upper, represented, upper_distance)
            || !CheckedSubtract(represented, lower, lower_distance))
        {
            return false;
        }
        full_cell[index] = (std::max)(
            std::fabs(upper_distance),
            std::fabs(lower_distance));
    }
    double residual_norm_guard = 0.0;
    double cell_norm_guard = 0.0;
    double residual_norm = 0.0;
    double cell_norm = 0.0;
    double combined = 0.0;
    return SafeNumpyNorm3(residual, residual_norm_guard)
        && SafeNumpyNorm3(full_cell, cell_norm_guard)
        && NextFiniteTowardMaximum(
            MathHypot3(residual),
            residual_norm)
        && NextFiniteTowardMaximum(
            MathHypot3(full_cell),
            cell_norm)
        && CheckedAdd(residual_norm, cell_norm, combined)
        && NextFiniteTowardMaximum(combined, output);
}

bool Binary64VectorResultRoundoffBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (value[index] <= -maximum || value[index] >= maximum)
        {
            return false;
        }
        const double upper = std::nextafter(
            value[index],
            maximum);
        const double lower = std::nextafter(
            value[index],
            -maximum);
        double upper_distance = 0.0;
        double lower_distance = 0.0;
        if (!CheckedSubtract(upper, value[index], upper_distance)
            || !CheckedSubtract(value[index], lower, lower_distance))
        {
            return false;
        }
        full_cell[index] = (std::max)(
            std::fabs(upper_distance),
            std::fabs(lower_distance));
    }
    double norm_guard = 0.0;
    return SafeNumpyNorm3(full_cell, norm_guard)
        && NextFiniteTowardMaximum(MathHypot3(full_cell), output);
}

enum class DirectionBoundOutcome : std::uint8_t
{
    Observed = 0U,
    NotObservable = 1U,
    Error = 2U
};

DirectionBoundOutcome RelativePositionDirectionBoundRad(
    const Vector3& own_position,
    const Vector3& opponent_position,
    const Vector3& relative_position,
    double& output) noexcept
{
    output = 0.0;
    double own_error = 0.0;
    double opponent_error = 0.0;
    double subtraction_error = 0.0;
    if (!Float32WireVectorErrorBound(own_position, own_error)
        || !Float32WireVectorErrorBound(opponent_position, opponent_error)
        || !Binary64VectorResultRoundoffBound(
            relative_position,
            subtraction_error))
    {
        return DirectionBoundOutcome::Error;
    }
    double paired_error = 0.0;
    double paired_error_up = 0.0;
    double combined_error = 0.0;
    double absolute_error = 0.0;
    double observed_range = 0.0;
    if (!CheckedAdd(own_error, opponent_error, paired_error)
        || !NextFiniteTowardMaximum(paired_error, paired_error_up)
        || !CheckedAdd(
            paired_error_up,
            subtraction_error,
            combined_error)
        || !NextFiniteTowardMaximum(combined_error, absolute_error)
        || !SafeNumpyNorm3(relative_position, observed_range))
    {
        return DirectionBoundOutcome::Error;
    }
    double range_lower_raw = 0.0;
    if (!CheckedSubtract(
            observed_range,
            absolute_error,
            range_lower_raw))
    {
        return DirectionBoundOutcome::Error;
    }
    const double range_lower = std::nextafter(
        range_lower_raw,
        0.0);
    if (!std::isfinite(range_lower) || range_lower <= 0.0)
    {
        return DirectionBoundOutcome::NotObservable;
    }
    double raw_ratio = 0.0;
    double ratio_up = 0.0;
    double angle = 0.0;
    if (!CheckedDivide(absolute_error, range_lower, raw_ratio)
        || !NextFiniteTowardMaximum(raw_ratio, ratio_up))
    {
        return DirectionBoundOutcome::Error;
    }
    const double ratio = (std::min)(1.0, ratio_up);
    angle = std::asin(ratio);
    return NextFiniteTowardMaximum(angle, output)
        ? DirectionBoundOutcome::Observed
        : DirectionBoundOutcome::Error;
}

bool SelectMatchedGunPhase(
    const double observed_range,
    const double ata_rad,
    const double t_sec,
    WezPhase& output,
    Status& status) noexcept
{
    const LadyLuck::Result<LadyLuck::WezPhaseMatch> official =
        LadyLuck::MatchWezPhase(observed_range, ata_rad, t_sec);
    if (!official.ok())
    {
        status = official.status;
        return false;
    }
    if (official.value.matched)
    {
        output = official.value.phase;
        return true;
    }

    bool eligible_found = false;
    WezPhase widest_eligible{};
    for (std::size_t index = 0U;
         index < LadyLuck::OfficialWezPhaseCount;
         ++index)
    {
        const LadyLuck::Result<WezPhase> phase =
            LadyLuck::OfficialWezPhaseAt(index);
        if (!phase.ok())
        {
            status = phase.status;
            return false;
        }
        if (t_sec >= phase.value.start_sec
            && observed_range >= phase.value.min_range_m
            && observed_range <= phase.value.max_range_m
            && (!eligible_found
                || phase.value.angle_rad > widest_eligible.angle_rad))
        {
            eligible_found = true;
            widest_eligible = phase.value;
        }
    }
    if (eligible_found)
    {
        output = widest_eligible;
        return true;
    }
    const LadyLuck::Result<WezPhase> active =
        LadyLuck::ActiveWezPhase(t_sec);
    if (!active.ok())
    {
        status = active.status;
        return false;
    }
    output = active.value;
    return true;
}

bool FiniteOptional(const GunAttackOptionalDouble& value) noexcept
{
    return !value.has_value || std::isfinite(value.value);
}

bool ValidateGeometry(
    const GunAttackGeometryObservation& geometry) noexcept
{
    const bool all_angles_present =
        geometry.relative_maneuver_plane_angle_rad.has_value
        && geometry.maneuver_plane_alignment_cosine.has_value
        && geometry.maneuver_plane_separation_rad.has_value
        && geometry.maneuver_plane_resolution_rad.has_value
        && geometry.own_lift_direction_resolution_rad.has_value
        && geometry.own_los_plane_incidence_rad.has_value
        && geometry.opponent_los_plane_incidence_rad.has_value
        && geometry.los_plane_resolution_rad.has_value;
    const bool any_angle_present =
        geometry.relative_maneuver_plane_angle_rad.has_value
        || geometry.maneuver_plane_alignment_cosine.has_value
        || geometry.maneuver_plane_separation_rad.has_value
        || geometry.maneuver_plane_resolution_rad.has_value
        || geometry.own_lift_direction_resolution_rad.has_value
        || geometry.own_los_plane_incidence_rad.has_value
        || geometry.opponent_los_plane_incidence_rad.has_value
        || geometry.los_plane_resolution_rad.has_value;
    return geometry.valid
        && std::isfinite(geometry.t_sec)
        && geometry.t_sec >= 0.0
        && FiniteVector(geometry.los_rate_world_radps)
        && std::isfinite(geometry.los_rate_radps)
        && geometry.los_rate_radps >= 0.0
        && std::isfinite(geometry.attacker_aim_error_rad)
        && geometry.attacker_aim_error_rad >= 0.0
        && std::isfinite(geometry.active_gun_cone_rad)
        && geometry.active_gun_cone_rad >= 0.0
        && FiniteOptional(geometry.relative_maneuver_plane_angle_rad)
        && FiniteOptional(geometry.maneuver_plane_alignment_cosine)
        && FiniteOptional(geometry.maneuver_plane_separation_rad)
        && FiniteOptional(geometry.maneuver_plane_resolution_rad)
        && FiniteOptional(geometry.own_lift_direction_resolution_rad)
        && FiniteOptional(geometry.own_los_plane_incidence_rad)
        && FiniteOptional(geometry.opponent_los_plane_incidence_rad)
        && FiniteOptional(geometry.los_plane_resolution_rad)
        && (geometry.maneuver_plane_observable
            ? (all_angles_present
                && geometry.same_maneuver_plane_within_resolution.has_value)
            : (!any_angle_present
                && !geometry.same_maneuver_plane_within_resolution.has_value));
}

void FailGeometry(
    GunAttackGeometryObservation& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = GunAttackGeometryObservation{};
    status.code = code;
}

void FailForm(
    GunAttackFormObservation& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = GunAttackFormObservation{};
    status.code = code;
}

void SetAllPlaneValues(
    GunAttackGeometryObservation& output,
    const double relative_angle,
    const double alignment_cosine,
    const double separation,
    const double plane_resolution,
    const double own_lift_resolution,
    const double own_incidence,
    const double opponent_incidence,
    const double los_resolution,
    const bool same_plane) noexcept
{
    output.relative_maneuver_plane_angle_rad =
        GunAttackOptionalDouble{true, relative_angle};
    output.maneuver_plane_alignment_cosine =
        GunAttackOptionalDouble{true, alignment_cosine};
    output.maneuver_plane_separation_rad =
        GunAttackOptionalDouble{true, separation};
    output.maneuver_plane_resolution_rad =
        GunAttackOptionalDouble{true, plane_resolution};
    output.own_lift_direction_resolution_rad =
        GunAttackOptionalDouble{true, own_lift_resolution};
    output.own_los_plane_incidence_rad =
        GunAttackOptionalDouble{true, own_incidence};
    output.opponent_los_plane_incidence_rad =
        GunAttackOptionalDouble{true, opponent_incidence};
    output.los_plane_resolution_rad =
        GunAttackOptionalDouble{true, los_resolution};
    output.maneuver_plane_observable = true;
    output.same_maneuver_plane_within_resolution =
        GunAttackOptionalBool{true, same_plane};
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

const char* GunAttackFormLabel(const GunAttackForm form) noexcept
{
    switch (form)
    {
    case GunAttackForm::Snapshot:
        return "SNAPSHOT";
    case GunAttackForm::Tracking:
        return "TRACKING";
    case GunAttackForm::Indeterminate:
    default:
        return "INDETERMINATE";
    }
}

const char* GunAttackFormReasonLabel(
    const GunAttackFormReason reason) noexcept
{
    switch (reason)
    {
    case GunAttackFormReason::ManualManeuverPlanesResolvablyDiffer:
        return "manual_maneuver_planes_resolvably_differ";
    case GunAttackFormReason::ManualHighCrossingOrNonTailPass:
        return "manual_high_crossing_or_non_tail_pass";
    case GunAttackFormReason::TailTrackingRegionUnresolved:
        return "tail_tracking_region_unresolved";
    case GunAttackFormReason::ManualFixedLosSamePlaneTailTracking:
        return "manual_fixed_los_same_plane_tail_tracking";
    case GunAttackFormReason::CausalAimContinuityNotYetObserved:
        return "causal_aim_continuity_not_yet_observed";
    case GunAttackFormReason::ManualContinuousAimSamePlaneTailTracking:
        return "manual_continuous_aim_same_plane_tail_tracking";
    case GunAttackFormReason::ManualMomentaryAimSolutionPassage:
        return "manual_momentary_aim_solution_passage";
    case GunAttackFormReason::ManualContinuouslyRetainedOfficialAimSolution:
        return "manual_continuously_retained_official_aim_solution";
    case GunAttackFormReason::ManeuverPlaneUnobservable:
    default:
        return "maneuver_plane_unobservable";
    }
}

void ObserveGunAttackGeometry(
    const DogfightGeometryFrame& frame,
    GunAttackGeometryObservation& output,
    Status& status) noexcept
{
    output = GunAttackGeometryObservation{};
    status = Status{};
    if (!std::isfinite(frame.t_sec))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (frame.t_sec < 0.0)
    {
        FailGeometry(output, status, StatusCode::InvalidArgument);
        return;
    }
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& opponent_position = frame.opponent.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& opponent_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(opponent_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(opponent_velocity))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }

    Vector3 relative_position{};
    Vector3 relative_velocity{};
    double range_squared = 0.0;
    if (!SafeSubtract(
            opponent_position,
            own_position,
            relative_position)
        || !SafeSubtract(
            opponent_velocity,
            own_velocity,
            relative_velocity)
        || !SafeDot3(
            relative_position,
            relative_position,
            range_squared))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (range_squared <= constants::Tiny)
    {
        FailGeometry(output, status, StatusCode::InvalidArgument);
        return;
    }
    const double observed_range = std::sqrt(range_squared);
    double inverse_range = 0.0;
    double inverse_range_squared = 0.0;
    Vector3 los_hat{};
    Vector3 relative_cross{};
    Vector3 los_rate_world{};
    double los_rate = 0.0;
    if (!std::isfinite(observed_range)
        || !CheckedDivide(1.0, observed_range, inverse_range)
        || !CheckedDivide(
            1.0,
            range_squared,
            inverse_range_squared)
        || !SafeScale(relative_position, inverse_range, los_hat)
        || !SafeCross(
            relative_position,
            relative_velocity,
            relative_cross)
        || !SafeScale(
            relative_cross,
            inverse_range_squared,
            los_rate_world)
        || !SafeNumpyNorm3(los_rate_world, los_rate))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }

    WezPhase matched_phase{};
    if (!SelectMatchedGunPhase(
            observed_range,
            frame.enemy_offense.ata_rad,
            frame.t_sec,
            matched_phase,
            status))
    {
        output = GunAttackGeometryObservation{};
        return;
    }
    const double aim_error = std::fabs(frame.enemy_offense.ata_rad);
    if (!std::isfinite(aim_error)
        || !std::isfinite(matched_phase.angle_rad)
        || matched_phase.angle_rad < 0.0)
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }

    output.t_sec = frame.t_sec;
    output.los_rate_world_radps = los_rate_world;
    output.los_rate_radps = los_rate;
    output.attacker_aim_error_rad = aim_error;
    output.active_gun_cone_rad = matched_phase.angle_rad;
    output.attacker_aim_inside_active_cone =
        aim_error < matched_phase.angle_rad;

    ManeuverPlaneProxy own_proxy{};
    ManeuverPlaneProxy opponent_proxy{};
    const ProxyOutcome own_proxy_outcome =
        AttitudeManeuverPlaneProxy(frame.own, own_proxy);
    if (own_proxy_outcome == ProxyOutcome::Error)
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }
    const ProxyOutcome opponent_proxy_outcome =
        AttitudeManeuverPlaneProxy(frame.opponent, opponent_proxy);
    if (opponent_proxy_outcome == ProxyOutcome::Error)
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }

    if (own_proxy_outcome == ProxyOutcome::Observed
        && opponent_proxy_outcome == ProxyOutcome::Observed)
    {
        Vector3 own_normal{};
        Vector3 opponent_normal{};
        if (!UnitWithThreshold(
                own_proxy.plane_normal_ned,
                constants::Tiny,
                own_normal)
            || !UnitWithThreshold(
                opponent_proxy.plane_normal_ned,
                constants::Tiny,
                opponent_normal))
        {
            FailGeometry(output, status, StatusCode::InvalidArgument);
            return;
        }
        const double normal_cosine = std::fabs(ClampUnit(
            Dot3(own_normal, opponent_normal)));
        const double separation = std::acos(normal_cosine);
        const double plane_resolution = (std::min)(
            0.5 * constants::Pi,
            own_proxy.plane_normal_resolution_rad
                + opponent_proxy.plane_normal_resolution_rad);
        double los_resolution = 0.0;
        const DirectionBoundOutcome los_resolution_outcome =
            RelativePositionDirectionBoundRad(
                own_position,
                opponent_position,
                relative_position,
                los_resolution);
        if (los_resolution_outcome == DirectionBoundOutcome::Error)
        {
            FailGeometry(output, status, StatusCode::NonFiniteInput);
            return;
        }
        if (los_resolution_outcome == DirectionBoundOutcome::Observed)
        {
            const double own_incidence = std::asin((std::min)(
                1.0,
                std::fabs(Dot3(los_hat, own_normal))));
            const double opponent_incidence = std::asin((std::min)(
                1.0,
                std::fabs(Dot3(los_hat, opponent_normal))));
            const double own_incidence_bound = (std::min)(
                0.5 * constants::Pi,
                los_resolution + own_proxy.plane_normal_resolution_rad);
            const double opponent_incidence_bound = (std::min)(
                0.5 * constants::Pi,
                los_resolution
                    + opponent_proxy.plane_normal_resolution_rad);
            const bool same_plane =
                separation <= plane_resolution
                && own_incidence <= own_incidence_bound
                && opponent_incidence <= opponent_incidence_bound;

            Vector3 own_lift{};
            Vector3 opponent_lift{};
            if (!UnitWithThreshold(
                    frame.own.down_ned,
                    constants::Tiny,
                    own_lift)
                || !UnitWithThreshold(
                    frame.opponent.down_ned,
                    constants::Tiny,
                    opponent_lift))
            {
                FailGeometry(output, status, StatusCode::InvalidArgument);
                return;
            }
            own_lift = Negate(own_lift);
            opponent_lift = Negate(opponent_lift);
            const Vector3 own_projected = Subtract(
                own_lift,
                Scale(los_hat, Dot3(own_lift, los_hat)));
            const Vector3 opponent_projected = Subtract(
                opponent_lift,
                Scale(los_hat, Dot3(opponent_lift, los_hat)));
            double own_projected_norm = 0.0;
            double opponent_projected_norm = 0.0;
            if (!SafeNumpyNorm3(own_projected, own_projected_norm)
                || !SafeNumpyNorm3(
                    opponent_projected,
                    opponent_projected_norm))
            {
                FailGeometry(output, status, StatusCode::NonFiniteInput);
                return;
            }
            if (own_projected_norm > constants::Tiny
                && opponent_projected_norm > constants::Tiny)
            {
                double own_projected_inverse = 0.0;
                double opponent_projected_inverse = 0.0;
                Vector3 own_projected_hat{};
                Vector3 opponent_projected_hat{};
                if (!CheckedDivide(
                        1.0,
                        own_projected_norm,
                        own_projected_inverse)
                    || !CheckedDivide(
                        1.0,
                        opponent_projected_norm,
                        opponent_projected_inverse)
                    || !SafeScale(
                        own_projected,
                        own_projected_inverse,
                        own_projected_hat)
                    || !SafeScale(
                        opponent_projected,
                        opponent_projected_inverse,
                        opponent_projected_hat))
                {
                    FailGeometry(
                        output,
                        status,
                        StatusCode::NonFiniteInput);
                    return;
                }
                const double alignment_cosine = ClampUnit(
                    Dot3(own_projected_hat, opponent_projected_hat));
                const double sine = Dot3(
                    los_hat,
                    Cross(own_projected_hat, opponent_projected_hat));
                const double relative_angle = std::atan2(
                    sine,
                    alignment_cosine);
                if (!std::isfinite(relative_angle)
                    || !std::isfinite(alignment_cosine)
                    || !std::isfinite(separation)
                    || !std::isfinite(plane_resolution)
                    || !std::isfinite(own_incidence)
                    || !std::isfinite(opponent_incidence)
                    || !std::isfinite(los_resolution))
                {
                    FailGeometry(output, status, StatusCode::NonFiniteInput);
                    return;
                }
                SetAllPlaneValues(
                    output,
                    relative_angle,
                    alignment_cosine,
                    separation,
                    plane_resolution,
                    own_proxy.lift_axis_resolution_rad,
                    own_incidence,
                    opponent_incidence,
                    los_resolution,
                    same_plane);
            }
        }
    }

    if (!FiniteVector(frame.own.nose_ned))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }
    Vector3 own_nose{};
    if (!UnitWithThreshold(
            frame.own.nose_ned,
            constants::Tiny,
            own_nose))
    {
        FailGeometry(output, status, StatusCode::InvalidArgument);
        return;
    }
    double own_speed = 0.0;
    double opponent_speed = 0.0;
    if (!SafeNumpyNorm3(own_velocity, own_speed)
        || !SafeNumpyNorm3(opponent_velocity, opponent_speed))
    {
        FailGeometry(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (own_speed > constants::Tiny
        && opponent_speed > constants::Tiny)
    {
        double own_speed_inverse = 0.0;
        double opponent_speed_inverse = 0.0;
        Vector3 own_velocity_hat{};
        Vector3 opponent_velocity_hat{};
        if (!CheckedDivide(1.0, own_speed, own_speed_inverse)
            || !CheckedDivide(
                1.0,
                opponent_speed,
                opponent_speed_inverse)
            || !SafeScale(
                own_velocity,
                own_speed_inverse,
                own_velocity_hat)
            || !SafeScale(
                opponent_velocity,
                opponent_speed_inverse,
                opponent_velocity_hat))
        {
            FailGeometry(output, status, StatusCode::NonFiniteInput);
            return;
        }
        const bool attacker_behind = Dot3(own_nose, los_hat) < 0.0;
        const bool same_direction_halfspace =
            Dot3(own_velocity_hat, opponent_velocity_hat) > 0.0;
        output.attacker_in_tail_tracking_region = GunAttackOptionalBool{
            true,
            attacker_behind && same_direction_halfspace};
    }

    output.valid = true;
    if (!ValidateGeometry(output))
    {
        FailGeometry(output, status, StatusCode::InvalidArgument);
    }
}

void ClassifyManualAttackFormEndpoints(
    const GunAttackGeometryObservation& geometry,
    GunAttackFormObservation& output,
    Status& status) noexcept
{
    output = GunAttackFormObservation{};
    status = Status{};
    if (!ValidateGeometry(geometry))
    {
        FailForm(output, status, StatusCode::InvalidArgument);
        return;
    }

    output.valid = true;
    output.geometry = geometry;
    output.same_maneuver_plane_endpoint =
        geometry.same_maneuver_plane_within_resolution;
    output.fixed_los_endpoint = geometry.los_rate_radps
        <= (std::numeric_limits<double>::epsilon)();
    if (!geometry.same_maneuver_plane_within_resolution.has_value)
    {
        output.attack_form = GunAttackForm::Indeterminate;
        output.reason = GunAttackFormReason::ManeuverPlaneUnobservable;
        return;
    }
    if (!geometry.same_maneuver_plane_within_resolution.value)
    {
        output.attack_form = GunAttackForm::Snapshot;
        output.reason = GunAttackFormReason::
            ManualManeuverPlanesResolvablyDiffer;
        return;
    }
    if (geometry.attacker_in_tail_tracking_region.has_value
        && !geometry.attacker_in_tail_tracking_region.value)
    {
        output.attack_form = GunAttackForm::Snapshot;
        output.reason = GunAttackFormReason::
            ManualHighCrossingOrNonTailPass;
        return;
    }
    if (!geometry.attacker_in_tail_tracking_region.has_value)
    {
        output.attack_form = GunAttackForm::Indeterminate;
        output.reason = GunAttackFormReason::TailTrackingRegionUnresolved;
        return;
    }
    if (output.fixed_los_endpoint
        && geometry.attacker_aim_inside_active_cone)
    {
        output.attack_form = GunAttackForm::Tracking;
        output.continuous_aim_solution = true;
        output.reason = GunAttackFormReason::
            ManualFixedLosSamePlaneTailTracking;
        return;
    }
    output.attack_form = GunAttackForm::Indeterminate;
    output.reason = GunAttackFormReason::
        CausalAimContinuityNotYetObserved;
}

GunAttackFormObserver::GunAttackFormObserver() noexcept
{
    Reset();
}

void GunAttackFormObserver::Configure(
    const GunAttackFormObserverConfig& config,
    Status& status) noexcept
{
    status = Status{};
    if (config.retention_authority_radps_valid
        && (!std::isfinite(config.retention_authority_radps)
            || config.retention_authority_radps <= 0.0))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    retention_authority_radps_valid_ =
        config.retention_authority_radps_valid;
    retention_authority_radps_ = config.retention_authority_radps_valid
        ? config.retention_authority_radps
        : 0.0;
    Reset();
}

void GunAttackFormObserver::Reset() noexcept
{
    previous_t_sec_valid_ = false;
    previous_t_sec_ = 0.0;
    previous_aim_inside_ = false;
    previous_same_plane_ = false;
    previous_tail_tracking_ = false;
    retention_first_t_valid_ = false;
    retention_first_t_ = 0.0;
    retention_previous_t_valid_ = false;
    retention_previous_t_ = 0.0;
}

void GunAttackFormObserver::Update(
    const DogfightGeometryFrame& frame,
    GunAttackFormObservation& output,
    Status& status) noexcept
{
    output = GunAttackFormObservation{};
    status = Status{};
    GunAttackGeometryObservation geometry{};
    ObserveGunAttackGeometry(frame, geometry, status);
    if (!status.sample_valid())
    {
        Reset();
        return;
    }
    GunAttackFormObservation result{};
    ClassifyManualAttackFormEndpoints(geometry, result, status);
    if (!status.sample_valid())
    {
        Reset();
        return;
    }

    const bool causal_next_sample = previous_t_sec_valid_
        && geometry.t_sec > previous_t_sec_;
    const bool current_same =
        geometry.same_maneuver_plane_within_resolution.has_value
        && geometry.same_maneuver_plane_within_resolution.value;
    const bool current_tail =
        geometry.attacker_in_tail_tracking_region.has_value
        && geometry.attacker_in_tail_tracking_region.value;
    const bool current_aim =
        geometry.attacker_aim_inside_active_cone;
    const bool continuous = causal_next_sample
        && current_same
        && current_tail
        && current_aim
        && previous_same_plane_
        && previous_tail_tracking_
        && previous_aim_inside_;
    const bool momentary = causal_next_sample
        && previous_aim_inside_
        && !current_aim
        && geometry.los_rate_radps
            > (std::numeric_limits<double>::epsilon)();

    if (result.attack_form == GunAttackForm::Indeterminate && continuous)
    {
        result.attack_form = GunAttackForm::Tracking;
        result.continuous_aim_solution = true;
        result.reason = GunAttackFormReason::
            ManualContinuousAimSamePlaneTailTracking;
    }
    else if (result.attack_form == GunAttackForm::Indeterminate && momentary)
    {
        result.attack_form = GunAttackForm::Snapshot;
        result.momentary_aim_solution = true;
        result.reason = GunAttackFormReason::
            ManualMomentaryAimSolutionPassage;
    }

    const double sample_t = geometry.t_sec;
    const double scoring_rate = frame.enemy_offense.damage_rate;
    if (!std::isfinite(scoring_rate))
    {
        Reset();
        FailForm(output, status, StatusCode::NonFiniteInput);
        return;
    }
    const bool scoring = scoring_rate > 0.0;
    if (scoring)
    {
        if (!(retention_previous_t_valid_
                && sample_t > retention_previous_t_))
        {
            retention_first_t_valid_ = true;
            retention_first_t_ = sample_t;
        }
        retention_previous_t_valid_ = true;
        retention_previous_t_ = sample_t;
    }
    else
    {
        retention_first_t_valid_ = false;
        retention_first_t_ = 0.0;
        retention_previous_t_valid_ = false;
        retention_previous_t_ = 0.0;
    }
    if (retention_authority_radps_valid_
        && retention_first_t_valid_
        && (sample_t - retention_first_t_)
            > constants::Pi / retention_authority_radps_
        && result.attack_form != GunAttackForm::Tracking)
    {
        result.attack_form = GunAttackForm::Tracking;
        result.continuous_aim_solution = true;
        result.reason = GunAttackFormReason::
            ManualContinuouslyRetainedOfficialAimSolution;
    }

    previous_t_sec_valid_ = true;
    previous_t_sec_ = geometry.t_sec;
    previous_aim_inside_ = current_aim;
    previous_same_plane_ = current_same;
    previous_tail_tracking_ = current_tail;
    output = result;
}

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
