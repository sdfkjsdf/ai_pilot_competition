#include "LadyLuck/guidance/GunDefenseControlIntent.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/FiniteMathState.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace
{
using LadyLuck::common::FiniteMathState;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
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
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right > 0.0 && left <= -maximum + right)
        || (right < 0.0 && left >= maximum + right))
    {
        return false;
    }
    output = left - right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
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
    output = left * right;
    if (!std::isfinite(output))
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
    const double numerator_magnitude = std::fabs(numerator);
    const double denominator_magnitude = std::fabs(denominator);
    const double maximum = (std::numeric_limits<double>::max)();
    if (denominator_magnitude < 1.0
        && numerator_magnitude >= maximum * denominator_magnitude)
    {
        return false;
    }
    output = numerator / denominator;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

FiniteMathState Norm3(
    const LadyLuck::Vector3& value,
    double& output) noexcept
{
    double x_squared = 0.0;
    double y_squared = 0.0;
    double z_squared = 0.0;
    double xz_squared = 0.0;
    double sum_squared = 0.0;
    if (!SafeMultiply(value[0], value[0], x_squared)
        || !SafeMultiply(value[1], value[1], y_squared)
        || !SafeMultiply(value[2], value[2], z_squared)
        || !SafeAdd(x_squared, z_squared, xz_squared)
        || !SafeAdd(xz_squared, y_squared, sum_squared))
    {
        output = 0.0;
        return FiniteMathState::ArithmeticUnavailable;
    }
    output = std::sqrt(sum_squared);
    return output < LadyLuck::constants::Tiny
        ? FiniteMathState::Degenerate
        : FiniteMathState::Available;
}

bool NormalizeAccepted(
    const LadyLuck::Vector3& value,
    const double magnitude,
    LadyLuck::Vector3& output) noexcept
{
    return magnitude >= LadyLuck::constants::Tiny
        && SafeDivide(value[0], magnitude, output[0])
        && SafeDivide(value[1], magnitude, output[1])
        && SafeDivide(value[2], magnitude, output[2]);
}

bool FirstHorizontalUnit(
    const LadyLuck::Vector3& los,
    const bool los_available,
    const LadyLuck::Vector3& nose,
    const LadyLuck::Vector3& velocity,
    LadyLuck::Vector3& output,
    LadyLuck::HorizontalBreakDirectionSource& source,
    bool& arithmetic_unavailable) noexcept
{
    const LadyLuck::Vector3 candidates[] = {los, nose, velocity};
    const LadyLuck::HorizontalBreakDirectionSource sources[] = {
        LadyLuck::HorizontalBreakDirectionSource::AttackerLos,
        LadyLuck::HorizontalBreakDirectionSource::OwnNose,
        LadyLuck::HorizontalBreakDirectionSource::OwnVelocity};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (index == 0U && !los_available)
        {
            arithmetic_unavailable = true;
            continue;
        }
        LadyLuck::Vector3 horizontal = candidates[index];
        horizontal[2] = 0.0;
        double magnitude = 0.0;
        const FiniteMathState norm_state = Norm3(horizontal, magnitude);
        if (norm_state == FiniteMathState::ArithmeticUnavailable)
        {
            arithmetic_unavailable = true;
            continue;
        }
        if (norm_state == FiniteMathState::Degenerate)
        {
            continue;
        }
        if (!NormalizeAccepted(horizontal, magnitude, output))
        {
            arithmetic_unavailable = true;
            continue;
        }
        source = sources[index];
        return true;
    }
    output = LadyLuck::Vector3{};
    source = LadyLuck::HorizontalBreakDirectionSource::None;
    return false;
}
} // namespace

namespace LadyLuck
{

void BuildHorizontalBreakReference(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign,
    HorizontalBreakReferenceReceipt& output,
    Status& status) noexcept
{
    output = HorizontalBreakReferenceReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};

    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || (side_sign != -1 && side_sign != 1))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.evaluated = true;
    output.side_sign = side_sign;

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& own_nose = frame.own.nose_ned;
    if (!FiniteVector(own_position)
        || !FiniteVector(adversary_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(own_nose))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    Vector3 los_horizontal{};
    const bool los_available = SafeSubtract(
            adversary_position[0], own_position[0], los_horizontal[0])
        && SafeSubtract(
            adversary_position[1], own_position[1], los_horizontal[1]);
    Vector3 los_direction{};
    bool arithmetic_unavailable = false;
    if (!FirstHorizontalUnit(
            los_horizontal,
            los_available,
            own_nose,
            own_velocity,
            los_direction,
            output.direction_source,
            arithmetic_unavailable))
    {
        output.reason = arithmetic_unavailable
            ? HorizontalBreakReferenceReason::ArithmeticUnavailable
            : HorizontalBreakReferenceReason::HorizontalDirectionUnavailable;
        return;
    }
    const Vector3 lateral_direction{{
        -los_direction[1], los_direction[0], 0.0}};

    const double range_m = frame.enemy_offense.range_m;
    double speed_mps = 0.0;
    const FiniteMathState speed_state = Norm3(own_velocity, speed_mps);
    const double capture_range_m = frame.enemy_offense.phase.max_range_m;
    if (!std::isfinite(range_m)
        || !std::isfinite(capture_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (range_m < constants::Tiny)
    {
        output.reason = HorizontalBreakReferenceReason::RangeUnavailable;
        return;
    }
    if (speed_state == FiniteMathState::ArithmeticUnavailable)
    {
        output.reason = HorizontalBreakReferenceReason::ArithmeticUnavailable;
        return;
    }
    if (speed_state == FiniteMathState::Degenerate)
    {
        output.reason = HorizontalBreakReferenceReason::OwnSpeedUnavailable;
        return;
    }
    if (capture_range_m < constants::Tiny)
    {
        output.reason =
            HorizontalBreakReferenceReason::CaptureRangeUnavailable;
        return;
    }

    double signed_range_m = 0.0;
    double lateral_x_m = 0.0;
    double lateral_y_m = 0.0;
    double aim_x_m = 0.0;
    double aim_y_m = 0.0;
    if (!SafeMultiply(
            static_cast<double>(side_sign), range_m, signed_range_m)
        || !SafeMultiply(
            signed_range_m, lateral_direction[0], lateral_x_m)
        || !SafeMultiply(
            signed_range_m, lateral_direction[1], lateral_y_m)
        || !SafeAdd(own_position[0], lateral_x_m, aim_x_m)
        || !SafeAdd(own_position[1], lateral_y_m, aim_y_m))
    {
        output.reason = HorizontalBreakReferenceReason::ArithmeticUnavailable;
        return;
    }

    output.aim_point_m = Vector3{{aim_x_m, aim_y_m, own_position[2]}};
    output.desired_speed_mps = speed_mps;
    output.capture_range_des_m = capture_range_m;
    output.command_available = true;
    output.reason = HorizontalBreakReferenceReason::Ready;
}

} // namespace LadyLuck
