#include "LadyLuck/guidance/GunDefense.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cstddef>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
enum class FiniteMathState : std::uint8_t
{
    Available = 0U,
    TooSmall = 1U,
    ArithmeticUnavailable = 2U
};

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

// Live add/main NumPy 1.26.4/MKL three-element reduction:
// (term0 + term2) + term1.
FiniteMathState PythonNorm3(
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
        ? FiniteMathState::TooSmall
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
        const FiniteMathState norm_state = PythonNorm3(horizontal, magnitude);
        if (norm_state == FiniteMathState::ArithmeticUnavailable)
        {
            arithmetic_unavailable = true;
            continue;
        }
        if (norm_state == FiniteMathState::TooSmall)
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
}

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

    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (side_sign != -1 && side_sign != 1)
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
    // Preserve Python _vec3 materialization order exactly.
    if (!FiniteVector(own_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!FiniteVector(adversary_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!FiniteVector(own_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!FiniteVector(own_nose))
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
    bool direction_arithmetic_unavailable = false;
    if (!FirstHorizontalUnit(
        los_horizontal,
        los_available,
        own_nose,
        own_velocity,
        los_direction,
        output.direction_source,
        direction_arithmetic_unavailable))
    {
        output.reason = direction_arithmetic_unavailable
            ? HorizontalBreakReferenceReason::ArithmeticUnavailable
            : HorizontalBreakReferenceReason::HorizontalDirectionUnavailable;
        return;
    }
    const Vector3 lateral_direction{{
        -los_direction[1],
        los_direction[0],
        0.0}};

    // Python materializes range -> speed -> capture, then validates them in
    // the same order.  The safe norm records arithmetic unavailability
    // without ever creating an Inf/NaN intermediate.
    const double range_m = frame.enemy_offense.range_m;
    double speed_mps = 0.0;
    const FiniteMathState speed_state = PythonNorm3(own_velocity, speed_mps);
    const double capture_range_m = frame.enemy_offense.phase.max_range_m;

    if (!std::isfinite(range_m))
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
    if (speed_state == FiniteMathState::TooSmall)
    {
        output.reason = HorizontalBreakReferenceReason::OwnSpeedUnavailable;
        return;
    }
    if (!std::isfinite(capture_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
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

Result<TacticalCommand> BuildHorizontalBreakCommand(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign) noexcept
{
    // Compatibility adapter for the frozen offline TacticalCommand parity
    // harness and development-only DBFM tree.  Production fixed-rate paths
    // consume BuildHorizontalBreakReference directly and never allocate
    // dynamic labels or enter this exception boundary.
    Result<TacticalCommand> result{};
    HorizontalBreakReferenceReceipt reference{};
    BuildHorizontalBreakReference(
        frame, side_sign, reference, result.status);
    if (!result.status.ok())
    {
        return result;
    }
    if (!reference.command_available)
    {
        // Preserve the historical Result API's rejected-event classification;
        // the new typed API above is the production Status-Ok fallback seam.
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    TacticalCommand candidate{};
    candidate.aim_point_m = reference.aim_point_m;
    candidate.desired_speed_mps = reference.desired_speed_mps;
    candidate.capture_range_des_m = reference.capture_range_des_m;
    try
    {
        candidate.behavior_label = "GUN_DEFENSE_HORIZONTAL_BREAK";
        candidate.mode_label = "DBFM";
    }
    catch (...)
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }
    return MakeValidatedTacticalCommand(candidate);
}

GunDefensePolicy::GunDefensePolicy() noexcept
{
    Reset();
}

void GunDefensePolicy::Reset() noexcept
{
    active_ = false;
    side_sign_ = 1;
    entry_count_ = 0U;
    toward_side_candidate_held_ = false;
}

std::int32_t GunDefensePolicy::NextSideSign() const noexcept
{
    return entry_count_ % 2U == 0U ? 1 : -1;
}

GunDefenseSnapshot GunDefensePolicy::Snapshot() const noexcept
{
    GunDefenseSnapshot snapshot{};
    snapshot.active = active_;
    snapshot.side_sign = side_sign_;
    snapshot.entry_count = entry_count_;
    snapshot.toward_side_candidate_held = toward_side_candidate_held_;
    return snapshot;
}

Result<GunDefenseOutput> GunDefensePolicy::Step(
    const DogfightGeometryFrame& frame,
    const bool admitted_threat_active,
    const OptionalValue<std::int32_t>& entry_side_sign) noexcept
{
    Result<GunDefenseOutput> result{};
    if (entry_side_sign.has_value
        && entry_side_sign.value != -1
        && entry_side_sign.value != 1)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    const bool entered = admitted_threat_active && !active_;
    const bool cleared = !admitted_threat_active && active_;
    if (!admitted_threat_active)
    {
        result.value.command.has_value = false;
        result.value.threat_active = false;
        result.value.entered = false;
        result.value.cleared = cleared;
        result.value.side_sign.has_value = false;
        result.value.entry_count = entry_count_;
        result.value.toward_side_candidate_held = false;
        result.value.missile_branch_enabled = false;

        active_ = false;
        toward_side_candidate_held_ = false;
        return result;
    }

    std::int32_t candidate_side_sign = side_sign_;
    std::uint64_t candidate_entry_count = entry_count_;
    bool candidate_toward_side_held = toward_side_candidate_held_;
    if (entered)
    {
        if (candidate_entry_count
            == (std::numeric_limits<std::uint64_t>::max)())
        {
            result.status.code = StatusCode::InvalidConfiguration;
            return result;
        }
        candidate_side_sign = entry_side_sign.has_value
            ? entry_side_sign.value
            : NextSideSign();
        ++candidate_entry_count;
        candidate_toward_side_held = entry_side_sign.has_value;
    }

    Result<TacticalCommand> command = BuildHorizontalBreakCommand(
        frame,
        candidate_side_sign);
    if (!command.ok())
    {
        result.status = command.status;
        return result;
    }

    try
    {
        result.value.command.value = std::move(command.value);
    }
    catch (...)
    {
        Result<GunDefenseOutput> failed{};
        failed.status.code = StatusCode::InvalidConfiguration;
        return failed;
    }
    result.value.command.has_value = true;
    result.value.threat_active = true;
    result.value.entered = entered;
    result.value.cleared = false;
    result.value.side_sign.has_value = true;
    result.value.side_sign.value = candidate_side_sign;
    result.value.entry_count = candidate_entry_count;
    result.value.toward_side_candidate_held = candidate_toward_side_held;
    result.value.missile_branch_enabled = false;

    active_ = true;
    side_sign_ = candidate_side_sign;
    entry_count_ = candidate_entry_count;
    toward_side_candidate_held_ = candidate_toward_side_held;
    return result;
}
}
