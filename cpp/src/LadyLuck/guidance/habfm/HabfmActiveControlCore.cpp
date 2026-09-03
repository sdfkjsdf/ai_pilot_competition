#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/maneuver/HabfmFixedOneCircleControlIntent.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace
{
constexpr double kBodyVelocityQuantumMps = 0.001 * 0.3048;
constexpr double kPlaneInfoFloat32RelativeQuantum = 0x1.0p-23;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
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
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
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
    if (left == 0.0 || right == 0.0)
    {
        return true;
    }
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    const double minimum = (std::numeric_limits<double>::min)();
    if (absolute_left < minimum || absolute_right < minimum)
    {
        return false;
    }
    if ((absolute_left > 1.0
            && absolute_right >= maximum / absolute_left)
        || (absolute_left < 1.0 && absolute_right < 1.0
            && absolute_right <= minimum / absolute_left))
    {
        return false;
    }
    output = left * right;
    if (!std::isfinite(output)
        || (output != 0.0 && std::fabs(output) < minimum))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedVectorSubtract(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    if (!CheckedSubtract(left[0], right[0], output[0])
        || !CheckedSubtract(left[1], right[1], output[1])
        || !CheckedSubtract(left[2], right[2], output[2]))
    {
        output = LadyLuck::Vector3{};
        return false;
    }
    return true;
}

// Scale first so finite vectors whose mathematical norm is representable do
// not overflow an intermediate square.  Components more than 510 binary
// exponents below the scale cannot affect a binary64 norm and are skipped
// before division/multiplication, avoiding underflow exceptions as well.
bool CheckedNorm3(
    const LadyLuck::Vector3& value,
    double& output) noexcept
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
    if (scale < (std::numeric_limits<double>::min)())
    {
        return false;
    }
    const int scale_exponent = std::ilogb(scale);
    double sum = 0.0;
    for (const double component : value)
    {
        const double magnitude = std::fabs(component);
        if (magnitude == 0.0
            || std::ilogb(magnitude) - scale_exponent < -510)
        {
            continue;
        }
        const double normalized = component / scale;
        double square = 0.0;
        if (!CheckedMultiply(normalized, normalized, square)
            || !CheckedAdd(sum, square, sum))
        {
            output = 0.0;
            return false;
        }
    }
    const double root = std::sqrt(sum);
    if (!std::isfinite(root)
        || scale >= (std::numeric_limits<double>::max)() / root
        || !CheckedMultiply(scale, root, output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedRangeRate(
    const LadyLuck::Vector3& relative_position,
    const double separation_m,
    const LadyLuck::Vector3& relative_velocity,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(relative_position)
        || !FiniteVector(relative_velocity)
        || !std::isfinite(separation_m)
        || separation_m <= 0.0)
    {
        return false;
    }
    LadyLuck::Vector3 unit_line_of_sight{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double magnitude = std::fabs(relative_position[index]);
        if (magnitude == 0.0
            || std::ilogb(magnitude) - std::ilogb(separation_m) < -510)
        {
            unit_line_of_sight[index] = 0.0;
        }
        else
        {
            unit_line_of_sight[index] =
                relative_position[index] / separation_m;
        }
    }
    const double velocity_scale = (std::max)(
        std::fabs(relative_velocity[0]),
        (std::max)(
            std::fabs(relative_velocity[1]),
            std::fabs(relative_velocity[2])));
    if (velocity_scale == 0.0)
    {
        return true;
    }
    if (velocity_scale < (std::numeric_limits<double>::min)())
    {
        return false;
    }
    double scaled_dot = 0.0;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double velocity = relative_velocity[index];
        if (velocity == 0.0 || unit_line_of_sight[index] == 0.0)
        {
            continue;
        }
        if (std::ilogb(std::fabs(velocity))
            - std::ilogb(velocity_scale) < -510)
        {
            continue;
        }
        const double scaled_velocity = velocity / velocity_scale;
        double product = 0.0;
        if (!CheckedMultiply(
                unit_line_of_sight[index], scaled_velocity, product)
            || !CheckedAdd(scaled_dot, product, scaled_dot))
        {
            output = 0.0;
            return false;
        }
    }
    if (scaled_dot == 0.0)
    {
        return true;
    }
    if (std::fabs(scaled_dot) < (std::numeric_limits<double>::min)())
    {
        return false;
    }
    // CheckedMultiply already guards the representable product without
    // evaluating max/abs(scaled_dot) when abs(scaled_dot) < 1.  The latter
    // quotient can itself overflow under /fp:strict despite a finite product.
    if (!CheckedMultiply(velocity_scale, scaled_dot, output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

LadyLuck::Result<LadyLuck::Vector3> HorizontalUnit(
    const LadyLuck::Vector3& value) noexcept
{
    LadyLuck::Result<LadyLuck::Vector3> result{};
    if (!FiniteVector(value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    LadyLuck::Vector3 horizontal = value;
    horizontal[2] = 0.0;
    const double magnitude = VectorNorm(horizontal);
    if (!std::isfinite(magnitude))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    if (magnitude < LadyLuck::constants::Tiny)
    {
        result.status.code = LadyLuck::StatusCode::InvalidArgument;
        return result;
    }
    result.value = LadyLuck::Vector3{{
        horizontal[0] / magnitude,
        horizontal[1] / magnitude,
        horizontal[2] / magnitude}};
    return result;
}

LadyLuck::Result<LadyLuck::Vector3> Unit3(
    const LadyLuck::Vector3& value) noexcept
{
    LadyLuck::Result<LadyLuck::Vector3> result{};
    double magnitude = 0.0;
    if (!CheckedNorm3(value, magnitude))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    if (magnitude < LadyLuck::constants::Tiny)
    {
        result.status.code = LadyLuck::StatusCode::InvalidArgument;
        return result;
    }
    result.value = LadyLuck::Vector3{{
        value[0] / magnitude,
        value[1] / magnitude,
        value[2] / magnitude}};
    if (!FiniteVector(result.value))
    {
        result.value = LadyLuck::Vector3{};
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return result;
}

LadyLuck::Result<LadyLuck::Vector3> RelativePosition(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Result<LadyLuck::Vector3> result{};
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    if (!CheckedVectorSubtract(
            frame.opponent.position_ned_m,
            frame.own.position_ned_m,
            result.value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return result;
}

LadyLuck::Vector3 Cross(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

LadyLuck::Result<double> HeadingRad(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Result<double> result{};
    const LadyLuck::Result<LadyLuck::Vector3> nose = HorizontalUnit(
        frame.own.nose_ned);
    if (!nose.ok())
    {
        result.status = nose.status;
        return result;
    }
    result.value = std::atan2(nose.value[1], nose.value[0]);
    if (!std::isfinite(result.value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return result;
}

LadyLuck::Result<std::int32_t> EntrySide(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Result<std::int32_t> result{};
    const LadyLuck::Result<LadyLuck::Vector3> nose = HorizontalUnit(
        frame.own.nose_ned);
    if (!nose.ok())
    {
        result.status = nose.status;
        return result;
    }
    const LadyLuck::Result<LadyLuck::Vector3> relative =
        RelativePosition(frame);
    if (!relative.ok())
    {
        result.status = relative.status;
        return result;
    }
    const LadyLuck::Result<LadyLuck::Vector3> line_of_sight =
        HorizontalUnit(relative.value);
    if (!line_of_sight.ok())
    {
        result.status = line_of_sight.status;
        return result;
    }
    const double cross_down =
        nose.value[0] * line_of_sight.value[1]
        - nose.value[1] * line_of_sight.value[0];
    result.value = cross_down >= 0.0 ? 1 : -1;
    return result;
}

double WrappedDelta(
    const double current_rad,
    const double previous_rad) noexcept
{
    const double difference = current_rad - previous_rad;
    return std::atan2(std::sin(difference), std::cos(difference));
}

double SpeedEvidenceBandMps(
    const double own_speed_mps,
    const double opponent_speed_mps) noexcept
{
    const double vector_quantum_mps =
        std::sqrt(3.0) * kBodyVelocityQuantumMps;
    const double own_error = vector_quantum_mps
        + std::fabs(own_speed_mps) * kPlaneInfoFloat32RelativeQuantum;
    const double opponent_error = vector_quantum_mps
        + std::fabs(opponent_speed_mps) * kPlaneInfoFloat32RelativeQuantum;
    return own_error + opponent_error;
}

LadyLuck::Status ValidatePositive(const double value) noexcept
{
    LadyLuck::Status status{};
    if (!std::isfinite(value))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    else if (value <= 0.0)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
    }
    return status;
}

LadyLuck::Status ValidateSpeedFloor(
    const LadyLuck::HabfmSpeedFloorSupply& speed_floor) noexcept
{
    if (!speed_floor.admitted)
    {
        return LadyLuck::Status{};
    }
    return ValidatePositive(speed_floor.floor_speed_mps);
}

LadyLuck::Status ValidateFrontalPass(
    const LadyLuck::HabfmFrontalPassSupply& frontal_pass) noexcept
{
    if (!frontal_pass.admitted)
    {
        return LadyLuck::Status{};
    }
    LadyLuck::Status status = ValidatePositive(frontal_pass.safe_abeam_m);
    if (!status.ok())
    {
        return status;
    }
    status = ValidatePositive(frontal_pass.compressed_abeam_m);
    if (!status.ok())
    {
        return status;
    }
    if (frontal_pass.compressed_abeam_m > frontal_pass.safe_abeam_m)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return status;
    }
    if (frontal_pass.side_sign != -1 && frontal_pass.side_sign != 1)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
    }
    return status;
}

LadyLuck::Status ValidateVerticalRoom(
    const LadyLuck::HabfmVerticalRoomReceipt& room) noexcept
{
    LadyLuck::Status status{};
    for (const double value : {
            room.vertical_offset_m,
            room.identity_depth_m.value,
            room.radius_clamp_m.value,
            room.room_angle_rad.value})
    {
        if (!std::isfinite(value))
        {
            status.code = LadyLuck::StatusCode::NonFiniteInput;
            return status;
        }
    }
    if (!room.admitted)
    {
        if (room.vertical_offset_m != 0.0
            || room.clamp_active_valid
            || room.room_angle_rad.has_value)
        {
            status.code = LadyLuck::StatusCode::InvalidArgument;
            return status;
        }
        if ((room.identity_depth_m.has_value
                && room.identity_depth_m.value <= 0.0)
            || (room.radius_clamp_m.has_value
                && room.radius_clamp_m.value <= 0.0))
        {
            status.code = LadyLuck::StatusCode::InvalidArgument;
        }
        return status;
    }
    if (room.vertical_offset_m == 0.0
        || !room.identity_depth_m.has_value
        || room.identity_depth_m.value <= 0.0
        || !room.radius_clamp_m.has_value
        || room.radius_clamp_m.value <= 0.0
        || !room.clamp_active_valid
        || !room.room_angle_rad.has_value
        || std::fabs(room.vertical_offset_m) > room.radius_clamp_m.value)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
    }
    return status;
}

LadyLuck::Status ValidateFarFleeAnchor(
    const LadyLuck::HabfmOptionalScalar& anchor_altitude_m) noexcept
{
    LadyLuck::Status status{};
    if (anchor_altitude_m.has_value
        && !std::isfinite(anchor_altitude_m.value))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return status;
}

LadyLuck::Status ValidateMergeIntentEvidence(
    const LadyLuck::HabfmMergeIntentEvidence& evidence) noexcept
{
    LadyLuck::Status status{};
    if (!std::isfinite(evidence.adversary_speed_mps)
        || !std::isfinite(evidence.speed_error_bound_mps)
        || (evidence.corner_speed_lower_mps.has_value
            && !std::isfinite(evidence.corner_speed_lower_mps.value))
        || (evidence.corner_speed_upper_mps.has_value
            && !std::isfinite(evidence.corner_speed_upper_mps.value)))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return status;
    }
    if (evidence.adversary_speed_mps < 0.0
        || evidence.speed_error_bound_mps < 0.0)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return status;
    }
    if (evidence.intent != LadyLuck::HabfmMergeIntentState::EnergyFightProven
        && evidence.intent != LadyLuck::HabfmMergeIntentState::NotProven)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return status;
    }
    for (const LadyLuck::HabfmOptionalScalar* bound : {
            &evidence.corner_speed_lower_mps,
            &evidence.corner_speed_upper_mps})
    {
        if (bound->has_value && bound->value <= 0.0)
        {
            status.code = LadyLuck::StatusCode::InvalidArgument;
            return status;
        }
    }
    if (evidence.evidence_admitted)
    {
        if (!evidence.corner_speed_lower_mps.has_value
            || !evidence.corner_speed_upper_mps.has_value
            || evidence.corner_speed_lower_mps.value
                > evidence.corner_speed_upper_mps.value)
        {
            status.code = LadyLuck::StatusCode::InvalidArgument;
            return status;
        }
    }
    if (evidence.intent == LadyLuck::HabfmMergeIntentState::EnergyFightProven
        && !evidence.evidence_admitted)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
    }
    return status;
}

bool CheckedHorizontalAimPoint(
    const LadyLuck::Vector3& origin,
    const double distance_m,
    const LadyLuck::Vector3& direction,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    double north_offset_m = 0.0;
    double east_offset_m = 0.0;
    if (!FiniteVector(origin)
        || !FiniteVector(direction)
        || !CheckedMultiply(distance_m, direction[0], north_offset_m)
        || !CheckedMultiply(distance_m, direction[1], east_offset_m)
        || !CheckedAdd(origin[0], north_offset_m, output[0])
        || !CheckedAdd(origin[1], east_offset_m, output[1]))
    {
        output = LadyLuck::Vector3{};
        return false;
    }
    output[2] = origin[2];
    return true;
}

bool CheckedAimPoint3(
    const LadyLuck::Vector3& origin,
    const double distance_m,
    const LadyLuck::Vector3& direction,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    if (!FiniteVector(origin)
        || !FiniteVector(direction)
        || !std::isfinite(distance_m))
    {
        return false;
    }
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        double offset_m = 0.0;
        if (!CheckedMultiply(distance_m, direction[index], offset_m)
            || !CheckedAdd(origin[index], offset_m, output[index]))
        {
            output = LadyLuck::Vector3{};
            return false;
        }
    }
    return true;
}

bool ThreeDimensionalMergeRequired(
    const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    LadyLuck::Vector3 relative{};
    if (!CheckedVectorSubtract(
            frame.opponent.position_ned_m,
            frame.own.position_ned_m,
            relative))
    {
        return false;
    }
    const double horizontal_speed = std::hypot(
        frame.own.velocity_ned_mps[0], frame.own.velocity_ned_mps[1]);
    const double horizontal_nose = std::hypot(
        frame.own.nose_ned[0], frame.own.nose_ned[1]);
    const double horizontal_los = std::hypot(relative[0], relative[1]);
    const double velocity_resolution_mps =
        std::sqrt(3.0) * kBodyVelocityQuantumMps;
    return std::isfinite(horizontal_speed)
        && std::isfinite(horizontal_nose)
        && std::isfinite(horizontal_los)
        && (horizontal_speed <= velocity_resolution_mps
            || horizontal_nose < LadyLuck::constants::Tiny
            || horizontal_los < LadyLuck::constants::Tiny);
}

void BuildHabfmThreeDimensionalMergeIntentInternal(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::HabfmSpeedFloorSupply& speed_floor,
    const LadyLuck::HabfmVerticalRoomReceipt& vertical_room,
    const LadyLuck::HabfmOptionalScalar& far_flee_anchor_altitude_m,
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status) noexcept
{
    output.Clear();
    status = ValidateSpeedFloor(speed_floor);
    if (!status.ok())
    {
        return;
    }
    status = ValidateFarFleeAnchor(far_flee_anchor_altitude_m);
    if (!status.ok())
    {
        return;
    }
    status = ValidateVerticalRoom(vertical_room);
    if (!status.ok())
    {
        return;
    }
    const LadyLuck::Result<LadyLuck::Vector3> relative =
        RelativePosition(frame);
    if (!relative.ok())
    {
        status = relative.status;
        return;
    }
    LadyLuck::Result<LadyLuck::Vector3> nose =
        Unit3(frame.own.nose_ned);
    if (!nose.ok()
        && nose.status.code == LadyLuck::StatusCode::InvalidArgument)
    {
        // A finite zero nose vector does not remove the measured flight-path
        // direction. Use velocity as the same-frame directional observation.
        nose = Unit3(frame.own.velocity_ned_mps);
    }
    if (!nose.ok())
    {
        status = nose.status;
        return;
    }
    (void)nose;
    LadyLuck::Result<LadyLuck::Vector3> direction = Unit3(relative.value);
    if (!direction.ok())
    {
        // A near-coincident finite position has no resolved LOS.  The current
        // measured nose was validated above and remains an observable
        // guidance direction even when the low-speed velocity course does not.
        direction = nose;
    }
    double speed_mps = 0.0;
    if (!direction.ok())
    {
        status = direction.status;
        return;
    }
    if (!CheckedNorm3(frame.own.velocity_ned_mps, speed_mps))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    double measured_separation_m = 0.0;
    if (!CheckedNorm3(relative.value, measured_separation_m))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    status = ValidatePositive(speed_mps);
    if (!status.ok())
    {
        return;
    }
    const double guidance_range_m =
        std::isfinite(frame.own_offense.range_m)
            && frame.own_offense.range_m > LadyLuck::constants::Tiny
        ? frame.own_offense.range_m
        : (std::max)(
            measured_separation_m,
            LadyLuck::constants::Tiny);
    const double capture_range_m =
        std::isfinite(frame.own_offense.phase.max_range_m)
            && frame.own_offense.phase.max_range_m
                > LadyLuck::constants::Tiny
        ? frame.own_offense.phase.max_range_m
        : guidance_range_m;

    LadyLuck::Vector3 aim_point{};
    if (!CheckedAimPoint3(
            frame.own.position_ned_m,
            guidance_range_m,
            direction.value,
            aim_point))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (far_flee_anchor_altitude_m.has_value)
    {
        aim_point[2] = -far_flee_anchor_altitude_m.value;
    }
    else if (vertical_room.admitted)
    {
        aim_point[2] = frame.own.position_ned_m[2]
            - vertical_room.vertical_offset_m;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = aim_point;
    output.desired_speed_mps = speed_floor.admitted
        ? (std::max)(speed_mps, speed_floor.floor_speed_mps)
        : speed_mps;
    output.capture_range_des_m = capture_range_m;
    output.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    output.behavior_id = LadyLuck::DoctrineBehaviorId::HabfmMergeApproach;
    output.mode_id = LadyLuck::DoctrineModeId::Habfm;
    output.writer_id = LadyLuck::ControlIntentWriterHabfm;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

LadyLuck::Result<LadyLuck::Vector3> FrontalAimPoint(
    const LadyLuck::DogfightGeometryFrame& frame,
    const double abeam_m,
    const std::int32_t side_sign) noexcept
{
    LadyLuck::Result<LadyLuck::Vector3> result{};
    const LadyLuck::Result<LadyLuck::Vector3> relative =
        RelativePosition(frame);
    if (!relative.ok())
    {
        result.status = relative.status;
        return result;
    }
    const LadyLuck::Result<LadyLuck::Vector3> line_of_sight =
        HorizontalUnit(relative.value);
    if (!line_of_sight.ok())
    {
        result.status = line_of_sight.status;
        return result;
    }
    const LadyLuck::Vector3 lateral{{
        -line_of_sight.value[1],
        line_of_sight.value[0],
        0.0}};
    double signed_abeam_m = 0.0;
    if (!CheckedMultiply(
            static_cast<double>(side_sign), abeam_m, signed_abeam_m)
        || !CheckedHorizontalAimPoint(
            frame.opponent.position_ned_m,
            signed_abeam_m,
            lateral,
            result.value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    }
    return result;
}

LadyLuck::HabfmActiveBranch BranchForProfile(
    const LadyLuck::HabfmCircleProfile profile) noexcept
{
    return profile == LadyLuck::HabfmCircleProfile::OneCircle
        ? LadyLuck::HabfmActiveBranch::OneCircle
        : LadyLuck::HabfmActiveBranch::TwoCircle;
}
}

namespace LadyLuck
{

void BuildHabfmThreeDimensionalMergeIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m,
    ControlIntent& output,
    Status& status) noexcept
{
    BuildHabfmThreeDimensionalMergeIntentInternal(
        frame,
        speed_floor,
        vertical_room,
        far_flee_anchor_altitude_m,
        output,
        status);
}

void EvaluateHabfmCommandGeometry(
    const DogfightGeometryFrame& frame,
    HabfmCommandGeometryReceipt& output,
    Status& status) noexcept
{
    output = HabfmCommandGeometryReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.own.nose_ned)
        || !std::isfinite(frame.own_offense.phase.max_range_m)
        || !std::isfinite(frame.enemy_offense.phase.max_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const Vector3 relative{{
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0],
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1],
        frame.opponent.position_ned_m[2] - frame.own.position_ned_m[2]}};
    if (!FiniteVector(relative))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.own_horizontal_speed_mps = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    output.own_speed_mps = std::hypot(
        output.own_horizontal_speed_mps,
        frame.own.velocity_ned_mps[2]);
    output.own_horizontal_nose_norm = std::hypot(
        frame.own.nose_ned[0],
        frame.own.nose_ned[1]);
    output.horizontal_line_of_sight_m = std::hypot(
        relative[0],
        relative[1]);
    if (!std::isfinite(output.own_speed_mps)
        || !std::isfinite(output.own_horizontal_speed_mps)
        || !std::isfinite(output.own_horizontal_nose_norm)
        || !std::isfinite(output.horizontal_line_of_sight_m))
    {
        output = HabfmCommandGeometryReceipt{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.valid = true;
    output.three_dimensional_merge_required =
        ThreeDimensionalMergeRequired(frame);
    if (output.own_speed_mps <= 0.0)
    {
        output.reason = HabfmCommandGeometryReason::OwnSpeedUnobservable;
        return;
    }
    double own_nose_norm = 0.0;
    if (!CheckedNorm3(frame.own.nose_ned, own_nose_norm))
    {
        output = HabfmCommandGeometryReceipt{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    // A finite live frame with observable velocity remains commandable even
    // when the reported nose vector is momentarily zero. The 3-D merge path
    // consumes velocity as its deterministic directional fallback.
    output.available = true;
    output.reason = HabfmCommandGeometryReason::Available;
}

void EvaluateHabfmFarFleeApproach(
    const DogfightGeometryFrame& frame,
    const bool feature_enabled,
    const HabfmFarFleeApproachState& previous_state,
    HabfmFarFleeApproachReceipt& output,
    Status& status) noexcept
{
    output = HabfmFarFleeApproachReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.valid = true;
    output.resolved_state = previous_state;
    if (!feature_enabled)
    {
        output.reason = HabfmFarFleeApproachReason::FeatureDisabled;
        return;
    }

    // A feature-enabled evaluation is fail-closed.  This default also removes
    // a stale/incomplete latch if any of the official evidence is unavailable.
    output.resolved_state = HabfmFarFleeApproachState{};
    output.reason = HabfmFarFleeApproachReason::EvidenceUnavailable;
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !std::isfinite(frame.own_offense.phase.max_range_m)
        || !std::isfinite(frame.enemy_offense.phase.max_range_m))
    {
        return;
    }

    Vector3 relative_position{};
    double separation_m = 0.0;
    if (!CheckedVectorSubtract(
            frame.opponent.position_ned_m,
            frame.own.position_ned_m,
            relative_position)
        || !CheckedNorm3(relative_position, separation_m))
    {
        return;
    }

    const double official_reach_boundary_m = (std::max)(
        frame.own_offense.phase.max_range_m,
        frame.enemy_offense.phase.max_range_m);
    output.separation_m = separation_m;
    output.official_reach_boundary_m = official_reach_boundary_m;
    if (separation_m <= 0.0
        || official_reach_boundary_m <= 0.0)
    {
        return;
    }

    output.evidence_available = true;
    if (separation_m <= official_reach_boundary_m)
    {
        output.released = previous_state.latched;
        output.reason =
            HabfmFarFleeApproachReason::InsideOrAtOfficialReach;
        return;
    }

    if (previous_state.latched)
    {
        if (!previous_state.anchor_altitude_m.has_value
            || !std::isfinite(previous_state.anchor_altitude_m.value))
        {
            output.evidence_available = false;
            return;
        }
        output.selected = true;
        output.resolved_state = previous_state;
        output.reason = HabfmFarFleeApproachReason::OutsideLatchHeld;
        return;
    }

    Vector3 relative_velocity{};
    double range_rate_mps = 0.0;
    if (!FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.velocity_ned_mps)
        || !CheckedVectorSubtract(
            frame.opponent.velocity_ned_mps,
            frame.own.velocity_ned_mps,
            relative_velocity)
        || !CheckedRangeRate(
            relative_position,
            separation_m,
            relative_velocity,
            range_rate_mps))
    {
        output.evidence_available = false;
        return;
    }
    output.range_rate_mps.has_value = true;
    output.range_rate_mps.value = range_rate_mps;
    if (range_rate_mps <= 0.0)
    {
        output.reason = HabfmFarFleeApproachReason::OutsideButNotOpening;
        return;
    }

    output.selected = true;
    output.armed = true;
    output.resolved_state.latched = true;
    output.resolved_state.anchor_altitude_m.has_value = true;
    output.resolved_state.anchor_altitude_m.value =
        -frame.own.position_ned_m[2];
    output.reason = HabfmFarFleeApproachReason::OutsideOpeningArmed;
}

Result<HabfmMergeProfileSelection> SelectHabfmMergeProfile(
    const DogfightGeometryFrame& frame) noexcept
{
    Result<HabfmMergeProfileSelection> result{};
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const double own_altitude_m = -frame.own.position_ned_m[2];
    const double opponent_altitude_m = -frame.opponent.position_ned_m[2];
    const double own_speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    const double opponent_speed_mps = VectorNorm(
        frame.opponent.velocity_ned_mps);
    if (!std::isfinite(own_speed_mps) || !std::isfinite(opponent_speed_mps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const Result<double> own_specific_energy = SpecificEnergyM(
        own_altitude_m, own_speed_mps);
    if (!own_specific_energy.ok())
    {
        result.status = own_specific_energy.status;
        return result;
    }
    const Result<double> opponent_specific_energy = SpecificEnergyM(
        opponent_altitude_m, opponent_speed_mps);
    if (!opponent_specific_energy.ok())
    {
        result.status = opponent_specific_energy.status;
        return result;
    }
    const Result<double> energy_band = EnergyEvidenceBandM(
        own_altitude_m,
        own_speed_mps,
        opponent_altitude_m,
        opponent_speed_mps);
    if (!energy_band.ok())
    {
        result.status = energy_band.status;
        return result;
    }

    HabfmMergeProfileSelection selection{};
    selection.delta_specific_energy_m =
        own_specific_energy.value - opponent_specific_energy.value;
    selection.evidence_band_m = energy_band.value;
    selection.delta_speed_mps = own_speed_mps - opponent_speed_mps;
    selection.speed_band_mps = SpeedEvidenceBandMps(
        own_speed_mps, opponent_speed_mps);
    if (selection.delta_specific_energy_m > selection.evidence_band_m)
    {
        selection.profile = HabfmCircleProfile::TwoCircle;
        selection.energy_state = HabfmMergeEnergyState::AdvantageProven;
    }
    else if (selection.delta_speed_mps < -selection.speed_band_mps)
    {
        selection.profile = HabfmCircleProfile::OneCircle;
        selection.energy_state = HabfmMergeEnergyState::DeficitProven;
    }
    else
    {
        selection.profile = HabfmCircleProfile::TwoCircle;
        selection.energy_state = HabfmMergeEnergyState::Indistinguishable;
    }
    result.value = selection;
    return result;
}

Result<HabfmLeadTurnEvidence> EvaluateHabfmLeadTurn(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& n_max_g,
    const bool n_max_admitted) noexcept
{
    Result<HabfmLeadTurnEvidence> result{};
    const Result<Vector3> relative_position = RelativePosition(frame);
    if (!relative_position.ok())
    {
        result.status = relative_position.status;
        return result;
    }
    if (!FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.velocity_ned_mps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const Vector3 relative_velocity{{
        frame.opponent.velocity_ned_mps[0] - frame.own.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1] - frame.own.velocity_ned_mps[1],
        frame.opponent.velocity_ned_mps[2] - frame.own.velocity_ned_mps[2]}};
    const double range_m = VectorNorm(relative_position.value);
    if (!std::isfinite(range_m))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    if (range_m <= 0.0)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    const double own_speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    if (!std::isfinite(own_speed_mps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const Vector3 cross = Cross(relative_position.value, relative_velocity);
    const double los_rate_radps = VectorNorm(cross) / (range_m * range_m);
    const double range_rate_mps =
        Dot(relative_position.value, relative_velocity) / range_m;
    if (!std::isfinite(los_rate_radps) || !std::isfinite(range_rate_mps))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    HabfmLeadTurnEvidence evidence{};
    evidence.los_rate_radps = los_rate_radps;
    evidence.range_rate_mps = range_rate_mps;
    evidence.closing = range_rate_mps < 0.0;
    evidence.evidence_admitted = n_max_admitted
        && n_max_g.has_value
        && std::isfinite(n_max_g.value)
        && n_max_g.value > 1.0
        && own_speed_mps > 0.0;
    if (evidence.evidence_admitted)
    {
        const double turn_radius_m = own_speed_mps * own_speed_mps
            / (constants::StandardGravityMps2
                * std::sqrt(n_max_g.value * n_max_g.value - 1.0));
        const double available_turn_rate_radps = own_speed_mps / turn_radius_m;
        if (!std::isfinite(available_turn_rate_radps)
            || available_turn_rate_radps <= 0.0)
        {
            result.status.code = StatusCode::InvalidConfiguration;
            return result;
        }
        evidence.available_turn_rate_radps.has_value = true;
        evidence.available_turn_rate_radps.value = available_turn_rate_radps;
    }

    if (!evidence.closing)
    {
        evidence.initiate = true;
        evidence.reason = HabfmLeadTurnReason::OpeningGeometryNoPendingMerge;
    }
    else if (!evidence.evidence_admitted)
    {
        // A missing turn-capability observation is not a lead-turn event.
        // Preserve the merge approach until sigma_dot >= omega_available is
        // actually observable; otherwise the measured current geometry can
        // manufacture an immediate turn before the event exists.
        evidence.initiate = false;
        evidence.reason =
            HabfmLeadTurnReason::TurnRateEvidenceAbsentHoldApproach;
    }
    else if (evidence.los_rate_radps
        >= evidence.available_turn_rate_radps.value)
    {
        evidence.initiate = true;
        evidence.reason = HabfmLeadTurnReason::LosRateReachedAvailableTurnRate;
    }
    else
    {
        evidence.initiate = false;
        evidence.reason =
            HabfmLeadTurnReason::LosRateBelowAvailableTurnRateHoldApproach;
    }
    result.value = evidence;
    return result;
}

void BuildHabfmMergeApproachIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    ControlIntent& output,
    Status& status) noexcept
{
    BuildHabfmMergeApproachIntent(
        frame,
        speed_floor,
        compress_separation_aim,
        frontal_pass,
        vertical_room,
        HabfmOptionalScalar{},
        output,
        status);
}

void BuildHabfmMergeApproachIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = ValidateSpeedFloor(speed_floor);
    if (!status.ok())
    {
        return;
    }
    status = ValidateFarFleeAnchor(far_flee_anchor_altitude_m);
    if (!status.ok())
    {
        return;
    }
    status = ValidateFrontalPass(frontal_pass);
    if (!status.ok())
    {
        return;
    }
    status = ValidateVerticalRoom(vertical_room);
    if (!status.ok())
    {
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Result<Vector3> course = HorizontalUnit(frame.own.velocity_ned_mps);
    if (!course.ok())
    {
        status = course.status;
        return;
    }
    const double speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    for (const double value : {
            frame.own_offense.range_m,
            frame.own_offense.phase.max_range_m,
            speed_mps})
    {
        status = ValidatePositive(value);
        if (!status.ok())
        {
            return;
        }
    }

    Vector3 aim_point{};
    if (frontal_pass.admitted)
    {
        const double abeam_m = compress_separation_aim
            ? frontal_pass.compressed_abeam_m
            : frontal_pass.safe_abeam_m;
        const Result<Vector3> frontal_aim = FrontalAimPoint(
            frame, abeam_m, frontal_pass.side_sign);
        if (!frontal_aim.ok())
        {
            status = frontal_aim.status;
            return;
        }
        aim_point = frontal_aim.value;
    }
    else if (compress_separation_aim)
    {
        const Result<Vector3> relative = RelativePosition(frame);
        if (!relative.ok())
        {
            status = relative.status;
            return;
        }
        const Result<Vector3> line_of_sight = HorizontalUnit(relative.value);
        if (!line_of_sight.ok())
        {
            status = line_of_sight.status;
            return;
        }
        if (!CheckedHorizontalAimPoint(
                frame.own.position_ned_m,
                frame.own_offense.range_m,
                line_of_sight.value,
                aim_point))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    else
    {
        if (!CheckedHorizontalAimPoint(
                frame.own.position_ned_m,
                frame.own_offense.range_m,
                course.value,
                aim_point))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    aim_point[2] = frame.own.position_ned_m[2];
    if (far_flee_anchor_altitude_m.has_value)
    {
        aim_point[2] = -far_flee_anchor_altitude_m.value;
    }
    else if (vertical_room.admitted)
    {
        aim_point[2] = frame.own.position_ned_m[2]
            - vertical_room.vertical_offset_m;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = aim_point;
    output.desired_speed_mps = speed_floor.admitted
        ? (std::max)(speed_mps, speed_floor.floor_speed_mps)
        : speed_mps;
    output.capture_range_des_m = frame.own_offense.phase.max_range_m;
    output.route_kind = ControlRouteKind::AimPoint;
    output.behavior_id = DoctrineBehaviorId::HabfmMergeApproach;
    output.mode_id = DoctrineModeId::Habfm;
    output.writer_id = ControlIntentWriterHabfm;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

void BuildHabfmEnergyFightIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmFrontalPassSupply& frontal_pass,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = ValidateSpeedFloor(speed_floor);
    if (!status.ok())
    {
        return;
    }
    status = ValidateFrontalPass(frontal_pass);
    if (!status.ok())
    {
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Result<Vector3> relative = RelativePosition(frame);
    if (!relative.ok())
    {
        status = relative.status;
        return;
    }
    const Result<Vector3> line_of_sight = HorizontalUnit(relative.value);
    if (!line_of_sight.ok())
    {
        status = line_of_sight.status;
        return;
    }
    const double speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    for (const double value : {
            frame.own_offense.range_m,
            frame.own_offense.phase.max_range_m,
            speed_mps})
    {
        status = ValidatePositive(value);
        if (!status.ok())
        {
            return;
        }
    }

    Vector3 aim_point{};
    if (frontal_pass.admitted)
    {
        const Result<Vector3> frontal_aim = FrontalAimPoint(
            frame, frontal_pass.safe_abeam_m, frontal_pass.side_sign);
        if (!frontal_aim.ok())
        {
            status = frontal_aim.status;
            return;
        }
        aim_point = frontal_aim.value;
    }
    else
    {
        aim_point = Vector3{{
            frame.own.position_ned_m[0]
                + frame.own_offense.range_m * line_of_sight.value[0],
            frame.own.position_ned_m[1]
                + frame.own_offense.range_m * line_of_sight.value[1],
            frame.own.position_ned_m[2]}};
    }
    aim_point[2] = frame.own.position_ned_m[2];

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = aim_point;
    output.desired_speed_mps = speed_floor.admitted
        ? (std::max)(speed_mps, speed_floor.floor_speed_mps)
        : speed_mps;
    output.capture_range_des_m = frame.own_offense.phase.max_range_m;
    output.route_kind = ControlRouteKind::AimPoint;
    output.behavior_id = DoctrineBehaviorId::HabfmEnergyFight;
    output.mode_id = DoctrineModeId::Habfm;
    output.writer_id = ControlIntentWriterHabfm;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

void BuildHabfmTwoCircleIntent(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    if (side_sign != -1 && side_sign != 1)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Result<Vector3> nose = HorizontalUnit(frame.own.nose_ned);
    if (!nose.ok())
    {
        status = nose.status;
        return;
    }
    const Vector3 lateral{{
        static_cast<double>(side_sign) * -nose.value[1],
        static_cast<double>(side_sign) * nose.value[0],
        0.0}};
    const Vector3 bisector{{
        nose.value[0] + lateral[0],
        nose.value[1] + lateral[1],
        nose.value[2] + lateral[2]}};
    const Result<Vector3> direction = HorizontalUnit(bisector);
    if (!direction.ok())
    {
        status = direction.status;
        return;
    }
    const double speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    for (const double value : {
            frame.own_offense.range_m,
            frame.own_offense.phase.max_range_m,
            speed_mps})
    {
        status = ValidatePositive(value);
        if (!status.ok())
        {
            return;
        }
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = Vector3{{
        frame.own.position_ned_m[0]
            + frame.own_offense.range_m * direction.value[0],
        frame.own.position_ned_m[1]
            + frame.own_offense.range_m * direction.value[1],
        frame.own.position_ned_m[2]}};
    output.desired_speed_mps = speed_mps;
    output.capture_range_des_m = frame.own_offense.phase.max_range_m;
    output.route_kind = ControlRouteKind::AimPoint;
    output.behavior_id = DoctrineBehaviorId::HabfmTwoCircle;
    output.mode_id = DoctrineModeId::Habfm;
    output.writer_id = ControlIntentWriterHabfm;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

HabfmActiveControlCore::HabfmActiveControlCore() noexcept
{
    ResetEpisode();
}

void HabfmActiveControlCore::ResetLegState() noexcept
{
    active_branch_ = HabfmActiveBranch::None;
    leg_selection_valid_ = false;
    leg_selection_ = HabfmMergeProfileSelection{};
    side_sign_valid_ = false;
    side_sign_ = 0;
    previous_heading_valid_ = false;
    previous_heading_rad_ = 0.0;
    progress_rad_ = 0.0;
}

void HabfmActiveControlCore::ResetEpisode() noexcept
{
    ResetLegState();
    guard_latched_profile_valid_ = false;
    guard_latched_profile_ = HabfmCircleProfile::TwoCircle;
    previous_closing_valid_ = false;
    previous_closing_ = false;
    far_flee_approach_state_ = HabfmFarFleeApproachState{};
}

void HabfmActiveControlCore::ResetLeg() noexcept
{
    ResetLegState();
}

HabfmActiveControlCoreSnapshot HabfmActiveControlCore::Snapshot() const noexcept
{
    HabfmActiveControlCoreSnapshot snapshot{};
    snapshot.active_branch = active_branch_;
    snapshot.leg_selection.has_value = leg_selection_valid_;
    snapshot.leg_selection.value = leg_selection_;
    snapshot.guard_latched_profile.has_value = guard_latched_profile_valid_;
    snapshot.guard_latched_profile.value = guard_latched_profile_;
    snapshot.side_sign.has_value = side_sign_valid_;
    snapshot.side_sign.value = side_sign_;
    snapshot.previous_heading_rad.has_value = previous_heading_valid_;
    snapshot.previous_heading_rad.value = previous_heading_rad_;
    snapshot.progress_rad = progress_rad_;
    snapshot.previous_closing.has_value = previous_closing_valid_;
    snapshot.previous_closing.value = previous_closing_;
    snapshot.far_flee_approach = far_flee_approach_state_;
    return snapshot;
}

void HabfmActiveControlCore::StepControlIntent(
    const DogfightGeometryFrame& frame,
    const HabfmActiveCoreInputs& inputs,
    HabfmActiveControlOutput& output,
    Status& status,
    const std::uint64_t blackboard_neutral_cue_streak) noexcept
{
    output = HabfmActiveControlOutput{};
    output.neutral_cue_streak = blackboard_neutral_cue_streak;
    status = ValidateMergeIntentEvidence(inputs.merge_intent);
    if (!status.ok())
    {
        ResetLegState();
        return;
    }

    EvaluateHabfmFarFleeApproach(
        frame,
        inputs.far_flee_approach_enabled,
        far_flee_approach_state_,
        output.far_flee_approach,
        status);
    if (!status.ok())
    {
        ResetLegState();
        output = HabfmActiveControlOutput{};
        return;
    }

    if (ThreeDimensionalMergeRequired(frame))
    {
        // Horizontal course/side is singular, but the current finite LOS and
        // flight-path vectors still define a complete 3-D merge reference.
        // Keep this in HABFM command ownership; do not revive HorizontalHold.
        ResetLegState();
        BuildHabfmThreeDimensionalMergeIntentInternal(
            frame,
            inputs.merge_speed_floor,
            inputs.merge_vertical_room,
            output.far_flee_approach.selected
                ? output.far_flee_approach.resolved_state.anchor_altitude_m
                : HabfmOptionalScalar{},
            output.intent,
            status);
        if (!status.ok())
        {
            output = HabfmActiveControlOutput{};
            return;
        }
        active_branch_ = HabfmActiveBranch::MergeApproach;
        far_flee_approach_state_ =
            output.far_flee_approach.resolved_state;
        output.branch = active_branch_;
        output.intent_present = true;
        return;
    }

    const Result<HabfmLeadTurnEvidence> lead = EvaluateHabfmLeadTurn(
        frame,
        inputs.capability_n_max_g,
        inputs.capability_n_max_admitted);
    if (!lead.ok())
    {
        status = lead.status;
        ResetLegState();
        return;
    }
    output.lead_turn = lead.value;

    if (lead.value.hold_approach()
        || output.far_flee_approach.selected)
    {
        if (active_branch_ == HabfmActiveBranch::OneCircle
            || active_branch_ == HabfmActiveBranch::TwoCircle)
        {
            ResetLegState();
        }
        BuildHabfmMergeApproachIntent(
            frame,
            inputs.merge_speed_floor,
            (inputs.merge_separation_policy.admitted
                    && inputs.merge_separation_policy.compress)
                || output.far_flee_approach.selected,
            inputs.frontal_pass,
            inputs.merge_vertical_room,
            output.far_flee_approach.selected
                ? output.far_flee_approach.resolved_state.anchor_altitude_m
                : HabfmOptionalScalar{},
            output.intent,
            status);
        if (!status.ok())
        {
            ResetLegState();
            output = HabfmActiveControlOutput{};
            return;
        }
        active_branch_ = HabfmActiveBranch::MergeApproach;
        far_flee_approach_state_ =
            output.far_flee_approach.resolved_state;
        output.branch = active_branch_;
        output.intent_present = true;
        return;
    }

    const bool energy_fight_selected =
        inputs.merge_intent.intent == HabfmMergeIntentState::EnergyFightProven
        && std::isfinite(frame.closing_speed_mps)
        && frame.closing_speed_mps > 0.0;
    if (energy_fight_selected)
    {
        if (active_branch_ == HabfmActiveBranch::OneCircle
            || active_branch_ == HabfmActiveBranch::TwoCircle)
        {
            ResetLegState();
        }
        BuildHabfmEnergyFightIntent(
            frame,
            inputs.merge_speed_floor,
            inputs.frontal_pass,
            output.intent,
            status);
        if (!status.ok())
        {
            ResetLegState();
            output = HabfmActiveControlOutput{};
            return;
        }
        active_branch_ = HabfmActiveBranch::EnergyFight;
        far_flee_approach_state_ =
            output.far_flee_approach.resolved_state;
        output.branch = active_branch_;
        output.intent_present = true;
        return;
    }

    if (active_branch_ != HabfmActiveBranch::OneCircle
        && active_branch_ != HabfmActiveBranch::TwoCircle)
    {
        ResetLegState();
    }
    if (!leg_selection_valid_)
    {
        const Result<HabfmMergeProfileSelection> proposed_result =
            SelectHabfmMergeProfile(frame);
        if (!proposed_result.ok())
        {
            status = proposed_result.status;
            ResetLegState();
            return;
        }
        HabfmMergeProfileSelection proposed = proposed_result.value;
        if (guard_latched_profile_valid_
            && guard_latched_profile_ != proposed.profile)
        {
            HabfmSelectorTransitionGuardReceipt guard{};
            guard.proposed_profile = proposed.profile;
            guard.latched_profile = guard_latched_profile_;
            guard.energy_advantage_proven =
                proposed.delta_specific_energy_m > proposed.evidence_band_m;
            guard.speed_deficit_proven =
                proposed.delta_speed_mps < -proposed.speed_band_mps;
            if (proposed.profile == HabfmCircleProfile::TwoCircle)
            {
                guard.transition_allowed = guard.energy_advantage_proven
                    && !guard.speed_deficit_proven;
            }
            else
            {
                guard.transition_allowed = guard.speed_deficit_proven
                    && !guard.energy_advantage_proven;
            }
            if (guard.transition_allowed)
            {
                guard.reason =
                    HabfmSelectorTransitionReason::SingleUncontradictedAxis;
                guard.resolved_profile = proposed.profile;
            }
            else
            {
                guard.reason = guard.energy_advantage_proven
                        && guard.speed_deficit_proven
                    ? HabfmSelectorTransitionReason::ConflictLatchHeld
                    : HabfmSelectorTransitionReason::UnprovenLatchHeld;
                guard.resolved_profile = guard_latched_profile_;
                proposed.profile = guard_latched_profile_;
            }
            output.transition_guard.has_value = true;
            output.transition_guard.value = guard;
        }
        leg_selection_ = proposed;
        leg_selection_valid_ = true;
        guard_latched_profile_ = proposed.profile;
        guard_latched_profile_valid_ = true;
    }
    output.profile_selection.has_value = true;
    output.profile_selection.value = leg_selection_;
    output.selected_profile.has_value = true;
    output.selected_profile.value = leg_selection_.profile;

    if (!side_sign_valid_)
    {
        const Result<std::int32_t> entry_side = EntrySide(frame);
        if (!entry_side.ok())
        {
            status = entry_side.status;
            ResetLegState();
            return;
        }
        const Result<double> heading = HeadingRad(frame);
        if (!heading.ok())
        {
            status = heading.status;
            ResetLegState();
            return;
        }
        side_sign_ = leg_selection_.profile == HabfmCircleProfile::OneCircle
            ? entry_side.value
            : -entry_side.value;
        side_sign_valid_ = true;
        previous_heading_rad_ = heading.value;
        previous_heading_valid_ = true;
        progress_rad_ = 0.0;
    }
    if (!previous_heading_valid_)
    {
        status.code = StatusCode::InvalidConfiguration;
        ResetLegState();
        return;
    }
    const Result<double> heading = HeadingRad(frame);
    if (!heading.ok())
    {
        status = heading.status;
        ResetLegState();
        return;
    }
    progress_rad_ += static_cast<double>(side_sign_)
        * WrappedDelta(heading.value, previous_heading_rad_);
    previous_heading_rad_ = heading.value;

    const Result<HabfmCheckpointCueEvidence> checkpoint =
        EvaluateCheckpointCue(frame);
    if (!checkpoint.ok())
    {
        status = checkpoint.status;
        ResetLegState();
        return;
    }
    output.checkpoint_cue.has_value = true;
    output.checkpoint_cue.value = checkpoint.value;
    if (!std::isfinite(frame.closing_speed_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        ResetLegState();
        return;
    }
    const bool merge_pass = previous_closing_valid_
        && previous_closing_
        && frame.closing_speed_mps < 0.0;
    if (frame.closing_speed_mps != 0.0)
    {
        previous_closing_valid_ = true;
        previous_closing_ = frame.closing_speed_mps > 0.0;
    }

    output.branch = BranchForProfile(leg_selection_.profile);
    output.turn_side_sign.has_value = true;
    output.turn_side_sign.value = side_sign_;
    output.turn_progress_rad = progress_rad_;
    if (merge_pass)
    {
        if (checkpoint.value.cue == HabfmCheckpointCueState::Neutral)
        {
            if (output.neutral_cue_streak
                == (std::numeric_limits<std::uint64_t>::max)())
            {
                status.code = StatusCode::InvalidConfiguration;
                ResetLegState();
                return;
            }
            ++output.neutral_cue_streak;
        }
        else
        {
            output.neutral_cue_streak = 0U;
        }
        output.leg_status = HabfmActiveCoreLegStatus::MergePass;
        output.merge_pass = true;
        output.mode_recheck = true;
        active_branch_ = output.branch;
        far_flee_approach_state_ =
            output.far_flee_approach.resolved_state;
        ResetLegState();
        return;
    }

    if (leg_selection_.profile == HabfmCircleProfile::OneCircle)
    {
        BuildHabfmFixedOneCircleIntent(
            frame, side_sign_, output.intent, status);
    }
    else
    {
        BuildHabfmTwoCircleIntent(
            frame, side_sign_, output.intent, status);
    }
    if (!status.ok())
    {
        ResetLegState();
        output = HabfmActiveControlOutput{};
        return;
    }
    active_branch_ = output.branch;
    far_flee_approach_state_ = output.far_flee_approach.resolved_state;
    output.intent_present = true;
}
}
