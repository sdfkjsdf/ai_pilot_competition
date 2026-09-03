#include "LadyLuck/maneuver/HabfmFixedOneCircleControlIntent.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
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
}

namespace LadyLuck
{
void BuildHabfmFixedOneCircleReference(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign,
    HabfmFixedOneCircleReference& output,
    Status& status) noexcept
{
    output = HabfmFixedOneCircleReference{};
    status = Status{};
    if (side_sign != -1 && side_sign != 1)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_position) || !FiniteVector(own_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Result<Vector3> own_nose = HorizontalUnit(frame.own.nose_ned);
    if (!own_nose.ok())
    {
        status = own_nose.status;
        return;
    }
    const Vector3 lateral{{
        static_cast<double>(side_sign) * -own_nose.value[1],
        static_cast<double>(side_sign) * own_nose.value[0],
        0.0}};

    const double range_m = frame.own_offense.range_m;
    const double capture_range_m = frame.own_offense.phase.max_range_m;
    const double speed_mps = VectorNorm(own_velocity);
    for (const double value : {range_m, capture_range_m, speed_mps})
    {
        if (!std::isfinite(value))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        if (value <= 0.0)
        {
            status.code = StatusCode::InvalidArgument;
            return;
        }
    }

    output.aim_point_m = Vector3{{
        own_position[0] + range_m * lateral[0],
        own_position[1] + range_m * lateral[1],
        own_position[2]}};
    output.desired_speed_mps = speed_mps;
    output.capture_range_des_m = capture_range_m;
}

void BuildHabfmFixedOneCircleIntent(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};

    HabfmFixedOneCircleReference reference{};
    BuildHabfmFixedOneCircleReference(
        frame,
        side_sign,
        reference,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = reference.aim_point_m;
    output.desired_speed_mps = reference.desired_speed_mps;
    output.capture_range_des_m = reference.capture_range_des_m;
    output.route_kind = ControlRouteKind::AimPoint;
    output.behavior_id = DoctrineBehaviorId::HabfmOneCircle;
    output.mode_id = DoctrineModeId::Habfm;
    output.writer_id = ControlIntentWriterHabfm;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}
}
