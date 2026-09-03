#include "LadyLuck/guidance/dbfm/DbfmHardTurnControlIntent.hpp"

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
        + (value[1] * value[1]
            + value[2] * value[2]));
}

enum class HorizontalDirectionState : std::uint8_t
{
    Available = 0U,
    Unobservable = 1U,
    NonFinite = 2U
};

HorizontalDirectionState ClassifyHorizontalDirection(
    const LadyLuck::Vector3& value) noexcept
{
    if (!FiniteVector(value))
    {
        return HorizontalDirectionState::NonFinite;
    }

    const LadyLuck::Vector3 horizontal{{value[0], value[1], 0.0}};
    const double magnitude = VectorNorm(horizontal);
    if (!std::isfinite(magnitude))
    {
        return HorizontalDirectionState::NonFinite;
    }
    return magnitude < LadyLuck::constants::Tiny
        ? HorizontalDirectionState::Unobservable
        : HorizontalDirectionState::Available;
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
    if (!FiniteVector(value))
    {
        result.status.code = LadyLuck::StatusCode::NonFiniteInput;
        return result;
    }
    const double magnitude = VectorNorm(value);
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
        value[0] / magnitude,
        value[1] / magnitude,
        value[2] / magnitude}};
    return result;
}
}

namespace LadyLuck
{
void BuildDbfmHardTurnReference(
    const DogfightGeometryFrame& frame,
    DbfmHardTurnReference& output,
    Status& status) noexcept
{
    output = DbfmHardTurnReference{};
    status = Status{};

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    if (!FiniteVector(own_position) || !FiniteVector(adversary_position))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const Vector3 attacker_offset{{
        adversary_position[0] - own_position[0],
        adversary_position[1] - own_position[1],
        adversary_position[2] - own_position[2]}};
    const Result<Vector3> own_nose = HorizontalUnit(frame.own.nose_ned);
    const Result<Vector3> attacker_los = HorizontalUnit(attacker_offset);
    if ((own_nose.status.code != StatusCode::Ok
            && own_nose.status.code != StatusCode::InvalidArgument)
        || (attacker_los.status.code != StatusCode::Ok
            && attacker_los.status.code != StatusCode::InvalidArgument))
    {
        status = own_nose.status.code != StatusCode::Ok
            && own_nose.status.code != StatusCode::InvalidArgument
            ? own_nose.status
            : attacker_los.status;
        return;
    }

    Result<Vector3> direction{};
    if (own_nose.ok() && attacker_los.ok())
    {
        const double toward_sign =
            own_nose.value[0] * attacker_los.value[1]
            - own_nose.value[1] * attacker_los.value[0];
        const Vector3 lateral_direction{{
            -attacker_los.value[1], attacker_los.value[0], 0.0}};
        const double side = toward_sign > 0.0 ? 1.0 : -1.0;
        const Vector3 break_direction{{
            -side * lateral_direction[0],
            -side * lateral_direction[1],
            -side * lateral_direction[2]}};

        Vector3 bisector{{
            own_nose.value[0] + break_direction[0],
            own_nose.value[1] + break_direction[1],
            own_nose.value[2] + break_direction[2]}};
        const Vector3 bisector_horizontal{{
            bisector[0], bisector[1], 0.0}};
        if (VectorNorm(bisector_horizontal) < constants::Tiny)
        {
            bisector = break_direction;
        }
        direction = HorizontalUnit(bisector);
    }
    else
    {
        // Horizontal heading or LOS is singular.  Use the same current
        // measured body-up vector already used by Gun 3-D containment; this
        // remains a DBFM maneuver reference, not a HorizontalHold fallback.
        direction = Unit3(Vector3{{
            -frame.own.down_ned[0],
            -frame.own.down_ned[1],
            -frame.own.down_ned[2]}});
    }
    if (!direction.ok())
    {
        status = direction.status;
        return;
    }

    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double measured_range_m = VectorNorm(attacker_offset);
    const double resolved_range_m = measured_range_m > constants::Tiny
        ? measured_range_m
        : constants::Tiny;
    const double range_m =
        std::isfinite(frame.enemy_offense.range_m)
            && frame.enemy_offense.range_m > 0.0
        ? frame.enemy_offense.range_m
        : resolved_range_m;
    const double capture_range_m =
        std::isfinite(frame.enemy_offense.phase.max_range_m)
            && frame.enemy_offense.phase.max_range_m > 0.0
        ? frame.enemy_offense.phase.max_range_m
        : range_m;
    const double speed_mps = VectorNorm(own_velocity);
    const double positive_values[] = {
        range_m,
        capture_range_m,
        speed_mps};
    for (const double value : positive_values)
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
        own_position[0] + range_m * direction.value[0],
        own_position[1] + range_m * direction.value[1],
        own_position[2] + range_m * direction.value[2]}};
    output.desired_speed_mps = speed_mps;
    output.capture_range_des_m = capture_range_m;
}

void ObserveDbfmHardTurnGeometry(
    const DogfightGeometryFrame& frame,
    DbfmHardTurnGeometryReceipt& output,
    Status& status) noexcept
{
    output = DbfmHardTurnGeometryReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const HorizontalDirectionState nose_state =
        ClassifyHorizontalDirection(frame.own.nose_ned);
    if (nose_state == HorizontalDirectionState::NonFinite)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.opponent.position_ned_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Vector3 attacker_offset{{
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0],
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1],
        frame.opponent.position_ned_m[2] - frame.own.position_ned_m[2]}};
    const HorizontalDirectionState los_state =
        ClassifyHorizontalDirection(attacker_offset);
    if (los_state == HorizontalDirectionState::NonFinite)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.valid = true;
    if (nose_state == HorizontalDirectionState::Unobservable
        || los_state == HorizontalDirectionState::Unobservable)
    {
        const Result<Vector3> body_up = Unit3(Vector3{{
            -frame.own.down_ned[0],
            -frame.own.down_ned[1],
            -frame.own.down_ned[2]}});
        if (!body_up.ok())
        {
            status = body_up.status;
            output = DbfmHardTurnGeometryReceipt{};
            return;
        }
    }

    output.command_geometry_available = true;
    output.unavailability = DbfmHardTurnGeometryUnavailability::None;
}

void BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    DbfmHardTurnReference reference{};
    BuildDbfmHardTurnReference(frame, reference, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    ControlIntent candidate{};
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = reference.aim_point_m;
    candidate.desired_speed_mps = reference.desired_speed_mps;
    candidate.capture_range_des_m = reference.capture_range_des_m;
    candidate.behavior_id = DoctrineBehaviorId::DbfmHardTurn;
    candidate.mode_id = DoctrineModeId::Dbfm;
    candidate.route_kind = ControlRouteKind::AimPoint;
    candidate.writer_id = ControlIntentWriterDbfmHardTurn;
    candidate.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
        return;
    }
    output = candidate;
}

void BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame,
    const DbfmHardTurnGeometryReceipt& geometry,
    ControlIntent& output,
    bool& command_ready,
    Status& status) noexcept
{
    output.Clear();
    command_ready = false;
    status = Status{};
    const bool declared_available =
        geometry.unavailability == DbfmHardTurnGeometryUnavailability::None;
    const bool recognized_unavailability =
        declared_available
        || geometry.unavailability ==
            DbfmHardTurnGeometryUnavailability::
                OwnNoseHorizontalUnobservable
        || geometry.unavailability ==
            DbfmHardTurnGeometryUnavailability::
                AttackerLosHorizontalUnobservable;
    if (!geometry.valid
        || !SameControlFrameIdentity(
            geometry.frame_identity,
            frame.frame_identity)
        || !recognized_unavailability
        || geometry.command_geometry_available != declared_available)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    if (!geometry.command_geometry_available)
    {
        // Retained only for compatibility with historical receipts. Current
        // finite vertical geometry is admitted by ObserveDbfmHardTurnGeometry.
        return;
    }

    BuildDbfmHardTurnCommand(frame, output, status);
    command_ready = status.code == StatusCode::Ok;
    if (!command_ready)
    {
        output.Clear();
    }
}
}
