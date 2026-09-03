#include "LadyLuck/guidance/dbfm/DbfmBreakLoadControlIntent.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/guidance/dbfm/PathAngleReference.hpp"

#include <cmath>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double PythonDot3(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    const double term0 = left[0] * right[0];
    const double term1 = left[1] * right[1];
    const double term2 = left[2] * right[2];
    return (term0 + term2) + term1;
}

double PythonNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(PythonDot3(value, value));
}

double PositiveZero(const double value) noexcept
{
    return value == 0.0 ? 0.0 : value;
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

bool IsBreakIntent(const LadyLuck::ControlIntent& command) noexcept
{
    return command.behavior_id
            == LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak
        || command.behavior_id == LadyLuck::DoctrineBehaviorId::DbfmBreak;
}

void BuildReferenceFromAimPoint(
    const LadyLuck::Vector3& aim_point_ned_m,
    const LadyLuck::DbfmBreakLoadKinematics& own,
    const double instantaneous_load_limit_g,
    LadyLuck::DbfmBreakLoadReference& output,
    LadyLuck::Status& status) noexcept
{
    output = LadyLuck::DbfmBreakLoadReference{};
    status = LadyLuck::Status{};
    if (!std::isfinite(instantaneous_load_limit_g))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (instantaneous_load_limit_g <= 1.0)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    if (!FiniteVector(aim_point_ned_m)
        || !FiniteVector(own.position_ned_m)
        || !FiniteVector(own.velocity_body_mps)
        || !FiniteVector(own.velocity_world_ned_mps)
        || !FiniteVector(own.rpy_rad))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }

    LadyLuck::PathAngleReferenceInput path_input{};
    path_input.aim_point_ned_m = aim_point_ned_m;
    path_input.position_ned_m = own.position_ned_m;
    path_input.velocity_body_mps = own.velocity_body_mps;
    path_input.rpy_rad = own.rpy_rad;
    const LadyLuck::Result<LadyLuck::Vector3> raw =
        LadyLuck::BuildPathAngleAccelerationReferenceNed(path_input);
    if (!raw.ok())
    {
        status = raw.status;
        return;
    }

    const double speed = PythonNorm3(own.velocity_world_ned_mps);
    if (!std::isfinite(speed))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (speed <= LadyLuck::constants::Tiny)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    const LadyLuck::Vector3 velocity_hat{{
        own.velocity_world_ned_mps[0] / speed,
        own.velocity_world_ned_mps[1] / speed,
        own.velocity_world_ned_mps[2] / speed}};
    const LadyLuck::Vector3 down{{0.0, 0.0, 1.0}};
    LadyLuck::Vector3 lateral_axis = Cross(down, velocity_hat);
    const double lateral_norm = PythonNorm3(lateral_axis);
    if (!std::isfinite(lateral_norm)
        || lateral_norm <= LadyLuck::constants::Tiny)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    for (double& value : lateral_axis)
    {
        value /= lateral_norm;
    }
    LadyLuck::Vector3 vertical_plane_axis = Cross(
        velocity_hat,
        lateral_axis);
    const double vertical_norm = PythonNorm3(vertical_plane_axis);
    if (!std::isfinite(vertical_norm) || vertical_norm <= 0.0)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    for (double& value : vertical_plane_axis)
    {
        value /= vertical_norm;
    }

    const LadyLuck::Vector3 raw_specific_force{{
        raw.value[0],
        raw.value[1],
        raw.value[2] - LadyLuck::constants::StandardGravityMps2}};
    const double raw_lateral_force = PythonDot3(
        raw_specific_force,
        lateral_axis);
    if (!std::isfinite(raw_lateral_force))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (std::fabs(raw_lateral_force) <= LadyLuck::constants::Tiny)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    const double vertical_plane_force = PythonDot3(
        raw_specific_force,
        vertical_plane_axis);
    const double total_force = instantaneous_load_limit_g
        * LadyLuck::constants::StandardGravityMps2;
    const double lateral_force_squared = total_force * total_force
        - vertical_plane_force * vertical_plane_force;
    if (!std::isfinite(lateral_force_squared))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (lateral_force_squared <= 0.0)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return;
    }
    const double lateral_force = std::copysign(
        std::sqrt(lateral_force_squared),
        raw_lateral_force);
    const LadyLuck::Vector3 admitted_specific_force{{
        vertical_plane_force * vertical_plane_axis[0]
            + lateral_force * lateral_axis[0],
        vertical_plane_force * vertical_plane_axis[1]
            + lateral_force * lateral_axis[1],
        vertical_plane_force * vertical_plane_axis[2]
            + lateral_force * lateral_axis[2]}};
    const LadyLuck::Vector3 admitted_acceleration{{
        PositiveZero(admitted_specific_force[0]),
        PositiveZero(admitted_specific_force[1]),
        PositiveZero(
            LadyLuck::constants::StandardGravityMps2
                + admitted_specific_force[2])}};
    LadyLuck::RollGV2Input roll_input{};
    roll_input.inertial_acceleration_ned_mps2 = admitted_acceleration;
    roll_input.velocity_ned_mps = own.velocity_world_ned_mps;
    roll_input.roll_actual_rad = own.rpy_rad[0];
    roll_input.pitch_rad = own.rpy_rad[1];
    roll_input.alpha_rad = std::atan2(
        own.velocity_body_mps[2],
        own.velocity_body_mps[0]);
    const LadyLuck::Result<LadyLuck::RollGV2Result> roll =
        LadyLuck::BuildRollGV2Reference(roll_input);
    if (!roll.ok())
    {
        status = roll.status;
        return;
    }
    if (std::fabs(roll.value.g_cmd - instantaneous_load_limit_g)
        > 1.0e-12 * std::fabs(instantaneous_load_limit_g))
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return;
    }

    output.raw_acceleration_ned_mps2 = raw.value;
    output.admitted_acceleration_ned_mps2 = admitted_acceleration;
    output.instantaneous_load_limit_g = instantaneous_load_limit_g;
    output.roll_pull = roll.value;
}

} // namespace

namespace LadyLuck
{

void BuildDbfmBreakLoadReference(
    const ControlIntent& command,
    const DbfmBreakLoadKinematics& own,
    const double instantaneous_load_limit_g,
    DbfmBreakLoadReference& output,
    Status& status) noexcept
{
    output = DbfmBreakLoadReference{};
    command.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!IsBreakIntent(command)
        || command.mode_id != DoctrineModeId::Dbfm
        || command.direct_p_cmd_radps.has_value
        || command.direct_nz_cmd_g.has_value
        || command.direct_acceleration_ned_mps2.has_value
        || command.k_roll.has_value
        || command.k_pitch.has_value)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    BuildReferenceFromAimPoint(
        command.aim_point_m,
        own,
        instantaneous_load_limit_g,
        output,
        status);
}

void ApplyDbfmBreakLoad(
    const ControlIntent& command,
    const DbfmBreakLoadKinematics& own,
    const DbfmBreakLoadEvidence& evidence,
    const DbfmBreakLoadConfig& config,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    command.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!config.enabled
        || !IsBreakIntent(command)
        || !evidence.capability_admitted
        || !evidence.instantaneous_load_limit_g.has_value)
    {
        output = command;
        status = Status{};
        return;
    }

    if (command.mode_id != DoctrineModeId::Dbfm
        || command.direct_p_cmd_radps.has_value
        || command.direct_nz_cmd_g.has_value
        || command.direct_acceleration_ned_mps2.has_value
        || command.k_roll.has_value
        || command.k_pitch.has_value)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!std::isfinite(evidence.instantaneous_load_limit_g.value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (evidence.instantaneous_load_limit_g.value <= 1.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!FiniteVector(own.position_ned_m)
        || !FiniteVector(own.velocity_body_mps)
        || !FiniteVector(own.velocity_world_ned_mps)
        || !FiniteVector(own.rpy_rad))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    DbfmBreakLoadReference reference{};
    BuildDbfmBreakLoadReference(
        command,
        own,
        evidence.instantaneous_load_limit_g.value,
        reference,
        status);
    if (status.code == StatusCode::InvalidArgument)
    {
        output = command;
        status = Status{};
        return;
    }
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    ControlIntent candidate = command;
    candidate.direct_acceleration_ned_mps2.has_value = true;
    candidate.direct_acceleration_ned_mps2.value =
        reference.admitted_acceleration_ned_mps2;
    candidate.direct_acceleration_tracking_enabled = true;
    candidate.direct_acceleration_magnitude_tracking_enabled =
        config.magnitude_tracking_enabled;
    candidate.direct_acceleration_loaded_roll_enabled =
        config.loaded_roll_enabled;
    candidate.route_kind = ControlRouteKind::DirectNedAcceleration;
    candidate.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
        return;
    }
    output = candidate;
}

} // namespace LadyLuck
