#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <cmath>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteMatrix(const LadyLuck::Matrix3RowMajor& value) noexcept
{
    for (const double element : value)
    {
        if (!std::isfinite(element))
        {
            return false;
        }
    }
    return true;
}

LadyLuck::Vector3 Subtract(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

bool FiniteKinematics(
    const LadyLuck::AircraftGeometryKinematics& value) noexcept
{
    return FiniteVector(value.position_ned_m)
        && FiniteVector(value.velocity_body_mps)
        && FiniteVector(value.velocity_ned_mps)
        && FiniteVector(value.rpy_rad)
        && FiniteVector(value.nose_ned)
        && FiniteVector(value.right_ned)
        && FiniteVector(value.down_ned)
        && FiniteMatrix(value.dcm_body_to_ned);
}

LadyLuck::WezSnapshotInput MakeSnapshotInput(
    const LadyLuck::AircraftGeometryKinematics& attacker,
    const LadyLuck::AircraftGeometryKinematics& target,
    const LadyLuck::DogfightGeometryInput& input) noexcept
{
    LadyLuck::WezSnapshotInput snapshot{};
    snapshot.attacker_pos_m = attacker.position_ned_m;
    snapshot.attacker_vel_ned_mps = attacker.velocity_ned_mps;
    snapshot.attacker_nose_ned = attacker.nose_ned;
    snapshot.target_pos_m = target.position_ned_m;
    snapshot.target_vel_ned_mps = target.velocity_ned_mps;
    snapshot.target_nose_ned = target.nose_ned;
    snapshot.has_target_nose = true;
    snapshot.has_target_accel = false;
    snapshot.t_sec = input.t_sec;
    snapshot.tau_sec = input.tau_sec;
    snapshot.capture_range_des_m = input.capture_range_des_m;
    snapshot.soft_sigma_deg = input.soft_sigma_deg;
    return snapshot;
}

bool FiniteFrame(const LadyLuck::DogfightGeometryFrame& frame) noexcept
{
    return LadyLuck::IsValidControlFrameIdentity(frame.frame_identity)
        && std::isfinite(frame.t_sec)
        && std::isfinite(frame.tau_sec)
        && FiniteKinematics(frame.own)
        && FiniteKinematics(frame.opponent)
        && FiniteVector(frame.rel_pos_own_body_m)
        && FiniteVector(frame.rel_vel_own_body_mps)
        && FiniteVector(frame.opponent_nose_own_body)
        && std::isfinite(frame.range_rate_mps)
        && std::isfinite(frame.closing_speed_mps)
        && std::isfinite(frame.capture_error_m)
        && std::isfinite(frame.enemy_capture_error_m)
        && std::isfinite(frame.own_offense.range_m)
        && std::isfinite(frame.enemy_offense.range_m)
        && std::isfinite(frame.own_offense.damage_rate)
        && std::isfinite(frame.enemy_offense.damage_rate)
        && std::isfinite(frame.own_offense.soft_potential)
        && std::isfinite(frame.enemy_offense.soft_potential);
}
}

namespace LadyLuck
{
Result<AircraftGeometryKinematics> BuildAircraftGeometryKinematics(
    const PlaneState& state) noexcept
{
    Result<AircraftGeometryKinematics> result{};
    if (!FiniteVector(state.position_ned_m)
        || !FiniteVector(state.velocity_body_mps)
        || !FiniteVector(state.rpy_rad))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    const Result<Matrix3RowMajor> ned_to_body =
        RpyToDcmNedToBody(state.rpy_rad);
    if (!ned_to_body.ok())
    {
        result.status = ned_to_body.status;
        return result;
    }

    AircraftGeometryKinematics& output = result.value;
    output.position_ned_m = state.position_ned_m;
    output.velocity_body_mps = state.velocity_body_mps;
    output.rpy_rad = state.rpy_rad;
    output.dcm_body_to_ned = MatrixTranspose(ned_to_body.value);
    output.velocity_ned_mps = MatrixVectorProduct(
        output.dcm_body_to_ned,
        state.velocity_body_mps);

    const Result<Vector3> nose = UnitVector(MatrixVectorProduct(
        output.dcm_body_to_ned,
        Vector3{{1.0, 0.0, 0.0}}));
    const Result<Vector3> right = UnitVector(MatrixVectorProduct(
        output.dcm_body_to_ned,
        Vector3{{0.0, 1.0, 0.0}}));
    const Result<Vector3> down = UnitVector(MatrixVectorProduct(
        output.dcm_body_to_ned,
        Vector3{{0.0, 0.0, 1.0}}));
    if (!nose.ok() || !right.ok() || !down.ok())
    {
        result.status = !nose.ok()
            ? nose.status
            : (!right.ok() ? right.status : down.status);
        result.value = AircraftGeometryKinematics{};
        return result;
    }
    output.nose_ned = nose.value;
    output.right_ned = right.value;
    output.down_ned = down.value;
    if (!FiniteKinematics(output))
    {
        result.status.code = StatusCode::NonFiniteInput;
        result.value = AircraftGeometryKinematics{};
    }
    return result;
}

Result<DogfightGeometryFrame> BuildDogfightGeometryFrame(
    const DogfightGeometryInput& input) noexcept
{
    Result<DogfightGeometryFrame> result{};
    if (!IsValidControlFrameIdentity(input.frame_identity)
        || input.frame_identity.frame_index != input.own.frame_index)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    if (!std::isfinite(input.t_sec)
        || !std::isfinite(input.tau_sec)
        || !std::isfinite(input.capture_range_des_m)
        || !std::isfinite(input.soft_sigma_deg))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    const Result<AircraftGeometryKinematics> own =
        BuildAircraftGeometryKinematics(input.own);
    const Result<AircraftGeometryKinematics> opponent =
        BuildAircraftGeometryKinematics(input.opponent);
    if (!own.ok() || !opponent.ok())
    {
        result.status = !own.ok() ? own.status : opponent.status;
        return result;
    }

    const Result<WezGeometry> own_offense = BuildWezGeometry(
        MakeSnapshotInput(own.value, opponent.value, input));
    const Result<WezGeometry> enemy_offense = BuildWezGeometry(
        MakeSnapshotInput(opponent.value, own.value, input));
    if (!own_offense.ok() || !enemy_offense.ok())
    {
        result.status = !own_offense.ok()
            ? own_offense.status
            : enemy_offense.status;
        return result;
    }

    const Vector3 rel_pos_ned = Subtract(
        opponent.value.position_ned_m,
        own.value.position_ned_m);
    const Vector3 rel_vel_ned = Subtract(
        opponent.value.velocity_ned_mps,
        own.value.velocity_ned_mps);
    const double current_range_m = VectorNorm(rel_pos_ned);
    if (!std::isfinite(current_range_m))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const Result<Vector3> range_los = current_range_m >= constants::Tiny
        ? UnitVector(rel_pos_ned, own.value.nose_ned)
        : UnitVector(own.value.nose_ned);
    if (!range_los.ok())
    {
        result.status = range_los.status;
        return result;
    }

    DogfightGeometryFrame& output = result.value;
    output.frame_identity = input.frame_identity;
    output.own_plane_id = input.own.plane_id;
    output.target_plane_id = input.opponent.plane_id;
    output.target_frame_index = input.opponent.frame_index;
    output.target_same_index = input.opponent.frame_index
        == input.frame_identity.frame_index;
    output.t_sec = input.t_sec;
    output.tau_sec = input.tau_sec;
    output.own = own.value;
    output.opponent = opponent.value;
    output.own_offense = own_offense.value;
    output.enemy_offense = enemy_offense.value;
    output.rel_pos_own_body_m = TransposeMatrixVectorProduct(
        own.value.dcm_body_to_ned,
        rel_pos_ned);
    output.rel_vel_own_body_mps = TransposeMatrixVectorProduct(
        own.value.dcm_body_to_ned,
        rel_vel_ned);
    output.opponent_nose_own_body = TransposeMatrixVectorProduct(
        own.value.dcm_body_to_ned,
        opponent.value.nose_ned);
    output.range_rate_mps = Dot(rel_vel_ned, range_los.value);
    output.closing_speed_mps = -output.range_rate_mps;
    output.capture_error_m = VectorNorm(Subtract(
        own_offense.value.capture_point_m,
        own.value.position_ned_m));
    output.enemy_capture_error_m = VectorNorm(Subtract(
        enemy_offense.value.capture_point_m,
        opponent.value.position_ned_m));

    if (!FiniteFrame(output))
    {
        result.status.code = StatusCode::NonFiniteInput;
        result.value = DogfightGeometryFrame{};
    }
    return result;
}
}
