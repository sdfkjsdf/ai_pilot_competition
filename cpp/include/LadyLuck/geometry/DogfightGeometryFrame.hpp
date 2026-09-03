#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/geometry/WezGeometry.hpp"

#include <cstdint>

namespace LadyLuck
{
struct AircraftGeometryKinematics
{
    Vector3 position_ned_m{};
    Vector3 velocity_body_mps{};
    Vector3 velocity_ned_mps{};
    Vector3 rpy_rad{};
    Vector3 nose_ned{};
    Vector3 right_ned{};
    Vector3 down_ned{};
    Matrix3RowMajor dcm_body_to_ned{};
};

struct DogfightGeometryInput
{
    // Causal identity of the accepted ownship estimator sample. The official
    // connector pairs the latest accepted ownship and opponent arrivals and
    // therefore does not require either raw plane index to equal the command
    // index (or each other).
    ControlFrameIdentity frame_identity{};
    PlaneState own{};
    PlaneState opponent{};
    // Decision/command chronology. This may be newer than the ownship sample
    // time when the second arrival completing a pair is the opponent packet.
    double t_sec = 0.0;
    double tau_sec = 0.0;
    double capture_range_des_m = 650.0;
    double soft_sigma_deg = 5.0;
};

struct DogfightGeometryFrame
{
    // Remains bound to the ownship estimator sample; t_sec below remains the
    // independently supplied decision chronology.
    ControlFrameIdentity frame_identity{};
    // Plane identities are part of the Python same-index geometry lineage.
    // Keeping them in the immutable frame lets long-lived observers reset on
    // a target swap even when episode/index/time happen to remain continuous.
    std::int32_t own_plane_id = -1;
    std::int32_t target_plane_id = -1;
    // Raw asynchronous target chronology is preserved rather than silently
    // relabeled as the ownship estimator frame. Consumers that require a
    // synchronized geometry sample must explicitly admit `target_same_index`.
    std::uint64_t target_frame_index = 0U;
    bool target_same_index = false;
    double t_sec = 0.0;
    double tau_sec = 0.0;
    AircraftGeometryKinematics own{};
    AircraftGeometryKinematics opponent{};
    WezGeometry own_offense{};
    WezGeometry enemy_offense{};
    Vector3 rel_pos_own_body_m{};
    Vector3 rel_vel_own_body_mps{};
    Vector3 opponent_nose_own_body{};
    double range_rate_mps = 0.0;
    double closing_speed_mps = 0.0;
    double capture_error_m = 0.0;
    double enemy_capture_error_m = 0.0;
};

Result<AircraftGeometryKinematics> BuildAircraftGeometryKinematics(
    const PlaneState& state) noexcept;
Result<DogfightGeometryFrame> BuildDogfightGeometryFrame(
    const DogfightGeometryInput& input) noexcept;
}
