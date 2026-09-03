#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/geometry/WezRule.hpp"

namespace LadyLuck
{
struct WezLimits
{
    double angle_rad = 0.0;
    double min_range_m = 0.0;
    double max_range_m = 0.0;
};

struct WezPhaseMatch
{
    bool matched = false;
    WezPhase phase{};
    double damage_rate = 0.0;
};

struct WezDamageScores
{
    double hard = 0.0;
    double soft = 0.0;
};

struct WezSnapshotInput
{
    Vector3 attacker_pos_m{};
    Vector3 attacker_vel_ned_mps{};
    Vector3 attacker_nose_ned{};
    Vector3 target_pos_m{};
    Vector3 target_vel_ned_mps{};
    Vector3 target_nose_ned{};
    bool has_target_nose = true;
    Vector3 target_accel_ned_mps2{};
    bool has_target_accel = false;
    double t_sec = 0.0;
    double tau_sec = 0.0;
    double capture_range_des_m = 650.0;
    double soft_sigma_deg = 5.0;
};

struct WezGeometry
{
    WezPhase phase{};
    double t_sec = 0.0;
    double tau_sec = 0.0;
    Vector3 attacker_pos_m{};
    Vector3 attacker_vel_ned_mps{};
    Vector3 attacker_nose_ned{};
    Vector3 target_pos_m{};
    Vector3 target_vel_ned_mps{};
    Vector3 target_nose_ned{};
    Vector3 target_pred_pos_m{};
    Vector3 target_pred_vel_ned_mps{};
    Vector3 attacker_pred_pos_m{};
    Vector3 capture_point_m{};
    double capture_range_des_m = 0.0;
    Vector3 los_hat_ned{};
    double range_m = 0.0;
    double ata_rad = 0.0;
    double aspect_rad = 0.0;
    double hca_rad = 0.0;
    double damage_rate = 0.0;
    double soft_potential = 0.0;
};

// UnitVector intentionally returns fallback unchanged when |vector| < Tiny.
// This mirrors the Python authority; callers that need a unit fallback perform
// the explicit nested normalization used by that source.
Result<Vector3> UnitVector(
    const Vector3& vector,
    const Vector3& fallback = Vector3{{1.0, 0.0, 0.0}}) noexcept;
Result<double> AngleBetween(
    const Vector3& left,
    const Vector3& right) noexcept;
Result<WezLimits> ActiveWezLimits(double t_sec) noexcept;
Result<WezPhaseMatch> MatchWezPhase(
    double range_m,
    double ata_rad,
    double t_sec) noexcept;
Result<WezDamageScores> ComputeWezDamageScores(
    double los_deg,
    double range_m,
    double t_sec,
    double soft_sigma_deg = 5.0) noexcept;
Result<WezGeometry> BuildWezGeometry(
    const WezSnapshotInput& input) noexcept;
}
