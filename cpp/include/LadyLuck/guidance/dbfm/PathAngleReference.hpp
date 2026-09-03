#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"

namespace LadyLuck
{
struct PathAngleReferenceLaw
{
    double course_gain_per_s = 0.45;
    double path_gain_per_s = 0.36;
    double course_rate_limit_radps = 0.30;
    double path_rate_limit_radps = 0.18;
    double path_command_limit_rad =
        0.61086523819801530703224651306312; // 35 deg
};

struct PathAngleReferenceInput
{
    Vector3 aim_point_ned_m{};
    Vector3 position_ned_m{};
    // The production law reconstructs NED velocity from these body-axis and
    // attitude values. It does not consume the separate DBFM world-velocity
    // field used by the BREAK lateral-force allocation.
    Vector3 velocity_body_mps{};
    Vector3 rpy_rad{};
};

// Pure reconstruction of the current Stage-07 path-angle law's raw inertial
// NED acceleration. This is a guidance reference, not p/q/r, Nz, surface,
// thrust, or observed aircraft response.
Result<Vector3> BuildPathAngleAccelerationReferenceNed(
    const PathAngleReferenceInput& input,
    const PathAngleReferenceLaw& law = PathAngleReferenceLaw{}) noexcept;
}
