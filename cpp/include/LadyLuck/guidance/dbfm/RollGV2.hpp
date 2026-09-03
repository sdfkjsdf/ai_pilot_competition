#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"

namespace LadyLuck
{
struct RollGV2Input
{
    Vector3 inertial_acceleration_ned_mps2{};
    Vector3 velocity_ned_mps{};
    double roll_actual_rad = 0.0;
    double pitch_rad = 0.0;
    double alpha_rad = 0.0;
    double roll_gain_per_s = 4.0;
    double max_roll_rate_radps = 2.0;
};

struct RollGV2Result
{
    double g_cmd = 0.0;
    double bank_command_rad = 0.0;
    double bank_error_rad = 0.0;
    double roll_rate_command_radps = 0.0;
    double nz_command_g = 0.0;
    double pitch_rate_command_radps = 0.0;
};

// Stateless geometry cross-check used by DBFM BreakLoad. It is not the live
// NED allocator and does not publish a control-surface command.
Result<RollGV2Result> BuildRollGV2Reference(
    const RollGV2Input& input) noexcept;
}
