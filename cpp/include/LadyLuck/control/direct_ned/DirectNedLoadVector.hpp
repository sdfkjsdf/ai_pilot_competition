#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{
// Numeric guidance/FCS boundary.  Tactical labels, writers, route selection,
// direct-beta ownership, and other mutually-exclusive fields are admitted by
// the upstream dispatcher before this command is materialized.
struct DirectNedLoadVectorCommand
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    Vector3 acceleration_ned_mps2{};
    bool roll_rate_reference_valid = false;
    double roll_rate_reference_radps = 0.0;
};

// Same-frame measured/admitted inputs consumed by the frozen production
// direct-NED A3/C6 allocator.  NED is Down-positive.  This is not simulator
// truth and contains no actuator or thrust command.
struct DirectNedLoadVectorState
{
    ControlFrameIdentity frame_identity{};
    Vector3 velocity_ned_mps{};
    Matrix3RowMajor c_body_from_ned{};
    double speed_mps = 0.0;
    double alpha_rad = 0.0;
    double beta_rad = 0.0;
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
    double nz_feasible_g = 0.0;
    double ground_speed_horizontal_mps = 0.0;
    double max_p_radps = 1.0 / 0.31821;
};

enum DirectNedGuardFlag : std::uint32_t
{
    GuardLowG = 1U << 0,
    GuardDirectionReversal = 1U << 1,
    GuardAntipodal = 1U << 2,
    GuardLiftAxisDegenerate = 1U << 3,
    GuardSpeedDegenerate = 1U << 4
};

// Shaped body-rate/load reference.  It stops before TECS thrust, CIS surface
// inversion, AutoGCAS, normalized ControlValue, and aircraft response.
struct DirectNedLoadVectorOutput
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double p_cmd_radps = 0.0;
    double q_cmd_radps = 0.0;
    double r_cmd_radps = 0.0;
    double nz_cmd_g = 0.0;
    double nz_cmd_raw_g = 0.0;
    double force_perp_norm_g = 0.0;
    double guard_weight = 0.0;
    Vector3 target_direction_ned{};
    Vector3 raw_direction_ned{};
    double direction_error_rad = 0.0;
    double direction_step_angle_rad = 0.0;
    double raw_direction_separation_rad = 0.0;
    double c6_gate_value = 0.0;
    std::uint32_t guard_flags = 0U;
    double clip_scale = 0.0;
    bool roll_rate_reference_valid = false;
    double roll_rate_reference_radps = 0.0;
    bool roll_rate_equivalent_bank_lead_valid = false;
    double roll_rate_equivalent_bank_lead_rad = 0.0;
    bool requested_direction_error_valid = false;
    double requested_direction_error_rad = 0.0;
};

struct DirectNedLoadVectorSnapshot
{
    bool has_direction_hold = false;
    Vector3 direction_hold_ned{};
    bool has_c6_gate = false;
    double c6_gate_value = 0.0;
    bool has_last_nz = false;
    double last_nz_cmd_g = 0.0;
    double servo_filtered_error_rad = 0.0;
    double servo_filtered_rate_radps = 0.0;
    std::int32_t servo_committed_turn_side = 0;
    double servo_last_p_cmd_radps = 0.0;
    double servo_last_feedback_error_rad = 0.0;
    bool servo_output_slew_active = false;
    bool has_last_output = false;
};

// Whole frozen production allocator used by the new High-G roll-before-pull
// command field.  The class intentionally exposes no tuning inputs.
class DirectNedLoadVector
{
public:
    DirectNedLoadVector() noexcept = default;

    void Reset() noexcept;
    void CopySnapshot(DirectNedLoadVectorSnapshot& output) const noexcept;

    void Step(
        const DirectNedLoadVectorCommand& command,
        const DirectNedLoadVectorState& state,
        double dt_s,
        DirectNedLoadVectorOutput& output,
        Status& status) noexcept;

private:
    bool has_direction_hold_ = false;
    Vector3 direction_hold_ned_{};
    bool has_c6_gate_ = false;
    double c6_gate_value_ = 0.0;
    bool has_last_nz_ = false;
    double last_nz_cmd_g_ = 0.0;
    double servo_filtered_error_rad_ = 0.0;
    double servo_filtered_rate_radps_ = 0.0;
    std::int32_t servo_committed_turn_side_ = 0;
    double servo_last_p_cmd_radps_ = 0.0;
    double servo_last_feedback_error_rad_ = 0.0;
    bool servo_output_slew_active_ = false;
    bool has_last_output_ = false;
};
}
}
}
