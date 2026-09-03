#pragma once

#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace safety
{

using LadyLuck::control::tecs_cis::NormalizedControlCommand;

enum class AutoGcasPhase : std::int32_t
{
    Inactive = -1,
    Roll = 0,
    Pull = 1,
    Level = 2
};

struct AutoGcasConfig
{
    bool enabled = true;
    double crash_floor_m = 304.8;
    double min_altitude_m = 350.0;
    double entry_effective_roll_rate_radps =
        0.87266462599716477; // 50 deg/s achieved-bank assumption
    double prediction_margin = 1.10;
    double onset_rate_gps = 4.0;
    // Retained by the Python API for compatibility; LEVEL release is governed
    // by the explicit prediction/attitude/rate/load/settle gates below.
    double release_altitude_m = 600.0;
    double wings_level_gate_rad = 0.17453292519943295;
    double settled_roll_rate_gate_radps = 0.08726646259971647;
    double minimum_pull_time_s = 0.0;
    double pull_n_g = 9.0;
    double maximum_roll_rate_radps = 3.14;
    double maximum_prediction_horizon_s = 10.0;
    double settle_gamma_rad = 0.017453292519943295;
    double minimum_settle_time_s = 0.25;
    double settle_nz_tolerance_g = 0.1;
    double settle_vertical_speed_gain_g_per_mps =
        0.02 / 0.3048;
    double settle_min_nz_g = 0.0;
};

// Side-effect-free current-frame entry input.  This is evaluated once after
// estimator commit and current physical-envelope construction, before the
// tactical BehaviorTree runs.
struct AutoGcasEntryInput
{
    ControlFrameIdentity estimator_frame_identity{};
    ControlFrameIdentity envelope_frame_identity{};
    double t_sec = 0.0;
    double dt_s = 0.0;
    PlaneState ownship{};
    double roll_rate_endpoint_radps = 0.0;
    bool roll_rate_endpoint_valid = false;
    double measured_nz_g = 0.0;
    bool measured_nz_valid = false;
    double available_nz_g = 0.0;
    bool available_nz_valid = false;
};

// Immutable receipt consumed both by tactical doctrine and by the final
// Auto-GCAS authority.  `valid` means the finite same-frame input was
// evaluated; `entry_available` and `entry_recoverable` remain independent
// physical-authority receipts and are never inferred from finite defaults.
struct AutoGcasEntryReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    AutoGcasEntryInput evaluated_input{};
    double climb_rate_mps = 0.0;
    bool entry_available = false;
    bool entry_should_activate = false;
    bool entry_boundary_breached = false;
    bool entry_recoverable = false;
    bool effective_nz_valid = false;
    double effective_nz_g = 0.0;
    bool prediction_valid = false;
    bool prediction_recovery_established = false;
    bool prediction_horizon_exhausted = false;
    bool predicted_bottom_altitude_valid = false;
    double predicted_bottom_altitude_m = 0.0;
};

struct AutoGcasSnapshot
{
    bool active = false;
    AutoGcasPhase phase = AutoGcasPhase::Roll;
    bool has_pull_start_time = false;
    double pull_start_time_s = 0.0;
    bool has_settle_candidate_time = false;
    double settle_candidate_time_s = 0.0;
    double latest_climb_rate_mps = 0.0;
};

struct AutoGcasReceipt
{
    bool override_active = false;
    AutoGcasPhase phase = AutoGcasPhase::Inactive;
    double recovery_load_factor_g = 0.0;
    double climb_rate_mps = 0.0;
    ControlFrameIdentity frame_identity{};
    bool entry_available = false;
    bool entry_should_activate = false;
    bool entry_boundary_breached = false;
    bool entry_recoverable = false;
    bool prediction_valid = false;
    bool prediction_recovery_established = false;
    bool prediction_horizon_exhausted = false;
    bool predicted_bottom_altitude_valid = false;
    double predicted_bottom_altitude_m = 0.0;
    NormalizedControlCommand post_command{};
};

// Terrain recovery uses the smaller of the configured ceiling and the current
// physical Nz authority. Missing authority is command-neutral.
class AutoGcas final
{
public:
    AutoGcas() noexcept;
    explicit AutoGcas(const AutoGcasConfig& config) noexcept;

    void CopyConfigurationValid(bool& output) const noexcept;
    void Reset() noexcept;
    void EvaluateEntry(
        const AutoGcasEntryInput& input,
        AutoGcasEntryReceipt& output,
        Status& status) const noexcept;
    // Build the complete actuator-domain recovery command for the selected
    // top-level Safety Task.  No nominal tactical command is
    // required as an input.
    void BuildRecoveryCommand(
        const AutoGcasEntryReceipt& entry_receipt,
        AutoGcasReceipt& output,
        Status& status) noexcept;
    void CopySnapshot(AutoGcasSnapshot& output) const noexcept;

private:
    AutoGcasConfig config_{};
    bool configuration_valid_ = true;
    AutoGcasSnapshot state_{};
};

static_assert(
    std::is_trivially_copyable<AutoGcasEntryInput>::value,
    "Auto-GCAS entry input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<AutoGcasEntryReceipt>::value,
    "Auto-GCAS entry receipt must stay allocation-free.");

} // namespace safety
} // namespace LadyLuck
