#pragma once

#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace control
{
namespace tecs_cis
{

// Stable guidance/FCS boundary. Route-5 owns these shaped references; this
// module does not inspect TacticalCommand or BehaviorTree labels.
struct BodyRateLoadEnergyCommand
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double p_cmd_radps = 0.0;
    double q_cmd_radps = 0.0;
    double r_cmd_radps = 0.0;
    double nz_cmd_g = 0.0;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double flight_path_angle_cmd_rad = 0.0;
    double specific_energy_rate_bias_m2ps3 = 0.0;
    bool integrator_hold = false;
};

// Normalized BattleServer action in fixed wire order. Negative elevator is
// pull/up in the pinned F-16 path; throttle is the provider [0,1] coordinate.
struct NormalizedControlCommand
{
    double aileron = 0.0;
    double elevator = 0.0;
    double rudder = 0.0;
    double throttle = 0.0;
    bool valid = false;
};

struct TecsCisDiagnostics
{
    double drag_estimate_n = 0.0;
    double thrust_velocity_projection = 0.0;
    double flight_path_angle_rad = 0.0;
    double specific_energy_rate_reference_m2ps3 = 0.0;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    double specific_energy_rate_command_m2ps3 = 0.0;
    double thrust_cmd_raw_n = 0.0;
    double thrust_cmd_limited_n = 0.0;
    double internal_throttle_f_cis = 0.0;
    double energy_integral_before_m2ps = 0.0;
    double energy_integral_after_m2ps = 0.0;
    double trim_mach = 0.0;
    double alpha_required_rad = 0.0;
    double elevator_required_rad = 0.0;
    double scheduler_gain = 0.0;
    double stateless_flaperon_pitch_bias = 0.0;
    double pitch_command_unclipped = 0.0;
    bool lower_thrust_saturated = false;
    bool upper_thrust_saturated = false;
    bool rate_measurement_valid = false;
    bool command_clipped = false;
};

// Exact completed-sample inputs used by d90
// completed_cis_v4_energy_rate_authority().  This receipt is captured from
// the values actually consumed by one accepted Step(), including the
// post-update integral that the Python competition adapter intentionally uses
// as the next sample's causal authority approximation.
struct TecsCisCompletedEnergyAuthorityReceipt
{
    ControlFrameIdentity source_frame_identity{};
    bool valid = false;
    bool continuous_total_energy_controller = false;
    bool rate_measurement_valid = false;
    bool controller_configuration_available = false;
    double energy_error_gain_per_s = 0.0;
    double energy_integral_gain_per_s2 = 0.0;
    double energy_rate_feedback_gain = 0.0;
    double total_energy_error_m2ps2 = 0.0;
    double energy_integral_error_m2ps = 0.0;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    double speed_mps = 0.0;
    double minimum_speed_mps = 0.0;
    double mass_kg = 0.0;
    double drag_estimate_n = 0.0;
    double thrust_velocity_projection = 0.0;
    double thrust_min_n = 0.0;
    double thrust_max_n = 0.0;
};

// Current backend configuration source for the desired-speed shaping bounds.
// The completed receipt above owns the gains used by its own sample; these
// fields describe the configuration that the forthcoming command will use.
struct TecsCisLongitudinalAuthorityConfiguration
{
    bool valid = false;
    bool continuous_total_energy_controller = false;
    double energy_error_gain_per_s = 0.0;
    double energy_integral_gain_per_s2 = 0.0;
    double energy_rate_feedback_gain = 0.0;
    double minimum_speed_mps = 0.0;
    double speed_command_rate_min_mps2 = 0.0;
    double speed_command_rate_max_mps2 = 0.0;
};

struct TecsCisOutput
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    NormalizedControlCommand command{};
    TecsCisDiagnostics diagnostics{};
    TecsCisCompletedEnergyAuthorityReceipt completed_energy_authority{};
};

struct TecsCisConfig
{
    double energy_error_gain_per_s = 0.32;
    double energy_integral_gain_per_s2 = 0.02;
    double energy_rate_feedback_gain = 0.25;
    double antiwindup_gain_per_s = 0.25;
    double integral_error_limit_m2ps = 50000.0;
    double minimum_speed_mps = 30.0;
    double minimum_thrust_velocity_projection = 0.10;
    double speed_command_rate_min_mps2 = -15.0;
    double speed_command_rate_max_mps2 = 15.0;
    double energy_rate_bias_min_m2ps3 = -5000.0;
    double energy_rate_bias_max_m2ps3 = 5000.0;
    double p_command_limit_radps = 1.0 / 0.31821;
    double nz_command_min_g = -4.0;
    double nz_command_max_g = 21.0;
    double stateless_flaperon_pitch_gain_per_g = 0.0765;
    double stateless_flaperon_pitch_bias_limit = 0.0325;
};

struct TecsCisSnapshot
{
    double energy_integral_error_m2ps = 0.0;
    std::uint64_t accepted_step_count = 0U;
};

class TecsCisControl final
{
public:
    TecsCisControl() noexcept;
    explicit TecsCisControl(const TecsCisConfig& config) noexcept;

    void CopyConfigurationValid(bool& output) const noexcept;
    void CopyLongitudinalAuthorityConfiguration(
        TecsCisLongitudinalAuthorityConfiguration& output,
        Status& status) const noexcept;
    void Reset() noexcept;
    void CopySnapshot(TecsCisSnapshot& output) const noexcept;

    // Pure same-frame projection from the current TECS/CIS integral state.
    // The live owner is not mutated.  next_snapshot exactly matches Step()
    // from the same prestate, including the intentional pre-pitch integral
    // commit on a late rejection.  A selected command is committed only by
    // one subsequent live Step().
    void Preview(
        const BodyRateLoadEnergyCommand& reference,
        const EstimatorOutputV6& estimate,
        const control::route5::CommandEnvelope& envelope,
        double dt_s,
        TecsCisOutput& output,
        TecsCisSnapshot& next_snapshot,
        Status& status) const noexcept;

    void Step(
        const BodyRateLoadEnergyCommand& reference,
        const EstimatorOutputV6& estimate,
        const control::route5::CommandEnvelope& envelope,
        double dt_s,
        TecsCisOutput& output,
        Status& status) noexcept;

private:
    TecsCisConfig config_{};
    bool configuration_valid_ = true;
    double energy_integral_error_m2ps_ = 0.0;
    std::uint64_t accepted_step_count_ = 0U;
};

} // namespace tecs_cis
} // namespace control
} // namespace LadyLuck

static_assert(
    std::is_nothrow_copy_constructible<
        LadyLuck::control::tecs_cis::TecsCisControl>::value,
    "TECS/CIS preview requires allocation-free nothrow value-copy state");
static_assert(
    std::is_trivially_copyable<
        LadyLuck::control::tecs_cis::TecsCisControl>::value,
    "TECS/CIS preview state must remain fixed-size value storage");
