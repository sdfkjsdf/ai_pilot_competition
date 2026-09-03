#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/control/route5/AimPointVectorGuidance.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/control/route5/DirectLoadVectorNMuAdapter.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace control
{
namespace route5
{

enum class VelocityBankSource : std::int32_t
{
    Unavailable = 0,
    QuaternionAttitude = 1
};

struct Route5GuidanceConfig
{
    double k_roll_to_chi = 0.25;
    double k_pitch_to_gamma = 0.20;
    double default_k_roll = 1.8;
    double default_k_pitch = 1.8;
    double k_chi_min = 0.10;
    double k_chi_max = 0.80;
    double k_gamma_min = 0.08;
    double k_gamma_max = 0.65;
    double chi_rate_max_radps = 0.30;
    double gamma_rate_max_radps = 0.18;
    double gamma_cmd_limit_rad = 0.6108652381980153; // 35 deg
    double acceleration_filter_tau_s = 0.1;
    double bank_direction_gate_g = 0.2;
    bool allow_inverted_default = true;
    double n_cmd_min_g = 0.05;
    double n_cmd_max_g = 6.0;
    double r_cmd_max_radps = 0.008;
    double beta_cmd_rad = 0.0;
    double speed_bias_gain_mps = 40.0;
    double direct_accel_min_mps2 = -10.0;
    double direct_accel_max_mps2 = 25.0;
    double legacy_speed_error_gain_per_s = 0.45;
    double velocity_bank_wn_radps = 6.0;
    double velocity_bank_zeta = 1.0;
    double velocity_bank_rate_max_radps = 3.1416;
    double velocity_bank_error_gain_per_s = 4.0;
};

struct Route5GuidanceOutput
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double n_cmd_raw_g = 0.0;
    double n_cmd_g = 0.0;
    double n_cmd_limit_g = 0.0;
    double mu_cmd_rad = 0.0;
    double p_cmd_raw_radps = 0.0;
    double p_cmd_radps = 0.0;
    double q_cmd_radps = 0.0;
    double r_cmd_raw_radps = 0.0;
    double r_cmd_radps = 0.0;
    double nz_cmd_g = 0.0;
    double beta_cmd_rad = 0.0;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double flight_path_angle_cmd_rad = 0.0;
    double specific_energy_rate_bias_m2ps3 = 0.0;
    double legacy_accel_reference_mps2 = 0.0;
    bool legacy_accel_reference_active = false;
    double k_chi_per_s = 0.0;
    double k_gamma_per_s = 0.0;
    bool allow_inverted = true;
    bool n_cmd_limited = false;
    bool p_cmd_limited = false;
    bool r_cmd_limited = false;
    VelocityBankSource velocity_bank_source = VelocityBankSource::Unavailable;
    Vector3 specific_force_wind_g{};
    Vector3 specific_force_ned_g{};
};

struct Route5GuidanceSnapshot
{
    bool aim_point_vector_mode_active = false;
    AimPointVectorGuidanceSnapshot aim_point_vector{};
    bool nmu_filter_initialized = false;
    double filtered_ay_g = 0.0;
    double filtered_az_g = 0.0;
    double continuous_mu_cmd_rad = 0.0;
    bool velocity_bank_initialized = false;
    double filtered_mu_rad = 0.0;
    double filtered_mu_rate_radps = 0.0;
    bool last_mu_observation_valid = false;
    double last_mu_observation_rad = 0.0;
    bool last_mu_error_valid = false;
    double last_mu_error_rad = 0.0;
    VelocityBankSource last_mu_source = VelocityBankSource::Unavailable;
};

// Stateful NMu path-angle plus LoadVector2Cis Route-5 guidance subset.
// State changes are deliberately non-atomic, matching Python's staged mutation.
class Route5Guidance final
{
public:
    Route5Guidance() noexcept;
    explicit Route5Guidance(const Route5GuidanceConfig& config) noexcept;

    void CopyConfigurationValid(bool& output) const noexcept;
    void CopyGammaCommandLimit(
        double& output_rad,
        Status& status) const noexcept;
    void CopyGammaRateLimit(
        double& output_radps,
        Status& status) const noexcept;
    void Reset() noexcept;
    void CopySnapshot(Route5GuidanceSnapshot& output) const noexcept;

    // Pure same-frame projection from the current controller state.  The
    // live Route-5 owner is not mutated.  next_snapshot is the exact state a
    // Step() on this same prestate would leave behind, including Python's
    // intentionally staged partial commits on a late rejection.  Selection
    // code must call Step() on the live owner exactly once to commit.
    void Preview(
        const ControlIntent& command,
        const PlaneState& ownship,
        const EstimatorOutputV6& estimate,
        const CommandEnvelope& envelope,
        double dt_s,
        Route5GuidanceOutput& output,
        Route5GuidanceSnapshot& next_snapshot,
        Status& status) const noexcept;

    void Step(
        const ControlIntent& command,
        const PlaneState& ownship,
        const EstimatorOutputV6& estimate,
        const CommandEnvelope& envelope,
        double dt_s,
        Route5GuidanceOutput& output,
        Status& status) noexcept;

private:
    Route5GuidanceConfig config_{};
    bool configuration_valid_ = true;
    bool aim_point_vector_mode_active_ = false;
    AimPointVectorGuidance aim_point_vector_guidance_{};
    DirectLoadVectorNMuAdapter direct_load_vector_adapter_{};
    bool nmu_filter_initialized_ = false;
    double filtered_ay_g_ = 0.0;
    double filtered_az_g_ = 0.0;
    double continuous_mu_cmd_rad_ = 0.0;
    bool velocity_bank_initialized_ = false;
    double filtered_mu_rad_ = 0.0;
    double filtered_mu_rate_radps_ = 0.0;
    bool last_mu_observation_valid_ = false;
    double last_mu_observation_rad_ = 0.0;
    bool last_mu_error_valid_ = false;
    double last_mu_error_rad_ = 0.0;
    VelocityBankSource last_mu_source_ = VelocityBankSource::Unavailable;
};

} // namespace route5
} // namespace control
} // namespace LadyLuck

static_assert(
    std::is_nothrow_copy_constructible<
        LadyLuck::control::route5::Route5Guidance>::value,
    "Route-5 preview requires allocation-free nothrow value-copy state");
static_assert(
    std::is_trivially_copyable<
        LadyLuck::control::route5::Route5Guidance>::value,
    "Route-5 preview state must remain fixed-size value storage");
