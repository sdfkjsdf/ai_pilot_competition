#include "LadyLuck/control/route5/Route5Guidance.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/Numerics.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace
{
using LadyLuck::Matrix3RowMajor;
using LadyLuck::PlaneState;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::ControlFrameIdentity;
using LadyLuck::ControlIntent;
using LadyLuck::SameControlFrameIdentity;
using LadyLuck::Vector3;
using LadyLuck::control::route5::CommandEnvelope;
using LadyLuck::control::route5::Route5GuidanceConfig;
using LadyLuck::control::route5::StallSpeedBoundarySource;

Status Failure(const StatusCode code) noexcept
{
    Status status{};
    status.code = code;
    return status;
}

double Clamp(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::min(upper, std::max(lower, value));
}

double WrapNmuModulo(const double value) noexcept
{
    const double two_pi = 2.0 * LadyLuck::Pi;
    double remainder = std::fmod(value + LadyLuck::Pi, two_pi);
    if (remainder < 0.0)
    {
        remainder += two_pi;
    }
    return remainder - LadyLuck::Pi;
}

double WrapVelocityBankAtan2(const double value) noexcept
{
    return std::atan2(std::sin(value), std::cos(value));
}

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

Matrix3RowMajor DirectEulerDcmNedToBody(const Vector3& rpy_rad) noexcept
{
    // engagement_geometry.velocity_ned uses (Tx @ Ty) @ Tz directly.  Keep
    // that distinct from the quaternion DCM used by mu_from_attitude below:
    // the two rotations are physically equivalent but not bit-equivalent.
    const double cosine_roll = std::cos(rpy_rad[0]);
    const double sine_roll = std::sin(rpy_rad[0]);
    const double cosine_pitch = std::cos(rpy_rad[1]);
    const double sine_pitch = std::sin(rpy_rad[1]);
    const double cosine_yaw = std::cos(rpy_rad[2]);
    const double sine_yaw = std::sin(rpy_rad[2]);
    const Matrix3RowMajor roll_matrix{{
        1.0, 0.0, 0.0,
        0.0, cosine_roll, sine_roll,
        0.0, -sine_roll, cosine_roll}};
    const Matrix3RowMajor pitch_matrix{{
        cosine_pitch, 0.0, -sine_pitch,
        0.0, 1.0, 0.0,
        sine_pitch, 0.0, cosine_pitch}};
    const Matrix3RowMajor yaw_matrix{{
        cosine_yaw, sine_yaw, 0.0,
        -sine_yaw, cosine_yaw, 0.0,
        0.0, 0.0, 1.0}};
    return LadyLuck::MatrixProduct(
        LadyLuck::MatrixProduct(roll_matrix, pitch_matrix),
        yaw_matrix);
}

bool ConfigurationValid(const Route5GuidanceConfig& config) noexcept
{
    const double finite_values[] = {
        config.k_roll_to_chi,
        config.k_pitch_to_gamma,
        config.default_k_roll,
        config.default_k_pitch,
        config.k_chi_min,
        config.k_chi_max,
        config.k_gamma_min,
        config.k_gamma_max,
        config.chi_rate_max_radps,
        config.gamma_rate_max_radps,
        config.gamma_cmd_limit_rad,
        config.acceleration_filter_tau_s,
        config.bank_direction_gate_g,
        config.n_cmd_min_g,
        config.n_cmd_max_g,
        config.r_cmd_max_radps,
        config.beta_cmd_rad,
        config.speed_bias_gain_mps,
        config.direct_accel_min_mps2,
        config.direct_accel_max_mps2,
        config.legacy_speed_error_gain_per_s,
        config.velocity_bank_wn_radps,
        config.velocity_bank_zeta,
        config.velocity_bank_rate_max_radps,
        config.velocity_bank_error_gain_per_s};
    for (const double value : finite_values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return config.k_chi_min <= config.k_chi_max
        && config.k_gamma_min <= config.k_gamma_max
        && config.chi_rate_max_radps >= 0.0
        && config.gamma_rate_max_radps >= 0.0
        && config.gamma_cmd_limit_rad >= 0.0
        && config.acceleration_filter_tau_s >= 0.0
        && config.bank_direction_gate_g >= 0.0
        && config.n_cmd_min_g >= 0.0
        && config.n_cmd_max_g >= config.n_cmd_min_g
        && config.r_cmd_max_radps >= 0.0
        && config.direct_accel_min_mps2
            <= config.direct_accel_max_mps2
        && config.velocity_bank_wn_radps >= 0.0
        && config.velocity_bank_zeta >= 0.0
        && config.velocity_bank_rate_max_radps >= 0.0;
}

Status ValidateCommandAndContext(
    const ControlIntent& command,
    const PlaneState& ownship,
    const LadyLuck::EstimatorOutputV6& estimate,
    const CommandEnvelope& envelope,
    const double dt_s,
    Vector3& velocity_ned_mps) noexcept
{
    if (!std::isfinite(dt_s) || dt_s <= 0.0)
    {
        return Failure(
            std::isfinite(dt_s)
                ? StatusCode::InvalidDt
                : StatusCode::NonFiniteInput);
    }
    if (!FiniteVector(command.aim_point_m)
        || !FiniteVector(ownship.position_ned_m)
        || !FiniteVector(ownship.rpy_rad)
        || !FiniteVector(ownship.velocity_body_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    if (!std::isfinite(command.desired_speed_mps)
        || (command.k_roll.has_value
            && !std::isfinite(command.k_roll.value))
        || (command.k_pitch.has_value
            && !std::isfinite(command.k_pitch.value))
        || (command.throttle_bias.has_value
            && !std::isfinite(command.throttle_bias.value)))
    {
        return Failure(StatusCode::NonFiniteInput);
    }

    const double estimate_values[] = {
        estimate.u,
        estimate.v,
        estimate.w,
        estimate.V,
        estimate.alt,
        estimate.roll,
        estimate.pitch,
        estimate.yaw,
        estimate.alpha,
        estimate.beta,
        estimate.mu,
        estimate.mass,
        estimate.gear_pos_norm,
        estimate.ground_speed_horizontal_mps};
    for (const double value : estimate_values)
    {
        if (!std::isfinite(value))
        {
            return Failure(StatusCode::NonFiniteInput);
        }
    }
    if (estimate.V <= 0.0 || estimate.mass <= 0.0)
    {
        return Failure(StatusCode::InvalidArgument);
    }
    const double envelope_values[] = {
        envelope.nz_feasible_g,
        envelope.nz_min_g,
        envelope.p_max_radps};
    for (const double value : envelope_values)
    {
        if (!std::isfinite(value))
        {
            return Failure(StatusCode::NonFiniteInput);
        }
    }
    if (!CommandEnvelopeSourceProvidesBounds(envelope.source)
        || envelope.nz_feasible_g <= 0.0
        || envelope.nz_min_g > envelope.nz_feasible_g
        || envelope.p_max_radps <= 0.0)
    {
        return Failure(StatusCode::InvalidConfiguration);
    }
    // The source is the governor's causal authority discriminator.  Do not
    // reconstruct it from duplicated flags or unrelated stall/roll metadata.
    // Do not recompute estimator/envelope derivatives here and compare them
    // with near-bit equality: they are representations of the same accepted
    // transaction, not independent sensors.  Route-5 only computes the NED
    // velocity required by its guidance law.
    const Matrix3RowMajor velocity_dcm_ned_to_body =
        DirectEulerDcmNedToBody(ownship.rpy_rad);
    velocity_ned_mps = LadyLuck::TransposeMatrixVectorProduct(
        velocity_dcm_ned_to_body,
        ownship.velocity_body_mps);
    if (!FiniteVector(velocity_ned_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    return Status{};
}

double YawScheduler(const double ground_speed_fps) noexcept
{
    if (ground_speed_fps <= 80.0)
    {
        return 0.0;
    }
    if (ground_speed_fps <= 100.0)
    {
        return 15.0 * (ground_speed_fps - 80.0) / 20.0;
    }
    if (ground_speed_fps <= 150.0)
    {
        return 15.0
            + 85.0 * (ground_speed_fps - 100.0) / 50.0;
    }
    return 100.0;
}
}

namespace LadyLuck
{
namespace control
{
namespace route5
{

Route5Guidance::Route5Guidance() noexcept
    : Route5Guidance(Route5GuidanceConfig{})
{
}

Route5Guidance::Route5Guidance(
    const Route5GuidanceConfig& config) noexcept
    : config_(config),
      configuration_valid_(ConfigurationValid(config))
{
    Reset();
}

void Route5Guidance::CopyConfigurationValid(bool& output) const noexcept
{
    output = configuration_valid_;
}

void Route5Guidance::CopyGammaCommandLimit(
    double& output_rad,
    Status& status) const noexcept
{
    output_rad = 0.0;
    status = Status{};
    if (!configuration_valid_
        || !std::isfinite(config_.gamma_cmd_limit_rad)
        || config_.gamma_cmd_limit_rad <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output_rad = config_.gamma_cmd_limit_rad;
}

void Route5Guidance::CopyGammaRateLimit(
    double& output_radps,
    Status& status) const noexcept
{
    output_radps = 0.0;
    status = Status{};
    if (!configuration_valid_
        || !std::isfinite(config_.gamma_rate_max_radps)
        || config_.gamma_rate_max_radps < 0.0)
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }
    output_radps = config_.gamma_rate_max_radps;
}

void Route5Guidance::Reset() noexcept
{
    aim_point_vector_mode_active_ = false;
    aim_point_vector_guidance_.Reset();
    nmu_filter_initialized_ = false;
    filtered_ay_g_ = 0.0;
    filtered_az_g_ = 0.0;
    continuous_mu_cmd_rad_ = 0.0;
    velocity_bank_initialized_ = false;
    filtered_mu_rad_ = 0.0;
    filtered_mu_rate_radps_ = 0.0;
    last_mu_observation_valid_ = false;
    last_mu_observation_rad_ = 0.0;
    last_mu_error_valid_ = false;
    last_mu_error_rad_ = 0.0;
    last_mu_source_ = VelocityBankSource::Unavailable;
}

void Route5Guidance::CopySnapshot(
    Route5GuidanceSnapshot& output) const noexcept
{
    output = Route5GuidanceSnapshot{};
    output.aim_point_vector_mode_active =
        aim_point_vector_mode_active_;
    aim_point_vector_guidance_.CopySnapshot(
        output.aim_point_vector);
    output.nmu_filter_initialized = nmu_filter_initialized_;
    output.filtered_ay_g = filtered_ay_g_;
    output.filtered_az_g = filtered_az_g_;
    output.continuous_mu_cmd_rad = continuous_mu_cmd_rad_;
    output.velocity_bank_initialized = velocity_bank_initialized_;
    output.filtered_mu_rad = filtered_mu_rad_;
    output.filtered_mu_rate_radps = filtered_mu_rate_radps_;
    output.last_mu_observation_valid = last_mu_observation_valid_;
    output.last_mu_observation_rad = last_mu_observation_rad_;
    output.last_mu_error_valid = last_mu_error_valid_;
    output.last_mu_error_rad = last_mu_error_rad_;
    output.last_mu_source = last_mu_source_;
}

void Route5Guidance::Preview(
    const ControlIntent& command,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const CommandEnvelope& envelope,
    const double dt_s,
    Route5GuidanceOutput& output,
    Route5GuidanceSnapshot& next_snapshot,
    Status& status) const noexcept
{
    Route5Guidance projected(*this);
    projected.Step(
        command,
        ownship,
        estimate,
        envelope,
        dt_s,
        output,
        status);
    projected.CopySnapshot(next_snapshot);
}

void Route5Guidance::Step(
    const ControlIntent& command,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const CommandEnvelope& envelope,
    const double dt_s,
    Route5GuidanceOutput& output,
    Status& status) noexcept
{
    output = Route5GuidanceOutput{};
    status = Status{};
    if (!configuration_valid_)
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }

    Vector3 velocity_ned_mps{};
    status = ValidateCommandAndContext(
        command,
        ownship,
        estimate,
        envelope,
        dt_s,
        velocity_ned_mps);
    if (!status.ok())
    {
        return;
    }

    const double k_roll = command.k_roll.has_value
        ? command.k_roll.value
        : config_.default_k_roll;
    const double k_pitch = command.k_pitch.has_value
        ? command.k_pitch.value
        : config_.default_k_pitch;
    const double k_chi = Clamp(
        config_.k_roll_to_chi * k_roll,
        config_.k_chi_min,
        config_.k_chi_max);
    const double k_gamma = Clamp(
        config_.k_pitch_to_gamma * k_pitch,
        config_.k_gamma_min,
        config_.k_gamma_max);
    const bool allow_inverted = command.path_inversion_allowed.has_value
        ? command.path_inversion_allowed.value
        : config_.allow_inverted_default;

    const Vector3 rho{{
        command.aim_point_m[0] - ownship.position_ned_m[0],
        command.aim_point_m[1] - ownship.position_ned_m[1],
        command.aim_point_m[2] - ownship.position_ned_m[2]}};
    const double course_rad = std::atan2(
        velocity_ned_mps[1],
        velocity_ned_mps[0]);
    const double horizontal_speed = std::hypot(
        velocity_ned_mps[0],
        velocity_ned_mps[1]);
    const double path_rad = std::atan2(
        -velocity_ned_mps[2],
        horizontal_speed);
    const double rho_horizontal = std::hypot(rho[0], rho[1]);
    const double path_cmd_rad = Clamp(
        std::atan2(-rho[2], std::max(rho_horizontal, constants::Epsilon)),
        -config_.gamma_cmd_limit_rad,
        config_.gamma_cmd_limit_rad);
    double pending_ax_wind_g = 0.0;
    double acceleration_y_g = 0.0;
    double acceleration_z_g = 0.0;
    if (command.route_kind == ControlRouteKind::AimPoint)
    {
        if (!aim_point_vector_mode_active_)
        {
            aim_point_vector_guidance_.Reset();
            aim_point_vector_mode_active_ = true;
        }
        AimPointVectorGuidanceOutput vector_guidance{};
        aim_point_vector_guidance_.Step(
            ownship.position_ned_m,
            command.aim_point_m,
            velocity_ned_mps,
            config_.gamma_cmd_limit_rad,
            dt_s,
            vector_guidance);
        if (!vector_guidance.valid)
        {
            status = Failure(
                vector_guidance.reason
                        == AimPointVectorGuidanceReason::HeldInvalidInput
                    ? StatusCode::NonFiniteInput
                    : StatusCode::InvalidArgument);
            return;
        }

        ControlIntent acceleration_command = command;
        acceleration_command.route_kind =
            ControlRouteKind::DirectLoadVectorAcceleration;
        acceleration_command.direct_load_vector_acceleration_ned_mps2
            .has_value = true;
        acceleration_command.direct_load_vector_acceleration_ned_mps2
            .value = vector_guidance.acceleration_ned_mps2;
        DirectLoadVectorNMuInput vector_input{};
        direct_load_vector_adapter_.Prepare(
            acceleration_command,
            velocity_ned_mps,
            vector_input,
            status);
        if (!status.ok() || !vector_input.valid)
        {
            if (status.ok())
            {
                status = Failure(StatusCode::InvalidConfiguration);
            }
            return;
        }
        pending_ax_wind_g = vector_input.specific_force_wind_g[0];
        acceleration_y_g = vector_input.specific_force_wind_g[1];
        acceleration_z_g = vector_input.specific_force_wind_g[2];
    }
    else if (command.route_kind
        == ControlRouteKind::DirectLoadVectorAcceleration)
    {
        aim_point_vector_mode_active_ = false;
        DirectLoadVectorNMuInput direct_input{};
        direct_load_vector_adapter_.Prepare(
            command,
            velocity_ned_mps,
            direct_input,
            status);
        if (!status.ok() || !direct_input.valid)
        {
            if (status.ok())
            {
                status = Failure(StatusCode::InvalidConfiguration);
            }
            return;
        }
        if (!SameControlFrameIdentity(
                direct_input.frame_identity,
                command.frame_identity))
        {
            status = Failure(StatusCode::InvalidConfiguration);
            return;
        }
        pending_ax_wind_g = direct_input.specific_force_wind_g[0];
        acceleration_y_g = direct_input.specific_force_wind_g[1];
        acceleration_z_g = direct_input.specific_force_wind_g[2];
    }
    else
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }
    if (!std::isfinite(pending_ax_wind_g)
        || !std::isfinite(acceleration_y_g)
        || !std::isfinite(acceleration_z_g))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }

    // Python NMuFilter commits these LPF states before it constructs the force
    // transform or observes velocity-bank.  Do not add transactional rollback.
    const double filter_alpha = config_.acceleration_filter_tau_s <= 0.0
        ? 1.0
        : dt_s / (config_.acceleration_filter_tau_s + dt_s);
    if (!nmu_filter_initialized_)
    {
        filtered_ay_g_ = acceleration_y_g;
        filtered_az_g_ = acceleration_z_g;
        nmu_filter_initialized_ = true;
    }
    else
    {
        filtered_ay_g_ += filter_alpha * (acceleration_y_g - filtered_ay_g_);
        filtered_az_g_ += filter_alpha * (acceleration_z_g - filtered_az_g_);
    }

    const double effective_ay_g = filtered_ay_g_;
    const double effective_az_g = allow_inverted
        ? filtered_az_g_
        : std::min(filtered_az_g_, 0.0);
    const double cosine_course = std::cos(course_rad);
    const double sine_course = std::sin(course_rad);
    const double cosine_path = std::cos(path_rad);
    const double sine_path = std::sin(path_rad);
    const Matrix3RowMajor dcm_wind_to_ned{{
        cosine_course * cosine_path,
        -sine_course,
        cosine_course * sine_path,
        sine_course * cosine_path,
        cosine_course,
        sine_course * sine_path,
        -sine_path,
        0.0,
        cosine_path}};
    const Vector3 force_wind_g{{
        pending_ax_wind_g,
        effective_ay_g,
        effective_az_g}};
    const Vector3 force_ned_g = MatrixVectorProduct(
        dcm_wind_to_ned,
        force_wind_g);
    const double n_cmd_raw_g = std::hypot(effective_ay_g, effective_az_g);
    const double bank_rad = std::atan2(effective_ay_g, -effective_az_g);
    if (!FiniteVector(force_ned_g)
        || !std::isfinite(n_cmd_raw_g)
        || !std::isfinite(bank_rad))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }
    if (n_cmd_raw_g >= config_.bank_direction_gate_g)
    {
        continuous_mu_cmd_rad_ += WrapNmuModulo(
            bank_rad - continuous_mu_cmd_rad_);
    }
    if (!std::isfinite(continuous_mu_cmd_rad_))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }

    // EstimatorOutputV6.mu is the canonical, same-frame quaternion velocity-
    // bank observation.  Route-5 consumes it instead of independently
    // re-deriving and near-bit comparing the same physical quantity.
    const double velocity_bank_observation = estimate.mu;
    if (!std::isfinite(velocity_bank_observation))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }

    if (!velocity_bank_initialized_)
    {
        filtered_mu_rad_ = velocity_bank_observation;
        filtered_mu_rate_radps_ = 0.0;
        velocity_bank_initialized_ = true;
    }
    const double mu_acceleration_radps2 =
        config_.velocity_bank_wn_radps * config_.velocity_bank_wn_radps
            * WrapVelocityBankAtan2(
                continuous_mu_cmd_rad_ - filtered_mu_rad_)
        - 2.0 * config_.velocity_bank_zeta
            * config_.velocity_bank_wn_radps * filtered_mu_rate_radps_;
    filtered_mu_rate_radps_ = Clamp(
        filtered_mu_rate_radps_ + mu_acceleration_radps2 * dt_s,
        -config_.velocity_bank_rate_max_radps,
        config_.velocity_bank_rate_max_radps);
    filtered_mu_rad_ = WrapVelocityBankAtan2(
        filtered_mu_rad_ + filtered_mu_rate_radps_ * dt_s);
    const double mu_error_rad = WrapVelocityBankAtan2(
        filtered_mu_rad_ - velocity_bank_observation);
    last_mu_observation_valid_ = true;
    last_mu_observation_rad_ = velocity_bank_observation;
    last_mu_error_valid_ = true;
    last_mu_error_rad_ = mu_error_rad;
    last_mu_source_ = VelocityBankSource::QuaternionAttitude;
    // The accepted current-frame envelope and TECS/CIS own the physical roll
    // limit.  Route-5 must not impose a second lower cap on any maneuver.
    const double controller_p_limit = std::min(
        1.0 / 0.31821,
        envelope.p_max_radps);
    const double p_cmd_raw_radps = Clamp(
        filtered_mu_rate_radps_
            + config_.velocity_bank_error_gain_per_s * mu_error_rad,
        -controller_p_limit,
        controller_p_limit);
    const double p_cmd_radps = p_cmd_raw_radps;

    double n_upper_g = std::min(
        config_.n_cmd_max_g,
        envelope.nz_feasible_g);
    if (command.total_load_factor_limit_g.has_value)
    {
        n_upper_g = std::min(
            n_upper_g,
            command.total_load_factor_limit_g.value);
    }
    const double n_cmd_g = Clamp(
        n_cmd_raw_g,
        config_.n_cmd_min_g,
        n_upper_g);
    // Do not apply the full maneuver load before the aircraft lift axis has
    // rolled toward the requested acceleration direction.  The direct-NED
    // controller uses the same cosine (C6) projection: at large direction
    // error it holds the current gravity-compensating load, then restores the
    // requested load continuously as bank alignment develops.  Without this
    // projection a horizontal AimPoint turn can initially become a pull-up.
    const double gravity_projection =
        std::cos(estimate.pitch) * std::cos(estimate.roll);
    const double lift_direction_error_rad = WrapVelocityBankAtan2(
        bank_rad - velocity_bank_observation);
    const double lift_direction_alignment = Clamp(
        std::cos(std::fabs(lift_direction_error_rad)),
        0.0,
        1.0);
    const double aligned_nz_cmd_g = n_cmd_g * std::cos(estimate.alpha);
    const double nz_cmd_g =
        lift_direction_alignment * aligned_nz_cmd_g
        + (1.0 - lift_direction_alignment) * gravity_projection;
    const double longitudinal_speed_mps =
        estimate.V * std::cos(estimate.alpha);
    const double q_cmd_radps = (nz_cmd_g
            - gravity_projection)
        * constants::StandardGravityMps2
        * numerics::RegularizedSignedInverse(
            longitudinal_speed_mps,
            numerics::CisPairLongitudinalSpeedRegularizationMps);

    const double speed = std::max(estimate.V, 1.0e-6);
    const double cosine_alpha = std::cos(estimate.alpha);
    const double sine_alpha = std::sin(estimate.alpha);
    const double coordination_error = 2.0
        * (estimate.beta - config_.beta_cmd_rad);
    const double inverse_cosine_alpha =
        numerics::RegularizedSignedInverse(
            cosine_alpha,
            numerics::CisPairCosineRegularization);
    double r_cmd_raw_radps = (
        p_cmd_radps * sine_alpha
        + (constants::StandardGravityMps2 / speed)
            * std::cos(estimate.pitch) * std::sin(estimate.roll)
        + coordination_error) * inverse_cosine_alpha;
    const double ny_cmd_g = speed / constants::StandardGravityMps2
        * coordination_error;
    const double ground_speed = std::max(
        estimate.ground_speed_horizontal_mps,
        0.0);
    const double yaw_gain = YawScheduler(
        ground_speed * constants::MetersToFeet);
    if (yaw_gain > 1.0e-9)
    {
        r_cmd_raw_radps += 0.25 * ny_cmd_g / yaw_gain;
    }
    const double r_cmd_radps = Clamp(
        r_cmd_raw_radps,
        -config_.r_cmd_max_radps,
        config_.r_cmd_max_radps);

    const double throttle_bias = command.throttle_bias.has_value
        ? command.throttle_bias.value
        : 0.0;
    const double desired_speed_mps = command.desired_speed_mps
        + config_.speed_bias_gain_mps * throttle_bias;
    const double direct_accel_mps2 = command.direct_accel_cmd_mps2.has_value
        ? Clamp(
            command.direct_accel_cmd_mps2.value,
            config_.direct_accel_min_mps2,
            config_.direct_accel_max_mps2)
        : 0.0;
    const double specific_energy_rate_bias_m2ps3 =
        command.specific_energy_rate_bias_m2ps3
        + std::max(estimate.V, 1.0e-6) * direct_accel_mps2;
    const double legacy_accel_reference_mps2 =
        config_.legacy_speed_error_gain_per_s
        * (desired_speed_mps - estimate.V);
    const double final_values[] = {
        p_cmd_raw_radps,
        p_cmd_radps,
        q_cmd_radps,
        r_cmd_raw_radps,
        r_cmd_radps,
        n_upper_g,
        n_cmd_g,
        nz_cmd_g,
        lift_direction_error_rad,
        lift_direction_alignment,
        desired_speed_mps,
        command.desired_speed_rate_mps2,
        path_cmd_rad,
        specific_energy_rate_bias_m2ps3,
        legacy_accel_reference_mps2};
    for (const double value : final_values)
    {
        if (!std::isfinite(value))
        {
            // NMu and velocity-bank states intentionally remain committed.
            status = Failure(StatusCode::NonFiniteInput);
            return;
        }
    }
    if (desired_speed_mps < 0.0)
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }

    output.frame_identity = command.frame_identity;
    output.valid = true;
    output.n_cmd_raw_g = n_cmd_raw_g;
    output.n_cmd_g = n_cmd_g;
    output.n_cmd_limit_g = n_upper_g;
    output.mu_cmd_rad = continuous_mu_cmd_rad_;
    output.p_cmd_raw_radps = p_cmd_raw_radps;
    output.p_cmd_radps = p_cmd_radps;
    output.q_cmd_radps = q_cmd_radps;
    output.r_cmd_raw_radps = r_cmd_raw_radps;
    output.r_cmd_radps = r_cmd_radps;
    output.nz_cmd_g = nz_cmd_g;
    output.beta_cmd_rad = config_.beta_cmd_rad;
    output.desired_speed_mps = desired_speed_mps;
    output.desired_speed_rate_mps2 = command.desired_speed_rate_mps2;
    output.flight_path_angle_cmd_rad = path_cmd_rad;
    output.specific_energy_rate_bias_m2ps3 =
        specific_energy_rate_bias_m2ps3;
    output.legacy_accel_reference_mps2 = legacy_accel_reference_mps2;
    output.legacy_accel_reference_active = false;
    output.k_chi_per_s = k_chi;
    output.k_gamma_per_s = k_gamma;
    output.allow_inverted = allow_inverted;
    output.n_cmd_limited = std::fabs(n_cmd_g - n_cmd_raw_g) > 1.0e-9;
    output.p_cmd_limited = std::fabs(p_cmd_radps - p_cmd_raw_radps) > 1.0e-9;
    output.r_cmd_limited = std::fabs(r_cmd_radps - r_cmd_raw_radps) > 1.0e-9;
    output.velocity_bank_source = VelocityBankSource::QuaternionAttitude;
    output.specific_force_wind_g = force_wind_g;
    output.specific_force_ned_g = force_ned_g;
    status = Status{};
}

} // namespace route5
} // namespace control
} // namespace LadyLuck
