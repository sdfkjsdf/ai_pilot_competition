#include "LadyLuck/control/direct_body/DirectBodyReferenceAdapter.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/Numerics.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double SideslipRegulatorGainPerS = 2.0;
constexpr double YawLoadGain = 0.25;

double Clip(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::max(lower, std::min(value, upper));
}

double YawScheduler(const double ground_speed_fps) noexcept
{
    if (ground_speed_fps <= 80.0)
    {
        return 0.0;
    }
    if (ground_speed_fps < 100.0)
    {
        const double fraction =
            (ground_speed_fps - 80.0) / (100.0 - 80.0);
        return 0.0 + fraction * (15.0 - 0.0);
    }
    if (ground_speed_fps < 150.0)
    {
        const double fraction =
            (ground_speed_fps - 100.0) / (150.0 - 100.0);
        return 15.0 + fraction * (100.0 - 15.0);
    }
    return 100.0;
}

void Fail(
    LadyLuck::control::tecs_cis::BodyRateLoadEnergyCommand& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::control::tecs_cis::BodyRateLoadEnergyCommand{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{
namespace control
{
namespace direct_body
{

void BuildDirectBodyReference(
    const ControlIntent& command,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const route5::CommandEnvelope& envelope,
    const double gamma_command_limit_rad,
    tecs_cis::BodyRateLoadEnergyCommand& output,
    Status& status) noexcept
{
    output = tecs_cis::BodyRateLoadEnergyCommand{};
    status = Status{};
    // The ControlCore validates the immutable intent and current-frame bundle
    // before dispatch.  Re-check only this adapter's route-specific fields.
    if (command.route_kind != ControlRouteKind::DirectBodyReferences
        || !command.direct_p_cmd_radps.has_value
        || !command.direct_nz_cmd_g.has_value
        || command.direct_acceleration_ned_mps2.has_value
        || command.direct_load_vector_acceleration_ned_mps2.has_value
        || command.direct_bank_cmd_rad.has_value
        || command.direct_turn_rate_cmd_radps.has_value
        || command.direct_accel_cmd_mps2.has_value)
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const double requested_p_radps = command.direct_p_cmd_radps.value;
    const double requested_nz_g = command.direct_nz_cmd_g.value;
    const double beta_command_rad = command.direct_beta_cmd_rad.has_value
        ? command.direct_beta_cmd_rad.value
        : 0.0;
    if (!route5::CommandEnvelopeSourceProvidesBounds(envelope.source)
        || !std::isfinite(envelope.nz_feasible_g)
        || !std::isfinite(envelope.p_max_radps)
        || envelope.p_max_radps <= 0.0
        || !std::isfinite(gamma_command_limit_rad)
        || gamma_command_limit_rad <= 0.0
        || !std::isfinite(requested_p_radps)
        || !std::isfinite(requested_nz_g)
        || !std::isfinite(beta_command_rad)
        || !std::isfinite(estimate.V)
        || !std::isfinite(estimate.alpha)
        || !std::isfinite(estimate.beta)
        || !std::isfinite(estimate.roll)
        || !std::isfinite(estimate.pitch)
        || !std::isfinite(estimate.ground_speed_horizontal_mps)
        || !std::isfinite(ownship.position_ned_m[0])
        || !std::isfinite(ownship.position_ned_m[1])
        || !std::isfinite(ownship.position_ned_m[2]))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    // cis_v4_command_backend._loadfactor_route_guidance: direct Nz is capped
    // only at the current physical upper authority; direct p is clipped by the
    // controller's current roll-rate limit.
    const double nz_command_g =
        std::min(requested_nz_g, envelope.nz_feasible_g);
    const double p_command_radps = Clip(
        requested_p_radps,
        -envelope.p_max_radps,
        envelope.p_max_radps);

    const double cosine_pitch = std::cos(estimate.pitch);
    const double cosine_roll = std::cos(estimate.roll);
    const double longitudinal_speed_mps =
        estimate.V * std::cos(estimate.alpha);
    const double q_command_radps =
        (nz_command_g - cosine_pitch * cosine_roll)
        * constants::StandardGravityMps2
        * numerics::RegularizedSignedInverse(
            longitudinal_speed_mps,
            numerics::CisPairLongitudinalSpeedRegularizationMps);

    const double speed_for_coordination_mps =
        std::max(estimate.V, 1.0e-6);
    const double cosine_alpha = std::cos(estimate.alpha);
    const double sine_alpha = std::sin(estimate.alpha);
    const double sine_roll = std::sin(estimate.roll);
    const double sideslip_regulator_radps =
        SideslipRegulatorGainPerS
        * (estimate.beta - beta_command_rad);
    const double inverse_cosine_alpha =
        numerics::RegularizedSignedInverse(
            cosine_alpha,
            numerics::CisPairCosineRegularization);
    const double r_command_radps = (
        p_command_radps * sine_alpha
        + (constants::StandardGravityMps2
            / speed_for_coordination_mps)
            * cosine_pitch * sine_roll
        + sideslip_regulator_radps)
        * inverse_cosine_alpha;
    const double lateral_load_command_g =
        (speed_for_coordination_mps
            / constants::StandardGravityMps2)
        * sideslip_regulator_radps;
    const double ground_speed_horizontal_mps =
        std::max(estimate.ground_speed_horizontal_mps, 0.0);
    const double yaw_scheduler = YawScheduler(
        ground_speed_horizontal_mps
        * constants::MetersToFeet);
    const double effective_r_command_radps = r_command_radps
        + (yaw_scheduler > 1.0e-9
            ? YawLoadGain * lateral_load_command_g / yaw_scheduler
            : 0.0);

    const Vector3 displacement_ned_m{{
        command.aim_point_m[0] - ownship.position_ned_m[0],
        command.aim_point_m[1] - ownship.position_ned_m[1],
        command.aim_point_m[2] - ownship.position_ned_m[2]}};
    const double horizontal_range_m = std::hypot(
        displacement_ned_m[0],
        displacement_ned_m[1]);
    const double raw_gamma_command_rad = std::atan2(
        -displacement_ned_m[2],
        std::max(horizontal_range_m, constants::Epsilon));
    const double gamma_command_rad = Clip(
        raw_gamma_command_rad,
        -gamma_command_limit_rad,
        gamma_command_limit_rad);

    if (!std::isfinite(nz_command_g)
        || !std::isfinite(p_command_radps)
        || !std::isfinite(q_command_radps)
        || !std::isfinite(effective_r_command_radps)
        || !std::isfinite(gamma_command_rad))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    output.frame_identity = command.frame_identity;
    output.valid = true;
    output.p_cmd_radps = p_command_radps;
    output.q_cmd_radps = q_command_radps;
    output.r_cmd_radps = effective_r_command_radps;
    output.nz_cmd_g = nz_command_g;
    output.desired_speed_mps = command.desired_speed_mps;
    output.desired_speed_rate_mps2 = command.desired_speed_rate_mps2;
    output.flight_path_angle_cmd_rad = gamma_command_rad;
    output.specific_energy_rate_bias_m2ps3 =
        command.specific_energy_rate_bias_m2ps3;
}

} // namespace direct_body
} // namespace control
} // namespace LadyLuck
