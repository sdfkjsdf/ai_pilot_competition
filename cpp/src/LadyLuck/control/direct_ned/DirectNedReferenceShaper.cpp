#include "LadyLuck/control/direct_ned/DirectNedReferenceShaper.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/Numerics.hpp"

#include <cmath>

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{
namespace
{

constexpr double Route5DirectRCommandLimitRadps = 0.008;

bool FiniteRawReference(
    const DirectNedLoadVectorOutput& reference) noexcept
{
    const double values[] = {
        reference.p_cmd_radps,
        reference.q_cmd_radps,
        reference.r_cmd_radps,
        reference.nz_cmd_g,
        reference.nz_cmd_raw_g,
        reference.force_perp_norm_g,
        reference.guard_weight,
        reference.direction_error_rad,
        reference.direction_step_angle_rad,
        reference.raw_direction_separation_rad,
        reference.c6_gate_value,
        reference.clip_scale,
        reference.roll_rate_reference_radps,
        reference.roll_rate_equivalent_bank_lead_rad,
        reference.requested_direction_error_rad};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    const bool vectors_finite =
        std::isfinite(reference.target_direction_ned[0])
        && std::isfinite(reference.target_direction_ned[1])
        && std::isfinite(reference.target_direction_ned[2])
        && std::isfinite(reference.raw_direction_ned[0])
        && std::isfinite(reference.raw_direction_ned[1])
        && std::isfinite(reference.raw_direction_ned[2]);
    return vectors_finite;
}

bool FiniteStateForPair(const DirectNedLoadVectorState& state) noexcept
{
    return std::isfinite(state.speed_mps)
        && std::isfinite(state.alpha_rad)
        && std::isfinite(state.pitch_rad)
        && std::isfinite(state.roll_rad)
        && std::isfinite(state.nz_feasible_g)
        && std::isfinite(state.max_p_radps)
        && state.speed_mps >= 0.0
        && state.nz_feasible_g >= 0.0
        && state.max_p_radps > 0.0;
}

bool FiniteLongitudinal(
    const DirectNedLongitudinalReference& reference) noexcept
{
    return std::isfinite(reference.desired_speed_mps)
        && std::isfinite(reference.desired_speed_rate_mps2)
        && std::isfinite(reference.flight_path_angle_cmd_rad)
        && std::isfinite(reference.specific_energy_rate_bias_m2ps3)
        && reference.desired_speed_mps >= 0.0;
}

bool EnvelopeProvidesCommandBounds(
    const route5::CommandEnvelope& envelope) noexcept
{
    return route5::CommandEnvelopeSourceProvidesBounds(envelope.source)
        && std::isfinite(envelope.nz_min_g)
        && std::isfinite(envelope.nz_feasible_g)
        && std::isfinite(envelope.p_max_radps)
        && envelope.nz_min_g <= envelope.nz_feasible_g
        && envelope.p_max_radps > 0.0;
}

double Clip(const double value, const double lower, const double upper) noexcept
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

} // namespace

void DirectNedReferenceShaper::Shape(
    const DirectNedLoadVectorOutput& raw_reference,
    const DirectNedLoadVectorState& state,
    const route5::CommandEnvelope& envelope,
    const DirectNedLongitudinalReference& longitudinal,
    const bool loaded_roll_enabled,
    tecs_cis::BodyRateLoadEnergyCommand& output,
    Status& status) const noexcept
{
    output = tecs_cis::BodyRateLoadEnergyCommand{};
    status = Status{};
    if (!raw_reference.valid || !longitudinal.valid)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!FiniteRawReference(raw_reference)
        || !FiniteStateForPair(state)
        || !FiniteLongitudinal(longitudinal))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!EnvelopeProvidesCommandBounds(envelope))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double p_cmd = Clip(
        raw_reference.p_cmd_radps,
        -envelope.p_max_radps,
        envelope.p_max_radps);
    const double r_cmd = Clip(
        raw_reference.r_cmd_radps,
        -Route5DirectRCommandLimitRadps,
        Route5DirectRCommandLimitRadps);
    // d90 backend applies only the upper command limit here.  CIS owns the
    // later full [nz_min,nz_max] clip.  This preserves source ordering even
    // though an off-domain negative command can then make the later clipped Nz
    // differ from this q-pair reference.
    // Loaded-roll allocation separates two physical actuators that the raw
    // direction servo otherwise couples through C6: aileron rotates the lift
    // axis toward the NED request, while elevator may simultaneously use the
    // already-admitted total-load magnitude. This preserves the requested
    // force magnitude during the roll transient without changing its target
    // direction or exceeding the current Nz envelope.
    const double governed_total_load_g =
        raw_reference.force_perp_norm_g * raw_reference.clip_scale;
    const double nz_source_g = loaded_roll_enabled
        ? governed_total_load_g
        : raw_reference.nz_cmd_g;
    const double nz_cmd = nz_source_g > envelope.nz_feasible_g
        ? envelope.nz_feasible_g
        : nz_source_g;
    const double gravity_projection =
        std::cos(state.pitch_rad) * std::cos(state.roll_rad);
    const double longitudinal_speed_mps =
        state.speed_mps * std::cos(state.alpha_rad);
    const double q_cmd = (nz_cmd - gravity_projection)
        * constants::StandardGravityMps2
        * numerics::RegularizedSignedInverse(
            longitudinal_speed_mps,
            numerics::CisPairLongitudinalSpeedRegularizationMps);

    const double shaped_values[] = {p_cmd, q_cmd, r_cmd, nz_cmd};
    for (const double value : shaped_values)
    {
        if (!std::isfinite(value))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }

    output.frame_identity = raw_reference.frame_identity;
    output.p_cmd_radps = p_cmd;
    output.q_cmd_radps = q_cmd;
    output.r_cmd_radps = r_cmd;
    output.nz_cmd_g = nz_cmd;
    output.desired_speed_mps = longitudinal.desired_speed_mps;
    output.desired_speed_rate_mps2 =
        longitudinal.desired_speed_rate_mps2;
    output.flight_path_angle_cmd_rad =
        longitudinal.flight_path_angle_cmd_rad;
    output.specific_energy_rate_bias_m2ps3 =
        longitudinal.specific_energy_rate_bias_m2ps3;
    output.integrator_hold = longitudinal.integrator_hold;
    output.valid = true;
    status.code = StatusCode::Ok;
}

} // namespace direct_ned
} // namespace control
} // namespace LadyLuck
