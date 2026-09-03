#include "LadyLuck/plant/core/AuxiliaryState.hpp"

#include <algorithm>
#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace core
{

namespace
{

constexpr double GravityMps2 = 9.80665;

bool IsFiniteInput(const FbwInput& input) noexcept
{
    for (const double command : input.command)
    {
        if (!std::isfinite(command))
        {
            return false;
        }
    }
    const double scalars[] = {
        input.p_rad_s, input.q_rad_s, input.r_rad_s, input.alpha_rad,
        input.roll_rad, input.pitch_rad, input.true_airspeed_mps,
        input.mach, input.dt_s, input.pitch_trim, input.yaw_trim,
        input.ground_speed_fps, input.local_pressure_pa,
        input.gear_position_normalized, input.weight_on_wheels
    };
    for (const double scalar : scalars)
    {
        if (!std::isfinite(scalar))
        {
            return false;
        }
    }
    return input.dt_s > 0.0;
}

} // namespace

AuxiliaryStateModel::AuxiliaryStateModel() noexcept = default;

AuxState AuxiliaryStateModel::InitialState(const double mass_kg) const noexcept
{
    AuxState state;
    state.mass_kg = mass_kg;
    state.pitch = pitch_.InitialState(0.0);
    return state;
}

PlantResult<double> AuxiliaryStateModel::CalibratedAirspeedKts(
    const double mach,
    const double local_pressure_pa) noexcept
{
    PlantResult<double> result;
    if (!std::isfinite(mach) || !std::isfinite(local_pressure_pa)
        || local_pressure_pa < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "calibrated-airspeed input must be finite with nonnegative pressure");
        return result;
    }
    const double magnitude_mach = std::abs(mach);
    if (magnitude_mach <= 0.0)
    {
        result.value = 0.0;
        result.status = PlantStatus::Success();
        return result;
    }
    const double local_pressure_psf = local_pressure_pa / 47.880259;
    double impact_pressure_psf = 0.0;
    if (magnitude_mach < 1.0)
    {
        impact_pressure_psf = local_pressure_psf
            * (std::pow(1.0 + 0.2 * magnitude_mach * magnitude_mach, 3.5)
                - 1.0);
    }
    else
    {
        constexpr double HeatCapacityRatio = 1.4;
        const double pt_over_p = std::pow(
            (HeatCapacityRatio + 1.0) * magnitude_mach * magnitude_mach / 2.0,
            HeatCapacityRatio / (HeatCapacityRatio - 1.0))
            * std::pow(
                (HeatCapacityRatio + 1.0)
                    / (2.0 * HeatCapacityRatio * magnitude_mach * magnitude_mach
                        - (HeatCapacityRatio - 1.0)),
                1.0 / (HeatCapacityRatio - 1.0));
        impact_pressure_psf = local_pressure_psf * (pt_over_p - 1.0);
    }
    const double pressure_ratio = impact_pressure_psf / 2116.228 + 1.0;
    if (pressure_ratio <= 1.0)
    {
        result.value = 0.0;
        result.status = PlantStatus::Success();
        return result;
    }
    constexpr double HeatCapacityRatio = 1.4;
    const double sea_level_mach = std::sqrt(
        2.0 / (HeatCapacityRatio - 1.0)
        * (std::pow(
            pressure_ratio,
            (HeatCapacityRatio - 1.0) / HeatCapacityRatio) - 1.0));
    result.value = 1116.4498 * sea_level_mach / 1.6878098;
    if (!std::isfinite(result.value))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "calibrated-airspeed model produced a non-finite result");
        return result;
    }
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<FbwStepOutput> AuxiliaryStateModel::StepModern(
    const AuxState& state,
    const FbwInput& input) const noexcept
{
    PlantResult<FbwStepOutput> result;
    if (!IsFiniteInput(input) || !std::isfinite(state.mass_kg)
        || state.mass_kg <= 0.0
        || !std::isfinite(state.previous_force_z_n)
        || !std::isfinite(state.previous_force_y_n))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "modern FBW input/state is invalid");
        return result;
    }
    const PlantResult<double> calibrated_airspeed = CalibratedAirspeedKts(
        input.mach,
        input.local_pressure_pa);
    if (!calibrated_airspeed.ok())
    {
        result.status = calibrated_airspeed.status;
        return result;
    }

    const double nz_feedback = state.previous_force_z_n
        / (state.mass_kg * GravityMps2);
    const double ny_feedback = state.previous_force_y_n
        / (state.mass_kg * GravityMps2);
    const PlantResult<fbw::RollStepOutput> roll = roll_.Step(
        state.roll,
        input.command[0],
        input.p_rad_s,
        input.dt_s,
        true,
        calibrated_airspeed.value);
    if (!roll.ok())
    {
        result.status = roll.status;
        return result;
    }
    const PlantResult<fbw::PitchStepOutput> pitch = pitch_.Step(
        state.pitch,
        input.command[1],
        input.q_rad_s,
        nz_feedback,
        input.alpha_rad,
        input.roll_rad,
        input.pitch_rad,
        input.dt_s,
        input.pitch_trim,
        true,
        calibrated_airspeed.value);
    if (!pitch.ok())
    {
        result.status = pitch.status;
        return result;
    }
    const PlantResult<fbw::YawStepOutput> yaw = yaw_.Step(
        state.yaw,
        input.command[2],
        input.r_rad_s,
        ny_feedback,
        input.ground_speed_fps,
        input.dt_s,
        input.yaw_trim,
        true,
        calibrated_airspeed.value);
    if (!yaw.ok())
    {
        result.status = yaw.status;
        return result;
    }
    const PlantResult<fbw::FlaperonStepOutput> flaperon = flaperon_.Step(
        state.flaperon,
        roll.value.roll_rate_command,
        calibrated_airspeed.value,
        input.mach,
        input.dt_s);
    if (!flaperon.ok())
    {
        result.status = flaperon.status;
        return result;
    }

    result.value.next_state = state;
    result.value.next_state.roll = roll.value.next_state;
    result.value.next_state.pitch = pitch.value.next_state;
    result.value.next_state.yaw = yaw.value.next_state;
    result.value.next_state.flaperon = flaperon.value.next_state;
    result.value.aileron_rad = roll.value.aileron_rad;
    result.value.elevator_rad = pitch.value.elevator_rad;
    result.value.rudder_rad = yaw.value.rudder_rad;
    result.value.flaperon_mix_rad = flaperon.value.flaperon_mix_rad;
    result.value.leading_edge_flap_rad = fbw::LeadingEdgeFlapPosition(
        input.alpha_rad,
        input.mach,
        input.gear_position_normalized,
        input.weight_on_wheels);
    result.value.nz_feedback = nz_feedback;
    result.value.ny_feedback = ny_feedback;
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<AuxState> AuxiliaryStateModel::UpdateWithFuelFlow(
    const AuxState& state,
    const double throttle,
    const double dt_s,
    const double force_z_n,
    const double force_y_n,
    const double fuel_flow_lb_s) const noexcept
{
    PlantResult<AuxState> result;
    if (!std::isfinite(throttle) || !std::isfinite(dt_s) || dt_s <= 0.0
        || !std::isfinite(force_z_n) || !std::isfinite(force_y_n)
        || !std::isfinite(fuel_flow_lb_s) || fuel_flow_lb_s < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "auxiliary update input must be finite with positive dt/fuel flow");
        return result;
    }

    AuxState next = state;
    double used_lb = fuel_flow_lb_s * dt_s;
    for (double& tank_lb : next.tanks_lb)
    {
        if (!std::isfinite(tank_lb) || tank_lb < 0.0)
        {
            result.status = PlantStatus::Failure(
                PlantStatusCode::InvalidState,
                "tank state must be finite and nonnegative");
            return result;
        }
        if (used_lb <= 0.0)
        {
            break;
        }
        if (tank_lb >= used_lb)
        {
            tank_lb -= used_lb;
            used_lb = 0.0;
        }
        else
        {
            used_lb -= tank_lb;
            tank_lb = 0.0;
        }
    }
    // Preserve auxiliary_states.py's active fuel-flow branch operation order:
    // (sum(tanks) * LB2KG) + (empty * LB2KG) + (pilot * LB2KG).
    // This intentionally differs from mass_balance.compute_mass(), which is
    // used for the current-frame published mass before fuel is consumed.
    double tank_total_lb = 0.0;
    for (const double tank_lb : next.tanks_lb)
    {
        tank_total_lb += tank_lb;
    }
    next.mass_kg = (tank_total_lb * 0.45359237)
        + (17400.0 * 0.45359237)
        + (230.0 * 0.45359237);
    if (!std::isfinite(next.mass_kg) || next.mass_kg <= 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "auxiliary fuel update produced an invalid mass");
        return result;
    }
    next.previous_force_z_n = force_z_n;
    next.previous_force_y_n = force_y_n;
    result.value = next;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace core
} // namespace plant
} // namespace LadyLuck
