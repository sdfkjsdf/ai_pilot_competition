#include "LadyLuck/plant/core/ProcessModel.hpp"

#include "LadyLuck/math/Attitude321.hpp"
#include "LadyLuck/plant/dynamics/MassBalance.hpp"

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

bool FiniteArray3(const std::array<double, 3>& values) noexcept
{
    return std::isfinite(values[0]) && std::isfinite(values[1])
        && std::isfinite(values[2]);
}

bool FiniteArray4(const std::array<double, 4>& values) noexcept
{
    return std::isfinite(values[0]) && std::isfinite(values[1])
        && std::isfinite(values[2]) && std::isfinite(values[3]);
}

bool FinitePlantState(const PlantState& state) noexcept
{
    return FiniteArray3(state.position_ned_m)
        && FiniteArray4(state.quaternion_wxyz)
        && FiniteArray3(state.velocity_body_mps)
        && FiniteArray3(state.omega_body_rad_s);
}

} // namespace

ProcessModel::ProcessModel() noexcept = default;

AuxState ProcessModel::InitialAuxiliaryState(const double mass_kg) const noexcept
{
    return auxiliary_.InitialState(mass_kg);
}

PlantResult<DerivedAirData> ProcessModel::Derived(
    const PlantState& state) const noexcept
{
    PlantResult<DerivedAirData> result;
    if (!FinitePlantState(state))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "plant state must be finite");
        return result;
    }
    const double u = state.velocity_body_mps[0];
    const double v = state.velocity_body_mps[1];
    const double w = state.velocity_body_mps[2];
    const double speed = std::max(std::sqrt(u * u + v * v + w * w), 1.0e-3);
    result.value.true_airspeed_mps = speed;
    result.value.alpha_rad = std::atan2(w, u);
    result.value.beta_rad = std::asin(
        fbw::Clip(v / speed, -1.0, 1.0));
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<IsaState> ProcessModel::Isa(const double altitude_m) noexcept
{
    PlantResult<IsaState> result;
    if (!std::isfinite(altitude_m))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "ISA altitude must be finite");
        return result;
    }
    const double altitude = std::max(altitude_m, 0.0);
    double temperature = 288.15
        - 0.0065 * std::min(altitude, 11000.0);
    if (altitude > 11000.0)
    {
        temperature = 216.65;
    }
    const double pressure = altitude <= 11000.0
        ? 101325.0 * std::pow(temperature / 288.15, 5.2559)
        : 22632.0 * std::exp(-(altitude - 11000.0) / 6341.6);
    result.value.density_kg_m3 = pressure / (287.05 * temperature);
    // Python process_model.isa intentionally uses **0.5, not np.sqrt.
    result.value.speed_of_sound_mps = std::pow(
        1.4 * 287.05 * temperature,
        0.5);
    if (!std::isfinite(result.value.density_kg_m3)
        || !std::isfinite(result.value.speed_of_sound_mps)
        || result.value.density_kg_m3 <= 0.0
        || result.value.speed_of_sound_mps <= 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "ISA model produced an invalid result");
        return result;
    }
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<ProcessStepOutput> ProcessModel::StepEstimateModern(
    const ProcessStepInput& input) const noexcept
{
    PlantResult<ProcessStepOutput> result;
    const std::array<double, 3>* const endpoint_omega_rad_s =
        input.omega_aero_rad_s.Get();
    if (!FinitePlantState(input.state) || !FiniteArray4(input.command)
        || (endpoint_omega_rad_s != nullptr
            && !FiniteArray3(*endpoint_omega_rad_s))
        || !std::isfinite(input.thrust_n)
        || !std::isfinite(input.dt_s) || input.dt_s <= 0.0
        || !std::isfinite(input.gear_position_normalized)
        || !std::isfinite(input.pitch_trim)
        || !std::isfinite(input.yaw_trim)
        || !std::isfinite(input.fuel_flow_lb_s)
        || input.fuel_flow_lb_s < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "modern process-model input is invalid");
        return result;
    }

    const PlantResult<DerivedAirData> derived = Derived(input.state);
    if (!derived.ok())
    {
        result.status = derived.status;
        return result;
    }
    const double altitude_m = -input.state.position_ned_m[2];
    const PlantResult<IsaState> atmosphere = Isa(altitude_m);
    if (!atmosphere.ok())
    {
        result.status = atmosphere.status;
        return result;
    }
    const double mach = derived.value.true_airspeed_mps
        / atmosphere.value.speed_of_sound_mps;
    const double dynamic_pressure_pa = 0.5
        * atmosphere.value.density_kg_m3
        * derived.value.true_airspeed_mps
        * derived.value.true_airspeed_mps;
    const double local_pressure_pa = atmosphere.value.density_kg_m3
        * atmosphere.value.speed_of_sound_mps
        * atmosphere.value.speed_of_sound_mps / 1.4;

    std::array<double, 3> rpy_rad{{0.0, 0.0, 0.0}};
    std::array<double, 9> dcm_ned_to_body{{
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    }};
    if (!LadyLuck::QuaternionToEuler321(input.state.quaternion_wxyz, rpy_rad)
        || !LadyLuck::QuaternionToDcmNedToBody(
            input.state.quaternion_wxyz,
            dcm_ned_to_body))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::DependencyFailure,
            "quaternion conversion failed");
        return result;
    }

    const double velocity_north = dcm_ned_to_body[0]
            * input.state.velocity_body_mps[0]
        + dcm_ned_to_body[3] * input.state.velocity_body_mps[1]
        + dcm_ned_to_body[6] * input.state.velocity_body_mps[2];
    const double velocity_east = dcm_ned_to_body[1]
            * input.state.velocity_body_mps[0]
        + dcm_ned_to_body[4] * input.state.velocity_body_mps[1]
        + dcm_ned_to_body[7] * input.state.velocity_body_mps[2];
    const double ground_speed_fps = std::sqrt(
        velocity_north * velocity_north
        + velocity_east * velocity_east) / 0.3048;

    FbwInput fbw_input;
    fbw_input.command = input.command;
    fbw_input.p_rad_s = input.state.omega_body_rad_s[0];
    fbw_input.q_rad_s = input.state.omega_body_rad_s[1];
    fbw_input.r_rad_s = input.state.omega_body_rad_s[2];
    fbw_input.alpha_rad = derived.value.alpha_rad;
    fbw_input.roll_rad = rpy_rad[0];
    fbw_input.pitch_rad = rpy_rad[1];
    fbw_input.true_airspeed_mps = derived.value.true_airspeed_mps;
    fbw_input.mach = mach;
    fbw_input.dt_s = input.dt_s;
    fbw_input.pitch_trim = input.pitch_trim;
    fbw_input.yaw_trim = input.yaw_trim;
    fbw_input.ground_speed_fps = ground_speed_fps;
    fbw_input.local_pressure_pa = local_pressure_pa;
    fbw_input.gear_position_normalized = input.gear_position_normalized;
    fbw_input.weight_on_wheels = 0.0;
    const PlantResult<FbwStepOutput> fbw_step = auxiliary_.StepModern(
        input.auxiliary,
        fbw_input);
    if (!fbw_step.ok())
    {
        result.status = fbw_step.status;
        return result;
    }

    const PlantResult<double> current_mass = dynamics::ComputeLoadedMassKg(
        input.auxiliary.tanks_lb);
    if (!current_mass.ok())
    {
        result.status = current_mass.status;
        return result;
    }

    aero::AeroInput aero_input;
    aero_input.alpha_rad = derived.value.alpha_rad;
    aero_input.beta_rad = derived.value.beta_rad;
    aero_input.mach = mach;
    aero_input.qbar_pa = dynamic_pressure_pa;
    aero_input.aileron_rad = fbw_step.value.aileron_rad;
    aero_input.elevator_rad = fbw_step.value.elevator_rad;
    aero_input.rudder_rad = fbw_step.value.rudder_rad;
    // The modern FBW channels above intentionally consume the interval rate.
    // Only the aerodynamic damping terms consume a supplied endpoint rate.
    // If the estimator has no endpoint reconstruction, match Python's
    // omega_aero_radps=None contract and reuse the interval rate exactly.
    const std::array<double, 3>& omega_aero_rad_s =
        endpoint_omega_rad_s != nullptr
            ? *endpoint_omega_rad_s
            : input.state.omega_body_rad_s;
    aero_input.p_rad_s = omega_aero_rad_s[0];
    aero_input.q_rad_s = omega_aero_rad_s[1];
    aero_input.r_rad_s = omega_aero_rad_s[2];
    aero_input.true_airspeed_mps = derived.value.true_airspeed_mps;
    aero_input.has_flaperon_mix = true;
    aero_input.flaperon_mix_rad = fbw_step.value.flaperon_mix_rad;
    aero_input.has_lef = true;
    aero_input.lef_rad = fbw_step.value.leading_edge_flap_rad;
    aero_input.has_speedbrake = false;
    aero_input.speedbrake_rad = 0.0;
    aero_input.has_gear = true;
    aero_input.gear_pos_norm = input.gear_position_normalized;
    const aero::AeroForces forces = aero_.Forces(aero_input);
    const double flaperon_force_z = aero_.FlaperonForceZ(aero_input);
    if (!std::isfinite(forces.fx_n) || !std::isfinite(forces.fy_n)
        || !std::isfinite(forces.fz_n)
        || !std::isfinite(flaperon_force_z))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::DependencyFailure,
            "aerodynamic model produced a non-finite force");
        return result;
    }

    const PlantResult<AuxState> updated_auxiliary =
        auxiliary_.UpdateWithFuelFlow(
            fbw_step.value.next_state,
            input.command[3],
            input.dt_s,
            forces.fz_n,
            forces.fy_n,
            input.fuel_flow_lb_s);
    if (!updated_auxiliary.ok())
    {
        result.status = updated_auxiliary.status;
        return result;
    }

    result.value.next_auxiliary = updated_auxiliary.value;
    result.value.force_z_n = forces.fz_n;
    result.value.force_y_n = forces.fy_n;
    result.value.mass_kg = current_mass.value;
    result.value.elevator_rad = fbw_step.value.elevator_rad;
    result.value.flaperon_force_z_n = flaperon_force_z;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace core
} // namespace plant
} // namespace LadyLuck
