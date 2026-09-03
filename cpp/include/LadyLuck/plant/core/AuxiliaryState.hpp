#pragma once

#include "LadyLuck/plant/fbw/Auxiliary.hpp"
#include "LadyLuck/plant/fbw/PitchChannel.hpp"
#include "LadyLuck/plant/fbw/RollChannel.hpp"
#include "LadyLuck/plant/fbw/YawChannel.hpp"
#include "LadyLuck/plant/types/AuxState.hpp"
#include "LadyLuck/plant/types/PlantStatus.hpp"

#include <array>

namespace LadyLuck
{
namespace plant
{
namespace core
{

struct FbwInput
{
    std::array<double, 4> command{{0.0, 0.0, 0.0, 0.0}};
    double p_rad_s = 0.0;
    double q_rad_s = 0.0;
    double r_rad_s = 0.0;
    double alpha_rad = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double true_airspeed_mps = 0.0;
    double mach = 0.0;
    double dt_s = 0.0;
    double pitch_trim = 0.0;
    double yaw_trim = 0.0;
    double ground_speed_fps = 0.0;
    double local_pressure_pa = 0.0;
    double gear_position_normalized = 0.0;
    double weight_on_wheels = 0.0;
};

struct FbwStepOutput
{
    AuxState next_state{};
    double aileron_rad = 0.0;
    double elevator_rad = 0.0;
    double rudder_rad = 0.0;
    double flaperon_mix_rad = 0.0;
    double leading_edge_flap_rad = 0.0;
    double nz_feedback = 0.0;
    double ny_feedback = 0.0;
};

class AuxiliaryStateModel
{
public:
    AuxiliaryStateModel() noexcept;

    AuxState InitialState(double mass_kg) const noexcept;
    PlantResult<FbwStepOutput> StepModern(
        const AuxState& state,
        const FbwInput& input) const noexcept;
    PlantResult<AuxState> UpdateWithFuelFlow(
        const AuxState& state,
        double throttle,
        double dt_s,
        double force_z_n,
        double force_y_n,
        double fuel_flow_lb_s) const noexcept;

    static PlantResult<double> CalibratedAirspeedKts(
        double mach,
        double local_pressure_pa) noexcept;

private:
    fbw::RollChannel roll_;
    fbw::PitchChannel pitch_;
    fbw::YawChannel yaw_;
    fbw::FlaperonMix flaperon_;
};

} // namespace core
} // namespace plant
} // namespace LadyLuck
