#pragma once

#include "LadyLuck/plant/types/PlantStatus.hpp"

namespace LadyLuck
{
namespace plant
{
namespace dynamics
{

struct AtmosphereState
{
    double temperature_k = 0.0;
    double density_kg_m3 = 0.0;
    double density_ratio = 0.0;
};

struct EngineStepOutput
{
    double thrust_n = 0.0;
    double fuel_flow_lb_s = 0.0;
};

class EngineModel
{
public:
    EngineModel() noexcept;

    PlantStatus Reset() noexcept;
    PlantResult<double> SeedStaticFuelFlow(
        double throttle_command,
        double mach,
        double altitude_m) noexcept;
    PlantResult<EngineStepOutput> ThrustAndFuel(
        double throttle_command,
        double mach,
        double altitude_m,
        double dt_s) noexcept;

    double n2_percent() const noexcept;
    double fuel_flow_pph() const noexcept;

    static double ThrottlePosition(double throttle_command) noexcept;
    static PlantResult<AtmosphereState> Atmosphere(double altitude_m) noexcept;
    static PlantResult<double> StaticFuelFlowLbPerSecond(
        double throttle_command,
        double mach,
        double altitude_m) noexcept;

private:
    double n2_percent_ = 100.0;
    double fuel_flow_pph_ = 0.0;
};

} // namespace dynamics
} // namespace plant
} // namespace LadyLuck
