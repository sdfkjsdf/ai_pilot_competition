#pragma once

#include "LadyLuck/plant/fbw/Common.hpp"

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

struct FlaperonStepOutput
{
    FlaperonState next_state{};
    double flaperon_mix_rad = 0.0;
};

class FlaperonMix
{
public:
    FlaperonMix() noexcept;

    PlantResult<FlaperonStepOutput> Step(
        const FlaperonState& state,
        double roll_rate_command,
        double calibrated_airspeed_kts,
        double mach,
        double dt_s) const noexcept;

private:
    Kinematic aileron_;
    Kinematic trailing_edge_flap_;
};

double LeadingEdgeFlapPosition(
    double alpha_rad,
    double mach,
    double gear_position_normalized,
    double weight_on_wheels) noexcept;

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
