#pragma once

#include "LadyLuck/plant/fbw/Common.hpp"

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

struct RollStepOutput
{
    RollChannelState next_state{};
    double roll_rate_normalized = 0.0;
    double roll_trim_error = 0.0;
    double roll_rate_pid = 0.0;
    double roll_rate_command = 0.0;
    double aileron_rad = 0.0;
};

class RollChannel
{
public:
    RollChannel() noexcept;

    PlantResult<RollStepOutput> Step(
        const RollChannelState& state,
        double aileron_command,
        double p_aero_rad_s,
        double dt_s,
        bool vc_kts_present,
        double vc_kts) const noexcept;

private:
    Pid pid_;
};

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
