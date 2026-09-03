#pragma once

#include "LadyLuck/plant/fbw/Common.hpp"

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

struct YawStepOutput
{
    YawChannelState next_state{};
    double yaw_rate_normalized = 0.0;
    double yaw_load_normalized = 0.0;
    double yaw_trim_error = 0.0;
    double yaw_load_pid = 0.0;
    double yaw_scheduled = 0.0;
    double rudder_position_normalized = 0.0;
    double rudder_rad = 0.0;
};

class YawChannel
{
public:
    YawChannel() noexcept;

    PlantResult<YawStepOutput> Step(
        const YawChannelState& state,
        double rudder_command,
        double r_aero_rad_s,
        double ny_normalized,
        double ground_speed_fps,
        double dt_s,
        double yaw_trim,
        bool vc_kts_present,
        double vc_kts) const noexcept;

private:
    Pid pid_;
};

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
