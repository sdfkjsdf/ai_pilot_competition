#pragma once

#include "LadyLuck/plant/fbw/Common.hpp"

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

struct PitchStepOutput
{
    PitchChannelState next_state{};
    double normal_acceleration_correction = 0.0;
    double load_correction = 0.0;
    double elevator_command_limited = 0.0;
    double elevator_scheduled = 0.0;
    double pitch_rate_normalized = 0.0;
    double load_normalized = 0.0;
    double pitch_trim_error = 0.0;
    double load_pid = 0.0;
    double pitch_scheduled = 0.0;
    double elevator_position_normalized = 0.0;
    double elevator_rad = 0.0;
};

class PitchChannel
{
public:
    PitchChannel() noexcept;

    PitchChannelState InitialState(double elevator_position_rad = 0.0) const noexcept;

    PlantResult<PitchStepOutput> Step(
        const PitchChannelState& state,
        double elevator_command,
        double q_aero_rad_s,
        double nz_normalized,
        double alpha_rad,
        double roll_rad,
        double pitch_rad,
        double dt_s,
        double pitch_trim,
        bool vc_kts_present,
        double vc_kts) const noexcept;

private:
    Pid pid_;
    Kinematic actuator_;
};

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
