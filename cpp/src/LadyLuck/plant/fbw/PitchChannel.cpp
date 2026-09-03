#include "LadyLuck/plant/fbw/PitchChannel.hpp"

#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

namespace
{

PidConfig PitchPidConfig() noexcept
{
    PidConfig config;
    config.kp = 0.3;
    config.ki = 0.025;
    config.kd = 0.0;
    config.integration = IntegrationMethod::AdamsBashforth2;
    config.clipping_enabled = true;
    config.clip_minimum = -1.0;
    config.clip_maximum = 1.0;
    return config;
}

KinematicConfig ElevatorKinematicConfig() noexcept
{
    KinematicConfig config;
    config.detents = {{-1.0, 1.0, 0.0}};
    config.transition_times_s = {{0.3, 0.3, 0.0}};
    config.count = 2U;
    config.scale_input = true;
    return config;
}

constexpr double ElevatorScheduleX[5] = {
    -0.5236, -0.5, 0.0, 0.5, 0.5236
};
constexpr double ElevatorScheduleY[5] = {
    0.0, 0.11, 1.0, 0.11, 0.0
};

} // namespace

PitchChannel::PitchChannel() noexcept
    : pid_(PitchPidConfig()),
      actuator_(ElevatorKinematicConfig())
{
}

PitchChannelState PitchChannel::InitialState(
    const double elevator_position_rad) const noexcept
{
    PitchChannelState state;
    state.elevator.position = 0.436 != 0.0
        ? elevator_position_rad / 0.436
        : 0.0;
    return state;
}

PlantResult<PitchStepOutput> PitchChannel::Step(
    const PitchChannelState& state,
    const double elevator_command,
    const double q_aero_rad_s,
    const double nz_normalized,
    const double alpha_rad,
    const double roll_rad,
    const double pitch_rad,
    const double dt_s,
    const double pitch_trim,
    const bool vc_kts_present,
    const double vc_kts) const noexcept
{
    PlantResult<PitchStepOutput> result;
    const double inputs[] = {
        elevator_command, q_aero_rad_s, nz_normalized, alpha_rad,
        roll_rad, pitch_rad, dt_s, pitch_trim
    };
    for (const double input : inputs)
    {
        if (!std::isfinite(input))
        {
            result.status = PlantStatus::Failure(
                PlantStatusCode::InvalidArgument,
                "pitch-channel input must be finite");
            return result;
        }
    }
    if (dt_s <= 0.0 || (vc_kts_present && !std::isfinite(vc_kts)))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "pitch-channel dt/speed input is invalid");
        return result;
    }

    const double trigger = (!vc_kts_present || vc_kts >= 5.0) ? 1.0 : 0.0;
    const double normal_acceleration_correction =
        std::cos(pitch_rad) * std::cos(roll_rad);
    const double load_correction = nz_normalized - normal_acceleration_correction;
    const double command_limited = Clip(
        elevator_command + pitch_trim,
        -1.0,
        0.44);
    const double command_scheduled = ScheduledGain(
        command_limited,
        alpha_rad,
        ElevatorScheduleX,
        ElevatorScheduleY,
        5U);
    const double alpha_limited_normalized = PureGain(alpha_rad, 1.0472);
    const double pitch_rate_normalized = PureGain(q_aero_rad_s, 6.2);
    const double load_normalized = PureGain(load_correction, 0.020);
    const double trim_error = command_scheduled
        + pitch_rate_normalized
        - load_normalized;
    const PlantResult<PidStepOutput> pid_result = pid_.Step(
        state.pid,
        trim_error,
        dt_s,
        true,
        trigger);
    if (!pid_result.ok())
    {
        result.status = pid_result.status;
        return result;
    }

    const double pitch_scheduled = Clip(
        command_scheduled + alpha_limited_normalized + pid_result.value.output,
        -1.0,
        1.0);
    const PlantResult<KinematicStepOutput> actuator_result = actuator_.Step(
        state.elevator,
        pitch_scheduled,
        dt_s);
    if (!actuator_result.ok())
    {
        result.status = actuator_result.status;
        return result;
    }
    const double elevator_rad = AerosurfaceScale(
        actuator_result.value.output,
        -1.0,
        1.0,
        -0.436,
        0.436);
    if (!std::isfinite(elevator_rad))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "pitch channel produced a non-finite result");
        return result;
    }

    result.value.next_state.pid = pid_result.value.next_state;
    result.value.next_state.elevator = actuator_result.value.next_state;
    result.value.normal_acceleration_correction = normal_acceleration_correction;
    result.value.load_correction = load_correction;
    result.value.elevator_command_limited = command_limited;
    result.value.elevator_scheduled = command_scheduled;
    result.value.pitch_rate_normalized = pitch_rate_normalized;
    result.value.load_normalized = load_normalized;
    result.value.pitch_trim_error = trim_error;
    result.value.load_pid = pid_result.value.output;
    result.value.pitch_scheduled = pitch_scheduled;
    result.value.elevator_position_normalized = actuator_result.value.output;
    result.value.elevator_rad = elevator_rad;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
