#include "LadyLuck/plant/fbw/YawChannel.hpp"

#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

namespace
{

PidConfig YawPidConfig() noexcept
{
    PidConfig config;
    config.kp = 0.105500;
    config.ki = 0.000010;
    config.kd = 0.00005;
    config.integration = IntegrationMethod::AdamsBashforth2;
    config.clipping_enabled = true;
    config.clip_minimum = -1.0;
    config.clip_maximum = 1.0;
    return config;
}

constexpr double YawRateScheduleX[3] = {80.0, 100.0, 150.0};
constexpr double YawRateScheduleY[3] = {0.0, 15.0, 100.0};

} // namespace

YawChannel::YawChannel() noexcept
    : pid_(YawPidConfig())
{
}

PlantResult<YawStepOutput> YawChannel::Step(
    const YawChannelState& state,
    const double rudder_command,
    const double r_aero_rad_s,
    const double ny_normalized,
    const double ground_speed_fps,
    const double dt_s,
    const double yaw_trim,
    const bool vc_kts_present,
    const double vc_kts) const noexcept
{
    PlantResult<YawStepOutput> result;
    const double inputs[] = {
        rudder_command, r_aero_rad_s, ny_normalized,
        ground_speed_fps, dt_s, yaw_trim
    };
    for (const double input : inputs)
    {
        if (!std::isfinite(input))
        {
            result.status = PlantStatus::Failure(
                PlantStatusCode::InvalidArgument,
                "yaw-channel input must be finite");
            return result;
        }
    }
    if (dt_s <= 0.0 || (vc_kts_present && !std::isfinite(vc_kts)))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "yaw-channel dt/speed input is invalid");
        return result;
    }

    const double trigger = (!vc_kts_present || vc_kts >= 10.0) ? 1.0 : 0.0;
    const double rate_normalized = ScheduledGain(
        r_aero_rad_s,
        ground_speed_fps,
        YawRateScheduleX,
        YawRateScheduleY,
        3U);
    const double load_normalized = PureGain(ny_normalized, 0.25);
    const double trim_error = rudder_command
        + rate_normalized
        + load_normalized;
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

    const double yaw_scheduled = Clip(
        rudder_command + yaw_trim + pid_result.value.output,
        -1.0,
        1.0);
    const double maximum_delta = 5.0 * dt_s;
    const double rudder_position = pid_result.value.output + Clip(
        yaw_scheduled - pid_result.value.output,
        -maximum_delta,
        maximum_delta);
    const double rudder_rad = AerosurfaceScale(
        rudder_position,
        -1.0,
        1.0,
        -0.524,
        0.524);
    if (!std::isfinite(rudder_rad))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "yaw channel produced a non-finite result");
        return result;
    }

    result.value.next_state.pid = pid_result.value.next_state;
    result.value.yaw_rate_normalized = rate_normalized;
    result.value.yaw_load_normalized = load_normalized;
    result.value.yaw_trim_error = trim_error;
    result.value.yaw_load_pid = pid_result.value.output;
    result.value.yaw_scheduled = yaw_scheduled;
    result.value.rudder_position_normalized = rudder_position;
    result.value.rudder_rad = rudder_rad;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
