#include "LadyLuck/plant/fbw/RollChannel.hpp"

#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

namespace
{

PidConfig RollPidConfig() noexcept
{
    PidConfig config;
    config.kp = 3.0;
    config.ki = 0.00050;
    config.kd = -0.00125;
    config.integration = IntegrationMethod::AdamsBashforth2;
    return config;
}

} // namespace

RollChannel::RollChannel() noexcept
    : pid_(RollPidConfig())
{
}

PlantResult<RollStepOutput> RollChannel::Step(
    const RollChannelState& state,
    const double aileron_command,
    const double p_aero_rad_s,
    const double dt_s,
    const bool vc_kts_present,
    const double vc_kts) const noexcept
{
    PlantResult<RollStepOutput> result;
    if (!std::isfinite(aileron_command) || !std::isfinite(p_aero_rad_s)
        || !std::isfinite(dt_s) || dt_s <= 0.0
        || (vc_kts_present && !std::isfinite(vc_kts)))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "roll-channel input must be finite and dt positive");
        return result;
    }

    const double trigger = vc_kts_present
        ? (vc_kts >= 20.0 ? 1.0 : 0.0)
        : 1.0;
    const double rate_normalized = PureGain(p_aero_rad_s, 0.31821);
    const double trim_error = aileron_command - rate_normalized;
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

    const double rate_command = Clip(
        pid_result.value.output + aileron_command,
        -1.0,
        1.0);
    const double aileron_rad = AerosurfaceScale(
        rate_command,
        -1.0,
        1.0,
        -0.375,
        0.375);
    if (!std::isfinite(aileron_rad))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "roll channel produced a non-finite result");
        return result;
    }

    result.value.next_state.pid = pid_result.value.next_state;
    result.value.roll_rate_normalized = rate_normalized;
    result.value.roll_trim_error = trim_error;
    result.value.roll_rate_pid = pid_result.value.output;
    result.value.roll_rate_command = rate_command;
    result.value.aileron_rad = aileron_rad;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
