#include "LadyLuck/plant/fbw/Common.hpp"

#include <algorithm>
#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

namespace
{

bool IsFinite(const double value) noexcept
{
    return std::isfinite(value) != 0;
}

} // namespace

double Clip(const double value, const double lower, const double upper) noexcept
{
    if (upper < lower)
    {
        return value;
    }
    return value < lower ? lower : (value > upper ? upper : value);
}

double PureGain(const double value, const double gain) noexcept
{
    return gain * value;
}

double InterpolateClamped(
    const double value,
    const double* const breakpoints,
    const double* const samples,
    const std::size_t count) noexcept
{
    if (breakpoints == nullptr || samples == nullptr || count == 0U)
    {
        return 0.0;
    }
    if (count == 1U || value <= breakpoints[0])
    {
        return samples[0];
    }
    if (value >= breakpoints[count - 1U])
    {
        return samples[count - 1U];
    }

    std::size_t upper_index = 1U;
    while (upper_index < count && value > breakpoints[upper_index])
    {
        ++upper_index;
    }
    const std::size_t lower_index = upper_index - 1U;
    const double x0 = breakpoints[lower_index];
    const double x1 = breakpoints[upper_index];
    if (!(x1 > x0))
    {
        return samples[lower_index];
    }
    const double fraction = (value - x0) / (x1 - x0);
    return samples[lower_index]
        + fraction * (samples[upper_index] - samples[lower_index]);
}

double ScheduledGain(
    const double value,
    const double independent,
    const double* const breakpoints,
    const double* const samples,
    const std::size_t count,
    const double gain) noexcept
{
    return gain * InterpolateClamped(
        independent,
        breakpoints,
        samples,
        count) * value;
}

double AerosurfaceScale(
    const double value,
    const double input_minimum,
    const double input_maximum,
    const double output_minimum,
    const double output_maximum,
    const double gain,
    const bool zero_centered) noexcept
{
    double output = 0.0;
    if (zero_centered)
    {
        if (value == 0.0)
        {
            output = 0.0;
        }
        else if (value > 0.0)
        {
            output = (value / input_maximum) * output_maximum;
        }
        else
        {
            output = (value / input_minimum) * output_minimum;
        }
    }
    else
    {
        output = output_minimum
            + ((value - input_minimum) / (input_maximum - input_minimum))
                * (output_maximum - output_minimum);
    }
    return output * gain;
}

Pid::Pid(const PidConfig& config) noexcept
    : config_(config)
{
}

PlantResult<PidStepOutput> Pid::Step(
    const PidState& state,
    const double input,
    const double dt_s,
    const bool trigger_present,
    const double trigger) const noexcept
{
    PlantResult<PidStepOutput> result;
    if (!IsFinite(input) || !IsFinite(dt_s) || dt_s < 0.0
        || !IsFinite(state.integral) || !IsFinite(state.input_previous)
        || !IsFinite(state.input_previous2)
        || (trigger_present && !IsFinite(trigger)))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "PID input/state must be finite and dt nonnegative");
        return result;
    }

    double integral = state.integral;
    const double test = trigger_present ? trigger : 0.0;
    const bool integrate = !trigger_present || std::abs(test) < 1.0e-6;
    if (integrate && config_.ki != 0.0
        && config_.integration != IntegrationMethod::None)
    {
        double delta = 0.0;
        switch (config_.integration)
        {
        case IntegrationMethod::Rectangular:
            delta = input;
            break;
        case IntegrationMethod::Trapezoidal:
            delta = 0.5 * (input + state.input_previous);
            break;
        case IntegrationMethod::AdamsBashforth2:
            delta = 1.5 * input - 0.5 * state.input_previous;
            break;
        case IntegrationMethod::AdamsBashforth3:
            delta = (23.0 * input - 16.0 * state.input_previous
                + 5.0 * state.input_previous2) / 12.0;
            break;
        case IntegrationMethod::None:
            break;
        }
        integral += config_.ki * dt_s * delta;
    }

    if (test < 0.0)
    {
        integral = 0.0;
    }

    const double derivative = dt_s > 0.0
        ? (input - state.input_previous) / dt_s
        : 0.0;
    double output = config_.standard_form
        ? config_.kp * (input + integral + config_.kd * derivative)
        : config_.kp * input + integral + config_.kd * derivative;
    if (config_.clipping_enabled)
    {
        output = Clip(output, config_.clip_minimum, config_.clip_maximum);
    }

    result.value.next_state.integral = integral;
    result.value.next_state.input_previous = input;
    result.value.next_state.input_previous2 = test < 0.0
        ? 0.0
        : state.input_previous;
    result.value.output = output;
    if (!IsFinite(integral) || !IsFinite(output))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "PID produced a non-finite result");
        return result;
    }
    result.status = PlantStatus::Success();
    return result;
}

Kinematic::Kinematic(const KinematicConfig& config) noexcept
    : config_(config)
{
}

PlantResult<KinematicStepOutput> Kinematic::Step(
    const KinematicState& state,
    const double input_command,
    const double dt_s) const noexcept
{
    PlantResult<KinematicStepOutput> result;
    if (config_.count < 2U || config_.count > config_.detents.size()
        || !IsFinite(state.position) || !IsFinite(input_command)
        || !IsFinite(dt_s) || dt_s < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "kinematic configuration/input is invalid");
        return result;
    }

    double output = state.position;
    double input = config_.scale_input
        ? input_command * config_.detents[config_.count - 1U]
        : input_command;
    input = Clip(input, config_.detents[0], config_.detents[config_.count - 1U]);
    double remaining_dt = dt_s;
    while (remaining_dt > 0.0 && std::abs(input - output) > 1.0e-12)
    {
        std::size_t index = 1U;
        if (input < output)
        {
            while (index < config_.count - 1U
                && config_.detents[index] < output)
            {
                ++index;
            }
        }
        else
        {
            while (index < config_.count - 1U
                && config_.detents[index] <= output)
            {
                ++index;
            }
        }

        const double transition_time = config_.transition_times_s[index];
        if (transition_time <= 0.0)
        {
            output = Clip(
                input,
                config_.detents[index - 1U],
                config_.detents[index]);
            break;
        }
        const double rate = (config_.detents[index]
            - config_.detents[index - 1U]) / transition_time;
        if (!IsFinite(rate) || rate == 0.0)
        {
            result.status = PlantStatus::Failure(
                PlantStatusCode::InvalidState,
                "kinematic detent rate is invalid");
            return result;
        }
        const double segment_input = Clip(
            input,
            config_.detents[index - 1U],
            config_.detents[index]);
        double segment_dt = std::abs((segment_input - output) / rate);
        if (remaining_dt < segment_dt)
        {
            segment_dt = remaining_dt;
            output += output < input ? segment_dt * rate : -segment_dt * rate;
        }
        else
        {
            output = segment_input;
        }
        remaining_dt -= segment_dt;
    }

    if (!IsFinite(output))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "kinematic model produced a non-finite result");
        return result;
    }
    result.value.next_state.position = output;
    result.value.output = output;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
