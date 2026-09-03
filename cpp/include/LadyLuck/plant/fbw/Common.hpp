#pragma once

#include "LadyLuck/plant/types/AuxState.hpp"
#include "LadyLuck/plant/types/PlantStatus.hpp"

#include <array>
#include <cstddef>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

double Clip(double value, double lower, double upper) noexcept;
double PureGain(double value, double gain = 1.0) noexcept;
double InterpolateClamped(
    double value,
    const double* breakpoints,
    const double* samples,
    std::size_t count) noexcept;
double ScheduledGain(
    double value,
    double independent,
    const double* breakpoints,
    const double* samples,
    std::size_t count,
    double gain = 1.0) noexcept;
double AerosurfaceScale(
    double value,
    double input_minimum,
    double input_maximum,
    double output_minimum,
    double output_maximum,
    double gain = 1.0,
    bool zero_centered = true) noexcept;

enum class IntegrationMethod
{
    Rectangular,
    Trapezoidal,
    AdamsBashforth2,
    AdamsBashforth3,
    None
};

struct PidConfig
{
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    IntegrationMethod integration = IntegrationMethod::AdamsBashforth2;
    bool clipping_enabled = false;
    double clip_minimum = 0.0;
    double clip_maximum = 0.0;
    bool standard_form = false;
};

struct PidStepOutput
{
    PidState next_state{};
    double output = 0.0;
};

class Pid
{
public:
    explicit Pid(const PidConfig& config) noexcept;

    PlantResult<PidStepOutput> Step(
        const PidState& state,
        double input,
        double dt_s,
        bool trigger_present,
        double trigger) const noexcept;

private:
    PidConfig config_{};
};

struct KinematicConfig
{
    std::array<double, 3> detents{{0.0, 0.0, 0.0}};
    std::array<double, 3> transition_times_s{{0.0, 0.0, 0.0}};
    std::size_t count = 0U;
    bool scale_input = true;
};

struct KinematicStepOutput
{
    KinematicState next_state{};
    double output = 0.0;
};

class Kinematic
{
public:
    explicit Kinematic(const KinematicConfig& config) noexcept;

    PlantResult<KinematicStepOutput> Step(
        const KinematicState& state,
        double input_command,
        double dt_s) const noexcept;

private:
    KinematicConfig config_{};
};

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
