#include "LadyLuck/plant/fbw/Auxiliary.hpp"

#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace fbw
{

namespace
{

KinematicConfig AileronKinematicConfig() noexcept
{
    KinematicConfig config;
    config.detents = {{-1.0, 1.0, 0.0}};
    config.transition_times_s = {{0.3, 0.3, 0.0}};
    config.count = 2U;
    config.scale_input = true;
    return config;
}

KinematicConfig TrailingEdgeFlapKinematicConfig() noexcept
{
    KinematicConfig config;
    config.detents = {{-1.0, 0.0, 1.0}};
    config.transition_times_s = {{3.0, 0.0, 3.0}};
    config.count = 3U;
    config.scale_input = true;
    return config;
}

constexpr double AileronSpeedCompensationX[2] = {0.0, 1.0};
constexpr double AileronSpeedCompensationY[2] = {1.0, 0.15};

} // namespace

FlaperonMix::FlaperonMix() noexcept
    : aileron_(AileronKinematicConfig()),
      trailing_edge_flap_(TrailingEdgeFlapKinematicConfig())
{
}

PlantResult<FlaperonStepOutput> FlaperonMix::Step(
    const FlaperonState& state,
    const double roll_rate_command,
    const double calibrated_airspeed_kts,
    const double mach,
    const double dt_s) const noexcept
{
    PlantResult<FlaperonStepOutput> result;
    if (!std::isfinite(roll_rate_command)
        || !std::isfinite(calibrated_airspeed_kts)
        || !std::isfinite(mach)
        || !std::isfinite(dt_s)
        || dt_s <= 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "flaperon input must be finite and dt positive");
        return result;
    }

    double trailing_edge_position_rad = 0.0;
    if (mach > 0.9)
    {
        trailing_edge_position_rad = -0.0349;
    }
    if (calibrated_airspeed_kts < 250.0)
    {
        trailing_edge_position_rad = 0.349;
    }
    const double trailing_edge_normalized =
        2.864789 * trailing_edge_position_rad;
    const PlantResult<KinematicStepOutput> trailing_edge_result =
        trailing_edge_flap_.Step(
            state.trailing_edge_flap,
            trailing_edge_normalized,
            dt_s);
    if (!trailing_edge_result.ok())
    {
        result.status = trailing_edge_result.status;
        return result;
    }

    const PlantResult<KinematicStepOutput> aileron_result = aileron_.Step(
        state.aileron,
        roll_rate_command,
        dt_s);
    if (!aileron_result.ok())
    {
        result.status = aileron_result.status;
        return result;
    }
    const double speed_compensated_aileron = aileron_result.value.output
        * InterpolateClamped(
            mach,
            AileronSpeedCompensationX,
            AileronSpeedCompensationY,
            2U);
    const double left = Clip(
        -trailing_edge_result.value.output - speed_compensated_aileron,
        -1.0,
        1.0);
    const double right = Clip(
        trailing_edge_result.value.output - speed_compensated_aileron,
        -1.0,
        1.0);
    const double flap_mix_rad = 1.4324 * (left + right);
    if (!std::isfinite(flap_mix_rad))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "flaperon model produced a non-finite result");
        return result;
    }

    result.value.next_state.aileron = aileron_result.value.next_state;
    result.value.next_state.trailing_edge_flap =
        trailing_edge_result.value.next_state;
    result.value.flaperon_mix_rad = flap_mix_rad;
    result.status = PlantStatus::Success();
    return result;
}

double LeadingEdgeFlapPosition(
    const double alpha_rad,
    const double mach,
    const double gear_position_normalized,
    const double weight_on_wheels) noexcept
{
    double position = 0.0;
    if (mach > 0.9)
    {
        position = -0.0349;
    }
    if (weight_on_wheels == 0.0 && alpha_rad > 0.0873)
    {
        position = 0.262;
    }
    if (gear_position_normalized == 0.0 && alpha_rad > 0.2618)
    {
        position = 0.436;
    }
    if (weight_on_wheels == 1.0 && gear_position_normalized > 0.0)
    {
        position = -0.0349;
    }
    return position;
}

} // namespace fbw
} // namespace plant
} // namespace LadyLuck
