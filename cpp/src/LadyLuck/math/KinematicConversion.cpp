#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>

namespace LadyLuck
{
Result<PlaneState> ConvertKinematicObservation(
    const KinematicObservation& input) noexcept
{
    Result<PlaneState> result{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (!std::isfinite(static_cast<double>(input.position_neu_m[index]))
            || !std::isfinite(static_cast<double>(input.rpy_deg[index]))
            || !std::isfinite(static_cast<double>(input.velocity_body_mps[index])))
        {
            result.status.code = StatusCode::NonFiniteInput;
            return result;
        }
    }

    PlaneState& output = result.value;
    output.frame_index = input.frame_index;
    output.plane_id = input.plane_id;
    output.force_side = input.force_side;
    output.position_ned_m = Vector3{{
        static_cast<double>(input.position_neu_m[0]),
        static_cast<double>(input.position_neu_m[1]),
        -static_cast<double>(input.position_neu_m[2])}};
    output.rpy_rad = Vector3{{
        WrapRadians(static_cast<double>(input.rpy_deg[0]) * DegreesToRadians),
        static_cast<double>(input.rpy_deg[1]) * DegreesToRadians,
        WrapRadians(static_cast<double>(input.rpy_deg[2]) * DegreesToRadians)}};
    output.velocity_body_mps = Vector3{{
        static_cast<double>(input.velocity_body_mps[0]),
        static_cast<double>(input.velocity_body_mps[1]),
        static_cast<double>(input.velocity_body_mps[2])}};
    const double u = output.velocity_body_mps[0];
    const double v = output.velocity_body_mps[1];
    const double w = output.velocity_body_mps[2];
    output.speed_mps = std::max(
        std::sqrt(u * u + v * v + w * w),
        1.0e-6);
    output.alpha_rad = std::atan2(w, u);
    output.beta_rad = WrapRadians(std::asin(std::max(
        -1.0,
        std::min(1.0, v / output.speed_mps))));
    return result;
}
}
