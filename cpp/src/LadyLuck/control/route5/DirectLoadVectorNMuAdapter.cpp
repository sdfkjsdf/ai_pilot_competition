#include "LadyLuck/control/route5/DirectLoadVectorNMuAdapter.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

void Fail(
    LadyLuck::control::route5::DirectLoadVectorNMuInput& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::control::route5::DirectLoadVectorNMuInput{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{
namespace control
{
namespace route5
{

void DirectLoadVectorNMuAdapter::Prepare(
    const ControlIntent& command,
    const Vector3& velocity_ned_mps,
    DirectLoadVectorNMuInput& output,
    Status& status) const noexcept
{
    output = DirectLoadVectorNMuInput{};
    status = Status{};

    // ControlIntent is validated once at the ControlCore/provider boundary.
    // This adapter owns only route-specific presence and numeric checks.
    if (command.route_kind
            != ControlRouteKind::DirectLoadVectorAcceleration
        || !command.direct_load_vector_acceleration_ned_mps2.has_value)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    const Vector3& acceleration_ned_mps2 =
        command.direct_load_vector_acceleration_ned_mps2.value;
    if (!FiniteVector(acceleration_ned_mps2)
        || !FiniteVector(velocity_ned_mps))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    // d90 NMuFilter.step_acceleration_ned operation order:
    // velocity -> (chi,gamma), C_w_r, C_w_r*(a_N/g), C_w_r*[0,0,1],
    // then subtract gravity.  phi is intentionally absent because production
    // paper_eq39 is false and dcm_ref_to_wind is roll-free.
    const double course_rad = std::atan2(
        velocity_ned_mps[1],
        velocity_ned_mps[0]);
    const double horizontal_speed_mps = std::hypot(
        velocity_ned_mps[0],
        velocity_ned_mps[1]);
    const double flight_path_angle_rad = std::atan2(
        -velocity_ned_mps[2],
        horizontal_speed_mps);
    const double cosine_course = std::cos(course_rad);
    const double sine_course = std::sin(course_rad);
    const double cosine_path = std::cos(flight_path_angle_rad);
    const double sine_path = std::sin(flight_path_angle_rad);

    const Vector3 acceleration_ned_g{{
        acceleration_ned_mps2[0]
            / constants::StandardGravityMps2,
        acceleration_ned_mps2[1]
            / constants::StandardGravityMps2,
        acceleration_ned_mps2[2]
            / constants::StandardGravityMps2}};
    const Vector3 acceleration_wind_g{{
        cosine_course * cosine_path * acceleration_ned_g[0]
            + sine_course * cosine_path * acceleration_ned_g[1]
            - sine_path * acceleration_ned_g[2],
        -sine_course * acceleration_ned_g[0]
            + cosine_course * acceleration_ned_g[1],
        cosine_course * sine_path * acceleration_ned_g[0]
            + sine_course * sine_path * acceleration_ned_g[1]
            + cosine_path * acceleration_ned_g[2]}};
    const Vector3 gravity_wind_g{{
        -sine_path,
        0.0,
        cosine_path}};
    const Vector3 specific_force_wind_g{{
        acceleration_wind_g[0] - gravity_wind_g[0],
        acceleration_wind_g[1] - gravity_wind_g[1],
        acceleration_wind_g[2] - gravity_wind_g[2]}};

    if (!std::isfinite(course_rad)
        || !std::isfinite(flight_path_angle_rad)
        || !FiniteVector(acceleration_wind_g)
        || !FiniteVector(gravity_wind_g)
        || !FiniteVector(specific_force_wind_g))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    output.frame_identity = command.frame_identity;
    output.valid = true;
    output.course_rad = course_rad;
    output.flight_path_angle_rad = flight_path_angle_rad;
    output.acceleration_wind_g = acceleration_wind_g;
    output.gravity_wind_g = gravity_wind_g;
    output.specific_force_wind_g = specific_force_wind_g;
}

} // namespace route5
} // namespace control
} // namespace LadyLuck
