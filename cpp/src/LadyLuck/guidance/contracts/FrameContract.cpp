#include "LadyLuck/guidance/contracts/FrameContract.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double Pi = 3.141592653589793238462643383279502884;
constexpr double DegreesToRadians = Pi / 180.0;

double WrapRadians(const double angle) noexcept
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool IsFinitePlane(const PlaneKinematicObservationV1& plane) noexcept
{
    return std::isfinite(static_cast<double>(plane.position_n_m))
        && std::isfinite(static_cast<double>(plane.position_e_m))
        && std::isfinite(static_cast<double>(plane.position_up_m))
        && std::isfinite(static_cast<double>(plane.roll_deg))
        && std::isfinite(static_cast<double>(plane.pitch_deg))
        && std::isfinite(static_cast<double>(plane.yaw_deg))
        && std::isfinite(static_cast<double>(plane.body_u_mps))
        && std::isfinite(static_cast<double>(plane.body_v_mps))
        && std::isfinite(static_cast<double>(plane.body_w_mps));
}

void ConvertPlane(
    const PlaneKinematicObservationV1& input,
    DerivedPlaneStateV1& output) noexcept
{
    output = DerivedPlaneStateV1{};
    output.frame_index = input.frame_index;
    output.plane_id = input.plane_id;
    output.force_side = input.force_side;
    output.position_ned_m[0] = static_cast<double>(input.position_n_m);
    output.position_ned_m[1] = static_cast<double>(input.position_e_m);
    output.position_ned_m[2] = -static_cast<double>(input.position_up_m);
    output.rpy_rad[0] = WrapRadians(static_cast<double>(input.roll_deg) * DegreesToRadians);
    output.rpy_rad[1] = static_cast<double>(input.pitch_deg) * DegreesToRadians;
    output.rpy_rad[2] = WrapRadians(static_cast<double>(input.yaw_deg) * DegreesToRadians);
    output.velocity_body_mps[0] = static_cast<double>(input.body_u_mps);
    output.velocity_body_mps[1] = static_cast<double>(input.body_v_mps);
    output.velocity_body_mps[2] = static_cast<double>(input.body_w_mps);

    const double u = output.velocity_body_mps[0];
    const double v = output.velocity_body_mps[1];
    const double w = output.velocity_body_mps[2];
    output.speed_mps = std::max(std::sqrt(u * u + v * v + w * w), 1.0e-6);
    output.alpha_rad = std::atan2(w, u);
    const double beta_argument = std::max(-1.0, std::min(1.0, v / output.speed_mps));
    output.beta_rad = WrapRadians(std::asin(beta_argument));
}
}

namespace AIP_Guidance
{
bool BuildFrameContractV1(
    const KinematicObservationInputV1& input,
    FrameContractDiagnosticsV1& output) noexcept
{
    output = FrameContractDiagnosticsV1{};
    output.abi_version = AIPILOT_ABI_VERSION_V1;
    output.struct_size = sizeof(FrameContractDiagnosticsV1);
    output.command_frame_index = input.command_frame_index;
    output.command_time_s = input.command_time_s;
    output.nominal_dt_s = input.nominal_dt_s;
    output.context_own_plane_id = input.context_own_plane_id;
    output.context_target_plane_id = input.context_target_plane_id;

    const bool own_header_valid = input.abi_version == AIPILOT_ABI_VERSION_V1
        && input.struct_size == sizeof(KinematicObservationInputV1)
        && input.ownship.abi_version == AIPILOT_ABI_VERSION_V1
        && input.ownship.struct_size == sizeof(PlaneKinematicObservationV1);
    const bool time_valid = std::isfinite(input.command_time_s)
        && std::isfinite(input.nominal_dt_s)
        && input.command_time_s >= 0.0
        && input.nominal_dt_s > 0.0
        && std::fabs(input.nominal_dt_s - AIPILOT_NOMINAL_DT_S_V1) <= 1.0e-15;
    if (!own_header_valid || !time_valid || !IsFinitePlane(input.ownship))
    {
        return false;
    }

    ConvertPlane(input.ownship, output.ownship);
    output.own_measurement_valid = 1U;

    const bool target_valid = input.target.abi_version == AIPILOT_ABI_VERSION_V1
        && input.target.struct_size == sizeof(PlaneKinematicObservationV1)
        && IsFinitePlane(input.target);
    if (target_valid)
    {
        ConvertPlane(input.target, output.target);
        output.target_observation_valid = 1U;
    }
    return true;
}
}
