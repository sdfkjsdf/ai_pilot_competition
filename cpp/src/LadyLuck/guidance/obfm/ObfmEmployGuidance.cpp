#include "LadyLuck/guidance/obfm/ObfmEmployGuidance.hpp"

#include <cmath>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void ObfmEmployAdmissionProvider::Observe(
    const DogfightGeometryFrame& frame,
    ObfmEmployAdmissionReceipt& output,
    Status& status) const noexcept
{
    output = ObfmEmployAdmissionReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!std::isfinite(frame.own_offense.damage_rate)
        || frame.own_offense.damage_rate < 0.0)
    {
        status.code = std::isfinite(frame.own_offense.damage_rate)
            ? StatusCode::InvalidArgument
            : StatusCode::NonFiniteInput;
        return;
    }

    output.valid = true;
    output.frame_identity = frame.frame_identity;
    output.admitted = frame.own_offense.damage_rate > 0.0;
    output.reason = output.admitted
        ? ObfmEmployAdmissionReason::None
        : ObfmEmployAdmissionReason::NoCurrentWeaponEffect;
}

void BuildObfmEmployGuidanceCandidate(
    const DogfightGeometryFrame& frame,
    const ObfmEmployGuidanceInput& input,
    ObfmEmployGuidanceCandidate& output,
    Status& status) noexcept
{
    output = ObfmEmployGuidanceCandidate{};
    status = Status{};
    if (!input.selected
        || !IsValidControlFrameIdentity(frame.frame_identity)
        || !input.station_hold.evaluated
        || !SameControlFrameIdentity(
            input.station_hold.frame_identity,
            frame.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !std::isfinite(frame.own_offense.phase.max_range_m)
        || !std::isfinite(input.station_hold.desired_speed_mps.value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const double own_speed_mps =
        NumpyNorm3(frame.own.velocity_ned_mps);
    const double official_capture_range_m =
        frame.own_offense.phase.max_range_m;
    if (!std::isfinite(own_speed_mps)
        || own_speed_mps <= 0.0
        || official_capture_range_m <= 0.0
        || (input.station_hold.desired_speed_mps.has_value
            && input.station_hold.desired_speed_mps.value <= 0.0))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    output.valid = true;
    output.frame_identity = frame.frame_identity;
    // WP-03 pure pursuit is the adversary's current position without lead.
    output.aim_point_ned_m = frame.opponent.position_ned_m;
    output.desired_speed_mps =
        input.station_hold.desired_speed_mps.has_value
        ? input.station_hold.desired_speed_mps.value
        : own_speed_mps;
    output.desired_speed_rate_mps2 = 0.0;
    output.capture_range_des_m = official_capture_range_m;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
