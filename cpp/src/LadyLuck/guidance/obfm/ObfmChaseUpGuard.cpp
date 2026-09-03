#include "LadyLuck/guidance/obfm/ObfmChaseUpGuard.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlIntent;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::ObfmChaseUpBehavior;
using LadyLuck::guidance::obfm::ObfmChaseUpGuardReason;
using LadyLuck::guidance::obfm::ObfmChaseUpGuardReceipt;

constexpr double Float32Epsilon =
    static_cast<double>(std::numeric_limits<float>::epsilon());

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    // NumPy 1.26 length-three dot reduction association used by add/main.
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

bool PursuitFamily(const ObfmChaseUpBehavior behavior) noexcept
{
    return behavior == ObfmChaseUpBehavior::Lag
        || behavior == ObfmChaseUpBehavior::Employ;
}

void StructuralFailure(
    ObfmChaseUpGuardReceipt& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = ObfmChaseUpGuardReceipt{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* ObfmChaseUpGuardReasonLabel(
    const ObfmChaseUpGuardReason reason) noexcept
{
    switch (reason)
    {
    case ObfmChaseUpGuardReason::BehaviorOutsidePursuitFamily:
        return "behavior_outside_pursuit_family";
    case ObfmChaseUpGuardReason::SustainedCornerUnavailable:
        return "sustained_corner_unavailable";
    case ObfmChaseUpGuardReason::SustainedCornerInvalid:
        return "sustained_corner_invalid";
    case ObfmChaseUpGuardReason::StateUnavailable:
        return "state_unavailable";
    case ObfmChaseUpGuardReason::AimWithinCeilingBand:
        return "aim_within_ceiling_band";
    case ObfmChaseUpGuardReason::CeilingClampSelected:
        return "ceiling_clamp_selected";
    default:
        return "unknown";
    }
}

void EvaluateObfmChaseUpGuard(
    const DogfightGeometryFrame& frame,
    const ObfmChaseUpBehavior upstream_behavior,
    const ControlIntent& upstream_intent,
    const em::MergeCornerInterval& sustained_corner,
    ObfmChaseUpGuardReceipt& output,
    Status& status) noexcept
{
    output = ObfmChaseUpGuardReceipt{};
    status = Status{};

    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !SameControlFrameIdentity(
            frame.frame_identity,
            upstream_intent.frame_identity))
    {
        StructuralFailure(output, status, StatusCode::InvalidArgument);
        return;
    }
    Status intent_status{};
    upstream_intent.Validate(intent_status);
    if (!intent_status.ok())
    {
        StructuralFailure(output, status, intent_status.code);
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.valid = true;
    output.candidate = upstream_intent;

    if (!PursuitFamily(upstream_behavior))
    {
        output.reason =
            ObfmChaseUpGuardReason::BehaviorOutsidePursuitFamily;
        return;
    }
    output.applicable = true;

    if (!sustained_corner.admitted())
    {
        output.reason =
            ObfmChaseUpGuardReason::SustainedCornerUnavailable;
        return;
    }
    const double sustained_speed_mps = sustained_corner.upper_mps.value;
    output.sustained_corner_speed_mps = sustained_speed_mps;
    if (!std::isfinite(sustained_speed_mps)
        || sustained_speed_mps <= 0.0)
    {
        // Python's guard retains the command on an unusable optional corner.
        output.reason = ObfmChaseUpGuardReason::SustainedCornerInvalid;
        return;
    }

    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(upstream_intent.aim_point_m))
    {
        output.reason = ObfmChaseUpGuardReason::StateUnavailable;
        return;
    }

    output.own_altitude_m = -frame.own.position_ned_m[2];
    output.own_speed_mps = NumpyNorm3(frame.own.velocity_ned_mps);
    if (!std::isfinite(output.own_altitude_m)
        || !std::isfinite(output.own_speed_mps))
    {
        output.reason = ObfmChaseUpGuardReason::StateUnavailable;
        return;
    }

    output.climb_budget_m = (std::max)(
        0.0,
        (output.own_speed_mps * output.own_speed_mps
            - sustained_speed_mps * sustained_speed_mps)
            / (2.0 * constants::StandardGravityMps2));
    output.ceiling_altitude_m =
        output.own_altitude_m + output.climb_budget_m;
    output.aim_altitude_m = -upstream_intent.aim_point_m[2];
    output.float32_band_m = (std::max)(
        (std::max)(
            std::fabs(output.aim_altitude_m),
            std::fabs(output.ceiling_altitude_m)),
        1.0) * Float32Epsilon;

    if (output.aim_altitude_m
        <= output.ceiling_altitude_m + output.float32_band_m)
    {
        output.reason = ObfmChaseUpGuardReason::AimWithinCeilingBand;
        return;
    }

    output.candidate.aim_point_m[2] = -output.ceiling_altitude_m;
    // This guard owns only the vertical pursuit ceiling. Longitudinal
    // authority has already been selected explicitly by the OBFM parent
    // (station hold or phase reference) and must remain unchanged.

    Status candidate_status{};
    output.candidate.Validate(candidate_status);
    if (!candidate_status.ok())
    {
        StructuralFailure(output, status, candidate_status.code);
        return;
    }
    output.modified = true;
    output.reason = ObfmChaseUpGuardReason::CeilingClampSelected;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
