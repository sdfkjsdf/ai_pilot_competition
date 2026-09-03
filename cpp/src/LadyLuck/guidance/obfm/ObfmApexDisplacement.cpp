#include "LadyLuck/guidance/obfm/ObfmApexDisplacement.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{

using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::ObfmApexDisplacementReason;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{left[0] - right[0], left[1] - right[1],
                    left[2] - right[2]}};
}

Vector3 Add(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{left[0] + right[0], left[1] + right[1],
                    left[2] + right[2]}};
}

Vector3 Scale(const Vector3& value, const double scale) noexcept
{
    return Vector3{{value[0] * scale, value[1] * scale, value[2] * scale}};
}

double Norm(const Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

bool ValidSide(const std::int32_t side) noexcept
{
    return side == -1 || side == 1;
}

std::int32_t AwaySide(
    const LadyLuck::DogfightGeometryFrame& frame,
    const bool preferred_available,
    const std::int32_t preferred_side) noexcept
{
    const double target_horizontal_speed = std::hypot(
        frame.opponent.velocity_ned_mps[0],
        frame.opponent.velocity_ned_mps[1]);
    if (target_horizontal_speed > 0.0)
    {
        const double tx = frame.opponent.velocity_ned_mps[0]
            / target_horizontal_speed;
        const double ty = frame.opponent.velocity_ned_mps[1]
            / target_horizontal_speed;
        const double rel_x = frame.own.position_ned_m[0]
            - frame.opponent.position_ned_m[0];
        const double rel_y = frame.own.position_ned_m[1]
            - frame.opponent.position_ned_m[1];
        const double signed_lateral = -ty * rel_x + tx * rel_y;
        if (signed_lateral > 0.0)
        {
            return -1;
        }
        if (signed_lateral < 0.0)
        {
            return 1;
        }
    }
    // Centreline is a symmetric tie, not missing tactical evidence.
    return preferred_available && ValidSide(preferred_side)
        ? preferred_side
        : 1;
}

bool BuildReachability(
    const LadyLuck::DogfightGeometryFrame& frame,
    const double turn_capability_n_g,
    const Vector3& lag_point,
    bool& direct_entry,
    double& turn_radius_m,
    double& required_accel_mps2,
    double& available_accel_mps2) noexcept
{
    direct_entry = false;
    turn_radius_m = 0.0;
    required_accel_mps2 = 0.0;
    available_accel_mps2 = 0.0;
    if (!FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(lag_point)
        || !std::isfinite(turn_capability_n_g))
    {
        return false;
    }
    const double speed = Norm(frame.own.velocity_ned_mps);
    const Vector3 offset = Subtract(lag_point, frame.own.position_ned_m);
    const double distance = Norm(offset);
    const double load_square = turn_capability_n_g * turn_capability_n_g - 1.0;
    if (!(speed > 0.0) || !(distance > 0.0) || !(load_square > 0.0))
    {
        return true;
    }
    const Vector3 velocity_hat = Scale(frame.own.velocity_ned_mps, 1.0 / speed);
    const Vector3 lag_hat = Scale(offset, 1.0 / distance);
    const double cosine = (std::max)(
        -1.0, (std::min)(1.0, Dot(velocity_hat, lag_hat)));
    const double sine = std::sqrt((std::max)(
        0.0, 1.0 - cosine * cosine));
    required_accel_mps2 = 2.0 * speed * speed * sine / distance;
    available_accel_mps2 = LadyLuck::constants::StandardGravityMps2
        * std::sqrt(load_square);
    turn_radius_m = speed * speed / available_accel_mps2;
    if (!std::isfinite(required_accel_mps2)
        || !std::isfinite(available_accel_mps2)
        || !std::isfinite(turn_radius_m))
    {
        return false;
    }
    direct_entry = required_accel_mps2 <= available_accel_mps2;
    return true;
}

void SetNormalNonAdmission(
    LadyLuck::guidance::obfm::ObfmApexDisplacementServiceReceipt& output,
    const ObfmApexDisplacementReason reason) noexcept
{
    output.reference.evaluated = true;
    output.reference.apex_latched = output.apex.apex_latched;
    output.reference.reason = reason;
    output.reason = reason;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* ObfmApexDisplacementReasonLabel(
    const ObfmApexDisplacementReason reason) noexcept
{
    switch (reason)
    {
    case ObfmApexDisplacementReason::SelectorServiceNotReached:
        return "selector_service_not_reached";
    case ObfmApexDisplacementReason::FrameEvidenceUnavailable:
        return "frame_evidence_unavailable";
    case ObfmApexDisplacementReason::FeatureDisabled:
        return "feature_disabled";
    case ObfmApexDisplacementReason::G16HighClimbArmed:
        return "g16_high_climb_armed";
    case ObfmApexDisplacementReason::G16HighApexLatched:
        return "g16_high_apex_latched";
    case ObfmApexDisplacementReason::ApexNotLatched:
        return "apex_not_latched";
    case ObfmApexDisplacementReason::LagReferenceUnavailable:
        return "lag_reference_unavailable";
    case ObfmApexDisplacementReason::TurnAuthorityUnavailable:
        return "turn_authority_unavailable";
    case ObfmApexDisplacementReason::DirectLagEntryAvailable:
        return "direct_lag_entry_available";
    case ObfmApexDisplacementReason::LateralDisplacementRequired:
        return "lateral_displacement_required";
    case ObfmApexDisplacementReason::DecoratorNotReached:
        return "decorator_not_reached";
    case ObfmApexDisplacementReason::DecoratorNotAdmitted:
        return "decorator_not_admitted";
    case ObfmApexDisplacementReason::DecoratorSelected:
        return "decorator_selected";
    case ObfmApexDisplacementReason::SafetySampleUnavailable:
        return "safety_sample_unavailable";
    case ObfmApexDisplacementReason::CommandReady:
        return "command_ready";
    case ObfmApexDisplacementReason::DeclaredReadyContradiction:
        return "declared_ready_contradiction";
    case ObfmApexDisplacementReason::UnownedClimbIgnored:
        return "unowned_climb_ignored";
    case ObfmApexDisplacementReason::SpacingClimbIgnored:
        return "spacing_climb_ignored";
    }
    return "unknown";
}

ObfmApexDisplacement::ObfmApexDisplacement() noexcept = default;

void ObfmApexDisplacement::ClearTaskLifecycle() noexcept
{
    task_active_ = false;
    away_side_sign_ = 0;
}

void ObfmApexDisplacement::ResetEpisode() noexcept
{
    climb_armed_ = false;
    apex_latched_ = false;
    climb_owner_ = ObfmApexClimbOwner::None;
    ClearTaskLifecycle();
}

void ObfmApexDisplacement::ObserveService(
    const DogfightGeometryFrame& frame,
    const ObfmApexDisplacementServiceInput& input,
    ObfmApexDisplacementServiceReceipt& output,
    Status& status) noexcept
{
    output = ObfmApexDisplacementServiceReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};
    if (!input.selector_service_reached)
    {
        output.reason = ObfmApexDisplacementReason::SelectorServiceNotReached;
        return;
    }
    if (!input.frame_evidence_declared_ready
        || !IsValidControlFrameIdentity(frame.frame_identity))
    {
        output.reason = ObfmApexDisplacementReason::FrameEvidenceUnavailable;
        return;
    }
    if (!FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.service_evaluated = true;
    output.apex.evaluated = true;
    const double down_velocity = frame.own.velocity_ned_mps[2];
    if (down_velocity < 0.0)
    {
        output.apex.vertical_phase = ObfmApexVerticalPhase::Climbing;
        if (!climb_armed_ && input.climb_owner == ObfmApexClimbOwner::G16High)
        {
            climb_armed_ = true;
            climb_owner_ = ObfmApexClimbOwner::G16High;
        }
        apex_latched_ = false;
        output.apex.reason = climb_armed_
            ? ObfmApexDisplacementReason::G16HighClimbArmed
            : input.climb_owner == ObfmApexClimbOwner::SpacingArrest
            ? ObfmApexDisplacementReason::SpacingClimbIgnored
            : ObfmApexDisplacementReason::UnownedClimbIgnored;
    }
    else if (down_velocity > 0.0)
    {
        output.apex.vertical_phase = ObfmApexVerticalPhase::Descending;
        if (climb_armed_ && climb_owner_ == ObfmApexClimbOwner::G16High)
        {
            climb_armed_ = false;
            climb_owner_ = ObfmApexClimbOwner::None;
            apex_latched_ = true;
            output.apex.reason =
                ObfmApexDisplacementReason::G16HighApexLatched;
        }
        else
        {
            output.apex.reason = apex_latched_
                ? ObfmApexDisplacementReason::G16HighApexLatched
                : ObfmApexDisplacementReason::ApexNotLatched;
        }
    }
    else
    {
        output.apex.vertical_phase = ObfmApexVerticalPhase::Level;
        output.apex.reason = apex_latched_
            ? ObfmApexDisplacementReason::G16HighApexLatched
            : ObfmApexDisplacementReason::ApexNotLatched;
    }
    output.apex.climb_armed = climb_armed_;
    output.apex.apex_latched = apex_latched_;
    output.apex.climb_owner = climb_owner_;

    if (!input.feature_enabled)
    {
        output.reason = ObfmApexDisplacementReason::FeatureDisabled;
        return;
    }
    if (!apex_latched_ && !task_active_)
    {
        SetNormalNonAdmission(
            output, ObfmApexDisplacementReason::ApexNotLatched);
        return;
    }
    if (!input.lag_point_available)
    {
        SetNormalNonAdmission(
            output, ObfmApexDisplacementReason::LagReferenceUnavailable);
        return;
    }
    if (!input.turn_capability_available
        || !(input.turn_capability_n_g > 1.0))
    {
        SetNormalNonAdmission(
            output, ObfmApexDisplacementReason::TurnAuthorityUnavailable);
        return;
    }
    if (!FiniteVector(input.lag_point_ned_m)
        || !std::isfinite(input.turn_capability_n_g)
        || (input.preferred_side_available
            && !ValidSide(input.preferred_side_sign)))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    bool direct_entry = false;
    double turn_radius = 0.0;
    double required_accel = 0.0;
    double available_accel = 0.0;
    if (!BuildReachability(
            frame,
            input.turn_capability_n_g,
            input.lag_point_ned_m,
            direct_entry,
            turn_radius,
            required_accel,
            available_accel))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!(turn_radius > 0.0) || !(available_accel > 0.0))
    {
        SetNormalNonAdmission(
            output, ObfmApexDisplacementReason::TurnAuthorityUnavailable);
        return;
    }

    output.reference.evaluated = true;
    output.reference.apex_latched = true;
    output.reference.direct_lag_entry_available = direct_entry;
    output.reference.lag_point_ned_m = input.lag_point_ned_m;
    output.reference.turn_radius_m = turn_radius;
    output.reference.required_lateral_accel_mps2 = required_accel;
    output.reference.available_lateral_accel_mps2 = available_accel;
    if (direct_entry)
    {
        output.reference.reason =
            ObfmApexDisplacementReason::DirectLagEntryAvailable;
        output.reason = output.reference.reason;
        apex_latched_ = false;
        return;
    }

    const std::int32_t side = task_active_
        ? away_side_sign_
        : AwaySide(
            frame,
            input.preferred_side_available,
            input.preferred_side_sign);
    output.reference.away_side_sign = side;
    output.reference.admitted = true;
    output.reference.reason =
        ObfmApexDisplacementReason::LateralDisplacementRequired;
    output.selected_result = true;
    output.selected_count = 1U;
    output.reason = output.reference.reason;
}

void ObfmApexDisplacement::EvaluateDecorator(
    const bool branch_reached,
    const ObfmApexDisplacementServiceReceipt& service,
    ObfmApexDisplacementSelection& output,
    Status& status) const noexcept
{
    output = ObfmApexDisplacementSelection{};
    output.frame_identity = service.frame_identity;
    output.branch_reached = branch_reached;
    status = Status{};
    if (!branch_reached)
    {
        output.reason = ObfmApexDisplacementReason::DecoratorNotReached;
        return;
    }
    if (!service.selected_result)
    {
        output.reason = ObfmApexDisplacementReason::DecoratorNotAdmitted;
        return;
    }
    if (!service.reference.admitted
        || service.selected_count != 1U
        || !IsValidControlFrameIdentity(service.frame_identity))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.selected = true;
    output.selection_count = 1U;
    output.reason = ObfmApexDisplacementReason::DecoratorSelected;
}

void ObfmApexDisplacement::EnterTask(
    const ObfmApexDisplacementServiceReceipt& service,
    const ObfmApexDisplacementSelection& selection,
    Status& status) noexcept
{
    status = Status{};
    if (task_active_
        || !selection.selected
        || selection.selection_count != 1U
        || !service.reference.admitted
        || !ValidSide(service.reference.away_side_sign))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    task_active_ = true;
    away_side_sign_ = service.reference.away_side_sign;
}

void ObfmApexDisplacement::TickTask(
    const DogfightGeometryFrame& frame,
    const ObfmApexDisplacementServiceReceipt& service,
    const ObfmApexDisplacementTaskInput& input,
    ObfmApexDisplacementTaskReceipt& output,
    Status& status) const noexcept
{
    output = ObfmApexDisplacementTaskReceipt{};
    output.frame_identity = frame.frame_identity;
    output.task_active = task_active_;
    status = Status{};
    if (!task_active_
        || !service.reference.admitted
        || !ValidSide(away_side_sign_))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!input.safety_sample_available)
    {
        output.reason = ObfmApexDisplacementReason::SafetySampleUnavailable;
        return;
    }
    if (!std::isfinite(input.desired_speed_mps)
        || !(input.desired_speed_mps > 0.0)
        || !std::isfinite(input.desired_speed_rate_mps2)
        || !std::isfinite(input.total_load_factor_limit_g)
        || !(input.total_load_factor_limit_g > 1.0)
        || !std::isfinite(input.capture_range_des_m)
        || !(input.capture_range_des_m > 0.0))
    {
        output.reason = ObfmApexDisplacementReason::DeclaredReadyContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    Vector3 target_horizontal = frame.opponent.velocity_ned_mps;
    target_horizontal[2] = 0.0;
    const double horizontal_speed = Norm(target_horizontal);
    if (!(horizontal_speed > 0.0))
    {
        output.reason = ObfmApexDisplacementReason::LagReferenceUnavailable;
        return;
    }
    const Vector3 tangent = Scale(target_horizontal, 1.0 / horizontal_speed);
    const Vector3 lateral{{-tangent[1], tangent[0], 0.0}};
    const Vector3 offset = Scale(
        lateral,
        static_cast<double>(away_side_sign_)
            * service.reference.turn_radius_m);

    output.candidate_valid = true;
    output.candidate_count = 1U;
    output.candidate.aim_point_ned_m = Add(
        service.reference.lag_point_ned_m, offset);
    output.candidate.desired_speed_mps = input.desired_speed_mps;
    output.candidate.desired_speed_rate_mps2 =
        input.desired_speed_rate_mps2;
    output.candidate.path_inversion_allowed = false;
    output.candidate.capture_range_des_m = input.capture_range_des_m;
    output.candidate.total_load_factor_limit_g =
        input.total_load_factor_limit_g;
    output.reason = ObfmApexDisplacementReason::CommandReady;
}

void ObfmApexDisplacement::HaltTask(
    ObfmApexDisplacementHaltReceipt& output) noexcept
{
    output = ObfmApexDisplacementHaltReceipt{};
    output.valid = true;
    output.was_active = task_active_;
    output.clear_command_only_if_still_owner = true;
    ClearTaskLifecycle();
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
