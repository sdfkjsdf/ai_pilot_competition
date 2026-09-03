#include "LadyLuck/guidance/prefire/RootPrefireResponseSelection.hpp"

#include "LadyLuck/common/BoundedScaleProjection.hpp"
#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

namespace
{

using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::prefire::PrefireOptionalDouble;
using LadyLuck::guidance::prefire::PrefireResponseSelectionReason;
using LadyLuck::guidance::prefire::PrefireResponseSelectionReceipt;
using LadyLuck::guidance::prefire::PrefireResponseSelected;
using LadyLuck::guidance::prefire::SnapshotPlaneChangeReason;
using LadyLuck::guidance::prefire::SnapshotPlaneChangeReceipt;
using LadyLuck::guidance::prefire::SnapshotPlaneChangeState;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

Vector3 Add(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
}

class PrefireTransverseScaleEvaluator final
{
public:
    PrefireTransverseScaleEvaluator(
        const Vector3& force_parallel,
        const Vector3& force_perpendicular,
        const Vector3& velocity_hat,
        const double desired_parallel_scalar,
        Vector3& evaluated_specific_force) noexcept
        : force_parallel_(force_parallel),
          force_perpendicular_(force_perpendicular),
          velocity_hat_(velocity_hat),
          desired_parallel_scalar_(desired_parallel_scalar),
          evaluated_specific_force_(evaluated_specific_force)
    {
    }

    bool operator()(
        const double scale,
        double& transverse) const noexcept
    {
        evaluated_specific_force_ = Add(
            force_parallel_,
            Scale(force_perpendicular_, scale));
        const double parallel_residual =
            Dot3(evaluated_specific_force_, velocity_hat_)
            - desired_parallel_scalar_;
        evaluated_specific_force_ = Subtract(
            evaluated_specific_force_,
            Scale(velocity_hat_, parallel_residual));
        const Vector3 recomputed_parallel = Scale(
            velocity_hat_,
            Dot3(evaluated_specific_force_, velocity_hat_));
        const Vector3 recomputed_perpendicular = Subtract(
            evaluated_specific_force_,
            recomputed_parallel);
        transverse = NumpyNorm3(recomputed_perpendicular);
        return std::isfinite(transverse)
            && FiniteVector(evaluated_specific_force_);
    }

private:
    const Vector3& force_parallel_;
    const Vector3& force_perpendicular_;
    const Vector3& velocity_hat_;
    double desired_parallel_scalar_ = 0.0;
    Vector3& evaluated_specific_force_;
};

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

bool CopyLabel(
    std::array<char,
        LadyLuck::guidance::prefire::PrefireResponseReasonLabelCapacity>&
        destination,
    const char* const first,
    const char* const second = nullptr) noexcept
{
    destination.fill('\0');
    if (first == nullptr)
    {
        return false;
    }
    std::size_t cursor = 0U;
    for (const char* source = first; *source != '\0'; ++source)
    {
        if (cursor + 1U >= destination.size())
        {
            return false;
        }
        destination[cursor++] = *source;
    }
    if (second != nullptr)
    {
        for (const char* source = second; *source != '\0'; ++source)
        {
            if (cursor + 1U >= destination.size())
            {
                return false;
            }
            destination[cursor++] = *source;
        }
    }
    destination[cursor] = '\0';
    return true;
}

const char* SelectionReasonStaticLabel(
    const PrefireResponseSelectionReason reason) noexcept
{
    switch (reason)
    {
    case PrefireResponseSelectionReason::NoIncomingCommand:
        return "no_incoming_command";
    case PrefireResponseSelectionReason::NonSnapshotFormKeepsBreak:
        return "non_snapshot_form_keeps_break";
    case PrefireResponseSelectionReason::CandidatePhaseUnresolved:
        return "candidate_phase_unresolved";
    case PrefireResponseSelectionReason::ThreatPredictionUnresolved:
        return "threat_prediction_unresolved";
    case PrefireResponseSelectionReason::BaselineDegenerateOwnSpeedMustBePositive:
        return "baseline_degenerate:own speed must be positive";
    case PrefireResponseSelectionReason::LiftSeedDegenerate:
        return "lift_seed_degenerate";
    case PrefireResponseSelectionReason::AwaySignDegenerate:
        return "away_sign_degenerate";
    case PrefireResponseSelectionReason::SnapshotFormTakesPlaneChange:
        return "snapshot_form_takes_plane_change";
    case PrefireResponseSelectionReason::PrefireResponseSelectionContractRejected:
        return "prefire_response_selection_contract_rejected";
    case PrefireResponseSelectionReason::PlaneChangeBlocked:
    default:
        return "plane_change_blocked:";
    }
}

void SetSelectionReason(
    PrefireResponseSelectionReceipt& output,
    const PrefireResponseSelectionReason reason,
    const SnapshotPlaneChangeReason plane_reason =
        SnapshotPlaneChangeReason::ShadowDisabledDefaultOff) noexcept
{
    output.reason = reason;
    if (reason == PrefireResponseSelectionReason::PlaneChangeBlocked)
    {
        CopyLabel(
            output.exact_reason_label,
            "plane_change_blocked:",
            LadyLuck::guidance::prefire::
                SnapshotPlaneChangeReasonLabel(plane_reason));
    }
    else
    {
        CopyLabel(
            output.exact_reason_label,
            SelectionReasonStaticLabel(reason));
    }
}

void SetPassthrough(
    PrefireResponseSelectionReceipt& output,
    const PrefireResponseSelectionReason reason,
    const bool attack_form_valid,
    const LadyLuck::guidance::prefire::PrefireGunAttackForm attack_form,
    const SnapshotPlaneChangeReason plane_reason =
        SnapshotPlaneChangeReason::ShadowDisabledDefaultOff) noexcept
{
    output = PrefireResponseSelectionReceipt{};
    output.engaged = false;
    output.selected = PrefireResponseSelected::BreakPassthrough;
    output.attack_form_valid = attack_form_valid;
    output.attack_form = attack_form;
    SetSelectionReason(output, reason, plane_reason);
}

void FailSelection(
    SnapshotPlaneChangeState& next_state,
    const SnapshotPlaneChangeState& state,
    PrefireResponseSelectionReceipt& output,
    Status& status,
    const StatusCode code) noexcept
{
    next_state = state;
    SetPassthrough(
        output,
        PrefireResponseSelectionReason::
            PrefireResponseSelectionContractRejected,
        false,
        LadyLuck::guidance::prefire::PrefireGunAttackForm::Indeterminate);
    status.code = code;
}

bool TransverseUnit(
    const Vector3& direction,
    const Vector3& velocity_hat,
    bool& valid,
    Vector3& output,
    Status& status) noexcept
{
    valid = false;
    output = Vector3{};
    if (!FiniteVector(direction) || !FiniteVector(velocity_hat))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    const Vector3 transverse = Subtract(
        direction,
        Scale(velocity_hat, Dot3(direction, velocity_hat)));
    const double norm = NumpyNorm3(transverse);
    if (!std::isfinite(norm))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (norm <= 0.0)
    {
        return true;
    }
    output = Scale(transverse, 1.0 / norm);
    valid = true;
    return true;
}

bool InitialTransverseDirectionFromAttitude(
    const Vector3& own_rpy_rad,
    const Vector3& own_velocity_ned_mps,
    bool& valid,
    Vector3& output,
    Status& status) noexcept
{
    valid = false;
    output = Vector3{};
    if (!FiniteVector(own_rpy_rad)
        || !FiniteVector(own_velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    const double speed = NumpyNorm3(own_velocity_ned_mps);
    if (!std::isfinite(speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (speed <= 0.0)
    {
        return true;
    }
    const double roll = own_rpy_rad[0];
    const double pitch = own_rpy_rad[1];
    const double yaw = own_rpy_rad[2];
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    // Negative third column of the exact Python ZYX body-to-world matrix.
    const Vector3 lift_axis{{
        -(cy * sp * cr + sy * sr),
        -(sy * sp * cr - cy * sr),
        -(cp * cr)}};
    const Vector3 velocity_hat = Scale(
        own_velocity_ned_mps,
        1.0 / speed);
    return TransverseUnit(
        lift_axis,
        velocity_hat,
        valid,
        output,
        status);
}

Vector3 RotateAbout(
    const Vector3& vector,
    const Vector3& axis_hat,
    const double angle_rad) noexcept
{
    const double cos_a = std::cos(angle_rad);
    const double sin_a = std::sin(angle_rad);
    return Add(
        Add(
            Scale(vector, cos_a),
            Scale(Cross(axis_hat, vector), sin_a)),
        Scale(
            axis_hat,
            Dot3(axis_hat, vector) * (1.0 - cos_a)));
}

bool AdmitEntryAcceleration(
    const Vector3& acceleration,
    const Vector3& velocity,
    const double available_nz,
    Vector3& admitted_acceleration,
    double& admitted_transverse,
    Status& status) noexcept
{
    admitted_acceleration = Vector3{};
    admitted_transverse = 0.0;
    if (!FiniteVector(acceleration)
        || !FiniteVector(velocity)
        || !std::isfinite(available_nz))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (available_nz <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const double speed = NumpyNorm3(velocity);
    if (!std::isfinite(speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (speed <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const Vector3 velocity_hat = Scale(velocity, 1.0 / speed);
    const Vector3 gravity{{
        0.0,
        0.0,
        LadyLuck::constants::StandardGravityMps2}};
    const Vector3 desired_specific_force = Subtract(acceleration, gravity);
    const double desired_parallel_scalar = Dot3(
        desired_specific_force,
        velocity_hat);
    const Vector3 force_parallel = Scale(
        velocity_hat,
        desired_parallel_scalar);
    const Vector3 force_perp = Subtract(
        desired_specific_force,
        force_parallel);
    const double raw_transverse = NumpyNorm3(force_perp);
    const double available = available_nz
        * LadyLuck::constants::StandardGravityMps2;
    if (!std::isfinite(raw_transverse) || !std::isfinite(available))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (raw_transverse < available)
    {
        admitted_acceleration = acceleration;
        admitted_transverse = raw_transverse;
        return true;
    }

    const double initial_scale = std::nextafter(available, 0.0)
        / raw_transverse;
    Vector3 evaluated_specific_force{};
    const PrefireTransverseScaleEvaluator evaluate_scale{
        force_parallel,
        force_perp,
        velocity_hat,
        desired_parallel_scalar,
        evaluated_specific_force};
    double admitted_scale = 0.0;
    if (!LadyLuck::common::LargestRepresentableScaleBelowBound(
            initial_scale,
            available,
            evaluate_scale,
            admitted_scale,
            admitted_transverse)
        || !evaluate_scale(admitted_scale, admitted_transverse)
        || !(admitted_transverse < available))
    {
        // A finite physical sample can still be unrepresentable at the strict
        // binary64 boundary.  That is ordinary plane-change nonadmission; the
        // caller keeps the already valid BREAK command in the same frame.
        return false;
    }
    admitted_acceleration = Add(gravity, evaluated_specific_force);
    return FiniteVector(admitted_acceleration);
}

void BlockPlaneChange(
    const SnapshotPlaneChangeReason reason,
    SnapshotPlaneChangeReceipt& output) noexcept
{
    output = SnapshotPlaneChangeReceipt{};
    output.reason = reason;
}

void SnapshotPlaneChangeStep(
    const SnapshotPlaneChangeState& state,
    const Vector3& own_rpy_rad,
    const Vector3& own_velocity_ned_mps,
    const double baseline_transverse_magnitude_mps2,
    const double official_cone_rad,
    const double predicted_solution_time_s,
    const double max_roll_rate_radps,
    const double own_available_nz_lower_g,
    const double away_sign,
    const double dt_s,
    SnapshotPlaneChangeState& next_state,
    SnapshotPlaneChangeReceipt& output,
    Status& status) noexcept
{
    next_state = state;
    output = SnapshotPlaneChangeReceipt{};
    status = Status{};
    if (away_sign != -1.0 && away_sign != 1.0)
    {
        status.code = StatusCode::InvalidArgument;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    if (!std::isfinite(dt_s)
        || !std::isfinite(max_roll_rate_radps)
        || !std::isfinite(official_cone_rad)
        || !std::isfinite(predicted_solution_time_s)
        || !std::isfinite(baseline_transverse_magnitude_mps2)
        || !std::isfinite(own_available_nz_lower_g))
    {
        status.code = StatusCode::NonFiniteInput;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    if (dt_s <= 0.0
        || max_roll_rate_radps <= 0.0
        || official_cone_rad <= 0.0
        || predicted_solution_time_s < 0.0
        || baseline_transverse_magnitude_mps2 < 0.0
        || own_available_nz_lower_g <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    if (predicted_solution_time_s == 0.0)
    {
        BlockPlaneChange(
            SnapshotPlaneChangeReason::SolutionTimeInvalid,
            output);
        return;
    }
    if (!FiniteVector(own_velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    const double speed = NumpyNorm3(own_velocity_ned_mps);
    if (!std::isfinite(speed))
    {
        status.code = StatusCode::NonFiniteInput;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    if (speed <= 0.0)
    {
        BlockPlaneChange(SnapshotPlaneChangeReason::SpeedNotPositive, output);
        return;
    }
    const Vector3 velocity_hat = Scale(
        own_velocity_ned_mps,
        1.0 / speed);

    bool carried_valid = false;
    Vector3 carried{};
    if (!state.commanded_transverse_direction_valid)
    {
        if (!InitialTransverseDirectionFromAttitude(
                own_rpy_rad,
                own_velocity_ned_mps,
                carried_valid,
                carried,
                status))
        {
            output.reason = SnapshotPlaneChangeReason::
                SnapshotPlaneChangeContractRejected;
            return;
        }
        if (!carried_valid)
        {
            BlockPlaneChange(
                SnapshotPlaneChangeReason::LiftAxisDegenerate,
                output);
            return;
        }
    }
    else
    {
        if (!TransverseUnit(
                state.commanded_transverse_direction_ned,
                velocity_hat,
                carried_valid,
                carried,
                status))
        {
            output.reason = SnapshotPlaneChangeReason::
                SnapshotPlaneChangeContractRejected;
            return;
        }
        if (!carried_valid)
        {
            BlockPlaneChange(
                SnapshotPlaneChangeReason::CarriedDirectionDegenerate,
                output);
            return;
        }
    }

    const double threat_rate = official_cone_rad
        / predicted_solution_time_s;
    const double applied_rate = (std::min)(
        threat_rate,
        max_roll_rate_radps);
    const double rotation_bound = max_roll_rate_radps * dt_s;
    const double applied = away_sign * applied_rate * dt_s;
    const Vector3 rotated = RotateAbout(carried, velocity_hat, applied);
    bool rotated_valid = false;
    Vector3 rotated_unit{};
    if (!TransverseUnit(
            rotated,
            velocity_hat,
            rotated_valid,
            rotated_unit,
            status))
    {
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }
    if (!rotated_valid)
    {
        BlockPlaneChange(
            SnapshotPlaneChangeReason::RotatedDirectionDegenerate,
            output);
        return;
    }

    const Vector3 gravity{{
        0.0,
        0.0,
        LadyLuck::constants::StandardGravityMps2}};
    const Vector3 desired_total = Add(
        gravity,
        Scale(rotated_unit, baseline_transverse_magnitude_mps2));
    Vector3 admitted_total{};
    double admitted_transverse = 0.0;
    if (!AdmitEntryAcceleration(
            desired_total,
            own_velocity_ned_mps,
            own_available_nz_lower_g,
            admitted_total,
            admitted_transverse,
            status))
    {
        if (status.code == StatusCode::Ok)
        {
            BlockPlaneChange(
                SnapshotPlaneChangeReason::TransverseProjectionUnavailable,
                output);
        }
        else
        {
            output.reason = SnapshotPlaneChangeReason::
                SnapshotPlaneChangeContractRejected;
        }
        return;
    }
    const double admitted_magnitude = NumpyNorm3(
        Subtract(admitted_total, gravity));
    if (!std::isfinite(admitted_magnitude)
        || !std::isfinite(admitted_transverse))
    {
        status.code = StatusCode::NonFiniteInput;
        output.reason = SnapshotPlaneChangeReason::
            SnapshotPlaneChangeContractRejected;
        return;
    }

    next_state.commanded_transverse_direction_valid = true;
    next_state.commanded_transverse_direction_ned = rotated_unit;
    output.engaged = true;
    output.reason = SnapshotPlaneChangeReason::ReachablePlaneChangeStep;
    output.commanded_transverse_direction_valid = true;
    output.commanded_transverse_direction_ned = rotated_unit;
    output.applied_rotation_rad = PrefireOptionalDouble{true, applied};
    output.threat_derived_rate_radps = PrefireOptionalDouble{
        true,
        threat_rate};
    output.rotation_bound_rad = PrefireOptionalDouble{
        true,
        rotation_bound};
    output.baseline_transverse_magnitude_mps2 = PrefireOptionalDouble{
        true,
        baseline_transverse_magnitude_mps2};
    output.admitted_transverse_magnitude_mps2 = PrefireOptionalDouble{
        true,
        admitted_magnitude};
    output.a_cmd_total_valid = true;
    output.a_cmd_total_ned_mps2 = admitted_total;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

const char* SnapshotPlaneChangeReasonLabel(
    const SnapshotPlaneChangeReason reason) noexcept
{
    switch (reason)
    {
    case SnapshotPlaneChangeReason::SolutionTimeInvalid:
        return "solution_time_invalid";
    case SnapshotPlaneChangeReason::SpeedNotPositive:
        return "speed_not_positive";
    case SnapshotPlaneChangeReason::LiftAxisDegenerate:
        return "lift_axis_degenerate";
    case SnapshotPlaneChangeReason::CarriedDirectionDegenerate:
        return "carried_direction_degenerate";
    case SnapshotPlaneChangeReason::RotatedDirectionDegenerate:
        return "rotated_direction_degenerate";
    case SnapshotPlaneChangeReason::ReachablePlaneChangeStep:
        return "reachable_plane_change_step";
    case SnapshotPlaneChangeReason::SnapshotPlaneChangeContractRejected:
        return "snapshot_plane_change_contract_rejected";
    case SnapshotPlaneChangeReason::TransverseProjectionUnavailable:
        return "transverse_projection_unavailable";
    case SnapshotPlaneChangeReason::ShadowDisabledDefaultOff:
    default:
        return "shadow_disabled_default_off";
    }
}

const char* PrefireResponseSelectedLabel(
    const PrefireResponseSelected selected) noexcept
{
    return selected == PrefireResponseSelected::SnapshotPlaneChange
        ? "SNAPSHOT_PLANE_CHANGE"
        : "BREAK_PASSTHROUGH";
}

const char* PrefireSnapshotPlaneChangeBehaviorLabel() noexcept
{
    return "PREFIRE_SNAPSHOT_PLANE_CHANGE";
}

void ResetSnapshotPlaneChangeState(
    SnapshotPlaneChangeState& state) noexcept
{
    state = SnapshotPlaneChangeState{};
}

void PathHoldTransverseMagnitudeMps2(
    const Vector3& own_velocity_ned_mps,
    double& output,
    Status& status) noexcept
{
    output = 0.0;
    status = Status{};
    if (!FiniteVector(own_velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double speed = NumpyNorm3(own_velocity_ned_mps);
    if (!std::isfinite(speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (speed <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    const Vector3 velocity_hat = Scale(
        own_velocity_ned_mps,
        1.0 / speed);
    const Vector3 gravity{{
        0.0,
        0.0,
        constants::StandardGravityMps2}};
    const Vector3 perpendicular = Subtract(
        gravity,
        Scale(velocity_hat, Dot3(gravity, velocity_hat)));
    output = NumpyNorm3(perpendicular);
    if (!std::isfinite(output))
    {
        status.code = StatusCode::NonFiniteInput;
    }
}

void PrefireAwaySign(
    const Vector3& commanded_or_lift_direction_ned,
    const Vector3& own_position_ned_m,
    const Vector3& attacker_position_ned_m,
    const Vector3& own_velocity_ned_mps,
    PrefireOptionalDouble& output,
    Status& status) noexcept
{
    output = PrefireOptionalDouble{};
    status = Status{};
    if (!FiniteVector(commanded_or_lift_direction_ned)
        || !FiniteVector(own_position_ned_m)
        || !FiniteVector(attacker_position_ned_m)
        || !FiniteVector(own_velocity_ned_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double speed = NumpyNorm3(own_velocity_ned_mps);
    if (!std::isfinite(speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (speed <= 0.0)
    {
        return;
    }
    const Vector3 velocity_hat = Scale(
        own_velocity_ned_mps,
        1.0 / speed);
    const Vector3 offset = Subtract(
        attacker_position_ned_m,
        own_position_ned_m);
    const Vector3 offset_perp = Subtract(
        offset,
        Scale(velocity_hat, Dot3(offset, velocity_hat)));
    const Vector3 direction_perp = Subtract(
        commanded_or_lift_direction_ned,
        Scale(
            velocity_hat,
            Dot3(commanded_or_lift_direction_ned, velocity_hat)));
    const double offset_norm = NumpyNorm3(offset_perp);
    const double direction_norm = NumpyNorm3(direction_perp);
    if (!std::isfinite(offset_norm) || !std::isfinite(direction_norm))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (offset_norm <= 0.0 || direction_norm <= 0.0)
    {
        return;
    }
    const Vector3 offset_hat = Scale(offset_perp, 1.0 / offset_norm);
    const Vector3 direction_hat = Scale(
        direction_perp,
        1.0 / direction_norm);
    const double sine = Dot3(
        Cross(offset_hat, direction_hat),
        velocity_hat);
    const double cosine = Dot3(offset_hat, direction_hat);
    const double angle = std::atan2(sine, cosine);
    output.has_value = true;
    output.value = angle < 0.0 ? -1.0 : 1.0;
}

void SelectPrefireResponse(
    const SnapshotPlaneChangeState& state,
    const bool incoming_gun_command_available,
    const PrefireGunAttackFormObservation* const attack_form_observation,
    const RootPrefireThreatShadowReceipt* const shadow_receipt,
    const Vector3& own_rpy_rad,
    const Vector3& own_position_ned_m,
    const Vector3& own_velocity_ned_mps,
    const Vector3& attacker_position_ned_m,
    const double max_roll_rate_radps,
    const double dt_s,
    SnapshotPlaneChangeState& next_state,
    PrefireResponseSelectionReceipt& output,
    Status& status) noexcept
{
    next_state = state;
    output = PrefireResponseSelectionReceipt{};
    status = Status{};
    if (!incoming_gun_command_available)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::NoIncomingCommand,
            false,
            PrefireGunAttackForm::Indeterminate);
        return;
    }

    bool attack_form_valid = false;
    PrefireGunAttackForm attack_form = PrefireGunAttackForm::Indeterminate;
    if (attack_form_observation != nullptr)
    {
        if (!attack_form_observation->valid)
        {
            FailSelection(
                next_state,
                state,
                output,
                status,
                StatusCode::InvalidArgument);
            return;
        }
        attack_form_valid = true;
        attack_form = attack_form_observation->attack_form;
    }
    if (attack_form != PrefireGunAttackForm::Snapshot)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::NonSnapshotFormKeepsBreak,
            attack_form_valid,
            attack_form);
        return;
    }

    const RootPrefirePhaseObservation* candidate_phase = nullptr;
    if (shadow_receipt != nullptr)
    {
        if (shadow_receipt->phase_observation_count
                > RootPrefireNonScratchPhaseCount
            || (shadow_receipt->candidate_phase_valid
                && !shadow_receipt->prefire_break_candidate))
        {
            FailSelection(
                next_state,
                state,
                output,
                status,
                StatusCode::InvalidArgument);
            return;
        }
        if (shadow_receipt->candidate_phase_valid)
        {
            for (std::size_t index = 0U;
                 index < shadow_receipt->phase_observation_count;
                 ++index)
            {
                const RootPrefirePhaseObservation& observation =
                    shadow_receipt->phase_observations[index];
                if (observation.phase.id
                    == shadow_receipt->candidate_phase)
                {
                    candidate_phase = &observation;
                    break;
                }
            }
            if (candidate_phase == nullptr)
            {
                FailSelection(
                    next_state,
                    state,
                    output,
                    status,
                    StatusCode::InvalidConfiguration);
                return;
            }
        }
    }
    if (candidate_phase == nullptr)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::CandidatePhaseUnresolved,
            true,
            attack_form);
        return;
    }
    const double cone_rad = candidate_phase->phase.angle_rad;
    if (!candidate_phase->time_to_solution_s.has_value)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::ThreatPredictionUnresolved,
            true,
            attack_form);
        return;
    }
    const double solution_time_s =
        candidate_phase->time_to_solution_s.value;
    if (!FiniteVector(own_velocity_ned_mps))
    {
        FailSelection(
            next_state,
            state,
            output,
            status,
            StatusCode::NonFiniteInput);
        return;
    }
    const double own_speed = NumpyNorm3(own_velocity_ned_mps);
    if (!std::isfinite(own_speed))
    {
        FailSelection(
            next_state,
            state,
            output,
            status,
            StatusCode::NonFiniteInput);
        return;
    }
    if (own_speed <= 0.0)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::
                BaselineDegenerateOwnSpeedMustBePositive,
            true,
            attack_form);
        return;
    }
    double baseline = 0.0;
    PathHoldTransverseMagnitudeMps2(
        own_velocity_ned_mps,
        baseline,
        status);
    if (status.code != StatusCode::Ok)
    {
        FailSelection(
            next_state,
            state,
            output,
            status,
            status.code);
        return;
    }

    Vector3 reference_direction{};
    if (state.commanded_transverse_direction_valid)
    {
        reference_direction = state.commanded_transverse_direction_ned;
    }
    else
    {
        bool reference_valid = false;
        if (!InitialTransverseDirectionFromAttitude(
                own_rpy_rad,
                own_velocity_ned_mps,
                reference_valid,
                reference_direction,
                status))
        {
            FailSelection(
                next_state,
                state,
                output,
                status,
                status.code);
            return;
        }
        if (!reference_valid)
        {
            SetPassthrough(
                output,
                PrefireResponseSelectionReason::LiftSeedDegenerate,
                true,
                attack_form);
            return;
        }
    }
    PrefireOptionalDouble away{};
    PrefireAwaySign(
        reference_direction,
        own_position_ned_m,
        attacker_position_ned_m,
        own_velocity_ned_mps,
        away,
        status);
    if (status.code != StatusCode::Ok)
    {
        FailSelection(
            next_state,
            state,
            output,
            status,
            status.code);
        return;
    }
    if (!away.has_value)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::AwaySignDegenerate,
            true,
            attack_form);
        return;
    }

    SnapshotPlaneChangeState candidate_state{};
    SnapshotPlaneChangeReceipt plane_change{};
    SnapshotPlaneChangeStep(
        state,
        own_rpy_rad,
        own_velocity_ned_mps,
        baseline,
        cone_rad,
        solution_time_s,
        max_roll_rate_radps,
        1.0,
        away.value,
        dt_s,
        candidate_state,
        plane_change,
        status);
    if (status.code != StatusCode::Ok)
    {
        FailSelection(
            next_state,
            state,
            output,
            status,
            status.code);
        return;
    }
    if (!plane_change.engaged)
    {
        SetPassthrough(
            output,
            PrefireResponseSelectionReason::PlaneChangeBlocked,
            true,
            attack_form,
            plane_change.reason);
        return;
    }

    next_state = candidate_state;
    output = PrefireResponseSelectionReceipt{};
    output.engaged = true;
    output.selected = PrefireResponseSelected::SnapshotPlaneChange;
    SetSelectionReason(
        output,
        PrefireResponseSelectionReason::SnapshotFormTakesPlaneChange);
    output.attack_form_valid = true;
    output.attack_form = attack_form;
    output.away_sign = away;
    output.baseline_transverse_magnitude_mps2 = PrefireOptionalDouble{
        true,
        baseline};
    output.plane_change_receipt_valid = true;
    output.plane_change = plane_change;
    output.command_overlay.valid = true;
    output.command_overlay.direct_load_vector_acceleration_ned_mps2 =
        plane_change.a_cmd_total_ned_mps2;
}

void ApplyPrefireSnapshotCommandOverlay(
    const ControlIntent& upstream,
    const PrefireSnapshotCommandOverlay& overlay,
    const DoctrineBehaviorId snapshot_behavior_id,
    const std::uint32_t selected_writer_id,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    if (!overlay.valid
        || !FiniteVector(
            overlay.direct_load_vector_acceleration_ned_mps2)
        || !overlay.clear_direct_p_cmd
        || !overlay.clear_direct_nz_cmd
        || !overlay.clear_direct_beta_cmd
        || !overlay.clear_direct_bank_cmd
        || !overlay.clear_direct_turn_rate_cmd
        || !overlay.clear_direct_acceleration_ned
        || !overlay.clear_direct_acceleration_roll_rate_reference
        || overlay.direct_acceleration_tracking_enabled
        || overlay.direct_acceleration_tracking_observation_only
        || overlay.direct_acceleration_magnitude_tracking_enabled
        || overlay.direct_acceleration_loaded_roll_enabled
        || overlay.direct_acceleration_load_component_compensation_enabled
        || overlay.direct_acceleration_yaw_coordination_enabled
        || overlay.direct_acceleration_roll_priority_yaw_enabled
        || snapshot_behavior_id == DoctrineBehaviorId::Invalid
        || selected_writer_id == ControlIntentWriterNone)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    upstream.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    output = upstream;
    output.behavior_id = snapshot_behavior_id;
    output.writer_id = selected_writer_id;
    output.direct_load_vector_acceleration_ned_mps2.has_value = true;
    output.direct_load_vector_acceleration_ned_mps2.value =
        overlay.direct_load_vector_acceleration_ned_mps2;
    output.direct_p_cmd_radps = IntentOptionalValue<double>{};
    output.direct_nz_cmd_g = IntentOptionalValue<double>{};
    output.direct_beta_cmd_rad = IntentOptionalValue<double>{};
    output.direct_bank_cmd_rad = IntentOptionalValue<double>{};
    output.direct_turn_rate_cmd_radps = IntentOptionalValue<double>{};
    output.direct_acceleration_ned_mps2 =
        IntentOptionalValue<Vector3>{};
    output.direct_acceleration_roll_rate_reference_radps =
        IntentOptionalValue<double>{};
    output.direct_acceleration_tracking_enabled = false;
    output.direct_acceleration_tracking_observation_only = false;
    output.direct_acceleration_magnitude_tracking_enabled = false;
    output.direct_acceleration_loaded_roll_enabled = false;
    output.direct_acceleration_load_component_compensation_enabled = false;
    output.direct_acceleration_yaw_coordination_enabled = false;
    output.direct_acceleration_roll_priority_yaw_enabled = false;
    output.route_kind = ControlRouteKind::DirectLoadVectorAcceleration;
    output.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
    }
}

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
