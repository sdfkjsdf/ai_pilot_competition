#include "LadyLuck/runtime/TacticalControlCore.hpp"


#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/control/direct_body/DirectBodyReferenceAdapter.hpp"
#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using LadyLuck::CommandFeedback;
using LadyLuck::KinematicObservation;
using LadyLuck::Matrix3RowMajor;
using LadyLuck::PlaneState;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::control::tecs_cis::NormalizedControlCommand;
using LadyLuck::control::route5::Route5GuidanceOutput;
using LadyLuck::control::route5::Route5GuidanceSnapshot;
using LadyLuck::control::tecs_cis::BodyRateLoadEnergyCommand;
using LadyLuck::control::tecs_cis::TecsCisOutput;
using LadyLuck::control::tecs_cis::TecsCisSnapshot;

constexpr double DirectNedThrottleBiasToSpeedMps = 40.0;

Status Failure(const StatusCode code) noexcept
{
    Status status{};
    status.code = code;
    return status;
}

bool ClassifyCommandOutcome(
    const LadyLuck::ControlIntent& intent,
    LadyLuck::runtime::ControlCommandOutcome& output) noexcept
{
    using LadyLuck::ControlIntentWriterAutoGcasRecovery;
    using LadyLuck::ControlIntentWriterDbfmHardTurn;
    using LadyLuck::ControlIntentWriterHabfm;
    using LadyLuck::ControlIntentWriterObfmLag;
    using LadyLuck::ControlRouteKind;
    using LadyLuck::DoctrineBehaviorId;
    using LadyLuck::DoctrineModeId;
    using LadyLuck::runtime::ControlCommandOutcome;

    output = ControlCommandOutcome::InputRejected;
    if (intent.writer_id == ControlIntentWriterAutoGcasRecovery
        && intent.route_kind == ControlRouteKind::SafetyRecovery
        && intent.behavior_id == DoctrineBehaviorId::AutoGcasRecovery
        && intent.mode_id == DoctrineModeId::Safety)
    {
        output = ControlCommandOutcome::Safety;
        return true;
    }
    if (intent.route_kind == ControlRouteKind::SafetyRecovery)
    {
        return false;
    }

    const bool obfm_terminal =
        intent.writer_id == ControlIntentWriterObfmLag
        && intent.behavior_id == DoctrineBehaviorId::Lag
        && intent.mode_id == DoctrineModeId::Obfm;
    const bool habfm_terminal =
        intent.writer_id == ControlIntentWriterHabfm
        && intent.mode_id == DoctrineModeId::Habfm
        && (intent.behavior_id == DoctrineBehaviorId::HabfmMergeApproach
            || intent.behavior_id == DoctrineBehaviorId::HabfmEnergyFight
            || intent.behavior_id == DoctrineBehaviorId::HabfmOneCircle
            || intent.behavior_id == DoctrineBehaviorId::HabfmTwoCircle);
    const bool dbfm_terminal =
        intent.writer_id == ControlIntentWriterDbfmHardTurn
        && intent.behavior_id == DoctrineBehaviorId::DbfmHardTurn
        && intent.mode_id == DoctrineModeId::Dbfm;
    output = obfm_terminal || habfm_terminal || dbfm_terminal
        ? ControlCommandOutcome::CurrentBase
        : ControlCommandOutcome::Tactical;
    return true;
}

KinematicObservation ConvertInput(
    const PlaneKinematicObservationV1& input) noexcept
{
    KinematicObservation output{};
    output.frame_index = input.frame_index;
    output.plane_id = input.plane_id;
    output.force_side = input.force_side;
    output.position_neu_m = {{
        input.position_n_m,
        input.position_e_m,
        input.position_up_m}};
    output.rpy_deg = {{
        input.roll_deg,
        input.pitch_deg,
        input.yaw_deg}};
    output.velocity_body_mps = {{
        input.body_u_mps,
        input.body_v_mps,
        input.body_w_mps}};
    return output;
}

double SaturateFiniteAxis(
    const double value,
    const double lower,
    const double upper) noexcept
{
    if (!std::isfinite(value))
    {
        // Preserve nonfinite evidence for the final finite-value rejection.
        // Saturating NaN/Inf would disguise an invalid FCS result as a valid
        // maximum actuator command.
        return value;
    }
    return (std::max)(lower, (std::min)(value, upper));
}

NormalizedControlCommand SaturateFiniteControl(
    const NormalizedControlCommand& input) noexcept
{
    return NormalizedControlCommand{
        SaturateFiniteAxis(input.aileron, -1.0, 1.0),
        SaturateFiniteAxis(input.elevator, -1.0, 1.0),
        SaturateFiniteAxis(input.rudder, -1.0, 1.0),
        SaturateFiniteAxis(input.throttle, 0.0, 1.0),
        input.valid};
}

ControlValue ToWire(const NormalizedControlCommand& input) noexcept
{
    const NormalizedControlCommand saturated = SaturateFiniteControl(input);
    return ControlValue{
        static_cast<float>(saturated.aileron),
        static_cast<float>(saturated.elevator),
        static_cast<float>(saturated.rudder),
        static_cast<float>(saturated.throttle)};
}

bool FiniteWire(const ControlValue& value) noexcept
{
    return std::isfinite(static_cast<double>(value.RollCMD))
        && std::isfinite(static_cast<double>(value.PitchCMD))
        && std::isfinite(static_cast<double>(value.RudderCMD))
        && std::isfinite(static_cast<double>(value.Throttle));
}

bool PlaneWireFinite(const PlaneKinematicObservationV1& input) noexcept
{
    return std::isfinite(static_cast<double>(input.position_n_m))
        && std::isfinite(static_cast<double>(input.position_e_m))
        && std::isfinite(static_cast<double>(input.position_up_m))
        && std::isfinite(static_cast<double>(input.roll_deg))
        && std::isfinite(static_cast<double>(input.pitch_deg))
        && std::isfinite(static_cast<double>(input.yaw_deg))
        && std::isfinite(static_cast<double>(input.body_u_mps))
        && std::isfinite(static_cast<double>(input.body_v_mps))
        && std::isfinite(static_cast<double>(input.body_w_mps));
}

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

void DirectEulerDcmNedToBody(
    const Vector3& rpy_rad,
    Matrix3RowMajor& output) noexcept
{
    // d90 direct-NED uses (Tx @ Ty) @ Tz, not its quaternion-equivalent DCM.
    // Preserve that operation order because the matrices are not bit-identical.
    const double cosine_roll = std::cos(rpy_rad[0]);
    const double sine_roll = std::sin(rpy_rad[0]);
    const double cosine_pitch = std::cos(rpy_rad[1]);
    const double sine_pitch = std::sin(rpy_rad[1]);
    const double cosine_yaw = std::cos(rpy_rad[2]);
    const double sine_yaw = std::sin(rpy_rad[2]);
    const Matrix3RowMajor roll_matrix{{
        1.0, 0.0, 0.0,
        0.0, cosine_roll, sine_roll,
        0.0, -sine_roll, cosine_roll}};
    const Matrix3RowMajor pitch_matrix{{
        cosine_pitch, 0.0, -sine_pitch,
        0.0, 1.0, 0.0,
        sine_pitch, 0.0, cosine_pitch}};
    const Matrix3RowMajor yaw_matrix{{
        cosine_yaw, sine_yaw, 0.0,
        -sine_yaw, cosine_yaw, 0.0,
        0.0, 0.0, 1.0}};
    output = LadyLuck::MatrixProduct(
        LadyLuck::MatrixProduct(roll_matrix, pitch_matrix),
        yaw_matrix);
}

void ValidateControlRouteSubset(
    const LadyLuck::ControlIntent& intent,
    LadyLuck::Status& status) noexcept
{
    status = LadyLuck::Status{};
    if (intent.desired_speed_mps < 0.0)
    {
        status = Failure(LadyLuck::StatusCode::InvalidArgument);
        return;
    }

    using LadyLuck::ControlRouteKind;
    switch (intent.route_kind)
    {
    case ControlRouteKind::AimPoint:
    case ControlRouteKind::DirectLoadVectorAcceleration:
        // Route-5 is executed transactionally below. Its configured throttle
        // bias and all derived arithmetic are admitted by the projected copy
        // before any persistent NMu/velocity-bank state is committed.
        return;
    case ControlRouteKind::DirectBodyReferences:
        if (!intent.direct_p_cmd_radps.has_value
            || !intent.direct_nz_cmd_g.has_value
            || intent.direct_accel_cmd_mps2.has_value)
        {
            status = Failure(LadyLuck::StatusCode::InvalidArgument);
        }
        return;
    case ControlRouteKind::DirectNedAcceleration:
    {
        // Optional beta/direct-accel/allocation fields do not own the base
        // finite NED acceleration. Unsupported overlays remain nonadmitted;
        // they must not erase the current-frame base reference.
        const double throttle_bias = intent.throttle_bias.has_value
            ? intent.throttle_bias.value
            : 0.0;
        const double speed_bias =
            DirectNedThrottleBiasToSpeedMps * throttle_bias;
        const double desired_speed = intent.desired_speed_mps + speed_bias;
        if (!std::isfinite(speed_bias) || !std::isfinite(desired_speed))
        {
            status = Failure(LadyLuck::StatusCode::NonFiniteInput);
        }
        else if (desired_speed < 0.0)
        {
            status = Failure(LadyLuck::StatusCode::InvalidArgument);
        }
        return;
    }
    case ControlRouteKind::DirectBankTurn:
    case ControlRouteKind::Invalid:
    default:
        // DirectBankTurn remains a schema-level representation only; this
        // control core has no body-rate/load adapter for it.
        status = Failure(LadyLuck::StatusCode::InvalidArgument);
        return;
    }
}

} // namespace

namespace LadyLuck
{
namespace runtime
{
namespace current_cis_v4_projection_math
{

bool SafeAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    // Inclusive rejection is intentional.  At exact rounded guard equality,
    // performing the operation can cross the RN-even overflow tie even when
    // the rounded precheck expression still equals DBL_MAX.
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    return std::isfinite(output);
}

bool SafeSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(right))
    {
        return false;
    }
    return SafeAdd(left, -right, output);
}

bool SafeMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if (left == 0.0 || right == 0.0)
    {
        output = left * right;
        return true;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    const double absolute_right = std::fabs(right);
    // Do not evaluate max / |right| for a sub-unit multiplier: that guard
    // expression can itself overflow even though the multiplication cannot.
    if (absolute_right >= 1.0
        && std::fabs(left) >= maximum / absolute_right)
    {
        return false;
    }
    output = left * right;
    return std::isfinite(output);
}

bool SafeDivide(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(numerator)
        || !std::isfinite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    const double absolute_denominator = std::fabs(denominator);
    if (absolute_denominator < 1.0
        && std::fabs(numerator)
            >= (std::numeric_limits<double>::max)()
                * absolute_denominator)
    {
        return false;
    }
    output = numerator / denominator;
    return std::isfinite(output);
}

} // namespace current_cis_v4_projection_math
} // namespace runtime
} // namespace LadyLuck

namespace
{

using LadyLuck::runtime::current_cis_v4_projection_math::SafeAdd;
using LadyLuck::runtime::current_cis_v4_projection_math::SafeDivide;
using LadyLuck::runtime::current_cis_v4_projection_math::SafeMultiply;
using LadyLuck::runtime::current_cis_v4_projection_math::SafeSubtract;

bool BoundedControllerCommand(
    const NormalizedControlCommand& command) noexcept
{
    return command.valid
        && std::isfinite(command.aileron)
        && std::isfinite(command.elevator)
        && std::isfinite(command.rudder)
        && std::isfinite(command.throttle)
        && command.aileron >= -1.0 && command.aileron <= 1.0
        && command.elevator >= -1.0 && command.elevator <= 1.0
        && command.rudder >= -1.0 && command.rudder <= 1.0
        && command.throttle >= -1.0 && command.throttle <= 1.0;
}

void BuildBodyRateReference(
    const Route5GuidanceOutput& route,
    const bool integrator_hold,
    BodyRateLoadEnergyCommand& output) noexcept
{
    output = BodyRateLoadEnergyCommand{};
    output.frame_identity = route.frame_identity;
    output.valid = route.valid;
    output.p_cmd_radps = route.p_cmd_radps;
    output.q_cmd_radps = route.q_cmd_radps;
    output.r_cmd_radps = route.r_cmd_radps;
    output.nz_cmd_g = route.nz_cmd_g;
    output.desired_speed_mps = route.desired_speed_mps;
    output.desired_speed_rate_mps2 = route.desired_speed_rate_mps2;
    output.flight_path_angle_cmd_rad = route.flight_path_angle_cmd_rad;
    output.specific_energy_rate_bias_m2ps3 =
        route.specific_energy_rate_bias_m2ps3;
    output.integrator_hold = integrator_hold;
}

bool EvaluateEnergyIntegratorHold(
    const LadyLuck::EstimatorOutputV6& estimate,
    const double flight_path_angle_cmd_rad,
    const bool current_handoff_active,
    const bool previous_auto_gcas_active,
    bool& next_handoff_active,
    bool& hold) noexcept
{
    next_handoff_active = current_handoff_active;
    hold = false;
    const double values[] = {
        estimate.V,
        estimate.u,
        estimate.v,
        estimate.w,
        estimate.roll,
        estimate.pitch,
        flight_path_angle_cmd_rad};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    if (estimate.V <= 0.0)
    {
        return false;
    }

    const double sine_pitch = std::sin(estimate.pitch);
    const double cosine_pitch = std::cos(estimate.pitch);
    const double sine_roll = std::sin(estimate.roll);
    const double cosine_roll = std::cos(estimate.roll);
    double first = 0.0;
    double second = 0.0;
    double third = 0.0;
    double partial = 0.0;
    double climb_rate_mps = 0.0;
    if (!SafeMultiply(estimate.u, sine_pitch, first)
        || !SafeMultiply(estimate.v, sine_roll, second)
        || !SafeMultiply(second, cosine_pitch, second)
        || !SafeSubtract(first, second, partial)
        || !SafeMultiply(estimate.w, cosine_roll, third)
        || !SafeMultiply(third, cosine_pitch, third)
        || !SafeSubtract(partial, third, climb_rate_mps))
    {
        return false;
    }
    double sine_gamma = 0.0;
    if (!SafeDivide(climb_rate_mps, estimate.V, sine_gamma))
    {
        return false;
    }
    sine_gamma = (std::min)(1.0, (std::max)(-1.0, sine_gamma));
    const double gamma_rad = std::asin(sine_gamma);
    if (!std::isfinite(gamma_rad))
    {
        return false;
    }

    hold = current_handoff_active;
    if (previous_auto_gcas_active)
    {
        hold = true;
    }
    else if (hold)
    {
        const double gamma_error = std::fabs(
            gamma_rad - flight_path_angle_cmd_rad);
        const double wrapped_roll = std::atan2(
            std::sin(estimate.roll),
            std::cos(estimate.roll));
        constexpr double GammaErrorGateRad =
            5.0 * LadyLuck::constants::Pi / 180.0;
        constexpr double RollErrorGateRad =
            15.0 * LadyLuck::constants::Pi / 180.0;
        if (gamma_error <= GammaErrorGateRad
            && std::fabs(wrapped_roll) <= RollErrorGateRad)
        {
            hold = false;
        }
    }
    next_handoff_active = hold;
    return true;
}

bool ForwardThrustForReference(
    const LadyLuck::runtime::CurrentCisV4EnergyProjectionReceipt& receipt,
    const double reference_m2ps3,
    double& thrust_n) noexcept
{
    thrust_n = 0.0;
    double rate_error = 0.0;
    double proportional = 0.0;
    double integral = 0.0;
    double feedback = 0.0;
    double rate_command = 0.0;
    double value = 0.0;
    if (!SafeSubtract(
            reference_m2ps3,
            receipt.specific_energy_rate_measured_m2ps3,
            rate_error)
        || !SafeMultiply(
            receipt.energy_error_gain_per_s,
            receipt.total_energy_error_m2ps2,
            proportional)
        || !SafeMultiply(
            receipt.energy_integral_gain_per_s2,
            receipt.energy_integral_before_m2ps,
            integral)
        || !SafeMultiply(
            receipt.energy_rate_feedback_gain,
            rate_error,
            feedback)
        || !SafeAdd(reference_m2ps3, proportional, rate_command)
        || !SafeAdd(rate_command, integral, rate_command)
        || !SafeAdd(rate_command, feedback, rate_command)
        || !SafeMultiply(receipt.mass_kg, rate_command, value)
        || !SafeDivide(value, receipt.speed_for_inverse_mps, value)
        || !SafeAdd(receipt.drag_estimate_n, value, value)
        || !SafeDivide(
            value,
            receipt.thrust_velocity_projection,
            thrust_n))
    {
        return false;
    }
    return true;
}

bool DeriveAuthority(
    LadyLuck::runtime::CurrentCisV4EnergyProjectionReceipt& receipt) noexcept
{
    const double values[] = {
        receipt.energy_error_gain_per_s,
        receipt.energy_integral_gain_per_s2,
        receipt.energy_rate_feedback_gain,
        receipt.total_energy_error_m2ps2,
        receipt.energy_integral_before_m2ps,
        receipt.specific_energy_rate_measured_m2ps3,
        receipt.speed_for_inverse_mps,
        receipt.mass_kg,
        receipt.drag_estimate_n,
        receipt.thrust_velocity_projection,
        receipt.thrust_min_n,
        receipt.thrust_max_n};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    if (receipt.energy_error_gain_per_s < 0.0
        || receipt.energy_integral_gain_per_s2 < 0.0
        || receipt.energy_rate_feedback_gain < 0.0
        || receipt.speed_for_inverse_mps <= 0.0
        || receipt.mass_kg <= 0.0
        || receipt.drag_estimate_n < 0.0
        || receipt.thrust_velocity_projection <= 0.0
        || receipt.thrust_velocity_projection > 1.0
        || receipt.thrust_min_n > receipt.thrust_max_n)
    {
        return false;
    }

    double denominator = 0.0;
    double kp_error = 0.0;
    double ki_integral = 0.0;
    double kr_measured = 0.0;
    double offset = 0.0;
    double inner = 0.0;
    double rate_min = 0.0;
    double rate_max = 0.0;
    double numerator = 0.0;
    if (!SafeAdd(1.0, receipt.energy_rate_feedback_gain, denominator)
        || !SafeMultiply(
            receipt.energy_error_gain_per_s,
            receipt.total_energy_error_m2ps2,
            kp_error)
        || !SafeMultiply(
            receipt.energy_integral_gain_per_s2,
            receipt.energy_integral_before_m2ps,
            ki_integral)
        || !SafeMultiply(
            receipt.energy_rate_feedback_gain,
            receipt.specific_energy_rate_measured_m2ps3,
            kr_measured)
        || !SafeAdd(kp_error, ki_integral, offset)
        || !SafeSubtract(offset, kr_measured, offset)
        || !SafeMultiply(
            receipt.thrust_velocity_projection,
            receipt.thrust_min_n,
            inner)
        || !SafeSubtract(inner, receipt.drag_estimate_n, inner)
        || !SafeMultiply(receipt.speed_for_inverse_mps, inner, rate_min)
        || !SafeDivide(rate_min, receipt.mass_kg, rate_min)
        || !SafeMultiply(
            receipt.thrust_velocity_projection,
            receipt.thrust_max_n,
            inner)
        || !SafeSubtract(inner, receipt.drag_estimate_n, inner)
        || !SafeMultiply(receipt.speed_for_inverse_mps, inner, rate_max)
        || !SafeDivide(rate_max, receipt.mass_kg, rate_max)
        || !SafeSubtract(rate_min, offset, numerator)
        || !SafeDivide(
            numerator,
            denominator,
            receipt.reference_min_m2ps3)
        || !SafeSubtract(rate_max, offset, numerator)
        || !SafeDivide(
            numerator,
            denominator,
            receipt.reference_max_m2ps3)
        || receipt.reference_min_m2ps3
            > receipt.reference_max_m2ps3)
    {
        return false;
    }
    receipt.authority_available = true;
    return true;
}
}

namespace LadyLuck
{
namespace runtime
{

const char* CurrentCisV4EnergyProjectionReasonLabel(
    const CurrentCisV4EnergyProjectionReason reason) noexcept
{
    switch (reason)
    {
    case CurrentCisV4EnergyProjectionReason::NotEvaluated:
        return "not_evaluated";
    case CurrentCisV4EnergyProjectionReason::WithinCurrentAuthority:
        return "within_current_authority";
    case CurrentCisV4EnergyProjectionReason::LowerReferenceProjected:
        return "lower_reference_projected";
    case CurrentCisV4EnergyProjectionReason::UpperReferenceProjected:
        return "upper_reference_projected";
    case CurrentCisV4EnergyProjectionReason::InvalidRawIntent:
        return "invalid_raw_intent";
    case CurrentCisV4EnergyProjectionReason::FrameIdentityMismatch:
        return "frame_identity_mismatch";
    case CurrentCisV4EnergyProjectionReason::UnsupportedRoute:
        return "unsupported_route";
    case CurrentCisV4EnergyProjectionReason::DirectEnergyBiasPresent:
        return "direct_energy_bias_present";
    case CurrentCisV4EnergyProjectionReason::RawRoutePreviewUnavailable:
        return "raw_route_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::ZeroBiasRoutePreviewUnavailable:
        return "zero_bias_route_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::RawTecsPreviewUnavailable:
        return "raw_tecs_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::ZeroBiasTecsPreviewUnavailable:
        return "zero_bias_tecs_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::RawControlUnbounded:
        return "raw_control_unbounded";
    case CurrentCisV4EnergyProjectionReason::MeasuredEnergyRateUnavailable:
        return "measured_energy_rate_unavailable";
    case CurrentCisV4EnergyProjectionReason::AuthorityInputsUnavailable:
        return "authority_inputs_unavailable";
    case CurrentCisV4EnergyProjectionReason::AuthorityArithmeticUnavailable:
        return "authority_arithmetic_unavailable";
    case CurrentCisV4EnergyProjectionReason::SaturatedReferenceNotLimited:
        return "saturated_reference_not_limited";
    case CurrentCisV4EnergyProjectionReason::BoundaryNotForwardRepresentable:
        return "boundary_not_forward_representable";
    case CurrentCisV4EnergyProjectionReason::InverseForwardClosureExceeded:
        return "inverse_forward_closure_exceeded";
    case CurrentCisV4EnergyProjectionReason::ProjectionRequiresPositiveBias:
        return "projection_requires_positive_bias";
    case CurrentCisV4EnergyProjectionReason::ProjectedRoutePreviewUnavailable:
        return "projected_route_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::ProjectedTecsPreviewUnavailable:
        return "projected_tecs_preview_unavailable";
    case CurrentCisV4EnergyProjectionReason::ProjectedControlUnbounded:
        return "projected_control_unbounded";
    case CurrentCisV4EnergyProjectionReason::ProjectedCommandStillSaturated:
        return "projected_command_still_saturated";
    case CurrentCisV4EnergyProjectionReason::NonEnergyFieldMutation:
        return "nonenergy_field_mutation";
    case CurrentCisV4EnergyProjectionReason::LiveControllerStateMutation:
        return "live_controller_state_mutation";
    case CurrentCisV4EnergyProjectionReason::ReferenceEquationMismatch:
        return "reference_equation_mismatch";
    }
    return "unknown";
}

CurrentCisV4EnergyProjectionPort::CurrentCisV4EnergyProjectionPort(
    const control::route5::Route5Guidance& route5,
    const control::tecs_cis::TecsCisControl& tecs_cis,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const control::route5::CommandEnvelope& envelope,
    const double dt_s,
    const bool gcas_energy_handoff_active,
    const bool previous_transmitted_auto_gcas_active) noexcept
    : route5_(route5),
      tecs_cis_(tecs_cis),
      ownship_(ownship),
      estimate_(estimate),
      envelope_(envelope),
      dt_s_(dt_s),
      gcas_energy_handoff_active_(gcas_energy_handoff_active),
      previous_transmitted_auto_gcas_active_(
          previous_transmitted_auto_gcas_active)
{
}

void CurrentCisV4EnergyProjectionPort::Project(
    const ControlIntent& raw,
    ControlIntent& output,
    CurrentCisV4EnergyProjectionReceipt& receipt,
    Status& status) const noexcept
{
    // `raw` and `output` are permitted to name the same caller-owned object.
    // Freeze the input before the first output write so a late projected-path
    // rejection can restore the exact original command instead of restoring
    // an already modified in-place bias.
    const ControlIntent raw_snapshot = raw;
    output = raw_snapshot;
    receipt = CurrentCisV4EnergyProjectionReceipt{};
    status = Status{};
    receipt.frame_identity = raw_snapshot.frame_identity;
    receipt.evaluated = true;
    receipt.raw_bias_m2ps3 =
        raw_snapshot.specific_energy_rate_bias_m2ps3;
    receipt.admitted_bias_m2ps3 =
        raw_snapshot.specific_energy_rate_bias_m2ps3;

    do
    {
        Status validation{};
        raw_snapshot.Validate(validation);
        if (!validation.ok())
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::InvalidRawIntent;
            break;
        }
        if (!IsValidControlFrameIdentity(envelope_.frame_identity)
            || !SameControlFrameIdentity(
                raw_snapshot.frame_identity,
                envelope_.frame_identity)
            || !estimate_.measurement_reset_epoch.has_value
            || !estimate_.measurement_frame_index.has_value
            || !estimate_.accepted_sample_t_sec.has_value
            || !std::isfinite(estimate_.accepted_sample_t_sec.value)
            || estimate_.accepted_sample_t_sec.value < 0.0)
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::FrameIdentityMismatch;
            receipt.contract_fault = true;
            break;
        }
        ControlFrameIdentity estimate_identity{};
        estimate_identity.valid = true;
        estimate_identity.episode_epoch =
            estimate_.measurement_reset_epoch.value;
        estimate_identity.frame_index =
            estimate_.measurement_frame_index.value;
        estimate_identity.source_time_s =
            estimate_.accepted_sample_t_sec.value;
        if (!SameControlFrameIdentity(
                raw_snapshot.frame_identity,
                estimate_identity)
            || ownship_.frame_index
                != raw_snapshot.frame_identity.frame_index)
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::FrameIdentityMismatch;
            receipt.contract_fault = true;
            break;
        }
        if (raw_snapshot.route_kind != ControlRouteKind::AimPoint)
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::UnsupportedRoute;
            break;
        }
        if (raw_snapshot.direct_accel_cmd_mps2.has_value)
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::DirectEnergyBiasPresent;
            receipt.contract_fault = true;
            break;
        }

        Route5GuidanceOutput raw_route{};
        Route5GuidanceSnapshot raw_route_next{};
        Status raw_route_status{};
        route5_.Preview(
            raw_snapshot,
            ownship_,
            estimate_,
            envelope_,
            dt_s_,
            raw_route,
            raw_route_next,
            raw_route_status);
        if (!raw_route_status.ok() || !raw_route.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                RawRoutePreviewUnavailable;
            break;
        }

        bool raw_next_handoff = false;
        bool raw_integrator_hold = false;
        if (!EvaluateEnergyIntegratorHold(
                estimate_,
                raw_route.flight_path_angle_cmd_rad,
                gcas_energy_handoff_active_,
                previous_transmitted_auto_gcas_active_,
                raw_next_handoff,
                raw_integrator_hold))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                RawRoutePreviewUnavailable;
            break;
        }
        BodyRateLoadEnergyCommand raw_reference{};
        BuildBodyRateReference(
            raw_route,
            raw_integrator_hold,
            raw_reference);
        TecsCisOutput raw_tecs{};
        TecsCisSnapshot raw_tecs_next{};
        Status raw_tecs_status{};
        tecs_cis_.Preview(
            raw_reference,
            estimate_,
            envelope_,
            dt_s_,
            raw_tecs,
            raw_tecs_next,
            raw_tecs_status);
        if (!raw_tecs_status.ok() || !raw_tecs.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                RawTecsPreviewUnavailable;
            break;
        }
        receipt.raw_control_bounded =
            BoundedControllerCommand(raw_tecs.command);
        receipt.raw_lower_saturated =
            raw_tecs.diagnostics.lower_thrust_saturated;
        receipt.raw_upper_saturated =
            raw_tecs.diagnostics.upper_thrust_saturated;
        receipt.raw_reference_m2ps3 = raw_tecs.diagnostics.
            specific_energy_rate_reference_m2ps3;
        receipt.raw_thrust_cmd_n =
            raw_tecs.diagnostics.thrust_cmd_raw_n;
        receipt.raw_rate_measurement_valid =
            raw_tecs.diagnostics.rate_measurement_valid;
        receipt.raw_specific_energy_rate_measured_m2ps3 =
            raw_tecs.diagnostics.specific_energy_rate_measured_m2ps3;
        receipt.raw_thrust_cmd_limited_n =
            raw_tecs.diagnostics.thrust_cmd_limited_n;
        receipt.route_k_gamma_per_s = raw_route.k_gamma_per_s;
        Status gamma_rate_status{};
        route5_.CopyGammaRateLimit(
            receipt.route_gamma_rate_limit_radps,
            gamma_rate_status);
        if (!gamma_rate_status.ok())
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                RawRoutePreviewUnavailable;
            break;
        }
        receipt.thrust_min_n =
            raw_tecs.completed_energy_authority.thrust_min_n;
        receipt.thrust_max_n =
            raw_tecs.completed_energy_authority.thrust_max_n;
        if (!receipt.raw_control_bounded)
        {
            receipt.reason =
                CurrentCisV4EnergyProjectionReason::RawControlUnbounded;
            receipt.contract_fault = true;
            break;
        }

        // A zero-explicit-bias preview is the exact controller-owned base
        // reference.  It avoids duplicating Route-5 or TECS shaping bounds.
        ControlIntent zero_bias = raw_snapshot;
        zero_bias.specific_energy_rate_bias_m2ps3 = 0.0;
        Route5GuidanceOutput zero_route{};
        Route5GuidanceSnapshot zero_route_next{};
        Status zero_route_status{};
        route5_.Preview(
            zero_bias,
            ownship_,
            estimate_,
            envelope_,
            dt_s_,
            zero_route,
            zero_route_next,
            zero_route_status);
        if (!zero_route_status.ok() || !zero_route.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ZeroBiasRoutePreviewUnavailable;
            break;
        }
        bool zero_next_handoff = false;
        bool zero_integrator_hold = false;
        if (!EvaluateEnergyIntegratorHold(
                estimate_,
                zero_route.flight_path_angle_cmd_rad,
                gcas_energy_handoff_active_,
                previous_transmitted_auto_gcas_active_,
                zero_next_handoff,
                zero_integrator_hold))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ZeroBiasRoutePreviewUnavailable;
            break;
        }
        BodyRateLoadEnergyCommand zero_reference{};
        BuildBodyRateReference(
            zero_route,
            zero_integrator_hold,
            zero_reference);
        TecsCisOutput zero_tecs{};
        TecsCisSnapshot zero_tecs_next{};
        Status zero_tecs_status{};
        tecs_cis_.Preview(
            zero_reference,
            estimate_,
            envelope_,
            dt_s_,
            zero_tecs,
            zero_tecs_next,
            zero_tecs_status);
        if (!zero_tecs_status.ok() || !zero_tecs.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ZeroBiasTecsPreviewUnavailable;
            break;
        }
        receipt.base_reference_m2ps3 = zero_tecs.diagnostics.
            specific_energy_rate_reference_m2ps3;

        if (!receipt.raw_lower_saturated
            && !receipt.raw_upper_saturated)
        {
            receipt.admitted = true;
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                WithinCurrentAuthority;
            receipt.all_nonenergy_fields_unchanged = true;
            receipt.projected_control_bounded =
                receipt.raw_control_bounded;
            receipt.projected_thrust_cmd_n = receipt.raw_thrust_cmd_n;
            receipt.projected_thrust_cmd_limited_n =
                receipt.raw_thrust_cmd_limited_n;
            receipt.admitted_reference_m2ps3 =
                receipt.raw_reference_m2ps3;
            receipt.projected_reference_m2ps3 =
                receipt.raw_reference_m2ps3;
            break;
        }
        if (!raw_tecs.diagnostics.rate_measurement_valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                MeasuredEnergyRateUnavailable;
            break;
        }
        if (!raw_tecs.completed_energy_authority.valid
            || !raw_tecs.completed_energy_authority.
                controller_configuration_available)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                AuthorityInputsUnavailable;
            receipt.contract_fault = true;
            break;
        }

        receipt.energy_error_gain_per_s = raw_tecs.
            completed_energy_authority.energy_error_gain_per_s;
        receipt.energy_integral_gain_per_s2 = raw_tecs.
            completed_energy_authority.energy_integral_gain_per_s2;
        receipt.energy_rate_feedback_gain = raw_tecs.
            completed_energy_authority.energy_rate_feedback_gain;
        receipt.total_energy_error_m2ps2 = raw_tecs.
            completed_energy_authority.total_energy_error_m2ps2;
        receipt.energy_integral_before_m2ps =
            raw_tecs.diagnostics.energy_integral_before_m2ps;
        receipt.specific_energy_rate_measured_m2ps3 = raw_tecs.
            completed_energy_authority.specific_energy_rate_measured_m2ps3;
        receipt.speed_for_inverse_mps = (std::max)(
            raw_tecs.completed_energy_authority.speed_mps,
            raw_tecs.completed_energy_authority.minimum_speed_mps);
        receipt.mass_kg =
            raw_tecs.completed_energy_authority.mass_kg;
        receipt.drag_estimate_n =
            raw_tecs.completed_energy_authority.drag_estimate_n;
        receipt.thrust_velocity_projection = raw_tecs.
            completed_energy_authority.thrust_velocity_projection;
        if (!DeriveAuthority(receipt))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                AuthorityArithmeticUnavailable;
            receipt.contract_fault = true;
            break;
        }

        const bool lower_limited = receipt.raw_reference_m2ps3
            < receipt.reference_min_m2ps3;
        const bool upper_limited = receipt.raw_reference_m2ps3
            > receipt.reference_max_m2ps3;
        if (lower_limited == upper_limited)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                SaturatedReferenceNotLimited;
            receipt.contract_fault = true;
            break;
        }
        double feasible_reference = lower_limited
            ? receipt.reference_min_m2ps3
            : receipt.reference_max_m2ps3;
        double forward_thrust = 0.0;
        if (!ForwardThrustForReference(
                receipt,
                feasible_reference,
                forward_thrust))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                AuthorityArithmeticUnavailable;
            receipt.contract_fault = true;
            break;
        }

        // Exact authority guardrail from add/main@45abc:
        // energy_reference_allocation._MAX_FORWARD_FEASIBILITY_ULP_STEPS.
        constexpr std::uint32_t MaxForwardClosureUlpSteps = 64U;
        bool closure_exhausted = false;
        bool closure_arithmetic_unavailable = false;
        bool boundary_exhausted = false;
        while ((lower_limited && forward_thrust < receipt.thrust_min_n)
            || (upper_limited && forward_thrust > receipt.thrust_max_n))
        {
            if (receipt.closure_ulp_steps >= MaxForwardClosureUlpSteps)
            {
                closure_exhausted = true;
                break;
            }
            // A finite endpoint produces the same one-ULP inward step as an
            // infinite direction without introducing an Inf value into the
            // 60 Hz arithmetic path.
            const double direction = lower_limited
                ? (std::numeric_limits<double>::max)()
                : -(std::numeric_limits<double>::max)();
            const double candidate = std::nextafter(
                feasible_reference,
                direction);
            if (candidate == feasible_reference
                || (lower_limited
                    && candidate > receipt.reference_max_m2ps3)
                || (upper_limited
                    && candidate < receipt.reference_min_m2ps3))
            {
                boundary_exhausted = true;
                break;
            }
            feasible_reference = candidate;
            ++receipt.closure_ulp_steps;
            if (!ForwardThrustForReference(
                    receipt,
                    feasible_reference,
                    forward_thrust))
            {
                closure_arithmetic_unavailable = true;
                break;
            }
        }
        if (closure_arithmetic_unavailable)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                AuthorityArithmeticUnavailable;
            receipt.contract_fault = true;
            break;
        }
        if (closure_exhausted)
        {
            // The inverse boundary and the forward TECS equation are both
            // finite, but their representable closure can require more than
            // the authority-owned 64-ULP work bound.  This is an ordinary
            // projection non-admission: retain the already validated raw
            // command so the caller can publish its same-frame base fallback.
            output = raw_snapshot;
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                InverseForwardClosureExceeded;
            receipt.admitted = false;
            receipt.contract_fault = false;
            break;
        }
        if (boundary_exhausted)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                BoundaryNotForwardRepresentable;
            break;
        }
        double admitted_bias = 0.0;
        if (!SafeSubtract(
                feasible_reference,
                receipt.base_reference_m2ps3,
                admitted_bias))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                AuthorityArithmeticUnavailable;
            receipt.contract_fault = true;
            break;
        }
        receipt.admitted_reference_m2ps3 = feasible_reference;
        receipt.admitted_bias_m2ps3 = admitted_bias;
        if (admitted_bias > 0.0)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectionRequiresPositiveBias;
            break;
        }

        output = raw_snapshot;
        output.specific_energy_rate_bias_m2ps3 = admitted_bias;
        // Construction is the authority: output is a raw copy with only the
        // explicitly owned energy-bias field changed.  Re-reading every byte
        // of the same object cannot add independent flight evidence.
        receipt.all_nonenergy_fields_unchanged = true;

        Route5GuidanceOutput projected_route{};
        Route5GuidanceSnapshot projected_route_next{};
        Status projected_route_status{};
        route5_.Preview(
            output,
            ownship_,
            estimate_,
            envelope_,
            dt_s_,
            projected_route,
            projected_route_next,
            projected_route_status);
        if (!projected_route_status.ok() || !projected_route.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectedRoutePreviewUnavailable;
            output = raw_snapshot;
            break;
        }
        bool projected_next_handoff = false;
        bool projected_integrator_hold = false;
        if (!EvaluateEnergyIntegratorHold(
                estimate_,
                projected_route.flight_path_angle_cmd_rad,
                gcas_energy_handoff_active_,
                previous_transmitted_auto_gcas_active_,
                projected_next_handoff,
                projected_integrator_hold))
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectedRoutePreviewUnavailable;
            output = raw_snapshot;
            break;
        }
        BodyRateLoadEnergyCommand projected_reference{};
        BuildBodyRateReference(
            projected_route,
            projected_integrator_hold,
            projected_reference);
        TecsCisOutput projected_tecs{};
        TecsCisSnapshot projected_tecs_next{};
        Status projected_tecs_status{};
        tecs_cis_.Preview(
            projected_reference,
            estimate_,
            envelope_,
            dt_s_,
            projected_tecs,
            projected_tecs_next,
            projected_tecs_status);
        if (!projected_tecs_status.ok() || !projected_tecs.valid)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectedTecsPreviewUnavailable;
            output = raw_snapshot;
            break;
        }
        receipt.projected_control_bounded =
            BoundedControllerCommand(projected_tecs.command);
        receipt.projected_lower_saturated =
            projected_tecs.diagnostics.lower_thrust_saturated;
        receipt.projected_upper_saturated =
            projected_tecs.diagnostics.upper_thrust_saturated;
        receipt.projected_thrust_cmd_n =
            projected_tecs.diagnostics.thrust_cmd_raw_n;
        receipt.projected_thrust_cmd_limited_n =
            projected_tecs.diagnostics.thrust_cmd_limited_n;
        receipt.projected_reference_m2ps3 = projected_tecs.diagnostics.
            specific_energy_rate_reference_m2ps3;
        if (!receipt.projected_control_bounded)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectedControlUnbounded;
            receipt.contract_fault = true;
            output = raw_snapshot;
            break;
        }
        if (receipt.projected_lower_saturated
            || receipt.projected_upper_saturated)
        {
            receipt.reason = CurrentCisV4EnergyProjectionReason::
                ProjectedCommandStillSaturated;
            // Projection is optional.  A finite, bounded preview that remains
            // saturated is ordinary non-admission, not corruption of the
            // already validated raw command.
            receipt.contract_fault = false;
            output = raw_snapshot;
            break;
        }
        receipt.admitted = true;
        receipt.reason = lower_limited
            ? CurrentCisV4EnergyProjectionReason::LowerReferenceProjected
            : CurrentCisV4EnergyProjectionReason::UpperReferenceProjected;
    }
    while (false);

}

TacticalControlCore::TacticalControlCore(
    ITacticalCommandProvider& command_provider) noexcept
    : command_provider_(command_provider)
{
    Status reset_status{};
    Reset(reset_status);
    static_cast<void>(reset_status);
}

void TacticalControlCore::Reset(Status& status) noexcept
{
    status = Status{};
    const EstimatorTransactionReceipt estimator_reset = estimator_.Reset();
    direct_ned_.Reset();
    direct_force_tracking_.Reset();
    route5_.Reset();
    tecs_cis_.Reset();
    auto_gcas_.Reset();
    command_provider_.Reset();

    const std::uint64_t next_generation = state_.generation + 1U;
    state_ = TacticalControlCoreSnapshot{};
    state_.generation = next_generation;
    state_.previous_transmitted = ControlValue{0.0F, 0.0F, 0.0F, 0.65F};
    RefreshSnapshots();

    bool tecs_cis_valid = false;
    bool auto_gcas_valid = false;
    bool route5_valid = false;
    route5_.CopyConfigurationValid(route5_valid);
    tecs_cis_.CopyConfigurationValid(tecs_cis_valid);
    auto_gcas_.CopyConfigurationValid(auto_gcas_valid);
    if (!estimator_reset.ok()
        || !estimator_.configuration_valid()
        || !route5_valid
        || !tecs_cis_valid
        || !auto_gcas_valid)
    {
        status = Failure(StatusCode::InvalidConfiguration);
    }
}

void TacticalControlCore::PrepareOwner(
    const std::int32_t owner_plane_id,
    const std::int32_t owner_force_side,
    Status& status) noexcept
{
    status = Status{};
    if (owner_plane_id < 0 || owner_force_side == 0)
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }
    if (state_.owner_prepared
        && (state_.owner_plane_id != owner_plane_id
            || state_.owner_force_side != owner_force_side))
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }
    state_.owner_prepared = true;
    state_.owner_plane_id = owner_plane_id;
    state_.owner_force_side = owner_force_side;
}

void TacticalControlCore::ValidateInput(
    const KinematicObservationInputV1& input,
    Status& status) const noexcept
{
    status = Status{};
    if (!state_.owner_prepared
        || input.abi_version != AIPILOT_ABI_VERSION_V1
        || input.struct_size != sizeof(KinematicObservationInputV1)
        || input.ownship.abi_version != AIPILOT_ABI_VERSION_V1
        || input.ownship.struct_size != sizeof(PlaneKinematicObservationV1)
        || input.target.abi_version != AIPILOT_ABI_VERSION_V1
        || input.target.struct_size != sizeof(PlaneKinematicObservationV1)
        || !std::isfinite(input.nominal_dt_s)
        || input.nominal_dt_s <= 0.0
        || !std::isfinite(input.command_time_s)
        || input.command_time_s < 0.0
        || input.context_own_plane_id != input.ownship.plane_id
        || input.context_target_plane_id != input.target.plane_id
        || input.ownship.plane_id != state_.owner_plane_id
        || input.ownship.force_side != state_.owner_force_side
        || input.target.plane_id < 0
        || input.target.plane_id == input.ownship.plane_id
        || input.target.force_side == 0
        || input.target.force_side == input.ownship.force_side)
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }
    if (!PlaneWireFinite(input.ownship)
        || !PlaneWireFinite(input.target))
    {
        status = Failure(StatusCode::NonFiniteInput);
    }
}

void TacticalControlCore::BuildFrameContextRequest(
    const KinematicObservationInputV1& input,
    FrameContextRequest& output) const noexcept
{
    output = FrameContextRequest{};
    output.measurement_frame_index = input.ownship.frame_index;
    output.command_frame_index.has_value = true;
    output.command_frame_index.value = input.command_frame_index;
    if (state_.has_previous_transmitted)
    {
        output.previous_command_frame.has_value = true;
        output.previous_command_frame.value = state_.previous_command_frame;
    }
}

void TacticalControlCore::Preflight(
    const KinematicObservationInputV1& input,
    ControlCorePreflightReceipt& output) const noexcept
{
    output = ControlCorePreflightReceipt{};
    ValidateInput(input, output.status);
    if (!output.status.ok())
    {
        output.fault_class = ControlCoreFaultClass::InputRejected;
        return;
    }
    if (AIPilotPreStartCommandNeutralV1(input))
    {
        // A finite placeholder state pair carries no usable flight direction
        // or line of sight. Consume it as an explicit command-neutral
        // pre-start state before estimator mutation and before a BehaviorTree
        // tick; it is neither a rejected live frame nor a terrain hazard.
        output.status = Status{};
        output.prestart_command_neutral = true;
        output.fault_class = ControlCoreFaultClass::None;
        output.estimator_fault_class = EstimatorFaultClass::None;
        return;
    }

    FrameContextRequest request{};
    BuildFrameContextRequest(input, request);
    FrameContextBuildResult context{};
    estimator_.MakeFrameContext(request, context);
    output.estimator_code = context.code;
    output.estimator_cause = context.cause;
    output.estimator_fault_class = context.fault_class;
    if (!context.ok())
    {
        output.fault_class = context.fault_class
                == EstimatorFaultClass::ExternalFrameRejected
            ? ControlCoreFaultClass::EstimatorFrameRejected
            : ControlCoreFaultClass::InternalFault;
        output.status = Failure(
            output.fault_class == ControlCoreFaultClass::EstimatorFrameRejected
                ? StatusCode::InvalidArgument
                : StatusCode::InvalidConfiguration);
        return;
    }

    output.status = Status{};
    output.behavior_tree_tick_allowed = true;
    output.fault_class = ControlCoreFaultClass::None;
    output.frame_context_ready = true;
    output.frame_context = context.value;
}

void TacticalControlCore::Step(
    const KinematicObservationInputV1& input,
    ControlCoreReceipt& output) noexcept
{
    ControlCorePreflightReceipt preflight{};
    Preflight(input, preflight);
    StepPrepared(input, preflight, output);
}

void TacticalControlCore::StepPrepared(
    const KinematicObservationInputV1& input,
    const ControlCorePreflightReceipt& preflight,
    ControlCoreReceipt& output) noexcept
{
    output = ControlCoreReceipt{};
    output.stage = ControlCoreStage::Preflight;
    ++state_.step_call_count;
    if (preflight.prestart_command_neutral)
    {
        output.status = Status{};
        output.prestart_command_neutral = true;
        output.fault_class = ControlCoreFaultClass::None;
        output.estimator_fault_class = EstimatorFaultClass::None;
        RecordCommandOutcome(output);
        RefreshSnapshots();
        return;
    }
    if (!preflight.behavior_tree_tick_allowed)
    {
        output.status = preflight.status;
        output.fault_class = preflight.fault_class;
        output.estimator_code = preflight.estimator_code;
        output.estimator_cause = preflight.estimator_cause;
        output.estimator_fault_class = preflight.estimator_fault_class;
        output.origin_failure_stage = output.stage;
        output.origin_status_code = output.status.code;
        if (output.fault_class == ControlCoreFaultClass::InputRejected)
        {
            ++state_.input_rejected_count;
        }
        else if (output.fault_class == ControlCoreFaultClass::InternalFault)
        {
            ++state_.internal_fault_count;
        }
        RecordCommandOutcome(output);
        RefreshSnapshots();
        return;
    }

    if (!preflight.frame_context_ready)
    {
        output.status = Failure(StatusCode::InvalidConfiguration);
        output.fault_class = ControlCoreFaultClass::InternalFault;
        output.origin_failure_stage = output.stage;
        output.origin_status_code = output.status.code;
        ++state_.internal_fault_count;
        RecordCommandOutcome(output);
        RefreshSnapshots();
        return;
    }

    StepPrimary(input, preflight.frame_context, output);
    if (!output.control_authorized)
    {
        // Previous actuator output remains estimator feedback only. It is not
        // a current-frame guidance command and is never relabelled Primary.
        output.origin_failure_stage = output.stage;
        output.origin_status_code = output.status.code;
    }
    RecordCommandOutcome(output);
    RefreshSnapshots();
}

void TacticalControlCore::RecordCommandOutcome(
    ControlCoreReceipt& output) noexcept
{
    output.current_base_event = CurrentBaseOwnershipEvent::None;
    const bool current_base = output.control_authorized
        && output.outcome == ControlCommandOutcome::CurrentBase;
    if (current_base && !state_.current_base_active)
    {
        output.current_base_event = CurrentBaseOwnershipEvent::Started;
        ++state_.current_base_started_count;
    }
    else if (!current_base && state_.current_base_active)
    {
        output.current_base_event = CurrentBaseOwnershipEvent::Ended;
        ++state_.current_base_ended_count;
    }
    state_.current_base_active = current_base;
    state_.last_command_outcome = output.outcome;
    state_.last_current_base_event = output.current_base_event;
}

TacticalControlPipelineState
TacticalControlCore::CapturePipelineState() const noexcept
{
    TacticalControlPipelineState snapshot{};
    snapshot.route5 = route5_;
    snapshot.direct_ned = direct_ned_;
    snapshot.direct_force_tracking = direct_force_tracking_;
    snapshot.tecs_cis = tecs_cis_;
    snapshot.auto_gcas = auto_gcas_;
    snapshot.gcas_energy_handoff_active =
        state_.gcas_energy_handoff_active;
    return snapshot;
}

void TacticalControlCore::RestorePipelineState(
    const TacticalControlPipelineState& snapshot) noexcept
{
    route5_ = snapshot.route5;
    direct_ned_ = snapshot.direct_ned;
    direct_force_tracking_ = snapshot.direct_force_tracking;
    tecs_cis_ = snapshot.tecs_cis;
    auto_gcas_ = snapshot.auto_gcas;
    state_.gcas_energy_handoff_active =
        snapshot.gcas_energy_handoff_active;
}

void TacticalControlCore::ExecuteIntent(
    const ControlIntent& intent,
    const Result<DogfightGeometryFrame>& frame,
    const EstimatorUpdateResult& estimate,
    const control::route5::CommandEnvelope& envelope,
    const FrameContext& context,
    const safety::AutoGcasEntryReceipt& gcas_entry,
    TacticalControlAttempt& attempt) noexcept
{
    attempt = TacticalControlAttempt{};
    if (intent.writer_id == ControlIntentWriterAutoGcasRecovery)
    {
        if (intent.route_kind != ControlRouteKind::SafetyRecovery)
        {
            attempt.status = Failure(StatusCode::InvalidConfiguration);
            attempt.failure_stage = ControlCoreStage::Guidance;
            return;
        }

        safety::AutoGcasReceipt recovery{};
        Status recovery_status{};
        auto_gcas_.BuildRecoveryCommand(
            gcas_entry,
            recovery,
            recovery_status);
        if (!recovery_status.ok() || !recovery.post_command.valid)
        {
            attempt.status = recovery_status.ok()
                ? Failure(StatusCode::InvalidConfiguration)
                : recovery_status;
            attempt.failure_stage = ControlCoreStage::AutoGcasApply;
            return;
        }

        attempt.wire_control = ToWire(recovery.post_command);
        if (!FiniteWire(attempt.wire_control))
        {
            attempt.status = Failure(StatusCode::NonFiniteInput);
            attempt.failure_stage = ControlCoreStage::WireValidation;
            return;
        }

        TacticalAge1ControlFeedback feedback{};
        feedback.source_kind =
            TacticalFeedbackSourceKind::AcceptedEstimatorFrame;
        feedback.source_frame_index_valid = true;
        feedback.source_frame_index = frame.value.frame_identity.frame_index;
        feedback.source_frame_identity = frame.value.frame_identity;
        feedback.command_backend_id =
            TacticalControlBackendId::AutoGcasRecovery;
        feedback.writer_id = intent.writer_id;
        feedback.behavior_id = intent.behavior_id;
        feedback.mode_id = intent.mode_id;
        feedback.nz_measured_valid = estimate.output.nz_valid;
        feedback.nz_measured_g = estimate.output.nz;
        feedback.nz_feasible_valid =
            control::route5::IsPhysicalNzCommandEnvelopeSource(
                envelope.source);
        feedback.nz_feasible_g = envelope.nz_feasible_g;

        const NormalizedControlCommand transmitted{
            static_cast<double>(attempt.wire_control.RollCMD),
            static_cast<double>(attempt.wire_control.PitchCMD),
            static_cast<double>(attempt.wire_control.RudderCMD),
            static_cast<double>(attempt.wire_control.Throttle),
            true};
        Status feedback_status{};
        tactical_input_builder_.CompleteFeedback(
            recovery,
            context.source_t_sec,
            transmitted,
            feedback,
            feedback_status);
        if (!feedback_status.ok())
        {
            attempt.status = feedback_status;
            attempt.failure_stage = ControlCoreStage::FeedbackComplete;
            return;
        }
        attempt.next_feedback = feedback;
        attempt.valid = true;
        attempt.status = Status{};
        attempt.auto_gcas_override = recovery.override_active;
        attempt.auto_gcas_phase = recovery.phase;
        return;
    }
    if (intent.route_kind == ControlRouteKind::SafetyRecovery)
    {
        attempt.status = Failure(StatusCode::InvalidConfiguration);
        attempt.failure_stage = ControlCoreStage::Guidance;
        return;
    }

    control::tecs_cis::BodyRateLoadEnergyCommand control_reference{};
    TacticalCompletedTotalLoadReceipt completed_total_load{};
    Status stage_status{};
    BuildControlReference(
        intent,
        frame.value,
        estimate.plane_state,
        estimate.output,
        envelope,
        context.sample_dt_sec,
        control_reference,
        completed_total_load,
        stage_status);
    if (!stage_status.ok() || !control_reference.valid)
    {
        attempt.status = stage_status.ok()
            ? Failure(StatusCode::InvalidConfiguration)
            : stage_status;
        attempt.failure_stage = ControlCoreStage::Guidance;
        return;
    }

    control::tecs_cis::TecsCisOutput control{};
    stage_status = Status{};
    tecs_cis_.Step(
        control_reference,
        estimate.output,
        envelope,
        context.sample_dt_sec,
        control,
        stage_status);
    if (!stage_status.ok() || !control.valid || !control.command.valid)
    {
        attempt.status = stage_status.ok()
            ? Failure(StatusCode::InvalidConfiguration)
            : stage_status;
        attempt.failure_stage = ControlCoreStage::TecsCis;
        return;
    }

    Status feedback_status{};
    tactical_input_builder_.PrepareFeedback(
        frame.value.frame_identity,
        intent,
        estimate.output,
        envelope,
        control_reference,
        control,
        completed_total_load,
        attempt.next_feedback,
        feedback_status);
    if (!feedback_status.ok())
    {
        // Age-1 evidence is optional telemetry.  It must not revoke the
        // current finite FCS command.
        attempt.next_feedback = TacticalAge1ControlFeedback{};
    }

    auto_gcas_.Reset();
    safety::AutoGcasReceipt gcas{};
    gcas.post_command = SaturateFiniteControl(control.command);
    gcas.post_command.valid = true;
    gcas.phase = safety::AutoGcasPhase::Inactive;
    gcas.frame_identity = control.frame_identity;

    attempt.wire_control = ToWire(gcas.post_command);
    if (!FiniteWire(attempt.wire_control))
    {
        attempt.status = Failure(StatusCode::NonFiniteInput);
        attempt.failure_stage = ControlCoreStage::WireValidation;
        return;
    }

    const NormalizedControlCommand transmitted_wire_command{
        static_cast<double>(attempt.wire_control.RollCMD),
        static_cast<double>(attempt.wire_control.PitchCMD),
        static_cast<double>(attempt.wire_control.RudderCMD),
        static_cast<double>(attempt.wire_control.Throttle),
        true};
    feedback_status = Status{};
    tactical_input_builder_.CompleteFeedback(
        gcas,
        context.source_t_sec,
        transmitted_wire_command,
        attempt.next_feedback,
        feedback_status);
    if (!feedback_status.ok())
    {
        attempt.next_feedback = TacticalAge1ControlFeedback{};
    }
    attempt.valid = true;
    attempt.status = Status{};
    attempt.auto_gcas_override = gcas.override_active;
    attempt.auto_gcas_phase = gcas.phase;
}

void TacticalControlCore::StepPrimary(
    const KinematicObservationInputV1& input,
    const FrameContext& context,
    ControlCoreReceipt& output) noexcept
{
    output = ControlCoreReceipt{};
    output.stage = ControlCoreStage::FrameContext;

    output.stage = ControlCoreStage::EstimatorUpdate;
    CommandFeedback feedback{};
    if (state_.has_previous_transmitted)
    {
        feedback.kind = ActionFeedbackKind::PreviousTransmittedAssumption;
        feedback.has_transmitted_wire_payload = true;
        feedback.transmitted_wire_payload = {{
            state_.previous_transmitted.RollCMD,
            state_.previous_transmitted.PitchCMD,
            state_.previous_transmitted.RudderCMD,
            state_.previous_transmitted.Throttle}};
        feedback.estimator_command_u_dll = {{
            static_cast<double>(state_.previous_transmitted.RollCMD),
            static_cast<double>(state_.previous_transmitted.PitchCMD),
            static_cast<double>(state_.previous_transmitted.RudderCMD),
            static_cast<double>(state_.previous_transmitted.Throttle)}};
        feedback.source_frame_index.has_value = true;
        feedback.source_frame_index.value = state_.previous_command_frame;
        feedback.source_t_sec.has_value = true;
        feedback.source_t_sec.value = state_.previous_command_time_s;
    }

    const EstimatorUpdateResult estimate = estimator_.Update(
        ConvertInput(input.ownship),
        feedback,
        context);
    output.estimator_code = estimate.transaction.code;
    output.estimator_cause = estimate.transaction.cause;
    output.estimator_fault_class = estimate.transaction.fault_class;
    output.estimator_transaction_committed =
        estimate.transaction.state_committed;
    if (!estimate.ok())
    {
        output.fault_class = estimate.transaction.fault_class
                == EstimatorFaultClass::ExternalFrameRejected
            ? ControlCoreFaultClass::EstimatorFrameRejected
            : ControlCoreFaultClass::InternalFault;
        output.status = Failure(
            output.fault_class
                    == ControlCoreFaultClass::EstimatorFrameRejected
                ? StatusCode::InvalidArgument
                : StatusCode::InvalidConfiguration);
        return;
    }
    output.stage = ControlCoreStage::EstimatorOutputValidation;
    if (!estimate.output.measurement_reset_epoch.has_value
        || !estimate.output.measurement_frame_index.has_value
        || !estimate.output.accepted_sample_t_sec.has_value
        || !std::isfinite(estimate.output.accepted_sample_t_sec.value)
        || estimate.output.accepted_sample_t_sec.value < 0.0)
    {
        output.status = Failure(StatusCode::InvalidConfiguration);
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    output.stage = ControlCoreStage::TargetConversion;
    const Result<PlaneState> target = ConvertKinematicObservation(
        ConvertInput(input.target));
    if (!target.ok())
    {
        output.status = target.status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    output.stage = ControlCoreStage::Geometry;
    DogfightGeometryInput geometry_input{};
    geometry_input.frame_identity.valid = true;
    geometry_input.frame_identity.episode_epoch =
        estimate.output.measurement_reset_epoch.value;
    geometry_input.frame_identity.frame_index =
        estimate.output.measurement_frame_index.value;
    geometry_input.frame_identity.source_time_s =
        estimate.output.accepted_sample_t_sec.value;
    geometry_input.own = estimate.plane_state;
    geometry_input.opponent = target.value;
    geometry_input.t_sec = context.source_t_sec;
    geometry_input.tau_sec = input.nominal_dt_s;
    const Result<DogfightGeometryFrame> frame =
        BuildDogfightGeometryFrame(geometry_input);
    if (!frame.ok())
    {
        output.status = frame.status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    output.stage = ControlCoreStage::Envelope;
    control::route5::CommandEnvelope envelope{};
    Status envelope_status{};
    governor_.EnvelopeFrom(estimate.output, envelope, envelope_status);
    if (!envelope_status.ok() || !envelope.valid)
    {
        output.status = envelope_status.ok()
            ? Failure(StatusCode::InvalidConfiguration)
            : envelope_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    output.stage = ControlCoreStage::AutoGcasEntryInput;
    safety::AutoGcasEntryInput gcas_entry_input{};
    BuildAutoGcasEntryInput(
        context,
        frame.value,
        estimate,
        envelope,
        gcas_entry_input);
    output.stage = ControlCoreStage::AutoGcasEntryEvaluation;
    safety::AutoGcasEntryReceipt gcas_entry{};
    Status gcas_entry_status{};
    auto_gcas_.EvaluateEntry(
        gcas_entry_input,
        gcas_entry,
        gcas_entry_status);
    if (!gcas_entry_status.ok())
    {
        // A finite, status-OK unavailable receipt is the ordinary optional
        // nonadmission. Non-OK here means malformed/nonfinite predictor input
        // or an internal contract failure and must not be disguised as
        // "terrain clear".
        output.status = gcas_entry_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    output.stage = ControlCoreStage::LongitudinalConfiguration;
    TacticalCurrentLongitudinalAuthorityEvidence
        current_longitudinal_evidence{};
    Status longitudinal_source_status{};
    tecs_cis_.CopyLongitudinalAuthorityConfiguration(
        current_longitudinal_evidence.tecs_configuration,
        longitudinal_source_status);
    if (!longitudinal_source_status.ok())
    {
        output.status = longitudinal_source_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }
    output.stage = ControlCoreStage::GammaLimit;
    route5_.CopyGammaCommandLimit(
        current_longitudinal_evidence.flight_path_gamma_limit_rad,
        longitudinal_source_status);
    if (!longitudinal_source_status.ok())
    {
        output.status = longitudinal_source_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }
    current_longitudinal_evidence.flight_path_gamma_limit_valid = true;
    output.stage = ControlCoreStage::NzfeasAuthority;
    governor_.CopyNzfeasAuthorityReceipt(
        envelope,
        current_longitudinal_evidence.nzfeas,
        longitudinal_source_status);
    if (!longitudinal_source_status.ok())
    {
        output.status = longitudinal_source_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }
    current_longitudinal_evidence.valid = true;

    output.stage = ControlCoreStage::TacticalInput;
    TacticalCommandBuildInput tactical_input{};
    Status tactical_input_status{};
    tactical_input_builder_.Build(
        frame.value,
        estimate.output,
        envelope,
        gcas_entry,
        current_longitudinal_evidence,
        state_.previous_tactical_feedback,
        tactical_input,
        tactical_input_status);
    const TacticalControlPipelineState pre_attempt_state =
        CapturePipelineState();
    bool tactical_failed = false;
    ControlCoreStage tactical_failure_stage = ControlCoreStage::TacticalInput;
    Status tactical_failure_status{};
    ControlIntent tactical{};
    TacticalControlAttempt selected_attempt{};
    ControlCommandOutcome selected_outcome =
        ControlCommandOutcome::InputRejected;
    if (!tactical_input_status.ok() || !tactical_input.valid)
    {
        tactical_failed = true;
        tactical_failure_status = tactical_input_status.ok()
            ? Failure(StatusCode::InvalidConfiguration)
            : tactical_input_status;
    }
    else
    {
        output.stage = ControlCoreStage::TacticalBuild;
        Status tactical_status{};
        CurrentCisV4EnergyProjectionPort projection_port(
            route5_,
            tecs_cis_,
            estimate.plane_state,
            estimate.output,
            envelope,
            context.sample_dt_sec,
            state_.gcas_energy_handoff_active,
            state_.previous_transmitted_auto_gcas_active);
        command_provider_.BuildWithProjection(
            tactical_input,
            projection_port,
            tactical,
            tactical_status);
        if (!tactical_status.ok())
        {
            tactical_failed = true;
            tactical_failure_stage = ControlCoreStage::TacticalBuild;
            tactical_failure_status = tactical_status;
        }
        else
        {
            if (!ClassifyCommandOutcome(tactical, selected_outcome))
            {
                tactical_failed = true;
                tactical_failure_stage = ControlCoreStage::TacticalValidate;
                tactical_failure_status =
                    Failure(StatusCode::InvalidConfiguration);
            }
        }
        if (!tactical_failed)
        {
            output.stage = ControlCoreStage::TacticalValidate;
            ExecuteIntent(
                tactical,
                frame,
                estimate,
                envelope,
                context,
                gcas_entry,
                selected_attempt);
            if (!selected_attempt.valid)
            {
                tactical_failed = true;
                tactical_failure_stage = selected_attempt.failure_stage;
                tactical_failure_status = selected_attempt.status;
            }
        }
    }

    if (tactical_failed)
    {
        RestorePipelineState(pre_attempt_state);
        command_provider_.AbortPrepared();
        output.origin_failure_stage = tactical_failure_stage;
        output.origin_status_code = tactical_failure_status.code;
        ++state_.internal_fault_count;
        output.stage = tactical_failure_stage;
        output.status = tactical_failure_status;
        output.fault_class = ControlCoreFaultClass::InternalFault;
        return;
    }

    Status provider_commit_status{};
    command_provider_.CommitPrepared(
        tactical.frame_identity,
        tactical.writer_id,
        provider_commit_status);
    if (!provider_commit_status.ok())
    {
        // The current command has already passed guidance, FCS/Auto-GCAS and
        // wire validation. A provider bookkeeping fault is diagnostic-only:
        // discard staged owner state but never replace this finite current
        // command with NoCommand, a retained command, or synthetic recovery.
        command_provider_.AbortPrepared();
        output.origin_failure_stage = ControlCoreStage::TacticalCommit;
        output.origin_status_code = provider_commit_status.code;
        ++state_.internal_fault_count;
    }

    output.status = Status{};
    output.stage = ControlCoreStage::Authorized;
    output.control_authorized = true;
    output.outcome = selected_outcome;
    output.fault_class = ControlCoreFaultClass::None;
    output.auto_gcas_override = selected_attempt.auto_gcas_override;
    output.auto_gcas_phase = selected_attempt.auto_gcas_phase;
    output.control = selected_attempt.wire_control;
    CommitTransmitted(
        output.control,
        input.command_frame_index,
        context.source_t_sec,
        selected_attempt.auto_gcas_override);
    state_.previous_tactical_feedback = selected_attempt.next_feedback;
    ++state_.primary_command_count;
}

void TacticalControlCore::BuildAutoGcasEntryInput(
    const FrameContext& context,
    const DogfightGeometryFrame& frame,
    const EstimatorUpdateResult& estimate,
    const control::route5::CommandEnvelope& envelope,
    safety::AutoGcasEntryInput& output) noexcept
{
    output = safety::AutoGcasEntryInput{};
    output.estimator_frame_identity = frame.frame_identity;
    output.envelope_frame_identity = envelope.frame_identity;
    output.t_sec = context.source_t_sec;
    output.dt_s = context.sample_dt_sec;
    output.ownship = estimate.plane_state;

    const bool initial_competition_rate_seed =
        !state_.has_previous_transmitted
        && estimate.output.action_feedback_kind
            == ActionFeedbackKind::ResetSeed
        && !estimate.output.action_source_frame_index.has_value
        && !estimate.output.pqr_valid
        && !estimate.output.pqr_endpoint_valid
        && estimate.output.pqr_gate == BodyRateGate::Init;
    const ControlFrameIdentity& initial_rate_seed_identity =
        state_.initial_rate_seed_identity;
    const bool initial_competition_interval_seed =
        state_.initial_rate_interval_pending
        && IsValidControlFrameIdentity(initial_rate_seed_identity)
        && initial_rate_seed_identity.episode_epoch
            == frame.frame_identity.episode_epoch
        && initial_rate_seed_identity.frame_index
            < frame.frame_identity.frame_index
        && estimate.output.pqr_valid
        && !estimate.output.pqr_endpoint_valid
        && estimate.output.pqr_gate == BodyRateGate::Update
        && std::isfinite(estimate.output.p);

    output.roll_rate_endpoint_radps = initial_competition_rate_seed
        ? 0.0
        : (initial_competition_interval_seed
            ? estimate.output.p
            : estimate.output.p_endpoint);
    output.roll_rate_endpoint_valid = initial_competition_rate_seed
        || initial_competition_interval_seed
        || (estimate.output.pqr_valid
            && estimate.output.pqr_endpoint_valid
            && std::isfinite(estimate.output.p_endpoint));
    if (initial_competition_rate_seed)
    {
        state_.initial_rate_interval_pending = true;
        state_.initial_rate_seed_identity = frame.frame_identity;
    }
    else if (state_.initial_rate_interval_pending)
    {
        state_.initial_rate_interval_pending = false;
        state_.initial_rate_seed_identity = ControlFrameIdentity{};
    }
    output.measured_nz_g = estimate.output.nz;
    output.measured_nz_valid = estimate.output.nz_valid
        && estimate.output.nz_source != EstimatorSource::Uninitialized
        && std::isfinite(estimate.output.nz);
    output.available_nz_g = envelope.nz_feasible_g;
    output.available_nz_valid =
        envelope.valid
        && envelope.enabled
        && envelope.command_containment_authority
        && control::route5::CommandEnvelopeSourceProvidesBounds(
            envelope.source)
        && std::isfinite(envelope.nz_feasible_g)
        && envelope.nz_feasible_g > 0.0;
}

void TacticalControlCore::BuildControlReference(
    const ControlIntent& tactical,
    const DogfightGeometryFrame& frame,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const control::route5::CommandEnvelope& envelope,
    const double dt_s,
    control::tecs_cis::BodyRateLoadEnergyCommand& output,
    TacticalCompletedTotalLoadReceipt& completed_total_load,
    Status& status) noexcept
{
    output = control::tecs_cis::BodyRateLoadEnergyCommand{};
    completed_total_load = TacticalCompletedTotalLoadReceipt{};
    status = Status{};

    ValidateControlRouteSubset(tactical, status);
    if (!status.ok())
    {
        return;
    }

    if (tactical.route_kind == ControlRouteKind::AimPoint
        || tactical.route_kind
            == ControlRouteKind::DirectLoadVectorAcceleration)
    {
        control::route5::Route5GuidanceOutput guidance{};
        control::route5::Route5Guidance projected_route5 = route5_;
        projected_route5.Step(
            tactical,
            ownship,
            estimate,
            envelope,
            dt_s,
            guidance,
            status);
        if (!status.ok() || !guidance.valid)
        {
            if (status.ok())
            {
                status = Failure(StatusCode::InvalidConfiguration);
            }
            return;
        }

        output.frame_identity = guidance.frame_identity;
        output.valid = true;
        output.p_cmd_radps = guidance.p_cmd_radps;
        output.q_cmd_radps = guidance.q_cmd_radps;
        output.r_cmd_radps = guidance.r_cmd_radps;
        output.nz_cmd_g = guidance.nz_cmd_g;
        output.desired_speed_mps = guidance.desired_speed_mps;
        output.desired_speed_rate_mps2 =
            guidance.desired_speed_rate_mps2;
        output.flight_path_angle_cmd_rad =
            guidance.flight_path_angle_cmd_rad;
        output.specific_energy_rate_bias_m2ps3 =
            guidance.specific_energy_rate_bias_m2ps3;
        UpdateEnergyIntegratorHold(
            estimate,
            output.flight_path_angle_cmd_rad,
            output.integrator_hold,
            status);
        if (!status.ok())
        {
            return;
        }
        const bool physical_total_load_available =
            control::route5::IsPhysicalNzCommandEnvelopeSource(
                envelope.source)
            && std::isfinite(envelope.nz_feasible_g)
            && envelope.nz_feasible_g > 0.0;
        if (physical_total_load_available
            && guidance.n_cmd_raw_g > 0.0
            && guidance.n_cmd_g > 0.0
            && guidance.n_cmd_limit_g > 0.0)
        {
            total_load_builder_.Build(
                tactical.frame_identity,
                guidance.n_cmd_raw_g,
                guidance.n_cmd_g,
                guidance.n_cmd_limit_g,
                TacticalCompletedTotalLoadSource::Route5NCommand,
                completed_total_load,
                status);
        }
        if (!status.ok())
        {
            return;
        }
        route5_ = projected_route5;
        direct_ned_.Reset();
        direct_force_tracking_.Reset();
        return;
    }

    if (tactical.route_kind == ControlRouteKind::DirectBodyReferences)
    {
        double gamma_command_limit_rad = 0.0;
        route5_.CopyGammaCommandLimit(gamma_command_limit_rad, status);
        if (!status.ok())
        {
            return;
        }
        control::direct_body::BuildDirectBodyReference(
            tactical,
            ownship,
            estimate,
            envelope,
            gamma_command_limit_rad,
            output,
            status);
        if (!status.ok() || !output.valid)
        {
            if (status.ok())
            {
                status = Failure(StatusCode::InvalidConfiguration);
            }
            return;
        }
        UpdateEnergyIntegratorHold(
            estimate,
            output.flight_path_angle_cmd_rad,
            output.integrator_hold,
            status);
        if (!status.ok())
        {
            return;
        }
        direct_ned_.Reset();
        direct_force_tracking_.Reset();
        return;
    }

    control::direct_ned::DirectNedLoadVectorState direct_state{};
    direct_state.frame_identity = envelope.frame_identity;
    const Vector3 rpy_rad{{estimate.roll, estimate.pitch, estimate.yaw}};
    DirectEulerDcmNedToBody(rpy_rad, direct_state.c_body_from_ned);
    const Vector3 velocity_body_mps{{estimate.u, estimate.v, estimate.w}};
    direct_state.velocity_ned_mps = TransposeMatrixVectorProduct(
        direct_state.c_body_from_ned,
        velocity_body_mps);
    direct_state.speed_mps = estimate.V;
    direct_state.alpha_rad = estimate.alpha;
    direct_state.beta_rad = estimate.beta;
    direct_state.pitch_rad = estimate.pitch;
    direct_state.roll_rad = estimate.roll;
    direct_state.nz_feasible_g = envelope.nz_feasible_g;
    direct_state.ground_speed_horizontal_mps =
        estimate.ground_speed_horizontal_mps;
    direct_state.max_p_radps = envelope.p_max_radps;

    control::direct_ned::DirectNedLoadVectorCommand direct_command{};
    direct_command.frame_identity = tactical.frame_identity;
    direct_command.valid = true;
    direct_command.acceleration_ned_mps2 =
        tactical.direct_acceleration_ned_mps2.value;
    direct_command.roll_rate_reference_valid =
        tactical.direct_acceleration_roll_rate_reference_radps.has_value;
    direct_command.roll_rate_reference_radps =
        tactical.direct_acceleration_roll_rate_reference_radps.value;

    control::direct_ned::DirectNedLoadVector projected_direct_ned =
        direct_ned_;
    control::direct_ned::ForceVectorTracking projected_force_tracking =
        direct_force_tracking_;
    if (!tactical.direct_acceleration_tracking_enabled)
    {
        projected_force_tracking.Reset();
    }

    if (tactical.direct_acceleration_tracking_enabled)
    {
        control::direct_ned::ForceVectorTrackingInput tracking_input{};
        tracking_input.requested_acceleration_ned_mps2 =
            direct_command.acceleration_ned_mps2;
        tracking_input.velocity_ned_mps = frame.own.velocity_ned_mps;
        tracking_input.dt_s = dt_s;
        tracking_input.observed_kinematic_bank_rad = estimate.mu;
        tracking_input.magnitude_tracking_enabled =
            tactical.direct_acceleration_magnitude_tracking_enabled;

        const double nose_norm = VectorNorm(frame.own.nose_ned);
        tracking_input.excluded_force_valid = estimate.thrust_valid
            && estimate.mass_valid
            && std::isfinite(estimate.thrust)
            && std::isfinite(estimate.mass)
            && estimate.mass > 0.0
            && FiniteVector(frame.own.nose_ned)
            && std::isfinite(nose_norm)
            && nose_norm > 0.0;
        if (tracking_input.excluded_force_valid)
        {
            const double thrust_specific_force_mps2 =
                estimate.thrust / estimate.mass;
            if (!std::isfinite(thrust_specific_force_mps2))
            {
                status = Failure(StatusCode::NonFiniteInput);
                return;
            }
            tracking_input.excluded_specific_force_ned_mps2 = Vector3{{
                thrust_specific_force_mps2
                    * frame.own.nose_ned[0] / nose_norm,
                thrust_specific_force_mps2
                    * frame.own.nose_ned[1] / nose_norm,
                thrust_specific_force_mps2
                    * frame.own.nose_ned[2] / nose_norm}};
            if (!FiniteVector(
                    tracking_input.excluded_specific_force_ned_mps2))
            {
                status = Failure(StatusCode::NonFiniteInput);
                return;
            }
        }

        control::direct_ned::ForceVectorTrackingOutput tracking_output{};
        projected_force_tracking.Step(
            tracking_input,
            tracking_output,
            status);
        if (!status.ok() || !tracking_output.valid)
        {
            if (status.ok())
            {
                status = Failure(StatusCode::InvalidConfiguration);
            }
            return;
        }
        direct_command.acceleration_ned_mps2 =
            tactical.direct_acceleration_tracking_observation_only
            ? tracking_output.requested_acceleration_ned_mps2
            : tracking_output.applied_acceleration_ned_mps2;
    }

    control::direct_ned::DirectNedLoadVectorOutput raw_reference{};
    projected_direct_ned.Step(
        direct_command,
        direct_state,
        dt_s,
        raw_reference,
        status);
    if (!status.ok() || !raw_reference.valid)
    {
        if (status.ok())
        {
            status = Failure(StatusCode::InvalidConfiguration);
        }
        return;
    }

    double gamma_command_limit_rad = 0.0;
    route5_.CopyGammaCommandLimit(gamma_command_limit_rad, status);
    if (!status.ok())
    {
        return;
    }
    const Vector3 displacement_ned_m{{
        tactical.aim_point_m[0] - ownship.position_ned_m[0],
        tactical.aim_point_m[1] - ownship.position_ned_m[1],
        tactical.aim_point_m[2] - ownship.position_ned_m[2]}};
    const double horizontal_range_m = std::hypot(
        displacement_ned_m[0],
        displacement_ned_m[1]);
    const double gamma_command_raw_rad = std::atan2(
        -displacement_ned_m[2],
        std::max(horizontal_range_m, constants::Epsilon));
    const double throttle_bias = tactical.throttle_bias.has_value
        ? tactical.throttle_bias.value
        : 0.0;

    control::direct_ned::DirectNedLongitudinalReference longitudinal{};
    longitudinal.frame_identity = tactical.frame_identity;
    longitudinal.valid = true;
    longitudinal.desired_speed_mps = tactical.desired_speed_mps
        + DirectNedThrottleBiasToSpeedMps * throttle_bias;
    longitudinal.desired_speed_rate_mps2 =
        tactical.desired_speed_rate_mps2;
    longitudinal.flight_path_angle_cmd_rad = std::max(
        -gamma_command_limit_rad,
        std::min(gamma_command_raw_rad, gamma_command_limit_rad));
    // The base direct-NED longitudinal owner consumes only the explicit
    // energy-rate bias. Unsupported optional direct-accel evidence is
    // command-neutral and does not replace this finite base reference.
    longitudinal.specific_energy_rate_bias_m2ps3 =
        tactical.specific_energy_rate_bias_m2ps3;

    direct_ned_shaper_.Shape(
        raw_reference,
        direct_state,
        envelope,
        longitudinal,
        tactical.direct_acceleration_loaded_roll_enabled
            && tactical.writer_id
                == ControlIntentWriterG4HighGBarrel,
        output,
        status);
    if (!status.ok() || !output.valid)
    {
        if (status.ok())
        {
            status = Failure(StatusCode::InvalidConfiguration);
        }
        return;
    }
    UpdateEnergyIntegratorHold(
        estimate,
        output.flight_path_angle_cmd_rad,
        output.integrator_hold,
        status);
    if (!status.ok())
    {
        return;
    }
    const double governed_total_load_g =
        raw_reference.force_perp_norm_g * raw_reference.clip_scale;
    const bool physical_total_load_available =
        control::route5::IsPhysicalNzCommandEnvelopeSource(envelope.source)
        && std::isfinite(envelope.nz_feasible_g)
        && envelope.nz_feasible_g > 0.0;
    if (physical_total_load_available
        && raw_reference.force_perp_norm_g > 0.0
        && governed_total_load_g > 0.0)
    {
        total_load_builder_.Build(
            tactical.frame_identity,
            raw_reference.force_perp_norm_g,
            governed_total_load_g,
            envelope.nz_feasible_g,
            TacticalCompletedTotalLoadSource::
                DirectNedVelocityNormalForce,
            completed_total_load,
            status);
    }
    if (!status.ok())
    {
        return;
    }
    direct_ned_ = projected_direct_ned;
    direct_force_tracking_ = projected_force_tracking;
}

void TacticalControlCore::UpdateEnergyIntegratorHold(
    const EstimatorOutputV6& estimate,
    const double flight_path_angle_cmd_rad,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    bool next_handoff_active = false;
    if (!EvaluateEnergyIntegratorHold(
            estimate,
            flight_path_angle_cmd_rad,
            state_.gcas_energy_handoff_active,
            state_.previous_transmitted_auto_gcas_active,
            next_handoff_active,
            output))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }
    state_.gcas_energy_handoff_active = next_handoff_active;
}

void TacticalControlCore::CommitTransmitted(
    const ControlValue& control,
    const std::uint64_t command_frame,
    const double command_time_s,
    const bool auto_gcas_active) noexcept
{
    state_.has_previous_transmitted = true;
    state_.previous_transmitted_auto_gcas_active = auto_gcas_active;
    state_.previous_command_frame = command_frame;
    state_.previous_command_time_s = command_time_s;
    state_.previous_transmitted = control;
}

void TacticalControlCore::RefreshSnapshots() noexcept
{
    state_.estimator = estimator_.Snapshot();
    direct_ned_.CopySnapshot(state_.direct_ned);
    direct_force_tracking_.CopySnapshot(state_.direct_force_tracking);
    route5_.CopySnapshot(state_.route5);
    tecs_cis_.CopySnapshot(state_.tecs_cis);
    auto_gcas_.CopySnapshot(state_.auto_gcas);
}

void TacticalControlCore::CopySnapshot(
    TacticalControlCoreSnapshot& output) const noexcept
{
    output = state_;
}

} // namespace runtime
} // namespace LadyLuck
