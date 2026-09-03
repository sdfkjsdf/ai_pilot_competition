#include "LadyLuck/guidance/prefire/RootGunTowardSideObservation.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::prefire::RootGunTowardSideReason;
using LadyLuck::guidance::prefire::RootGunTowardSideShadowReceipt;
using LadyLuck::guidance::prefire::SignedLateralInterval;

constexpr double kBattleServerRpyQuantumRad =
    LadyLuck::constants::Pi / 180.0 / 1000.0;

double PositiveInfinity() noexcept
{
    return (std::numeric_limits<double>::infinity)();
}

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Hypot3(const Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

void Reject(
    RootGunTowardSideShadowReceipt& output,
    const RootGunTowardSideReason reason) noexcept
{
    output.reason = reason;
}

void Fail(
    RootGunTowardSideShadowReceipt& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = RootGunTowardSideShadowReceipt{};
    output.reason =
        RootGunTowardSideReason::RootGunTowardSideObserverContractRejected;
    status.code = code;
}

bool AddUp(const double a, const double b, double& output) noexcept
{
    output = std::nextafter(a + b, PositiveInfinity());
    return std::isfinite(output);
}

bool AddDown(const double a, const double b, double& output) noexcept
{
    output = std::nextafter(a + b, -PositiveInfinity());
    return std::isfinite(output);
}

bool SubtractDown(const double a, const double b, double& output) noexcept
{
    output = std::nextafter(a - b, -PositiveInfinity());
    return std::isfinite(output);
}

bool SubtractUp(const double a, const double b, double& output) noexcept
{
    output = std::nextafter(a - b, PositiveInfinity());
    return std::isfinite(output);
}

bool MultiplyNonnegativeUp(
    const double a,
    const double b,
    double& output) noexcept
{
    if (a < 0.0 || b < 0.0)
    {
        return false;
    }
    output = std::nextafter(a * b, PositiveInfinity());
    return std::isfinite(output);
}

bool ProductInterval(
    const double a,
    const double b,
    double& lower,
    double& upper) noexcept
{
    const double product = a * b;
    if (!std::isfinite(product))
    {
        return false;
    }
    lower = std::nextafter(product, -PositiveInfinity());
    upper = std::nextafter(product, PositiveInfinity());
    return std::isfinite(lower) && std::isfinite(upper);
}

bool DivisionByPositiveInterval(
    const double numerator,
    const double denominator_lower,
    const double denominator_upper,
    double& lower,
    double& upper) noexcept
{
    if (!(denominator_lower > 0.0)
        || denominator_lower > denominator_upper)
    {
        return false;
    }
    const double lower_nominal = numerator >= 0.0
        ? numerator / denominator_upper
        : numerator / denominator_lower;
    const double upper_nominal = numerator >= 0.0
        ? numerator / denominator_lower
        : numerator / denominator_upper;
    lower = std::nextafter(lower_nominal, -PositiveInfinity());
    upper = std::nextafter(upper_nominal, PositiveInfinity());
    return std::isfinite(lower) && std::isfinite(upper);
}

bool NormalizationDivisionErrorL1(
    const double raw_x,
    const double raw_y,
    const double computed_norm,
    const Vector3& stored_course,
    double& output) noexcept
{
    const double norm_lower = std::nextafter(computed_norm, 0.0);
    const double norm_upper = std::nextafter(
        computed_norm,
        PositiveInfinity());
    if (!std::isfinite(norm_lower)
        || !std::isfinite(norm_upper)
        || norm_lower <= 0.0)
    {
        return false;
    }

    double errors[2] = {0.0, 0.0};
    const double raw[2] = {raw_x, raw_y};
    for (std::size_t index = 0U; index < 2U; ++index)
    {
        double exact_lower = 0.0;
        double exact_upper = 0.0;
        if (!DivisionByPositiveInterval(
                raw[index],
                norm_lower,
                norm_upper,
                exact_lower,
                exact_upper))
        {
            return false;
        }
        const double lower_error = std::nextafter(
            std::fabs(stored_course[index] - exact_lower),
            PositiveInfinity());
        const double upper_error = std::nextafter(
            std::fabs(exact_upper - stored_course[index]),
            PositiveInfinity());
        errors[index] = (std::max)(lower_error, upper_error);
        if (!std::isfinite(errors[index]))
        {
            return false;
        }
    }
    return AddUp(errors[0], errors[1], output);
}

struct AxisSample
{
    Vector3 relative_position{};
    Vector3 horizontal_axis{};
    double relative_position_error_bound_m = 0.0;
    double axis_error_bound_rad = 0.0;
    double normalization_error_bound_l1 = 0.0;
};

bool BuildAxisSample(
    const Vector3& relative_position,
    const double relative_position_error_bound_m,
    const Vector3& raw_horizontal_axis,
    const double axis_error_bound_rad,
    AxisSample& output) noexcept
{
    if (!FiniteVector(relative_position)
        || !FiniteVector(raw_horizontal_axis)
        || raw_horizontal_axis[2] != 0.0
        || !std::isfinite(relative_position_error_bound_m)
        || relative_position_error_bound_m < 0.0
        || !std::isfinite(axis_error_bound_rad)
        || axis_error_bound_rad < 0.0
        || axis_error_bound_rad > LadyLuck::constants::Pi)
    {
        return false;
    }
    const double norm = std::hypot(
        raw_horizontal_axis[0],
        raw_horizontal_axis[1]);
    if (!std::isfinite(norm) || norm <= 0.0)
    {
        return false;
    }
    const Vector3 normalized{{
        raw_horizontal_axis[0] / norm,
        raw_horizontal_axis[1] / norm,
        0.0}};
    if (!FiniteVector(normalized))
    {
        return false;
    }
    double normalization_error = 0.0;
    if (!NormalizationDivisionErrorL1(
            raw_horizontal_axis[0],
            raw_horizontal_axis[1],
            norm,
            normalized,
            normalization_error))
    {
        return false;
    }
    output.relative_position = relative_position;
    output.horizontal_axis = normalized;
    output.relative_position_error_bound_m =
        relative_position_error_bound_m;
    output.axis_error_bound_rad = axis_error_bound_rad;
    output.normalization_error_bound_l1 = normalization_error;
    return true;
}

bool AxisNormDefectBound(
    const Vector3& axis,
    double& output) noexcept
{
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;
    if (!ProductInterval(axis[0], axis[0], x_lower, x_upper)
        || !ProductInterval(axis[1], axis[1], y_lower, y_upper))
    {
        return false;
    }
    double squared_lower = 0.0;
    double squared_upper = 0.0;
    if (!AddDown(x_lower, y_lower, squared_lower)
        || !AddUp(x_upper, y_upper, squared_upper))
    {
        return false;
    }
    const double lower_defect = std::nextafter(
        std::fabs(squared_lower - 1.0),
        PositiveInfinity());
    const double upper_defect = std::nextafter(
        std::fabs(squared_upper - 1.0),
        PositiveInfinity());
    output = (std::max)(lower_defect, upper_defect);
    return std::isfinite(output);
}

bool BoundedSignedLateralInterval(
    const AxisSample& sample,
    SignedLateralInterval& output) noexcept
{
    const Vector3& axis = sample.horizontal_axis;
    const Vector3& relative_position = sample.relative_position;
    const double product_a = axis[0] * relative_position[1];
    const double product_b = axis[1] * relative_position[0];
    const double nominal = product_a - product_b;
    if (!std::isfinite(product_a)
        || !std::isfinite(product_b)
        || !std::isfinite(nominal))
    {
        return false;
    }

    double product_a_lower = 0.0;
    double product_a_upper = 0.0;
    double product_b_lower = 0.0;
    double product_b_upper = 0.0;
    if (!ProductInterval(
            axis[0],
            relative_position[1],
            product_a_lower,
            product_a_upper)
        || !ProductInterval(
            axis[1],
            relative_position[0],
            product_b_lower,
            product_b_upper))
    {
        return false;
    }

    double determinant_lower = 0.0;
    double determinant_upper = 0.0;
    if (!SubtractDown(
            product_a_lower,
            product_b_upper,
            determinant_lower)
        || !SubtractUp(
            product_a_upper,
            product_b_lower,
            determinant_upper))
    {
        return false;
    }

    double norm_defect = 0.0;
    if (!AxisNormDefectBound(axis, norm_defect))
    {
        return false;
    }
    const double angular_chord_bound = (std::min)(
        sample.axis_error_bound_rad,
        2.0);
    double normalization_bound = 0.0;
    if (!MultiplyNonnegativeUp(
            2.0,
            sample.normalization_error_bound_l1,
            normalization_bound))
    {
        return false;
    }
    double first_axis_bound = 0.0;
    double axis_vector_bound = 0.0;
    if (!AddUp(angular_chord_bound, norm_defect, first_axis_bound)
        || !AddUp(
            first_axis_bound,
            normalization_bound,
            axis_vector_bound))
    {
        return false;
    }
    double rho_l1_upper = 0.0;
    if (!AddUp(
            std::fabs(relative_position[0]),
            std::fabs(relative_position[1]),
            rho_l1_upper))
    {
        return false;
    }
    double axis_projection_bound = 0.0;
    if (!MultiplyNonnegativeUp(
            axis_vector_bound,
            rho_l1_upper,
            axis_projection_bound))
    {
        return false;
    }
    double beta = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    if (!AddUp(
            sample.relative_position_error_bound_m,
            axis_projection_bound,
            beta)
        || !SubtractDown(determinant_lower, beta, lower)
        || !AddUp(determinant_upper, beta, upper))
    {
        return false;
    }
    output.nominal = nominal;
    output.lower = lower;
    output.upper = upper;
    output.resolved_sign = lower > 0.0 ? 1 : (upper < 0.0 ? -1 : 0);
    return true;
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    if (!FiniteVector(value))
    {
        return false;
    }
    const float float_infinity =
        (std::numeric_limits<float>::infinity)();
    Vector3 residual{};
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float represented_f32 = static_cast<float>(value[index]);
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(
            std::nextafter(represented_f32, float_infinity));
        const double lower = static_cast<double>(
            std::nextafter(represented_f32, -float_infinity));
        if (!std::isfinite(represented)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return false;
        }
        residual[index] = value[index] - represented;
        full_cell[index] = (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }
    const double residual_norm = std::nextafter(
        Hypot3(residual),
        PositiveInfinity());
    const double cell_norm = std::nextafter(
        Hypot3(full_cell),
        PositiveInfinity());
    output = std::nextafter(
        residual_norm + cell_norm,
        PositiveInfinity());
    return std::isfinite(output);
}

bool Binary64VectorResultRoundoffBound(
    const Vector3& value,
    double& output) noexcept
{
    if (!FiniteVector(value))
    {
        return false;
    }
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double upper = std::nextafter(
            value[index],
            PositiveInfinity());
        const double lower = std::nextafter(
            value[index],
            -PositiveInfinity());
        full_cell[index] = (std::max)(
            std::fabs(upper - value[index]),
            std::fabs(value[index] - lower));
        if (!std::isfinite(full_cell[index]))
        {
            return false;
        }
    }
    output = std::nextafter(
        Hypot3(full_cell),
        PositiveInfinity());
    return std::isfinite(output);
}

bool HorizontalCourseErrorBoundRad(
    const Vector3& direction,
    const double direction_error_bound_rad,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    if (!FiniteVector(direction)
        || !std::isfinite(direction_error_bound_rad)
        || direction_error_bound_rad < 0.0
        || direction_error_bound_rad > LadyLuck::constants::Pi)
    {
        return false;
    }
    const double magnitude_upper = std::nextafter(
        Hypot3(direction),
        PositiveInfinity());
    const double horizontal_lower = std::nextafter(
        std::hypot(direction[0], direction[1]),
        0.0);
    if (!std::isfinite(magnitude_upper) || magnitude_upper <= 0.0)
    {
        return false;
    }
    const double horizontal_fraction_lower = std::nextafter(
        horizontal_lower / magnitude_upper,
        0.0);
    if (direction_error_bound_rad >= 0.5 * LadyLuck::constants::Pi
        || horizontal_fraction_lower <= direction_error_bound_rad)
    {
        return true;
    }
    const double ratio_upper = (std::min)(
        1.0,
        std::nextafter(
            direction_error_bound_rad / horizontal_fraction_lower,
            PositiveInfinity()));
    output = std::nextafter(
        std::asin(ratio_upper),
        PositiveInfinity());
    observable = std::isfinite(output);
    return observable;
}

bool HorizontalResultDirectionErrorBoundRad(
    const Vector3& direction,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    double result_error_bound = 0.0;
    if (!Binary64VectorResultRoundoffBound(direction, result_error_bound))
    {
        return false;
    }
    const double magnitude_lower = std::nextafter(
        Hypot3(direction),
        0.0);
    if (!std::isfinite(magnitude_lower)
        || magnitude_lower <= result_error_bound)
    {
        return true;
    }
    const double ratio_upper = (std::min)(
        1.0,
        std::nextafter(
            result_error_bound / magnitude_lower,
            PositiveInfinity()));
    const double direction_bound = std::nextafter(
        std::asin(ratio_upper),
        PositiveInfinity());
    return HorizontalCourseErrorBoundRad(
        direction,
        direction_bound,
        observable,
        output);
}

bool ExactTowardSide(
    const DogfightGeometryFrame& frame,
    const std::int32_t tie_side,
    std::int32_t& output) noexcept
{
    if (tie_side != -1 && tie_side != 1)
    {
        return false;
    }
    Vector3 nose = frame.own.nose_ned;
    nose[2] = 0.0;
    Vector3 los = Subtract(
        frame.opponent.position_ned_m,
        frame.own.position_ned_m);
    los[2] = 0.0;
    const double nose_norm = NumpyNorm3(nose);
    const double los_norm = NumpyNorm3(los);
    if (!std::isfinite(nose_norm)
        || !std::isfinite(los_norm)
        || nose_norm < LadyLuck::constants::Tiny
        || los_norm < LadyLuck::constants::Tiny)
    {
        return false;
    }
    nose = Vector3{{
        nose[0] / nose_norm,
        nose[1] / nose_norm,
        0.0}};
    los = Vector3{{
        los[0] / los_norm,
        los[1] / los_norm,
        0.0}};
    if (Dot3(nose, los) >= 0.0)
    {
        return false;
    }
    const double cross_sign = nose[0] * los[1] - nose[1] * los[0];
    output = cross_sign > 0.0
        ? -1
        : (cross_sign < 0.0 ? 1 : tie_side);
    return true;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

const char* RootGunTowardSideReasonLabel(
    const RootGunTowardSideReason reason) noexcept
{
    switch (reason)
    {
    case RootGunTowardSideReason::RootGunOwnerInactive:
        return "root_gun_owner_inactive";
    case RootGunTowardSideReason::SameIndexGeometryFrameEnvelopeMissing:
        return "same_index_geometry_frame_envelope_missing";
    case RootGunTowardSideReason::SameIndexGeometryFrameEnvelopeTypeInvalid:
        return "same_index_geometry_frame_envelope_type_invalid";
    case RootGunTowardSideReason::RootGunTowardSideGeometryContractRejected:
        return "root_gun_toward_side_geometry_contract_rejected";
    case RootGunTowardSideReason::HorizontalNoseUnobservableWithinBounds:
        return "horizontal_nose_unobservable_within_bounds";
    case RootGunTowardSideReason::AttackerNotResolvedInStrictRearHalfspace:
        return "attacker_not_resolved_in_strict_rear_halfspace";
    case RootGunTowardSideReason::AttackerSideUnresolvedWithinBounds:
        return "attacker_side_unresolved_within_bounds";
    case RootGunTowardSideReason::SelectedRootGunCommandMissing:
        return "selected_root_gun_command_missing";
    case RootGunTowardSideReason::SelectedRootGunCommandTypeInvalid:
        return "selected_root_gun_command_type_invalid";
    case RootGunTowardSideReason::SelectedRootGunCommandDirectionUnobservable:
        return "selected_root_gun_command_direction_unobservable";
    case RootGunTowardSideReason::SelectedRootGunCommandSideUnresolvedWithinBounds:
        return "selected_root_gun_command_side_unresolved_within_bounds";
    case RootGunTowardSideReason::RootGunTowardSideObservationPublished:
        return "root_gun_toward_side_observation_published";
    case RootGunTowardSideReason::RootGunTowardSideObserverContractRejected:
    default:
        return "root_gun_toward_side_observer_contract_rejected";
    }
}

void ObserveRootGunTowardSideShadow(
    const DogfightGeometryFrame& frame,
    const SameIndexGeometryFrameEnvelope* const envelope,
    const ControlIntent* const selected_command,
    const bool root_gun_owner_active,
    RootGunTowardSideShadowReceipt& output,
    Status& status) noexcept
{
    output = RootGunTowardSideShadowReceipt{};
    status = Status{};
    if (!root_gun_owner_active)
    {
        Reject(output, RootGunTowardSideReason::RootGunOwnerInactive);
        return;
    }
    if (envelope == nullptr)
    {
        Reject(
            output,
            RootGunTowardSideReason::SameIndexGeometryFrameEnvelopeMissing);
        return;
    }
    if (!IsValidControlFrameIdentity(envelope->frame_identity)
        || !SameControlFrameIdentity(
            envelope->frame_identity,
            frame.frame_identity)
        || !std::isfinite(envelope->t_sec)
        || envelope->t_sec < 0.0
        || envelope->t_sec != frame.t_sec)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& attacker_position = frame.opponent.position_ned_m;
    const Vector3& own_nose = frame.own.nose_ned;
    if (!FiniteVector(own_position)
        || !FiniteVector(attacker_position)
        || !FiniteVector(own_nose))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    double own_position_bound = 0.0;
    double attacker_position_bound = 0.0;
    if (!Float32WireVectorErrorBound(
            own_position,
            own_position_bound)
        || !Float32WireVectorErrorBound(
            attacker_position,
            attacker_position_bound))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    const Vector3 relative_position = Subtract(
        attacker_position,
        own_position);
    double subtraction_bound = 0.0;
    if (!Binary64VectorResultRoundoffBound(
            relative_position,
            subtraction_bound))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    const double first_position_sum = std::nextafter(
        own_position_bound + attacker_position_bound,
        PositiveInfinity());
    const double position_bound = std::nextafter(
        first_position_sum + subtraction_bound,
        PositiveInfinity());
    const double nose_direction_bound = std::nextafter(
        3.0 * kBattleServerRpyQuantumRad,
        PositiveInfinity());
    bool nose_observable = false;
    double nose_horizontal_bound = 0.0;
    if (!HorizontalCourseErrorBoundRad(
            own_nose,
            nose_direction_bound,
            nose_observable,
            nose_horizontal_bound))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }
    if (!nose_observable)
    {
        Reject(
            output,
            RootGunTowardSideReason::HorizontalNoseUnobservableWithinBounds);
        return;
    }

    AxisSample nose_sample{};
    const Vector3 horizontal_nose_raw{{
        own_nose[0],
        own_nose[1],
        0.0}};
    if (!BuildAxisSample(
            relative_position,
            position_bound,
            horizontal_nose_raw,
            nose_horizontal_bound,
            nose_sample)
        || !BoundedSignedLateralInterval(
            nose_sample,
            output.attacker_side_interval))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }
    output.attacker_side_interval_valid = true;

    const Vector3 clockwise_axis{{
        nose_sample.horizontal_axis[1],
        -nose_sample.horizontal_axis[0],
        0.0}};
    AxisSample clockwise_sample{};
    if (!BuildAxisSample(
            relative_position,
            position_bound,
            clockwise_axis,
            nose_horizontal_bound,
            clockwise_sample)
        || !BoundedSignedLateralInterval(
            clockwise_sample,
            output.rear_projection_interval))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }
    output.rear_projection_interval_valid = true;
    output.evaluated = true;

    if (output.rear_projection_interval.upper >= 0.0)
    {
        Reject(
            output,
            RootGunTowardSideReason::AttackerNotResolvedInStrictRearHalfspace);
        return;
    }
    if (output.attacker_side_interval.resolved_sign == 0)
    {
        Reject(
            output,
            RootGunTowardSideReason::AttackerSideUnresolvedWithinBounds);
        return;
    }

    output.toward_side_sign =
        -output.attacker_side_interval.resolved_sign;
    output.toward_side_sign_valid = true;
    std::int32_t exact_side = 0;
    if (!ExactTowardSide(
            frame,
            output.toward_side_sign,
            exact_side)
        || exact_side != output.toward_side_sign)
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    if (selected_command == nullptr)
    {
        Reject(
            output,
            RootGunTowardSideReason::SelectedRootGunCommandMissing);
        return;
    }
    const Vector3& aim_point = selected_command->aim_point_m;
    if (!FiniteVector(aim_point))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    Vector3 aim_direction = Subtract(aim_point, own_position);
    aim_direction[2] = 0.0;
    bool aim_observable = false;
    double aim_direction_bound = 0.0;
    if (!HorizontalResultDirectionErrorBoundRad(
            aim_direction,
            aim_observable,
            aim_direction_bound))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }
    if (!aim_observable)
    {
        Reject(
            output,
            RootGunTowardSideReason::SelectedRootGunCommandDirectionUnobservable);
        return;
    }
    AxisSample command_sample{};
    if (!BuildAxisSample(
            relative_position,
            position_bound,
            aim_direction,
            aim_direction_bound,
            command_sample)
        || !BoundedSignedLateralInterval(
            command_sample,
            output.selected_command_side_interval))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }
    output.selected_command_side_interval_valid = true;
    if (output.selected_command_side_interval.resolved_sign == 0)
    {
        Reject(
            output,
            RootGunTowardSideReason::SelectedRootGunCommandSideUnresolvedWithinBounds);
        return;
    }

    output.selected_command_side_sign =
        -output.selected_command_side_interval.resolved_sign;
    output.selected_command_side_sign_valid = true;
    output.selected_command_matches_toward =
        output.selected_command_side_sign == output.toward_side_sign;
    output.selected_command_matches_toward_valid = true;
    Reject(
        output,
        RootGunTowardSideReason::RootGunTowardSideObservationPublished);
}

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
