#include "LadyLuck/guidance/obfm/RelativeFlightPathCrossingObserver.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::AircraftGeometryKinematics;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::RelativeFlightPathCrossingObserver;
using LadyLuck::guidance::obfm::RelativeFlightPathCrossingReason;
using LadyLuck::guidance::obfm::RelativeFlightPathCrossingReceipt;
using LadyLuck::guidance::obfm::RelativeFlightPathSignedLateralInterval;

using FlightPathSample =
    RelativeFlightPathCrossingObserver::HorizontalFlightPathSample;

constexpr double kBodyVelocityQuantumMps = 0.001 * 0.3048;
constexpr double kRpyQuantumRad =
    LadyLuck::constants::Pi / (180.0 * 1000.0);

enum class SampleBuildOutcome : std::uint8_t
{
    Observed = 0U,
    HorizontalCourseUnobservable = 1U,
    Invalid = 2U
};

enum class DirectionResolutionOutcome : std::uint8_t
{
    Resolved = 0U,
    Unobservable = 1U,
    Invalid = 2U
};

double PositiveInfinity() noexcept
{
    return (std::numeric_limits<double>::infinity)();
}

double NegativeInfinity() noexcept
{
    return -(std::numeric_limits<double>::infinity)();
}

double NextUp(const double value) noexcept
{
    return std::nextafter(value, PositiveInfinity());
}

double NextDown(const double value) noexcept
{
    return std::nextafter(value, NegativeInfinity());
}

bool Finite(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

double MathHypot3(const Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

bool AddUp(const double left, const double right, double& output) noexcept
{
    output = NextUp(left + right);
    return std::isfinite(output);
}

bool AddDown(const double left, const double right, double& output) noexcept
{
    output = NextDown(left + right);
    return std::isfinite(output);
}

bool SubtractUp(
    const double left,
    const double right,
    double& output) noexcept
{
    output = NextUp(left - right);
    return std::isfinite(output);
}

bool SubtractDown(
    const double left,
    const double right,
    double& output) noexcept
{
    output = NextDown(left - right);
    return std::isfinite(output);
}

bool MultiplyNonnegativeUp(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (left < 0.0 || right < 0.0)
    {
        return false;
    }
    output = NextUp(left * right);
    return std::isfinite(output);
}

bool ProductInterval(
    const double left,
    const double right,
    double& lower,
    double& upper) noexcept
{
    const double product = left * right;
    lower = NextDown(product);
    upper = NextUp(product);
    return std::isfinite(product)
        && std::isfinite(lower)
        && std::isfinite(upper);
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(value))
    {
        return false;
    }
    Vector3 represented{};
    Vector3 full_cell{};
    const float positive_infinity =
        (std::numeric_limits<float>::infinity)();
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        const float encoded = static_cast<float>(value[axis]);
        if (!std::isfinite(encoded))
        {
            return false;
        }
        const float upper = std::nextafter(encoded, positive_infinity);
        const float lower = std::nextafter(encoded, -positive_infinity);
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        represented[axis] = static_cast<double>(encoded);
        full_cell[axis] = (std::max)(
            std::fabs(static_cast<double>(upper) - represented[axis]),
            std::fabs(represented[axis] - static_cast<double>(lower)));
    }
    const double residual = NextUp(MathHypot3(Subtract(value, represented)));
    const double cell = NextUp(MathHypot3(full_cell));
    output = NextUp(residual + cell);
    return std::isfinite(output) && output >= 0.0;
}

bool Binary64VectorRoundoffBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(value))
    {
        return false;
    }
    Vector3 full_cell{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        const double upper = NextUp(value[axis]);
        const double lower = NextDown(value[axis]);
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        full_cell[axis] = (std::max)(
            std::fabs(upper - value[axis]),
            std::fabs(value[axis] - lower));
    }
    output = NextUp(MathHypot3(full_cell));
    return std::isfinite(output) && output >= 0.0;
}

DirectionResolutionOutcome BodyVelocityDirectionResolution(
    const Vector3& body_velocity,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(body_velocity))
    {
        return DirectionResolutionOutcome::Invalid;
    }
    const double speed = MathHypot3(body_velocity);
    Vector3 half_cell{};
    const float positive_infinity =
        (std::numeric_limits<float>::infinity)();
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        const float encoded = static_cast<float>(body_velocity[axis]);
        const float upper = std::nextafter(encoded, positive_infinity);
        const float lower = std::nextafter(encoded, -positive_infinity);
        if (!std::isfinite(encoded)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return DirectionResolutionOutcome::Invalid;
        }
        const double represented = static_cast<double>(encoded);
        half_cell[axis] = 0.5 * (std::max)(
            std::fabs(static_cast<double>(upper) - represented),
            std::fabs(represented - static_cast<double>(lower)));
    }
    const double error = std::sqrt(3.0) * kBodyVelocityQuantumMps
        + MathHypot3(half_cell);
    const double true_speed_lower = speed - error;
    if (!std::isfinite(speed)
        || !std::isfinite(error))
    {
        return DirectionResolutionOutcome::Invalid;
    }
    if (true_speed_lower <= 0.0)
    {
        return DirectionResolutionOutcome::Unobservable;
    }
    const double ratio = error >= true_speed_lower
        ? 1.0
        : error / true_speed_lower;
    output = std::asin(ratio);
    return std::isfinite(output)
        ? DirectionResolutionOutcome::Resolved
        : DirectionResolutionOutcome::Invalid;
}

DirectionResolutionOutcome AircraftFlightDirectionError(
    const AircraftGeometryKinematics& aircraft,
    double& output) noexcept
{
    output = 0.0;
    double body_bound = 0.0;
    const DirectionResolutionOutcome body_outcome =
        BodyVelocityDirectionResolution(
            aircraft.velocity_body_mps,
            body_bound);
    if (body_outcome != DirectionResolutionOutcome::Resolved)
    {
        return body_outcome;
    }
    output = NextUp(body_bound + 3.0 * kRpyQuantumRad);
    return std::isfinite(output)
            && output >= 0.0
            && output <= LadyLuck::constants::Pi
        ? DirectionResolutionOutcome::Resolved
        : DirectionResolutionOutcome::Invalid;
}

bool HorizontalCourseError(
    const Vector3& direction,
    const double direction_error,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    if (!Finite(direction)
        || !std::isfinite(direction_error)
        || direction_error < 0.0
        || direction_error > LadyLuck::constants::Pi)
    {
        return false;
    }
    const double magnitude_upper = NextUp(MathHypot3(direction));
    const double horizontal_lower = std::nextafter(
        std::hypot(direction[0], direction[1]),
        0.0);
    if (!std::isfinite(magnitude_upper) || magnitude_upper <= 0.0)
    {
        return false;
    }
    const double fraction_lower = std::nextafter(
        horizontal_lower / magnitude_upper,
        0.0);
    if (direction_error >= 0.5 * LadyLuck::constants::Pi
        || fraction_lower <= direction_error)
    {
        return true;
    }
    const double ratio_upper = (std::min)(
        1.0,
        NextUp(direction_error / fraction_lower));
    output = NextUp(std::asin(ratio_upper));
    observable = std::isfinite(output);
    return observable;
}

bool DivideByPositiveInterval(
    const double numerator,
    const double denominator_lower,
    const double denominator_upper,
    double& lower,
    double& upper) noexcept
{
    lower = 0.0;
    upper = 0.0;
    if (!(0.0 < denominator_lower
        && denominator_lower <= denominator_upper))
    {
        return false;
    }
    if (numerator >= 0.0)
    {
        lower = NextDown(numerator / denominator_upper);
        upper = NextUp(numerator / denominator_lower);
    }
    else
    {
        lower = NextDown(numerator / denominator_lower);
        upper = NextUp(numerator / denominator_upper);
    }
    return std::isfinite(lower) && std::isfinite(upper);
}

bool CourseNormalizationError(
    const double raw_x,
    const double raw_y,
    const double norm,
    const Vector3& stored,
    double& output) noexcept
{
    output = 0.0;
    const double norm_lower = std::nextafter(norm, 0.0);
    const double norm_upper = NextUp(norm);
    if (!std::isfinite(norm_lower)
        || !std::isfinite(norm_upper)
        || norm_lower <= 0.0)
    {
        return false;
    }
    double lower_x = 0.0;
    double upper_x = 0.0;
    double lower_y = 0.0;
    double upper_y = 0.0;
    if (!DivideByPositiveInterval(
            raw_x,
            norm_lower,
            norm_upper,
            lower_x,
            upper_x)
        || !DivideByPositiveInterval(
            raw_y,
            norm_lower,
            norm_upper,
            lower_y,
            upper_y))
    {
        return false;
    }
    const double error_x = (std::max)(
        NextUp(std::fabs(stored[0] - lower_x)),
        NextUp(std::fabs(upper_x - stored[0])));
    const double error_y = (std::max)(
        NextUp(std::fabs(stored[1] - lower_y)),
        NextUp(std::fabs(upper_y - stored[1])));
    return AddUp(error_x, error_y, output);
}

SampleBuildOutcome BuildFlightPathSample(
    const DogfightGeometryFrame& frame,
    FlightPathSample& output) noexcept
{
    output = FlightPathSample{};
    double own_position_error = 0.0;
    double target_position_error = 0.0;
    if (!Float32WireVectorErrorBound(
            frame.own.position_ned_m,
            own_position_error)
        || !Float32WireVectorErrorBound(
            frame.opponent.position_ned_m,
            target_position_error))
    {
        return SampleBuildOutcome::Invalid;
    }
    double direction_error = 0.0;
    const DirectionResolutionOutcome direction_outcome =
        AircraftFlightDirectionError(frame.own, direction_error);
    if (direction_outcome == DirectionResolutionOutcome::Unobservable)
    {
        return SampleBuildOutcome::HorizontalCourseUnobservable;
    }
    if (direction_outcome != DirectionResolutionOutcome::Resolved)
    {
        return SampleBuildOutcome::Invalid;
    }
    bool course_observable = false;
    double course_error = 0.0;
    if (!HorizontalCourseError(
            frame.own.velocity_ned_mps,
            direction_error,
            course_observable,
            course_error))
    {
        return SampleBuildOutcome::Invalid;
    }
    if (!course_observable)
    {
        return SampleBuildOutcome::HorizontalCourseUnobservable;
    }
    const double course_norm = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    if (!std::isfinite(course_norm) || course_norm <= 0.0)
    {
        return SampleBuildOutcome::Invalid;
    }
    const Vector3 course{{
        frame.own.velocity_ned_mps[0] / course_norm,
        frame.own.velocity_ned_mps[1] / course_norm,
        0.0}};
    double normalization_error = 0.0;
    if (!Finite(course)
        || !CourseNormalizationError(
            frame.own.velocity_ned_mps[0],
            frame.own.velocity_ned_mps[1],
            course_norm,
            course,
            normalization_error))
    {
        return SampleBuildOutcome::Invalid;
    }

    const Vector3 relative_position = Subtract(
        frame.opponent.position_ned_m,
        frame.own.position_ned_m);
    double subtraction_roundoff = 0.0;
    double combined_position_error = 0.0;
    if (!Binary64VectorRoundoffBound(
            relative_position,
            subtraction_roundoff)
        || !AddUp(
            own_position_error,
            target_position_error,
            combined_position_error)
        || !AddUp(
            combined_position_error,
            subtraction_roundoff,
            combined_position_error))
    {
        return SampleBuildOutcome::Invalid;
    }

    output.own_plane_id = frame.own_plane_id;
    output.target_plane_id = frame.target_plane_id;
    output.episode_epoch = frame.frame_identity.episode_epoch;
    output.sample_index = frame.frame_identity.frame_index;
    output.t_sec = frame.t_sec;
    output.relative_position_ned_m = relative_position;
    output.own_horizontal_course_ned = course;
    output.relative_position_error_bound_m = combined_position_error;
    output.own_course_error_bound_rad = course_error;
    output.own_course_normalization_error_bound_l1 = normalization_error;
    return SampleBuildOutcome::Observed;
}

bool AxisNormDefectBound(const Vector3& axis, double& output) noexcept
{
    output = 0.0;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;
    double squared_lower = 0.0;
    double squared_upper = 0.0;
    if (!ProductInterval(axis[0], axis[0], x_lower, x_upper)
        || !ProductInterval(axis[1], axis[1], y_lower, y_upper)
        || !AddDown(x_lower, y_lower, squared_lower)
        || !AddUp(x_upper, y_upper, squared_upper))
    {
        return false;
    }
    output = (std::max)(
        NextUp(std::fabs(squared_lower - 1.0)),
        NextUp(std::fabs(squared_upper - 1.0)));
    return std::isfinite(output);
}

bool BuildSignedLateralInterval(
    const FlightPathSample& axis_sample,
    const FlightPathSample& position_sample,
    RelativeFlightPathSignedLateralInterval& output) noexcept
{
    output = RelativeFlightPathSignedLateralInterval{};
    const Vector3& axis = axis_sample.own_horizontal_course_ned;
    const Vector3& relative = position_sample.relative_position_ned_m;
    const double product_a = axis[0] * relative[1];
    const double product_b = axis[1] * relative[0];
    const double nominal = product_a - product_b;
    if (!std::isfinite(product_a)
        || !std::isfinite(product_b)
        || !std::isfinite(nominal))
    {
        return false;
    }
    double a_lower = 0.0;
    double a_upper = 0.0;
    double b_lower = 0.0;
    double b_upper = 0.0;
    double determinant_lower = 0.0;
    double determinant_upper = 0.0;
    if (!ProductInterval(axis[0], relative[1], a_lower, a_upper)
        || !ProductInterval(axis[1], relative[0], b_lower, b_upper)
        || !SubtractDown(a_lower, b_upper, determinant_lower)
        || !SubtractUp(a_upper, b_lower, determinant_upper))
    {
        return false;
    }

    double norm_defect = 0.0;
    double normalization_bound = 0.0;
    double axis_vector_bound = 0.0;
    double rho_l1 = 0.0;
    double axis_projection = 0.0;
    double beta = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    if (!AxisNormDefectBound(axis, norm_defect)
        || !MultiplyNonnegativeUp(
            2.0,
            axis_sample.own_course_normalization_error_bound_l1,
            normalization_bound)
        || !AddUp(
            (std::min)(axis_sample.own_course_error_bound_rad, 2.0),
            norm_defect,
            axis_vector_bound)
        || !AddUp(
            axis_vector_bound,
            normalization_bound,
            axis_vector_bound)
        || !AddUp(
            std::fabs(relative[0]),
            std::fabs(relative[1]),
            rho_l1)
        || !MultiplyNonnegativeUp(
            axis_vector_bound,
            rho_l1,
            axis_projection)
        || !AddUp(
            position_sample.relative_position_error_bound_m,
            axis_projection,
            beta)
        || !SubtractDown(determinant_lower, beta, lower)
        || !AddUp(determinant_upper, beta, upper))
    {
        return false;
    }
    output.valid = true;
    output.nominal_m = nominal;
    output.lower_m = lower;
    output.upper_m = upper;
    output.resolved_sign = lower > 0.0 ? 1 : (upper < 0.0 ? -1 : 0);
    return true;
}

bool ValidSameIndexFrameContract(
    const DogfightGeometryFrame& frame,
    StatusCode& failure) noexcept
{
    failure = StatusCode::InvalidArgument;
    if (!std::isfinite(frame.t_sec)
        || !std::isfinite(frame.frame_identity.source_time_s)
        || !Finite(frame.own.position_ned_m)
        || !Finite(frame.opponent.position_ned_m)
        || !Finite(frame.own.velocity_body_mps)
        || !Finite(frame.own.velocity_ned_mps))
    {
        failure = StatusCode::NonFiniteInput;
        return false;
    }
    return LadyLuck::IsValidControlFrameIdentity(frame.frame_identity)
        && frame.own_plane_id >= 0
        && frame.target_plane_id >= 0
        && frame.own_plane_id != frame.target_plane_id
        && frame.target_same_index
        && frame.target_frame_index == frame.frame_identity.frame_index;
}

void PopulateCurrent(
    const FlightPathSample& current,
    RelativeFlightPathCrossingReceipt& output) noexcept
{
    output.current_own_plane_id = current.own_plane_id;
    output.current_target_plane_id = current.target_plane_id;
    output.current_episode_epoch = current.episode_epoch;
    output.current_sample_index = current.sample_index;
    output.current_t_sec = current.t_sec;
}

void PopulatePrevious(
    const FlightPathSample& previous,
    RelativeFlightPathCrossingReceipt& output) noexcept
{
    output.previous_sample_present = true;
    output.previous_own_plane_id = previous.own_plane_id;
    output.previous_target_plane_id = previous.target_plane_id;
    output.previous_episode_epoch = previous.episode_epoch;
    output.previous_sample_index = previous.sample_index;
    output.previous_t_sec = previous.t_sec;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* RelativeFlightPathCrossingReasonLabel(
    const RelativeFlightPathCrossingReason reason) noexcept
{
    switch (reason)
    {
    case RelativeFlightPathCrossingReason::FirstSampleNoHistory:
        return "first_sample_no_history";
    case RelativeFlightPathCrossingReason::TrackLineageMismatch:
        return "track_lineage_mismatch";
    case RelativeFlightPathCrossingReason::SourceEpochMismatch:
        return "source_epoch_mismatch";
    case RelativeFlightPathCrossingReason::
            SampleIndexNotExactlyConsecutive:
        return "sample_index_not_exactly_consecutive";
    case RelativeFlightPathCrossingReason::
            SampleTimeNotStrictlyIncreasing:
        return "sample_time_not_strictly_increasing";
    case RelativeFlightPathCrossingReason::
            DerivedLateralIntervalNotFinite:
        return "derived_lateral_interval_not_finite";
    case RelativeFlightPathCrossingReason::
            LateralSideUnresolvedWithinErrorBounds:
        return "lateral_side_unresolved_within_error_bounds";
    case RelativeFlightPathCrossingReason::FrozenCourseAxesDisagree:
        return "frozen_course_axes_disagree";
    case RelativeFlightPathCrossingReason::NoRelativeFlightPathCrossing:
        return "no_relative_flight_path_crossing";
    case RelativeFlightPathCrossingReason::
            RelativeFlightPathCrossingResolved:
        return "relative_flight_path_crossing_resolved";
    case RelativeFlightPathCrossingReason::
            HorizontalCourseUnobservableWithinBounds:
        return "horizontal_course_unobservable_within_bounds";
    case RelativeFlightPathCrossingReason::CrossingSampleContractRejected:
        return "crossing_sample_contract_rejected";
    default:
        return "unknown";
    }
}

void RelativeFlightPathCrossingObserver::Reset() noexcept
{
    previous_sample_valid_ = false;
    previous_sample_ = HorizontalFlightPathSample{};
}

void RelativeFlightPathCrossingObserver::Observe(
    const DogfightGeometryFrame& frame,
    RelativeFlightPathCrossingReceipt& output,
    Status& status) noexcept
{
    output = RelativeFlightPathCrossingReceipt{};
    status = Status{};

    StatusCode contract_failure = StatusCode::InvalidArgument;
    if (!ValidSameIndexFrameContract(frame, contract_failure))
    {
        Reset();
        output.reason = RelativeFlightPathCrossingReason::
            CrossingSampleContractRejected;
        status.code = contract_failure;
        return;
    }

    HorizontalFlightPathSample current{};
    const SampleBuildOutcome sample_outcome =
        BuildFlightPathSample(frame, current);
    if (sample_outcome == SampleBuildOutcome::HorizontalCourseUnobservable)
    {
        Reset();
        output.reason = RelativeFlightPathCrossingReason::
            HorizontalCourseUnobservableWithinBounds;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    if (sample_outcome != SampleBuildOutcome::Observed)
    {
        Reset();
        output.reason = RelativeFlightPathCrossingReason::
            CrossingSampleContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }

    output.evaluated = true;
    PopulateCurrent(current, output);
    if (!previous_sample_valid_)
    {
        previous_sample_ = current;
        previous_sample_valid_ = true;
        output.reason =
            RelativeFlightPathCrossingReason::FirstSampleNoHistory;
        status.code = StatusCode::Seeded;
        return;
    }

    const HorizontalFlightPathSample previous = previous_sample_;
    PopulatePrevious(previous, output);
    previous_sample_ = current;

    if (current.own_plane_id != previous.own_plane_id
        || current.target_plane_id != previous.target_plane_id)
    {
        output.reason = RelativeFlightPathCrossingReason::
            TrackLineageMismatch;
        status.code = StatusCode::FrameGap;
        return;
    }
    if (current.episode_epoch != previous.episode_epoch)
    {
        output.reason = RelativeFlightPathCrossingReason::
            SourceEpochMismatch;
        status.code = StatusCode::FrameGap;
        return;
    }
    if (previous.sample_index ==
            (std::numeric_limits<std::uint64_t>::max)()
        || current.sample_index != previous.sample_index + 1U)
    {
        output.reason = RelativeFlightPathCrossingReason::
            SampleIndexNotExactlyConsecutive;
        status.code = StatusCode::FrameGap;
        return;
    }
    if (current.t_sec <= previous.t_sec)
    {
        output.reason = RelativeFlightPathCrossingReason::
            SampleTimeNotStrictlyIncreasing;
        status.code = StatusCode::FrameGap;
        return;
    }
    const double sample_dt_s = current.t_sec - previous.t_sec;
    if (!std::isfinite(sample_dt_s) || sample_dt_s <= 0.0)
    {
        output.reason = RelativeFlightPathCrossingReason::
            SampleTimeNotStrictlyIncreasing;
        status.code = StatusCode::FrameGap;
        return;
    }

    output.sample_dt_valid = true;
    output.sample_dt_s = sample_dt_s;
    if (!BuildSignedLateralInterval(
            previous,
            previous,
            output.previous_axis_previous_position)
        || !BuildSignedLateralInterval(
            previous,
            current,
            output.previous_axis_current_position)
        || !BuildSignedLateralInterval(
            current,
            previous,
            output.current_axis_previous_position)
        || !BuildSignedLateralInterval(
            current,
            current,
            output.current_axis_current_position))
    {
        output.sample_dt_valid = false;
        output.sample_dt_s = 0.0;
        output.previous_axis_previous_position =
            RelativeFlightPathSignedLateralInterval{};
        output.previous_axis_current_position =
            RelativeFlightPathSignedLateralInterval{};
        output.current_axis_previous_position =
            RelativeFlightPathSignedLateralInterval{};
        output.current_axis_current_position =
            RelativeFlightPathSignedLateralInterval{};
        output.reason = RelativeFlightPathCrossingReason::
            DerivedLateralIntervalNotFinite;
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    output.geometry_evaluable = true;
    const std::int32_t signs[4] = {
        output.previous_axis_previous_position.resolved_sign,
        output.previous_axis_current_position.resolved_sign,
        output.current_axis_previous_position.resolved_sign,
        output.current_axis_current_position.resolved_sign};
    const bool all_resolved = signs[0] != 0
        && signs[1] != 0
        && signs[2] != 0
        && signs[3] != 0;
    output.endpoint_sides_resolved = all_resolved
        && signs[0] == signs[2]
        && signs[1] == signs[3];
    if (!all_resolved)
    {
        output.reason = RelativeFlightPathCrossingReason::
            LateralSideUnresolvedWithinErrorBounds;
        return;
    }
    if (signs[0] != signs[2] || signs[1] != signs[3])
    {
        output.reason = RelativeFlightPathCrossingReason::
            FrozenCourseAxesDisagree;
        return;
    }
    if (signs[1] == -signs[0])
    {
        output.dual_frozen_axis_crossing_resolved = true;
        output.crossing_event_valid = true;
        output.crossing_event_t_sec = frame.t_sec;
        output.reason = RelativeFlightPathCrossingReason::
            RelativeFlightPathCrossingResolved;
        return;
    }
    output.reason = RelativeFlightPathCrossingReason::
        NoRelativeFlightPathCrossing;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
