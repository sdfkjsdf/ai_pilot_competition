#include "LadyLuck/guidance/obfm/G3SAdversaryCourseReversal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::AdversaryReversalObservation;
using LadyLuck::guidance::obfm::AdversaryReversalObservationReason;
using LadyLuck::guidance::obfm::G3SOptionalDouble;
using LadyLuck::guidance::obfm::G3SReversalScaleRad;

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

struct DoubleLength
{
    double hi = 0.0;
    double lo = 0.0;
};

DoubleLength DoubleLengthFastSum(
    const double left,
    const double right) noexcept
{
    const double sum = left + right;
    return DoubleLength{sum, (left - sum) + right};
}

DoubleLength DoubleLengthMultiply(
    const double left,
    const double right) noexcept
{
    const double product = left * right;
    return DoubleLength{
        product,
        std::fma(left, right, -product)};
}

template <std::size_t CoordinateCount>
double PythonMathHypot(
    std::array<double, CoordinateCount> coordinates) noexcept
{
    double maximum = 0.0;
    for (std::size_t index = 0U; index < CoordinateCount; ++index)
    {
        coordinates[index] = std::fabs(coordinates[index]);
        maximum = (std::max)(maximum, coordinates[index]);
    }
    if (std::isinf(maximum))
    {
        return maximum;
    }
    if (!std::isfinite(maximum))
    {
        return (std::numeric_limits<double>::quiet_NaN)();
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    const double minimum_normal =
        (std::numeric_limits<double>::min)();
    if (maximum_exponent < -1023)
    {
        for (std::size_t index = 0U; index < CoordinateCount; ++index)
        {
            coordinates[index] /= minimum_normal;
        }
        return minimum_normal * PythonMathHypot(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < CoordinateCount; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const DoubleLength product = DoubleLengthMultiply(scaled, scaled);
        const DoubleLength sum = DoubleLengthFastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const DoubleLength negative_square = DoubleLengthMultiply(
        -result,
        result);
    const DoubleLength corrected_sum = DoubleLengthFastSum(
        compensated_sum,
        negative_square.hi);
    compensated_sum = corrected_sum.hi;
    fraction_one += negative_square.lo;
    fraction_two += corrected_sum.lo;
    const double correction =
        compensated_sum - 1.0 + (fraction_one + fraction_two);
    result += correction / (2.0 * result);
    return result / scale;
}

double MathHypot2(const double x, const double y) noexcept
{
    return PythonMathHypot<2U>(std::array<double, 2U>{{x, y}});
}

double MathHypot3(const Vector3& value) noexcept
{
    return PythonMathHypot<3U>(
        std::array<double, 3U>{{value[0], value[1], value[2]}});
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
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
        MathHypot3(residual),
        PositiveInfinity());
    const double cell_norm = std::nextafter(
        MathHypot3(full_cell),
        PositiveInfinity());
    output = std::nextafter(
        residual_norm + cell_norm,
        PositiveInfinity());
    return std::isfinite(output);
}

AdversaryReversalObservation InvalidObservation(
    const AdversaryReversalObservationReason reason) noexcept
{
    AdversaryReversalObservation output{};
    output.reason = reason;
    output.reversal_scale_rad = G3SReversalScaleRad;
    return output;
}

void ClearOptional(G3SOptionalDouble& value) noexcept
{
    value = G3SOptionalDouble{};
}

void SetOptional(
    G3SOptionalDouble& value,
    const double scalar) noexcept
{
    value.has_value = true;
    value.value = scalar;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* AdversaryReversalObservationReasonLabel(
    const AdversaryReversalObservationReason reason) noexcept
{
    switch (reason)
    {
    case AdversaryReversalObservationReason::AdversaryCourseNotResolved:
        return "adversary_course_not_resolved";
    case AdversaryReversalObservationReason::FirstCourseSample:
        return "first_course_sample";
    case AdversaryReversalObservationReason::CourseRotationNotResolved:
        return "course_rotation_not_resolved";
    case AdversaryReversalObservationReason::CourseRotationResolved:
        return "course_rotation_resolved";
    case AdversaryReversalObservationReason::ObserverContractRejected:
        return "g3_s_reversal_observer_contract_rejected";
    case AdversaryReversalObservationReason::FrameStateNotFinite:
    default:
        return "frame_state_not_finite";
    }
}

AdversaryCourseReversalObserver::
    AdversaryCourseReversalObserver() noexcept
{
    Reset();
}

void AdversaryCourseReversalObserver::Reset() noexcept
{
    previous_course_valid_ = false;
    previous_course_x_ = 0.0;
    previous_course_y_ = 0.0;
    previous_course_error_valid_ = false;
    previous_course_error_rad_ = 0.0;
    current_sign_ = 0;
    ClearOptional(current_start_t_);
    ClearOptional(current_last_t_);
    current_net_rad_ = 0.0;
    challenger_sign_ = 0;
    ClearOptional(challenger_start_t_);
    ClearOptional(challenger_last_t_);
    challenger_net_rad_ = 0.0;
    previous_sign_ = 0;
    ClearOptional(previous_duration_s_);
    ClearOptional(previous_start_t_);
}

G3SOptionalDouble
AdversaryCourseReversalObserver::AlternationEpisodeStartT() const noexcept
{
    return previous_start_t_.has_value
        ? previous_start_t_
        : current_start_t_;
}

void AdversaryCourseReversalObserver::BuildObservation(
    const bool valid,
    const AdversaryReversalObservationReason reason,
    const double now_t,
    AdversaryReversalObservation& output) const noexcept
{
    bool reversal = false;
    if (previous_sign_ != 0
        && current_sign_ != 0
        && previous_sign_ != current_sign_
        && previous_duration_s_.has_value
        && current_net_rad_ >= G3SReversalScaleRad)
    {
        const double current_duration =
            current_last_t_.value - current_start_t_.value;
        const double rhythm_scale = previous_duration_s_.value;
        reversal = current_duration <= rhythm_scale
            && now_t - current_last_t_.value <= rhythm_scale;
    }

    output = AdversaryReversalObservation{};
    output.valid = valid;
    output.reason = reason;
    output.reversal_current = reversal;
    output.current_sign = current_sign_;
    output.current_run_start_t = current_start_t_;
    output.current_run_last_t = current_last_t_;
    output.current_run_net_rad = current_net_rad_;
    output.previous_run_sign = previous_sign_;
    output.previous_run_duration_s = previous_duration_s_;
    output.reversal_scale_rad = G3SReversalScaleRad;
}

void AdversaryCourseReversalObserver::Update(
    const DogfightGeometryFrame& frame,
    AdversaryReversalObservation& output,
    Status& status) noexcept
{
    output = AdversaryReversalObservation{};
    status = Status{};
    const double t_sec = frame.t_sec;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!std::isfinite(t_sec) || !FiniteVector(adversary_velocity))
    {
        output = InvalidObservation(
            AdversaryReversalObservationReason::FrameStateNotFinite);
        return;
    }

    double velocity_bound = 0.0;
    if (!Float32WireVectorErrorBound(
            adversary_velocity,
            velocity_bound))
    {
        Reset();
        output = InvalidObservation(
            AdversaryReversalObservationReason::ObserverContractRejected);
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const double horizontal = MathHypot2(
        adversary_velocity[0],
        adversary_velocity[1]);
    if (!(horizontal - velocity_bound > 0.0))
    {
        BuildObservation(
            true,
            AdversaryReversalObservationReason::
                AdversaryCourseNotResolved,
            t_sec,
            output);
        return;
    }

    const double course_x = adversary_velocity[0] / horizontal;
    const double course_y = adversary_velocity[1] / horizontal;
    const double course_error_rad =
        velocity_bound / (horizontal - velocity_bound);
    const bool had_previous_course = previous_course_valid_;
    const double previous_course_x = previous_course_x_;
    const double previous_course_y = previous_course_y_;
    const double previous_error = previous_course_error_rad_;
    previous_course_valid_ = true;
    previous_course_x_ = course_x;
    previous_course_y_ = course_y;
    previous_course_error_valid_ = true;
    previous_course_error_rad_ = course_error_rad;
    if (!had_previous_course)
    {
        BuildObservation(
            true,
            AdversaryReversalObservationReason::FirstCourseSample,
            t_sec,
            output);
        return;
    }

    const double cross_z =
        previous_course_x * course_y - previous_course_y * course_x;
    const double dot =
        previous_course_x * course_x + previous_course_y * course_y;
    const double delta_rad = std::atan2(cross_z, dot);
    if (!(std::fabs(delta_rad)
            > course_error_rad + previous_error))
    {
        BuildObservation(
            true,
            AdversaryReversalObservationReason::
                CourseRotationNotResolved,
            t_sec,
            output);
        return;
    }

    const std::int32_t sign = delta_rad > 0.0 ? 1 : -1;
    if (current_sign_ == 0)
    {
        current_sign_ = sign;
        SetOptional(current_start_t_, t_sec);
        SetOptional(current_last_t_, t_sec);
        current_net_rad_ = std::fabs(delta_rad);
    }
    else if (sign == current_sign_)
    {
        challenger_sign_ = 0;
        ClearOptional(challenger_start_t_);
        ClearOptional(challenger_last_t_);
        challenger_net_rad_ = 0.0;
        SetOptional(current_last_t_, t_sec);
        current_net_rad_ += std::fabs(delta_rad);
    }
    else
    {
        if (challenger_sign_ == sign)
        {
            SetOptional(challenger_last_t_, t_sec);
            challenger_net_rad_ += std::fabs(delta_rad);
        }
        else
        {
            challenger_sign_ = sign;
            SetOptional(challenger_start_t_, t_sec);
            SetOptional(challenger_last_t_, t_sec);
            challenger_net_rad_ = std::fabs(delta_rad);
        }
        if (challenger_net_rad_ >= G3SReversalScaleRad)
        {
            if (current_net_rad_ >= G3SReversalScaleRad)
            {
                previous_sign_ = current_sign_;
                previous_start_t_ = current_start_t_;
                SetOptional(
                    previous_duration_s_,
                    current_last_t_.value - current_start_t_.value);
            }
            current_sign_ = challenger_sign_;
            current_start_t_ = challenger_start_t_;
            current_last_t_ = challenger_last_t_;
            current_net_rad_ = challenger_net_rad_;
            challenger_sign_ = 0;
            ClearOptional(challenger_start_t_);
            ClearOptional(challenger_last_t_);
            challenger_net_rad_ = 0.0;
        }
    }

    BuildObservation(
        true,
        AdversaryReversalObservationReason::CourseRotationResolved,
        t_sec,
        output);
}

bool ScissorsSituationResolved(
    const AdversaryReversalObservation* const observation) noexcept
{
    return observation != nullptr
        && observation->valid
        && observation->reversal_current;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
