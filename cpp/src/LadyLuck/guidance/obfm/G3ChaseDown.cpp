#include "LadyLuck/guidance/obfm/G3ChaseDown.hpp"

#include "LadyLuck/common/CompensatedDouble.hpp"
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
using LadyLuck::common::CompensatedDouble;
using LadyLuck::common::ExactProduct;
using LadyLuck::common::FastSum;
using LadyLuck::guidance::obfm::G3ChaseDownObservation;
using LadyLuck::guidance::obfm::G3ChaseDownObservationReason;
using LadyLuck::guidance::obfm::G3ChaseDownPursuitBehavior;
using LadyLuck::guidance::obfm::G3ChaseDownSelectionReason;
using LadyLuck::guidance::obfm::G3ChaseDownSelectionReceipt;

constexpr double kFloat32Epsilon =
    static_cast<double>(std::numeric_limits<float>::epsilon());

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

double Dot3NumpyAssociation(
    const Vector3& left,
    const Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3NumpyAssociation(value, value));
}

// Allocation-free n=3 specialization of CPython 3.12 vector_norm(), the
// implementation behind d90's three-argument math.hypot calls.
double MathHypot3(const Vector3& value) noexcept
{
    Vector3 coordinates{{
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])}};
    const double maximum = (std::max)(
        coordinates[0],
        (std::max)(coordinates[1], coordinates[2]));
    if (std::isinf(maximum))
    {
        return maximum;
    }
    if (!FiniteVector(coordinates))
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
        coordinates[0] /= minimum_normal;
        coordinates[1] /= minimum_normal;
        coordinates[2] /= minimum_normal;
        return minimum_normal * MathHypot3(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const CompensatedDouble product = ExactProduct(scaled, scaled);
        const CompensatedDouble sum = FastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const CompensatedDouble negative_square = ExactProduct(
        -result,
        result);
    const CompensatedDouble corrected_sum = FastSum(
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

bool DirectionConeHalfAngleRad(
    const Vector3& vector,
    const double wire_bound_m,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    const double norm = NumpyNorm3(vector);
    if (!std::isfinite(norm)
        || !std::isfinite(wire_bound_m)
        || wire_bound_m < 0.0)
    {
        return false;
    }
    if (norm <= wire_bound_m)
    {
        return true;
    }
    const double ratio = wire_bound_m / (norm - wire_bound_m);
    output = ratio >= 1.0
        ? std::nextafter(0.5 * LadyLuck::constants::Pi, 0.0)
        : std::nextafter(std::asin(ratio), PositiveInfinity());
    observable = std::isfinite(output);
    return observable;
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
        MathHypot3(direction),
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

double WrappedDeltaRad(
    const double current,
    const double previous) noexcept
{
    double delta = current - previous;
    while (delta > LadyLuck::constants::Pi)
    {
        delta -= 2.0 * LadyLuck::constants::Pi;
    }
    while (delta <= -LadyLuck::constants::Pi)
    {
        delta += 2.0 * LadyLuck::constants::Pi;
    }
    return delta;
}

G3ChaseDownObservation Unobserved(
    const G3ChaseDownObservationReason reason) noexcept
{
    G3ChaseDownObservation output{};
    output.reason = reason;
    return output;
}

G3ChaseDownObservation Observed(
    const G3ChaseDownObservationReason reason,
    const bool admitted,
    const std::int32_t turn_sign,
    const bool descent_resolved,
    const bool own_faster_resolved,
    const bool own_above_resolved) noexcept
{
    G3ChaseDownObservation output{};
    output.valid = true;
    output.reason = reason;
    output.admitted = admitted;
    output.turn_sign = turn_sign;
    output.descent_resolved = descent_resolved;
    output.own_faster_resolved = own_faster_resolved;
    output.own_above_resolved = own_above_resolved;
    return output;
}

bool PursuitBehaviorAdmitted(
    const G3ChaseDownPursuitBehavior behavior) noexcept
{
    return behavior == G3ChaseDownPursuitBehavior::Lag
        || behavior == G3ChaseDownPursuitBehavior::Employ;
}

void RejectSelection(
    G3ChaseDownSelectionReceipt& output,
    const G3ChaseDownSelectionReason reason) noexcept
{
    output = G3ChaseDownSelectionReceipt{};
    output.reason = reason;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* G3ChaseDownObservationReasonLabel(
    const G3ChaseDownObservationReason reason) noexcept
{
    switch (reason)
    {
    case G3ChaseDownObservationReason::AdversaryDirectionNotResolved:
        return "adversary_direction_not_resolved";
    case G3ChaseDownObservationReason::AdversaryCourseNotResolved:
        return "adversary_course_not_resolved";
    case G3ChaseDownObservationReason::InsufficientCausalWindow:
        return "insufficient_causal_window";
    case G3ChaseDownObservationReason::TurnNotResolved:
        return "turn_not_resolved";
    case G3ChaseDownObservationReason::TurnSignNotSustained:
        return "turn_sign_not_sustained";
    case G3ChaseDownObservationReason::DescentNotResolved:
        return "descent_not_resolved";
    case G3ChaseDownObservationReason::OwnAboveNotResolved:
        return "own_above_not_resolved";
    case G3ChaseDownObservationReason::
            SustainedDivingTurnBelowCrossingPredicted:
        return "sustained_diving_turn_below_crossing_predicted";
    case G3ChaseDownObservationReason::ObserverContractRejected:
        return "g3_chase_down_observer_contract_rejected";
    case G3ChaseDownObservationReason::StateNotObservable:
    default:
        return "state_not_observable";
    }
}

const char* G3ChaseDownSelectionReasonLabel(
    const G3ChaseDownSelectionReason reason) noexcept
{
    switch (reason)
    {
    case G3ChaseDownSelectionReason::BehaviorOutsidePursuitFamily:
        return "behavior_outside_pursuit_family";
    case G3ChaseDownSelectionReason::StateOrAimNotFinite:
        return "state_or_aim_not_finite";
    case G3ChaseDownSelectionReason::AimAlreadyAtOrAboveOwnAltitude:
        return "aim_already_at_or_above_own_altitude";
    case G3ChaseDownSelectionReason::OwnAltitudeFloorSelected:
        return "own_altitude_floor_selected";
    case G3ChaseDownSelectionReason::SelectionContractRejected:
        return "g3_chase_down_selection_contract_rejected";
    case G3ChaseDownSelectionReason::ObservationMissingOrUnadmitted:
    default:
        return "observation_missing_or_unadmitted";
    }
}

G3ChaseDownObserver::G3ChaseDownObserver() noexcept
{
    Reset();
}

void G3ChaseDownObserver::Reset() noexcept
{
    window_ = std::array<CourseSample, 3U>{};
    window_count_ = 0U;
}

void G3ChaseDownObserver::Update(
    const DogfightGeometryFrame& frame,
    G3ChaseDownObservation& output,
    Status& status) noexcept
{
    output = G3ChaseDownObservation{};
    status = Status{};
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity)
        || !FiniteVector(own_position)
        || !FiniteVector(own_velocity))
    {
        Reset();
        output = Unobserved(G3ChaseDownObservationReason::StateNotObservable);
        return;
    }

    double adversary_velocity_bound = 0.0;
    double own_velocity_bound = 0.0;
    double adversary_position_bound = 0.0;
    double own_position_bound = 0.0;
    if (!Float32WireVectorErrorBound(
            adversary_velocity,
            adversary_velocity_bound)
        || !Float32WireVectorErrorBound(own_velocity, own_velocity_bound)
        || !Float32WireVectorErrorBound(
            adversary_position,
            adversary_position_bound)
        || !Float32WireVectorErrorBound(own_position, own_position_bound))
    {
        Reset();
        output.reason =
            G3ChaseDownObservationReason::ObserverContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }

    bool direction_observable = false;
    double direction_cone_rad = 0.0;
    if (!DirectionConeHalfAngleRad(
            adversary_velocity,
            adversary_velocity_bound,
            direction_observable,
            direction_cone_rad))
    {
        Reset();
        output.reason =
            G3ChaseDownObservationReason::ObserverContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!direction_observable)
    {
        Reset();
        output = Unobserved(
            G3ChaseDownObservationReason::AdversaryDirectionNotResolved);
        return;
    }

    bool course_observable = false;
    double course_bound_rad = 0.0;
    if (!HorizontalCourseErrorBoundRad(
            adversary_velocity,
            direction_cone_rad,
            course_observable,
            course_bound_rad))
    {
        Reset();
        output.reason =
            G3ChaseDownObservationReason::ObserverContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!course_observable)
    {
        Reset();
        output = Unobserved(
            G3ChaseDownObservationReason::AdversaryCourseNotResolved);
        return;
    }

    const bool descent_resolved =
        adversary_velocity[2] - adversary_velocity_bound > 0.0;
    const CourseSample current_sample{
        std::atan2(adversary_velocity[1], adversary_velocity[0]),
        course_bound_rad,
        descent_resolved};
    if (window_count_ < 3U)
    {
        window_[window_count_++] = current_sample;
    }
    else
    {
        window_[0] = window_[1];
        window_[1] = window_[2];
        window_[2] = current_sample;
    }

    const double own_speed = NumpyNorm3(own_velocity);
    const double adversary_speed = NumpyNorm3(adversary_velocity);
    const bool own_faster = own_speed - own_velocity_bound
        > adversary_speed + adversary_velocity_bound;
    const double own_altitude = -own_position[2];
    const double adversary_altitude = -adversary_position[2];
    const bool own_above = own_altitude - own_position_bound
        > adversary_altitude + adversary_position_bound;

    if (window_count_ < 3U)
    {
        output = Observed(
            G3ChaseDownObservationReason::InsufficientCausalWindow,
            false,
            0,
            descent_resolved,
            own_faster,
            own_above);
        return;
    }

    const double first_delta = WrappedDeltaRad(
        window_[1].course_rad,
        window_[0].course_rad);
    const double second_delta = WrappedDeltaRad(
        window_[2].course_rad,
        window_[1].course_rad);
    const double first_bound = window_[0].course_bound_rad
        + window_[1].course_bound_rad;
    const double second_bound = window_[1].course_bound_rad
        + window_[2].course_bound_rad;
    const std::int32_t first_sign = std::fabs(first_delta) <= first_bound
        ? 0
        : (first_delta > 0.0 ? 1 : -1);
    const std::int32_t second_sign = std::fabs(second_delta) <= second_bound
        ? 0
        : (second_delta > 0.0 ? 1 : -1);
    if (first_sign == 0 || second_sign == 0)
    {
        output = Observed(
            G3ChaseDownObservationReason::TurnNotResolved,
            false,
            0,
            descent_resolved,
            own_faster,
            own_above);
        return;
    }
    if (first_sign != second_sign)
    {
        output = Observed(
            G3ChaseDownObservationReason::TurnSignNotSustained,
            false,
            0,
            descent_resolved,
            own_faster,
            own_above);
        return;
    }
    const std::int32_t turn_sign = first_sign;
    if (!window_[0].descent_resolved
        || !window_[1].descent_resolved
        || !window_[2].descent_resolved)
    {
        output = Observed(
            G3ChaseDownObservationReason::DescentNotResolved,
            false,
            turn_sign,
            descent_resolved,
            own_faster,
            own_above);
        return;
    }
    if (!own_above)
    {
        output = Observed(
            G3ChaseDownObservationReason::OwnAboveNotResolved,
            false,
            turn_sign,
            descent_resolved,
            own_faster,
            own_above);
        return;
    }
    output = Observed(
        G3ChaseDownObservationReason::
            SustainedDivingTurnBelowCrossingPredicted,
        true,
        turn_sign,
        true,
        own_faster,
        true);
}

void EvaluateG3ChaseDown(
    const DogfightGeometryFrame& frame,
    const G3ChaseDownPursuitBehavior upstream_behavior,
    const Vector3& upstream_aim_point_m,
    const G3ChaseDownObservation* const observation,
    G3ChaseDownSelectionReceipt& output,
    Status& status) noexcept
{
    output = G3ChaseDownSelectionReceipt{};
    status = Status{};
    if (observation == nullptr || !observation->admitted)
    {
        RejectSelection(
            output,
            G3ChaseDownSelectionReason::ObservationMissingOrUnadmitted);
        return;
    }
    if (!PursuitBehaviorAdmitted(upstream_behavior))
    {
        RejectSelection(
            output,
            G3ChaseDownSelectionReason::BehaviorOutsidePursuitFamily);
        return;
    }
    const Vector3& own_position = frame.own.position_ned_m;
    if (!FiniteVector(own_position) || !FiniteVector(upstream_aim_point_m))
    {
        RejectSelection(
            output,
            G3ChaseDownSelectionReason::StateOrAimNotFinite);
        return;
    }

    const double own_altitude_m = -own_position[2];
    const double aim_altitude_m = -upstream_aim_point_m[2];
    const double band_m = (std::max)(
        (std::max)(std::fabs(aim_altitude_m), std::fabs(own_altitude_m)),
        1.0)
        * kFloat32Epsilon;
    if (!(aim_altitude_m < own_altitude_m - band_m))
    {
        RejectSelection(
            output,
            G3ChaseDownSelectionReason::AimAlreadyAtOrAboveOwnAltitude);
        return;
    }

    output.selected = true;
    output.reason = G3ChaseDownSelectionReason::OwnAltitudeFloorSelected;
    output.overlay.valid = true;
    output.overlay.aim_point_m = Vector3{{
        upstream_aim_point_m[0],
        upstream_aim_point_m[1],
        -own_altitude_m}};
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
