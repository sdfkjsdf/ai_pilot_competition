#include "LadyLuck/guidance/g10/G10SecondUseLagReacquisitionProvider.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::AircraftGeometryKinematics;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g10::G10OptionalDouble;
using LadyLuck::guidance::g10::G10OptionalInterval;
using LadyLuck::guidance::g10::G10PursuitClassificationReason;
using LadyLuck::guidance::g10::G10PursuitClassificationReceipt;
using LadyLuck::guidance::g10::G10PursuitState;
using LadyLuck::guidance::obfm::PursuitPathPlaneGate;
using LadyLuck::guidance::obfm::RollingScissorsPlaneRelation;

constexpr double BattleServerRpyQuantumRad =
    LadyLuck::constants::Pi / 180000.0;
constexpr double BattleServerBodyVelocityQuantumMps = 0.0003048;

enum class Disposition : std::uint8_t
{
    Valid = 0U,
    PhysicalNonAdmission = 1U,
    Invalid = 2U
};

struct PreparedDirection
{
    Disposition disposition = Disposition::Invalid;
    Vector3 direction_ned{};
    double direction_bound_rad = 0.0;
    StatusCode fault = StatusCode::InvalidArgument;
};

struct Chord
{
    bool valid = false;
    double duration_s = 0.0;
    double rotation_rad = 0.0;
    double rotation_bound_rad = 0.0;
    Vector3 normal_ned{};
    double normal_bound_rad = 0.0;
};

struct PathPlane
{
    bool valid = false;
    PursuitPathPlaneGate gate = PursuitPathPlaneGate::Unavailable;
    Vector3 normal_ned{};
    Vector3 lift_ned{};
    double normal_bound_rad = 0.0;
    double lift_bound_rad = 0.0;
    double recent_duration_s = 0.0;
    double recent_rotation_rad = 0.0;
    double recent_rotation_bound_rad = 0.0;
};

struct AttitudePlane
{
    bool valid = false;
    Vector3 flight_ned{};
    Vector3 lift_ned{};
    Vector3 normal_ned{};
    double flight_bound_rad = 0.0;
    double lift_bound_rad = 0.0;
    double normal_bound_rad = 0.0;
};

struct Interval
{
    double lower = 0.0;
    double upper = 0.0;
};

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    // NumPy's three-element inner-loop association on the d90 authority.
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

Vector3 Add3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

Vector3 Sub3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale3(const Vector3& value, const double scale) noexcept
{
    return Vector3{{value[0] * scale, value[1] * scale, value[2] * scale}};
}

Vector3 Cross3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

bool Unit3(const Vector3& value, Vector3& output) noexcept
{
    output = Vector3{};
    if (!FiniteVector(value))
    {
        return false;
    }
    const double magnitude = Norm3(value);
    if (!std::isfinite(magnitude) || magnitude <= 0.0)
    {
        return false;
    }
    output = Scale3(value, 1.0 / magnitude);
    return FiniteVector(output);
}

double Clamp(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return (std::max)(lower, (std::min)(upper, value));
}

double PythonUlp(const double value) noexcept
{
    const double magnitude = std::fabs(value);
    return std::nextafter(
        magnitude,
        (std::numeric_limits<double>::infinity)()) - magnitude;
}

void SetOptional(G10OptionalDouble& output, const double value) noexcept
{
    output.has_value = true;
    output.value = value;
}

void SetOptional(G10OptionalInterval& output, const Interval& value) noexcept
{
    output.has_value = true;
    output.lower = value.lower;
    output.upper = value.upper;
}

Interval Product(const Interval& first, const Interval& second) noexcept
{
    const double values[4U] = {
        first.lower * second.lower,
        first.lower * second.upper,
        first.upper * second.lower,
        first.upper * second.upper};
    return Interval{
        *(std::min_element)(values, values + 4U),
        *(std::max_element)(values, values + 4U)};
}

Interval Add(const Interval& first, const Interval& second) noexcept
{
    return Interval{
        first.lower + second.lower,
        first.upper + second.upper};
}

Interval Subtract(const Interval& first, const Interval& second) noexcept
{
    return Interval{
        first.lower - second.upper,
        first.upper - second.lower};
}

bool Divide(
    const Interval& numerator,
    const Interval& denominator,
    Interval& output) noexcept
{
    output = Interval{};
    if (denominator.lower <= 0.0 && denominator.upper >= 0.0)
    {
        return false;
    }
    const double values[4U] = {
        numerator.lower / denominator.lower,
        numerator.lower / denominator.upper,
        numerator.upper / denominator.lower,
        numerator.upper / denominator.upper};
    output = Interval{
        *(std::min_element)(values, values + 4U),
        *(std::max_element)(values, values + 4U)};
    return true;
}

bool Float32HalfCell(
    const Vector3& value,
    Vector3& represented,
    Vector3& half_cell) noexcept
{
    represented = Vector3{};
    half_cell = Vector3{};
    if (!FiniteVector(value))
    {
        return false;
    }
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float wire = static_cast<float>(value[index]);
        if (!std::isfinite(wire))
        {
            return false;
        }
        const float upper = std::nextafter(
            wire,
            (std::numeric_limits<float>::infinity)());
        const float lower = std::nextafter(
            wire,
            -(std::numeric_limits<float>::infinity)());
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        represented[index] = static_cast<double>(wire);
        half_cell[index] = 0.5 * (std::max)(
            std::fabs(static_cast<double>(upper) - represented[index]),
            std::fabs(represented[index] - static_cast<double>(lower)));
    }
    return true;
}

bool BodyVelocityError(
    const Vector3& velocity_body_mps,
    double& speed_mps,
    double& error_mps) noexcept
{
    speed_mps = 0.0;
    error_mps = 0.0;
    Vector3 represented{};
    Vector3 half_cell{};
    if (!Float32HalfCell(velocity_body_mps, represented, half_cell))
    {
        return false;
    }
    speed_mps = Norm3(velocity_body_mps);
    error_mps = std::sqrt(3.0) * BattleServerBodyVelocityQuantumMps
        + Norm3(half_cell);
    return std::isfinite(speed_mps)
        && std::isfinite(error_mps)
        && error_mps >= 0.0;
}

Disposition AxisDirectionBound(
    const AircraftGeometryKinematics& aircraft,
    double& output,
    StatusCode& fault) noexcept
{
    output = 0.0;
    fault = StatusCode::InvalidArgument;
    double speed = 0.0;
    double error = 0.0;
    if (!BodyVelocityError(aircraft.velocity_body_mps, speed, error))
    {
        fault = StatusCode::NonFiniteInput;
        return Disposition::Invalid;
    }
    const double true_speed_lower = speed - error;
    if (true_speed_lower <= 0.0)
    {
        fault = StatusCode::Ok;
        return Disposition::PhysicalNonAdmission;
    }
    output = std::asin((std::min)(1.0, error / true_speed_lower))
        + 3.0 * BattleServerRpyQuantumRad;
    if (!std::isfinite(output) || output < 0.0)
    {
        fault = StatusCode::InvalidArgument;
        return Disposition::Invalid;
    }
    fault = StatusCode::Ok;
    return Disposition::Valid;
}

bool CrossDirectionBound(
    const Vector3& first,
    const Vector3& second,
    const double first_bound,
    const double second_bound,
    double& output) noexcept
{
    output = 0.0;
    const double cross_magnitude = Norm3(Cross3(first, second));
    const double chord_error =
        2.0 * std::sin(first_bound / 2.0)
        + 2.0 * std::sin(second_bound / 2.0);
    const double lower_magnitude = cross_magnitude - chord_error;
    if (lower_magnitude <= 0.0)
    {
        return false;
    }
    output = 2.0 * std::asin((std::min)(
        1.0,
        chord_error / lower_magnitude));
    return std::isfinite(output);
}

bool Float32VectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    Vector3 represented{};
    Vector3 half_cell{};
    if (!Float32HalfCell(value, represented, half_cell))
    {
        return false;
    }
    output = Norm3(Sub3(value, represented)) + Norm3(half_cell);
    return std::isfinite(output) && output >= 0.0;
}

void ResetPath(
    std::array<double, 3U>& times,
    std::array<Vector3, 3U>& directions,
    std::array<double, 3U>& bounds,
    std::size_t& count) noexcept
{
    times = std::array<double, 3U>{};
    directions = std::array<Vector3, 3U>{};
    bounds = std::array<double, 3U>{};
    count = 0U;
}

void PrepareDirectionSample(
    const AircraftGeometryKinematics& aircraft,
    PreparedDirection& output) noexcept
{
    output = PreparedDirection{};
    if (!FiniteVector(aircraft.velocity_ned_mps))
    {
        output.fault = StatusCode::NonFiniteInput;
        return;
    }
    if (!Unit3(aircraft.velocity_ned_mps, output.direction_ned))
    {
        output.disposition = Disposition::PhysicalNonAdmission;
        output.fault = StatusCode::Ok;
        return;
    }
    output.disposition = AxisDirectionBound(
        aircraft,
        output.direction_bound_rad,
        output.fault);
}

void BuildChord(
    const double start_time,
    const Vector3& start_direction,
    const double start_bound,
    const double end_time,
    const Vector3& end_direction,
    const double end_bound,
    Chord& output) noexcept
{
    output = Chord{};
    const Vector3 cross = Cross3(start_direction, end_direction);
    const double sine = Norm3(cross);
    const double cosine = Clamp(Dot3(start_direction, end_direction), -1.0, 1.0);
    const double rotation = std::atan2(sine, cosine);
    const double endpoint_error = start_bound + end_bound;
    const double duration = end_time - start_time;
    if (duration <= 0.0
        || sine <= 0.0
        || rotation <= 2.0 * endpoint_error)
    {
        return;
    }
    const double delta_cross = (std::min)(2.0, endpoint_error);
    const double true_cross_lower = std::sin(rotation) - delta_cross;
    if (true_cross_lower <= 0.0)
    {
        return;
    }
    const double axis_error = std::asin((std::min)(
        1.0,
        delta_cross / true_cross_lower));
    if (!std::isfinite(axis_error))
    {
        return;
    }
    output.valid = true;
    output.duration_s = duration;
    output.rotation_rad = rotation;
    output.rotation_bound_rad = endpoint_error;
    output.normal_ned = Scale3(cross, 1.0 / sine);
    output.normal_bound_rad = axis_error;
}

void UpdatePath(
    std::array<double, 3U>& times,
    std::array<Vector3, 3U>& directions,
    std::array<double, 3U>& bounds,
    std::size_t& count,
    const PreparedDirection& prepared,
    const double sample_time,
    const double sample_dt,
    PathPlane& output) noexcept
{
    output = PathPlane{};
    if (prepared.disposition == Disposition::PhysicalNonAdmission)
    {
        ResetPath(times, directions, bounds, count);
        output.gate = PursuitPathPlaneGate::PathSampleNotObservable;
        return;
    }
    if (count > 0U)
    {
        const double previous_time = times[count - 1U];
        const double observed_dt = sample_time - previous_time;
        const double tolerance = 8.0 * (
            PythonUlp(sample_time)
            + PythonUlp(previous_time)
            + PythonUlp(sample_dt));
        if (observed_dt <= 0.0
            || std::fabs(observed_dt - sample_dt) > tolerance)
        {
            ResetPath(times, directions, bounds, count);
            times[0U] = sample_time;
            directions[0U] = prepared.direction_ned;
            bounds[0U] = prepared.direction_bound_rad;
            count = 1U;
            output.gate = PursuitPathPlaneGate::
                PathSampleTimeLineageDiscontinuous;
            return;
        }
    }
    if (count < 3U)
    {
        times[count] = sample_time;
        directions[count] = prepared.direction_ned;
        bounds[count] = prepared.direction_bound_rad;
        ++count;
    }
    else
    {
        times[0U] = times[1U];
        times[1U] = times[2U];
        times[2U] = sample_time;
        directions[0U] = directions[1U];
        directions[1U] = directions[2U];
        directions[2U] = prepared.direction_ned;
        bounds[0U] = bounds[1U];
        bounds[1U] = bounds[2U];
        bounds[2U] = prepared.direction_bound_rad;
    }
    if (count < 3U)
    {
        output.gate = PursuitPathPlaneGate::TwoTurnChordsNotInitialized;
        return;
    }
    Chord older{};
    Chord recent{};
    BuildChord(
        times[0U], directions[0U], bounds[0U],
        times[1U], directions[1U], bounds[1U], older);
    BuildChord(
        times[1U], directions[1U], bounds[1U],
        times[2U], directions[2U], bounds[2U], recent);
    if (!older.valid || !recent.valid)
    {
        output.gate = PursuitPathPlaneGate::
            TwoTurnChordsNotResolvedOutsideDirectionError;
        return;
    }
    const double separation = std::acos(Clamp(
        std::fabs(Dot3(older.normal_ned, recent.normal_ned)),
        0.0,
        1.0));
    const double separation_bound = (std::min)(
        0.5 * LadyLuck::constants::Pi,
        older.normal_bound_rad + recent.normal_bound_rad);
    if (separation > separation_bound)
    {
        output.gate = PursuitPathPlaneGate::ConsecutiveTurnPlaneConesDisjoint;
        return;
    }
    Vector3 lift{};
    double lift_bound = 0.0;
    if (!Unit3(Cross3(recent.normal_ned, prepared.direction_ned), lift)
        || !CrossDirectionBound(
            recent.normal_ned,
            prepared.direction_ned,
            recent.normal_bound_rad,
            prepared.direction_bound_rad,
            lift_bound))
    {
        output.gate = PursuitPathPlaneGate::PathPlaneLiftAxisNotObservable;
        return;
    }
    output.valid = true;
    output.gate = PursuitPathPlaneGate::TwoIntervalPathPlaneEstablished;
    output.normal_ned = recent.normal_ned;
    output.lift_ned = lift;
    output.normal_bound_rad = recent.normal_bound_rad;
    output.lift_bound_rad = lift_bound;
    output.recent_duration_s = recent.duration_s;
    output.recent_rotation_rad = recent.rotation_rad;
    output.recent_rotation_bound_rad = recent.rotation_bound_rad;
}

Disposition AttitudePlaneProxy(
    const AircraftGeometryKinematics& aircraft,
    AttitudePlane& output,
    StatusCode& fault) noexcept
{
    output = AttitudePlane{};
    fault = StatusCode::InvalidArgument;
    if (!FiniteVector(aircraft.velocity_ned_mps)
        || !FiniteVector(aircraft.down_ned))
    {
        fault = StatusCode::NonFiniteInput;
        return Disposition::Invalid;
    }
    Vector3 flight{};
    Vector3 body_up{};
    if (!Unit3(aircraft.velocity_ned_mps, flight)
        || !Unit3(Scale3(aircraft.down_ned, -1.0), body_up))
    {
        fault = StatusCode::Ok;
        return Disposition::PhysicalNonAdmission;
    }
    double flight_bound = 0.0;
    const Disposition direction = AxisDirectionBound(
        aircraft,
        flight_bound,
        fault);
    if (direction != Disposition::Valid)
    {
        return direction;
    }
    const double body_up_bound = 3.0 * BattleServerRpyQuantumRad;
    Vector3 normal{};
    double normal_bound = 0.0;
    if (!Unit3(Cross3(flight, body_up), normal)
        || !CrossDirectionBound(
            flight,
            body_up,
            flight_bound,
            body_up_bound,
            normal_bound))
    {
        fault = StatusCode::Ok;
        return Disposition::PhysicalNonAdmission;
    }
    Vector3 lift{};
    if (!Unit3(Cross3(normal, flight), lift))
    {
        fault = StatusCode::Ok;
        return Disposition::PhysicalNonAdmission;
    }
    if (Dot3(lift, body_up) < 0.0)
    {
        lift = Scale3(lift, -1.0);
        normal = Scale3(normal, -1.0);
    }
    output.valid = true;
    output.flight_ned = flight;
    output.lift_ned = lift;
    output.normal_ned = normal;
    output.flight_bound_rad = flight_bound;
    output.lift_bound_rad = (std::min)(
        LadyLuck::constants::Pi,
        normal_bound + flight_bound);
    output.normal_bound_rad = normal_bound;
    fault = StatusCode::Ok;
    return Disposition::Valid;
}

G10PursuitClassificationReceipt Unobserved(
    const G10PursuitClassificationReason reason) noexcept
{
    G10PursuitClassificationReceipt output{};
    output.reason = reason;
    return output;
}

void FillCommonClassification(
    G10PursuitClassificationReceipt& output,
    const double target_offset,
    const Interval& target_interval,
    const double forward_distance,
    const Interval& forward_interval,
    const double locality_bound,
    const double sampling_resolution) noexcept
{
    SetOptional(output.target_path_offset_m, target_offset);
    SetOptional(output.target_path_offset_interval_m, target_interval);
    SetOptional(output.forward_distance_m, forward_distance);
    SetOptional(output.forward_distance_interval_m, forward_interval);
    SetOptional(output.target_path_locality_bound_m, locality_bound);
    SetOptional(output.pure_sampling_resolution_m, sampling_resolution);
}

void ClassifyNoseRay(
    const Vector3& own_position,
    const Vector3& own_nose,
    const double nose_bound,
    const Vector3& target_position,
    const Vector3& target_velocity,
    const double target_bound,
    const double sample_dt,
    const double locality_bound,
    const double position_bound,
    G10PursuitClassificationReceipt& output,
    StatusCode& fault) noexcept
{
    output = G10PursuitClassificationReceipt{};
    fault = StatusCode::Ok;
    Vector3 nose{};
    Vector3 tangent{};
    if (!FiniteVector(own_position)
        || !FiniteVector(own_nose)
        || !FiniteVector(target_position)
        || !FiniteVector(target_velocity))
    {
        fault = StatusCode::NonFiniteInput;
        return;
    }
    if (!Unit3(own_nose, nose) || !Unit3(target_velocity, tangent))
    {
        output.reason = G10PursuitClassificationReason::
            NoseOrTargetTangentZeroMagnitude;
        return;
    }
    if (!std::isfinite(sample_dt)
        || sample_dt <= 0.0
        || !std::isfinite(nose_bound)
        || nose_bound < 0.0
        || !std::isfinite(target_bound)
        || target_bound < 0.0
        || !std::isfinite(position_bound)
        || position_bound < 0.0
        || !std::isfinite(locality_bound)
        || locality_bound <= 0.0)
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    if (nose_bound > LadyLuck::constants::Pi
        || target_bound > LadyLuck::constants::Pi)
    {
        output.reason = G10PursuitClassificationReason::
            SampleOrObservationResolutionInvalid;
        return;
    }
    const Vector3 relative = Sub3(target_position, own_position);
    const double relative_magnitude = Norm3(relative);
    const double b_nominal = Dot3(nose, tangent);
    const double e_nominal = Dot3(nose, relative);
    const double f_nominal = Dot3(tangent, relative);
    const double b_error = nose_bound + target_bound;
    const double e_error = relative_magnitude * nose_bound + position_bound;
    const double f_error = relative_magnitude * target_bound + position_bound;
    const Interval b_interval{
        (std::max)(-1.0, b_nominal - b_error),
        (std::min)(1.0, b_nominal + b_error)};
    const Interval e_interval{e_nominal - e_error, e_nominal + e_error};
    const Interval f_interval{f_nominal - f_error, f_nominal + f_error};
    const Interval b_square = Product(b_interval, b_interval);
    const Interval denominator{
        1.0 - b_square.upper,
        1.0 - b_square.lower};
    if (denominator.lower <= 0.0)
    {
        output.reason = G10PursuitClassificationReason::
            NoseRayNearParallelToTargetPath;
        return;
    }
    Interval offset_interval{};
    Interval forward_interval{};
    if (!Divide(
            Subtract(Product(b_interval, e_interval), f_interval),
            denominator,
            offset_interval)
        || !Divide(
            Subtract(e_interval, Product(b_interval, f_interval)),
            denominator,
            forward_interval))
    {
        output.reason = G10PursuitClassificationReason::
            NoseRayNearParallelToTargetPath;
        return;
    }
    const double denominator_nominal = 1.0 - b_nominal * b_nominal;
    const double offset_nominal =
        (b_nominal * e_nominal - f_nominal) / denominator_nominal;
    const double forward_nominal =
        (e_nominal - b_nominal * f_nominal) / denominator_nominal;
    const double sampling_resolution = Norm3(target_velocity) * sample_dt;
    if (!std::isfinite(offset_nominal)
        || !std::isfinite(forward_nominal)
        || !std::isfinite(sampling_resolution)
        || !std::isfinite(offset_interval.lower)
        || !std::isfinite(offset_interval.upper)
        || !std::isfinite(forward_interval.lower)
        || !std::isfinite(forward_interval.upper))
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    FillCommonClassification(
        output,
        offset_nominal,
        offset_interval,
        forward_nominal,
        forward_interval,
        locality_bound,
        sampling_resolution);
    if (forward_interval.lower <= 0.0)
    {
        output.reason = G10PursuitClassificationReason::
            CrossingNotProvedAheadOfAttacker;
        return;
    }
    if ((std::max)(
            std::fabs(offset_interval.lower),
            std::fabs(offset_interval.upper)) > locality_bound)
    {
        output.reason = G10PursuitClassificationReason::
            OffsetBeyondTargetPathLocality;
        return;
    }
    if (offset_interval.upper < -sampling_resolution)
    {
        output.state = G10PursuitState::Lag;
    }
    else if (offset_interval.lower > sampling_resolution)
    {
        output.state = G10PursuitState::Lead;
    }
    else if (offset_interval.lower >= -sampling_resolution
        && offset_interval.upper <= sampling_resolution)
    {
        output.state = G10PursuitState::Pure;
    }
    else
    {
        output.reason = G10PursuitClassificationReason::
            OffsetIntervalStraddlesPureBand;
        return;
    }
    output.valid = true;
    output.reason = G10PursuitClassificationReason::
        NoseRayPathCrossingAdmitted;
}

void ClassifyLiftMeridian(
    const Vector3& own_position,
    const Vector3& own_flight,
    const double flight_bound,
    const Vector3& directed_lift,
    const double lift_bound,
    const Vector3& target_position,
    const Vector3& target_velocity,
    const double target_bound,
    const double sample_dt,
    const double locality_bound,
    const double position_bound,
    G10PursuitClassificationReceipt& output,
    StatusCode& fault) noexcept
{
    output = G10PursuitClassificationReceipt{};
    fault = StatusCode::Ok;
    if (!FiniteVector(own_position)
        || !FiniteVector(own_flight)
        || !FiniteVector(directed_lift)
        || !FiniteVector(target_position)
        || !FiniteVector(target_velocity))
    {
        fault = StatusCode::NonFiniteInput;
        return;
    }
    Vector3 flight{};
    Vector3 lift{};
    Vector3 tangent{};
    if (!Unit3(own_flight, flight)
        || !Unit3(directed_lift, lift)
        || !Unit3(target_velocity, tangent))
    {
        output.reason = G10PursuitClassificationReason::
            FlightLiftOrTargetTangentZeroMagnitude;
        return;
    }
    if (!std::isfinite(sample_dt)
        || sample_dt <= 0.0
        || !std::isfinite(flight_bound)
        || flight_bound < 0.0
        || !std::isfinite(lift_bound)
        || lift_bound < 0.0
        || !std::isfinite(target_bound)
        || target_bound < 0.0
        || !std::isfinite(position_bound)
        || position_bound < 0.0
        || !std::isfinite(locality_bound)
        || locality_bound <= 0.0)
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    if (flight_bound > LadyLuck::constants::Pi
        || lift_bound > LadyLuck::constants::Pi
        || target_bound > LadyLuck::constants::Pi)
    {
        output.reason = G10PursuitClassificationReason::
            SampleOrObservationResolutionInvalid;
        return;
    }
    Vector3 normal{};
    double normal_bound = 0.0;
    if (!Unit3(Cross3(flight, lift), normal)
        || !CrossDirectionBound(
            flight, lift, flight_bound, lift_bound, normal_bound))
    {
        output.reason = G10PursuitClassificationReason::
            FlightAndLiftDoNotResolveManeuverPlane;
        return;
    }
    Vector3 meridian_lift{};
    double meridian_lift_bound = 0.0;
    if (!Unit3(Cross3(normal, flight), meridian_lift)
        || !CrossDirectionBound(
            normal,
            flight,
            normal_bound,
            flight_bound,
            meridian_lift_bound))
    {
        output.reason = G10PursuitClassificationReason::
            OrientedMeridianLiftAxisNotResolved;
        return;
    }
    if (Dot3(meridian_lift, lift) < 0.0)
    {
        meridian_lift = Scale3(meridian_lift, -1.0);
    }
    const Vector3 relative = Sub3(target_position, own_position);
    const double relative_magnitude = Norm3(relative);
    const double normal_chord_error = 2.0 * std::sin(normal_bound / 2.0);
    const double lift_chord_error = 2.0 * std::sin(meridian_lift_bound / 2.0);
    const double tangent_chord_error = 2.0 * std::sin(target_bound / 2.0);
    const double normal_relative = Dot3(normal, relative);
    const double normal_relative_error =
        relative_magnitude * normal_chord_error + position_bound;
    const Interval numerator{
        -normal_relative - normal_relative_error,
        -normal_relative + normal_relative_error};
    const double normal_tangent = Dot3(normal, tangent);
    const double normal_tangent_error = (std::min)(
        2.0,
        normal_chord_error
            + tangent_chord_error
            + normal_chord_error * tangent_chord_error);
    const Interval denominator{
        (std::max)(-1.0, normal_tangent - normal_tangent_error),
        (std::min)(1.0, normal_tangent + normal_tangent_error)};
    Interval target_interval{};
    if (!Divide(numerator, denominator, target_interval))
    {
        output.reason = G10PursuitClassificationReason::
            TargetTangentPlaneCrossingNotResolved;
        return;
    }
    const double target_offset = -normal_relative / normal_tangent;
    if (!std::isfinite(target_offset)
        || !std::isfinite(target_interval.lower)
        || !std::isfinite(target_interval.upper))
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    const double lift_relative = Dot3(meridian_lift, relative);
    const double lift_relative_error =
        relative_magnitude * lift_chord_error + position_bound;
    const Interval lift_relative_interval{
        lift_relative - lift_relative_error,
        lift_relative + lift_relative_error};
    const double lift_tangent = Dot3(meridian_lift, tangent);
    const double lift_tangent_error = (std::min)(
        2.0,
        lift_chord_error
            + tangent_chord_error
            + lift_chord_error * tangent_chord_error);
    const Interval lift_tangent_interval{
        (std::max)(-1.0, lift_tangent - lift_tangent_error),
        (std::min)(1.0, lift_tangent + lift_tangent_error)};
    const Interval lift_side_interval = Add(
        lift_relative_interval,
        Product(target_interval, lift_tangent_interval));
    const double lift_side = lift_relative + target_offset * lift_tangent;
    const double sampling_resolution = Norm3(target_velocity) * sample_dt;
    if (!std::isfinite(lift_side)
        || !std::isfinite(lift_side_interval.lower)
        || !std::isfinite(lift_side_interval.upper)
        || !std::isfinite(sampling_resolution))
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    SetOptional(output.target_path_offset_m, target_offset);
    SetOptional(output.target_path_offset_interval_m, target_interval);
    SetOptional(output.lift_side_distance_m, lift_side);
    SetOptional(output.lift_side_distance_interval_m, lift_side_interval);
    SetOptional(output.pure_sampling_resolution_m, sampling_resolution);
    if (lift_side_interval.lower <= 0.0)
    {
        output.reason = G10PursuitClassificationReason::
            TargetCrossingNotProvenOnPositiveLiftSide;
        return;
    }
    const double flight_chord_error = 2.0 * std::sin(flight_bound / 2.0);
    const double flight_relative = Dot3(flight, relative);
    const double flight_relative_error =
        relative_magnitude * flight_chord_error + position_bound;
    const Interval flight_relative_interval{
        flight_relative - flight_relative_error,
        flight_relative + flight_relative_error};
    const double flight_tangent = Dot3(flight, tangent);
    const double flight_tangent_error = (std::min)(
        2.0,
        flight_chord_error
            + tangent_chord_error
            + flight_chord_error * tangent_chord_error);
    const Interval flight_tangent_interval{
        (std::max)(-1.0, flight_tangent - flight_tangent_error),
        (std::min)(1.0, flight_tangent + flight_tangent_error)};
    const Interval forward_interval = Add(
        flight_relative_interval,
        Product(target_interval, flight_tangent_interval));
    const double forward_distance =
        flight_relative + target_offset * flight_tangent;
    if (!std::isfinite(forward_distance)
        || !std::isfinite(forward_interval.lower)
        || !std::isfinite(forward_interval.upper))
    {
        fault = StatusCode::InvalidArgument;
        return;
    }
    SetOptional(output.forward_distance_m, forward_distance);
    SetOptional(output.forward_distance_interval_m, forward_interval);
    SetOptional(output.target_path_locality_bound_m, locality_bound);
    if (forward_interval.lower <= 0.0)
    {
        output.reason = G10PursuitClassificationReason::
            TargetCrossingNotProvenAheadOfAttacker;
        return;
    }
    if (target_interval.lower < -locality_bound
        || target_interval.upper > locality_bound)
    {
        output.reason = G10PursuitClassificationReason::
            TargetTangentExtrapolationOutsideLocalityDomain;
        return;
    }
    if (target_interval.upper < -sampling_resolution)
    {
        output.state = G10PursuitState::Lag;
    }
    else if (target_interval.lower > sampling_resolution)
    {
        output.state = G10PursuitState::Lead;
    }
    else if (target_interval.lower >= -sampling_resolution
        && target_interval.upper <= sampling_resolution)
    {
        output.state = G10PursuitState::Pure;
    }
    else
    {
        output.reason = G10PursuitClassificationReason::
            TargetPathStateBoundaryNotResolved;
        return;
    }
    output.valid = true;
    output.reason = G10PursuitClassificationReason::
        OrientedLiftMeridianClassified;
}

bool TargetLocalityBound(
    const Vector3& relative,
    const double position_error,
    const Vector3& target_velocity,
    const Vector3& target_body_velocity,
    const PathPlane& target_path,
    bool& available,
    double& output,
    StatusCode& fault) noexcept
{
    available = false;
    output = 0.0;
    fault = StatusCode::Ok;
    if (!target_path.valid)
    {
        return true;
    }
    double speed_error = 0.0;
    if (!FiniteVector(relative)
        || !FiniteVector(target_velocity)
        || !Float32VectorErrorBound(target_body_velocity, speed_error))
    {
        fault = StatusCode::NonFiniteInput;
        return false;
    }
    const double duration = target_path.recent_duration_s;
    const double rotation = target_path.recent_rotation_rad;
    const double rotation_error = target_path.recent_rotation_bound_rad;
    if (!std::isfinite(duration)
        || !std::isfinite(rotation)
        || !std::isfinite(rotation_error)
        || !std::isfinite(position_error)
        || !std::isfinite(speed_error)
        || duration <= 0.0
        || rotation <= 0.0
        || rotation_error < 0.0
        || position_error < 0.0
        || speed_error < 0.0)
    {
        fault = StatusCode::InvalidArgument;
        return false;
    }
    const double separation_lower = Norm3(relative) - position_error;
    const double speed_lower = Norm3(target_velocity) - speed_error;
    const double rotation_upper = rotation + rotation_error;
    if (rotation_upper <= 0.0)
    {
        fault = StatusCode::InvalidArgument;
        return false;
    }
    if (separation_lower <= 0.0 || speed_lower <= 0.0)
    {
        return true;
    }
    const double radius_lower = speed_lower * duration / rotation_upper;
    output = (std::min)(separation_lower, radius_lower);
    if (!std::isfinite(output))
    {
        fault = StatusCode::InvalidArgument;
        return false;
    }
    available = output > 0.0;
    return true;
}

void ApplyLagProjection(
    LadyLuck::guidance::g10::G10SecondUseLagReacquisitionReceipt& output) noexcept
{
    using LadyLuck::guidance::g10::G10LagReacquisitionReason;
    const G10PursuitClassificationReceipt* selected = nullptr;
    if (output.plane_relation
        == RollingScissorsPlaneRelation::WithinObservationResolution)
    {
        selected = &output.nose_ray_classification;
        if (!selected->valid)
        {
            output.reason = G10LagReacquisitionReason::
                SimilarPlaneLiteralNoseUnresolved;
            return;
        }
        output.lag_reacquired.has_value = true;
        output.lag_reacquired.value = selected->state == G10PursuitState::Lag;
        switch (selected->state)
        {
        case G10PursuitState::Lag:
            output.reason = G10LagReacquisitionReason::SimilarPlaneLiteralNoseLag;
            break;
        case G10PursuitState::Pure:
            output.reason = G10LagReacquisitionReason::SimilarPlaneLiteralNosePure;
            break;
        case G10PursuitState::Lead:
        default:
            output.reason = G10LagReacquisitionReason::SimilarPlaneLiteralNoseLead;
            break;
        }
        output.evaluated = true;
        return;
    }
    if (output.plane_relation
        == RollingScissorsPlaneRelation::ResolvablySeparated)
    {
        if (!output.behavior_switch_admitted)
        {
            output.reason = G10LagReacquisitionReason::SeparatedPlaneUnresolved;
            return;
        }
        selected = &output.resolved_separated_plane_classification;
        if (!selected->valid)
        {
            output.reason = G10LagReacquisitionReason::
                SeparatedPlaneDirectedLiftMeridianUnresolved;
            return;
        }
        output.lag_reacquired.has_value = true;
        output.lag_reacquired.value = selected->state == G10PursuitState::Lag;
        switch (selected->state)
        {
        case G10PursuitState::Lag:
            output.reason = G10LagReacquisitionReason::
                SeparatedPlaneDirectedLiftMeridianLag;
            break;
        case G10PursuitState::Pure:
            output.reason = G10LagReacquisitionReason::
                SeparatedPlaneDirectedLiftMeridianPure;
            break;
        case G10PursuitState::Lead:
        default:
            output.reason = G10LagReacquisitionReason::
                SeparatedPlaneDirectedLiftMeridianLead;
            break;
        }
        output.evaluated = true;
        return;
    }
    output.reason = G10LagReacquisitionReason::
        ManeuverPlaneUnresolvedNotObservable;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

const char* G10PursuitClassificationReasonLabel(
    const G10PursuitClassificationReason reason) noexcept
{
    switch (reason)
    {
    case G10PursuitClassificationReason::NoseOrTargetTangentZeroMagnitude:
        return "nose_or_target_tangent_has_zero_magnitude";
    case G10PursuitClassificationReason::SampleOrObservationResolutionInvalid:
        return "sample_or_observation_resolution_invalid";
    case G10PursuitClassificationReason::NoseRayNearParallelToTargetPath:
        return "nose_ray_near_parallel_to_target_path";
    case G10PursuitClassificationReason::CrossingNotProvedAheadOfAttacker:
        return "crossing_not_proved_ahead_of_attacker";
    case G10PursuitClassificationReason::OffsetBeyondTargetPathLocality:
        return "offset_beyond_target_path_locality";
    case G10PursuitClassificationReason::OffsetIntervalStraddlesPureBand:
        return "offset_interval_straddles_pure_band";
    case G10PursuitClassificationReason::NoseRayPathCrossingAdmitted:
        return "nose_ray_path_crossing_admitted";
    case G10PursuitClassificationReason::AttitudeNoseOrTargetDirectionUnavailable:
        return "attitude_nose_or_target_direction_unavailable";
    case G10PursuitClassificationReason::FlightLiftOrTargetTangentZeroMagnitude:
        return "flight_lift_or_target_tangent_has_zero_magnitude";
    case G10PursuitClassificationReason::FlightAndLiftDoNotResolveManeuverPlane:
        return "flight_and_lift_do_not_resolve_maneuver_plane";
    case G10PursuitClassificationReason::OrientedMeridianLiftAxisNotResolved:
        return "oriented_meridian_lift_axis_not_resolved";
    case G10PursuitClassificationReason::TargetTangentPlaneCrossingNotResolved:
        return "target_tangent_plane_crossing_not_resolved";
    case G10PursuitClassificationReason::TargetCrossingNotProvenOnPositiveLiftSide:
        return "target_crossing_not_proven_on_positive_lift_side";
    case G10PursuitClassificationReason::TargetCrossingNotProvenAheadOfAttacker:
        return "target_crossing_not_proven_ahead_of_attacker";
    case G10PursuitClassificationReason::TargetTangentExtrapolationOutsideLocalityDomain:
        return "target_tangent_extrapolation_outside_locality_domain";
    case G10PursuitClassificationReason::TargetPathStateBoundaryNotResolved:
        return "target_path_state_boundary_not_resolved";
    case G10PursuitClassificationReason::OrientedLiftMeridianClassified:
        return "uncertainty_bounded_oriented_lift_meridian_classified";
    case G10PursuitClassificationReason::EstablishedPathLiftAxisOrTargetDirectionUnavailable:
        return "established_path_lift_axis_or_target_direction_unavailable";
    case G10PursuitClassificationReason::AttitudeLiftAxisOrTargetDirectionUnavailable:
        return "attitude_lift_axis_or_target_direction_unavailable";
    case G10PursuitClassificationReason::ManualLiftCriterionRequiresResolvedPlaneSeparation:
        return "manual_lift_criterion_requires_resolved_plane_separation";
    case G10PursuitClassificationReason::NotObserved:
    default:
        return "not_observed";
    }
}

const char* G10PursuitSwitchReasonLabel(
    const G10PursuitSwitchReason reason) noexcept
{
    switch (reason)
    {
    case G10PursuitSwitchReason::LiftProxySourcesDisagree:
        return "lift_proxy_sources_disagree";
    case G10PursuitSwitchReason::ResolvedClassificationInvalid:
        return "resolved_classification_invalid";
    case G10PursuitSwitchReason::ClassificationTrustworthyNoManualBoundaryClaim:
        return "second_contract_classification_trustworthy_no_manual_boundary_claim";
    case G10PursuitSwitchReason::PlaneSeparationNotResolved:
    default:
        return "plane_separation_not_resolved";
    }
}

const char* G10LagReacquisitionReasonLabel(
    const G10LagReacquisitionReason reason) noexcept
{
    switch (reason)
    {
    case G10LagReacquisitionReason::DescendingLagOwnerInactive:
        return "descending_lag_owner_inactive";
    case G10LagReacquisitionReason::DescendingLagOwnerTransitionSeeded:
        return "descending_lag_owner_transition_seeded";
    case G10LagReacquisitionReason::DescendingLagCommandNotAppliedBeforeState:
        return "descending_lag_command_not_applied_before_state";
    case G10LagReacquisitionReason::ManeuverPlaneUnresolvedNotObservable:
        return "maneuver_plane_unresolved:NOT_OBSERVABLE";
    case G10LagReacquisitionReason::SimilarPlaneLiteralNoseUnresolved:
        return "similar_plane_literal_nose_unresolved";
    case G10LagReacquisitionReason::SimilarPlaneLiteralNoseLag:
        return "similar_plane_literal_nose:LAG";
    case G10LagReacquisitionReason::SimilarPlaneLiteralNosePure:
        return "similar_plane_literal_nose:PURE";
    case G10LagReacquisitionReason::SimilarPlaneLiteralNoseLead:
        return "similar_plane_literal_nose:LEAD";
    case G10LagReacquisitionReason::SeparatedPlaneUnresolved:
        return "separated_plane_unresolved";
    case G10LagReacquisitionReason::SeparatedPlaneDirectedLiftMeridianUnresolved:
        return "separated_plane_directed_lift_meridian_unresolved";
    case G10LagReacquisitionReason::SeparatedPlaneDirectedLiftMeridianLag:
        return "separated_plane_directed_lift_meridian:LAG";
    case G10LagReacquisitionReason::SeparatedPlaneDirectedLiftMeridianPure:
        return "separated_plane_directed_lift_meridian:PURE";
    case G10LagReacquisitionReason::SeparatedPlaneDirectedLiftMeridianLead:
        return "separated_plane_directed_lift_meridian:LEAD";
    case G10LagReacquisitionReason::ContractRejected:
        return "g10_lag_reacquisition_contract_rejected";
    case G10LagReacquisitionReason::NotUpdated:
    default:
        return "not_updated";
    }
}

void G10SecondUseLagReacquisitionProvider::ResetPursuitHistory() noexcept
{
    ResetPath(
        own_path_.times_s,
        own_path_.directions_ned,
        own_path_.direction_bounds_rad,
        own_path_.count);
    ResetPath(
        target_path_.times_s,
        target_path_.directions_ned,
        target_path_.direction_bounds_rad,
        target_path_.count);
}

void G10SecondUseLagReacquisitionProvider::Reset() noexcept
{
    ResetPursuitHistory();
    descending_owner_active_ = false;
}

void G10SecondUseLagReacquisitionProvider::Update(
    const DogfightGeometryFrame& frame,
    const double sample_dt_s,
    const G10SecondUseLagReacquisitionInput& input,
    G10SecondUseLagReacquisitionReceipt& output,
    Status& status) noexcept
{
    output = G10SecondUseLagReacquisitionReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};
    const bool descending_owner_active = input.owner_selected
        && input.owner_phase
            == G10SecondUseOwnerPhase::DescendingLagReacquire;
    output.descending_lag_owner_active = descending_owner_active;
    output.descending_lag_command_applied_before_state =
        input.descending_lag_command_applied_before_state;
    if (!descending_owner_active)
    {
        if (descending_owner_active_)
        {
            ResetPursuitHistory();
        }
        descending_owner_active_ = false;
        output.valid = true;
        output.reason = G10LagReacquisitionReason::DescendingLagOwnerInactive;
        return;
    }
    if (!std::isfinite(sample_dt_s) || sample_dt_s <= 0.0)
    {
        ResetPursuitHistory();
        status.code = std::isfinite(sample_dt_s)
            ? StatusCode::InvalidDt
            : StatusCode::NonFiniteInput;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    if (!std::isfinite(frame.t_sec) || frame.t_sec < 0.0)
    {
        ResetPursuitHistory();
        status.code = std::isfinite(frame.t_sec)
            ? StatusCode::InvalidArgument
            : StatusCode::NonFiniteInput;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    const bool transition = !descending_owner_active_;
    if (transition)
    {
        ResetPursuitHistory();
        descending_owner_active_ = true;
        output.pursuit_epoch_reset = true;
    }

    PreparedDirection own_prepared{};
    PreparedDirection target_prepared{};
    PrepareDirectionSample(frame.own, own_prepared);
    PrepareDirectionSample(frame.opponent, target_prepared);
    if (own_prepared.disposition == Disposition::Invalid
        || target_prepared.disposition == Disposition::Invalid)
    {
        ResetPursuitHistory();
        status.code = own_prepared.disposition == Disposition::Invalid
            ? own_prepared.fault
            : target_prepared.fault;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    PathPlane own_path{};
    PathPlane target_path{};
    UpdatePath(
        own_path_.times_s,
        own_path_.directions_ned,
        own_path_.direction_bounds_rad,
        own_path_.count,
        own_prepared,
        frame.t_sec,
        sample_dt_s,
        own_path);
    UpdatePath(
        target_path_.times_s,
        target_path_.directions_ned,
        target_path_.direction_bounds_rad,
        target_path_.count,
        target_prepared,
        frame.t_sec,
        sample_dt_s,
        target_path);
    output.own_path_gate = own_path.gate;
    output.target_path_gate = target_path.gate;

    AttitudePlane own_attitude{};
    AttitudePlane target_attitude{};
    StatusCode fault = StatusCode::Ok;
    const Disposition own_attitude_disposition =
        AttitudePlaneProxy(frame.own, own_attitude, fault);
    if (own_attitude_disposition == Disposition::Invalid)
    {
        ResetPursuitHistory();
        status.code = fault;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    const Disposition target_attitude_disposition =
        AttitudePlaneProxy(frame.opponent, target_attitude, fault);
    (void)target_attitude;
    if (target_attitude_disposition == Disposition::Invalid)
    {
        ResetPursuitHistory();
        status.code = fault;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }

    if (own_path.valid && target_path.valid)
    {
        const double separation = std::acos(Clamp(
            std::fabs(Dot3(own_path.normal_ned, target_path.normal_ned)),
            0.0,
            1.0));
        const double separation_bound = (std::min)(
            0.5 * LadyLuck::constants::Pi,
            own_path.normal_bound_rad + target_path.normal_bound_rad);
        output.plane_relation = separation > separation_bound
            ? RollingScissorsPlaneRelation::ResolvablySeparated
            : RollingScissorsPlaneRelation::WithinObservationResolution;
        SetOptional(output.plane_separation_rad, separation);
        SetOptional(
            output.plane_separation_resolution_bound_rad,
            separation_bound);
    }

    double own_direction_bound = 0.0;
    const Disposition own_direction = AxisDirectionBound(
        frame.own,
        own_direction_bound,
        fault);
    double target_direction_bound = 0.0;
    const Disposition target_direction = AxisDirectionBound(
        frame.opponent,
        target_direction_bound,
        fault);
    if (own_direction == Disposition::Invalid
        || target_direction == Disposition::Invalid)
    {
        ResetPursuitHistory();
        status.code = fault;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    const bool own_direction_available = own_direction == Disposition::Valid;
    const bool target_direction_available =
        target_direction == Disposition::Valid;
    Vector3 own_flight{};
    const bool own_flight_available =
        Unit3(frame.own.velocity_ned_mps, own_flight);
    double own_position_error = 0.0;
    double target_position_error = 0.0;
    if (!Float32VectorErrorBound(
            frame.own.position_ned_m,
            own_position_error)
        || !Float32VectorErrorBound(
            frame.opponent.position_ned_m,
            target_position_error))
    {
        ResetPursuitHistory();
        status.code = StatusCode::NonFiniteInput;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }
    const double position_bound = own_position_error + target_position_error;
    const Vector3 relative = Sub3(
        frame.opponent.position_ned_m,
        frame.own.position_ned_m);
    bool locality_available = false;
    double locality_bound = 0.0;
    if (!TargetLocalityBound(
            relative,
            position_bound,
            frame.opponent.velocity_ned_mps,
            frame.opponent.velocity_body_mps,
            target_path,
            locality_available,
            locality_bound,
            fault))
    {
        ResetPursuitHistory();
        status.code = fault;
        output.reason = G10LagReacquisitionReason::ContractRejected;
        return;
    }

    if (own_path.valid
        && target_path.valid
        && own_flight_available
        && own_direction_available
        && target_direction_available
        && locality_available)
    {
        ClassifyLiftMeridian(
            frame.own.position_ned_m,
            own_flight,
            own_direction_bound,
            own_path.lift_ned,
            own_path.lift_bound_rad,
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            target_direction_bound,
            sample_dt_s,
            locality_bound,
            position_bound,
            output.path_lift_classification,
            fault);
        if (fault != StatusCode::Ok)
        {
            ResetPursuitHistory();
            status.code = fault;
            output.reason = G10LagReacquisitionReason::ContractRejected;
            return;
        }
    }
    else
    {
        output.path_lift_classification = Unobserved(
            G10PursuitClassificationReason::
                EstablishedPathLiftAxisOrTargetDirectionUnavailable);
    }
    if (own_attitude.valid
        && target_path.valid
        && target_direction_available
        && locality_available)
    {
        ClassifyLiftMeridian(
            frame.own.position_ned_m,
            own_attitude.flight_ned,
            own_attitude.flight_bound_rad,
            own_attitude.lift_ned,
            own_attitude.lift_bound_rad,
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            target_direction_bound,
            sample_dt_s,
            locality_bound,
            position_bound,
            output.attitude_lift_classification,
            fault);
        if (fault != StatusCode::Ok)
        {
            ResetPursuitHistory();
            status.code = fault;
            output.reason = G10LagReacquisitionReason::ContractRejected;
            return;
        }
    }
    else
    {
        output.attitude_lift_classification = Unobserved(
            G10PursuitClassificationReason::
                AttitudeLiftAxisOrTargetDirectionUnavailable);
    }
    if (target_direction_available && locality_available)
    {
        ClassifyNoseRay(
            frame.own.position_ned_m,
            frame.own.nose_ned,
            BattleServerRpyQuantumRad,
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            target_direction_bound,
            sample_dt_s,
            locality_bound,
            position_bound,
            output.nose_ray_classification,
            fault);
        if (fault != StatusCode::Ok)
        {
            ResetPursuitHistory();
            status.code = fault;
            output.reason = G10LagReacquisitionReason::ContractRejected;
            return;
        }
    }
    else
    {
        output.nose_ray_classification = Unobserved(
            G10PursuitClassificationReason::
                AttitudeNoseOrTargetDirectionUnavailable);
    }
    output.lift_source_disagreement =
        output.plane_relation
            == RollingScissorsPlaneRelation::ResolvablySeparated
        && output.path_lift_classification.valid
        && output.attitude_lift_classification.valid
        && output.path_lift_classification.state
            != output.attitude_lift_classification.state;
    if (output.plane_relation
        == RollingScissorsPlaneRelation::ResolvablySeparated)
    {
        output.resolved_separated_plane_classification =
            output.attitude_lift_classification;
    }
    else
    {
        output.resolved_separated_plane_classification = Unobserved(
            G10PursuitClassificationReason::
                ManualLiftCriterionRequiresResolvedPlaneSeparation);
    }
    if (output.plane_relation
        != RollingScissorsPlaneRelation::ResolvablySeparated)
    {
        output.behavior_switch_reason =
            G10PursuitSwitchReason::PlaneSeparationNotResolved;
    }
    else if (output.lift_source_disagreement)
    {
        output.behavior_switch_reason =
            G10PursuitSwitchReason::LiftProxySourcesDisagree;
    }
    else if (!output.resolved_separated_plane_classification.valid)
    {
        output.behavior_switch_reason =
            G10PursuitSwitchReason::ResolvedClassificationInvalid;
    }
    else
    {
        output.behavior_switch_admitted = true;
        output.behavior_switch_reason = G10PursuitSwitchReason::
            ClassificationTrustworthyNoManualBoundaryClaim;
    }
    output.valid = true;
    if (transition)
    {
        output.reason = G10LagReacquisitionReason::
            DescendingLagOwnerTransitionSeeded;
        return;
    }
    if (!input.descending_lag_command_applied_before_state)
    {
        output.reason = G10LagReacquisitionReason::
            DescendingLagCommandNotAppliedBeforeState;
        return;
    }
    ApplyLagProjection(output);
}

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
