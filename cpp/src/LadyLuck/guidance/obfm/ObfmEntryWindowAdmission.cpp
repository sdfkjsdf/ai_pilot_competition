#include "LadyLuck/guidance/obfm/ObfmEntryWindowAdmission.hpp"

#include "LadyLuck/geometry/WezRule.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{
namespace
{

bool Finite(const double value) noexcept
{
    return std::isfinite(value) != 0;
}

bool FiniteVector(const Vector3& value) noexcept
{
    return Finite(value[0]) && Finite(value[1]) && Finite(value[2]);
}

bool FiniteTargetCircle(const ObfmEntryTargetTurnCircle& circle) noexcept
{
    return FiniteVector(circle.target_position_ned_m)
        && FiniteVector(circle.target_velocity_ned_mps)
        && FiniteVector(circle.target_omega_ned_rad_s)
        && FiniteVector(circle.plane_normal_ned)
        && FiniteVector(circle.centre_direction_ned)
        && FiniteVector(circle.circle_centre_ned_m)
        && Finite(circle.radius_m)
        && Finite(circle.speed_mps)
        && Finite(circle.normal_turn_rate_rad_s)
        && Finite(circle.observer_rate_resolution_rad_s);
}

bool FiniteWindowGeometry(const ObfmEntryWindowGeometry& geometry) noexcept
{
    return FiniteTargetCircle(geometry.circle)
        && FiniteVector(geometry.own_position_ned_m)
        && FiniteVector(geometry.own_velocity_ned_mps)
        && FiniteVector(geometry.own_projected_course_ned)
        && FiniteVector(geometry.entry_radius_direction_ned)
        && FiniteVector(geometry.entry_point_ned_m)
        && FiniteVector(geometry.entry_vector_ned_m)
        && Finite(geometry.along_course_distance_m)
        && Finite(geometry.entry_distance_m);
}

bool FinitePassageSample(
    const ObfmEntryWindowPassageSample& sample) noexcept
{
    return FiniteVector(sample.window_axis_ned)
        && FiniteVector(sample.tangent_axis_ned)
        && FiniteVector(sample.own_plane_projection_ned_m)
        && Finite(sample.plane_offset_m)
        && Finite(sample.signed_tangent_distance_m)
        && Finite(sample.radial_fraction)
        && FiniteVector(sample.course_intersection_ned_m)
        && Finite(sample.course_intersection_distance_m);
}

bool FinitePassageEvent(const ObfmEntryWindowPassageEvent& event) noexcept
{
    return Finite(event.interpolation_fraction)
        && Finite(event.radial_fraction_at_crossing)
        && Finite(event.plane_offset_m_at_crossing)
        && FiniteVector(event.crossing_point_ned_m);
}

bool AdmittedObservationContractValid(
    const ObfmEntryWindowObservationReceipt& observation) noexcept
{
    return observation.evaluated
        && observation.admitted
        && observation.reason == ObfmEntryWindowReason::EntryWindowAhead
        && IsValidControlFrameIdentity(observation.frame_identity)
        && observation.target_path_rate_valid
        && observation.target_path_rate_feature_ready
        && Finite(observation.target_path_relative_rotation_rad)
        && Finite(observation.target_path_rate_rad_s)
        && Finite(observation.admission_rate_resolution_rad_s)
        && FiniteVector(observation.target_path_omega_ned_rad_s)
        && observation.geometry_available
        && FiniteWindowGeometry(observation.geometry)
        && observation.entry_point_velocity_available
        && FiniteVector(observation.entry_point_velocity_ned_mps)
        && observation.passage_sample.available
        && FinitePassageSample(observation.passage_sample)
        && FinitePassageEvent(observation.passage_event);
}

bool CheckedAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double maximum = std::numeric_limits<double>::max();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    return CheckedAdd(left, -right, output);
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double left_abs = std::fabs(left);
    const double right_abs = std::fabs(right);
    const double maximum = std::numeric_limits<double>::max();
    if ((left_abs > 1.0 && right_abs >= maximum / left_abs)
        || (right_abs > 1.0 && left_abs >= maximum / right_abs))
    {
        return false;
    }
    output = left * right;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedDivide(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(numerator) || !Finite(denominator) || denominator == 0.0)
    {
        return false;
    }
    const double numerator_abs = std::fabs(numerator);
    const double denominator_abs = std::fabs(denominator);
    if (numerator_abs != 0.0
        && denominator_abs < 1.0
        && numerator_abs
            >= std::numeric_limits<double>::max() * denominator_abs)
    {
        return false;
    }
    output = numerator / denominator;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool VectorNorm(const Vector3& value, double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    output = std::hypot(std::hypot(value[0], value[1]), value[2]);
    return Finite(output);
}

bool Normalize(
    const Vector3& value,
    Vector3& direction,
    double& magnitude) noexcept
{
    direction = Vector3{};
    magnitude = 0.0;
    if (!VectorNorm(value, magnitude) || magnitude <= 0.0)
    {
        return false;
    }
    return CheckedDivide(value[0], magnitude, direction[0])
        && CheckedDivide(value[1], magnitude, direction[1])
        && CheckedDivide(value[2], magnitude, direction[2]);
}

bool CheckedDot(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    output = 0.0;
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double accumulated = 0.0;
    return CheckedMultiply(left[0], right[0], term0)
        && CheckedMultiply(left[1], right[1], term1)
        && CheckedMultiply(left[2], right[2], term2)
        && CheckedAdd(term0, term1, accumulated)
        && CheckedAdd(accumulated, term2, output);
}

bool CheckedCross(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    double left_term = 0.0;
    double right_term = 0.0;
    if (!CheckedMultiply(left[1], right[2], left_term)
        || !CheckedMultiply(left[2], right[1], right_term)
        || !CheckedSubtract(left_term, right_term, output[0])
        || !CheckedMultiply(left[2], right[0], left_term)
        || !CheckedMultiply(left[0], right[2], right_term)
        || !CheckedSubtract(left_term, right_term, output[1])
        || !CheckedMultiply(left[0], right[1], left_term)
        || !CheckedMultiply(left[1], right[0], right_term)
        || !CheckedSubtract(left_term, right_term, output[2]))
    {
        output = Vector3{};
        return false;
    }
    return true;
}

bool CheckedScale(
    const Vector3& value,
    const double scale,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedMultiply(value[0], scale, output[0])
        && CheckedMultiply(value[1], scale, output[1])
        && CheckedMultiply(value[2], scale, output[2]);
}

bool CheckedVectorAdd(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedAdd(left[0], right[0], output[0])
        && CheckedAdd(left[1], right[1], output[1])
        && CheckedAdd(left[2], right[2], output[2]);
}

bool CheckedVectorSubtract(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return CheckedSubtract(left[0], right[0], output[0])
        && CheckedSubtract(left[1], right[1], output[1])
        && CheckedSubtract(left[2], right[2], output[2]);
}

bool BodyVelocityDirectionResolution(
    const Vector3& velocity_body_mps,
    double& output) noexcept
{
    output = 0.0;
    double speed = 0.0;
    if (!VectorNorm(velocity_body_mps, speed) || speed <= 0.0)
    {
        return false;
    }

    Vector3 half_cell{};
    const double float_maximum =
        static_cast<double>(std::numeric_limits<float>::max());
    for (std::size_t index = 0U; index < half_cell.size(); ++index)
    {
        const double source = velocity_body_mps[index];
        if (std::fabs(source) > float_maximum)
        {
            return false;
        }
        const float value = static_cast<float>(source);
        if (!std::isfinite(value))
        {
            return false;
        }
        const float upper = std::nextafter(
            value,
            std::numeric_limits<float>::infinity());
        const float lower = std::nextafter(
            value,
            -std::numeric_limits<float>::infinity());
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        const double value_double = static_cast<double>(value);
        const double upper_delta =
            std::fabs(static_cast<double>(upper) - value_double);
        const double lower_delta =
            std::fabs(value_double - static_cast<double>(lower));
        half_cell[index] = 0.5 * std::max(upper_delta, lower_delta);
    }

    double half_cell_norm = 0.0;
    double wire_diagonal = 0.0;
    double error_bound = 0.0;
    double true_speed_lower = 0.0;
    if (!VectorNorm(half_cell, half_cell_norm)
        || !CheckedMultiply(
            1.7320508075688772,
            kBattleServerBodyVelocityQuantumMps,
            wire_diagonal)
        || !CheckedAdd(wire_diagonal, half_cell_norm, error_bound)
        || !CheckedSubtract(speed, error_bound, true_speed_lower)
        || true_speed_lower <= 0.0)
    {
        return false;
    }
    double ratio = 0.0;
    if (!CheckedDivide(error_bound, true_speed_lower, ratio))
    {
        return false;
    }
    ratio = std::min(1.0, std::max(0.0, ratio));
    output = std::asin(ratio);
    return Finite(output) && output >= 0.0;
}

bool AttitudeRateResolution(
    const double dt_s,
    double& output) noexcept
{
    output = 0.0;
    double numerator = 0.0;
    return Finite(dt_s) && dt_s > 0.0
        && CheckedMultiply(6.0, kBattleServerRpyQuantumRad, numerator)
        && CheckedDivide(numerator, dt_s, output)
        && output >= 0.0;
}

bool BuildTargetCircle(
    const Vector3& target_position_ned_m,
    const Vector3& target_velocity_ned_mps,
    const Vector3& omega_ned_rad_s,
    const double observer_resolution_rad_s,
    ObfmEntryTargetTurnCircle& output) noexcept
{
    output = ObfmEntryTargetTurnCircle{};
    Vector3 velocity_direction{};
    double speed = 0.0;
    if (!FiniteVector(target_position_ned_m)
        || !Finite(observer_resolution_rad_s)
        || observer_resolution_rad_s < 0.0
        || !Normalize(
            target_velocity_ned_mps,
            velocity_direction,
            speed))
    {
        return false;
    }

    Vector3 centre_velocity{};
    Vector3 centre_direction{};
    Vector3 plane_vector{};
    Vector3 plane_normal{};
    double centre_velocity_magnitude = 0.0;
    double plane_vector_magnitude = 0.0;
    double normal_turn_rate = 0.0;
    if (!CheckedCross(omega_ned_rad_s, target_velocity_ned_mps, centre_velocity)
        || !Normalize(
            centre_velocity,
            centre_direction,
            centre_velocity_magnitude)
        || !CheckedCross(target_velocity_ned_mps, centre_velocity, plane_vector)
        || !Normalize(plane_vector, plane_normal, plane_vector_magnitude)
        || !CheckedDot(omega_ned_rad_s, plane_normal, normal_turn_rate)
        || normal_turn_rate <= observer_resolution_rad_s)
    {
        return false;
    }

    double radius = 0.0;
    Vector3 centre_offset{};
    Vector3 centre{};
    if (!CheckedDivide(speed, normal_turn_rate, radius)
        || radius <= 0.0
        || !CheckedScale(centre_direction, radius, centre_offset)
        || !CheckedVectorAdd(target_position_ned_m, centre_offset, centre))
    {
        return false;
    }

    output.target_position_ned_m = target_position_ned_m;
    output.target_velocity_ned_mps = target_velocity_ned_mps;
    output.target_omega_ned_rad_s = omega_ned_rad_s;
    output.plane_normal_ned = plane_normal;
    output.centre_direction_ned = centre_direction;
    output.circle_centre_ned_m = centre;
    output.radius_m = radius;
    output.speed_mps = speed;
    output.normal_turn_rate_rad_s = normal_turn_rate;
    output.observer_rate_resolution_rad_s = observer_resolution_rad_s;
    return true;
}

bool BuildEntryGeometry(
    const ObfmEntryTargetTurnCircle& circle,
    const Vector3& own_position_ned_m,
    const Vector3& own_velocity_ned_mps,
    ObfmEntryWindowGeometry& output,
    ObfmEntryWindowReason& reason) noexcept
{
    output = ObfmEntryWindowGeometry{};
    reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
    if (!FiniteVector(own_position_ned_m))
    {
        return false;
    }
    Vector3 own_course{};
    double own_speed = 0.0;
    if (!Normalize(own_velocity_ned_mps, own_course, own_speed))
    {
        reason = ObfmEntryWindowReason::OwnWorldVelocityUnavailable;
        return false;
    }
    double normal_component = 0.0;
    Vector3 normal_projection{};
    Vector3 projected{};
    Vector3 projected_course{};
    double projected_magnitude = 0.0;
    if (!CheckedDot(own_course, circle.plane_normal_ned, normal_component)
        || !CheckedScale(
            circle.plane_normal_ned,
            normal_component,
            normal_projection)
        || !CheckedVectorSubtract(own_course, normal_projection, projected)
        || !Normalize(projected, projected_course, projected_magnitude))
    {
        reason = ObfmEntryWindowReason::OwnCourseProjectionUnavailable;
        return false;
    }

    Vector3 entry_radius_raw{};
    Vector3 entry_radius{};
    double entry_radius_magnitude = 0.0;
    Vector3 radius_offset{};
    Vector3 entry_point{};
    Vector3 entry_vector{};
    double entry_distance = 0.0;
    double along_course = 0.0;
    if (!CheckedCross(projected_course, circle.plane_normal_ned, entry_radius_raw)
        || !Normalize(
            entry_radius_raw,
            entry_radius,
            entry_radius_magnitude)
        || !CheckedScale(entry_radius, circle.radius_m, radius_offset)
        || !CheckedVectorAdd(circle.circle_centre_ned_m, radius_offset, entry_point)
        || !CheckedVectorSubtract(entry_point, own_position_ned_m, entry_vector)
        || !VectorNorm(entry_vector, entry_distance)
        || !CheckedDot(entry_vector, own_course, along_course))
    {
        return false;
    }

    output.circle = circle;
    output.own_position_ned_m = own_position_ned_m;
    output.own_velocity_ned_mps = own_velocity_ned_mps;
    output.own_projected_course_ned = projected_course;
    output.entry_radius_direction_ned = entry_radius;
    output.entry_point_ned_m = entry_point;
    output.entry_vector_ned_m = entry_vector;
    output.along_course_distance_m = along_course;
    output.entry_distance_m = entry_distance;
    output.ahead = along_course > 0.0;
    return true;
}

bool BuildPassageSample(
    const ObfmEntryWindowGeometry& geometry,
    ObfmEntryWindowPassageSample& output) noexcept
{
    output = ObfmEntryWindowPassageSample{};
    Vector3 window_vector{};
    Vector3 window_axis{};
    double window_length = 0.0;
    if (!CheckedVectorSubtract(
            geometry.circle.circle_centre_ned_m,
            geometry.entry_point_ned_m,
            window_vector)
        || !Normalize(window_vector, window_axis, window_length))
    {
        return false;
    }

    Vector3 own_relative{};
    double plane_offset = 0.0;
    Vector3 plane_offset_vector{};
    Vector3 own_plane_projection{};
    Vector3 relative{};
    double signed_tangent = 0.0;
    double radial_numerator = 0.0;
    double radial_fraction = 0.0;
    double intersection_distance = 0.0;
    Vector3 intersection_offset{};
    Vector3 intersection{};
    if (!CheckedVectorSubtract(
            geometry.own_position_ned_m,
            geometry.entry_point_ned_m,
            own_relative)
        || !CheckedDot(
            own_relative,
            geometry.circle.plane_normal_ned,
            plane_offset)
        || !CheckedScale(
            geometry.circle.plane_normal_ned,
            plane_offset,
            plane_offset_vector)
        || !CheckedVectorSubtract(
            geometry.own_position_ned_m,
            plane_offset_vector,
            own_plane_projection)
        || !CheckedVectorSubtract(
            own_plane_projection,
            geometry.entry_point_ned_m,
            relative)
        || !CheckedDot(
            relative,
            geometry.own_projected_course_ned,
            signed_tangent)
        || !CheckedDot(relative, window_axis, radial_numerator)
        || !CheckedDivide(radial_numerator, window_length, radial_fraction)
        || !CheckedMultiply(-1.0, signed_tangent, intersection_distance)
        || !CheckedScale(
            geometry.own_projected_course_ned,
            intersection_distance,
            intersection_offset)
        || !CheckedVectorAdd(
            own_plane_projection,
            intersection_offset,
            intersection))
    {
        return false;
    }

    output.available = true;
    output.window_axis_ned = window_axis;
    output.tangent_axis_ned = geometry.own_projected_course_ned;
    output.own_plane_projection_ned_m = own_plane_projection;
    output.plane_offset_m = plane_offset;
    output.signed_tangent_distance_m = signed_tangent;
    output.radial_fraction = radial_fraction;
    output.course_intersection_ned_m = intersection;
    output.course_intersection_distance_m = intersection_distance;
    output.course_intersection_ahead = intersection_distance > 0.0;
    output.course_intersection_on_segment =
        radial_fraction >= 0.0 && radial_fraction <= 1.0;
    return true;
}

bool BuildPassageEvent(
    const ObfmEntryWindowPassageSample& previous,
    const ObfmEntryWindowPassageSample& current,
    ObfmEntryWindowPassageEvent& output) noexcept
{
    output = ObfmEntryWindowPassageEvent{};
    output.evaluated = true;
    if (!previous.available || !current.available)
    {
        output.reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return true;
    }

    double tangent_alignment = 0.0;
    double window_alignment = 0.0;
    if (!CheckedDot(
            previous.tangent_axis_ned,
            current.tangent_axis_ned,
            tangent_alignment)
        || !CheckedDot(
            previous.window_axis_ned,
            current.window_axis_ned,
            window_alignment))
    {
        output.reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return true;
    }
    if (tangent_alignment <= 0.0 || window_alignment <= 0.0)
    {
        output.reason =
            ObfmEntryWindowReason::WindowFrameOrientationDiscontinuous;
        return true;
    }

    output.available = true;
    double delta_signed = 0.0;
    if (!CheckedSubtract(
            current.signed_tangent_distance_m,
            previous.signed_tangent_distance_m,
            delta_signed))
    {
        output.available = false;
        output.reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return true;
    }
    if (delta_signed <= 0.0)
    {
        output.reason = ObfmEntryWindowReason::NoForwardRelativeProgress;
        return true;
    }
    if (!(previous.signed_tangent_distance_m < 0.0
        && current.signed_tangent_distance_m >= 0.0))
    {
        output.reason = ObfmEntryWindowReason::NoTangentCoordinateCrossing;
        return true;
    }

    double negative_previous = 0.0;
    double fraction = 0.0;
    double radial_delta = 0.0;
    double radial_step = 0.0;
    double radial_at_crossing = 0.0;
    double plane_delta = 0.0;
    double plane_step = 0.0;
    double plane_at_crossing = 0.0;
    Vector3 projection_delta{};
    Vector3 projection_step{};
    Vector3 crossing_point{};
    if (!CheckedMultiply(
            -1.0,
            previous.signed_tangent_distance_m,
            negative_previous)
        || !CheckedDivide(negative_previous, delta_signed, fraction)
        || fraction < 0.0
        || fraction > 1.0
        || !CheckedSubtract(
            current.radial_fraction,
            previous.radial_fraction,
            radial_delta)
        || !CheckedMultiply(fraction, radial_delta, radial_step)
        || !CheckedAdd(
            previous.radial_fraction,
            radial_step,
            radial_at_crossing)
        || !CheckedSubtract(
            current.plane_offset_m,
            previous.plane_offset_m,
            plane_delta)
        || !CheckedMultiply(fraction, plane_delta, plane_step)
        || !CheckedAdd(
            previous.plane_offset_m,
            plane_step,
            plane_at_crossing)
        || !CheckedVectorSubtract(
            current.own_plane_projection_ned_m,
            previous.own_plane_projection_ned_m,
            projection_delta)
        || !CheckedScale(projection_delta, fraction, projection_step)
        || !CheckedVectorAdd(
            previous.own_plane_projection_ned_m,
            projection_step,
            crossing_point))
    {
        output.available = false;
        output.reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return true;
    }

    output.interpolation_available = true;
    output.interpolation_fraction = fraction;
    output.radial_fraction_at_crossing = radial_at_crossing;
    output.plane_offset_m_at_crossing = plane_at_crossing;
    output.crossing_point_ned_m = crossing_point;
    output.projected_passage =
        radial_at_crossing >= 0.0 && radial_at_crossing <= 1.0;
    output.reason = output.projected_passage
        ? ObfmEntryWindowReason::ProjectedWindowPassage
        : ObfmEntryWindowReason::TangentCrossingOutsideWindowSegment;
    return true;
}

ObfmEntrySetupCompletionKind CompletionKind(
    const ObfmEntryWindowObservationReceipt& observation) noexcept
{
    const ObfmEntryWindowPassageEvent& event = observation.passage_event;
    if (!event.evaluated || !event.available
        || !event.interpolation_available
        || event.interpolation_fraction < 0.0
        || event.interpolation_fraction > 1.0)
    {
        return ObfmEntrySetupCompletionKind::None;
    }
    if (event.reason == ObfmEntryWindowReason::ProjectedWindowPassage
        && event.projected_passage)
    {
        return ObfmEntrySetupCompletionKind::ProjectedSegmentPassage;
    }
    if (event.reason
            == ObfmEntryWindowReason::TangentCrossingOutsideWindowSegment
        && !event.projected_passage)
    {
        return ObfmEntrySetupCompletionKind::
            OutsideSegmentDevelopmentEvent;
    }
    return ObfmEntrySetupCompletionKind::None;
}

void ClearGeometryHistory(
    bool& previous_entry_point_available,
    Vector3& previous_entry_point,
    bool& previous_passage_available,
    ObfmEntryWindowPassageSample& previous_passage) noexcept
{
    previous_entry_point_available = false;
    previous_entry_point = Vector3{};
    previous_passage_available = false;
    previous_passage = ObfmEntryWindowPassageSample{};
}

void CacheEstablishedTurnOutput(
    const DogfightGeometryFrame& frame,
    const ObfmEntryEstablishedTurnReceipt& output,
    const Status& status,
    bool& cached_identity_valid,
    ControlFrameIdentity& cached_identity,
    std::int32_t& cached_own_plane_id,
    std::int32_t& cached_target_plane_id,
    ObfmEntryEstablishedTurnReceipt& cached_receipt,
    StatusCode& cached_status_code) noexcept
{
    cached_identity_valid = true;
    cached_identity = frame.frame_identity;
    cached_own_plane_id = frame.own_plane_id;
    cached_target_plane_id = frame.target_plane_id;
    cached_receipt = output;
    cached_status_code = status.code;
}

class EstablishedTurnCacheAction final
{
public:
    EstablishedTurnCacheAction(
        const DogfightGeometryFrame& frame,
        const ObfmEntryEstablishedTurnReceipt& output,
        const Status& status,
        bool& cached_identity_valid,
        ControlFrameIdentity& cached_identity,
        std::int32_t& cached_own_plane_id,
        std::int32_t& cached_target_plane_id,
        ObfmEntryEstablishedTurnReceipt& cached_receipt,
        StatusCode& cached_status_code) noexcept
        : frame_(frame),
          output_(output),
          status_(status),
          cached_identity_valid_(cached_identity_valid),
          cached_identity_(cached_identity),
          cached_own_plane_id_(cached_own_plane_id),
          cached_target_plane_id_(cached_target_plane_id),
          cached_receipt_(cached_receipt),
          cached_status_code_(cached_status_code)
    {
    }

    void operator()() noexcept
    {
        CacheEstablishedTurnOutput(
            frame_,
            output_,
            status_,
            cached_identity_valid_,
            cached_identity_,
            cached_own_plane_id_,
            cached_target_plane_id_,
            cached_receipt_,
            cached_status_code_);
    }

private:
    const DogfightGeometryFrame& frame_;
    const ObfmEntryEstablishedTurnReceipt& output_;
    const Status& status_;
    bool& cached_identity_valid_;
    ControlFrameIdentity& cached_identity_;
    std::int32_t& cached_own_plane_id_;
    std::int32_t& cached_target_plane_id_;
    ObfmEntryEstablishedTurnReceipt& cached_receipt_;
    StatusCode& cached_status_code_;
};

} // namespace

bool BuildObfmEntryTargetOnlyTransportedPoint(
    const ObfmEntryTargetTurnCircle& current_circle,
    const Vector3& previous_own_position_ned_m,
    const Vector3& previous_own_velocity_ned_mps,
    Vector3& transported_entry_point_ned_m,
    ObfmEntryWindowReason& reason) noexcept
{
    transported_entry_point_ned_m = Vector3{};
    ObfmEntryWindowGeometry transported{};
    if (!BuildEntryGeometry(
            current_circle,
            previous_own_position_ned_m,
            previous_own_velocity_ned_mps,
            transported,
            reason))
    {
        return false;
    }
    transported_entry_point_ned_m = transported.entry_point_ned_m;
    return true;
}

const char* ObfmEntryWindowReasonLabel(
    const ObfmEntryWindowReason reason) noexcept
{
    switch (reason)
    {
    case ObfmEntryWindowReason::Reset:
        return "reset";
    case ObfmEntryWindowReason::SelectorServiceNotReached:
        return "selector_service_not_reached";
    case ObfmEntryWindowReason::FrameEvidenceUnavailable:
        return "frame_evidence_unavailable";
    case ObfmEntryWindowReason::DtUnavailable:
        return "dt_unavailable";
    case ObfmEntryWindowReason::TargetWorldVelocityUnavailable:
        return "target_world_velocity_unavailable";
    case ObfmEntryWindowReason::TargetBodyVelocityUnavailable:
        return "target_body_velocity_unavailable";
    case ObfmEntryWindowReason::TargetPathRateInit:
        return "target_path_rate_init";
    case ObfmEntryWindowReason::TargetPathRateUnderResolution:
        return "target_path_rate_under_resolution";
    case ObfmEntryWindowReason::TargetPathRotationAxisUnavailable:
        return "target_path_rotation_axis_unavailable";
    case ObfmEntryWindowReason::TargetCircleUnavailable:
        return "target_circle_unavailable";
    case ObfmEntryWindowReason::OwnWorldVelocityUnavailable:
        return "own_world_velocity_unavailable";
    case ObfmEntryWindowReason::OwnCourseProjectionUnavailable:
        return "own_course_projection_unavailable";
    case ObfmEntryWindowReason::EntryWindowGeometryUnavailable:
        return "entry_window_geometry_unavailable";
    case ObfmEntryWindowReason::EntryWindowNotAhead:
        return "entry_window_not_ahead";
    case ObfmEntryWindowReason::EntryPointVelocityInit:
        return "entry_point_velocity_init";
    case ObfmEntryWindowReason::EntryWindowAhead:
        return "entry_window_ahead";
    case ObfmEntryWindowReason::WindowFrameOrientationDiscontinuous:
        return "window_frame_orientation_discontinuous";
    case ObfmEntryWindowReason::NoForwardRelativeProgress:
        return "no_forward_relative_progress";
    case ObfmEntryWindowReason::NoTangentCoordinateCrossing:
        return "no_tangent_coordinate_crossing";
    case ObfmEntryWindowReason::ProjectedWindowPassage:
        return "projected_window_passage";
    case ObfmEntryWindowReason::TangentCrossingOutsideWindowSegment:
        return "tangent_crossing_outside_window_segment";
    case ObfmEntryWindowReason::FeatureDisabled:
        return "feature_disabled";
    case ObfmEntryWindowReason::SpacingOwnerDependencyDisabled:
        return "spacing_owner_dependency_disabled";
    case ObfmEntryWindowReason::SafetyEvidenceUnavailable:
        return "safety_evidence_unavailable";
    case ObfmEntryWindowReason::SafetyNotAdmitted:
        return "safety_not_admitted";
    case ObfmEntryWindowReason::SpacingHandoffDeferredCurrentEnergy:
        return "spacing_handoff_deferred_current_energy";
    case ObfmEntryWindowReason::EntrySetupCompletedProjectedPassage:
        return "entry_setup_completed:projected_segment_passage";
    case ObfmEntryWindowReason::EntrySetupCompletedOutsideSegmentDevelopment:
        return "entry_setup_completed:outside_segment_development_event";
    case ObfmEntryWindowReason::EntrySetupSelected:
        return "entry_setup_selected";
    case ObfmEntryWindowReason::EntrySetupEntered:
        return "entry_setup_entered";
    case ObfmEntryWindowReason::EntrySetupContinued:
        return "entry_setup_continued";
    case ObfmEntryWindowReason::EntryObservationLost:
        return "entry_observation_lost";
    case ObfmEntryWindowReason::SpacingOwnerHandoff:
        return "spacing_owner_handoff";
    case ObfmEntryWindowReason::OfficialEmployPreemption:
        return "official_employ_preemption";
    case ObfmEntryWindowReason::EntrySetupTreePreempted:
        return "entry_setup_tree_preempted";
    case ObfmEntryWindowReason::EntrySetupReleased:
        return "entry_setup_released";
    case ObfmEntryWindowReason::EntrySetupInactive:
        return "entry_setup_inactive";
    case ObfmEntryWindowReason::OwnerContractUnavailable:
        return "owner_contract_unavailable";
    case ObfmEntryWindowReason::DeclaredReadyFrameIdentityInvalid:
        return "declared_ready_frame_identity_invalid";
    case ObfmEntryWindowReason::DeclaredReadyFrameNonfinite:
        return "declared_ready_frame_nonfinite";
    case ObfmEntryWindowReason::ServiceReceiptContradiction:
        return "service_receipt_contradiction";
    case ObfmEntryWindowReason::TaskLifecycleContradiction:
        return "task_lifecycle_contradiction";
    case ObfmEntryWindowReason::TurnCircleNotEstablished:
        return "turn_circle_not_established";
    }
    return "unknown";
}

const char* ObfmEntryEstablishedTurnReasonLabel(
    const ObfmEntryEstablishedTurnReason reason) noexcept
{
    switch (reason)
    {
    case ObfmEntryEstablishedTurnReason::Reset:
        return "reset";
    case ObfmEntryEstablishedTurnReason::FrameEvidenceUnavailable:
        return "frame_evidence_unavailable";
    case ObfmEntryEstablishedTurnReason::RecentOfficialChordUnavailable:
        return "recent_official_angle_chord_unavailable";
    case ObfmEntryEstablishedTurnReason::OlderOfficialChordUnavailable:
        return "older_official_angle_chord_unavailable";
    case ObfmEntryEstablishedTurnReason::TurnRateIntervalsDisjoint:
        return "turn_rate_intervals_disjoint";
    case ObfmEntryEstablishedTurnReason::TurnPlaneConesDisjoint:
        return "turn_plane_cones_disjoint";
    case ObfmEntryEstablishedTurnReason::CircleUnobservable:
        return "circle_unobservable";
    case ObfmEntryEstablishedTurnReason::TwoOfficialChordsConsistent:
        return "two_official_angle_chords_consistent";
    case ObfmEntryEstablishedTurnReason::HistoryCapacityExceeded:
        return "history_capacity_exceeded";
    case ObfmEntryEstablishedTurnReason::OfficialRuleUnavailable:
        return "official_rule_unavailable";
    case ObfmEntryEstablishedTurnReason::DeclaredReadyFrameIdentityInvalid:
        return "declared_ready_frame_identity_invalid";
    case ObfmEntryEstablishedTurnReason::DeclaredReadyFrameNonfinite:
        return "declared_ready_frame_nonfinite";
    }
    return "unknown";
}

ObfmEntryEstablishedTurnObserver::
    ObfmEntryEstablishedTurnObserver() noexcept
{
    Reset();
}

void ObfmEntryEstablishedTurnObserver::ResetPhysicalHistory() noexcept
{
    turn_history_head_ = 0U;
    turn_history_count_ = 0U;
    turn_observer_time_s_ = 0.0;
}

void ObfmEntryEstablishedTurnObserver::Reset() noexcept
{
    ResetPhysicalHistory();
    cached_identity_valid_ = false;
    cached_identity_ = ControlFrameIdentity{};
    cached_own_plane_id_ = -1;
    cached_target_plane_id_ = -1;
    cached_receipt_ = ObfmEntryEstablishedTurnReceipt{};
    cached_status_code_ = StatusCode::Seeded;
}

void ObfmEntryEstablishedTurnObserver::AppendTurnSample(
    const TurnSupportSample& sample,
    bool& appended) noexcept
{
    appended = false;
    if (turn_history_count_ >= turn_history_.size())
    {
        return;
    }
    const std::size_t tail =
        (turn_history_head_ + turn_history_count_) % turn_history_.size();
    turn_history_[tail] = sample;
    ++turn_history_count_;
    appended = true;
}

void ObfmEntryEstablishedTurnObserver::BuildSupportChord(
    const std::size_t end_offset,
    const double official_support_angle_rad,
    std::size_t& anchor_offset,
    ObfmEntryTurnChordReceipt& output) const noexcept
{
    output = ObfmEntryTurnChordReceipt{};
    anchor_offset = 0U;
    if (!Finite(official_support_angle_rad)
        || official_support_angle_rad <= 0.0
        || end_offset == 0U
        || end_offset >= turn_history_count_)
    {
        return;
    }

    const TurnSupportSample& end = turn_history_[
        (turn_history_head_ + end_offset) % turn_history_.size()];
    for (std::size_t candidate = end_offset; candidate > 0U; --candidate)
    {
        const std::size_t current_anchor = candidate - 1U;
        const TurnSupportSample& anchor = turn_history_[
            (turn_history_head_ + current_anchor) % turn_history_.size()];
        Vector3 cross{};
        double sine = 0.0;
        double cosine = 0.0;
        if (!CheckedCross(anchor.direction_ned, end.direction_ned, cross)
            || !VectorNorm(cross, sine)
            || !CheckedDot(anchor.direction_ned, end.direction_ned, cosine))
        {
            continue;
        }
        cosine = (std::max)(-1.0, (std::min)(1.0, cosine));
        const double rotation = std::atan2(sine, cosine);
        if (!Finite(rotation) || rotation < official_support_angle_rad)
        {
            continue;
        }

        double wire_endpoint_error = 0.0;
        double endpoint_with_anchor_error = 0.0;
        double endpoint_error = 0.0;
        double twice_endpoint_error = 0.0;
        if (!CheckedMultiply(
                6.0,
                kBattleServerRpyQuantumRad,
                wire_endpoint_error)
            || !CheckedAdd(
                wire_endpoint_error,
                anchor.body_direction_error_bound_rad,
                endpoint_with_anchor_error)
            || !CheckedAdd(
                endpoint_with_anchor_error,
                end.body_direction_error_bound_rad,
                endpoint_error)
            || !CheckedMultiply(2.0, endpoint_error, twice_endpoint_error)
            || sine <= 0.0
            || rotation <= twice_endpoint_error)
        {
            continue;
        }

        const double delta_cross = (std::min)(2.0, endpoint_error);
        double true_cross_lower = 0.0;
        double axis_ratio = 0.0;
        double duration = 0.0;
        double mean_rate = 0.0;
        double rate_error = 0.0;
        double upper_rate = 0.0;
        Vector3 normal{};
        if (!CheckedSubtract(
                std::sin(rotation),
                delta_cross,
                true_cross_lower)
            || true_cross_lower <= 0.0
            || !CheckedDivide(delta_cross, true_cross_lower, axis_ratio)
            || !CheckedSubtract(end.time_s, anchor.time_s, duration)
            || duration <= 0.0
            || !CheckedDivide(rotation, duration, mean_rate)
            || !CheckedDivide(endpoint_error, duration, rate_error)
            || !CheckedAdd(mean_rate, rate_error, upper_rate)
            || !CheckedScale(cross, 1.0 / sine, normal))
        {
            continue;
        }
        axis_ratio = (std::max)(0.0, (std::min)(1.0, axis_ratio));
        const double axis_error = std::asin(axis_ratio);
        double lower_rate = 0.0;
        if (!Finite(axis_error)
            || !CheckedSubtract(mean_rate, rate_error, lower_rate))
        {
            continue;
        }

        output.valid = true;
        output.duration_s = duration;
        output.rotation_rad = rotation;
        output.endpoint_direction_error_bound_rad = endpoint_error;
        output.plane_normal_ned = normal;
        output.mean_turn_rate_rad_s = mean_rate;
        output.turn_rate_lower_rad_s = (std::max)(0.0, lower_rate);
        output.turn_rate_upper_rad_s = upper_rate;
        output.plane_axis_error_bound_rad = axis_error;
        anchor_offset = current_anchor;
        return;
    }
}

void ObfmEntryEstablishedTurnObserver::Observe(
    const DogfightGeometryFrame& frame,
    const ObfmEntryWindowObservationInput& input,
    ObfmEntryEstablishedTurnReceipt& output,
    Status& status) noexcept
{
    output = ObfmEntryEstablishedTurnReceipt{};
    output.frame_identity = frame.frame_identity;
    output.evaluated = true;
    status = Status{};

    if (!input.frame_evidence_available)
    {
        Reset();
        output.frame_identity = frame.frame_identity;
        output.evaluated = true;
        output.reason =
            ObfmEntryEstablishedTurnReason::FrameEvidenceUnavailable;
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || frame.own_plane_id < 0
        || frame.target_plane_id < 0
        || frame.own_plane_id == frame.target_plane_id
        || !frame.target_same_index
        || frame.target_frame_index != frame.frame_identity.frame_index)
    {
        Reset();
        output.frame_identity = frame.frame_identity;
        output.evaluated = true;
        output.reason = ObfmEntryEstablishedTurnReason::
            DeclaredReadyFrameIdentityInvalid;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!Finite(input.dt_s)
        || !Finite(frame.t_sec)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps)
        || !FiniteVector(frame.opponent.velocity_body_mps))
    {
        Reset();
        output.frame_identity = frame.frame_identity;
        output.evaluated = true;
        output.reason = ObfmEntryEstablishedTurnReason::
            DeclaredReadyFrameNonfinite;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (input.dt_s <= 0.0)
    {
        Reset();
        output.frame_identity = frame.frame_identity;
        output.evaluated = true;
        output.reason = ObfmEntryEstablishedTurnReason::
            FrameEvidenceUnavailable;
        return;
    }

    if (cached_identity_valid_
        && SameControlFrameIdentity(
            cached_identity_,
            frame.frame_identity)
        && cached_own_plane_id_ == frame.own_plane_id
        && cached_target_plane_id_ == frame.target_plane_id)
    {
        output = cached_receipt_;
        status.code = cached_status_code_;
        return;
    }

    const bool identity_restarted = cached_identity_valid_
        && (cached_own_plane_id_ != frame.own_plane_id
            || cached_target_plane_id_ != frame.target_plane_id
            || cached_identity_.episode_epoch
                != frame.frame_identity.episode_epoch
            || cached_identity_.frame_index
                == (std::numeric_limits<std::uint64_t>::max)()
            || frame.frame_identity.frame_index
                != cached_identity_.frame_index + 1U
            || frame.frame_identity.source_time_s
                <= cached_identity_.source_time_s);
    if (identity_restarted)
    {
        ResetPhysicalHistory();
        status.code = StatusCode::FrameGap;
    }
    output.identity_restarted = identity_restarted;

    EstablishedTurnCacheAction cache_output{
        frame,
        output,
        status,
        cached_identity_valid_,
        cached_identity_,
        cached_own_plane_id_,
        cached_target_plane_id_,
        cached_receipt_,
        cached_status_code_};

    const Result<WezPhase> p1 = OfficialWezPhaseAt(0U);
    const Result<WezPhase> p2 = OfficialWezPhaseAt(1U);
    double official_history_s = 0.0;
    if (!p1.sample_valid()
        || !p2.sample_valid()
        || !Finite(p1.value.angle_rad)
        || p1.value.angle_rad <= 0.0
        || !CheckedSubtract(
            p2.value.start_sec,
            p1.value.start_sec,
            official_history_s)
        || official_history_s <= 0.0)
    {
        ResetPhysicalHistory();
        output.reason =
            ObfmEntryEstablishedTurnReason::OfficialRuleUnavailable;
        status.code = StatusCode::InvalidConfiguration;
        cache_output();
        return;
    }

    Vector3 target_direction{};
    double target_speed = 0.0;
    double body_direction_error = 0.0;
    if (!Normalize(
            frame.opponent.velocity_ned_mps,
            target_direction,
            target_speed)
        || !BodyVelocityDirectionResolution(
            frame.opponent.velocity_body_mps,
            body_direction_error))
    {
        ResetPhysicalHistory();
        output.reason = ObfmEntryEstablishedTurnReason::CircleUnobservable;
        cache_output();
        return;
    }

    double next_observer_time = 0.0;
    double oldest_time = 0.0;
    if (!CheckedAdd(
            turn_observer_time_s_,
            input.dt_s,
            next_observer_time)
        || !CheckedSubtract(
            next_observer_time,
            official_history_s,
            oldest_time))
    {
        ResetPhysicalHistory();
        output.reason = ObfmEntryEstablishedTurnReason::
            DeclaredReadyFrameNonfinite;
        status.code = StatusCode::NonFiniteInput;
        cache_output();
        return;
    }
    turn_observer_time_s_ = next_observer_time;
    while (turn_history_count_ > 0U
        && turn_history_[turn_history_head_].time_s < oldest_time)
    {
        turn_history_head_ =
            (turn_history_head_ + 1U) % turn_history_.size();
        --turn_history_count_;
    }

    TurnSupportSample sample{};
    sample.time_s = turn_observer_time_s_;
    sample.direction_ned = target_direction;
    sample.body_direction_error_bound_rad = body_direction_error;
    bool appended = false;
    AppendTurnSample(sample, appended);
    if (!appended)
    {
        ResetPhysicalHistory();
        output.reason =
            ObfmEntryEstablishedTurnReason::HistoryCapacityExceeded;
        status.code = StatusCode::ObservationInvalid;
        cache_output();
        return;
    }

    std::size_t recent_anchor = 0U;
    BuildSupportChord(
        turn_history_count_ - 1U,
        p1.value.angle_rad,
        recent_anchor,
        output.recent_chord);
    if (!output.recent_chord.valid)
    {
        output.reason = ObfmEntryEstablishedTurnReason::
            RecentOfficialChordUnavailable;
        cache_output();
        return;
    }
    std::size_t older_anchor = 0U;
    BuildSupportChord(
        recent_anchor,
        p1.value.angle_rad,
        older_anchor,
        output.older_chord);
    if (!output.older_chord.valid)
    {
        output.reason = ObfmEntryEstablishedTurnReason::
            OlderOfficialChordUnavailable;
        cache_output();
        return;
    }

    const bool rate_intervals_overlap = (std::max)(
        output.older_chord.turn_rate_lower_rad_s,
        output.recent_chord.turn_rate_lower_rad_s)
        <= (std::min)(
            output.older_chord.turn_rate_upper_rad_s,
            output.recent_chord.turn_rate_upper_rad_s);
    double normal_dot = 0.0;
    double cone_sum = 0.0;
    if (!CheckedDot(
            output.older_chord.plane_normal_ned,
            output.recent_chord.plane_normal_ned,
            normal_dot)
        || !CheckedAdd(
            output.older_chord.plane_axis_error_bound_rad,
            output.recent_chord.plane_axis_error_bound_rad,
            cone_sum))
    {
        output.reason = ObfmEntryEstablishedTurnReason::CircleUnobservable;
        cache_output();
        return;
    }
    normal_dot = (std::max)(-1.0, (std::min)(1.0, normal_dot));
    const double plane_separation = std::acos(normal_dot);
    const bool plane_cones_overlap = Finite(plane_separation)
        && plane_separation <= cone_sum;
    if (!rate_intervals_overlap)
    {
        output.reason = ObfmEntryEstablishedTurnReason::
            TurnRateIntervalsDisjoint;
        cache_output();
        return;
    }
    if (!plane_cones_overlap)
    {
        output.reason = ObfmEntryEstablishedTurnReason::
            TurnPlaneConesDisjoint;
        cache_output();
        return;
    }

    double rate_resolution = 0.0;
    Vector3 omega{};
    if (!CheckedDivide(
            output.recent_chord.endpoint_direction_error_bound_rad,
            output.recent_chord.duration_s,
            rate_resolution)
        || !CheckedScale(
            output.recent_chord.plane_normal_ned,
            output.recent_chord.mean_turn_rate_rad_s,
            omega)
        || !BuildTargetCircle(
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            omega,
            rate_resolution,
            output.circle))
    {
        ResetPhysicalHistory();
        output.reason = ObfmEntryEstablishedTurnReason::CircleUnobservable;
        status.code = StatusCode::ObservationInvalid;
        cache_output();
        return;
    }

    output.admitted = true;
    output.reason = ObfmEntryEstablishedTurnReason::
        TwoOfficialChordsConsistent;
    if (older_anchor > 0U)
    {
        for (std::size_t removed = 0U; removed < older_anchor; ++removed)
        {
            turn_history_head_ =
                (turn_history_head_ + 1U) % turn_history_.size();
            --turn_history_count_;
        }
    }
    cache_output();
}

ObfmEntryWindowAdmission::ObfmEntryWindowAdmission() noexcept
{
    ResetEpisode();
}

void ObfmEntryWindowAdmission::ResetObservation() noexcept
{
    previous_target_direction_available_ = false;
    previous_target_direction_ned_ = Vector3{};
    previous_body_direction_resolution_available_ = false;
    previous_body_direction_resolution_rad_ = 0.0;
    ClearGeometryHistory(
        previous_entry_point_available_,
        previous_entry_point_ned_m_,
        previous_passage_sample_available_,
        previous_passage_sample_);
}

void ObfmEntryWindowAdmission::ClearOwner() noexcept
{
    owner_active_ = false;
    completed_this_tick_ = false;
    completion_kind_ = ObfmEntrySetupCompletionKind::None;
    pending_halt_reason_ =
        ObfmEntryWindowReason::EntrySetupTreePreempted;
}

void ObfmEntryWindowAdmission::ResetEpisode() noexcept
{
    ResetObservation();
    ClearOwner();
}

void ObfmEntryWindowAdmission::ResetOwnerBranch() noexcept
{
    ClearOwner();
}

void ObfmEntryWindowAdmission::Observe(
    const DogfightGeometryFrame& frame,
    const ObfmEntryWindowObservationInput& input,
    ObfmEntryWindowObservationReceipt& output,
    Status& status) noexcept
{
    output = ObfmEntryWindowObservationReceipt{};
    output.frame_identity = frame.frame_identity;
    output.evaluated = true;
    status = Status{};

    if (!input.frame_evidence_available)
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::FrameEvidenceUnavailable;
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        ResetObservation();
        output.reason =
            ObfmEntryWindowReason::DeclaredReadyFrameIdentityInvalid;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!frame.target_same_index)
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::FrameEvidenceUnavailable;
        return;
    }
    if (frame.target_frame_index != frame.frame_identity.frame_index)
    {
        ResetObservation();
        output.reason =
            ObfmEntryWindowReason::DeclaredReadyFrameIdentityInvalid;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!Finite(input.dt_s))
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::DeclaredReadyFrameNonfinite;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (input.dt_s <= 0.0)
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::DtUnavailable;
        return;
    }
    if (!FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps)
        || !FiniteVector(frame.opponent.velocity_body_mps)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps))
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::DeclaredReadyFrameNonfinite;
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    Vector3 target_direction{};
    double target_speed = 0.0;
    if (!Normalize(
            frame.opponent.velocity_ned_mps,
            target_direction,
            target_speed))
    {
        ResetObservation();
        output.reason =
            ObfmEntryWindowReason::TargetWorldVelocityUnavailable;
        return;
    }
    double body_resolution = 0.0;
    if (!BodyVelocityDirectionResolution(
            frame.opponent.velocity_body_mps,
            body_resolution))
    {
        ResetObservation();
        output.reason =
            ObfmEntryWindowReason::TargetBodyVelocityUnavailable;
        return;
    }
    double attitude_resolution = 0.0;
    if (!AttitudeRateResolution(input.dt_s, attitude_resolution))
    {
        ResetObservation();
        output.reason = ObfmEntryWindowReason::DtUnavailable;
        return;
    }

    const bool had_previous = previous_target_direction_available_
        && previous_body_direction_resolution_available_;
    const Vector3 previous_direction = previous_target_direction_ned_;
    const double previous_body_resolution =
        previous_body_direction_resolution_rad_;
    previous_target_direction_available_ = true;
    previous_target_direction_ned_ = target_direction;
    previous_body_direction_resolution_available_ = true;
    previous_body_direction_resolution_rad_ = body_resolution;

    if (!had_previous)
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.admission_rate_resolution_rad_s = attitude_resolution;
        output.reason = ObfmEntryWindowReason::TargetPathRateInit;
        return;
    }

    Vector3 cross{};
    double sine = 0.0;
    double cosine = 0.0;
    if (!CheckedCross(previous_direction, target_direction, cross)
        || !VectorNorm(cross, sine)
        || !CheckedDot(previous_direction, target_direction, cosine))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason =
            ObfmEntryWindowReason::TargetPathRotationAxisUnavailable;
        return;
    }
    cosine = std::min(1.0, std::max(-1.0, cosine));
    const double rotation = std::atan2(sine, cosine);
    double endpoint_body_resolution = 0.0;
    double endpoint_rate_resolution = 0.0;
    double total_rate_resolution = 0.0;
    double turn_rate = 0.0;
    double twice_resolution = 0.0;
    if (!Finite(rotation) || rotation < 0.0
        || !CheckedAdd(
            previous_body_resolution,
            body_resolution,
            endpoint_body_resolution)
        || !CheckedDivide(
            endpoint_body_resolution,
            input.dt_s,
            endpoint_rate_resolution)
        || !CheckedAdd(
            attitude_resolution,
            endpoint_rate_resolution,
            total_rate_resolution)
        || !CheckedDivide(rotation, input.dt_s, turn_rate)
        || !CheckedMultiply(2.0, total_rate_resolution, twice_resolution))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason =
            ObfmEntryWindowReason::TargetPathRotationAxisUnavailable;
        return;
    }
    output.target_path_rate_valid = true;
    output.target_path_relative_rotation_rad = rotation;
    output.target_path_rate_rad_s = turn_rate;
    output.admission_rate_resolution_rad_s = total_rate_resolution;
    if (turn_rate <= twice_resolution)
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason =
            ObfmEntryWindowReason::TargetPathRateUnderResolution;
        return;
    }
    if (sine <= 0.0)
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason =
            ObfmEntryWindowReason::TargetPathRotationAxisUnavailable;
        return;
    }

    Vector3 axis{};
    double unused_axis_magnitude = 0.0;
    Vector3 omega{};
    if (!Normalize(cross, axis, unused_axis_magnitude)
        || !CheckedScale(axis, turn_rate, omega))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason =
            ObfmEntryWindowReason::TargetPathRotationAxisUnavailable;
        return;
    }
    output.target_path_rate_feature_ready = true;
    output.target_path_omega_ned_rad_s = omega;

    ObfmEntryTargetTurnCircle circle{};
    if (!BuildTargetCircle(
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            omega,
            total_rate_resolution,
            circle))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = ObfmEntryWindowReason::TargetCircleUnavailable;
        return;
    }

    ObfmEntryWindowReason geometry_reason =
        ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
    ObfmEntryWindowGeometry geometry{};
    if (!BuildEntryGeometry(
            circle,
            frame.own.position_ned_m,
            frame.own.velocity_ned_mps,
            geometry,
            geometry_reason))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = geometry_reason;
        return;
    }
    output.geometry_available = true;
    output.geometry = geometry;

    ObfmEntryWindowPassageSample passage{};
    if (!BuildPassageSample(geometry, passage))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.geometry_available = false;
        output.geometry = ObfmEntryWindowGeometry{};
        output.reason =
            ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return;
    }
    output.passage_sample = passage;
    if (previous_passage_sample_available_)
    {
        BuildPassageEvent(
            previous_passage_sample_,
            passage,
            output.passage_event);
    }
    previous_passage_sample_available_ = true;
    previous_passage_sample_ = passage;

    if (!geometry.ahead)
    {
        previous_entry_point_available_ = false;
        previous_entry_point_ned_m_ = Vector3{};
        output.reason = ObfmEntryWindowReason::EntryWindowNotAhead;
        return;
    }

    if (!previous_entry_point_available_)
    {
        previous_entry_point_available_ = true;
        previous_entry_point_ned_m_ = geometry.entry_point_ned_m;
        output.reason = ObfmEntryWindowReason::EntryPointVelocityInit;
        return;
    }
    previous_entry_point_ned_m_ = geometry.entry_point_ned_m;
    output.entry_point_velocity_available = true;
    output.entry_point_velocity_ned_mps = circle.target_velocity_ned_mps;
    output.admitted = true;
    output.reason = ObfmEntryWindowReason::EntryWindowAhead;
}

void ObfmEntryWindowAdmission::ObserveFromEstablishedTurn(
    const DogfightGeometryFrame& frame,
    const ObfmEntryWindowObservationInput& input,
    const ObfmEntryEstablishedTurnReceipt& established_turn,
    ObfmEntryWindowObservationReceipt& output,
    Status& status) noexcept
{
    output = ObfmEntryWindowObservationReceipt{};
    output.frame_identity = frame.frame_identity;
    output.evaluated = true;
    status = Status{};
    if (!input.frame_evidence_available)
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = ObfmEntryWindowReason::FrameEvidenceUnavailable;
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !frame.target_same_index
        || frame.target_frame_index != frame.frame_identity.frame_index
        || !established_turn.evaluated
        || !SameControlFrameIdentity(
            established_turn.frame_identity,
            frame.frame_identity))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = ObfmEntryWindowReason::
            DeclaredReadyFrameIdentityInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!established_turn.admitted)
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = ObfmEntryWindowReason::TurnCircleNotEstablished;
        return;
    }
    const ObfmEntryTargetTurnCircle& circle = established_turn.circle;
    if (!FiniteTargetCircle(circle)
        || !established_turn.recent_chord.valid
        || !Finite(established_turn.recent_chord.rotation_rad)
        || !Finite(established_turn.recent_chord.mean_turn_rate_rad_s)
        || !Finite(circle.observer_rate_resolution_rad_s))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = ObfmEntryWindowReason::ServiceReceiptContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.target_path_rate_valid = true;
    output.target_path_rate_feature_ready = true;
    output.target_path_relative_rotation_rad =
        established_turn.recent_chord.rotation_rad;
    output.target_path_rate_rad_s =
        established_turn.recent_chord.mean_turn_rate_rad_s;
    output.admission_rate_resolution_rad_s =
        circle.observer_rate_resolution_rad_s;
    output.target_path_omega_ned_rad_s = circle.target_omega_ned_rad_s;

    ObfmEntryWindowReason geometry_reason =
        ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
    ObfmEntryWindowGeometry geometry{};
    if (!BuildEntryGeometry(
            circle,
            frame.own.position_ned_m,
            frame.own.velocity_ned_mps,
            geometry,
            geometry_reason))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.reason = geometry_reason;
        return;
    }
    output.geometry_available = true;
    output.geometry = geometry;

    ObfmEntryWindowPassageSample passage{};
    if (!BuildPassageSample(geometry, passage))
    {
        ClearGeometryHistory(
            previous_entry_point_available_,
            previous_entry_point_ned_m_,
            previous_passage_sample_available_,
            previous_passage_sample_);
        output.geometry_available = false;
        output.geometry = ObfmEntryWindowGeometry{};
        output.reason = ObfmEntryWindowReason::EntryWindowGeometryUnavailable;
        return;
    }
    output.passage_sample = passage;
    if (previous_passage_sample_available_)
    {
        BuildPassageEvent(
            previous_passage_sample_,
            passage,
            output.passage_event);
    }
    previous_passage_sample_available_ = true;
    previous_passage_sample_ = passage;

    if (!geometry.ahead)
    {
        previous_entry_point_available_ = false;
        previous_entry_point_ned_m_ = Vector3{};
        output.reason = ObfmEntryWindowReason::EntryWindowNotAhead;
        return;
    }
    if (!previous_entry_point_available_)
    {
        previous_entry_point_available_ = true;
        previous_entry_point_ned_m_ = geometry.entry_point_ned_m;
        output.reason = ObfmEntryWindowReason::EntryPointVelocityInit;
        return;
    }
    previous_entry_point_ned_m_ = geometry.entry_point_ned_m;
    output.entry_point_velocity_available = true;
    output.entry_point_velocity_ned_mps = circle.target_velocity_ned_mps;
    output.admitted = true;
    output.reason = ObfmEntryWindowReason::EntryWindowAhead;
}

void ObfmEntryWindowAdmission::EvaluateService(
    const ObfmEntrySetupServiceInput& input,
    const ObfmEntryWindowObservationReceipt& observation,
    ObfmEntrySetupServiceReceipt& output,
    Status& status) noexcept
{
    output = ObfmEntrySetupServiceReceipt{};
    output.frame_identity = observation.frame_identity;
    output.owner_was_active = owner_active_;
    status = Status{};
    completed_this_tick_ = false;
    completion_kind_ = ObfmEntrySetupCompletionKind::None;

    if (!input.selector_service_reached)
    {
        output.reason =
            ObfmEntryWindowReason::SelectorServiceNotReached;
        return;
    }
    output.service_evaluated = true;
    if (!input.feature_enabled)
    {
        output.reason = ObfmEntryWindowReason::FeatureDisabled;
        if (owner_active_)
        {
            pending_halt_reason_ = output.reason;
        }
        return;
    }
    output.enabled_result = true;
    if (!input.spacing_owner_enabled)
    {
        output.enabled_result = false;
        output.reason =
            ObfmEntryWindowReason::SpacingOwnerDependencyDisabled;
        if (owner_active_)
        {
            pending_halt_reason_ = output.reason;
        }
        return;
    }

    if (observation.admitted
        && !AdmittedObservationContractValid(observation))
    {
        output.reason = ObfmEntryWindowReason::ServiceReceiptContradiction;
        status.code = StatusCode::InvalidConfiguration;
        if (owner_active_)
        {
            pending_halt_reason_ = output.reason;
        }
        return;
    }

    if (owner_active_ && input.spacing_handoff_deferred_current_energy)
    {
        if (!observation.admitted)
        {
            output.reason = observation.reason;
            pending_halt_reason_ = observation.reason;
            return;
        }
        if (!input.safety_evidence_available)
        {
            output.reason =
                ObfmEntryWindowReason::SafetyEvidenceUnavailable;
            pending_halt_reason_ = output.reason;
            return;
        }
        if (!input.safety_admitted)
        {
            output.reason = ObfmEntryWindowReason::SafetyNotAdmitted;
            pending_halt_reason_ = output.reason;
            return;
        }
        output.selected_result = true;
        output.reason =
            ObfmEntryWindowReason::SpacingHandoffDeferredCurrentEnergy;
        pending_halt_reason_ =
            ObfmEntryWindowReason::EntrySetupTreePreempted;
        return;
    }

    const ObfmEntrySetupCompletionKind completion = owner_active_
        ? CompletionKind(observation)
        : ObfmEntrySetupCompletionKind::None;
    if (completion != ObfmEntrySetupCompletionKind::None)
    {
        if (!input.safety_evidence_available)
        {
            output.reason =
                ObfmEntryWindowReason::SafetyEvidenceUnavailable;
            pending_halt_reason_ = output.reason;
            return;
        }
        if (!input.safety_admitted)
        {
            output.reason = ObfmEntryWindowReason::SafetyNotAdmitted;
            pending_halt_reason_ = output.reason;
            return;
        }
        completed_this_tick_ = true;
        completion_kind_ = completion;
        output.completed_this_tick = true;
        output.completion_kind = completion;
        output.reason = completion
                == ObfmEntrySetupCompletionKind::ProjectedSegmentPassage
            ? ObfmEntryWindowReason::EntrySetupCompletedProjectedPassage
            : ObfmEntryWindowReason::
                EntrySetupCompletedOutsideSegmentDevelopment;
        pending_halt_reason_ =
            ObfmEntryWindowReason::EntrySetupTreePreempted;
        return;
    }

    if (!observation.admitted)
    {
        output.reason = observation.reason;
        if (owner_active_)
        {
            pending_halt_reason_ = observation.reason;
        }
        return;
    }
    if (!input.safety_evidence_available)
    {
        output.reason = ObfmEntryWindowReason::SafetyEvidenceUnavailable;
        if (owner_active_)
        {
            pending_halt_reason_ = output.reason;
        }
        return;
    }
    if (!input.safety_admitted)
    {
        output.reason = ObfmEntryWindowReason::SafetyNotAdmitted;
        if (owner_active_)
        {
            pending_halt_reason_ = output.reason;
        }
        return;
    }
    output.selected_result = true;
    output.reason = ObfmEntryWindowReason::EntrySetupSelected;
    pending_halt_reason_ =
        ObfmEntryWindowReason::EntrySetupTreePreempted;
}

void ObfmEntryWindowAdmission::EnterOwner(
    const ObfmEntrySetupServiceReceipt& service,
    Status& status) noexcept
{
    status = Status{};
    if (owner_active_ || !service.service_evaluated
        || !service.enabled_result || !service.selected_result
        || !IsValidControlFrameIdentity(service.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    owner_active_ = true;
    completed_this_tick_ = false;
    completion_kind_ = ObfmEntrySetupCompletionKind::None;
    pending_halt_reason_ =
        ObfmEntryWindowReason::EntrySetupTreePreempted;
}

void ObfmEntryWindowAdmission::TickOwner(
    const ObfmEntrySetupServiceReceipt& service,
    const ObfmEntryWindowObservationReceipt& observation,
    ObfmEntrySetupTaskReceipt& output,
    Status& status) noexcept
{
    output = ObfmEntrySetupTaskReceipt{};
    output.frame_identity = observation.frame_identity;
    output.task_active = owner_active_;
    status = Status{};
    if (!owner_active_)
    {
        output.reason = ObfmEntryWindowReason::EntrySetupInactive;
        return;
    }
    if (!service.selected_result)
    {
        output.reason = observation.admitted
            ? service.reason
            : ObfmEntryWindowReason::EntryObservationLost;
        return;
    }
    if (!service.service_evaluated || !service.enabled_result
        || !AdmittedObservationContractValid(observation)
        || !SameControlFrameIdentity(
            service.frame_identity,
            observation.frame_identity))
    {
        output.reason = ObfmEntryWindowReason::TaskLifecycleContradiction;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.producer_ready = true;
    output.producer_count = 1U;
    output.reason = service.owner_was_active
        ? ObfmEntryWindowReason::EntrySetupContinued
        : ObfmEntryWindowReason::EntrySetupEntered;
}

void ObfmEntryWindowAdmission::HaltOwner(
    const ObfmEntrySetupHaltCause cause,
    ObfmEntrySetupHaltReceipt& output) noexcept
{
    output = ObfmEntrySetupHaltReceipt{};
    output.valid = true;
    output.was_active = owner_active_;
    if (!owner_active_)
    {
        output.reason = ObfmEntryWindowReason::EntrySetupInactive;
        ClearOwner();
        return;
    }

    switch (cause)
    {
    case ObfmEntrySetupHaltCause::SpacingOwnerActive:
        output.reason = ObfmEntryWindowReason::SpacingOwnerHandoff;
        break;
    case ObfmEntrySetupHaltCause::CompletedThisTick:
        output.reason = completion_kind_
                == ObfmEntrySetupCompletionKind::ProjectedSegmentPassage
            ? ObfmEntryWindowReason::EntrySetupCompletedProjectedPassage
            : ObfmEntryWindowReason::
                EntrySetupCompletedOutsideSegmentDevelopment;
        break;
    case ObfmEntrySetupHaltCause::OfficialEmployActive:
        output.released_this_tick = true;
        output.reason = ObfmEntryWindowReason::OfficialEmployPreemption;
        break;
    case ObfmEntrySetupHaltCause::OtherPreemption:
        output.released_this_tick = true;
        output.append_abort_reason = true;
        output.reason = pending_halt_reason_;
        break;
    }
    ClearOwner();
}

bool ObfmEntryWindowAdmission::owner_active() const noexcept
{
    return owner_active_;
}

ObfmEntryWindowReason
ObfmEntryWindowAdmission::pending_halt_reason() const noexcept
{
    return pending_halt_reason_;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
