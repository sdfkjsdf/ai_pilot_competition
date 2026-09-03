#include "LadyLuck/guidance/committed/G16HighObservation.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::Vector3;

constexpr double kBodyVelocityQuantumMps = 0.001 * 0.3048;
constexpr double kRpyQuantumRad =
    LadyLuck::constants::Pi / (180.0 * 1000.0);
constexpr double kOfficialSupportAngleRad =
    LadyLuck::constants::Pi / 180.0;
constexpr double kOfficialHistorySeconds = 100.0;

bool Finite(const Vector3& value) noexcept
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
        scalar * value[0],
        scalar * value[1],
        scalar * value[2]}};
}

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Norm(const Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

bool Unit(const Vector3& value, Vector3& output) noexcept
{
    output = Vector3{};
    const double magnitude = Norm(value);
    if (!Finite(value) || !std::isfinite(magnitude) || magnitude <= 0.0)
    {
        return false;
    }
    output = Scale(value, 1.0 / magnitude);
    return Finite(output);
}

double NextUp(const double value) noexcept
{
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

bool Float32Cell(
    const Vector3& value,
    const bool half_cell,
    Vector3& represented,
    Vector3& cell) noexcept
{
    represented = Vector3{};
    cell = Vector3{};
    if (!Finite(value))
    {
        return false;
    }
    const float positive_infinity =
        std::numeric_limits<float>::infinity();
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
        const double width = (std::max)(
            std::abs(static_cast<double>(upper) - represented[axis]),
            std::abs(represented[axis] - static_cast<double>(lower)));
        cell[axis] = half_cell ? 0.5 * width : width;
    }
    return Finite(represented) && Finite(cell);
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    Vector3 represented{};
    Vector3 full_cell{};
    if (!Float32Cell(value, false, represented, full_cell))
    {
        return false;
    }
    const double residual = NextUp(Norm(Subtract(value, represented)));
    const double cell = NextUp(Norm(full_cell));
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
    Vector3 cell{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        const double upper = std::nextafter(
            value[axis],
            std::numeric_limits<double>::infinity());
        const double lower = std::nextafter(
            value[axis],
            -std::numeric_limits<double>::infinity());
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        cell[axis] = (std::max)(
            std::abs(upper - value[axis]),
            std::abs(value[axis] - lower));
    }
    output = NextUp(Norm(cell));
    return std::isfinite(output) && output >= 0.0;
}

bool BodyVelocityResolution(
    const Vector3& body_velocity,
    double& norm_error_bound_mps,
    double& direction_error_bound_rad) noexcept
{
    norm_error_bound_mps = 0.0;
    direction_error_bound_rad = 0.0;
    Vector3 represented{};
    Vector3 half_cell{};
    if (!Float32Cell(body_velocity, true, represented, half_cell))
    {
        return false;
    }
    const double speed = Norm(body_velocity);
    norm_error_bound_mps =
        std::sqrt(3.0) * kBodyVelocityQuantumMps + Norm(half_cell);
    const double true_speed_lower = speed - norm_error_bound_mps;
    if (!std::isfinite(speed)
        || !std::isfinite(norm_error_bound_mps)
        || true_speed_lower <= 0.0)
    {
        return false;
    }
    direction_error_bound_rad = std::asin((std::min)(
        1.0,
        norm_error_bound_mps / true_speed_lower));
    return std::isfinite(direction_error_bound_rad)
        && direction_error_bound_rad >= 0.0;
}

bool WorldVelocityErrorBound(
    const LadyLuck::AircraftGeometryKinematics& aircraft,
    double& output) noexcept
{
    output = 0.0;
    double body_wire_error = 0.0;
    double world_roundoff = 0.0;
    if (!Float32WireVectorErrorBound(
            aircraft.velocity_body_mps,
            body_wire_error)
        || !Binary64VectorRoundoffBound(
            aircraft.velocity_ned_mps,
            world_roundoff))
    {
        return false;
    }
    const double body_error = body_wire_error
        + std::sqrt(3.0) * kBodyVelocityQuantumMps;
    const double attitude_error = 3.0 * kRpyQuantumRad;
    const double rotation_error = 2.0
        * (Norm(aircraft.velocity_body_mps) + body_error)
        * std::sin(0.5 * attitude_error);
    output = body_error + rotation_error + world_roundoff;
    return std::isfinite(output) && output >= 0.0;
}

bool CrossDirectionResolution(
    const Vector3& first,
    const Vector3& second,
    const double first_bound_rad,
    const double second_bound_rad,
    double& output) noexcept
{
    output = 0.0;
    const double cross_magnitude = Norm(Cross(first, second));
    const double chord_error = 2.0 * std::sin(0.5 * first_bound_rad)
        + 2.0 * std::sin(0.5 * second_bound_rad);
    const double lower_magnitude = cross_magnitude - chord_error;
    if (!std::isfinite(cross_magnitude)
        || !std::isfinite(chord_error)
        || lower_magnitude <= 0.0)
    {
        return false;
    }
    output = 2.0 * std::asin((std::min)(
        1.0,
        chord_error / lower_magnitude));
    return std::isfinite(output) && output >= 0.0;
}

bool VelocityNormalDirection(
    const Vector3& value,
    const Vector3& velocity_hat,
    Vector3& output) noexcept
{
    return Unit(
        Subtract(value, Scale(velocity_hat, Dot(value, velocity_hat))),
        output);
}

bool SourceContractValid(
    const LadyLuck::runtime::TacticalCommandBuildInput& input) noexcept
{
    return input.valid
        && LadyLuck::IsValidControlFrameIdentity(input.frame.frame_identity)
        && input.frame.own_plane_id >= 0
        && input.frame.target_plane_id >= 0
        && input.frame.target_plane_id != input.frame.own_plane_id
        && input.frame.target_same_index
        && input.frame.target_frame_index
            == input.frame.frame_identity.frame_index
        && std::isfinite(input.frame.t_sec)
        && input.frame.t_sec >= 0.0
        && std::isfinite(input.accepted_estimator.sample_dt_s)
        && input.accepted_estimator.sample_dt_s > 0.0
        && Finite(input.frame.own.position_ned_m)
        && Finite(input.frame.opponent.position_ned_m)
        && Finite(input.frame.own.velocity_body_mps)
        && Finite(input.frame.opponent.velocity_body_mps)
        && Finite(input.frame.own.velocity_ned_mps)
        && Finite(input.frame.opponent.velocity_ned_mps)
        && Finite(input.frame.own.down_ned);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

void G16HighObservation::ResetPhysicalHistory() noexcept
{
    turn_history_head_ = 0U;
    turn_history_count_ = 0U;
    turn_observer_time_s_ = 0.0;
    apex_previous_sample_valid_ = false;
    apex_climb_armed_ = false;
}

void G16HighObservation::Reset() noexcept
{
    ResetPhysicalHistory();
    cached_identity_valid_ = false;
    cached_identity_ = ControlFrameIdentity{};
    cached_own_plane_id_ = -1;
    cached_target_plane_id_ = -1;
    cached_receipt_ = G16HighObservationReceipt{};
    cached_status_code_ = StatusCode::Seeded;
}

void G16HighObservation::AppendTurnSample(
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

void G16HighObservation::BuildSupportChord(
    const std::size_t end_offset,
    std::size_t& anchor_offset,
    G16TurnChordReceipt& output) const noexcept
{
    output = G16TurnChordReceipt{};
    anchor_offset = 0U;
    if (end_offset == 0U || end_offset >= turn_history_count_)
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
        const Vector3 cross = Cross(anchor.direction_ned, end.direction_ned);
        const double sine = Norm(cross);
        const double cosine = (std::max)(
            -1.0,
            (std::min)(1.0, Dot(anchor.direction_ned, end.direction_ned)));
        const double rotation = std::atan2(sine, cosine);
        if (!std::isfinite(rotation) || rotation < kOfficialSupportAngleRad)
        {
            continue;
        }
        const double endpoint_error = 6.0 * kRpyQuantumRad
            + anchor.body_direction_error_bound_rad
            + end.body_direction_error_bound_rad;
        if (!std::isfinite(endpoint_error)
            || sine <= 0.0
            || rotation <= 2.0 * endpoint_error)
        {
            continue;
        }
        const double delta_cross = (std::min)(2.0, endpoint_error);
        const double true_cross_lower = std::sin(rotation) - delta_cross;
        if (!std::isfinite(true_cross_lower) || true_cross_lower <= 0.0)
        {
            continue;
        }
        const double axis_error = std::asin((std::min)(
            1.0,
            delta_cross / true_cross_lower));
        const double duration = end.time_s - anchor.time_s;
        if (!std::isfinite(axis_error)
            || !std::isfinite(duration)
            || duration <= 0.0)
        {
            continue;
        }
        const double mean_rate = rotation / duration;
        const double rate_error = endpoint_error / duration;
        const double lower_rate = (std::max)(0.0, mean_rate - rate_error);
        const double upper_rate = mean_rate + rate_error;
        const Vector3 normal = Scale(cross, 1.0 / sine);
        if (!std::isfinite(mean_rate)
            || !std::isfinite(rate_error)
            || !std::isfinite(lower_rate)
            || !std::isfinite(upper_rate)
            || !Finite(normal))
        {
            continue;
        }
        output.valid = true;
        output.duration_s = duration;
        output.rotation_rad = rotation;
        output.endpoint_direction_error_bound_rad = endpoint_error;
        output.plane_normal_ned = normal;
        output.mean_turn_rate_radps = mean_rate;
        output.turn_rate_lower_radps = lower_rate;
        output.turn_rate_upper_radps = upper_rate;
        output.plane_axis_error_bound_rad = axis_error;
        anchor_offset = current_anchor;
        return;
    }
}

void G16HighObservation::ObserveEstablishedTurn(
    const runtime::TacticalCommandBuildInput& input,
    const double body_speed_error_bound_mps,
    const double body_direction_error_bound_rad,
    G16EstablishedTurnCircleReceipt& output,
    Status& status) noexcept
{
    output = G16EstablishedTurnCircleReceipt{};
    status = Status{};
    Vector3 target_direction{};
    if (!Unit(input.frame.opponent.velocity_ned_mps, target_direction)
        || !std::isfinite(body_speed_error_bound_mps)
        || body_speed_error_bound_mps < 0.0
        || !std::isfinite(body_direction_error_bound_rad)
        || body_direction_error_bound_rad < 0.0)
    {
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    const double next_observer_time =
        turn_observer_time_s_ + input.accepted_estimator.sample_dt_s;
    if (!std::isfinite(next_observer_time))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    turn_observer_time_s_ = next_observer_time;
    const double oldest_time = turn_observer_time_s_ - kOfficialHistorySeconds;
    while (turn_history_count_ > 0U
        && turn_history_[turn_history_head_].time_s < oldest_time)
    {
        turn_history_head_ = (turn_history_head_ + 1U) % turn_history_.size();
        --turn_history_count_;
    }

    TurnSupportSample sample{};
    sample.time_s = turn_observer_time_s_;
    sample.direction_ned = target_direction;
    sample.body_direction_error_bound_rad = body_direction_error_bound_rad;
    bool appended = false;
    AppendTurnSample(sample, appended);
    output.evaluated = true;
    if (!appended)
    {
        output.reason = G16EstablishedTurnReason::HistoryCapacityExceeded;
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    std::size_t recent_anchor = 0U;
    BuildSupportChord(
        turn_history_count_ - 1U,
        recent_anchor,
        output.recent_chord);
    if (!output.recent_chord.valid)
    {
        output.reason =
            G16EstablishedTurnReason::RecentOfficialChordUnavailable;
        return;
    }
    std::size_t older_anchor = 0U;
    BuildSupportChord(recent_anchor, older_anchor, output.older_chord);
    if (!output.older_chord.valid)
    {
        output.reason =
            G16EstablishedTurnReason::OlderOfficialChordUnavailable;
        return;
    }

    const bool rate_intervals_overlap = (std::max)(
        output.older_chord.turn_rate_lower_radps,
        output.recent_chord.turn_rate_lower_radps)
        <= (std::min)(
            output.older_chord.turn_rate_upper_radps,
            output.recent_chord.turn_rate_upper_radps);
    const double normal_dot = (std::max)(
        -1.0,
        (std::min)(
            1.0,
            Dot(
                output.older_chord.plane_normal_ned,
                output.recent_chord.plane_normal_ned)));
    const double plane_separation = std::acos(normal_dot);
    const bool plane_cones_overlap = std::isfinite(plane_separation)
        && plane_separation <= (
            output.older_chord.plane_axis_error_bound_rad
            + output.recent_chord.plane_axis_error_bound_rad);
    if (!rate_intervals_overlap)
    {
        output.reason = G16EstablishedTurnReason::TurnRateIntervalsDisjoint;
        return;
    }
    if (!plane_cones_overlap)
    {
        output.reason = G16EstablishedTurnReason::TurnPlaneConesDisjoint;
        return;
    }

    const double resolution =
        output.recent_chord.endpoint_direction_error_bound_rad
        / output.recent_chord.duration_s;
    const Vector3 omega = Scale(
        output.recent_chord.plane_normal_ned,
        output.recent_chord.mean_turn_rate_radps);
    const Vector3 centre_velocity = Cross(
        omega,
        input.frame.opponent.velocity_ned_mps);
    Vector3 centre_direction{};
    Vector3 plane_normal{};
    if (!Unit(centre_velocity, centre_direction)
        || !Unit(
            Cross(
                input.frame.opponent.velocity_ned_mps,
                centre_velocity),
            plane_normal))
    {
        output.reason = G16EstablishedTurnReason::CircleUnobservable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    const double target_speed = Norm(input.frame.opponent.velocity_ned_mps);
    const double normal_rate = Dot(omega, plane_normal);
    if (!std::isfinite(resolution)
        || !std::isfinite(target_speed)
        || target_speed <= 0.0
        || !std::isfinite(normal_rate)
        || normal_rate <= resolution)
    {
        output.reason = G16EstablishedTurnReason::CircleUnobservable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    const double radius = target_speed / normal_rate;
    if (!std::isfinite(radius) || radius <= 0.0)
    {
        output.reason = G16EstablishedTurnReason::CircleUnobservable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }
    const Vector3 centre = Add(
        input.frame.opponent.position_ned_m,
        Scale(centre_direction, radius));
    if (!Finite(centre))
    {
        output.reason = G16EstablishedTurnReason::CircleUnobservable;
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    output.admitted = true;
    output.reason = G16EstablishedTurnReason::TwoOfficialChordsConsistent;
    output.plane_normal_ned = plane_normal;
    output.centre_direction_ned = centre_direction;
    output.circle_centre_ned_m = centre;
    output.radius_m = radius;
    output.target_speed_mps = target_speed;
    output.target_speed_error_bound_mps = body_speed_error_bound_mps;
    output.normal_turn_rate_radps = normal_rate;
    output.observer_rate_resolution_radps = resolution;

    if (older_anchor > 0U)
    {
        for (std::size_t removed = 0U; removed < older_anchor; ++removed)
        {
            turn_history_head_ =
                (turn_history_head_ + 1U) % turn_history_.size();
            --turn_history_count_;
        }
    }
}

void G16HighObservation::ObserveApex(
    const runtime::TacticalCommandBuildInput& input,
    const double own_world_velocity_error_bound_mps,
    G16ApexObservationReceipt& output,
    Status& status) noexcept
{
    output = G16ApexObservationReceipt{};
    status = Status{};
    const double vertical_down = input.frame.own.velocity_ned_mps[2];
    const double lower = std::nextafter(
        vertical_down - own_world_velocity_error_bound_mps,
        -std::numeric_limits<double>::infinity());
    const double upper = std::nextafter(
        vertical_down + own_world_velocity_error_bound_mps,
        std::numeric_limits<double>::infinity());
    if (!std::isfinite(vertical_down)
        || !std::isfinite(own_world_velocity_error_bound_mps)
        || own_world_velocity_error_bound_mps < 0.0
        || !std::isfinite(lower)
        || !std::isfinite(upper))
    {
        status.code = StatusCode::ObservationInvalid;
        return;
    }

    output.vertical_velocity_down_mps = vertical_down;
    output.vertical_velocity_lower_mps = lower;
    output.vertical_velocity_upper_mps = upper;
    output.vertical_phase = upper < 0.0
        ? G16VerticalPhase::Climbing
        : lower > 0.0
        ? G16VerticalPhase::Descending
        : G16VerticalPhase::UnresolvedZeroInterval;

    if (!apex_previous_sample_valid_)
    {
        apex_previous_sample_valid_ = true;
        apex_climb_armed_ =
            output.vertical_phase == G16VerticalPhase::Climbing;
        output.evaluated = false;
        output.apex_crossed = false;
        output.climb_armed = apex_climb_armed_;
        output.reason = G16ApexReason::FirstSampleNoHistory;
        return;
    }

    output.evaluated = true;
    output.apex_crossed = false;
    if (output.vertical_phase == G16VerticalPhase::Climbing)
    {
        apex_climb_armed_ = true;
        output.reason = G16ApexReason::ResolvedClimbArmed;
    }
    else if (output.vertical_phase == G16VerticalPhase::UnresolvedZeroInterval)
    {
        output.reason = apex_climb_armed_
            ? G16ApexReason::ZeroIntervalRetainsClimbArm
            : G16ApexReason::ZeroIntervalWithoutPriorResolvedClimb;
    }
    else if (apex_climb_armed_)
    {
        output.apex_crossed = true;
        apex_climb_armed_ = false;
        output.reason =
            G16ApexReason::ResolvedClimbToDescentApexCrossing;
    }
    else
    {
        output.reason =
            G16ApexReason::ResolvedDescentWithoutPriorResolvedClimb;
    }
    output.climb_armed = apex_climb_armed_;
}

void G16HighObservation::ObserveRollIn(
    const runtime::TacticalCommandBuildInput& input,
    const G16ApexObservationReceipt& apex,
    G16HighRollObservationReceipt& output,
    Status& status) const noexcept
{
    output = G16HighRollObservationReceipt{};
    status = Status{};
    Vector3 flight_direction{};
    Vector3 body_up{};
    const Vector3 negative_down = Scale(input.frame.own.down_ned, -1.0);
    double body_norm_error = 0.0;
    double body_direction_error = 0.0;
    if (!Unit(input.frame.own.velocity_ned_mps, flight_direction)
        || !Unit(negative_down, body_up)
        || !BodyVelocityResolution(
            input.frame.own.velocity_body_mps,
            body_norm_error,
            body_direction_error)
        || Norm(input.frame.own.velocity_body_mps) - body_norm_error <= 0.0)
    {
        output.reason = G16HighRollReason::AttitudePlaneNotObservable;
        return;
    }
    const double flight_error =
        body_direction_error + 3.0 * kRpyQuantumRad;
    const double body_up_error = 3.0 * kRpyQuantumRad;
    Vector3 plane_normal{};
    double plane_error = 0.0;
    if (!Unit(Cross(flight_direction, body_up), plane_normal)
        || !CrossDirectionResolution(
            flight_direction,
            body_up,
            flight_error,
            body_up_error,
            plane_error))
    {
        output.reason = G16HighRollReason::AttitudePlaneNotObservable;
        return;
    }
    Vector3 lift_axis{};
    if (!Unit(Cross(plane_normal, flight_direction), lift_axis))
    {
        output.reason = G16HighRollReason::AttitudePlaneNotObservable;
        return;
    }
    if (Dot(lift_axis, body_up) < 0.0)
    {
        lift_axis = Scale(lift_axis, -1.0);
        plane_normal = Scale(plane_normal, -1.0);
    }
    const double lift_error = (std::min)(
        constants::Pi,
        plane_error + flight_error);

    const Vector3 relative_position = Subtract(
        input.frame.opponent.position_ned_m,
        input.frame.own.position_ned_m);
    double own_position_error = 0.0;
    double target_position_error = 0.0;
    double subtraction_roundoff = 0.0;
    if (!Float32WireVectorErrorBound(
            input.frame.own.position_ned_m,
            own_position_error)
        || !Float32WireVectorErrorBound(
            input.frame.opponent.position_ned_m,
            target_position_error)
        || !Binary64VectorRoundoffBound(
            relative_position,
            subtraction_roundoff))
    {
        output.reason = G16HighRollReason::DefenderDirectionNotObservable;
        return;
    }
    const double relative_position_error = NextUp(
        own_position_error + target_position_error + subtraction_roundoff);
    const double relative_range = Norm(relative_position);
    if (!std::isfinite(relative_range)
        || relative_range <= 0.0
        || !std::isfinite(relative_position_error)
        || relative_position_error >= relative_range)
    {
        output.reason = G16HighRollReason::DefenderDirectionNotObservable;
        return;
    }
    const double target_direction_error = NextUp(std::asin((std::min)(
        1.0,
        relative_position_error / relative_range)));
    Vector3 observed_bank{};
    Vector3 defender_direction{};
    if (!VelocityNormalDirection(
            lift_axis,
            flight_direction,
            observed_bank)
        || !VelocityNormalDirection(
            relative_position,
            flight_direction,
            defender_direction)
        || !std::isfinite(target_direction_error)
        || target_direction_error < 0.0
        || target_direction_error >= constants::Pi)
    {
        output.reason = G16HighRollReason::DefenderDirectionNotObservable;
        return;
    }

    const double nominal = std::acos((std::max)(
        -1.0,
        (std::min)(1.0, Dot(observed_bank, defender_direction))));
    const double combined_error = (std::min)(
        constants::Pi,
        lift_error + target_direction_error);
    const double lower = (std::max)(0.0, nominal - combined_error);
    const double upper = (std::min)(
        constants::Pi,
        nominal + combined_error);
    if (!std::isfinite(nominal)
        || !std::isfinite(combined_error)
        || !std::isfinite(lower)
        || !std::isfinite(upper))
    {
        output.reason = G16HighRollReason::DefenderDirectionNotObservable;
        return;
    }

    output.evaluated = true;
    output.flight_direction_ned = flight_direction;
    output.maneuver_plane_normal_ned = plane_normal;
    output.lift_axis_ned = lift_axis;
    output.flight_direction_error_bound_rad = flight_error;
    output.maneuver_plane_error_bound_rad = plane_error;
    output.lift_axis_error_bound_rad = lift_error;
    output.defender_direction_error_bound_rad = target_direction_error;
    output.nominal_alignment_angle_rad = nominal;
    output.alignment_angle_lower_rad = lower;
    output.alignment_angle_upper_rad = upper;

    if (upper < 0.5 * constants::Pi)
    {
        output.turning_toward_defender_resolved = true;
        output.turning_toward_defender = true;
    }
    else if (lower >= 0.5 * constants::Pi)
    {
        output.turning_toward_defender_resolved = true;
        output.turning_toward_defender = false;
    }
    output.descending_resolved =
        apex.vertical_phase != G16VerticalPhase::UnresolvedZeroInterval;
    output.descending =
        apex.vertical_phase == G16VerticalPhase::Descending;

    if (output.turning_toward_defender_resolved
        && output.turning_toward_defender
        && output.descending_resolved
        && output.descending)
    {
        output.high_roll_in_complete_resolved = true;
        output.high_roll_in_complete = true;
        output.reason = G16HighRollReason::
            ActualBankTowardDefenderInResolvedDescent;
    }
    else if ((output.turning_toward_defender_resolved
            && !output.turning_toward_defender)
        || (output.descending_resolved && !output.descending))
    {
        output.high_roll_in_complete_resolved = true;
        output.high_roll_in_complete = false;
        output.reason =
            G16HighRollReason::TowardDefenderRollOrDescentNotComplete;
    }
    else
    {
        output.reason =
            G16HighRollReason::RollDirectionOrVerticalPhaseUnresolved;
    }
}

void G16HighObservation::Observe(
    const runtime::TacticalCommandBuildInput& input,
    G16HighObservationReceipt& output,
    Status& status) noexcept
{
    output = G16HighObservationReceipt{};
    status = Status{};
    if (!SourceContractValid(input))
    {
        Reset();
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const ControlFrameIdentity identity = input.frame.frame_identity;
    if (cached_identity_valid_
        && SameControlFrameIdentity(cached_identity_, identity)
        && input.frame.own_plane_id == cached_own_plane_id_
        && input.frame.target_plane_id == cached_target_plane_id_)
    {
        output = cached_receipt_;
        status.code = cached_status_code_;
        return;
    }

    const bool had_previous_identity = cached_identity_valid_;
    const bool identity_restarted = cached_identity_valid_
        && (input.frame.own_plane_id != cached_own_plane_id_
            || input.frame.target_plane_id != cached_target_plane_id_
            || identity.episode_epoch != cached_identity_.episode_epoch
            || cached_identity_.frame_index
                == std::numeric_limits<std::uint64_t>::max()
            || identity.frame_index != cached_identity_.frame_index + 1U
            || identity.source_time_s <= cached_identity_.source_time_s);
    if (identity_restarted)
    {
        ResetPhysicalHistory();
    }
    const bool physical_history_was_empty =
        turn_history_count_ == 0U && !apex_previous_sample_valid_;

    output.frame_identity = identity;
    output.source_simultaneous = true;
    output.identity_restarted = identity_restarted;

    double target_body_norm_error = 0.0;
    double target_body_direction_error = 0.0;
    double own_world_velocity_error = 0.0;
    if (!BodyVelocityResolution(
            input.frame.opponent.velocity_body_mps,
            target_body_norm_error,
            target_body_direction_error)
        || !WorldVelocityErrorBound(
            input.frame.own,
            own_world_velocity_error))
    {
        ResetPhysicalHistory();
        status.code = StatusCode::ObservationInvalid;
        cached_identity_valid_ = true;
        cached_identity_ = identity;
        cached_own_plane_id_ = input.frame.own_plane_id;
        cached_target_plane_id_ = input.frame.target_plane_id;
        cached_receipt_ = output;
        cached_status_code_ = status.code;
        return;
    }

    ObserveEstablishedTurn(
        input,
        target_body_norm_error,
        target_body_direction_error,
        output.established_turn,
        status);
    if (!status.sample_valid())
    {
        ResetPhysicalHistory();
        output.established_turn.reason =
            output.established_turn.reason
                == G16EstablishedTurnReason::HistoryCapacityExceeded
            ? G16EstablishedTurnReason::HistoryCapacityExceeded
            : G16EstablishedTurnReason::CircleUnobservable;
        cached_identity_valid_ = true;
        cached_identity_ = identity;
        cached_own_plane_id_ = input.frame.own_plane_id;
        cached_target_plane_id_ = input.frame.target_plane_id;
        cached_receipt_ = output;
        cached_status_code_ = status.code;
        return;
    }

    ObserveApex(input, own_world_velocity_error, output.apex, status);
    if (!status.sample_valid())
    {
        ResetPhysicalHistory();
        cached_identity_valid_ = true;
        cached_identity_ = identity;
        cached_own_plane_id_ = input.frame.own_plane_id;
        cached_target_plane_id_ = input.frame.target_plane_id;
        cached_receipt_ = output;
        cached_status_code_ = status.code;
        return;
    }
    ObserveRollIn(input, output.apex, output.roll_in, status);
    if (!status.sample_valid())
    {
        ResetPhysicalHistory();
        cached_identity_valid_ = true;
        cached_identity_ = identity;
        cached_own_plane_id_ = input.frame.own_plane_id;
        cached_target_plane_id_ = input.frame.target_plane_id;
        cached_receipt_ = output;
        cached_status_code_ = status.code;
        return;
    }

    output.valid = true;
    status.code = identity_restarted
        ? StatusCode::FrameGap
        : !had_previous_identity || physical_history_was_empty
        ? StatusCode::Seeded
        : StatusCode::Ok;
    cached_identity_valid_ = true;
    cached_identity_ = identity;
    cached_own_plane_id_ = input.frame.own_plane_id;
    cached_target_plane_id_ = input.frame.target_plane_id;
    cached_receipt_ = output;
    cached_status_code_ = status.code;
}

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
