#include "LadyLuck/guidance/g13/G13FlatScissors.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::Vector3;
using LadyLuck::guidance::g13::G13FlatScissorsAdmissionReason;
using LadyLuck::guidance::g13::G13FlatScissorsAdmissionStatus;
using LadyLuck::guidance::g13::G13FlatScissorsObservation;
using LadyLuck::guidance::g13::G13FlatScissorsScopeGrade;
using LadyLuck::guidance::g13::G13SignedInterval;
using LadyLuck::guidance::g13::G13Truth;

constexpr double kBodyVelocityQuantumMps = 0.001 * 0.3048;
constexpr double kRpyQuantumRad =
    LadyLuck::constants::Pi / (180.0 * 1000.0);

bool Finite(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    // NumPy's three-element dot reduction order used by d90.
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
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

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double NextUp(const double value) noexcept
{
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

double NextDown(const double value) noexcept
{
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
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

bool SubtractUp(const double left, const double right, double& output) noexcept
{
    output = NextUp(left - right);
    return std::isfinite(output);
}

bool SubtractDown(const double left, const double right, double& output) noexcept
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
    const float positive_infinity = std::numeric_limits<float>::infinity();
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
            std::abs(static_cast<double>(upper) - represented[axis]),
            std::abs(represented[axis] - static_cast<double>(lower)));
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
            std::abs(upper - value[axis]),
            std::abs(value[axis] - lower));
    }
    output = NextUp(MathHypot3(full_cell));
    return std::isfinite(output) && output >= 0.0;
}

bool BodyVelocityDirectionResolution(
    const Vector3& body_velocity,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(body_velocity))
    {
        return false;
    }
    const double speed = NumpyNorm3(body_velocity);
    Vector3 half_cell{};
    const float positive_infinity = std::numeric_limits<float>::infinity();
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        const float encoded = static_cast<float>(body_velocity[axis]);
        const float upper = std::nextafter(encoded, positive_infinity);
        const float lower = std::nextafter(encoded, -positive_infinity);
        if (!std::isfinite(encoded)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return false;
        }
        const double represented = static_cast<double>(encoded);
        half_cell[axis] = 0.5 * (std::max)(
            std::abs(static_cast<double>(upper) - represented),
            std::abs(represented - static_cast<double>(lower)));
    }
    const double error = std::sqrt(3.0) * kBodyVelocityQuantumMps
        + NumpyNorm3(half_cell);
    const double true_speed_lower = speed - error;
    if (!std::isfinite(speed)
        || !std::isfinite(error)
        || true_speed_lower <= 0.0)
    {
        return false;
    }
    output = std::asin((std::min)(1.0, error / true_speed_lower));
    return std::isfinite(output);
}

bool AircraftFlightDirectionError(
    const LadyLuck::AircraftGeometryKinematics& aircraft,
    double& output) noexcept
{
    output = 0.0;
    double body = 0.0;
    if (!BodyVelocityDirectionResolution(aircraft.velocity_body_mps, body))
    {
        return false;
    }
    output = NextUp(body + 3.0 * kRpyQuantumRad);
    return std::isfinite(output)
        && output >= 0.0
        && output <= LadyLuck::constants::Pi;
}

bool HorizontalCourseError(
    const Vector3& direction,
    const double direction_error,
    double& output) noexcept
{
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
        return false;
    }
    const double ratio_upper = (std::min)(
        1.0,
        NextUp(direction_error / fraction_lower));
    output = NextUp(std::asin(ratio_upper));
    return std::isfinite(output);
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
        NextUp(std::abs(stored[0] - lower_x)),
        NextUp(std::abs(upper_x - stored[0])));
    const double error_y = (std::max)(
        NextUp(std::abs(stored[1] - lower_y)),
        NextUp(std::abs(upper_y - stored[1])));
    return AddUp(error_x, error_y, output);
}

bool BuildFlightPathSample(
    const LadyLuck::runtime::TacticalCommandBuildInput& input,
    LadyLuck::guidance::g13::G13FlatScissorsObserver::FlightPathSample&
        output) noexcept
{
    output = LadyLuck::guidance::g13::G13FlatScissorsObserver::
        FlightPathSample{};
    const auto& frame = input.frame;
    double own_position_error = 0.0;
    double opponent_position_error = 0.0;
    if (!Finite(frame.own.position_ned_m)
        || !Finite(frame.opponent.position_ned_m)
        || !Finite(frame.own.velocity_ned_mps)
        || !Float32WireVectorErrorBound(
            frame.own.position_ned_m,
            own_position_error)
        || !Float32WireVectorErrorBound(
            frame.opponent.position_ned_m,
            opponent_position_error))
    {
        return false;
    }
    double direction_error = 0.0;
    double course_error = 0.0;
    if (!AircraftFlightDirectionError(frame.own, direction_error)
        || !HorizontalCourseError(
            frame.own.velocity_ned_mps,
            direction_error,
            course_error))
    {
        return false;
    }
    const double course_norm = std::hypot(
        frame.own.velocity_ned_mps[0],
        frame.own.velocity_ned_mps[1]);
    if (!std::isfinite(course_norm) || course_norm <= 0.0)
    {
        return false;
    }
    Vector3 course{{
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
        return false;
    }
    const Vector3 relative_position = Subtract(
        frame.opponent.position_ned_m,
        frame.own.position_ned_m);
    double subtraction_roundoff = 0.0;
    double combined_position = 0.0;
    if (!Binary64VectorRoundoffBound(
            relative_position,
            subtraction_roundoff)
        || !AddUp(
            own_position_error,
            opponent_position_error,
            combined_position)
        || !AddUp(
            combined_position,
            subtraction_roundoff,
            combined_position))
    {
        return false;
    }
    output.valid = true;
    output.frame_identity = frame.frame_identity;
    output.relative_position_ned_m = relative_position;
    output.own_horizontal_course_ned = course;
    output.relative_position_error_bound_m = combined_position;
    output.own_course_error_bound_rad = course_error;
    output.own_course_normalization_error_bound_l1 = normalization_error;
    return true;
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
        NextUp(std::abs(squared_lower - 1.0)),
        NextUp(std::abs(squared_upper - 1.0)));
    return std::isfinite(output);
}

bool BuildSignedLateralInterval(
    const LadyLuck::guidance::g13::G13FlatScissorsObserver::FlightPathSample&
        axis_sample,
    const LadyLuck::guidance::g13::G13FlatScissorsObserver::FlightPathSample&
        position_sample,
    G13SignedInterval& output) noexcept
{
    output = G13SignedInterval{};
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
        || !AddUp(std::abs(relative[0]), std::abs(relative[1]), rho_l1)
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
    output.nominal = nominal;
    output.lower = lower;
    output.upper = upper;
    output.resolved_sign = lower > 0.0 ? 1 : upper < 0.0 ? -1 : 0;
    return true;
}

bool ResolveHorizontalTurnSign(
    const LadyLuck::AircraftGeometryKinematics& previous,
    const LadyLuck::AircraftGeometryKinematics& current,
    std::int32_t& output) noexcept
{
    output = 0;
    double previous_direction_error = 0.0;
    double current_direction_error = 0.0;
    double previous_course_error = 0.0;
    double current_course_error = 0.0;
    if (!AircraftFlightDirectionError(previous, previous_direction_error)
        || !AircraftFlightDirectionError(current, current_direction_error)
        || !HorizontalCourseError(
            previous.velocity_ned_mps,
            previous_direction_error,
            previous_course_error)
        || !HorizontalCourseError(
            current.velocity_ned_mps,
            current_direction_error,
            current_course_error))
    {
        return false;
    }
    const double previous_norm = std::hypot(
        previous.velocity_ned_mps[0],
        previous.velocity_ned_mps[1]);
    const double current_norm = std::hypot(
        current.velocity_ned_mps[0],
        current.velocity_ned_mps[1]);
    if (previous_norm == 0.0 || current_norm == 0.0)
    {
        return false;
    }
    const double previous_n = previous.velocity_ned_mps[0] / previous_norm;
    const double previous_e = previous.velocity_ned_mps[1] / previous_norm;
    const double current_n = current.velocity_ned_mps[0] / current_norm;
    const double current_e = current.velocity_ned_mps[1] / current_norm;
    const double cross_down = previous_n * current_e - previous_e * current_n;
    const double dot = previous_n * current_n + previous_e * current_e;
    const double angle = std::atan2(cross_down, dot);
    double error = 0.0;
    if (!AddUp(previous_course_error, current_course_error, error)
        || error >= LadyLuck::constants::Pi
        || std::abs(angle) >= LadyLuck::constants::Pi - error)
    {
        return false;
    }
    const double lower = NextDown(angle - error);
    const double upper = NextUp(angle + error);
    if (lower > 0.0)
    {
        output = 1;
        return true;
    }
    if (upper < 0.0)
    {
        output = -1;
        return true;
    }
    return false;
}

G13Truth TruthFromBool(const bool value) noexcept
{
    return value ? G13Truth::True : G13Truth::False;
}

G13Truth StrictPositive(const G13SignedInterval& interval) noexcept
{
    if (!interval.valid)
    {
        return G13Truth::Unresolved;
    }
    if (interval.lower > 0.0)
    {
        return G13Truth::True;
    }
    if (interval.upper < 0.0)
    {
        return G13Truth::False;
    }
    return G13Truth::Unresolved;
}

void ClassifyAdmission(G13FlatScissorsObservation& output) noexcept
{
    output.admission_status = G13FlatScissorsAdmissionStatus::NoAuthority;
    output.admission_reason = G13FlatScissorsAdmissionReason::SourceUnresolved;
    if (output.bounded_source_valid == G13Truth::Unresolved)
    {
        return;
    }
    if (output.bounded_source_valid == G13Truth::False)
    {
        output.admission_reason = G13FlatScissorsAdmissionReason::SourceInvalid;
        return;
    }
    if (!output.source_identity.valid)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::SourceIdentityUnresolved;
        return;
    }
    if (output.scope_grade == G13FlatScissorsScopeGrade::NotObservable)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::ScopeNotObservable;
        return;
    }
    if (output.scope_grade == G13FlatScissorsScopeGrade::Refuted)
    {
        output.admission_status = G13FlatScissorsAdmissionStatus::Hold;
        output.admission_reason = G13FlatScissorsAdmissionReason::ScopeRefuted;
        return;
    }
    if (output.far_los_steady_veto == G13Truth::Unresolved)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::LosVetoUnresolved;
        return;
    }
    if (output.far_los_steady_veto == G13Truth::True)
    {
        output.admission_status = G13FlatScissorsAdmissionStatus::Hold;
        output.admission_reason =
            G13FlatScissorsAdmissionReason::FarSteadyLosVeto;
        return;
    }
    if (output.fpo_before_defender_body_39_observed == G13Truth::Unresolved)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::FpoOrderUnresolved;
        return;
    }
    if (output.fpo_before_defender_body_39_observed == G13Truth::False)
    {
        output.admission_status = G13FlatScissorsAdmissionStatus::Hold;
        output.admission_reason =
            G13FlatScissorsAdmissionReason::FpoOrderRefuted;
        return;
    }
    if (output.attacker_pre_passage_observed == G13Truth::Unresolved)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::AttackerPrePassageUnresolved;
        return;
    }
    if (output.attacker_pre_passage_observed == G13Truth::False)
    {
        output.admission_status = G13FlatScissorsAdmissionStatus::Hold;
        output.admission_reason =
            G13FlatScissorsAdmissionReason::AttackerAlreadyPassed;
        return;
    }
    if (output.attacker_original_turn_committed == G13Truth::Unresolved)
    {
        output.admission_reason =
            G13FlatScissorsAdmissionReason::AttackerTurnCommitmentUnresolved;
        return;
    }
    if (output.attacker_original_turn_committed == G13Truth::False)
    {
        output.admission_status = G13FlatScissorsAdmissionStatus::Hold;
        output.admission_reason =
            G13FlatScissorsAdmissionReason::AttackerCounterTurnObserved;
        return;
    }
    output.admission_status =
        G13FlatScissorsAdmissionStatus::ReverseEvaluate;
    output.admission_reason = G13FlatScissorsAdmissionReason::ReverseEvaluate;
}

LadyLuck::runtime::TacticalCommandBuildInput SwappedInput(
    const LadyLuck::runtime::TacticalCommandBuildInput& input,
    const double dt) noexcept
{
    LadyLuck::runtime::TacticalCommandBuildInput swapped = input;
    swapped.frame.own_plane_id = input.frame.target_plane_id;
    swapped.frame.target_plane_id = input.frame.own_plane_id;
    swapped.frame.own = input.frame.opponent;
    swapped.frame.opponent = input.frame.own;
    swapped.frame.own_offense = input.frame.enemy_offense;
    swapped.frame.enemy_offense = input.frame.own_offense;
    swapped.accepted_estimator.sample_dt_s = dt;
    return swapped;
}

bool SameSourceInterval(
    const LadyLuck::runtime::TacticalCommandBuildInput& previous,
    const LadyLuck::runtime::TacticalCommandBuildInput& current,
    LadyLuck::guidance::g13::G13RoleExplicitEpisodeIdentity& output) noexcept
{
    output = LadyLuck::guidance::g13::G13RoleExplicitEpisodeIdentity{};
    if (!LadyLuck::IsValidControlFrameIdentity(previous.frame.frame_identity)
        || !LadyLuck::IsValidControlFrameIdentity(current.frame.frame_identity)
        || previous.frame.own_plane_id < 0
        || previous.frame.target_plane_id < 0
        || current.frame.own_plane_id != previous.frame.own_plane_id
        || current.frame.target_plane_id != previous.frame.target_plane_id
        || current.frame.own_plane_id == current.frame.target_plane_id
        || current.frame.frame_identity.episode_epoch
            != previous.frame.frame_identity.episode_epoch
        || previous.frame.frame_identity.frame_index
            == std::numeric_limits<std::uint64_t>::max()
        || current.frame.frame_identity.frame_index
            != previous.frame.frame_identity.frame_index + 1U
        || !std::isfinite(previous.frame.t_sec)
        || !std::isfinite(current.frame.t_sec)
        || current.frame.t_sec <= previous.frame.t_sec)
    {
        return false;
    }
    output.valid = true;
    output.defender_aircraft_id = current.frame.own_plane_id;
    output.attacker_aircraft_id = current.frame.target_plane_id;
    output.episode_epoch = current.frame.frame_identity.episode_epoch;
    output.previous_sample_index = previous.frame.frame_identity.frame_index;
    output.current_sample_index = current.frame.frame_identity.frame_index;
    output.previous_t_sec = previous.frame.t_sec;
    output.current_t_sec = current.frame.t_sec;
    return true;
}

bool SameFrameInputContract(
    const LadyLuck::runtime::TacticalCommandBuildInput& input) noexcept
{
    return input.valid
        && LadyLuck::IsValidControlFrameIdentity(input.frame.frame_identity)
        && input.frame.target_same_index
        && input.frame.target_frame_index
            == input.frame.frame_identity.frame_index
        && input.frame.own_plane_id >= 0
        && input.frame.target_plane_id >= 0
        && input.frame.own_plane_id != input.frame.target_plane_id
        && std::isfinite(input.frame.t_sec)
        && input.frame.t_sec >= 0.0;
}

G13SignedInterval MarginInterval(
    const LadyLuck::guidance::committed::G16BoundaryReceipt& boundary) noexcept
{
    G13SignedInterval output{};
    if (!boundary.valid
        || !std::isfinite(boundary.signed_margin_m)
        || !std::isfinite(boundary.signed_margin_error_bound_m)
        || boundary.signed_margin_error_bound_m < 0.0)
    {
        return output;
    }
    output.valid = true;
    output.nominal = boundary.signed_margin_m;
    output.lower = NextDown(
        boundary.signed_margin_m
        - boundary.signed_margin_error_bound_m);
    output.upper = NextUp(
        boundary.signed_margin_m
        + boundary.signed_margin_error_bound_m);
    if (!std::isfinite(output.lower) || !std::isfinite(output.upper))
    {
        return G13SignedInterval{};
    }
    output.resolved_sign = output.lower > 0.0
        ? 1
        : output.upper < 0.0 ? -1 : 0;
    return output;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g13
{

void G13FlatScissorsObserver::Reset() noexcept
{
    defender_boundary_observer_.Reset();
    attacker_boundary_observer_.Reset();
    boundary_seeded_ = false;
    previous_valid_ = false;
    previous_input_ = runtime::TacticalCommandBuildInput{};
    previous_fpo_sample_ = FlightPathSample{};
    previous_defender_margin_valid_ = false;
    previous_defender_margin_ = G13SignedInterval{};
    previous_attacker_turn_sign_valid_ = false;
    previous_attacker_turn_sign_ = 0;
    last_resolved_attacker_turn_sign_valid_ = false;
    last_resolved_attacker_turn_sign_ = 0;
    attacker_counter_turn_observed_ = false;
}

void G13FlatScissorsObserver::Anchor(
    const runtime::TacticalCommandBuildInput& input,
    const FlightPathSample& sample) noexcept
{
    previous_valid_ = true;
    previous_input_ = input;
    previous_fpo_sample_ = sample;
}

void G13FlatScissorsObserver::Update(
    const runtime::TacticalCommandBuildInput& input,
    G13FlatScissorsObservation& output,
    Status& status) noexcept
{
    output = G13FlatScissorsObservation{};
    status = Status{};
    output.valid = true;
    output.frame_identity = input.frame.frame_identity;

    if (!SameFrameInputContract(input))
    {
        Reset();
        output.bounded_source_valid = G13Truth::False;
        ClassifyAdmission(output);
        return;
    }

    FlightPathSample current_sample{};
    if (!BuildFlightPathSample(input, current_sample))
    {
        Reset();
        output.bounded_source_valid = G13Truth::False;
        ClassifyAdmission(output);
        return;
    }

    if (!previous_valid_)
    {
        Anchor(input, current_sample);
        output.bounded_source_valid = G13Truth::Unresolved;
        ClassifyAdmission(output);
        return;
    }

    if (!SameSourceInterval(previous_input_, input, output.source_identity))
    {
        Reset();
        Anchor(input, current_sample);
        output.bounded_source_valid = G13Truth::False;
        ClassifyAdmission(output);
        return;
    }

    const double dt = output.source_identity.current_t_sec
        - output.source_identity.previous_t_sec;
    guidance::committed::G16ProductionEvidenceReceipt defender_boundary{};
    guidance::committed::G16ProductionEvidenceReceipt attacker_boundary{};
    Status boundary_status{};
    if (!boundary_seeded_)
    {
        runtime::TacticalCommandBuildInput previous = previous_input_;
        previous.accepted_estimator.sample_dt_s = dt;
        runtime::TacticalCommandBuildInput previous_swapped =
            SwappedInput(previous, dt);
        defender_boundary_observer_.Observe(
            previous_swapped,
            defender_boundary,
            boundary_status);
        if (!boundary_status.ok())
        {
            Reset();
            status = boundary_status;
            output = G13FlatScissorsObservation{};
            return;
        }
        attacker_boundary_observer_.Observe(
            previous,
            attacker_boundary,
            boundary_status);
        if (!boundary_status.ok())
        {
            Reset();
            status = boundary_status;
            output = G13FlatScissorsObservation{};
            return;
        }
        boundary_seeded_ = true;
    }
    runtime::TacticalCommandBuildInput current = input;
    current.accepted_estimator.sample_dt_s = dt;
    runtime::TacticalCommandBuildInput current_swapped =
        SwappedInput(current, dt);
    defender_boundary_observer_.Observe(
        current_swapped,
        defender_boundary,
        boundary_status);
    if (!boundary_status.ok())
    {
        Reset();
        status = boundary_status;
        output = G13FlatScissorsObservation{};
        return;
    }
    attacker_boundary_observer_.Observe(
        current,
        attacker_boundary,
        boundary_status);
    if (!boundary_status.ok())
    {
        Reset();
        status = boundary_status;
        output = G13FlatScissorsObservation{};
        return;
    }
    output.defender_body_39_margin_m =
        MarginInterval(defender_boundary.boundary);
    output.attacker_body_39_margin_m =
        MarginInterval(attacker_boundary.boundary);

    G13SignedInterval intervals[4]{};
    if (!BuildSignedLateralInterval(
            previous_fpo_sample_,
            previous_fpo_sample_,
            intervals[0])
        || !BuildSignedLateralInterval(
            previous_fpo_sample_,
            current_sample,
            intervals[1])
        || !BuildSignedLateralInterval(
            current_sample,
            previous_fpo_sample_,
            intervals[2])
        || !BuildSignedLateralInterval(
            current_sample,
            current_sample,
            intervals[3]))
    {
        Reset();
        output = G13FlatScissorsObservation{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    output.fpo_geometry_evaluable = true;
    const bool signs_resolved = intervals[0].resolved_sign != 0
        && intervals[1].resolved_sign != 0
        && intervals[2].resolved_sign != 0
        && intervals[3].resolved_sign != 0;
    output.fpo_dual_frozen_axis_crossing_resolved = signs_resolved
        && intervals[0].resolved_sign == intervals[2].resolved_sign
        && intervals[1].resolved_sign == intervals[3].resolved_sign
        && intervals[1].resolved_sign == -intervals[0].resolved_sign;
    if (!output.fpo_geometry_evaluable || !signs_resolved)
    {
        output.fpo_before_defender_body_39_observed = G13Truth::Unresolved;
    }
    else if (!output.fpo_dual_frozen_axis_crossing_resolved)
    {
        output.fpo_before_defender_body_39_observed = G13Truth::False;
    }
    else if (!previous_defender_margin_valid_
        || !output.defender_body_39_margin_m.valid)
    {
        output.fpo_before_defender_body_39_observed = G13Truth::Unresolved;
    }
    else if (previous_defender_margin_.lower > 0.0
        && output.defender_body_39_margin_m.lower > 0.0)
    {
        output.fpo_before_defender_body_39_observed = G13Truth::True;
    }
    else if (previous_defender_margin_.upper < 0.0
        || output.defender_body_39_margin_m.upper < 0.0)
    {
        output.fpo_before_defender_body_39_observed = G13Truth::False;
    }
    else
    {
        output.fpo_before_defender_body_39_observed = G13Truth::Unresolved;
    }

    std::int32_t defender_turn_sign = 0;
    std::int32_t attacker_turn_sign = 0;
    output.defender_turn_sign_valid = ResolveHorizontalTurnSign(
        previous_input_.frame.own,
        input.frame.own,
        defender_turn_sign);
    output.attacker_turn_sign_valid = ResolveHorizontalTurnSign(
        previous_input_.frame.opponent,
        input.frame.opponent,
        attacker_turn_sign);
    output.defender_turn_sign = output.defender_turn_sign_valid
        ? defender_turn_sign
        : 0;
    output.attacker_turn_sign = output.attacker_turn_sign_valid
        ? attacker_turn_sign
        : 0;
    output.previous_attacker_turn_sign_valid =
        previous_attacker_turn_sign_valid_;
    output.previous_attacker_turn_sign = previous_attacker_turn_sign_valid_
        ? previous_attacker_turn_sign_
        : 0;
    if (last_resolved_attacker_turn_sign_valid_
        && output.attacker_turn_sign_valid
        && output.attacker_turn_sign
            != last_resolved_attacker_turn_sign_)
    {
        attacker_counter_turn_observed_ = true;
    }
    if (output.attacker_turn_sign_valid)
    {
        last_resolved_attacker_turn_sign_valid_ = true;
        last_resolved_attacker_turn_sign_ = output.attacker_turn_sign;
    }
    output.attacker_original_turn_committed =
        attacker_counter_turn_observed_
        ? G13Truth::False
        : (!previous_attacker_turn_sign_valid_
                || !output.attacker_turn_sign_valid)
            ? G13Truth::Unresolved
            : G13Truth::True;
    if (!output.defender_turn_sign_valid
        || !output.attacker_turn_sign_valid)
    {
        output.horizontal_same_turn_scope = G13Truth::Unresolved;
        output.scope_grade = G13FlatScissorsScopeGrade::NotObservable;
    }
    else if (output.defender_turn_sign != output.attacker_turn_sign)
    {
        output.horizontal_same_turn_scope = G13Truth::False;
        output.scope_grade = G13FlatScissorsScopeGrade::Refuted;
    }
    else
    {
        output.horizontal_same_turn_scope = G13Truth::True;
        output.scope_grade = G13FlatScissorsScopeGrade::Admitted;
    }
    output.attacker_pre_passage_observed =
        StrictPositive(output.attacker_body_39_margin_m);

    const Vector3 relative_position = Subtract(
        input.frame.opponent.position_ned_m,
        input.frame.own.position_ned_m);
    const double range_m = NumpyNorm3(relative_position);
    if (std::isfinite(range_m) && range_m > 0.0)
    {
        const Vector3 relative_velocity = Subtract(
            input.frame.opponent.velocity_ned_mps,
            input.frame.own.velocity_ned_mps);
        const double los_rate = NumpyNorm3(
            Cross(relative_position, relative_velocity))
            / (range_m * range_m);
        const double steady_bound = 2.0
            * std::sqrt(3.0)
            * kBodyVelocityQuantumMps
            / range_m;
        if (!std::isfinite(los_rate)
            || los_rate < 0.0
            || !std::isfinite(steady_bound)
            || steady_bound < 0.0)
        {
            Reset();
            output = G13FlatScissorsObservation{};
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        output.los_observation_valid = true;
        output.range_m = range_m;
        output.los_rate_rad_s = los_rate;
        output.los_steady_bound_rad_s = steady_bound;
        output.los_steady = los_rate <= steady_bound;
        const double maximum = input.frame.enemy_offense.phase.max_range_m;
        double own_position_error = 0.0;
        double opponent_position_error = 0.0;
        if (!std::isfinite(maximum)
            || maximum <= 0.0
            || !Float32WireVectorErrorBound(
                input.frame.own.position_ned_m,
                own_position_error)
            || !Float32WireVectorErrorBound(
                input.frame.opponent.position_ned_m,
                opponent_position_error))
        {
            Reset();
            output = G13FlatScissorsObservation{};
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        const double position_error = own_position_error
            + opponent_position_error;
        const double lower = NextDown(range_m - position_error);
        const double upper = NextUp(range_m + position_error);
        output.attacker_official_far = lower > maximum
            ? G13Truth::True
            : upper < maximum
                ? G13Truth::False
                : G13Truth::Unresolved;
        output.far_los_steady_veto = !output.los_steady
            ? G13Truth::False
            : output.attacker_official_far;
    }
    else
    {
        output.los_observation_valid = false;
        output.attacker_official_far = G13Truth::Unresolved;
        output.far_los_steady_veto = G13Truth::Unresolved;
    }

    output.bounded_source_valid = G13Truth::True;
    ClassifyAdmission(output);

    previous_input_ = input;
    previous_fpo_sample_ = current_sample;
    previous_defender_margin_valid_ =
        output.defender_body_39_margin_m.valid;
    previous_defender_margin_ = output.defender_body_39_margin_m;
    previous_attacker_turn_sign_valid_ = output.attacker_turn_sign_valid;
    previous_attacker_turn_sign_ = output.attacker_turn_sign;
}

} // namespace g13
} // namespace guidance
} // namespace LadyLuck
