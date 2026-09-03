#include "LadyLuck/guidance/obfm/PursuitPlaneSeparationObserver.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::AircraftGeometryKinematics;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::PursuitPathPlaneGate;

constexpr double BattleServerRpyQuantumRad =
    LadyLuck::constants::Pi / 180000.0;
constexpr double BattleServerBodyVelocityQuantumMps = 0.001 * 0.3048;

enum class SampleDisposition : std::uint8_t
{
    Valid = 0U,
    PhysicalNonAdmission = 1U,
    Invalid = 2U
};

struct PathDirectionSample
{
    double time_s = 0.0;
    Vector3 direction_ned{};
    double direction_resolution_rad = 0.0;
};

struct PreparedPathSample
{
    SampleDisposition disposition = SampleDisposition::Invalid;
    PathDirectionSample sample{};
    StatusCode fault = StatusCode::InvalidArgument;
};

struct PathChordEvidence
{
    bool valid = false;
    double duration_s = 0.0;
    double rotation_rad = 0.0;
    double rotation_resolution_rad = 0.0;
    Vector3 plane_normal_ned{};
    double plane_normal_resolution_rad = 0.0;
};

struct PathPlaneObservation
{
    bool valid = false;
    PursuitPathPlaneGate gate = PursuitPathPlaneGate::Unavailable;
    Vector3 plane_normal_ned{};
    double plane_normal_resolution_rad = 0.0;
};

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
    output = Vector3{{
        value[0] / magnitude,
        value[1] / magnitude,
        value[2] / magnitude}};
    return FiniteVector(output);
}

double Clamp(const double value, const double lower, const double upper) noexcept
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

bool BodyVelocityErrorBound(
    const Vector3& velocity_body_mps,
    double& speed_mps,
    double& error_bound_mps) noexcept
{
    speed_mps = 0.0;
    error_bound_mps = 0.0;
    if (!FiniteVector(velocity_body_mps))
    {
        return false;
    }
    speed_mps = Norm3(velocity_body_mps);
    if (!std::isfinite(speed_mps))
    {
        return false;
    }

    Vector3 half_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float represented_f32 =
            static_cast<float>(velocity_body_mps[index]);
        if (!std::isfinite(represented_f32))
        {
            return false;
        }
        const float upper_f32 = std::nextafter(
            represented_f32,
            (std::numeric_limits<float>::infinity)());
        const float lower_f32 = std::nextafter(
            represented_f32,
            -(std::numeric_limits<float>::infinity)());
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(upper_f32);
        const double lower = static_cast<double>(lower_f32);
        if (!std::isfinite(upper) || !std::isfinite(lower))
        {
            return false;
        }
        half_cell[index] = 0.5 * (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }
    error_bound_mps = std::sqrt(3.0)
            * BattleServerBodyVelocityQuantumMps
        + Norm3(half_cell);
    return std::isfinite(error_bound_mps) && error_bound_mps >= 0.0;
}

void PrepareSample(
    const AircraftGeometryKinematics& aircraft,
    const double sample_time_s,
    PreparedPathSample& output) noexcept
{
    output = PreparedPathSample{};
    output.sample.time_s = sample_time_s;
    if (!FiniteVector(aircraft.velocity_ned_mps))
    {
        output.fault = StatusCode::NonFiniteInput;
        return;
    }
    if (!Unit3(aircraft.velocity_ned_mps, output.sample.direction_ned))
    {
        output.disposition = SampleDisposition::PhysicalNonAdmission;
        output.fault = StatusCode::Ok;
        return;
    }

    double body_speed_mps = 0.0;
    double body_error_mps = 0.0;
    if (!BodyVelocityErrorBound(
            aircraft.velocity_body_mps,
            body_speed_mps,
            body_error_mps))
    {
        output.fault = StatusCode::NonFiniteInput;
        return;
    }
    const double true_speed_lower_mps = body_speed_mps - body_error_mps;
    if (true_speed_lower_mps <= 0.0)
    {
        output.disposition = SampleDisposition::PhysicalNonAdmission;
        output.fault = StatusCode::Ok;
        return;
    }
    const double ratio = (std::min)(
        1.0,
        body_error_mps / true_speed_lower_mps);
    const double body_direction_bound_rad = std::asin(ratio);
    output.sample.direction_resolution_rad = body_direction_bound_rad
        + 3.0 * BattleServerRpyQuantumRad;
    if (!std::isfinite(output.sample.direction_resolution_rad)
        || output.sample.direction_resolution_rad < 0.0)
    {
        output.fault = StatusCode::NonFiniteInput;
        return;
    }
    output.disposition = SampleDisposition::Valid;
    output.fault = StatusCode::Ok;
}

void ResetPathState(
    std::array<double, 3U>& times_s,
    std::array<Vector3, 3U>& directions_ned,
    std::array<double, 3U>& direction_bounds_rad,
    std::size_t& sample_count) noexcept
{
    times_s = std::array<double, 3U>{};
    directions_ned = std::array<Vector3, 3U>{};
    direction_bounds_rad = std::array<double, 3U>{};
    sample_count = 0U;
}

void BuildChord(
    const PathDirectionSample& start,
    const PathDirectionSample& end,
    PathChordEvidence& output) noexcept
{
    output = PathChordEvidence{};
    const Vector3 cross = Cross3(start.direction_ned, end.direction_ned);
    const double sine = Norm3(cross);
    const double cosine = Clamp(
        Dot3(start.direction_ned, end.direction_ned),
        -1.0,
        1.0);
    const double rotation = std::atan2(sine, cosine);
    const double endpoint_error = start.direction_resolution_rad
        + end.direction_resolution_rad;
    const double duration = end.time_s - start.time_s;
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
    output.valid = std::isfinite(axis_error);
    if (!output.valid)
    {
        output = PathChordEvidence{};
        return;
    }
    output.duration_s = duration;
    output.rotation_rad = rotation;
    output.rotation_resolution_rad = endpoint_error;
    output.plane_normal_ned = Vector3{{
        cross[0] / sine,
        cross[1] / sine,
        cross[2] / sine}};
    output.plane_normal_resolution_rad = axis_error;
}

bool CrossDirectionResolution(
    const Vector3& first,
    const Vector3& second,
    const double first_bound_rad,
    const double second_bound_rad,
    double& output) noexcept
{
    output = 0.0;
    const double cross_magnitude = Norm3(Cross3(first, second));
    const double chord_error =
        2.0 * std::sin(first_bound_rad / 2.0)
        + 2.0 * std::sin(second_bound_rad / 2.0);
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

void UpdatePathState(
    std::array<double, 3U>& times_s,
    std::array<Vector3, 3U>& directions_ned,
    std::array<double, 3U>& direction_bounds_rad,
    std::size_t& sample_count,
    const PreparedPathSample& prepared,
    const double sample_dt_s,
    PathPlaneObservation& output) noexcept
{
    output = PathPlaneObservation{};
    if (prepared.disposition == SampleDisposition::PhysicalNonAdmission)
    {
        ResetPathState(
            times_s,
            directions_ned,
            direction_bounds_rad,
            sample_count);
        output.gate = PursuitPathPlaneGate::PathSampleNotObservable;
        return;
    }

    const PathDirectionSample& sample = prepared.sample;
    if (sample_count > 0U)
    {
        const double previous_time = times_s[sample_count - 1U];
        const double observed_dt = sample.time_s - previous_time;
        const double ulp_tolerance = 8.0 * (
            PythonUlp(sample.time_s)
            + PythonUlp(previous_time)
            + PythonUlp(sample_dt_s));
        if (observed_dt <= 0.0
            || std::fabs(observed_dt - sample_dt_s) > ulp_tolerance)
        {
            ResetPathState(
                times_s,
                directions_ned,
                direction_bounds_rad,
                sample_count);
            times_s[0U] = sample.time_s;
            directions_ned[0U] = sample.direction_ned;
            direction_bounds_rad[0U] = sample.direction_resolution_rad;
            sample_count = 1U;
            output.gate = PursuitPathPlaneGate::
                PathSampleTimeLineageDiscontinuous;
            return;
        }
    }

    if (sample_count < 3U)
    {
        times_s[sample_count] = sample.time_s;
        directions_ned[sample_count] = sample.direction_ned;
        direction_bounds_rad[sample_count] =
            sample.direction_resolution_rad;
        ++sample_count;
    }
    else
    {
        times_s[0U] = times_s[1U];
        times_s[1U] = times_s[2U];
        times_s[2U] = sample.time_s;
        directions_ned[0U] = directions_ned[1U];
        directions_ned[1U] = directions_ned[2U];
        directions_ned[2U] = sample.direction_ned;
        direction_bounds_rad[0U] = direction_bounds_rad[1U];
        direction_bounds_rad[1U] = direction_bounds_rad[2U];
        direction_bounds_rad[2U] = sample.direction_resolution_rad;
    }
    if (sample_count < 3U)
    {
        output.gate =
            PursuitPathPlaneGate::TwoTurnChordsNotInitialized;
        return;
    }

    const PathDirectionSample samples[3U] = {
        PathDirectionSample{
            times_s[0U], directions_ned[0U], direction_bounds_rad[0U]},
        PathDirectionSample{
            times_s[1U], directions_ned[1U], direction_bounds_rad[1U]},
        PathDirectionSample{
            times_s[2U], directions_ned[2U], direction_bounds_rad[2U]}};
    PathChordEvidence older{};
    PathChordEvidence recent{};
    BuildChord(samples[0U], samples[1U], older);
    BuildChord(samples[1U], samples[2U], recent);
    if (!older.valid || !recent.valid)
    {
        output.gate = PursuitPathPlaneGate::
            TwoTurnChordsNotResolvedOutsideDirectionError;
        return;
    }

    const double consecutive_separation = std::acos(Clamp(
        std::fabs(Dot3(
            older.plane_normal_ned,
            recent.plane_normal_ned)),
        0.0,
        1.0));
    const double consecutive_bound = (std::min)(
        0.5 * LadyLuck::constants::Pi,
        older.plane_normal_resolution_rad
            + recent.plane_normal_resolution_rad);
    if (consecutive_separation > consecutive_bound)
    {
        output.gate =
            PursuitPathPlaneGate::ConsecutiveTurnPlaneConesDisjoint;
        return;
    }

    Vector3 lift_axis{};
    double lift_axis_error = 0.0;
    if (!Unit3(
            Cross3(recent.plane_normal_ned, samples[2U].direction_ned),
            lift_axis)
        || !CrossDirectionResolution(
            recent.plane_normal_ned,
            samples[2U].direction_ned,
            recent.plane_normal_resolution_rad,
            samples[2U].direction_resolution_rad,
            lift_axis_error))
    {
        output.gate =
            PursuitPathPlaneGate::PathPlaneLiftAxisNotObservable;
        return;
    }
    (void)lift_axis;
    (void)lift_axis_error;
    output.valid = true;
    output.gate = PursuitPathPlaneGate::TwoIntervalPathPlaneEstablished;
    output.plane_normal_ned = recent.plane_normal_ned;
    output.plane_normal_resolution_rad =
        recent.plane_normal_resolution_rad;
}

StatusCode NonObservableStatus(
    const PathPlaneObservation& own,
    const PathPlaneObservation& target) noexcept
{
    if (own.gate == PursuitPathPlaneGate::
            PathSampleTimeLineageDiscontinuous
        || target.gate == PursuitPathPlaneGate::
            PathSampleTimeLineageDiscontinuous)
    {
        return StatusCode::FrameGap;
    }
    if (own.gate == PursuitPathPlaneGate::TwoTurnChordsNotInitialized
        || target.gate == PursuitPathPlaneGate::TwoTurnChordsNotInitialized)
    {
        return StatusCode::Seeded;
    }
    return StatusCode::ObservationInvalid;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void PursuitPlaneSeparationObserver::Reset() noexcept
{
    own_sample_times_s_ = std::array<double, 3U>{};
    own_sample_directions_ned_ = std::array<Vector3, 3U>{};
    own_sample_direction_bounds_rad_ = std::array<double, 3U>{};
    own_sample_count_ = 0U;
    target_sample_times_s_ = std::array<double, 3U>{};
    target_sample_directions_ned_ = std::array<Vector3, 3U>{};
    target_sample_direction_bounds_rad_ = std::array<double, 3U>{};
    target_sample_count_ = 0U;
}

void PursuitPlaneSeparationObserver::Observe(
    const DogfightGeometryFrame& frame,
    const double sample_dt_s,
    RollingScissorsPlaneSeparationReceipt& output,
    Status& status) noexcept
{
    output = RollingScissorsPlaneSeparationReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        Reset();
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!std::isfinite(sample_dt_s) || sample_dt_s <= 0.0)
    {
        Reset();
        status.code = std::isfinite(sample_dt_s)
            ? StatusCode::InvalidDt
            : StatusCode::NonFiniteInput;
        return;
    }
    if (!std::isfinite(frame.t_sec))
    {
        Reset();
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (frame.t_sec < 0.0)
    {
        Reset();
        status.code = StatusCode::InvalidArgument;
        return;
    }

    PreparedPathSample own_sample{};
    PreparedPathSample target_sample{};
    PrepareSample(frame.own, frame.t_sec, own_sample);
    if (own_sample.disposition == SampleDisposition::Invalid)
    {
        Reset();
        status.code = own_sample.fault;
        return;
    }
    PrepareSample(frame.opponent, frame.t_sec, target_sample);
    if (target_sample.disposition == SampleDisposition::Invalid)
    {
        Reset();
        status.code = target_sample.fault;
        return;
    }

    PathPlaneObservation own_observation{};
    PathPlaneObservation target_observation{};
    UpdatePathState(
        own_sample_times_s_,
        own_sample_directions_ned_,
        own_sample_direction_bounds_rad_,
        own_sample_count_,
        own_sample,
        sample_dt_s,
        own_observation);
    UpdatePathState(
        target_sample_times_s_,
        target_sample_directions_ned_,
        target_sample_direction_bounds_rad_,
        target_sample_count_,
        target_sample,
        sample_dt_s,
        target_observation);

    output.valid = true;
    output.frame_identity = frame.frame_identity;
    output.own_path_gate = own_observation.gate;
    output.target_path_gate = target_observation.gate;
    output.own_path_plane_valid = own_observation.valid;
    output.target_path_plane_valid = target_observation.valid;
    if (!own_observation.valid || !target_observation.valid)
    {
        status.code = NonObservableStatus(
            own_observation,
            target_observation);
        return;
    }

    const double cosine = std::fabs(Dot3(
        own_observation.plane_normal_ned,
        target_observation.plane_normal_ned));
    output.plane_separation_rad = std::acos(Clamp(cosine, 0.0, 1.0));
    output.plane_separation_resolution_bound_rad = (std::min)(
        0.5 * constants::Pi,
        own_observation.plane_normal_resolution_rad
            + target_observation.plane_normal_resolution_rad);
    if (!std::isfinite(output.plane_separation_rad)
        || !std::isfinite(
            output.plane_separation_resolution_bound_rad))
    {
        output = RollingScissorsPlaneSeparationReceipt{};
        Reset();
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    output.separation_valid = true;
    output.relation = output.plane_separation_rad
            > output.plane_separation_resolution_bound_rad
        ? RollingScissorsPlaneRelation::ResolvablySeparated
        : RollingScissorsPlaneRelation::WithinObservationResolution;
    status = Status{};
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
