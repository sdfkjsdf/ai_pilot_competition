#include "LadyLuck/guidance/committed/G16ProductionEvidence.hpp"

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
constexpr double kPositiveCapabilityScale = 1.1953828780506852;
constexpr double kMaximumCommandLoadFactorG = 9.0;

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

double NextDown(const double value) noexcept
{
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
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
        const double full = (std::max)(
            std::abs(static_cast<double>(upper) - represented[axis]),
            std::abs(represented[axis] - static_cast<double>(lower)));
        cell[axis] = half_cell ? 0.5 * full : full;
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

bool BodyVelocityDirectionResolution(
    const Vector3& body_velocity,
    double& output) noexcept
{
    output = 0.0;
    Vector3 represented{};
    Vector3 half_cell{};
    if (!Float32Cell(body_velocity, true, represented, half_cell))
    {
        return false;
    }
    const double speed = Norm(body_velocity);
    const double error = std::sqrt(3.0) * kBodyVelocityQuantumMps
        + Norm(half_cell);
    const double speed_lower = speed - error;
    if (!std::isfinite(speed) || !std::isfinite(error) || speed_lower <= 0.0)
    {
        return false;
    }
    output = std::asin((std::min)(1.0, error / speed_lower))
        + 3.0 * kRpyQuantumRad;
    return std::isfinite(output) && output >= 0.0;
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
    const double rotation_error = 2.0
        * (Norm(aircraft.velocity_body_mps) + body_error)
        * std::sin(1.5 * kRpyQuantumRad);
    output = body_error + rotation_error + world_roundoff;
    return std::isfinite(output) && output >= 0.0;
}

bool NoseErrorBound(const Vector3& nose, double& output) noexcept
{
    output = 0.0;
    double roundoff = 0.0;
    if (!Binary64VectorRoundoffBound(nose, roundoff))
    {
        return false;
    }
    output = 2.0 * std::sin(1.5 * kRpyQuantumRad) + roundoff;
    return std::isfinite(output) && output >= 0.0;
}

double DotErrorBound(
    const Vector3& left,
    const double left_error,
    const Vector3& right,
    const double right_error) noexcept
{
    return Norm(left) * right_error
        + Norm(right) * left_error
        + left_error * right_error;
}

bool CrossDirectionResolution(
    const Vector3& first,
    const Vector3& second,
    const double first_bound,
    const double second_bound,
    double& output) noexcept
{
    output = 0.0;
    const double cross_magnitude = Norm(Cross(first, second));
    const double chord_error = 2.0 * std::sin(0.5 * first_bound)
        + 2.0 * std::sin(0.5 * second_bound);
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

struct PathChord
{
    bool valid = false;
    double duration_s = 0.0;
    double rotation_rad = 0.0;
    double rotation_resolution_rad = 0.0;
    Vector3 normal{};
    double normal_resolution_rad = 0.0;
};

// Keep the chord calculation expressed on scalar/vector values so the
// provider's private fixed-capacity Sample type never crosses its class seam.
void BuildPathChordValues(
    const double start_time,
    const Vector3& start_velocity,
    const double start_resolution,
    const double end_time,
    const Vector3& end_velocity,
    const double end_resolution,
    PathChord& output) noexcept
{
    output = PathChord{};
    Vector3 start{};
    Vector3 end{};
    if (!Unit(start_velocity, start) || !Unit(end_velocity, end))
    {
        return;
    }
    const Vector3 cross = Cross(start, end);
    const double sine = Norm(cross);
    const double cosine = (std::max)(-1.0, (std::min)(1.0, Dot(start, end)));
    const double rotation = std::atan2(sine, cosine);
    const double endpoint_error = start_resolution + end_resolution;
    const double duration = end_time - start_time;
    if (!std::isfinite(duration)
        || !std::isfinite(rotation)
        || !std::isfinite(endpoint_error)
        || duration <= 0.0
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
    const double normal_error = std::asin((std::min)(
        1.0,
        delta_cross / true_cross_lower));
    output.valid = std::isfinite(normal_error);
    output.duration_s = duration;
    output.rotation_rad = rotation;
    output.rotation_resolution_rad = endpoint_error;
    output.normal = Scale(cross, 1.0 / sine);
    output.normal_resolution_rad = normal_error;
}

bool SameIndexProductionSource(
    const LadyLuck::runtime::TacticalCommandBuildInput& input) noexcept
{
    return input.frame.target_same_index
        && input.frame.target_frame_index
            == input.frame.frame_identity.frame_index;
}

bool FiniteCompletedEnergy(
    const LadyLuck::control::tecs_cis::
        TecsCisCompletedEnergyAuthorityReceipt& receipt) noexcept
{
    return receipt.valid
        && LadyLuck::IsValidControlFrameIdentity(receipt.source_frame_identity)
        && receipt.continuous_total_energy_controller
        && receipt.rate_measurement_valid
        && receipt.controller_configuration_available
        && std::isfinite(receipt.energy_error_gain_per_s)
        && std::isfinite(receipt.energy_integral_gain_per_s2)
        && std::isfinite(receipt.energy_rate_feedback_gain)
        && std::isfinite(receipt.total_energy_error_m2ps2)
        && std::isfinite(receipt.energy_integral_error_m2ps)
        && std::isfinite(receipt.specific_energy_rate_measured_m2ps3)
        && std::isfinite(receipt.speed_mps)
        && std::isfinite(receipt.minimum_speed_mps)
        && std::isfinite(receipt.mass_kg)
        && receipt.mass_kg > 0.0
        && std::isfinite(receipt.drag_estimate_n)
        && std::isfinite(receipt.thrust_velocity_projection)
        && std::isfinite(receipt.thrust_min_n)
        && std::isfinite(receipt.thrust_max_n)
        && receipt.thrust_min_n <= receipt.thrust_max_n;
}

bool FinalOutwardRangeExit(
    const Vector3& relative_position,
    const Vector3& relative_velocity,
    const double weapon_range,
    const double horizon,
    double& exit_time) noexcept
{
    exit_time = 0.0;
    const double a = Dot(relative_velocity, relative_velocity);
    const double b = 2.0 * Dot(relative_position, relative_velocity);
    const double c = Dot(relative_position, relative_position)
        - weapon_range * weapon_range;
    const double endpoint = (a * horizon + b) * horizon + c;
    if (!std::isfinite(a)
        || !std::isfinite(b)
        || !std::isfinite(c)
        || !std::isfinite(endpoint)
        || endpoint <= 0.0)
    {
        return false;
    }
    if (a == 0.0)
    {
        return true;
    }
    const double discriminant = b * b - 4.0 * a * c;
    if (!std::isfinite(discriminant))
    {
        return false;
    }
    if (discriminant < 0.0)
    {
        return true;
    }
    const double root_delta = std::sqrt(discriminant);
    const double root_scale = 2.0 * a;
    const double lower = (-b - root_delta) / root_scale;
    const double upper = (-b + root_delta) / root_scale;
    if (upper < 0.0 || lower > horizon)
    {
        return true;
    }
    exit_time = (std::max)(0.0, upper);
    return std::isfinite(exit_time);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

void G16ProductionEvidenceProvider::Reset() noexcept
{
    samples_ = std::array<Sample, 3U>{};
    sample_count_ = 0U;
}

void G16ProductionEvidenceProvider::SeedSample(
    const Sample& sample) noexcept
{
    if (sample_count_ < samples_.size())
    {
        samples_[sample_count_] = sample;
        ++sample_count_;
        return;
    }
    samples_[0] = samples_[1];
    samples_[1] = samples_[2];
    samples_[2] = sample;
}

void G16ProductionEvidenceProvider::BuildBoundary(
    const runtime::TacticalCommandBuildInput& input,
    G16BoundaryReceipt& output,
    Status& status) noexcept
{
    output = G16BoundaryReceipt{};
    status = Status{};
    if (sample_count_ < 3U)
    {
        return;
    }

    const double t0 = samples_[0].time_s;
    const double t1 = samples_[1].time_s;
    const double t2 = samples_[2].time_s;
    if (!(t0 < t1 && t1 < t2))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double first[3] = {
        (t2 - t1) / ((t0 - t1) * (t0 - t2)),
        (t2 - t0) / ((t1 - t0) * (t1 - t2)),
        (2.0 * t2 - t0 - t1) / ((t2 - t0) * (t2 - t1))};
    const double second[3] = {
        2.0 / ((t0 - t1) * (t0 - t2)),
        2.0 / ((t1 - t0) * (t1 - t2)),
        2.0 / ((t2 - t0) * (t2 - t1))};
    Vector3 own_acceleration{};
    Vector3 defender_acceleration{};
    Vector3 raw_nose_rate{};
    Vector3 raw_nose_second{};
    for (std::size_t sample = 0U; sample < 3U; ++sample)
    {
        own_acceleration = Add(
            own_acceleration,
            Scale(samples_[sample].own_velocity_ned_mps, first[sample]));
        defender_acceleration = Add(
            defender_acceleration,
            Scale(samples_[sample].defender_velocity_ned_mps, first[sample]));
        raw_nose_rate = Add(
            raw_nose_rate,
            Scale(samples_[sample].defender_nose_ned, first[sample]));
        raw_nose_second = Add(
            raw_nose_second,
            Scale(samples_[sample].defender_nose_ned, second[sample]));
    }
    Vector3 nose{};
    Vector3 defender_path{};
    if (!Finite(own_acceleration)
        || !Finite(defender_acceleration)
        || !Finite(raw_nose_rate)
        || !Finite(raw_nose_second)
        || !Unit(samples_[2].defender_nose_ned, nose)
        || !Unit(samples_[2].defender_velocity_ned_mps, defender_path))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const Vector3 nose_rate = Subtract(
        raw_nose_rate,
        Scale(nose, Dot(raw_nose_rate, nose)));
    const Vector3 nose_second_tangent = Subtract(
        raw_nose_second,
        Scale(nose, Dot(raw_nose_second, nose)));
    const Vector3 nose_second = Subtract(
        nose_second_tangent,
        Scale(nose, Dot(nose_rate, nose_rate)));
    const Vector3 relative_position = Subtract(
        input.frame.own.position_ned_m,
        input.frame.opponent.position_ned_m);
    const Vector3 relative_velocity = Subtract(
        samples_[2].own_velocity_ned_mps,
        samples_[2].defender_velocity_ned_mps);
    const Vector3 relative_acceleration = Subtract(
        own_acceleration,
        defender_acceleration);
    const double signed_margin = -Dot(relative_position, nose);
    const double margin_rate = -(
        Dot(relative_velocity, nose)
        + Dot(relative_position, nose_rate));
    const double noncontrol_acceleration = -(
        Dot(relative_acceleration, nose)
        + 2.0 * Dot(relative_velocity, nose_rate)
        + Dot(relative_position, nose_second));

    double position_own_error = 0.0;
    double position_defender_error = 0.0;
    if (!Float32WireVectorErrorBound(
            input.frame.own.position_ned_m,
            position_own_error)
        || !Float32WireVectorErrorBound(
            input.frame.opponent.position_ned_m,
            position_defender_error))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double relative_position_error =
        position_own_error + position_defender_error;
    double own_acceleration_error = 0.0;
    double defender_acceleration_error = 0.0;
    double raw_nose_rate_error = 0.0;
    double raw_nose_second_error = 0.0;
    for (std::size_t sample = 0U; sample < 3U; ++sample)
    {
        own_acceleration_error += std::abs(first[sample])
            * samples_[sample].own_velocity_error_bound_mps;
        defender_acceleration_error += std::abs(first[sample])
            * samples_[sample].defender_velocity_error_bound_mps;
        raw_nose_rate_error += std::abs(first[sample])
            * samples_[sample].defender_nose_error_bound;
        raw_nose_second_error += std::abs(second[sample])
            * samples_[sample].defender_nose_error_bound;
    }
    const double nose_error = samples_[2].defender_nose_error_bound;
    const double nose_rate_error = raw_nose_rate_error
        + 2.0 * nose_error * (Norm(raw_nose_rate) + raw_nose_rate_error);
    const double nose_second_projection_error = raw_nose_second_error
        + 2.0 * nose_error
            * (Norm(raw_nose_second) + raw_nose_second_error);
    const double nose_second_error = nose_second_projection_error
        + Dot(nose_rate, nose_rate) * nose_error
        + nose_rate_error * (2.0 * Norm(nose_rate) + nose_rate_error);
    const double relative_velocity_error =
        samples_[2].own_velocity_error_bound_mps
        + samples_[2].defender_velocity_error_bound_mps;
    const double relative_acceleration_error =
        own_acceleration_error + defender_acceleration_error;
    const double margin_error = DotErrorBound(
        relative_position,
        relative_position_error,
        nose,
        nose_error);
    const double margin_rate_error = DotErrorBound(
        relative_velocity,
        relative_velocity_error,
        nose,
        nose_error)
        + DotErrorBound(
            relative_position,
            relative_position_error,
            nose_rate,
            nose_rate_error);
    const double margin_acceleration_error = DotErrorBound(
        relative_acceleration,
        relative_acceleration_error,
        nose,
        nose_error)
        + 2.0 * DotErrorBound(
            relative_velocity,
            relative_velocity_error,
            nose_rate,
            nose_rate_error)
        + DotErrorBound(
            relative_position,
            relative_position_error,
            nose_second,
            nose_second_error);
    if (!std::isfinite(signed_margin)
        || !std::isfinite(margin_rate)
        || !std::isfinite(noncontrol_acceleration)
        || !std::isfinite(margin_error)
        || margin_error < 0.0
        || !std::isfinite(margin_rate_error)
        || margin_rate_error < 0.0
        || !std::isfinite(margin_acceleration_error)
        || margin_acceleration_error < 0.0)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.valid = true;
    output.signed_margin_m = signed_margin;
    output.margin_rate_mps = margin_rate;
    output.noncontrol_margin_accel_mps2 = noncontrol_acceleration;
    output.signed_margin_error_bound_m = margin_error;
    output.margin_rate_error_bound_mps = margin_rate_error;
    output.noncontrol_margin_accel_error_bound_mps2 =
        margin_acceleration_error;
    output.positive_margin_direction_ned = Scale(nose, -1.0);
    output.own_acceleration_ned_mps2 = own_acceleration;
    output.derivative_support_duration_s = t2 - t0;

    PathChord older{};
    PathChord recent{};
    BuildPathChordValues(
        t0,
        samples_[0].defender_velocity_ned_mps,
        samples_[0].defender_direction_resolution_rad,
        t1,
        samples_[1].defender_velocity_ned_mps,
        samples_[1].defender_direction_resolution_rad,
        older);
    BuildPathChordValues(
        t1,
        samples_[1].defender_velocity_ned_mps,
        samples_[1].defender_direction_resolution_rad,
        t2,
        samples_[2].defender_velocity_ned_mps,
        samples_[2].defender_direction_resolution_rad,
        recent);
    if (!older.valid || !recent.valid)
    {
        return;
    }
    const double plane_separation = std::acos((std::max)(
        0.0,
        (std::min)(1.0, std::abs(Dot(older.normal, recent.normal)))));
    const double separation_bound = (std::min)(
        0.5 * constants::Pi,
        older.normal_resolution_rad + recent.normal_resolution_rad);
    if (plane_separation > separation_bound)
    {
        return;
    }
    Vector3 current_direction{};
    Vector3 lift_axis{};
    double lift_axis_error = 0.0;
    if (!Unit(samples_[2].defender_velocity_ned_mps, current_direction)
        || !Unit(Cross(recent.normal, current_direction), lift_axis)
        || !CrossDirectionResolution(
            recent.normal,
            current_direction,
            recent.normal_resolution_rad,
            samples_[2].defender_direction_resolution_rad,
            lift_axis_error))
    {
        return;
    }
    output.defender_turn_support_rotation_rad = recent.rotation_rad;
    output.defender_turn_support_rotation_resolution_rad =
        recent.rotation_resolution_rad;
    if (lift_axis_error < 0.5 * constants::Pi)
    {
        output.defender_turn_side_resolved = true;
        output.defender_turn_acceleration_direction_ned = lift_axis;
        output.defender_turn_direction_resolution_rad = lift_axis_error;
    }
}

void G16ProductionEvidenceProvider::BuildCapabilitiesAndDecision(
    const runtime::TacticalCommandBuildInput& input,
    G16ProductionEvidenceReceipt& output,
    Status& status) noexcept
{
    status = Status{};
    output.source_simultaneous = SameIndexProductionSource(input);
    if (!output.boundary.valid)
    {
        output.reason = G16EvidenceReason::ThreeCausalSamplesNotInitialized;
        return;
    }
    if (!output.source_simultaneous)
    {
        output.reason = G16EvidenceReason::SameIndexSourceUnavailable;
        return;
    }

    output.own_speed_mps = Norm(input.frame.own.velocity_ned_mps);
    const double opponent_speed = Norm(input.frame.opponent.velocity_ned_mps);
    output.enemy_range_m = input.frame.enemy_offense.range_m;
    output.enemy_outer_wez_range_m =
        input.frame.enemy_offense.phase.max_range_m;
    const Vector3 range_relative_position = Subtract(
        input.frame.opponent.position_ned_m,
        input.frame.own.position_ned_m);
    const double position_range_m = Norm(range_relative_position);
    double own_position_error_m = 0.0;
    double opponent_position_error_m = 0.0;
    const bool range_interval_available = std::isfinite(position_range_m)
        && Float32WireVectorErrorBound(
            input.frame.own.position_ned_m,
            own_position_error_m)
        && Float32WireVectorErrorBound(
            input.frame.opponent.position_ned_m,
            opponent_position_error_m);
    if (range_interval_available)
    {
        const double position_error_m = own_position_error_m
            + opponent_position_error_m;
        const double lower_m = NextDown(position_range_m - position_error_m);
        const double upper_m = NextUp(position_range_m + position_error_m);
        if (std::isfinite(position_error_m)
            && position_error_m >= 0.0
            && std::isfinite(lower_m)
            && std::isfinite(upper_m)
            && lower_m <= upper_m)
        {
            output.enemy_range_interval.valid = true;
            output.enemy_range_interval.error_bound_m = position_error_m;
            output.enemy_range_interval.lower_m = lower_m;
            output.enemy_range_interval.upper_m = upper_m;
        }
    }
    output.own_velocity_direction_resolution_valid =
        BodyVelocityDirectionResolution(
            input.frame.own.velocity_body_mps,
            output.own_velocity_direction_resolution_rad);
    output.official_employ_active = input.frame.own_offense.damage_rate > 0.0;
    if (!std::isfinite(output.own_speed_mps)
        || !std::isfinite(opponent_speed)
        || !std::isfinite(output.enemy_range_m)
        || output.enemy_range_m < 0.0
        || !std::isfinite(output.enemy_outer_wez_range_m)
        || output.enemy_outer_wez_range_m <= 0.0)
    {
        output.reason = G16EvidenceReason::ArithmeticInvalid;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (output.own_speed_mps <= 0.0)
    {
        output.reason = G16EvidenceReason::OwnTurnCapabilityUnavailable;
        return;
    }
    if (opponent_speed <= 0.0)
    {
        output.reason = G16EvidenceReason::OpponentTurnCapabilityUnavailable;
        return;
    }

    em_envelope_.ObserveCharacterizedRawN(
        output.own_speed_mps,
        -input.frame.own.position_ned_m[2],
        false,
        output.own_turn_capability.raw_lookup);
    em_envelope_.ObserveCharacterizedRawN(
        opponent_speed,
        -input.frame.opponent.position_ned_m[2],
        false,
        output.opponent_turn_capability.raw_lookup);
    if (output.own_turn_capability.raw_lookup.admitted()
        && output.own_turn_capability.raw_lookup.load_factor_g.has_value
        && output.own_turn_capability.raw_lookup.load_factor_g.value > 1.0)
    {
        output.own_turn_capability.capability_g = (std::min)(
            kMaximumCommandLoadFactorG,
            kPositiveCapabilityScale
                * output.own_turn_capability.raw_lookup.load_factor_g.value);
        output.own_turn_capability.admitted =
            std::isfinite(output.own_turn_capability.capability_g);
        output.own_turn_capability.physical_authority =
            output.own_turn_capability.admitted;
    }
    if (output.opponent_turn_capability.raw_lookup.admitted()
        && output.opponent_turn_capability.raw_lookup.load_factor_g.has_value
        && output.opponent_turn_capability.raw_lookup.load_factor_g.value > 1.0)
    {
        output.opponent_turn_capability.capability_g = (std::min)(
            kMaximumCommandLoadFactorG,
            kPositiveCapabilityScale
                * output.opponent_turn_capability.raw_lookup.load_factor_g.value);
        output.opponent_turn_capability.admitted =
            std::isfinite(output.opponent_turn_capability.capability_g);
        output.opponent_turn_capability.physical_authority =
            output.opponent_turn_capability.admitted;
    }
    output.escape_window_status = G16EscapeWindowStatus::Unresolved;
    output.escape_window_admitted = false;
    if (output.opponent_turn_capability.admitted)
    {
        const Vector3 relative_position = Subtract(
            input.frame.own.position_ned_m,
            input.frame.opponent.position_ned_m);
        const Vector3 relative_velocity = Subtract(
            input.frame.own.velocity_ned_mps,
            input.frame.opponent.velocity_ned_mps);
        Vector3 los_to_own{};
        Vector3 opponent_tangent{};
        if (Unit(relative_position, los_to_own)
            && Unit(
                input.frame.opponent.velocity_ned_mps,
                opponent_tangent))
        {
            const double face_angle = std::acos((std::max)(
                -1.0,
                (std::min)(1.0, Dot(opponent_tangent, los_to_own))));
            const double load = output.opponent_turn_capability.capability_g;
            const double maximum_turn_rate = constants::StandardGravityMps2
                * std::sqrt(load * load - 1.0) / opponent_speed;
            const double time_to_bite = face_angle / maximum_turn_rate;
            double time_to_exit = 0.0;
            const bool exit_available = FinalOutwardRangeExit(
                relative_position,
                relative_velocity,
                output.enemy_outer_wez_range_m,
                time_to_bite,
                time_to_exit);
            const bool already_outside =
                output.enemy_range_m > output.enemy_outer_wez_range_m;
            const bool open = exit_available
                && (time_to_exit < time_to_bite
                    || (already_outside && time_to_bite == 0.0));
            if (!std::isfinite(face_angle)
                || !std::isfinite(maximum_turn_rate)
                || maximum_turn_rate <= 0.0
                || !std::isfinite(time_to_bite))
            {
                output.reason = G16EvidenceReason::ArithmeticInvalid;
                status.code = StatusCode::NonFiniteInput;
                return;
            }
            output.escape_window_admitted = true;
            output.escape_window_status = open
                ? G16EscapeWindowStatus::Open
                : G16EscapeWindowStatus::Closed;
        }
    }
    // Current geometry, boundary, WEZ, and bounded turn inputs already form a
    // valid command-neutral receipt. Optional completed-energy evidence may
    // refine G16-E prevention, but its absence must not hide that receipt from
    // the v_cmd-driven High owner or active G5b lifecycle.
    output.valid = true;
    if (input.feedback_freshness != runtime::TacticalFeedbackFreshness::Fresh
        || !input.previous_control_feedback.valid
        || !FiniteCompletedEnergy(
            input.previous_control_feedback.completed_energy_authority))
    {
        output.reason =
            G16EvidenceReason::FreshCompletedEnergyAuthorityUnavailable;
        return;
    }
    if (!output.own_turn_capability.admitted)
    {
        output.reason = G16EvidenceReason::OwnTurnCapabilityUnavailable;
        return;
    }
    output.previous_completed_energy_authority =
        input.previous_control_feedback.completed_energy_authority;

    const auto& energy = output.previous_completed_energy_authority;
    const double thrust_a =
        energy.thrust_velocity_projection * energy.thrust_min_n;
    const double thrust_b =
        energy.thrust_velocity_projection * energy.thrust_max_n;
    const double longitudinal_min =
        ((std::min)(thrust_a, thrust_b) - energy.drag_estimate_n)
        / energy.mass_kg;
    const double longitudinal_max =
        ((std::max)(thrust_a, thrust_b) - energy.drag_estimate_n)
        / energy.mass_kg;
    Vector3 tangent{};
    if (!Unit(input.frame.own.velocity_ned_mps, tangent))
    {
        output.reason = G16EvidenceReason::ArithmeticInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double longitudinal_projection = Dot(
        tangent,
        output.boundary.positive_margin_direction_ned);
    const double longitudinal_a = longitudinal_projection * longitudinal_min;
    const double longitudinal_b = longitudinal_projection * longitudinal_max;
    const double gravity_projection = constants::StandardGravityMps2
        * output.boundary.positive_margin_direction_ned[2];
    output.positive_margin_control_accel_max_mps2 =
        output.own_turn_capability.capability_g
            * constants::StandardGravityMps2
        + gravity_projection
        + (std::max)(longitudinal_a, longitudinal_b);
    output.positive_margin_control_accel_max_valid = std::isfinite(
        output.positive_margin_control_accel_max_mps2);
    if (!output.positive_margin_control_accel_max_valid)
    {
        output.reason = G16EvidenceReason::ArithmeticInvalid;
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const double margin_lower = output.boundary.signed_margin_m
        - output.boundary.signed_margin_error_bound_m;
    const double margin_upper = output.boundary.signed_margin_m
        + output.boundary.signed_margin_error_bound_m;
    if (margin_lower <= 0.0)
    {
        output.prevention_status = G16PreventionStatus::Unresolved;
        output.reason = G16EvidenceReason::StrictlyBehindBoundaryUnresolved;
    }
    else
    {
        const double favorable_margin_rate = output.boundary.margin_rate_mps
            + output.boundary.margin_rate_error_bound_mps;
        const double closing_lower = (std::max)(0.0, -favorable_margin_rate);
        if (closing_lower == 0.0)
        {
            output.positive_margin_control_accel_required_mps2 = 0.0;
            output.positive_margin_control_accel_required_valid = true;
            output.prevention_status = G16PreventionStatus::NotRefuted;
            output.reason = G16EvidenceReason::BoundaryClosureNotResolved;
        }
        else
        {
            const double local_required = closing_lower * closing_lower
                / (2.0 * margin_upper);
            const double favorable_noncontrol =
                output.boundary.noncontrol_margin_accel_mps2
                + output.boundary.
                    noncontrol_margin_accel_error_bound_mps2;
            output.positive_margin_control_accel_required_mps2 =
                local_required - favorable_noncontrol;
            output.positive_margin_control_accel_required_valid = std::isfinite(
                output.positive_margin_control_accel_required_mps2);
            if (!output.positive_margin_control_accel_required_valid)
            {
                output.reason = G16EvidenceReason::ArithmeticInvalid;
                status.code = StatusCode::NonFiniteInput;
                return;
            }
            const double surplus =
                output.positive_margin_control_accel_max_mps2
                - output.positive_margin_control_accel_required_mps2;
            output.prevention_status = surplus >= 0.0
                ? G16PreventionStatus::NotRefuted
                : G16PreventionStatus::LocalInfeasible;
            output.reason = surplus >= 0.0
                ? G16EvidenceReason::OptimisticPreventionNotRefuted
                : G16EvidenceReason::EscapeWindowClosed;
        }
    }

    if (output.escape_window_status == G16EscapeWindowStatus::Unresolved)
    {
        output.reason = G16EvidenceReason::OpponentTurnCapabilityUnavailable;
        output.handoff_status = G16HandoffStatus::Unresolved;
    }
    else if (output.prevention_status == G16PreventionStatus::Unresolved)
    {
        output.handoff_status = G16HandoffStatus::Unresolved;
    }
    else
    {
        output.prevention_failure_candidate =
            output.prevention_status == G16PreventionStatus::LocalInfeasible
            && output.escape_window_status == G16EscapeWindowStatus::Open;
        output.handoff_status = output.prevention_failure_candidate
            ? G16HandoffStatus::Requested
            : G16HandoffStatus::NotRequested;
    }

    if (output.boundary.defender_turn_side_resolved)
    {
        const Vector3 egress = Scale(
            output.boundary.defender_turn_acceleration_direction_ned,
            -1.0);
        const double projection = Dot(egress, input.frame.opponent.right_ned);
        if (std::isfinite(projection) && projection != 0.0)
        {
            output.selected_egress_side_resolved = true;
            output.selected_egress_side_sign = projection > 0.0 ? 1 : -1;
        }
    }
    if (output.prevention_failure_candidate)
    {
        output.reason = output.selected_egress_side_resolved
            ? G16EvidenceReason::EgressCandidate
            : G16EvidenceReason::TurnSideUnresolved;
    }
    output.valid = true;
}

void G16ProductionEvidenceProvider::Observe(
    const runtime::TacticalCommandBuildInput& input,
    G16ProductionEvidenceReceipt& output,
    Status& status) noexcept
{
    output = G16ProductionEvidenceReceipt{};
    status = Status{};
    if (!input.valid
        || !IsValidControlFrameIdentity(input.frame.frame_identity)
        || !std::isfinite(input.frame.t_sec)
        || input.frame.t_sec < 0.0
        || !std::isfinite(input.accepted_estimator.sample_dt_s)
        || input.accepted_estimator.sample_dt_s <= 0.0
        || !Finite(input.frame.own.position_ned_m)
        || !Finite(input.frame.opponent.position_ned_m)
        || !Finite(input.frame.own.velocity_body_mps)
        || !Finite(input.frame.opponent.velocity_body_mps)
        || !Finite(input.frame.own.velocity_ned_mps)
        || !Finite(input.frame.opponent.velocity_ned_mps)
        || !Finite(input.frame.opponent.nose_ned)
        || !Finite(input.frame.opponent.right_ned))
    {
        Reset();
        output.reason = G16EvidenceReason::InvalidSourceContract;
        status.code = StatusCode::InvalidArgument;
        return;
    }

    Sample sample{};
    sample.time_s = input.frame.t_sec;
    sample.own_velocity_ned_mps = input.frame.own.velocity_ned_mps;
    sample.defender_velocity_ned_mps =
        input.frame.opponent.velocity_ned_mps;
    sample.defender_nose_ned = input.frame.opponent.nose_ned;
    if (!WorldVelocityErrorBound(
            input.frame.own,
            sample.own_velocity_error_bound_mps)
        || !WorldVelocityErrorBound(
            input.frame.opponent,
            sample.defender_velocity_error_bound_mps)
        || !NoseErrorBound(
            input.frame.opponent.nose_ned,
            sample.defender_nose_error_bound))
    {
        Reset();
        output.reason = G16EvidenceReason::BoundaryUncertaintyUnavailable;
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!BodyVelocityDirectionResolution(
            input.frame.opponent.velocity_body_mps,
            sample.defender_direction_resolution_rad))
    {
        // A finite opponent speed below the BattleServer velocity-cell
        // resolution has no sign-definite flight direction.  That is a normal
        // command-neutral non-observation: reset the three-sample history and
        // let Root/Mode selection continue without admitting G16.  Non-finite
        // source values were rejected above and remain contract faults.
        Reset();
        output.reason = G16EvidenceReason::BoundaryUncertaintyUnavailable;
        status = Status{};
        return;
    }

    if (sample_count_ > 0U)
    {
        const double previous_time = samples_[sample_count_ - 1U].time_s;
        const double observed_dt = sample.time_s - previous_time;
        const double expected_dt = input.accepted_estimator.sample_dt_s;
        const double tolerance = 8.0 * (
            std::abs(std::nextafter(
                sample.time_s,
                std::numeric_limits<double>::infinity()) - sample.time_s)
            + std::abs(std::nextafter(
                previous_time,
                std::numeric_limits<double>::infinity()) - previous_time)
            + std::abs(std::nextafter(
                expected_dt,
                std::numeric_limits<double>::infinity()) - expected_dt));
        if (observed_dt <= 0.0
            || std::abs(observed_dt - expected_dt) > tolerance)
        {
            Reset();
        }
    }
    SeedSample(sample);

    output.frame_identity = input.frame.frame_identity;
    output.frame = input.frame;
    BuildBoundary(input, output.boundary, status);
    if (!status.ok())
    {
        Reset();
        output = G16ProductionEvidenceReceipt{};
        output.reason = G16EvidenceReason::ArithmeticInvalid;
        return;
    }
    BuildCapabilitiesAndDecision(input, output, status);
    if (!status.ok())
    {
        Reset();
        output = G16ProductionEvidenceReceipt{};
        output.reason = G16EvidenceReason::ArithmeticInvalid;
    }
}

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
