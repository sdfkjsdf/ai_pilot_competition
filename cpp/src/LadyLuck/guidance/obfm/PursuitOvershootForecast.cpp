#include "LadyLuck/guidance/obfm/PursuitOvershootForecast.hpp"

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
using LadyLuck::guidance::obfm::PursuitEscapeWindowObservation;
using LadyLuck::guidance::obfm::PursuitEscapeWindowReason;
using LadyLuck::guidance::obfm::PursuitEscapeWindowStatus;
using LadyLuck::guidance::obfm::PursuitOptionalDouble;
using LadyLuck::guidance::obfm::PursuitOvershootForecast;
using LadyLuck::guidance::obfm::PursuitOvershootForecastReason;
using LadyLuck::guidance::obfm::PursuitOvershootForecastStatus;

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kFloat32Epsilon =
    static_cast<double>(std::numeric_limits<float>::epsilon());

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyDot3(const Vector3& left, const Vector3& right) noexcept
{
    // Frozen NumPy 1.26.4 length-three reduction association used by d90.
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(NumpyDot3(value, value));
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

template <std::size_t CoordinateCount>
double PythonMathHypot(
    const std::array<double, CoordinateCount>& values) noexcept
{
    // Exact finite specialization of CPython 3.12 vector_norm(), the backing
    // implementation for d90's n-ary math.hypot calls.
    constexpr double Splitter = 134217729.0;
    std::array<double, CoordinateCount> coordinates{};
    double maximum = 0.0;
    for (std::size_t index = 0U; index < CoordinateCount; ++index)
    {
        coordinates[index] = std::fabs(values[index]);
        maximum = (std::max)(maximum, coordinates[index]);
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    double correction_sum = 1.0;
    double fraction_one = 0.0;
    if (maximum_exponent >= -1023)
    {
        const double scale = std::ldexp(1.0, -maximum_exponent);
        double fraction_two = 0.0;
        double fraction_three = 0.0;
        for (const double coordinate : coordinates)
        {
            double x = coordinate * scale;
            const double split = x * Splitter;
            const double high = split - (split - x);
            const double low = x - high;

            x = high * high;
            double old_sum = correction_sum;
            correction_sum += x;
            fraction_one += (old_sum - correction_sum) + x;

            x = 2.0 * high * low;
            old_sum = correction_sum;
            correction_sum += x;
            fraction_two += (old_sum - correction_sum) + x;

            fraction_three += low * low;
        }
        double root = std::sqrt(
            correction_sum - 1.0
            + (fraction_one + fraction_two + fraction_three));

        double x = root;
        const double split = x * Splitter;
        const double high = split - (split - x);
        const double low = x - high;

        x = -high * high;
        double old_sum = correction_sum;
        correction_sum += x;
        fraction_one += (old_sum - correction_sum) + x;

        x = -2.0 * high * low;
        old_sum = correction_sum;
        correction_sum += x;
        fraction_two += (old_sum - correction_sum) + x;

        x = -low * low;
        old_sum = correction_sum;
        correction_sum += x;
        fraction_three += (old_sum - correction_sum) + x;

        x = correction_sum - 1.0
            + (fraction_one + fraction_two + fraction_three);
        return (root + x / (2.0 * root)) / scale;
    }

    for (const double coordinate : coordinates)
    {
        double x = coordinate / maximum;
        x *= x;
        const double old_sum = correction_sum;
        correction_sum += x;
        fraction_one += (old_sum - correction_sum) + x;
    }
    return maximum * std::sqrt(correction_sum - 1.0 + fraction_one);
}

double PythonMathHypot2(const double first, const double second) noexcept
{
    return PythonMathHypot<2U>({{first, second}});
}

double PythonMathHypot3(const Vector3& value) noexcept
{
    return PythonMathHypot<3U>({{value[0], value[1], value[2]}});
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    Vector3 represented{};
    Vector3 residual{};
    Vector3 full_cell{};
    const float positive_infinity =
        (std::numeric_limits<float>::infinity)();
    const float negative_infinity = -positive_infinity;
    for (std::size_t index = 0U; index < value.size(); ++index)
    {
        const float wire_value = static_cast<float>(value[index]);
        represented[index] = static_cast<double>(wire_value);
        if (!std::isfinite(represented[index]))
        {
            return false;
        }
        const float upper_wire = std::nextafter(
            wire_value,
            positive_infinity);
        const float lower_wire = std::nextafter(
            wire_value,
            negative_infinity);
        const double upper = static_cast<double>(upper_wire);
        const double lower = static_cast<double>(lower_wire);
        full_cell[index] = (std::max)(
            std::fabs(upper - represented[index]),
            std::fabs(represented[index] - lower));
        if (!std::isfinite(full_cell[index]))
        {
            return false;
        }
        residual[index] = value[index] - represented[index];
    }

    const double positive_double_infinity =
        (std::numeric_limits<double>::infinity)();
    const double residual_norm = std::nextafter(
        PythonMathHypot3(residual),
        positive_double_infinity);
    const double cell_norm = std::nextafter(
        PythonMathHypot3(full_cell),
        positive_double_infinity);
    output = std::nextafter(
        residual_norm + cell_norm,
        positive_double_infinity);
    return std::isfinite(output);
}

PursuitOptionalDouble Optional(const double value) noexcept
{
    return PursuitOptionalDouble{true, value};
}

void MakeUnresolved(
    PursuitOvershootForecast& output,
    const PursuitOvershootForecastReason reason) noexcept
{
    output.valid = true;
    output.status = PursuitOvershootForecastStatus::Unresolved;
    output.reason = reason;
    output.maintained_passage_projected = false;
    output.along_track_m = PursuitOptionalDouble{};
    output.closure_mps = PursuitOptionalDouble{};
    output.braking_distance_m = PursuitOptionalDouble{};
    output.optimistic_arrest_mps2 = PursuitOptionalDouble{};
}

bool FinalOutwardWeaponRangeExitTime(
    const Vector3& relative_position_m,
    const Vector3& relative_velocity_mps,
    const double weapon_outer_range_m,
    const double max_time_s,
    PursuitOptionalDouble& output) noexcept
{
    output = PursuitOptionalDouble{};
    const double a = NumpyDot3(
        relative_velocity_mps,
        relative_velocity_mps);
    const double b = 2.0 * NumpyDot3(
        relative_position_m,
        relative_velocity_mps);
    const double c = NumpyDot3(relative_position_m, relative_position_m)
        - weapon_outer_range_m * weapon_outer_range_m;
    const double boundary = (a * max_time_s + b) * max_time_s + c;
    if (boundary <= 0.0)
    {
        return true;
    }
    if (a == 0.0)
    {
        output = Optional(0.0);
        return true;
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0)
    {
        output = Optional(0.0);
        return true;
    }
    const double root_scale = 2.0 * a;
    const double root_delta = std::sqrt(discriminant);
    const double lower_root = (-b - root_delta) / root_scale;
    const double upper_root = (-b + root_delta) / root_scale;
    if (upper_root < 0.0 || lower_root > max_time_s)
    {
        output = Optional(0.0);
        return true;
    }
    output = Optional((std::max)(0.0, upper_root));
    return true;
}

bool BuildEscapeWindow(
    const LadyLuck::DogfightGeometryFrame& frame,
    const Vector3& relative_position_m,
    const double separation_m,
    const double opponent_speed_mps,
    const double opponent_altitude_m,
    const LadyLuck::guidance::em::StrictEnergyManeuverEnvelope& envelope,
    PursuitEscapeWindowObservation& output,
    Status& status) noexcept
{
    output = PursuitEscapeWindowObservation{};
    output.available = true;

    LadyLuck::guidance::em::CharacterizedRawNLookup raw{};
    envelope.ObserveCharacterizedRawN(
        opponent_speed_mps,
        opponent_altitude_m,
        false,
        raw);
    if (raw.admitted()
        && raw.load_factor_g.has_value
        && std::isfinite(raw.load_factor_g.value)
        && raw.load_factor_g.value > 1.0)
    {
        // Exact G19 opponent shadow: pre-margin characterized N with no 1.195
        // ownship-command scale and no 9-g command clamp.
        output.opponent_capability_g = Optional(raw.load_factor_g.value);
    }

    const double weapon_outer_range_m = frame.enemy_offense.phase.max_range_m;
    if (!std::isfinite(weapon_outer_range_m)
        || weapon_outer_range_m <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        output = PursuitEscapeWindowObservation{};
        return false;
    }
    output.weapon_outer_range_m = weapon_outer_range_m;
    output.initial_range_m = separation_m;

    const bool capability_valid = output.opponent_capability_g.has_value
        && opponent_speed_mps > 0.0;
    if (!capability_valid)
    {
        output.admitted = false;
        output.status = PursuitEscapeWindowStatus::Unresolved;
        output.reason =
            PursuitEscapeWindowReason::OpponentTurnCapabilityNotAdmitted;
        return true;
    }

    const double opponent_n_g = output.opponent_capability_g.value;
    const Vector3 los_to_own{{
        relative_position_m[0] / separation_m,
        relative_position_m[1] / separation_m,
        relative_position_m[2] / separation_m}};
    const Vector3 opponent_tangent{{
        frame.opponent.velocity_ned_mps[0] / opponent_speed_mps,
        frame.opponent.velocity_ned_mps[1] / opponent_speed_mps,
        frame.opponent.velocity_ned_mps[2] / opponent_speed_mps}};
    const double face_cosine = NumpyDot3(opponent_tangent, los_to_own);
    const double face_angle_rad = std::acos((std::min)(
        1.0,
        (std::max)(-1.0, face_cosine)));

    // Preserve coordinated_max_turn_rate_rad_s operation order: radius first,
    // then speed/radius.  Algebraic simplification changes binary64 results.
    const double turn_radius_m = opponent_speed_mps * opponent_speed_mps
        / (kStandardGravityMps2
            * std::sqrt(opponent_n_g * opponent_n_g - 1.0));
    const double maximum_turn_rate_rad_s =
        opponent_speed_mps / turn_radius_m;
    const double time_to_bite_s =
        face_angle_rad / maximum_turn_rate_rad_s;

    const Vector3 relative_velocity_mps = Subtract(
        frame.own.velocity_ned_mps,
        frame.opponent.velocity_ned_mps);
    const Vector3 position_at_bite_m{{
        relative_position_m[0]
            + relative_velocity_mps[0] * time_to_bite_s,
        relative_position_m[1]
            + relative_velocity_mps[1] * time_to_bite_s,
        relative_position_m[2]
            + relative_velocity_mps[2] * time_to_bite_s}};
    const double range_at_bite_m = NumpyNorm3(position_at_bite_m);
    PursuitOptionalDouble time_to_exit_s{};
    FinalOutwardWeaponRangeExitTime(
        relative_position_m,
        relative_velocity_mps,
        weapon_outer_range_m,
        time_to_bite_s,
        time_to_exit_s);
    const bool already_outside = separation_m > weapon_outer_range_m;
    const bool open = time_to_exit_s.has_value
        && (time_to_exit_s.value < time_to_bite_s
            || (already_outside && time_to_bite_s == 0.0));

    output.admitted = true;
    output.status = open
        ? PursuitEscapeWindowStatus::Open
        : PursuitEscapeWindowStatus::Closed;
    output.range_at_bite_m = Optional(range_at_bite_m);
    output.initial_opponent_face_angle_rad = Optional(face_angle_rad);
    output.opponent_maximum_turn_rate_rad_s =
        Optional(maximum_turn_rate_rad_s);
    output.time_to_bite_s = Optional(time_to_bite_s);
    output.time_to_weapon_range_exit_s = time_to_exit_s;
    if (time_to_exit_s.has_value)
    {
        output.time_margin_s = Optional(
            time_to_bite_s - time_to_exit_s.value);
    }
    output.reason = open
        ? PursuitEscapeWindowReason::WeaponRangeExitBeforeOpponentFace
        : PursuitEscapeWindowReason::OpponentFaceBeforeOrAtWeaponRangeExit;
    return true;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* PursuitOvershootShadowProvenance() noexcept
{
    return "MANUAL_COUNTER_SCISSORS_PREDICTION_DUTY_SHADOW_RIDE_ALONG";
}

const char* PursuitEscapeWindowScope() noexcept
{
    return "MANUAL_G19_CURRENT_RELATIVE_PATH_WEAPON_EXIT_BEFORE_MAX_TURN_FACE";
}

const char* PursuitEscapeWindowReasonLabel(
    const PursuitEscapeWindowReason reason) noexcept
{
    switch (reason)
    {
    case PursuitEscapeWindowReason::OpponentTurnCapabilityNotAdmitted:
        return "opponent_turn_capability_not_admitted";
    case PursuitEscapeWindowReason::WeaponRangeExitBeforeOpponentFace:
        return "weapon_range_exit_before_opponent_face";
    case PursuitEscapeWindowReason::OpponentFaceBeforeOrAtWeaponRangeExit:
        return "opponent_face_before_or_at_weapon_range_exit";
    default:
        return "opponent_turn_capability_not_admitted";
    }
}

const char* PursuitOvershootForecastReasonLabel(
    const PursuitOvershootForecastReason reason) noexcept
{
    switch (reason)
    {
    case PursuitOvershootForecastReason::PublicationNotAdmitted:
        return "publication_not_admitted";
    case PursuitOvershootForecastReason::FrameStateNotFinite:
        return "frame_state_not_finite";
    case PursuitOvershootForecastReason::AdversaryCourseNotResolved:
        return "adversary_course_not_resolved";
    case PursuitOvershootForecastReason::OwnAlreadyAhead:
        return "own_already_ahead";
    case PursuitOvershootForecastReason::AlongTrackSignNotResolved:
        return "along_track_sign_not_resolved";
    case PursuitOvershootForecastReason::ClosureNotResolvedPositive:
        return "closure_not_resolved_positive";
    case PursuitOvershootForecastReason::PublicationDomainNotTrusted:
        return "publication_domain_not_trusted";
    case PursuitOvershootForecastReason::MaintainedPursuitOvershootForced:
        return "maintained_pursuit_overshoot_forced";
    case PursuitOvershootForecastReason::ArrestNotRefuted:
        return "arrest_not_refuted";
    default:
        return "publication_not_admitted";
    }
}

PursuitOvershootForecaster::PursuitOvershootForecaster() noexcept = default;

PursuitOvershootForecaster::PursuitOvershootForecaster(
    const PursuitOvershootForecasterConfig& config) noexcept
    : config_(config)
{
}

void PursuitOvershootForecaster::Update(
    const DogfightGeometryFrame& frame,
    PursuitOvershootForecast& output,
    Status& status) const noexcept
{
    output = PursuitOvershootForecast{};
    status = Status{};
    PursuitOvershootForecast result{};

    const em::EmTableAuthority authority = envelope_.Authority();
    if (!authority.table_identity_valid
        || !authority.schema_valid
        || !authority.provenance_valid)
    {
        MakeUnresolved(
            result,
            PursuitOvershootForecastReason::PublicationNotAdmitted);
        output = result;
        return;
    }

    const Vector3& own_pos = frame.own.position_ned_m;
    const Vector3& own_vel = frame.own.velocity_ned_mps;
    const Vector3& opponent_pos = frame.opponent.position_ned_m;
    const Vector3& opponent_vel = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_pos)
        || !FiniteVector(own_vel)
        || !FiniteVector(opponent_pos)
        || !FiniteVector(opponent_vel))
    {
        MakeUnresolved(
            result,
            PursuitOvershootForecastReason::FrameStateNotFinite);
        output = result;
        return;
    }

    double own_position_bound_m = 0.0;
    double own_velocity_bound_mps = 0.0;
    double opponent_position_bound_m = 0.0;
    double opponent_velocity_bound_mps = 0.0;
    if (!Float32WireVectorErrorBound(own_pos, own_position_bound_m)
        || !Float32WireVectorErrorBound(own_vel, own_velocity_bound_mps)
        || !Float32WireVectorErrorBound(
            opponent_pos,
            opponent_position_bound_m)
        || !Float32WireVectorErrorBound(
            opponent_vel,
            opponent_velocity_bound_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const double own_speed_mps = NumpyNorm3(own_vel);
    const double opponent_speed_mps = NumpyNorm3(opponent_vel);
    const double own_altitude_m = -own_pos[2];
    const double opponent_altitude_m = -opponent_pos[2];
    result.speed_advantage_mps = Optional(
        own_speed_mps - opponent_speed_mps);

    em::PursuitOvershootEmQuery em_query{};
    envelope_.ObservePursuitOvershootComposite(
        own_speed_mps,
        own_altitude_m,
        config_.mass_kg_available,
        config_.mass_kg,
        em_query);

    const double own_energy_m = own_altitude_m
        + own_speed_mps * own_speed_mps / (2.0 * kStandardGravityMps2);
    const double opponent_energy_m = opponent_altitude_m
        + opponent_speed_mps * opponent_speed_mps
            / (2.0 * kStandardGravityMps2);
    const double energy_band_m = (std::max)(
        (std::max)(std::fabs(own_energy_m), std::fabs(opponent_energy_m)),
        1.0) * kFloat32Epsilon;
    const double standing_gap_m = own_energy_m - opponent_energy_m;
    result.energy_standing_gap_m = Optional(standing_gap_m);
    if (em_query.sustained_corner_mps.has_value
        && std::isfinite(em_query.sustained_corner_mps.value)
        && em_query.sustained_corner_mps.value > 0.0)
    {
        const double corner_mps = em_query.sustained_corner_mps.value;
        const double zoom_budget_m = (std::max)(
            0.0,
            (own_speed_mps * own_speed_mps - corner_mps * corner_mps)
                / (2.0 * kStandardGravityMps2));
        result.zoom_budget_m = Optional(zoom_budget_m);
        result.branch_a_admitted = standing_gap_m > energy_band_m
            && zoom_budget_m > energy_band_m;
    }

    const Vector3 relative_position_m = Subtract(own_pos, opponent_pos);
    const double separation_m = NumpyNorm3(relative_position_m);
    if (separation_m > 0.0)
    {
        if (!BuildEscapeWindow(
                frame,
                relative_position_m,
                separation_m,
                opponent_speed_mps,
                opponent_altitude_m,
                envelope_,
                result.escape_window,
                status))
        {
            return;
        }
    }

    const double opponent_horizontal_mps = PythonMathHypot2(
        opponent_vel[0],
        opponent_vel[1]);
    if (!(opponent_horizontal_mps - opponent_velocity_bound_mps > 0.0))
    {
        MakeUnresolved(
            result,
            PursuitOvershootForecastReason::AdversaryCourseNotResolved);
        output = result;
        return;
    }
    const double course_north = opponent_vel[0] / opponent_horizontal_mps;
    const double course_east = opponent_vel[1] / opponent_horizontal_mps;

    const double own_horizontal_mps = PythonMathHypot2(
        own_vel[0],
        own_vel[1]);
    if (own_horizontal_mps - own_velocity_bound_mps > 0.0)
    {
        const double own_course_north = own_vel[0] / own_horizontal_mps;
        const double own_course_east = own_vel[1] / own_horizontal_mps;
        const double cosine = own_course_north * course_north
            + own_course_east * course_east;
        result.crossing_angle_rad = Optional(std::acos((std::min)(
            1.0,
            (std::max)(-1.0, cosine))));
    }

    const double along_track_m =
        (own_pos[0] - opponent_pos[0]) * course_north
        + (own_pos[1] - opponent_pos[1]) * course_east;
    const double closure_mps =
        (own_vel[0] - opponent_vel[0]) * course_north
        + (own_vel[1] - opponent_vel[1]) * course_east;
    const double position_bound_m =
        own_position_bound_m + opponent_position_bound_m;
    const double velocity_bound_mps =
        own_velocity_bound_mps + opponent_velocity_bound_mps;
    result.along_track_m = Optional(along_track_m);
    result.closure_mps = Optional(closure_mps);
    result.valid = true;

    if (along_track_m - position_bound_m > 0.0)
    {
        result.status = PursuitOvershootForecastStatus::NotForced;
        result.reason = PursuitOvershootForecastReason::OwnAlreadyAhead;
        output = result;
        return;
    }
    if (!(along_track_m + position_bound_m < 0.0))
    {
        result.status = PursuitOvershootForecastStatus::NotForced;
        result.reason =
            PursuitOvershootForecastReason::AlongTrackSignNotResolved;
        output = result;
        return;
    }
    const double closure_lower_mps = closure_mps - velocity_bound_mps;
    if (!(closure_lower_mps > 0.0))
    {
        result.status = PursuitOvershootForecastStatus::NotForced;
        result.reason =
            PursuitOvershootForecastReason::ClosureNotResolvedPositive;
        output = result;
        return;
    }

    if (!em_query.arrest_band_trusted
        || !em_query.optimistic_arrest_mps2.has_value)
    {
        MakeUnresolved(
            result,
            PursuitOvershootForecastReason::PublicationDomainNotTrusted);
        output = result;
        return;
    }
    const double arrest_mps2 = em_query.optimistic_arrest_mps2.value;
    const double room_m = -along_track_m + position_bound_m;
    result.optimistic_arrest_mps2 = Optional(arrest_mps2);
    result.maintained_passage_projected = true;
    if (arrest_mps2 <= 0.0)
    {
        result.status = PursuitOvershootForecastStatus::Forced;
        result.reason = PursuitOvershootForecastReason::
            MaintainedPursuitOvershootForced;
        result.braking_distance_m = Optional(
            (std::numeric_limits<double>::infinity)());
        output = result;
        return;
    }

    const double braking_distance_m = closure_lower_mps * closure_lower_mps
        / (2.0 * arrest_mps2);
    result.braking_distance_m = Optional(braking_distance_m);
    if (room_m < braking_distance_m)
    {
        result.status = PursuitOvershootForecastStatus::Forced;
        result.reason = PursuitOvershootForecastReason::
            MaintainedPursuitOvershootForced;
    }
    else
    {
        result.status = PursuitOvershootForecastStatus::NotForced;
        result.reason = PursuitOvershootForecastReason::ArrestNotRefuted;
    }
    output = result;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
