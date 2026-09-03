#include "LadyLuck/guidance/obfm/ObfmSpacingOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::ObfmEnergyRateAuthorityObservation;
using LadyLuck::ObfmEnergyRateAuthorityStatus;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::ObfmSpacingArrivalFeasibilityReceipt;
using LadyLuck::guidance::obfm::ObfmSpacingArrestPathAllocation;
using LadyLuck::guidance::obfm::ObfmSpacingGeometry;
using LadyLuck::guidance::obfm::ObfmSpacingGuidanceCommand;
using LadyLuck::guidance::obfm::ObfmSpacingOwnerPhase;
using LadyLuck::guidance::obfm::ObfmSpacingOwnerReason;
using LadyLuck::guidance::obfm::ObfmSpacingOwnerServiceReceipt;
using LadyLuck::guidance::obfm::ObfmSpacingOwnerTaskReceipt;
using LadyLuck::guidance::obfm::ObfmSpacingReacquireGeometry;
using LadyLuck::guidance::obfm::ObfmSpacingRecoveryVelocityBoundReceipt;
using LadyLuck::guidance::obfm::ObfmSpacingSafetyGrade;
using LadyLuck::guidance::obfm::ObfmSpacingSafetyReceipt;

enum class CalculationDomain : std::uint8_t
{
    Available = 0U,
    FiniteUnavailable = 1U,
    DeclaredReadyContradiction = 2U
};

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool MultiplyFinite(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    if (left == 0.0 || right == 0.0)
    {
        return true;
    }
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    // Only the >1 quadrant can overflow.  Avoid max/small, which can itself
    // overflow inside an overflow guard.  Equality is rejected because the
    // exact product may lie above DBL_MAX while max/right rounded upward.
    if (absolute_left > 1.0
        && absolute_right > 1.0
        && absolute_left >= maximum / absolute_right)
    {
        return false;
    }
    output = left * right;
    return std::isfinite(output);
}

bool AddFinite(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    // The guard subtractions stay inside [-DBL_MAX, DBL_MAX].  Conservative
    // equality also contains round-to-nearest boundary ambiguity.
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    return std::isfinite(output);
}

bool SubtractFinite(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(right))
    {
        return false;
    }
    return AddFinite(left, -right, output);
}

bool DivideFinite(
    const double numerator,
    const double denominator,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(numerator)
        || !std::isfinite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    if (numerator != 0.0)
    {
        const double absolute_numerator = std::fabs(numerator);
        const double absolute_denominator = std::fabs(denominator);
        if (absolute_denominator < 1.0)
        {
            // max*|d| cannot overflow in this branch and is never subnormal
            // for a nonzero binary64 denominator.  Reject equality to contain
            // a quotient whose exact value may round beyond DBL_MAX.
            const double safe_numerator_limit =
                (std::numeric_limits<double>::max)()
                * absolute_denominator;
            if (absolute_numerator >= safe_numerator_limit)
            {
                return false;
            }
        }
    }
    output = numerator / denominator;
    return std::isfinite(output);
}

bool ScaleFinite(
    const Vector3& value,
    const double scalar,
    Vector3& output) noexcept
{
    output = Vector3{};
    return MultiplyFinite(value[0], scalar, output[0])
        && MultiplyFinite(value[1], scalar, output[1])
        && MultiplyFinite(value[2], scalar, output[2]);
}

bool AddFinite(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return AddFinite(left[0], right[0], output[0])
        && AddFinite(left[1], right[1], output[1])
        && AddFinite(left[2], right[2], output[2]);
}

bool SubtractFinite(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    return SubtractFinite(left[0], right[0], output[0])
        && SubtractFinite(left[1], right[1], output[1])
        && SubtractFinite(left[2], right[2], output[2]);
}

bool DotFinite(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    output = 0.0;
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double partial = 0.0;
    return MultiplyFinite(left[0], right[0], term0)
        && MultiplyFinite(left[1], right[1], term1)
        && MultiplyFinite(left[2], right[2], term2)
        && AddFinite(term0, term1, partial)
        && AddFinite(partial, term2, output);
}

bool CrossFinite(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    output = Vector3{};
    double positive = 0.0;
    double negative = 0.0;
    if (!MultiplyFinite(left[1], right[2], positive)
        || !MultiplyFinite(left[2], right[1], negative)
        || !SubtractFinite(positive, negative, output[0])
        || !MultiplyFinite(left[2], right[0], positive)
        || !MultiplyFinite(left[0], right[2], negative)
        || !SubtractFinite(positive, negative, output[1])
        || !MultiplyFinite(left[0], right[1], positive)
        || !MultiplyFinite(left[1], right[0], negative)
        || !SubtractFinite(positive, negative, output[2]))
    {
        output = Vector3{};
        return false;
    }
    return true;
}

bool NormFinite(
    const Vector3& value,
    const std::size_t dimension,
    double& output) noexcept
{
    output = 0.0;
    if ((dimension != 2U && dimension != 3U) || !FiniteVector(value))
    {
        return false;
    }
    double scale = (std::max)(std::fabs(value[0]), std::fabs(value[1]));
    if (dimension == 3U)
    {
        scale = (std::max)(scale, std::fabs(value[2]));
    }
    if (scale == 0.0)
    {
        return true;
    }
    double sum = 0.0;
    for (std::size_t index = 0U; index < dimension; ++index)
    {
        double normalized = 0.0;
        double square = 0.0;
        double next_sum = 0.0;
        if (!DivideFinite(value[index], scale, normalized)
            || !MultiplyFinite(normalized, normalized, square)
            || !AddFinite(sum, square, next_sum))
        {
            return false;
        }
        sum = next_sum;
    }
    const double root = std::sqrt(sum);
    double admitted_bound = 0.0;
    if (!std::isfinite(root) || !MultiplyFinite(scale, root, admitted_bound))
    {
        return false;
    }
    // The scaled calculation above proves the mathematical norm is inside
    // binary64 before std::hypot is called.  Keep the frozen ordinary-domain
    // operation and rounding order used by da8 and the existing receipts.
    output = dimension == 2U
        ? std::hypot(value[0], value[1])
        : std::hypot(std::hypot(value[0], value[1]), value[2]);
    return std::isfinite(output);
}

bool Norm2Finite(const Vector3& value, double& output) noexcept
{
    return NormFinite(value, 2U, output);
}

bool Norm3Finite(const Vector3& value, double& output) noexcept
{
    return NormFinite(value, 3U, output);
}

bool Unit(
    const Vector3& value,
    Vector3& output,
    double& magnitude) noexcept
{
    output = Vector3{};
    if (!Norm3Finite(value, magnitude)
        || magnitude <= LadyLuck::constants::Tiny)
    {
        return false;
    }
    double inverse = 0.0;
    if (!DivideFinite(1.0, magnitude, inverse)
        || !ScaleFinite(value, inverse, output))
    {
        return false;
    }
    return FiniteVector(output);
}

double ClipUnit(const double value) noexcept
{
    return (std::max)(-1.0, (std::min)(1.0, value));
}

bool Float32Scalar(const double value) noexcept
{
    if (!std::isfinite(value))
    {
        return false;
    }
    const double float_maximum = static_cast<double>(
        (std::numeric_limits<float>::max)());
    if (std::fabs(value) > float_maximum)
    {
        return false;
    }
    // Every finite binary32 encoding lies in this closed magnitude interval.
    // The later ABI conversion is therefore safe; no speculative narrowing
    // conversion is needed merely to prove the domain here.
    return true;
}

bool Float32Vector(const Vector3& value) noexcept
{
    return Float32Scalar(value[0])
        && Float32Scalar(value[1])
        && Float32Scalar(value[2]);
}

bool Float32Command(const ObfmSpacingGuidanceCommand& command) noexcept
{
    return Float32Vector(command.aim_point_ned_m)
        && Float32Scalar(command.desired_speed_mps)
        && Float32Scalar(command.desired_speed_rate_mps2)
        && Float32Scalar(command.specific_energy_rate_bias_m2ps3)
        && Float32Scalar(command.capture_range_des_m);
}

bool FiniteFrameKinematics(const DogfightGeometryFrame& frame) noexcept
{
    return std::isfinite(frame.t_sec)
        && FiniteVector(frame.own.position_ned_m)
        && FiniteVector(frame.own.velocity_ned_mps)
        && FiniteVector(frame.own.nose_ned)
        && FiniteVector(frame.opponent.position_ned_m)
        && FiniteVector(frame.opponent.velocity_ned_mps)
        && std::isfinite(frame.own_offense.damage_rate)
        && std::isfinite(frame.own_offense.phase.min_range_m)
        && std::isfinite(frame.own_offense.phase.max_range_m);
}

bool SameOwnerIdentity(
    const DogfightGeometryFrame& frame,
    const std::uint64_t episode_epoch,
    const std::int32_t own_plane_id,
    const std::int32_t target_plane_id) noexcept
{
    return frame.frame_identity.episode_epoch == episode_epoch
        && frame.own_plane_id == own_plane_id
        && frame.target_plane_id == target_plane_id;
}

CalculationDomain BuildAxisGeometry(
    const DogfightGeometryFrame& frame,
    const double official_range_m,
    ObfmSpacingGeometry& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingGeometry{};
    reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    if (!FiniteFrameKinematics(frame))
    {
        reason = ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (!std::isfinite(official_range_m)
        || official_range_m <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::OfficialRangeInvalid;
        return CalculationDomain::FiniteUnavailable;
    }

    Vector3 target_horizontal = frame.opponent.velocity_ned_mps;
    target_horizontal[2] = 0.0;
    double target_speed = 0.0;
    if (!Norm2Finite(target_horizontal, target_speed))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (target_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::TargetHorizontalCourseUndefined;
        return CalculationDomain::FiniteUnavailable;
    }
    double inverse_target_speed = 0.0;
    Vector3 target_axis{};
    double own_speed = 0.0;
    if (!DivideFinite(1.0, target_speed, inverse_target_speed)
        || !ScaleFinite(target_horizontal, inverse_target_speed, target_axis)
        || !Norm3Finite(frame.own.velocity_ned_mps, own_speed))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (own_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::OwnSpeedNotPositive;
        return CalculationDomain::FiniteUnavailable;
    }

    Vector3 relative{};
    if (!SubtractFinite(
            frame.opponent.position_ned_m,
            frame.own.position_ned_m,
            relative))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    relative[2] = 0.0;
    Vector3 official_displacement{};
    Vector3 station_error{};
    double signed_spacing = 0.0;
    Vector3 along_track{};
    Vector3 cross_track{};
    Vector3 relative_velocity{};
    double projected_closure = 0.0;
    double structural_rate = 0.0;
    double structural_closure = 0.0;
    Vector3 cross_track_velocity{};
    Vector3 arrest_velocity{};
    double arrest_speed = 0.0;
    if (!ScaleFinite(target_axis, official_range_m, official_displacement)
        || !SubtractFinite(relative, official_displacement, station_error)
        || !DotFinite(station_error, target_axis, signed_spacing)
        || !ScaleFinite(target_axis, signed_spacing, along_track)
        || !SubtractFinite(station_error, along_track, cross_track)
        || !SubtractFinite(
            frame.own.velocity_ned_mps,
            frame.opponent.velocity_ned_mps,
            relative_velocity)
        || !DotFinite(relative_velocity, target_axis, projected_closure)
        || !DivideFinite(target_speed, official_range_m, structural_rate)
        || !MultiplyFinite(
            structural_rate,
            signed_spacing,
            structural_closure)
        || !ScaleFinite(cross_track, structural_rate, cross_track_velocity)
        || !AddFinite(
            target_horizontal,
            cross_track_velocity,
            arrest_velocity)
        || !Norm2Finite(arrest_velocity, arrest_speed))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (arrest_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::ArrestHorizontalReferenceUndefined;
        return CalculationDomain::FiniteUnavailable;
    }

    output.valid = true;
    output.target_axis_ned = target_axis;
    output.station_error_horizontal_m = station_error;
    output.cross_track_error_horizontal_m = cross_track;
    output.target_horizontal_velocity_ned_mps = target_horizontal;
    output.arrest_horizontal_velocity_ned_mps = arrest_velocity;
    output.signed_station_spacing_m = signed_spacing;
    output.projected_closure_mps = projected_closure;
    output.structural_station_closure_mps = structural_closure;
    output.target_horizontal_speed_mps = target_speed;
    output.arrest_horizontal_speed_mps = arrest_speed;
    output.own_speed_mps = own_speed;
    output.official_range_m = official_range_m;
    return CalculationDomain::Available;
}

CalculationDomain BuildReacquireGeometry(
    const DogfightGeometryFrame& frame,
    ObfmSpacingReacquireGeometry& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingReacquireGeometry{};
    reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    if (!FiniteFrameKinematics(frame))
    {
        reason = ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    const double minimum_m = frame.own_offense.phase.min_range_m;
    const double maximum_m = frame.own_offense.phase.max_range_m;
    if (minimum_m <= LadyLuck::constants::Tiny
        || maximum_m <= minimum_m)
    {
        reason = ObfmSpacingOwnerReason::OfficialRangeInvalid;
        return CalculationDomain::FiniteUnavailable;
    }
    double range_sum_m = 0.0;
    double capture_range_m = 0.0;
    if (!AddFinite(minimum_m, maximum_m, range_sum_m)
        || !MultiplyFinite(0.5, range_sum_m, capture_range_m)
        || capture_range_m <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::OfficialRangeInvalid;
        return CalculationDomain::FiniteUnavailable;
    }

    Vector3 target_course{};
    double target_speed = 0.0;
    if (!Unit(frame.opponent.velocity_ned_mps, target_course, target_speed))
    {
        reason = ObfmSpacingOwnerReason::TargetCourseUndefined;
        return CalculationDomain::FiniteUnavailable;
    }
    double own_speed = 0.0;
    if (!Norm3Finite(frame.own.velocity_ned_mps, own_speed)
        || own_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::OwnSpeedNotPositive;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 relative{};
    if (!SubtractFinite(
            frame.opponent.position_ned_m,
            frame.own.position_ned_m,
            relative))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 los{};
    double slant_range = 0.0;
    if (!Unit(relative, los, slant_range))
    {
        reason = ObfmSpacingOwnerReason::CoincidentGeometry;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 capture_displacement{};
    Vector3 station_error{};
    double structural_rate = 0.0;
    Vector3 station_correction{};
    Vector3 station_velocity{};
    double station_speed = 0.0;
    double station_horizontal_speed = 0.0;
    double signed_spacing = 0.0;
    if (!ScaleFinite(target_course, capture_range_m, capture_displacement)
        || !SubtractFinite(relative, capture_displacement, station_error)
        || !DivideFinite(target_speed, capture_range_m, structural_rate)
        || !ScaleFinite(station_error, structural_rate, station_correction)
        || !AddFinite(
            frame.opponent.velocity_ned_mps,
            station_correction,
            station_velocity)
        || !Norm3Finite(station_velocity, station_speed)
        || !Norm2Finite(station_velocity, station_horizontal_speed)
        || !DotFinite(station_error, target_course, signed_spacing))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (station_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::ReacquireStationVelocityUndefined;
        return CalculationDomain::FiniteUnavailable;
    }
    if (station_horizontal_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::ReacquireHorizontalCourseUndefined;
        return CalculationDomain::FiniteUnavailable;
    }

    output.valid = true;
    output.target_course_ned = target_course;
    output.target_velocity_ned_mps = frame.opponent.velocity_ned_mps;
    output.own_velocity_ned_mps = frame.own.velocity_ned_mps;
    output.los_direction_ned = los;
    output.station_error_ned_m = station_error;
    output.station_velocity_ned_mps = station_velocity;
    output.signed_station_spacing_m = signed_spacing;
    output.slant_range_m = slant_range;
    output.target_speed_mps = target_speed;
    output.station_horizontal_speed_mps = station_horizontal_speed;
    output.station_speed_mps = station_speed;
    output.own_speed_mps = own_speed;
    output.structural_rate_per_s = structural_rate;
    output.capture_range_m = capture_range_m;
    output.official_min_range_m = minimum_m;
    output.official_max_range_m = maximum_m;
    return CalculationDomain::Available;
}

CalculationDomain ValidateEnergyAuthority(
    const ObfmEnergyRateAuthorityObservation& authority,
    const bool neutral_required,
    ObfmSpacingOwnerReason& reason) noexcept
{
    if (!authority.evaluated || !authority.valid
        || authority.status != ObfmEnergyRateAuthorityStatus::AuthorityAvailable)
    {
        reason = ObfmSpacingOwnerReason::EnergyAuthorityUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!LadyLuck::IsValidControlFrameIdentity(authority.source_frame_identity)
        || !std::isfinite(authority.reference_min_m2ps3)
        || !std::isfinite(authority.reference_max_m2ps3)
        || authority.reference_min_m2ps3 > authority.reference_max_m2ps3)
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadyEnergyAuthorityContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (neutral_required)
    {
        if (!(authority.reference_min_m2ps3 <= 0.0
            && 0.0 <= authority.reference_max_m2ps3))
        {
            reason = ObfmSpacingOwnerReason::NeutralEnergyAuthorityUnavailable;
            return CalculationDomain::FiniteUnavailable;
        }
    }
    else if (authority.reference_min_m2ps3 > 0.0)
    {
        reason = ObfmSpacingOwnerReason::
            EnergyAuthorityDoesNotAdmitDissipation;
        return CalculationDomain::FiniteUnavailable;
    }
    return CalculationDomain::Available;
}

CalculationDomain ProjectPreviousEnergyBias(
    const ObfmEnergyRateAuthorityObservation& authority,
    const double raw_bias_m2ps3,
    const bool neutral_required,
    double& admitted_bias_m2ps3,
    ObfmSpacingOwnerReason& reason) noexcept
{
    admitted_bias_m2ps3 = 0.0;
    if (!std::isfinite(raw_bias_m2ps3))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const CalculationDomain authority_domain = ValidateEnergyAuthority(
        authority,
        neutral_required,
        reason);
    if (authority_domain != CalculationDomain::Available)
    {
        // Energy-bias shaping is optional.  Preserve the finite aim-point and
        // speed command and its raw nonpositive bias when no compatible prior
        // controller authority is available.  TECS owns the final physical
        // energy-rate and thrust saturation.
        admitted_bias_m2ps3 = neutral_required ? 0.0 : raw_bias_m2ps3;
        return CalculationDomain::Available;
    }
    if (neutral_required)
    {
        admitted_bias_m2ps3 = 0.0;
        return CalculationDomain::Available;
    }
    const double raw_reference = raw_bias_m2ps3;
    admitted_bias_m2ps3 = raw_reference
        < authority.reference_min_m2ps3
        ? authority.reference_min_m2ps3
        : raw_reference > authority.reference_max_m2ps3
        ? authority.reference_max_m2ps3
        : raw_reference;
    if (!std::isfinite(admitted_bias_m2ps3)
        || admitted_bias_m2ps3 > 0.0)
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadyEnergyAuthorityContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    return CalculationDomain::Available;
}

CalculationDomain ValidateGammaLimit(
    const bool available,
    const double value,
    double& gamma_limit,
    ObfmSpacingOwnerReason& reason) noexcept
{
    gamma_limit = 0.0;
    if (!available)
    {
        reason = ObfmSpacingOwnerReason::SafetySampleUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!std::isfinite(value))
    {
        reason = ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    gamma_limit = std::fabs(value);
    if (gamma_limit <= 0.0
        || gamma_limit >= 0.5 * LadyLuck::constants::Pi)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    return CalculationDomain::Available;
}

CalculationDomain BuildArrestPathAllocation(
    const ObfmSpacingGeometry& geometry,
    const bool gamma_available,
    const double gamma_value,
    ObfmSpacingArrestPathAllocation& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingArrestPathAllocation{};
    if (!geometry.valid
        || !FiniteVector(geometry.arrest_horizontal_velocity_ned_mps)
        || !FiniteVector(geometry.target_axis_ned)
        || !std::isfinite(geometry.arrest_horizontal_speed_mps)
        || !std::isfinite(geometry.own_speed_mps)
        || !std::isfinite(geometry.signed_station_spacing_m)
        || geometry.signed_station_spacing_m <= 0.0
        || geometry.arrest_horizontal_speed_mps
            <= LadyLuck::constants::Tiny
        || geometry.own_speed_mps <= LadyLuck::constants::Tiny
        || geometry.arrest_horizontal_speed_mps >= geometry.own_speed_mps)
    {
        reason = geometry.signed_station_spacing_m <= 0.0
            ? ObfmSpacingOwnerReason::StationNotAhead
            : geometry.arrest_horizontal_speed_mps >= geometry.own_speed_mps
            ? ObfmSpacingOwnerReason::ClimbAllocationUnavailable
            : ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    double gamma_limit = 0.0;
    const CalculationDomain gamma_domain = ValidateGammaLimit(
        gamma_available,
        gamma_value,
        gamma_limit,
        reason);
    if (gamma_domain != CalculationDomain::Available)
    {
        return gamma_domain;
    }

    // H and V are finite, positive, and H < V before division, so the acos
    // argument is strictly inside (0, 1); no clamp or fitted tolerance enters
    // the authority operation order.
    double ratio = 0.0;
    if (!DivideFinite(
            geometry.arrest_horizontal_speed_mps,
            geometry.own_speed_mps,
            ratio)
        || ratio <= 0.0
        || ratio >= 1.0)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double raw_gamma = std::acos(ratio);
    if (!std::isfinite(raw_gamma) || raw_gamma < 0.0)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double admitted_gamma = (std::min)(raw_gamma, gamma_limit);
    const double sine = std::sin(admitted_gamma);
    const double cosine = std::cos(admitted_gamma);
    if (!std::isfinite(sine)
        || !std::isfinite(cosine)
        || sine <= 0.0
        || cosine <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    double inverse_horizontal_speed = 0.0;
    if (!DivideFinite(
            1.0,
            geometry.arrest_horizontal_speed_mps,
            inverse_horizontal_speed))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 horizontal_direction{};
    double target_axis_projection = 0.0;
    if (!ScaleFinite(
            geometry.arrest_horizontal_velocity_ned_mps,
            inverse_horizontal_speed,
            horizontal_direction)
        || !DotFinite(
            horizontal_direction,
            geometry.target_axis_ned,
            target_axis_projection))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    double speed_rate = 0.0;
    if (!MultiplyFinite(
            -LadyLuck::constants::StandardGravityMps2,
            sine,
            speed_rate))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double deceleration_partial = 0.0;
    double target_axis_deceleration = 0.0;
    if (!MultiplyFinite(
            -speed_rate,
            cosine,
            deceleration_partial)
        || !MultiplyFinite(
            deceleration_partial,
            target_axis_projection,
            target_axis_deceleration)
        || speed_rate >= 0.0
        || target_axis_deceleration <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::
            ArrivalArrestDecelerationUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    output.valid = true;
    output.raw_gamma_rad = raw_gamma;
    output.admitted_gamma_rad = admitted_gamma;
    output.desired_speed_rate_mps2 = speed_rate;
    output.target_axis_reference_deceleration_mps2 =
        target_axis_deceleration;
    return CalculationDomain::Available;
}

CalculationDomain EvaluateArrivalFeasibility(
    const ObfmSpacingGeometry& geometry,
    const double coordinate_closure_mps,
    const bool gamma_available,
    const double gamma_value,
    ObfmSpacingArrivalFeasibilityReceipt& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingArrivalFeasibilityReceipt{};
    output.evaluated = true;
    output.reason = ObfmSpacingOwnerReason::ArrivalArrestDistanceAvailable;
    if (!std::isfinite(geometry.projected_closure_mps)
        || !std::isfinite(coordinate_closure_mps)
        || !std::isfinite(geometry.signed_station_spacing_m))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        output.reason = reason;
        return CalculationDomain::FiniteUnavailable;
    }
    if (geometry.projected_closure_mps <= 0.0
        || coordinate_closure_mps <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::DualArrivalClosureNotPositive;
        output.reason = reason;
        return CalculationDomain::Available;
    }

    const CalculationDomain allocation_domain = BuildArrestPathAllocation(
        geometry,
        gamma_available,
        gamma_value,
        output.path_allocation,
        reason);
    if (allocation_domain != CalculationDomain::Available)
    {
        output.reason = reason;
        return allocation_domain;
    }

    const double deceleration = output.path_allocation
        .target_axis_reference_deceleration_mps2;
    if (deceleration <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::ArrivalArrestArithmeticUnavailable;
        output.reason = reason;
        return CalculationDomain::FiniteUnavailable;
    }

    // Preserve c*c and 2*a operation order, but do not perform an operation
    // until its representable binary64 domain has been admitted.
    double denominator = 0.0;
    double projected_numerator = 0.0;
    double coordinate_numerator = 0.0;
    if (!MultiplyFinite(2.0, deceleration, denominator)
        || !MultiplyFinite(
            geometry.projected_closure_mps,
            geometry.projected_closure_mps,
            projected_numerator)
        || !MultiplyFinite(
            coordinate_closure_mps,
            coordinate_closure_mps,
            coordinate_numerator)
        || denominator <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::ArrivalArrestArithmeticUnavailable;
        output.reason = reason;
        return CalculationDomain::FiniteUnavailable;
    }
    double projected_distance = 0.0;
    double coordinate_distance = 0.0;
    if (!DivideFinite(
            projected_numerator,
            denominator,
            projected_distance)
        || !DivideFinite(
            coordinate_numerator,
            denominator,
            coordinate_distance)
        || projected_distance < 0.0
        || coordinate_distance < 0.0)
    {
        reason = ObfmSpacingOwnerReason::ArrivalArrestArithmeticUnavailable;
        output.reason = reason;
        return CalculationDomain::FiniteUnavailable;
    }

    output.stopping_distances_available = true;
    output.projected_stopping_distance_m = projected_distance;
    output.coordinate_stopping_distance_m = coordinate_distance;
    output.admitted = projected_distance >= geometry.signed_station_spacing_m
        && coordinate_distance >= geometry.signed_station_spacing_m;
    output.reason = output.admitted
        ? ObfmSpacingOwnerReason::DualClosureArrivalInfeasibleLatched
        : ObfmSpacingOwnerReason::ArrivalArrestDistanceAvailable;
    reason = output.reason;
    return CalculationDomain::Available;
}

CalculationDomain BuildAxisCommand(
    const DogfightGeometryFrame& frame,
    const ObfmSpacingGeometry& geometry,
    const ObfmSpacingOwnerPhase phase,
    const bool gamma_available,
    const double gamma_value,
    const ObfmEnergyRateAuthorityObservation& authority,
    ObfmSpacingGuidanceCommand& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingGuidanceCommand{};
    if (!geometry.valid)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const bool path = phase == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase == ObfmSpacingOwnerPhase::PostHitRminArrest;
    const bool level = phase == ObfmSpacingOwnerPhase::LevelRecovery;
    if (!path && !level)
    {
        reason = ObfmSpacingOwnerReason::TaskLifecycleContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (geometry.signed_station_spacing_m <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::StationNotAhead;
        return CalculationDomain::FiniteUnavailable;
    }
    if (path
        && geometry.arrest_horizontal_speed_mps >= geometry.own_speed_mps)
    {
        reason = ObfmSpacingOwnerReason::ClimbAllocationUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!std::isfinite(geometry.signed_station_spacing_m)
        || !std::isfinite(geometry.arrest_horizontal_speed_mps)
        || geometry.arrest_horizontal_speed_mps
            <= LadyLuck::constants::Tiny
        || !std::isfinite(geometry.target_horizontal_speed_mps)
        || geometry.target_horizontal_speed_mps < 0.0
        || !std::isfinite(geometry.own_speed_mps)
        || geometry.own_speed_mps <= LadyLuck::constants::Tiny
        || !std::isfinite(geometry.official_range_m)
        || geometry.official_range_m <= LadyLuck::constants::Tiny
        || !FiniteVector(geometry.arrest_horizontal_velocity_ned_mps)
        || !FiniteVector(frame.own.position_ned_m))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double inverse_horizontal_speed = 0.0;
    Vector3 horizontal_direction{};
    if (!DivideFinite(
            1.0,
            geometry.arrest_horizontal_speed_mps,
            inverse_horizontal_speed)
        || !ScaleFinite(
            geometry.arrest_horizontal_velocity_ned_mps,
            inverse_horizontal_speed,
            horizontal_direction))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    ObfmSpacingArrestPathAllocation path_allocation{};
    double admitted_gamma = 0.0;
    if (path)
    {
        const CalculationDomain allocation_domain =
            BuildArrestPathAllocation(
            geometry,
            gamma_available,
            gamma_value,
            path_allocation,
            reason);
        if (allocation_domain != CalculationDomain::Available)
        {
            return allocation_domain;
        }
        admitted_gamma = path_allocation.admitted_gamma_rad;
    }

    const double cosine = std::cos(admitted_gamma);
    const double sine = std::sin(admitted_gamma);
    if (!std::isfinite(cosine)
        || !std::isfinite(sine)
        || cosine <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 desired_direction{};
    if (!ScaleFinite(
            horizontal_direction,
            cosine,
            desired_direction))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    desired_direction[2] = -sine;
    Vector3 aim_displacement{};
    Vector3 aim_point{};
    if (!ScaleFinite(
            desired_direction,
            geometry.official_range_m,
            aim_displacement)
        || !AddFinite(
            frame.own.position_ned_m,
            aim_displacement,
            aim_point))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double speed_rate_mps2 = path
        ? path_allocation.desired_speed_rate_mps2
        : 0.0;
    double structural_rate = 0.0;
    double gamma_limited_speed_target = 0.0;
    if (!DivideFinite(
            geometry.target_horizontal_speed_mps,
            geometry.official_range_m,
            structural_rate)
        || !DivideFinite(
            geometry.arrest_horizontal_speed_mps,
            cosine,
            gamma_limited_speed_target))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double speed_difference = 0.0;
    if (!SubtractFinite(
            geometry.own_speed_mps,
            gamma_limited_speed_target,
            speed_difference))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double speed_excess = (std::max)(0.0, speed_difference);
    double structural_sink = 0.0;
    double raw_bias = 0.0;
    if (!MultiplyFinite(
            structural_rate,
            speed_excess,
            structural_sink)
        || !MultiplyFinite(
            -geometry.own_speed_mps,
            structural_sink,
            raw_bias))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!FiniteVector(desired_direction)
        || !FiniteVector(aim_point)
        || !std::isfinite(speed_rate_mps2)
        || !std::isfinite(structural_rate)
        || !std::isfinite(gamma_limited_speed_target)
        || !std::isfinite(speed_excess)
        || !std::isfinite(structural_sink)
        || !std::isfinite(raw_bias))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const bool neutral = phase == ObfmSpacingOwnerPhase::PostHitRminArrest;
    double admitted_bias = 0.0;
    const CalculationDomain energy_domain = ProjectPreviousEnergyBias(
        authority,
        neutral ? 0.0 : raw_bias,
        neutral,
        admitted_bias,
        reason);
    if (energy_domain != CalculationDomain::Available)
    {
        return energy_domain;
    }

    output.aim_point_ned_m = aim_point;
    output.desired_speed_mps = geometry.own_speed_mps;
    output.desired_speed_rate_mps2 = speed_rate_mps2;
    output.specific_energy_rate_bias_m2ps3 = admitted_bias;
    output.path_inversion_allowed = false;
    output.capture_range_des_m = geometry.official_range_m;
    if (!Float32Command(output))
    {
        output = ObfmSpacingGuidanceCommand{};
        reason = ObfmSpacingOwnerReason::CommandFloat32DomainUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    reason = ObfmSpacingOwnerReason::CommandReady;
    return CalculationDomain::Available;
}

CalculationDomain BuildWezCommand(
    const DogfightGeometryFrame& frame,
    const ObfmSpacingReacquireGeometry& geometry,
    const bool gamma_available,
    const double gamma_value,
    const ObfmEnergyRateAuthorityObservation& authority,
    ObfmSpacingGuidanceCommand& output,
    ObfmSpacingOwnerReason& reason) noexcept
{
    output = ObfmSpacingGuidanceCommand{};
    if (!geometry.valid)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (frame.own_offense.damage_rate > 0.0)
    {
        reason = ObfmSpacingOwnerReason::OfficialEmployAvailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (geometry.slant_range_m <= geometry.official_min_range_m)
    {
        reason = ObfmSpacingOwnerReason::OfficialMinimumRangeReached;
        return CalculationDomain::FiniteUnavailable;
    }
    if (geometry.signed_station_spacing_m <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::OfficialMidrangeStationPassed;
        return CalculationDomain::FiniteUnavailable;
    }

    double gamma_limit = 0.0;
    const CalculationDomain gamma_domain = ValidateGammaLimit(
        gamma_available,
        gamma_value,
        gamma_limit,
        reason);
    if (gamma_domain != CalculationDomain::Available)
    {
        return gamma_domain;
    }
    double target_horizontal_speed = 0.0;
    if (!Norm2Finite(
            geometry.target_velocity_ned_mps,
            target_horizontal_speed)
        || target_horizontal_speed <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::TargetHorizontalCourseUndefined;
        return CalculationDomain::FiniteUnavailable;
    }
    const double target_gamma = std::atan2(
        -geometry.target_velocity_ned_mps[2],
        target_horizontal_speed);
    if (!std::isfinite(target_gamma))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (std::fabs(target_gamma) > gamma_limit)
    {
        reason = ObfmSpacingOwnerReason::TargetPathExceedsAuthority;
        return CalculationDomain::FiniteUnavailable;
    }

    Vector3 nose{};
    double nose_magnitude = 0.0;
    if (!Unit(frame.own.nose_ned, nose, nose_magnitude))
    {
        reason = ObfmSpacingOwnerReason::BoresightGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 velocity_direction{};
    double velocity_magnitude = 0.0;
    if (!Unit(
            geometry.own_velocity_ned_mps,
            velocity_direction,
            velocity_magnitude))
    {
        reason = ObfmSpacingOwnerReason::BoresightGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double raw_cosine = 0.0;
    if (!DotFinite(nose, geometry.los_direction_ned, raw_cosine))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double cosine = ClipUnit(raw_cosine);
    if (cosine <= 0.0)
    {
        reason = ObfmSpacingOwnerReason::BoresightTargetNotForward;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 axis{};
    double denominator = 0.0;
    if (!CrossFinite(nose, geometry.los_direction_ned, axis)
        || !AddFinite(1.0, cosine, denominator)
        || denominator <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::BoresightGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 first_cross{};
    Vector3 second_cross{};
    double inverse_denominator = 0.0;
    Vector3 second_correction{};
    Vector3 first_sum{};
    Vector3 rotated_velocity{};
    if (!CrossFinite(axis, velocity_direction, first_cross)
        || !CrossFinite(axis, first_cross, second_cross)
        || !DivideFinite(1.0, denominator, inverse_denominator)
        || !ScaleFinite(
            second_cross,
            inverse_denominator,
            second_correction)
        || !AddFinite(velocity_direction, first_cross, first_sum)
        || !AddFinite(first_sum, second_correction, rotated_velocity))
    {
        reason = ObfmSpacingOwnerReason::BoresightGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 compensated_direction{};
    double compensated_magnitude = 0.0;
    if (!Unit(
            rotated_velocity,
            compensated_direction,
            compensated_magnitude))
    {
        reason = ObfmSpacingOwnerReason::BoresightGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double horizontal_norm = 0.0;
    if (!Norm2Finite(compensated_direction, horizontal_norm)
        || horizontal_norm <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::ReacquireHorizontalCourseUndefined;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 horizontal_direction{};
    if (!DivideFinite(
            compensated_direction[0],
            horizontal_norm,
            horizontal_direction[0])
        || !DivideFinite(
            compensated_direction[1],
            horizontal_norm,
            horizontal_direction[1]))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double raw_gamma = std::atan2(
        -compensated_direction[2],
        horizontal_norm);
    if (!std::isfinite(raw_gamma))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    const double admitted_gamma = (std::min)(
        gamma_limit,
        (std::max)(-gamma_limit, raw_gamma));
    const double admitted_cosine = std::cos(admitted_gamma);
    const double admitted_sine = std::sin(admitted_gamma);
    if (!std::isfinite(admitted_cosine)
        || !std::isfinite(admitted_sine)
        || admitted_cosine <= LadyLuck::constants::Tiny)
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    Vector3 desired_direction{};
    Vector3 aim_displacement{};
    Vector3 aim_point{};
    double speed_rate = 0.0;
    if (!ScaleFinite(
            horizontal_direction,
            admitted_cosine,
            desired_direction))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    desired_direction[2] = -admitted_sine;
    if (!ScaleFinite(
            desired_direction,
            geometry.capture_range_m,
            aim_displacement)
        || !AddFinite(frame.own.position_ned_m, aim_displacement, aim_point)
        || !MultiplyFinite(
            -LadyLuck::constants::StandardGravityMps2,
            admitted_sine,
            speed_rate))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    double admitted_bias = 0.0;
    const CalculationDomain energy_domain = ProjectPreviousEnergyBias(
        authority,
        0.0,
        true,
        admitted_bias,
        reason);
    if (energy_domain != CalculationDomain::Available)
    {
        return energy_domain;
    }
    if (!FiniteVector(desired_direction)
        || !FiniteVector(aim_point)
        || !std::isfinite(speed_rate))
    {
        reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }

    output.aim_point_ned_m = aim_point;
    output.desired_speed_mps = geometry.own_speed_mps;
    output.desired_speed_rate_mps2 = speed_rate;
    output.specific_energy_rate_bias_m2ps3 = admitted_bias;
    output.path_inversion_allowed = false;
    output.capture_range_des_m = geometry.capture_range_m;
    if (!Float32Command(output))
    {
        output = ObfmSpacingGuidanceCommand{};
        reason = ObfmSpacingOwnerReason::CommandFloat32DomainUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    reason = ObfmSpacingOwnerReason::CommandReady;
    return CalculationDomain::Available;
}

CalculationDomain ApplyCurrentProjection(
    const ControlFrameIdentity& frame_identity,
    const LadyLuck::guidance::obfm::ObfmSpacingCurrentEnergyProjection&
        projection,
    ObfmSpacingGuidanceCommand& command,
    ObfmSpacingOwnerReason& reason) noexcept
{
    if (!projection.evaluated)
    {
        reason = ObfmSpacingOwnerReason::CurrentEnergyProjectionUnavailable;
        // Energy projection is an optional refinement.  The raw command has
        // already passed finite geometry/speed/gamma construction, so missing
        // projection evidence leaves that command intact.
        return CalculationDomain::Available;
    }
    if (!LadyLuck::IsValidControlFrameIdentity(projection.frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            frame_identity,
            projection.frame_identity)
        || !std::isfinite(projection.raw_bias_m2ps3)
        || !std::isfinite(projection.admitted_bias_m2ps3))
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadyProjectionContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (!projection.admitted)
    {
        reason = ObfmSpacingOwnerReason::CurrentEnergyProjectionRejected;
        return CalculationDomain::Available;
    }
    if (!projection.all_nonenergy_fields_unchanged)
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadyProjectionContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (projection.admitted_bias_m2ps3 > 0.0)
    {
        // A positive-bias projection is not admissible for Spacing arrest, but
        // it does not invalidate the already finite raw guidance command.
        reason = ObfmSpacingOwnerReason::CurrentEnergyProjectionRejected;
        return CalculationDomain::Available;
    }
    const ObfmSpacingGuidanceCommand raw_command = command;
    command.specific_energy_rate_bias_m2ps3 =
        projection.admitted_bias_m2ps3;
    if (!Float32Command(command))
    {
        command = raw_command;
        reason = ObfmSpacingOwnerReason::CurrentEnergyProjectionRejected;
        return CalculationDomain::Available;
    }
    reason = ObfmSpacingOwnerReason::CommandReady;
    return CalculationDomain::Available;
}

bool ResolveSafetyContract(
    const bool task_active,
    const bool post_hit_pending,
    const ObfmSpacingOwnerPhase phase,
    ObfmSpacingOwnerPhase& safety_phase,
    ObfmSpacingSafetyGrade& grade) noexcept
{
    safety_phase = ObfmSpacingOwnerPhase::Inactive;
    grade = ObfmSpacingSafetyGrade::None;
    if (!task_active)
    {
        safety_phase = post_hit_pending
            ? ObfmSpacingOwnerPhase::PostHitRminArrest
            : ObfmSpacingOwnerPhase::PathEnergyExchange;
        grade = ObfmSpacingSafetyGrade::StrictEntry;
        return true;
    }
    safety_phase = phase;
    if (phase == ObfmSpacingOwnerPhase::PostHitRminArrest)
    {
        grade = ObfmSpacingSafetyGrade::StrictEntry;
        return true;
    }
    if (phase == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase == ObfmSpacingOwnerPhase::LevelRecovery
        || phase == ObfmSpacingOwnerPhase::WezReacquire)
    {
        grade = ObfmSpacingSafetyGrade::RunningFaultOnly;
        return true;
    }
    return false;
}

CalculationDomain ValidateSafetyReceipt(
    const ControlFrameIdentity& expected_frame,
    const ObfmSpacingSafetyReceipt& receipt,
    const ObfmSpacingSafetyGrade grade,
    bool& evaluated,
    bool& admitted,
    ObfmSpacingOwnerReason& reason) noexcept
{
    evaluated = false;
    admitted = false;
    if ((receipt.strict_entry_admitted
            && !receipt.strict_entry_evaluated)
        || (receipt.running_fault_only_admitted
            && !receipt.running_fault_only_evaluated)
        || grade == ObfmSpacingSafetyGrade::None)
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadySafetyReceiptContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }

    if (grade == ObfmSpacingSafetyGrade::StrictEntry)
    {
        evaluated = receipt.strict_entry_evaluated;
        admitted = receipt.strict_entry_admitted;
    }
    else
    {
        evaluated = receipt.running_fault_only_evaluated;
        admitted = receipt.running_fault_only_admitted;
    }

    const bool any_evidence_declared =
        receipt.strict_entry_evaluated
        || receipt.strict_entry_admitted
        || receipt.running_fault_only_evaluated
        || receipt.running_fault_only_admitted;
    if (any_evidence_declared
        && (!IsValidControlFrameIdentity(receipt.frame_identity)
            || !SameControlFrameIdentity(
                expected_frame,
                receipt.frame_identity)))
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadySafetyReceiptContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    if (!evaluated)
    {
        reason = ObfmSpacingOwnerReason::SafetySampleUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!admitted)
    {
        reason = ObfmSpacingOwnerReason::SafetyNotAdmitted;
        return CalculationDomain::FiniteUnavailable;
    }
    return CalculationDomain::Available;
}

bool SelectedSafetyReceiptCoherent(
    const ObfmSpacingOwnerServiceReceipt& service) noexcept
{
    bool evaluated = false;
    bool admitted = false;
    ObfmSpacingOwnerReason reason =
        ObfmSpacingOwnerReason::SafetySampleUnavailable;
    return service.safety_phase != ObfmSpacingOwnerPhase::Inactive
        && service.safety_grade_required != ObfmSpacingSafetyGrade::None
        && service.safety_grade_evaluated
        && service.safety_grade_admitted
        && ValidateSafetyReceipt(
            service.frame_identity,
            service.safety,
            service.safety_grade_required,
            evaluated,
            admitted,
            reason) == CalculationDomain::Available
        && evaluated
        && admitted;
}

CalculationDomain ValidateRecoveryVelocityBound(
    const ControlFrameIdentity& expected_frame,
    const ObfmSpacingRecoveryVelocityBoundReceipt& receipt,
    double& bound_mps,
    ObfmSpacingOwnerReason& reason) noexcept
{
    bound_mps = 0.0;
    if (!receipt.evaluated)
    {
        reason = ObfmSpacingOwnerReason::
            RecoveryVelocityBoundUnavailable;
        return CalculationDomain::FiniteUnavailable;
    }
    if (!IsValidControlFrameIdentity(receipt.frame_identity)
        || !SameControlFrameIdentity(
            expected_frame,
            receipt.frame_identity)
        || !std::isfinite(receipt.own_down_velocity_error_bound_mps)
        || receipt.own_down_velocity_error_bound_mps < 0.0)
    {
        reason = ObfmSpacingOwnerReason::
            DeclaredReadyRecoveryVelocityBoundContradiction;
        return CalculationDomain::DeclaredReadyContradiction;
    }
    bound_mps = receipt.own_down_velocity_error_bound_mps;
    return CalculationDomain::Available;
}

void SetContractStatus(
    const ObfmSpacingOwnerReason reason,
    Status& status) noexcept
{
    switch (reason)
    {
    case ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite:
        status.code = StatusCode::NonFiniteInput;
        return;
    case ObfmSpacingOwnerReason::DeclaredReadyFrameIdentityInvalid:
    case ObfmSpacingOwnerReason::DeclaredReadyEnergyAuthorityContradiction:
    case ObfmSpacingOwnerReason::DeclaredReadyProjectionContradiction:
    case ObfmSpacingOwnerReason::TaskLifecycleContradiction:
    case ObfmSpacingOwnerReason::ServiceReceiptContradiction:
    case ObfmSpacingOwnerReason::DeclaredReadySafetyReceiptContradiction:
    case ObfmSpacingOwnerReason::
        DeclaredReadyRecoveryVelocityBoundContradiction:
        status.code = StatusCode::InvalidConfiguration;
        return;
    default:
        status.code = StatusCode::Ok;
        return;
    }
}

void SetSpacingRelease(
    ObfmSpacingOwnerTaskReceipt& output,
    const ObfmSpacingOwnerReason reason) noexcept
{
    output.release_required = true;
    output.candidate_valid = false;
    output.candidate_count = 0U;
    output.reason = reason;
}

void SetSpacingContractFailure(
    ObfmSpacingOwnerTaskReceipt& output,
    Status& status,
    const ObfmSpacingOwnerReason reason) noexcept
{
    output.candidate_valid = false;
    output.candidate_count = 0U;
    output.reason = reason;
    SetContractStatus(reason, status);
}

class SpacingReleaseAction final
{
public:
    explicit SpacingReleaseAction(
        ObfmSpacingOwnerTaskReceipt& output) noexcept
        : output_(output)
    {
    }

    void operator()(const ObfmSpacingOwnerReason reason) noexcept
    {
        SetSpacingRelease(output_, reason);
    }

private:
    ObfmSpacingOwnerTaskReceipt& output_;
};

class SpacingContractAction final
{
public:
    SpacingContractAction(
        ObfmSpacingOwnerTaskReceipt& output,
        Status& status) noexcept
        : output_(output), status_(status)
    {
    }

    void operator()(const ObfmSpacingOwnerReason reason) noexcept
    {
        SetSpacingContractFailure(output_, status_, reason);
    }

private:
    ObfmSpacingOwnerTaskReceipt& output_;
    Status& status_;
};

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

bool ObfmSpacingArithmeticBoundarySafeForTesting() noexcept
{
    const double maximum = (std::numeric_limits<double>::max)();
    const double multiplier = 1.5;
    const double boundary_multiplicand = maximum / multiplier;
    double scalar = 1.0;
    const bool multiply_boundary_rejected = !MultiplyFinite(
        multiplier,
        boundary_multiplicand,
        scalar) && scalar == 0.0;
    scalar = 1.0;
    const bool divide_boundary_rejected = !DivideFinite(
        maximum * 0.5,
        0.5,
        scalar) && scalar == 0.0;
    const bool float32_boundary_rejected = !Float32Scalar(maximum);

    const Vector3 extreme{{maximum, maximum, maximum}};
    double norm = 1.0;
    const bool norm2_rejected = !Norm2Finite(extreme, norm) && norm == 0.0;
    norm = 1.0;
    const bool norm3_rejected = !Norm3Finite(extreme, norm) && norm == 0.0;
    Vector3 vector{};
    const bool cross_rejected = !CrossFinite(
        extreme,
        Vector3{{maximum, -maximum, maximum}},
        vector) && vector == Vector3{};
    const bool aim_add_rejected = !AddFinite(
        extreme,
        Vector3{{maximum, 0.0, 0.0}},
        vector) && vector == Vector3{};
    return multiply_boundary_rejected
        && divide_boundary_rejected
        && float32_boundary_rejected
        && norm2_rejected
        && norm3_rejected
        && cross_rejected
        && aim_add_rejected;
}

const char* ObfmSpacingOwnerReasonLabel(
    const ObfmSpacingOwnerReason reason) noexcept
{
    switch (reason)
    {
    case ObfmSpacingOwnerReason::SelectorBranchNotReached:
        return "selector_branch_not_reached";
    case ObfmSpacingOwnerReason::FeatureDisabled:
        return "feature_disabled";
    case ObfmSpacingOwnerReason::FrameEvidenceUnavailable:
        return "frame_evidence_unavailable";
    case ObfmSpacingOwnerReason::SafetySampleUnavailable:
        return "safety_sample_unavailable";
    case ObfmSpacingOwnerReason::SafetyNotAdmitted:
        return "safety_not_admitted";
    case ObfmSpacingOwnerReason::EnergyAuthorityUnavailable:
        return "energy_authority_unavailable";
    case ObfmSpacingOwnerReason::EnergyAuthorityDoesNotAdmitDissipation:
        return "energy_authority_does_not_admit_dissipation";
    case ObfmSpacingOwnerReason::NeutralEnergyAuthorityUnavailable:
        return "neutral_energy_authority_unavailable";
    case ObfmSpacingOwnerReason::OfficialEmployAvailable:
        return "official_employ_available";
    case ObfmSpacingOwnerReason::OfficialEpochPrimed:
        return "official_epoch_primed";
    case ObfmSpacingOwnerReason::OfficialEpochChanged:
        return "official_epoch_changed";
    case ObfmSpacingOwnerReason::TimeNotIncreasing:
        return "time_not_increasing";
    case ObfmSpacingOwnerReason::TargetHorizontalCourseUndefined:
        return "target_horizontal_course_undefined";
    case ObfmSpacingOwnerReason::TargetCourseUndefined:
        return "target_course_undefined";
    case ObfmSpacingOwnerReason::OwnSpeedNotPositive:
        return "own_speed_not_positive";
    case ObfmSpacingOwnerReason::OfficialRangeInvalid:
        return "official_range_invalid";
    case ObfmSpacingOwnerReason::CoincidentGeometry:
        return "coincident_geometry";
    case ObfmSpacingOwnerReason::ArrestHorizontalReferenceUndefined:
        return "arrest_horizontal_reference_undefined";
    case ObfmSpacingOwnerReason::StationNotAhead:
        return "station_not_ahead";
    case ObfmSpacingOwnerReason::ClimbAllocationUnavailable:
        return "climb_allocation_unavailable";
    case ObfmSpacingOwnerReason::NoExcessProjectedClosure:
        return "no_excess_projected_closure";
    case ObfmSpacingOwnerReason::CoordinateClosureNotExcess:
        return "coordinate_closure_not_excess";
    case ObfmSpacingOwnerReason::DualClosureExcessLatched:
        return "dual_closure_excess_latched";
    case ObfmSpacingOwnerReason::LatchedActive:
        return "latched_active";
    case ObfmSpacingOwnerReason::CurrentEnergyProjectionRequired:
        return "current_energy_projection_required";
    case ObfmSpacingOwnerReason::CurrentEnergyProjectionUnavailable:
        return "current_energy_projection_unavailable";
    case ObfmSpacingOwnerReason::CurrentEnergyProjectionRejected:
        return "current_energy_projection_rejected";
    case ObfmSpacingOwnerReason::DecoratorNotReached:
        return "decorator_not_reached";
    case ObfmSpacingOwnerReason::DecoratorNotAdmitted:
        return "decorator_not_admitted";
    case ObfmSpacingOwnerReason::DecoratorSelected:
        return "decorator_selected";
    case ObfmSpacingOwnerReason::CommittedActive:
        return "committed_active";
    case ObfmSpacingOwnerReason::PostHitPending:
        return "post_hit_pending";
    case ObfmSpacingOwnerReason::CompletionPrimed:
        return "completion_primed";
    case ObfmSpacingOwnerReason::AwaitingClosureArrest:
        return "awaiting_closure_arrest";
    case ObfmSpacingOwnerReason::ClosureArrestLatched:
        return "closure_arrest_latched";
    case ObfmSpacingOwnerReason::AwaitingStationTurnaround:
        return "awaiting_station_turnaround";
    case ObfmSpacingOwnerReason::ArrestConfirmed:
        return "arrest_confirmed";
    case ObfmSpacingOwnerReason::StationPassedBeforeCompletion:
        return "station_passed_before_completion";
    case ObfmSpacingOwnerReason::RecoveryTracking:
        return "recovery_tracking";
    case ObfmSpacingOwnerReason::RecoveryActivated:
        return "recovery_activated";
    case ObfmSpacingOwnerReason::AwaitingVerticalTurnaround:
        return "awaiting_vertical_turnaround";
    case ObfmSpacingOwnerReason::VerticalTurnaroundLatched:
        return "vertical_turnaround_latched";
    case ObfmSpacingOwnerReason::AwaitingRecoveryEndpoint:
        return "awaiting_recovery_endpoint";
    case ObfmSpacingOwnerReason::RecoveryCompleted:
        return "recovery_completed";
    case ObfmSpacingOwnerReason::StationPassedDuringRecovery:
        return "station_passed_during_recovery";
    case ObfmSpacingOwnerReason::OfficialEpochChangedDuringRecovery:
        return "official_epoch_changed_during_recovery";
    case ObfmSpacingOwnerReason::WezRecoveryEvidenceUnavailable:
        return "wez_recovery_evidence_unavailable";
    case ObfmSpacingOwnerReason::WezEpochChanged:
        return "wez_epoch_changed";
    case ObfmSpacingOwnerReason::OfficialMinimumRangeReached:
        return "official_minimum_range_reached";
    case ObfmSpacingOwnerReason::OfficialMidrangeStationPassed:
        return "official_midrange_station_passed";
    case ObfmSpacingOwnerReason::ReacquireHorizontalCourseUndefined:
        return "reacquire_horizontal_course_undefined";
    case ObfmSpacingOwnerReason::ReacquireStationVelocityUndefined:
        return "reacquire_station_velocity_undefined";
    case ObfmSpacingOwnerReason::TargetPathExceedsAuthority:
        return "target_path_exceeds_authority";
    case ObfmSpacingOwnerReason::BoresightGeometryUnavailable:
        return "boresight_geometry_unavailable";
    case ObfmSpacingOwnerReason::BoresightTargetNotForward:
        return "boresight_target_not_forward";
    case ObfmSpacingOwnerReason::CommandGeometryUnavailable:
        return "command_geometry_unavailable";
    case ObfmSpacingOwnerReason::CommandFloat32DomainUnavailable:
        return "command_float32_domain_unavailable";
    case ObfmSpacingOwnerReason::CommandReady:
        return "command_ready";
    case ObfmSpacingOwnerReason::TaskCompleted:
        return "task_completed";
    case ObfmSpacingOwnerReason::ReleaseToLowerFallback:
        return "release_to_lower_fallback";
    case ObfmSpacingOwnerReason::TreePreempted:
        return "tree_preempted";
    case ObfmSpacingOwnerReason::OfficialEmployPreemption:
        return "official_employ_preemption";
    case ObfmSpacingOwnerReason::EpisodeOrTargetChanged:
        return "episode_or_target_changed";
    case ObfmSpacingOwnerReason::DeclaredReadyFrameIdentityInvalid:
        return "declared_ready_frame_identity_invalid";
    case ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite:
        return "declared_ready_frame_nonfinite";
    case ObfmSpacingOwnerReason::DeclaredReadyEnergyAuthorityContradiction:
        return "declared_ready_energy_authority_contradiction";
    case ObfmSpacingOwnerReason::DeclaredReadyProjectionContradiction:
        return "declared_ready_projection_contradiction";
    case ObfmSpacingOwnerReason::TaskLifecycleContradiction:
        return "task_lifecycle_contradiction";
    case ObfmSpacingOwnerReason::ServiceReceiptContradiction:
        return "service_receipt_contradiction";
    case ObfmSpacingOwnerReason::RecoveryVelocityBoundUnavailable:
        return "recovery_velocity_bound_unavailable";
    case ObfmSpacingOwnerReason::DeclaredReadySafetyReceiptContradiction:
        return "declared_ready_safety_receipt_contradiction";
    case ObfmSpacingOwnerReason::
        DeclaredReadyRecoveryVelocityBoundContradiction:
        return "declared_ready_recovery_velocity_bound_contradiction";
    case ObfmSpacingOwnerReason::DualClosureArrivalInfeasibleLatched:
        return "dual_closure_arrival_infeasible_latched";
    case ObfmSpacingOwnerReason::DualArrivalClosureNotPositive:
        return "dual_arrival_closure_not_positive";
    case ObfmSpacingOwnerReason::ArrivalArrestDecelerationUnavailable:
        return "arrival_arrest_deceleration_unavailable";
    case ObfmSpacingOwnerReason::ArrivalArrestDistanceAvailable:
        return "arrival_arrest_distance_available";
    case ObfmSpacingOwnerReason::ArrivalArrestArithmeticUnavailable:
        return "arrival_arrest_arithmetic_unavailable";
    default:
        return "unknown_spacing_reason";
    }
}

ObfmSpacingOwner::ObfmSpacingOwner() noexcept
{
    ResetEpisode();
}

void ObfmSpacingOwner::ResetEntry() noexcept
{
    entry_epoch_valid_ = false;
    entry_phase_id_ = WezPhaseId::P1;
    entry_official_range_m_ = 0.0;
    entry_previous_time_s_ = 0.0;
    entry_previous_spacing_m_ = 0.0;
    entry_latched_ = false;
    entry_last_frame_valid_ = false;
    entry_last_frame_identity_ = ControlFrameIdentity{};
}

void ObfmSpacingOwner::ResetCompletion() noexcept
{
    completion_phase_ = ObfmSpacingCompletionPhase::Unprimed;
    completion_epoch_valid_ = false;
    completion_phase_id_ = WezPhaseId::P1;
    completion_official_range_m_ = 0.0;
    completion_previous_time_s_ = 0.0;
    completion_previous_closure_mps_ = 0.0;
    completion_previous_spacing_m_ = 0.0;
}

void ObfmSpacingOwner::ResetRecovery() noexcept
{
    recovery_phase_ = ObfmSpacingRecoveryPhase::Unprimed;
    recovery_epoch_valid_ = false;
    recovery_phase_id_ = WezPhaseId::P1;
    recovery_official_range_m_ = 0.0;
    recovery_previous_time_s_ = 0.0;
    recovery_previous_down_velocity_mps_ = 0.0;
    recovery_previous_altitude_m_ = 0.0;
    recovery_previous_spacing_m_ = 0.0;
    recovery_climb_history_observed_ = false;
    recovery_post_turnaround_spacing_increase_observed_ = false;
}

void ObfmSpacingOwner::ClearLifecycle(const bool clear_post_hit) noexcept
{
    ResetEntry();
    ResetCompletion();
    ResetRecovery();
    task_active_ = false;
    employ_preemption_pending_ = false;
    current_projection_required_ = false;
    entry_projection_valid_ = false;
    entry_projection_admitted_ = false;
    entry_projection_identity_ = ControlFrameIdentity{};
    entry_projection_raw_bias_m2ps3_ = 0.0;
    entry_projection_admitted_bias_m2ps3_ = 0.0;
    task_entry_frame_identity_ = ControlFrameIdentity{};
    phase_ = ObfmSpacingOwnerPhase::Inactive;
    frozen_wez_epoch_valid_ = false;
    frozen_wez_phase_id_ = WezPhaseId::P1;
    frozen_wez_official_range_m_ = 0.0;
    owner_identity_valid_ = false;
    owner_episode_epoch_ = 0U;
    owner_own_plane_id_ = -1;
    owner_target_plane_id_ = -1;
    if (clear_post_hit)
    {
        post_hit_pending_ = false;
    }
}

void ObfmSpacingOwner::ResetEpisode() noexcept
{
    ClearLifecycle(true);
}

void ObfmSpacingOwner::ObserveService(
    const DogfightGeometryFrame& frame,
    const ObfmSpacingOwnerServiceInput& input,
    ObfmSpacingOwnerServiceReceipt& output,
    Status& status) noexcept
{
    output = ObfmSpacingOwnerServiceReceipt{};
    status = Status{};
    if (!input.selector_branch_reached)
    {
        output.reason = ObfmSpacingOwnerReason::SelectorBranchNotReached;
        return;
    }
    output.service_evaluated = true;
    output.feature_enabled = input.feature_enabled;
    if (!input.feature_enabled)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::FeatureDisabled;
        if (!task_active_)
        {
            ResetEntry();
            ResetCompletion();
            ResetRecovery();
            post_hit_pending_ = false;
        }
        return;
    }
    if (!input.frame_evidence_declared_ready)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::FrameEvidenceUnavailable;
        if (!task_active_ && !post_hit_pending_)
        {
            ResetEntry();
            ResetCompletion();
            ResetRecovery();
        }
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        output.reason =
            ObfmSpacingOwnerReason::DeclaredReadyFrameIdentityInvalid;
        SetContractStatus(output.reason, status);
        return;
    }
    output.frame_identity = frame.frame_identity;
    if (!FiniteFrameKinematics(frame))
    {
        output.reason = ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite;
        SetContractStatus(output.reason, status);
        return;
    }
    if (!frame.target_same_index)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::FrameEvidenceUnavailable;
        return;
    }
    if (frame.own_offense.damage_rate < 0.0)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::FrameEvidenceUnavailable;
        return;
    }

    if (task_active_
        && (!owner_identity_valid_
            || !SameOwnerIdentity(
                frame,
                owner_episode_epoch_,
                owner_own_plane_id_,
                owner_target_plane_id_)))
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::EpisodeOrTargetChanged;
        return;
    }

    // Exact da8 active-owner precedence: an official hit transfers this
    // frame to EMPLOY before the running fault-only safety surface is read.
    // HaltTask(true) then preserves the single post-hit token for the next
    // strict-entry Rmin-arrest frame.  A coincident finite safety rejection
    // must not erase that causal transfer.
    if (task_active_
        && frame.own_offense.damage_rate > 0.0
        && phase_ != ObfmSpacingOwnerPhase::PostHitRminArrest)
    {
        employ_preemption_pending_ = true;
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::OfficialEmployAvailable;
        return;
    }

    if (!ResolveSafetyContract(
            task_active_,
            post_hit_pending_,
            phase_,
            output.safety_phase,
            output.safety_grade_required))
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::TaskLifecycleContradiction;
        SetContractStatus(output.reason, status);
        return;
    }
    output.safety = input.safety;
    ObfmSpacingOwnerReason safety_reason =
        ObfmSpacingOwnerReason::SafetySampleUnavailable;
    const CalculationDomain safety_domain = ValidateSafetyReceipt(
        frame.frame_identity,
        input.safety,
        output.safety_grade_required,
        output.safety_grade_evaluated,
        output.safety_grade_admitted,
        safety_reason);
    if (safety_domain != CalculationDomain::Available)
    {
        output.selection_finalized = true;
        output.reason = safety_reason;
        if (!task_active_ && !post_hit_pending_)
        {
            ResetEntry();
            ResetCompletion();
            ResetRecovery();
        }
        if (safety_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            SetContractStatus(output.reason, status);
        }
        return;
    }

    if (task_active_)
    {
        output.selection_finalized = true;
        output.selected_result = true;
        output.selected_count = 1U;
        output.entry_latched = entry_latched_;
        output.reason = ObfmSpacingOwnerReason::CommittedActive;
        return;
    }
    if (post_hit_pending_)
    {
        output.selection_finalized = true;
        output.selected_result = true;
        output.selected_count = 1U;
        output.reason = ObfmSpacingOwnerReason::PostHitPending;
        return;
    }
    ObfmSpacingOwnerReason geometry_reason =
        ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    const CalculationDomain geometry_domain = BuildAxisGeometry(
        frame,
        frame.own_offense.phase.max_range_m,
        output.geometry,
        geometry_reason);
    if (geometry_domain != CalculationDomain::Available)
    {
        output.selection_finalized = true;
        output.reason = geometry_reason;
        ResetEntry();
        ResetCompletion();
        ResetRecovery();
        if (geometry_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            SetContractStatus(output.reason, status);
        }
        return;
    }

    const WezPhaseId phase_id = frame.own_offense.phase.id;
    const double official_range_m = output.geometry.official_range_m;
    const bool epoch_changed = entry_epoch_valid_
        && (entry_phase_id_ != phase_id
            || entry_official_range_m_ != official_range_m);
    if (!entry_epoch_valid_ || epoch_changed)
    {
        entry_epoch_valid_ = true;
        entry_phase_id_ = phase_id;
        entry_official_range_m_ = official_range_m;
        entry_previous_time_s_ = frame.t_sec;
        entry_previous_spacing_m_ =
            output.geometry.signed_station_spacing_m;
        entry_latched_ = false;
        entry_last_frame_valid_ = true;
        entry_last_frame_identity_ = frame.frame_identity;
        output.entry_latched = false;
        output.selection_finalized = true;
        output.reason = epoch_changed
            ? ObfmSpacingOwnerReason::OfficialEpochChanged
            : ObfmSpacingOwnerReason::OfficialEpochPrimed;
        return;
    }
    double dt_s = 0.0;
    if (!SubtractFinite(frame.t_sec, entry_previous_time_s_, dt_s)
        || dt_s <= 0.0)
    {
        ResetEntry();
        ResetCompletion();
        ResetRecovery();
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::TimeNotIncreasing;
        return;
    }
    double spacing_delta = 0.0;
    double coordinate_closure = 0.0;
    if (!SubtractFinite(
            output.geometry.signed_station_spacing_m,
            entry_previous_spacing_m_,
            spacing_delta)
        || !DivideFinite(-spacing_delta, dt_s, coordinate_closure))
    {
        ResetEntry();
        ResetCompletion();
        ResetRecovery();
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
        return;
    }
    entry_previous_time_s_ = frame.t_sec;
    entry_previous_spacing_m_ =
        output.geometry.signed_station_spacing_m;
    entry_last_frame_valid_ = true;
    entry_last_frame_identity_ = frame.frame_identity;
    output.coordinate_closure_available = true;
    output.coordinate_closure_mps = coordinate_closure;

    if (frame.own_offense.damage_rate > 0.0)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::OfficialEmployAvailable;
        return;
    }
    if (output.geometry.signed_station_spacing_m <= 0.0)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::StationNotAhead;
        return;
    }
    if (output.geometry.arrest_horizontal_speed_mps
        >= output.geometry.own_speed_mps)
    {
        output.selection_finalized = true;
        output.reason = ObfmSpacingOwnerReason::ClimbAllocationUnavailable;
        return;
    }
    if (entry_latched_)
    {
        output.reason = ObfmSpacingOwnerReason::LatchedActive;
        output.entry_latch_reason = output.reason;
    }
    else
    {
        const bool projected_closure_excess =
            output.geometry.projected_closure_mps
                > output.geometry.structural_station_closure_mps;
        // These are not duplicate proofs for a turning target.  The
        // coordinate derivative also contains the rotating target-axis term
        // that is absent from the instantaneous velocity projection.
        const bool coordinate_closure_excess = coordinate_closure
            > output.geometry.structural_station_closure_mps;
        if (projected_closure_excess && coordinate_closure_excess)
        {
            entry_latched_ = true;
            output.reason = ObfmSpacingOwnerReason::DualClosureExcessLatched;
            output.entry_latch_reason = output.reason;
        }
        else
        {
            const ObfmSpacingOwnerReason original_reason =
                projected_closure_excess
                ? ObfmSpacingOwnerReason::CoordinateClosureNotExcess
                : ObfmSpacingOwnerReason::NoExcessProjectedClosure;
            if (input.flight_path_gamma_limit_available)
            {
                ObfmSpacingOwnerReason arrival_reason =
                    ObfmSpacingOwnerReason::ArrivalArrestDistanceAvailable;
                const CalculationDomain arrival_domain =
                    EvaluateArrivalFeasibility(
                        output.geometry,
                        coordinate_closure,
                        input.flight_path_gamma_limit_available,
                        input.flight_path_gamma_limit_rad,
                        output.arrival_feasibility,
                        arrival_reason);
                if (arrival_domain
                    == CalculationDomain::DeclaredReadyContradiction)
                {
                    output.selection_finalized = true;
                    output.reason = arrival_reason;
                    SetContractStatus(output.reason, status);
                    return;
                }
                if (arrival_domain == CalculationDomain::Available
                    && output.arrival_feasibility.admitted)
                {
                    entry_latched_ = true;
                    output.reason = ObfmSpacingOwnerReason::
                        DualClosureArrivalInfeasibleLatched;
                    output.entry_latch_reason = output.reason;
                }
            }
            if (!entry_latched_)
            {
                output.selection_finalized = true;
                output.reason = original_reason;
                return;
            }
        }
    }
    output.entry_latched = true;

    ObfmSpacingOwnerReason command_reason =
        ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    const CalculationDomain command_domain = BuildAxisCommand(
        frame,
        output.geometry,
        ObfmSpacingOwnerPhase::PathEnergyExchange,
        input.flight_path_gamma_limit_available,
        input.flight_path_gamma_limit_rad,
        input.previous_energy_authority,
        output.preprojected_candidate,
        command_reason);
    if (command_domain != CalculationDomain::Available)
    {
        output.selection_finalized = true;
        output.reason = command_reason;
        if (command_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            SetContractStatus(output.reason, status);
        }
        return;
    }
    output.preprojected_candidate_valid = true;
    output.projection_required = input.current_energy_projection_required;
    if (input.current_energy_projection_required)
    {
        output.selection_finalized = false;
        output.reason =
            ObfmSpacingOwnerReason::CurrentEnergyProjectionRequired;
        return;
    }
    output.selection_finalized = true;
    output.selected_result = true;
    output.selected_count = 1U;
}

void ObfmSpacingOwner::FinalizeServiceProjection(
    const ObfmSpacingOwnerServiceReceipt& preliminary,
    const ObfmSpacingCurrentEnergyProjection& projection,
    ObfmSpacingOwnerServiceReceipt& output,
    Status& status) const noexcept
{
    output = preliminary;
    status = Status{};
    const bool latch_reason_valid = preliminary.entry_latch_reason
            == ObfmSpacingOwnerReason::DualClosureExcessLatched
        || preliminary.entry_latch_reason
            == ObfmSpacingOwnerReason::DualClosureArrivalInfeasibleLatched
        || preliminary.entry_latch_reason
            == ObfmSpacingOwnerReason::LatchedActive;
    if (!preliminary.service_evaluated
        || !preliminary.projection_required
        || preliminary.selection_finalized
        || !preliminary.entry_latched
        || !latch_reason_valid
        || !preliminary.preprojected_candidate_valid
        || preliminary.safety_phase
            != ObfmSpacingOwnerPhase::PathEnergyExchange
        || preliminary.safety_grade_required
            != ObfmSpacingSafetyGrade::StrictEntry
        || !SelectedSafetyReceiptCoherent(preliminary)
        || preliminary.reason
            != ObfmSpacingOwnerReason::CurrentEnergyProjectionRequired
        || !IsValidControlFrameIdentity(preliminary.frame_identity))
    {
        output.selected_result = false;
        output.selected_count = 0U;
        output.reason = ObfmSpacingOwnerReason::ServiceReceiptContradiction;
        SetContractStatus(output.reason, status);
        return;
    }
    ObfmSpacingOwnerReason projection_reason =
        ObfmSpacingOwnerReason::CurrentEnergyProjectionUnavailable;
    const double raw_bias =
        preliminary.preprojected_candidate.specific_energy_rate_bias_m2ps3;
    const CalculationDomain projection_domain = ApplyCurrentProjection(
        preliminary.frame_identity,
        projection,
        output.preprojected_candidate,
        projection_reason);
    output.selection_finalized = true;
    output.reason = projection_reason;
    if (projection_domain != CalculationDomain::Available)
    {
        output.selected_result = false;
        output.selected_count = 0U;
        if (projection_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            SetContractStatus(output.reason, status);
        }
        return;
    }
    output.current_projection_valid = true;
    output.current_projection_admitted = projection_reason
        == ObfmSpacingOwnerReason::CommandReady;
    output.current_projection_raw_bias_m2ps3 = raw_bias;
    output.current_projection_admitted_bias_m2ps3 =
        output.preprojected_candidate.specific_energy_rate_bias_m2ps3;
    output.selected_result = true;
    output.selected_count = 1U;
    output.reason = preliminary.entry_latch_reason;
}

void ObfmSpacingOwner::EvaluateDecorator(
    const bool branch_reached,
    const ObfmSpacingOwnerServiceReceipt& service,
    ObfmSpacingOwnerSelection& output,
    Status& status) const noexcept
{
    output = ObfmSpacingOwnerSelection{};
    status = Status{};
    output.branch_reached = branch_reached;
    if (!branch_reached)
    {
        output.reason = ObfmSpacingOwnerReason::DecoratorNotReached;
        return;
    }
    if (!service.service_evaluated || !service.selection_finalized)
    {
        output.reason = ObfmSpacingOwnerReason::ServiceReceiptContradiction;
        SetContractStatus(output.reason, status);
        return;
    }
    output.frame_identity = service.frame_identity;
    if (!service.selected_result)
    {
        output.reason = ObfmSpacingOwnerReason::DecoratorNotAdmitted;
        return;
    }
    ObfmSpacingOwnerPhase expected_safety_phase =
        ObfmSpacingOwnerPhase::Inactive;
    ObfmSpacingSafetyGrade expected_safety_grade =
        ObfmSpacingSafetyGrade::None;
    if (!IsValidControlFrameIdentity(service.frame_identity)
        || service.selected_count != 1U
        || !ResolveSafetyContract(
            task_active_,
            post_hit_pending_,
            phase_,
            expected_safety_phase,
            expected_safety_grade)
        || service.safety_phase != expected_safety_phase
        || service.safety_grade_required != expected_safety_grade
        || !SelectedSafetyReceiptCoherent(service))
    {
        output.reason = ObfmSpacingOwnerReason::ServiceReceiptContradiction;
        SetContractStatus(output.reason, status);
        return;
    }
    output.selected = true;
    output.selection_count = 1U;
    output.reason = ObfmSpacingOwnerReason::DecoratorSelected;
}

void ObfmSpacingOwner::EnterTask(
    const ObfmSpacingOwnerServiceReceipt& service,
    const ObfmSpacingOwnerSelection& selection,
    Status& status) noexcept
{
    status = Status{};
    ObfmSpacingOwnerPhase expected_safety_phase =
        ObfmSpacingOwnerPhase::Inactive;
    ObfmSpacingSafetyGrade expected_safety_grade =
        ObfmSpacingSafetyGrade::None;
    if (task_active_
        || !selection.selected
        || selection.selection_count != 1U
        || !service.selected_result
        || service.selected_count != 1U
        || !SameControlFrameIdentity(
            service.frame_identity,
            selection.frame_identity)
        || !ResolveSafetyContract(
            false,
            post_hit_pending_,
            phase_,
            expected_safety_phase,
            expected_safety_grade)
        || service.safety_phase != expected_safety_phase
        || service.safety_grade_required != expected_safety_grade
        || !SelectedSafetyReceiptCoherent(service))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const bool post_hit_entry = post_hit_pending_;
    if (!post_hit_entry && !entry_latched_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    task_active_ = true;
    phase_ = post_hit_entry
        ? ObfmSpacingOwnerPhase::PostHitRminArrest
        : ObfmSpacingOwnerPhase::PathEnergyExchange;
    post_hit_pending_ = false;
    employ_preemption_pending_ = false;
    current_projection_required_ = service.projection_required;
    entry_projection_valid_ = service.current_projection_valid;
    entry_projection_admitted_ = service.current_projection_admitted;
    entry_projection_identity_ = service.frame_identity;
    entry_projection_raw_bias_m2ps3_ =
        service.current_projection_raw_bias_m2ps3;
    entry_projection_admitted_bias_m2ps3_ =
        service.current_projection_admitted_bias_m2ps3;
    task_entry_frame_identity_ = service.frame_identity;
    ResetCompletion();
    ResetRecovery();
    frozen_wez_epoch_valid_ = false;
    owner_identity_valid_ = true;
    owner_episode_epoch_ = service.frame_identity.episode_epoch;
    // Plane identities are established by the first Task tick, because the
    // immutable Service receipt intentionally carries only causal frame ID.
    owner_own_plane_id_ = -1;
    owner_target_plane_id_ = -1;
}

void ObfmSpacingOwner::TickTask(
    const DogfightGeometryFrame& frame,
    const ObfmSpacingOwnerServiceReceipt& service,
    const ObfmSpacingOwnerTaskInput& input,
    ObfmSpacingOwnerTaskReceipt& output,
    Status& status) noexcept
{
    output = ObfmSpacingOwnerTaskReceipt{};
    status = Status{};
    output.frame_identity = frame.frame_identity;
    output.task_active = task_active_;
    output.phase = phase_;
    output.completion_phase = completion_phase_;
    output.recovery_phase = recovery_phase_;

    SpacingReleaseAction release{output};
    SpacingContractAction contract{output, status};

    if (!task_active_
        || phase_ == ObfmSpacingOwnerPhase::Inactive
        || !service.service_evaluated
        || !service.selection_finalized
        || !service.selected_result
        || service.selected_count != 1U
        || !SameControlFrameIdentity(
            frame.frame_identity,
            service.frame_identity))
    {
        contract(ObfmSpacingOwnerReason::TaskLifecycleContradiction);
        return;
    }
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        contract(
            ObfmSpacingOwnerReason::DeclaredReadyFrameIdentityInvalid);
        return;
    }
    if (!FiniteFrameKinematics(frame))
    {
        contract(ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite);
        return;
    }
    if (!frame.target_same_index || frame.own_offense.damage_rate < 0.0)
    {
        release(ObfmSpacingOwnerReason::FrameEvidenceUnavailable);
        return;
    }
    if (owner_own_plane_id_ < 0 && owner_target_plane_id_ < 0)
    {
        owner_own_plane_id_ = frame.own_plane_id;
        owner_target_plane_id_ = frame.target_plane_id;
    }
    if (!owner_identity_valid_
        || !SameOwnerIdentity(
            frame,
            owner_episode_epoch_,
            owner_own_plane_id_,
            owner_target_plane_id_))
    {
        release(ObfmSpacingOwnerReason::EpisodeOrTargetChanged);
        return;
    }
    const bool entry_sample = SameControlFrameIdentity(
        frame.frame_identity,
        task_entry_frame_identity_);
    const ObfmSpacingOwnerPhase expected_safety_phase = phase_;
    ObfmSpacingSafetyGrade expected_safety_grade =
        ObfmSpacingSafetyGrade::None;
    if (phase_ == ObfmSpacingOwnerPhase::PostHitRminArrest
        || (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
            && entry_sample))
    {
        expected_safety_grade = ObfmSpacingSafetyGrade::StrictEntry;
    }
    else if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase_ == ObfmSpacingOwnerPhase::LevelRecovery
        || phase_ == ObfmSpacingOwnerPhase::WezReacquire)
    {
        expected_safety_grade = ObfmSpacingSafetyGrade::RunningFaultOnly;
    }
    if (expected_safety_grade == ObfmSpacingSafetyGrade::None
        || service.safety_phase != expected_safety_phase
        || service.safety_grade_required != expected_safety_grade
        || !service.safety_grade_evaluated
        || !service.safety_grade_admitted)
    {
        contract(ObfmSpacingOwnerReason::ServiceReceiptContradiction);
        return;
    }
    bool safety_evaluated = false;
    bool safety_admitted = false;
    ObfmSpacingOwnerReason safety_reason =
        ObfmSpacingOwnerReason::SafetySampleUnavailable;
    const CalculationDomain safety_domain = ValidateSafetyReceipt(
        frame.frame_identity,
        service.safety,
        expected_safety_grade,
        safety_evaluated,
        safety_admitted,
        safety_reason);
    if (safety_domain != CalculationDomain::Available
        || !safety_evaluated
        || !safety_admitted)
    {
        contract(
            safety_domain == CalculationDomain::DeclaredReadyContradiction
                ? safety_reason
                : ObfmSpacingOwnerReason::ServiceReceiptContradiction);
        return;
    }
    output.safety_phase = expected_safety_phase;
    output.safety_grade_consumed = expected_safety_grade;
    output.safety_frame_identity = service.safety.frame_identity;
    output.safety_admitted = true;
    if (frame.own_offense.damage_rate > 0.0
        && phase_ != ObfmSpacingOwnerPhase::PostHitRminArrest)
    {
        employ_preemption_pending_ = true;
        release(ObfmSpacingOwnerReason::OfficialEmployAvailable);
        return;
    }

    double recovery_velocity_bound_mps = 0.0;
    if (phase_ == ObfmSpacingOwnerPhase::LevelRecovery
        && input.recovery_velocity_bound.evaluated)
    {
        output.recovery_velocity_bound = input.recovery_velocity_bound;
        ObfmSpacingOwnerReason bound_reason = ObfmSpacingOwnerReason::
            RecoveryVelocityBoundUnavailable;
        const CalculationDomain bound_domain =
            ValidateRecoveryVelocityBound(
                frame.frame_identity,
                input.recovery_velocity_bound,
                recovery_velocity_bound_mps,
                bound_reason);
        if (bound_domain == CalculationDomain::Available)
        {
            output.recovery_velocity_bound_consumed = true;
        }
        else
        {
            // This receipt only widens the measured v_D zero crossing.  A
            // missing or incoherent optional tolerance must not erase the
            // already finite LEVEL_RECOVERY command; exact measured sign is
            // the total fallback.
            recovery_velocity_bound_mps = 0.0;
        }
    }

    ObfmSpacingGeometry axis_geometry{};
    ObfmSpacingReacquireGeometry reacquire_geometry{};
    ObfmSpacingOwnerReason geometry_reason =
        ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    CalculationDomain geometry_domain = CalculationDomain::FiniteUnavailable;
    if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase_ == ObfmSpacingOwnerPhase::LevelRecovery)
    {
        geometry_domain = BuildAxisGeometry(
            frame,
            frame.own_offense.phase.max_range_m,
            axis_geometry,
            geometry_reason);
    }
    else if (phase_ == ObfmSpacingOwnerPhase::PostHitRminArrest)
    {
        geometry_domain = BuildAxisGeometry(
            frame,
            frame.own_offense.phase.min_range_m,
            axis_geometry,
            geometry_reason);
    }
    else if (phase_ == ObfmSpacingOwnerPhase::WezReacquire)
    {
        geometry_domain = BuildReacquireGeometry(
            frame,
            reacquire_geometry,
            geometry_reason);
    }
    else
    {
        contract(ObfmSpacingOwnerReason::TaskLifecycleContradiction);
        return;
    }
    if (geometry_domain != CalculationDomain::Available)
    {
        if (geometry_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            contract(geometry_reason);
        }
        else
        {
            release(geometry_reason);
        }
        return;
    }

    const WezPhaseId official_phase_id = frame.own_offense.phase.id;
    const bool axis_phase =
        phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase_ == ObfmSpacingOwnerPhase::LevelRecovery;

    // Exact active-entry observation order.  The admitting Service already
    // consumed the first selected frame, so the Task never differentiates it
    // twice.  Normal epoch/time/geometry loss releases to the next sibling.
    if (axis_phase
        && (!entry_last_frame_valid_
            || !SameControlFrameIdentity(
                entry_last_frame_identity_,
                frame.frame_identity)))
    {
        const bool epoch_changed = !entry_epoch_valid_
            || entry_phase_id_ != official_phase_id
            || entry_official_range_m_ != axis_geometry.official_range_m;
        if (epoch_changed)
        {
            release(ObfmSpacingOwnerReason::OfficialEpochChanged);
            return;
        }
        double dt_s = 0.0;
        if (!SubtractFinite(frame.t_sec, entry_previous_time_s_, dt_s)
            || dt_s <= 0.0)
        {
            ResetEntry();
            ResetCompletion();
            ResetRecovery();
            release(ObfmSpacingOwnerReason::TimeNotIncreasing);
            return;
        }
        double spacing_delta = 0.0;
        double coordinate_closure = 0.0;
        if (!SubtractFinite(
                axis_geometry.signed_station_spacing_m,
                entry_previous_spacing_m_,
                spacing_delta)
            || !DivideFinite(-spacing_delta, dt_s, coordinate_closure))
        {
            release(ObfmSpacingOwnerReason::CommandGeometryUnavailable);
            return;
        }
        entry_previous_time_s_ = frame.t_sec;
        entry_previous_spacing_m_ =
            axis_geometry.signed_station_spacing_m;
        entry_last_frame_valid_ = true;
        entry_last_frame_identity_ = frame.frame_identity;
        if (axis_geometry.signed_station_spacing_m <= 0.0)
        {
            release(ObfmSpacingOwnerReason::StationNotAhead);
            return;
        }
        if (axis_geometry.arrest_horizontal_speed_mps
                >= axis_geometry.own_speed_mps
            && completion_phase_
                != ObfmSpacingCompletionPhase::Completed)
        {
            release(ObfmSpacingOwnerReason::ClimbAllocationUnavailable);
            return;
        }
        if (!entry_latched_)
        {
            release(ObfmSpacingOwnerReason::ReleaseToLowerFallback);
            return;
        }
    }

    // Ordered closure-crossing then later station-increase evidence.
    if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase_ == ObfmSpacingOwnerPhase::PostHitRminArrest)
    {
        const bool epoch_changed = completion_epoch_valid_
            && (completion_phase_id_ != official_phase_id
                || completion_official_range_m_
                    != axis_geometry.official_range_m);
        if (!completion_epoch_valid_ || epoch_changed)
        {
            completion_epoch_valid_ = true;
            completion_phase_id_ = official_phase_id;
            completion_official_range_m_ =
                axis_geometry.official_range_m;
            completion_previous_time_s_ = frame.t_sec;
            completion_previous_closure_mps_ =
                axis_geometry.projected_closure_mps;
            completion_previous_spacing_m_ =
                axis_geometry.signed_station_spacing_m;
            completion_phase_ = ObfmSpacingCompletionPhase::Primed;
            output.reason = ObfmSpacingOwnerReason::CompletionPrimed;
        }
        else
        {
            if (frame.t_sec <= completion_previous_time_s_)
            {
                ResetCompletion();
                release(ObfmSpacingOwnerReason::TimeNotIncreasing);
                return;
            }
            const double previous_closure =
                completion_previous_closure_mps_;
            const double previous_spacing =
                completion_previous_spacing_m_;
            const bool crossing = previous_closure > 0.0
                && axis_geometry.projected_closure_mps <= 0.0;
            const bool spacing_increased =
                axis_geometry.signed_station_spacing_m > previous_spacing;
            completion_previous_time_s_ = frame.t_sec;
            completion_previous_closure_mps_ =
                axis_geometry.projected_closure_mps;
            completion_previous_spacing_m_ =
                axis_geometry.signed_station_spacing_m;

            if (completion_phase_ == ObfmSpacingCompletionPhase::Failed)
            {
                release(
                    ObfmSpacingOwnerReason::StationPassedBeforeCompletion);
                return;
            }
            if (axis_geometry.signed_station_spacing_m <= 0.0
                && completion_phase_
                    != ObfmSpacingCompletionPhase::Completed)
            {
                completion_phase_ = ObfmSpacingCompletionPhase::Failed;
                release(
                    ObfmSpacingOwnerReason::StationPassedBeforeCompletion);
                return;
            }
            if (completion_phase_ == ObfmSpacingCompletionPhase::Primed)
            {
                if (crossing)
                {
                    completion_phase_ = ObfmSpacingCompletionPhase::
                        ClosureArrestLatched;
                    output.reason =
                        ObfmSpacingOwnerReason::ClosureArrestLatched;
                }
                else
                {
                    output.reason =
                        ObfmSpacingOwnerReason::AwaitingClosureArrest;
                }
            }
            else if (completion_phase_
                == ObfmSpacingCompletionPhase::ClosureArrestLatched)
            {
                if (spacing_increased)
                {
                    completion_phase_ =
                        ObfmSpacingCompletionPhase::Completed;
                    output.arrest_confirmed_this_tick = true;
                    output.reason = ObfmSpacingOwnerReason::ArrestConfirmed;
                }
                else
                {
                    output.reason =
                        ObfmSpacingOwnerReason::AwaitingStationTurnaround;
                }
            }
        }

        if (phase_ == ObfmSpacingOwnerPhase::PostHitRminArrest
            && completion_phase_ == ObfmSpacingCompletionPhase::Completed)
        {
            output.phase = phase_;
            output.completion_phase = completion_phase_;
            output.recovery_phase = recovery_phase_;
            output.task_completed = true;
            output.reason = ObfmSpacingOwnerReason::TaskCompleted;
            return;
        }
    }

    if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        && completion_phase_ == ObfmSpacingCompletionPhase::Completed
        && axis_geometry.arrest_horizontal_speed_mps
            >= axis_geometry.own_speed_mps)
    {
        phase_ = ObfmSpacingOwnerPhase::LevelRecovery;
        output.phase_changed = true;
    }

    // Ordered measured climb recovery.  A turnaround sample cannot complete;
    // completion needs a later horizontal endpoint/closure/spacing sample.
    if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        || phase_ == ObfmSpacingOwnerPhase::LevelRecovery)
    {
        const bool recovery_requested =
            phase_ == ObfmSpacingOwnerPhase::LevelRecovery;
        const double down_velocity = frame.own.velocity_ned_mps[2];
        const double altitude_m = -frame.own.position_ned_m[2];
        const bool endpoint_available =
            axis_geometry.arrest_horizontal_speed_mps
            >= axis_geometry.own_speed_mps;
        if (!std::isfinite(down_velocity) || !std::isfinite(altitude_m))
        {
            contract(ObfmSpacingOwnerReason::DeclaredReadyFrameNonfinite);
            return;
        }
        if (!recovery_epoch_valid_)
        {
            recovery_epoch_valid_ = true;
            recovery_phase_id_ = official_phase_id;
            recovery_official_range_m_ = axis_geometry.official_range_m;
            recovery_previous_time_s_ = frame.t_sec;
            recovery_previous_down_velocity_mps_ = down_velocity;
            recovery_previous_altitude_m_ = altitude_m;
            recovery_previous_spacing_m_ =
                axis_geometry.signed_station_spacing_m;
            recovery_climb_history_observed_ = down_velocity < 0.0;
            recovery_phase_ = recovery_requested
                ? ObfmSpacingRecoveryPhase::LevelRecovery
                : ObfmSpacingRecoveryPhase::Tracking;
            output.reason = recovery_requested
                ? ObfmSpacingOwnerReason::RecoveryActivated
                : ObfmSpacingOwnerReason::RecoveryTracking;
        }
        else
        {
            if (frame.t_sec <= recovery_previous_time_s_)
            {
                ResetRecovery();
                release(ObfmSpacingOwnerReason::TimeNotIncreasing);
                return;
            }
            const bool recovery_active_before =
                recovery_phase_ == ObfmSpacingRecoveryPhase::LevelRecovery
                || recovery_phase_ == ObfmSpacingRecoveryPhase::
                    VerticalTurnaroundLatched
                || recovery_phase_ == ObfmSpacingRecoveryPhase::Completed
                || recovery_phase_ == ObfmSpacingRecoveryPhase::Failed;
            const bool activated_this_sample = recovery_requested
                && !recovery_active_before;
            const bool epoch_changed =
                recovery_phase_id_ != official_phase_id
                || recovery_official_range_m_
                    != axis_geometry.official_range_m;
            const double previous_down =
                recovery_previous_down_velocity_mps_;
            const double previous_spacing =
                recovery_previous_spacing_m_;
            const bool spacing_increased =
                axis_geometry.signed_station_spacing_m > previous_spacing;

            if (epoch_changed)
            {
                if (recovery_active_before || recovery_requested)
                {
                    recovery_phase_ = ObfmSpacingRecoveryPhase::Failed;
                    release(ObfmSpacingOwnerReason::
                        OfficialEpochChangedDuringRecovery);
                    return;
                }
                recovery_phase_id_ = official_phase_id;
                recovery_official_range_m_ =
                    axis_geometry.official_range_m;
                recovery_previous_time_s_ = frame.t_sec;
                recovery_previous_down_velocity_mps_ = down_velocity;
                recovery_previous_altitude_m_ = altitude_m;
                recovery_previous_spacing_m_ =
                    axis_geometry.signed_station_spacing_m;
                recovery_climb_history_observed_ = down_velocity < 0.0;
                recovery_phase_ = ObfmSpacingRecoveryPhase::Tracking;
            }
            else if (recovery_phase_ == ObfmSpacingRecoveryPhase::Failed)
            {
                release(ObfmSpacingOwnerReason::StationPassedDuringRecovery);
                return;
            }
            else if (recovery_phase_ != ObfmSpacingRecoveryPhase::Completed)
            {
                if (activated_this_sample)
                {
                    recovery_phase_ = ObfmSpacingRecoveryPhase::LevelRecovery;
                    output.reason = ObfmSpacingOwnerReason::RecoveryActivated;
                }
                const bool recovery_active_now =
                    recovery_phase_ == ObfmSpacingRecoveryPhase::LevelRecovery
                    || recovery_phase_ == ObfmSpacingRecoveryPhase::
                        VerticalTurnaroundLatched;
                if (!recovery_active_now)
                {
                    recovery_climb_history_observed_ =
                        recovery_climb_history_observed_
                        || down_velocity < 0.0;
                    output.reason = ObfmSpacingOwnerReason::RecoveryTracking;
                }
                else
                {
                    if (axis_geometry.signed_station_spacing_m <= 0.0)
                    {
                        recovery_phase_ = ObfmSpacingRecoveryPhase::Failed;
                        release(ObfmSpacingOwnerReason::
                            StationPassedDuringRecovery);
                        return;
                    }
                    // Equivalent to v_D + validated_bound >= 0 without an
                    // intermediate addition that could overflow.
                    const bool nonclimbing = down_velocity
                        >= -recovery_velocity_bound_mps;
                    const bool crossing = previous_down < 0.0
                        && nonclimbing;
                    const bool active_entry_nonclimb =
                        activated_this_sample
                        && recovery_climb_history_observed_
                        && nonclimbing
                        && !crossing;
                    recovery_climb_history_observed_ =
                        recovery_climb_history_observed_
                        || down_velocity < 0.0;
                    if (recovery_phase_
                            == ObfmSpacingRecoveryPhase::LevelRecovery
                        && (crossing || active_entry_nonclimb))
                    {
                        recovery_phase_ = ObfmSpacingRecoveryPhase::
                            VerticalTurnaroundLatched;
                        // Preserve the measured separation effect at the
                        // turnaround.  Requiring it to occur again on the
                        // following endpoint sample makes completion depend
                        // on sample coincidence rather than aircraft motion.
                        recovery_post_turnaround_spacing_increase_observed_ =
                            spacing_increased;
                        output.reason = ObfmSpacingOwnerReason::
                            VerticalTurnaroundLatched;
                    }
                    else if (recovery_phase_
                        == ObfmSpacingRecoveryPhase::LevelRecovery)
                    {
                        output.reason = ObfmSpacingOwnerReason::
                            AwaitingVerticalTurnaround;
                    }
                    else
                    {
                        recovery_post_turnaround_spacing_increase_observed_ =
                            recovery_post_turnaround_spacing_increase_observed_
                            || spacing_increased;
                        if (endpoint_available
                            && axis_geometry.projected_closure_mps <= 0.0
                            && recovery_post_turnaround_spacing_increase_observed_)
                        {
                            recovery_phase_ =
                                ObfmSpacingRecoveryPhase::Completed;
                            output.recovery_completed_this_tick = true;
                            output.reason =
                                ObfmSpacingOwnerReason::RecoveryCompleted;
                        }
                        else
                        {
                            output.reason = ObfmSpacingOwnerReason::
                                AwaitingRecoveryEndpoint;
                        }
                    }
                }
                recovery_previous_time_s_ = frame.t_sec;
                recovery_previous_down_velocity_mps_ = down_velocity;
                recovery_previous_altitude_m_ = altitude_m;
                recovery_previous_spacing_m_ =
                    axis_geometry.signed_station_spacing_m;
            }
        }

        if (recovery_phase_ == ObfmSpacingRecoveryPhase::Completed)
        {
            phase_ = ObfmSpacingOwnerPhase::WezReacquire;
            output.phase_changed = true;
            frozen_wez_epoch_valid_ = true;
            frozen_wez_phase_id_ = recovery_phase_id_;
            frozen_wez_official_range_m_ = recovery_official_range_m_;
            geometry_reason = ObfmSpacingOwnerReason::CommandGeometryUnavailable;
            geometry_domain = BuildReacquireGeometry(
                frame,
                reacquire_geometry,
                geometry_reason);
            if (geometry_domain != CalculationDomain::Available)
            {
                if (geometry_domain
                    == CalculationDomain::DeclaredReadyContradiction)
                {
                    contract(geometry_reason);
                }
                else
                {
                    release(geometry_reason);
                }
                return;
            }
        }
    }

    if (phase_ == ObfmSpacingOwnerPhase::WezReacquire)
    {
        if (!frozen_wez_epoch_valid_
            || recovery_phase_ != ObfmSpacingRecoveryPhase::Completed)
        {
            release(
                ObfmSpacingOwnerReason::WezRecoveryEvidenceUnavailable);
            return;
        }
        if (official_phase_id != frozen_wez_phase_id_
            || frame.own_offense.phase.max_range_m
                != frozen_wez_official_range_m_)
        {
            release(ObfmSpacingOwnerReason::WezEpochChanged);
            return;
        }
    }

    ObfmSpacingGuidanceCommand command{};
    ObfmSpacingOwnerReason command_reason =
        ObfmSpacingOwnerReason::CommandGeometryUnavailable;
    CalculationDomain command_domain = CalculationDomain::FiniteUnavailable;
    if (phase_ == ObfmSpacingOwnerPhase::WezReacquire)
    {
        command_domain = BuildWezCommand(
            frame,
            reacquire_geometry,
            input.flight_path_gamma_limit_available,
            input.flight_path_gamma_limit_rad,
            input.previous_energy_authority,
            command,
            command_reason);
    }
    else
    {
        command_domain = BuildAxisCommand(
            frame,
            axis_geometry,
            phase_,
            input.flight_path_gamma_limit_available,
            input.flight_path_gamma_limit_rad,
            input.previous_energy_authority,
            command,
            command_reason);
    }
    if (command_domain != CalculationDomain::Available)
    {
        if (command_domain == CalculationDomain::DeclaredReadyContradiction)
        {
            contract(command_reason);
        }
        else
        {
            release(command_reason);
        }
        return;
    }

    if (phase_ == ObfmSpacingOwnerPhase::PathEnergyExchange
        && current_projection_required_)
    {
        output.projection_required = true;
        output.preprojected_candidate = command;
        output.preprojected_candidate_valid = true;
        const bool entry_projection_matches = entry_projection_valid_
            && SameControlFrameIdentity(
                entry_projection_identity_,
                frame.frame_identity);
        if (!entry_projection_matches
            && !input.current_energy_projection.evaluated)
        {
            // Command-neutral first pass for a caller-owned shadow copy.  The
            // caller may project this raw tuple synchronously, discard this
            // mutated shadow, and replay one identical original shadow with
            // the typed result.  No candidate/release/completion authority is
            // asserted by this provisional receipt.
            output.task_completed = false;
            output.release_required = false;
            output.candidate_valid = false;
            output.candidate_count = 0U;
            output.reason =
                ObfmSpacingOwnerReason::CurrentEnergyProjectionRequired;
            return;
        }
        CalculationDomain projection_domain =
            CalculationDomain::FiniteUnavailable;
        if (entry_projection_matches)
        {
            if (!std::isfinite(entry_projection_raw_bias_m2ps3_)
                || !std::isfinite(entry_projection_admitted_bias_m2ps3_)
                || entry_projection_admitted_bias_m2ps3_ > 0.0)
            {
                command_reason = ObfmSpacingOwnerReason::
                    DeclaredReadyProjectionContradiction;
                projection_domain =
                    CalculationDomain::DeclaredReadyContradiction;
            }
            else
            {
                command.specific_energy_rate_bias_m2ps3 =
                    entry_projection_admitted_bias_m2ps3_;
                projection_domain = Float32Command(command)
                    ? CalculationDomain::Available
                    : CalculationDomain::FiniteUnavailable;
                command_reason = projection_domain
                        == CalculationDomain::Available
                    ? (entry_projection_admitted_
                        ? ObfmSpacingOwnerReason::CommandReady
                        : ObfmSpacingOwnerReason::
                            CurrentEnergyProjectionRejected)
                    : ObfmSpacingOwnerReason::
                        CommandFloat32DomainUnavailable;
            }
        }
        else
        {
            projection_domain = ApplyCurrentProjection(
                frame.frame_identity,
                input.current_energy_projection,
                command,
                command_reason);
        }
        if (projection_domain != CalculationDomain::Available)
        {
            if (projection_domain
                == CalculationDomain::DeclaredReadyContradiction)
            {
                contract(command_reason);
            }
            else
            {
                release(command_reason);
            }
            return;
        }
    }

    output.task_active = true;
    output.phase = phase_;
    output.completion_phase = completion_phase_;
    output.recovery_phase = recovery_phase_;
    output.candidate = command;
    output.candidate_valid = true;
    output.candidate_count = 1U;
    output.reason = command_reason;
}

void ObfmSpacingOwner::HaltTask(
    const bool official_employ_preemption,
    ObfmSpacingOwnerHaltReceipt& output,
    Status& status) noexcept
{
    output = ObfmSpacingOwnerHaltReceipt{};
    status = Status{};
    output.was_active = task_active_;
    output.official_employ_preemption = official_employ_preemption;
    if (official_employ_preemption
        && (!task_active_ || !employ_preemption_pending_
            || phase_ == ObfmSpacingOwnerPhase::PostHitRminArrest))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (official_employ_preemption)
    {
        ClearLifecycle(false);
        post_hit_pending_ = true;
    }
    else
    {
        ClearLifecycle(true);
    }
    output.valid = true;
    output.post_hit_pending = post_hit_pending_;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
