#include "LadyLuck/guidance/prefire/OfficialGunAttackResponse.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using LadyLuck::ControlIntent;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::prefire::OfficialGunSnapshotReference;

constexpr std::uint64_t PcgMultiplierHigh = UINT64_C(0x2360ed051fc65da4);
constexpr std::uint64_t PcgMultiplierLow = UINT64_C(0x4385df649fccf645);
constexpr std::uint64_t PcgIncrementHigh = UINT64_C(0x3b532b59957ad40a);
constexpr std::uint64_t PcgIncrementLow = UINT64_C(0x3b5267c90781bf85);
constexpr std::uint64_t PcgSeed17StateHigh = UINT64_C(0x90032ae9301c65d5);
constexpr std::uint64_t PcgSeed17StateLow = UINT64_C(0x8fc8e3434980b0a7);

// A binary64 interval cannot require more midpoint refinements than the full
// normal/subnormal exponent span plus two significands. The bound is a
// termination proof, not a tactical convergence tolerance.
constexpr std::int32_t RepresentableBisectionLimit =
    std::numeric_limits<double>::max_exponent
    - std::numeric_limits<double>::min_exponent
    + (2 * std::numeric_limits<double>::digits)
    + 8;

struct Wide64Product
{
    std::uint64_t high = 0U;
    std::uint64_t low = 0U;
};

struct RealRoots
{
    std::array<double, 2U> value{};
    std::uint32_t count = 0U;
};

struct JinkCandidates
{
    std::array<Vector3, 4U> value{};
    std::uint32_t count = 0U;
};

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool CheckedAdd(
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
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!std::isfinite(output))
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
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right < 0.0 && left >= maximum + right)
        || (right > 0.0 && left <= -maximum + right))
    {
        return false;
    }
    output = left - right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double absolute_left = std::fabs(left);
    const double absolute_right = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((absolute_left > 1.0
            && absolute_right >= maximum / absolute_left)
        || (absolute_right > 1.0
            && absolute_left >= maximum / absolute_right))
    {
        return false;
    }
    output = left * right;
    if (!std::isfinite(output))
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
    if (!std::isfinite(numerator)
        || !std::isfinite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    if (numerator == 0.0)
    {
        output = numerator / denominator;
        return true;
    }
    const double absolute_numerator = std::fabs(numerator);
    const double absolute_denominator = std::fabs(denominator);
    const double maximum = (std::numeric_limits<double>::max)();
    if (absolute_denominator < 1.0
        && absolute_numerator >= maximum * absolute_denominator)
    {
        return false;
    }
    output = numerator / denominator;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool SafeAdd(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    const Vector3 left_copy = left;
    const Vector3 right_copy = right;
    output = Vector3{};
    if (!FiniteVector(left_copy) || !FiniteVector(right_copy))
    {
        return false;
    }
    Vector3 candidate{};
    if (!CheckedAdd(left_copy[0], right_copy[0], candidate[0])
        || !CheckedAdd(left_copy[1], right_copy[1], candidate[1])
        || !CheckedAdd(left_copy[2], right_copy[2], candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeSubtract(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    const Vector3 left_copy = left;
    const Vector3 right_copy = right;
    output = Vector3{};
    if (!FiniteVector(left_copy) || !FiniteVector(right_copy))
    {
        return false;
    }
    Vector3 candidate{};
    if (!CheckedSubtract(left_copy[0], right_copy[0], candidate[0])
        || !CheckedSubtract(left_copy[1], right_copy[1], candidate[1])
        || !CheckedSubtract(left_copy[2], right_copy[2], candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeScale(
    const Vector3& value,
    const double scale,
    Vector3& output) noexcept
{
    const Vector3 value_copy = value;
    output = Vector3{};
    if (!FiniteVector(value_copy) || !std::isfinite(scale))
    {
        return false;
    }
    Vector3 candidate{};
    if (!CheckedMultiply(value_copy[0], scale, candidate[0])
        || !CheckedMultiply(value_copy[1], scale, candidate[1])
        || !CheckedMultiply(value_copy[2], scale, candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeDot(
    const Vector3& left,
    const Vector3& right,
    double& output) noexcept
{
    const Vector3 left_copy = left;
    const Vector3 right_copy = right;
    output = 0.0;
    if (!FiniteVector(left_copy) || !FiniteVector(right_copy))
    {
        return false;
    }
    double term0 = 0.0;
    double term1 = 0.0;
    double term2 = 0.0;
    double partial = 0.0;
    return CheckedMultiply(left_copy[0], right_copy[0], term0)
        && CheckedMultiply(left_copy[1], right_copy[1], term1)
        && CheckedMultiply(left_copy[2], right_copy[2], term2)
        && CheckedAdd(term0, term1, partial)
        && CheckedAdd(partial, term2, output);
}

bool SafeCross(
    const Vector3& left,
    const Vector3& right,
    Vector3& output) noexcept
{
    const Vector3 left_copy = left;
    const Vector3 right_copy = right;
    output = Vector3{};
    if (!FiniteVector(left_copy) || !FiniteVector(right_copy))
    {
        return false;
    }
    Vector3 candidate{};
    double left_term = 0.0;
    double right_term = 0.0;
    if (!CheckedMultiply(left_copy[1], right_copy[2], left_term)
        || !CheckedMultiply(left_copy[2], right_copy[1], right_term)
        || !CheckedSubtract(left_term, right_term, candidate[0])
        || !CheckedMultiply(left_copy[2], right_copy[0], left_term)
        || !CheckedMultiply(left_copy[0], right_copy[2], right_term)
        || !CheckedSubtract(left_term, right_term, candidate[1])
        || !CheckedMultiply(left_copy[0], right_copy[1], left_term)
        || !CheckedMultiply(left_copy[1], right_copy[0], right_term)
        || !CheckedSubtract(left_term, right_term, candidate[2]))
    {
        return false;
    }
    output = candidate;
    return true;
}

bool SafeNorm(const Vector3& value, double& output) noexcept
{
    const Vector3 value_copy = value;
    output = 0.0;
    if (!FiniteVector(value_copy))
    {
        return false;
    }
    const double maximum_component = std::fmax(
        std::fmax(std::fabs(value_copy[0]), std::fabs(value_copy[1])),
        std::fabs(value_copy[2]));
    if (maximum_component == 0.0)
    {
        return true;
    }
    double scaled0 = 0.0;
    double scaled1 = 0.0;
    double scaled2 = 0.0;
    double square0 = 0.0;
    double square1 = 0.0;
    double square2 = 0.0;
    double partial = 0.0;
    double sum = 0.0;
    if (!CheckedDivide(value_copy[0], maximum_component, scaled0)
        || !CheckedDivide(value_copy[1], maximum_component, scaled1)
        || !CheckedDivide(value_copy[2], maximum_component, scaled2)
        || !CheckedMultiply(scaled0, scaled0, square0)
        || !CheckedMultiply(scaled1, scaled1, square1)
        || !CheckedMultiply(scaled2, scaled2, square2)
        || !CheckedAdd(square0, square1, partial)
        || !CheckedAdd(partial, square2, sum)
        || sum < 0.0)
    {
        return false;
    }
    const double scaled_norm = std::sqrt(sum);
    return std::isfinite(scaled_norm)
        && CheckedMultiply(maximum_component, scaled_norm, output);
}

bool UnitVector(const Vector3& value, Vector3& output) noexcept
{
    double magnitude = 0.0;
    double inverse_magnitude = 0.0;
    if (!SafeNorm(value, magnitude)
        || magnitude <= LadyLuck::constants::Tiny
        || !CheckedDivide(1.0, magnitude, inverse_magnitude))
    {
        return false;
    }
    return SafeScale(value, inverse_magnitude, output);
}

bool ProjectNormalTo(
    const Vector3& value,
    const Vector3& axis,
    Vector3& output) noexcept
{
    Vector3 axis_hat{};
    double along = 0.0;
    Vector3 along_axis{};
    Vector3 projected{};
    return UnitVector(axis, axis_hat)
        && SafeDot(value, axis_hat, along)
        && SafeScale(axis_hat, along, along_axis)
        && SafeSubtract(value, along_axis, projected)
        && UnitVector(projected, output);
}

bool AngleToAxis(
    const Vector3& value,
    const Vector3& axis,
    double& angle_rad) noexcept
{
    double along = 0.0;
    Vector3 along_axis{};
    Vector3 transverse{};
    double transverse_norm = 0.0;
    if (!SafeDot(value, axis, along)
        || !SafeScale(axis, along, along_axis)
        || !SafeSubtract(value, along_axis, transverse)
        || !SafeNorm(transverse, transverse_norm))
    {
        return false;
    }
    angle_rad = std::atan2(transverse_norm, along);
    return std::isfinite(angle_rad);
}

bool SignedAngleAboutAxis(
    const Vector3& source,
    const Vector3& target,
    const Vector3& axis,
    double& angle_rad) noexcept
{
    double cosine = 0.0;
    Vector3 cross{};
    double sine = 0.0;
    if (!SafeDot(source, target, cosine)
        || !SafeCross(source, target, cross)
        || !SafeDot(axis, cross, sine))
    {
        return false;
    }
    if (cosine > 1.0)
    {
        cosine = 1.0;
    }
    else if (cosine < -1.0)
    {
        cosine = -1.0;
    }
    angle_rad = std::atan2(sine, cosine);
    return std::isfinite(angle_rad);
}

bool SameDirection(
    const Vector3& left,
    const Vector3& right) noexcept
{
    Vector3 difference{};
    double distance = 0.0;
    return SafeSubtract(left, right, difference)
        && SafeNorm(difference, distance)
        && distance <= std::sqrt(std::numeric_limits<double>::epsilon());
}

bool BaselineDirection(
    const LadyLuck::DogfightGeometryFrame& frame,
    const ControlIntent& baseline,
    Vector3& output) noexcept
{
    Vector3 displacement{};
    return SafeSubtract(
            baseline.aim_point_m,
            frame.own.position_ned_m,
            displacement)
        && UnitVector(displacement, output);
}

bool BaselineTurnSense(
    const LadyLuck::DogfightGeometryFrame& frame,
    const ControlIntent& baseline,
    std::int32_t& output) noexcept
{
    const Vector3 down{0.0, 0.0, 1.0};
    Vector3 velocity{};
    Vector3 lateral_raw{};
    Vector3 lateral{};
    Vector3 baseline_direction{};
    double side_value = 0.0;
    if (!UnitVector(frame.own.velocity_ned_mps, velocity)
        || !SafeCross(down, velocity, lateral_raw)
        || !UnitVector(lateral_raw, lateral)
        || !BaselineDirection(frame, baseline, baseline_direction)
        || !SafeDot(baseline_direction, lateral, side_value)
        || std::fabs(side_value) <= LadyLuck::constants::Tiny)
    {
        return false;
    }
    output = side_value > 0.0 ? 1 : -1;
    return true;
}

bool ValidBaseBreak(
    const LadyLuck::DogfightGeometryFrame& frame,
    const ControlIntent& baseline) noexcept
{
    Status validation{};
    baseline.Validate(validation);
    return validation.sample_valid()
        && LadyLuck::SameControlFrameIdentity(
            frame.frame_identity,
            baseline.frame_identity)
        && baseline.route_kind == LadyLuck::ControlRouteKind::AimPoint
        && baseline.behavior_id
            == LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak
        && baseline.mode_id == LadyLuck::DoctrineModeId::Dbfm
        && baseline.writer_id
            == LadyLuck::ControlIntentWriterGunDefenseHorizontalBreak;
}

bool ValidObservation(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::guidance::prefire::GunAttackFormObservation& observation)
    noexcept
{
    return observation.valid
        && observation.geometry.valid
        && std::isfinite(frame.t_sec)
        && frame.t_sec >= 0.0
        && std::isfinite(observation.geometry.t_sec)
        && FiniteVector(observation.geometry.los_rate_world_radps);
}

bool BuildJinkCandidates(
    const LadyLuck::DogfightGeometryFrame& frame,
    const ControlIntent& baseline,
    const Vector3& los,
    JinkCandidates& output) noexcept
{
    output = JinkCandidates{};
    Vector3 baseline_direction{};
    if (!BaselineDirection(frame, baseline, baseline_direction))
    {
        return false;
    }

    Vector3 axes[2U]{};
    bool axis_valid[2U]{false, false};
    axis_valid[0U] = ProjectNormalTo(baseline_direction, los, axes[0U]);
    Vector3 negative_down{};
    if (!SafeScale(frame.own.down_ned, -1.0, negative_down))
    {
        return false;
    }
    axis_valid[1U] = ProjectNormalTo(negative_down, los, axes[1U]);

    for (std::uint32_t axis_index = 0U; axis_index < 2U; ++axis_index)
    {
        if (!axis_valid[axis_index])
        {
            continue;
        }
        for (std::int32_t sign = 1; sign >= -1; sign -= 2)
        {
            Vector3 candidate{};
            if (!SafeScale(axes[axis_index], static_cast<double>(sign), candidate))
            {
                return false;
            }
            bool unique = true;
            for (std::uint32_t existing = 0U;
                 existing < output.count;
                 ++existing)
            {
                if (SameDirection(candidate, output.value[existing]))
                {
                    unique = false;
                    break;
                }
            }
            if (unique && output.count < output.value.size())
            {
                output.value[output.count] = candidate;
                ++output.count;
            }
        }
    }
    return output.count >= 2U;
}

RealRoots RealQuadraticRoots(
    const double a,
    const double b,
    const double c) noexcept
{
    RealRoots roots{};
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
    {
        return roots;
    }
    if (a == 0.0)
    {
        if (b == 0.0)
        {
            return roots;
        }
        double root = 0.0;
        if (CheckedDivide(-c, b, root))
        {
            roots.value[0U] = root;
            roots.count = 1U;
        }
        return roots;
    }

    double b_squared = 0.0;
    double a_c = 0.0;
    double four_a_c = 0.0;
    double discriminant = 0.0;
    if (!CheckedMultiply(b, b, b_squared)
        || !CheckedMultiply(a, c, a_c)
        || !CheckedMultiply(4.0, a_c, four_a_c)
        || !CheckedSubtract(b_squared, four_a_c, discriminant)
        || discriminant < 0.0)
    {
        return roots;
    }
    const double square_root = std::sqrt(discriminant);
    if (!std::isfinite(square_root))
    {
        return roots;
    }
    if (square_root == 0.0)
    {
        double numerator = 0.0;
        double root = 0.0;
        if (CheckedMultiply(-0.5, b, numerator)
            && CheckedDivide(numerator, a, root))
        {
            roots.value[0U] = root;
            roots.count = 1U;
        }
        return roots;
    }

    double signed_sum = 0.0;
    double q = 0.0;
    if (!CheckedAdd(b, std::copysign(square_root, b), signed_sum)
        || !CheckedMultiply(-0.5, signed_sum, q))
    {
        return roots;
    }
    if (q == 0.0)
    {
        double numerator = 0.0;
        double root = 0.0;
        if (CheckedMultiply(-0.5, b, numerator)
            && CheckedDivide(numerator, a, root))
        {
            roots.value[0U] = root;
            roots.count = 1U;
        }
        return roots;
    }
    double first = 0.0;
    if (CheckedDivide(q, a, first))
    {
        roots.value[roots.count] = first;
        ++roots.count;
    }
    double second = 0.0;
    if (CheckedDivide(c, q, second)
        && roots.count < roots.value.size())
    {
        roots.value[roots.count] = second;
        ++roots.count;
    }
    return roots;
}

bool PredictedAta(
    const Vector3& base_relative,
    const Vector3& delta_per_tan_bank,
    const Vector3& nose_hat,
    const double tan_bank,
    double& ata_rad) noexcept
{
    Vector3 delta{};
    Vector3 relative{};
    return SafeScale(delta_per_tan_bank, tan_bank, delta)
        && SafeAdd(base_relative, delta, relative)
        && AngleToAxis(relative, nose_hat, ata_rad);
}

bool FirstRepresentableExit(
    const Vector3& base_relative,
    const Vector3& delta_per_tan_bank,
    const Vector3& nose_hat,
    const double cone_rad,
    const std::int32_t side_sign,
    const double outside_magnitude,
    double& selected_tan_bank) noexcept
{
    if ((side_sign != -1 && side_sign != 1)
        || !std::isfinite(outside_magnitude)
        || outside_magnitude <= 0.0)
    {
        return false;
    }
    double outside_ata = 0.0;
    if (!PredictedAta(
            base_relative,
            delta_per_tan_bank,
            nose_hat,
            static_cast<double>(side_sign) * outside_magnitude,
            outside_ata)
        || outside_ata < cone_rad)
    {
        return false;
    }

    double inside = 0.0;
    double outside = outside_magnitude;
    for (std::int32_t iteration = 0;
         iteration < RepresentableBisectionLimit;
         ++iteration)
    {
        const double midpoint = inside + 0.5 * (outside - inside);
        if (!std::isfinite(midpoint))
        {
            return false;
        }
        if (midpoint == inside || midpoint == outside)
        {
            selected_tan_bank = static_cast<double>(side_sign) * outside;
            return std::isfinite(selected_tan_bank);
        }
        double midpoint_ata = 0.0;
        if (!PredictedAta(
                base_relative,
                delta_per_tan_bank,
                nose_hat,
                static_cast<double>(side_sign) * midpoint,
                midpoint_ata))
        {
            return false;
        }
        if (midpoint_ata >= cone_rad)
        {
            outside = midpoint;
        }
        else
        {
            inside = midpoint;
        }
    }
    return false;
}

bool SnapshotWezEscapeReference(
    const LadyLuck::DogfightGeometryFrame& frame,
    const double horizon_s,
    const double cone_rad,
    const double load_limit_g,
    const std::int32_t tie_side_sign,
    OfficialGunSnapshotReference& output) noexcept
{
    output = OfficialGunSnapshotReference{};
    if (!std::isfinite(horizon_s) || horizon_s <= 0.0
        || !std::isfinite(cone_rad) || cone_rad <= 0.0
        || cone_rad >= LadyLuck::constants::Pi / 2.0
        || !std::isfinite(load_limit_g) || load_limit_g < 1.0
        || (tie_side_sign != -1 && tie_side_sign != 1))
    {
        return false;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& attacker_position = frame.opponent.position_ned_m;
    const Vector3& attacker_velocity = frame.opponent.velocity_ned_mps;
    const Vector3& attacker_nose = frame.opponent.nose_ned;
    if (!FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(attacker_position)
        || !FiniteVector(attacker_velocity)
        || !FiniteVector(attacker_nose))
    {
        return false;
    }

    Vector3 velocity_hat{};
    Vector3 nose_hat{};
    const Vector3 down{0.0, 0.0, 1.0};
    Vector3 lateral_raw{};
    Vector3 lateral_axis{};
    Vector3 support_raw{};
    Vector3 support_axis{};
    if (!UnitVector(own_velocity, velocity_hat)
        || !UnitVector(attacker_nose, nose_hat)
        || !SafeCross(down, velocity_hat, lateral_raw)
        || !UnitVector(lateral_raw, lateral_axis)
        || !SafeCross(velocity_hat, lateral_axis, support_raw)
        || !SafeScale(support_raw, -1.0, support_raw)
        || !UnitVector(support_raw, support_axis))
    {
        return false;
    }

    const double upward_support_fraction = -support_axis[2];
    if (!std::isfinite(upward_support_fraction)
        || upward_support_fraction <= LadyLuck::constants::Tiny)
    {
        return false;
    }
    double load_product = 0.0;
    if (!CheckedMultiply(
            load_limit_g,
            upward_support_fraction,
            load_product)
        || load_product < 1.0)
    {
        return false;
    }
    double load_product_square = 0.0;
    double tan_bank_square = 0.0;
    if (!CheckedMultiply(
            load_product,
            load_product,
            load_product_square)
        || !CheckedSubtract(load_product_square, 1.0, tan_bank_square))
    {
        return false;
    }
    const double tan_bank_limit = std::sqrt(
        tan_bank_square > 0.0 ? tan_bank_square : 0.0);
    if (!std::isfinite(tan_bank_limit))
    {
        return false;
    }

    double gravity_over_support = 0.0;
    if (!CheckedDivide(
            LadyLuck::constants::StandardGravityMps2,
            upward_support_fraction,
            gravity_over_support))
    {
        return false;
    }
    Vector3 support_specific_force{};
    Vector3 support_acceleration{};
    if (!SafeScale(support_axis, gravity_over_support, support_specific_force)
        || !SafeAdd(
            Vector3{0.0, 0.0, LadyLuck::constants::StandardGravityMps2},
            support_specific_force,
            support_acceleration))
    {
        return false;
    }

    Vector3 own_horizon_velocity{};
    Vector3 attacker_horizon_velocity{};
    Vector3 coast_relative{};
    Vector3 temporary{};
    if (!SafeScale(own_velocity, horizon_s, own_horizon_velocity)
        || !SafeScale(
            attacker_velocity,
            horizon_s,
            attacker_horizon_velocity)
        || !SafeAdd(own_position, own_horizon_velocity, coast_relative)
        || !SafeSubtract(coast_relative, attacker_position, coast_relative)
        || !SafeSubtract(
            coast_relative,
            attacker_horizon_velocity,
            coast_relative))
    {
        return false;
    }
    double coast_ata = 0.0;
    if (!AngleToAxis(coast_relative, nose_hat, coast_ata))
    {
        return false;
    }

    double horizon_square = 0.0;
    double half_horizon_square = 0.0;
    double lateral_displacement_scale = 0.0;
    if (!CheckedMultiply(horizon_s, horizon_s, horizon_square)
        || !CheckedMultiply(0.5, horizon_square, half_horizon_square)
        || !CheckedMultiply(
            half_horizon_square,
            gravity_over_support,
            lateral_displacement_scale))
    {
        return false;
    }
    Vector3 base_relative{};
    Vector3 delta_per_tan_bank{};
    if (!SafeScale(
            support_acceleration,
            half_horizon_square,
            temporary)
        || !SafeAdd(coast_relative, temporary, base_relative)
        || !SafeScale(
            lateral_axis,
            lateral_displacement_scale,
            delta_per_tan_bank))
    {
        return false;
    }

    double center_ata = 0.0;
    if (!PredictedAta(
            base_relative,
            delta_per_tan_bank,
            nose_hat,
            0.0,
            center_ata))
    {
        return false;
    }
    double selected_tan_bank = 0.0;
    bool boundary_reachable = center_ata >= cone_rad;
    bool load_saturated = false;

    if (!boundary_reachable && tan_bank_limit > 0.0)
    {
        const double cosine = std::cos(cone_rad);
        double cosine_squared = 0.0;
        double nose_base = 0.0;
        double nose_delta = 0.0;
        double delta_norm_square = 0.0;
        double base_delta = 0.0;
        double base_norm_square = 0.0;
        if (!CheckedMultiply(cosine, cosine, cosine_squared)
            || !SafeDot(nose_hat, base_relative, nose_base)
            || !SafeDot(nose_hat, delta_per_tan_bank, nose_delta)
            || !SafeDot(
                delta_per_tan_bank,
                delta_per_tan_bank,
                delta_norm_square)
            || !SafeDot(base_relative, delta_per_tan_bank, base_delta)
            || !SafeDot(base_relative, base_relative, base_norm_square))
        {
            return false;
        }
        double nose_delta_square = 0.0;
        double cosine_delta_norm = 0.0;
        double quadratic_a = 0.0;
        double nose_base_delta = 0.0;
        double cosine_base_delta = 0.0;
        double quadratic_b_inner = 0.0;
        double quadratic_b = 0.0;
        double nose_base_square = 0.0;
        double cosine_base_norm = 0.0;
        double quadratic_c = 0.0;
        if (!CheckedMultiply(
                nose_delta,
                nose_delta,
                nose_delta_square)
            || !CheckedMultiply(
                cosine_squared,
                delta_norm_square,
                cosine_delta_norm)
            || !CheckedSubtract(
                nose_delta_square,
                cosine_delta_norm,
                quadratic_a)
            || !CheckedMultiply(
                nose_base,
                nose_delta,
                nose_base_delta)
            || !CheckedMultiply(
                cosine_squared,
                base_delta,
                cosine_base_delta)
            || !CheckedSubtract(
                nose_base_delta,
                cosine_base_delta,
                quadratic_b_inner)
            || !CheckedMultiply(2.0, quadratic_b_inner, quadratic_b)
            || !CheckedMultiply(
                nose_base,
                nose_base,
                nose_base_square)
            || !CheckedMultiply(
                cosine_squared,
                base_norm_square,
                cosine_base_norm)
            || !CheckedSubtract(
                nose_base_square,
                cosine_base_norm,
                quadratic_c))
        {
            return false;
        }
        const RealRoots all_roots = RealQuadraticRoots(
            quadratic_a,
            quadratic_b,
            quadratic_c);
        std::array<double, 2U> roots{};
        std::uint32_t root_count = 0U;
        for (std::uint32_t index = 0U; index < all_roots.count; ++index)
        {
            const double root = all_roots.value[index];
            if (root != 0.0
                && std::fabs(root) <= tan_bank_limit
                && root_count < roots.size())
            {
                roots[root_count] = root;
                ++root_count;
            }
        }

        std::array<double, 2U> feasible_exits{};
        std::uint32_t feasible_count = 0U;
        for (std::int32_t candidate_side = -1;
             candidate_side <= 1;
             candidate_side += 2)
        {
            double endpoint_ata = 0.0;
            if (!PredictedAta(
                    base_relative,
                    delta_per_tan_bank,
                    nose_hat,
                    static_cast<double>(candidate_side) * tan_bank_limit,
                    endpoint_ata))
            {
                return false;
            }
            bool outside_available = false;
            double outside_magnitude = 0.0;
            if (endpoint_ata >= cone_rad)
            {
                outside_available = true;
                outside_magnitude = tan_bank_limit;
            }
            else
            {
                std::array<double, 2U> side_roots{};
                std::uint32_t side_count = 0U;
                for (std::uint32_t index = 0U; index < root_count; ++index)
                {
                    const double root = roots[index];
                    const std::int32_t root_side = std::signbit(root) ? -1 : 1;
                    if (root_side == candidate_side
                        && side_count < side_roots.size())
                    {
                        side_roots[side_count] = std::fabs(root);
                        ++side_count;
                    }
                }
                if (side_count == 2U)
                {
                    if (side_roots[1U] < side_roots[0U])
                    {
                        const double swap = side_roots[0U];
                        side_roots[0U] = side_roots[1U];
                        side_roots[1U] = swap;
                    }
                    const double midpoint = side_roots[0U]
                        + 0.5 * (side_roots[1U] - side_roots[0U]);
                    double midpoint_ata = 0.0;
                    if (!std::isfinite(midpoint)
                        || !PredictedAta(
                            base_relative,
                            delta_per_tan_bank,
                            nose_hat,
                            static_cast<double>(candidate_side) * midpoint,
                            midpoint_ata))
                    {
                        return false;
                    }
                    if (midpoint_ata >= cone_rad)
                    {
                        outside_available = true;
                        outside_magnitude = midpoint;
                    }
                }
            }
            if (outside_available && feasible_count < feasible_exits.size())
            {
                double exit = 0.0;
                if (!FirstRepresentableExit(
                        base_relative,
                        delta_per_tan_bank,
                        nose_hat,
                        cone_rad,
                        candidate_side,
                        outside_magnitude,
                        exit))
                {
                    return false;
                }
                feasible_exits[feasible_count] = exit;
                ++feasible_count;
            }
        }

        if (feasible_count > 0U)
        {
            selected_tan_bank = feasible_exits[0U];
            for (std::uint32_t index = 1U; index < feasible_count; ++index)
            {
                const double candidate = feasible_exits[index];
                const double candidate_magnitude = std::fabs(candidate);
                const double selected_magnitude = std::fabs(selected_tan_bank);
                const std::int32_t candidate_side =
                    std::signbit(candidate) ? -1 : 1;
                const std::int32_t selected_side =
                    std::signbit(selected_tan_bank) ? -1 : 1;
                if (candidate_magnitude < selected_magnitude
                    || (candidate_magnitude == selected_magnitude
                        && candidate_side == tie_side_sign
                        && selected_side != tie_side_sign))
                {
                    selected_tan_bank = candidate;
                }
            }
            boundary_reachable = true;
        }
        else
        {
            double positive_ata = 0.0;
            double negative_ata = 0.0;
            if (!PredictedAta(
                    base_relative,
                    delta_per_tan_bank,
                    nose_hat,
                    tan_bank_limit,
                    positive_ata)
                || !PredictedAta(
                    base_relative,
                    delta_per_tan_bank,
                    nose_hat,
                    -tan_bank_limit,
                    negative_ata))
            {
                return false;
            }
            if (positive_ata > negative_ata)
            {
                selected_tan_bank = tan_bank_limit;
            }
            else if (negative_ata > positive_ata)
            {
                selected_tan_bank = -tan_bank_limit;
            }
            else
            {
                selected_tan_bank =
                    static_cast<double>(tie_side_sign) * tan_bank_limit;
            }
            load_saturated = true;
        }
    }
    else if (!boundary_reachable)
    {
        load_saturated = true;
    }

    const std::int32_t side_sign = selected_tan_bank == 0.0
        ? tie_side_sign
        : (std::signbit(selected_tan_bank) ? -1 : 1);
    const double lift_scale = std::hypot(1.0, selected_tan_bank);
    if (!std::isfinite(lift_scale) || lift_scale <= 0.0)
    {
        return false;
    }
    Vector3 lateral_component{};
    Vector3 target_lift_raw{};
    Vector3 target_lift{};
    double inverse_lift_scale = 0.0;
    if (!SafeScale(lateral_axis, selected_tan_bank, lateral_component)
        || !SafeAdd(support_axis, lateral_component, target_lift_raw)
        || !CheckedDivide(1.0, lift_scale, inverse_lift_scale)
        || !SafeScale(
            target_lift_raw,
            inverse_lift_scale,
            target_lift))
    {
        return false;
    }
    double target_load = 0.0;
    if (!CheckedDivide(
            lift_scale,
            upward_support_fraction,
            target_load))
    {
        return false;
    }
    const double target_bank = std::atan(selected_tan_bank);
    const double maximum = (std::numeric_limits<double>::max)();
    const double permitted_load = load_limit_g == maximum
        ? maximum
        : std::nextafter(
            load_limit_g,
            maximum);
    double commanded_ata = 0.0;
    if (!std::isfinite(target_load)
        || target_load < 1.0
        || target_load > permitted_load
        || !std::isfinite(target_bank)
        || !PredictedAta(
            base_relative,
            delta_per_tan_bank,
            nose_hat,
            selected_tan_bank,
            commanded_ata))
    {
        return false;
    }

    output.valid = true;
    output.target_lift_direction_ned = target_lift;
    output.target_bank_rad = target_bank;
    output.target_load_factor_g = target_load;
    output.horizon_s = horizon_s;
    output.cone_angle_rad = cone_rad;
    output.coast_predicted_ata_rad = coast_ata;
    output.commanded_predicted_ata_rad = commanded_ata;
    output.boundary_reachable = boundary_reachable;
    output.load_saturated = load_saturated;
    output.side_sign = side_sign;
    return true;
}

bool ValidForceFeedback(
    const LadyLuck::guidance::prefire::OfficialGunForceFeedback* feedback,
    double& target_bank_rad,
    double& observed_bank_rad) noexcept
{
    if (feedback == nullptr
        || !feedback->available
        || !feedback->fresh
        || !feedback->cis_v4_backend
        || !feedback->snapshot_behavior
        || !feedback->direct_force_tracking_requested
        || !feedback->observation_only
        || !feedback->target_bank_valid
        || !feedback->observed_bank_valid
        || !std::isfinite(feedback->target_bank_rad)
        || !std::isfinite(feedback->observed_bank_rad)
        || std::cos(feedback->target_bank_rad) <= 0.0
        || std::cos(feedback->observed_bank_rad) <= 0.0)
    {
        return false;
    }
    target_bank_rad = feedback->target_bank_rad;
    observed_bank_rad = feedback->observed_bank_rad;
    return true;
}

double WrappedDifference(
    const double left,
    const double right) noexcept
{
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return 0.0;
    }
    double difference = 0.0;
    if (!CheckedSubtract(left, right, difference))
    {
        constexpr double TwoPi = 2.0 * LadyLuck::constants::Pi;
        const double left_wrapped = std::remainder(left, TwoPi);
        const double right_wrapped = std::remainder(right, TwoPi);
        if (!std::isfinite(left_wrapped)
            || !std::isfinite(right_wrapped)
            || !CheckedSubtract(
                left_wrapped,
                right_wrapped,
                difference))
        {
            return 0.0;
        }
    }
    return std::atan2(std::sin(difference), std::cos(difference));
}

Wide64Product Multiply64(
    const std::uint64_t left,
    const std::uint64_t right) noexcept
{
    constexpr std::uint64_t LowMask = UINT64_C(0xffffffff);
    const std::uint64_t left_low = left & LowMask;
    const std::uint64_t left_high = left >> 32U;
    const std::uint64_t right_low = right & LowMask;
    const std::uint64_t right_high = right >> 32U;

    const std::uint64_t first = left_low * right_low;
    const std::uint64_t second = left_high * right_low + (first >> 32U);
    std::uint64_t middle = second & LowMask;
    const std::uint64_t carry = second >> 32U;
    middle += left_low * right_high;

    Wide64Product output{};
    output.high = left_high * right_high + carry + (middle >> 32U);
    output.low = (middle << 32U) | (first & LowMask);
    return output;
}

std::uint64_t RotateRight64(
    const std::uint64_t value,
    const std::uint32_t rotation) noexcept
{
    const std::uint32_t amount = rotation & 63U;
    return (value >> amount)
        | (value << ((0U - amount) & 63U));
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

const char* OfficialGunAttackResponseReasonLabel(
    const OfficialGunAttackResponseReason reason) noexcept
{
    switch (reason)
    {
    case OfficialGunAttackResponseReason::NotEvaluated:
        return "NOT_EVALUATED";
    case OfficialGunAttackResponseReason::NonOwner:
        return "NON_OWNER";
    case OfficialGunAttackResponseReason::FeatureDisabled:
        return "FEATURE_DISABLED";
    case OfficialGunAttackResponseReason::ActivationContractRejected:
        return "ACTIVATION_CONTRACT_REJECTED";
    case OfficialGunAttackResponseReason::BaseBreakContractRejected:
        return "BASE_BREAK_CONTRACT_REJECTED";
    case OfficialGunAttackResponseReason::ResponseContractUnavailable:
        return "RESPONSE_CONTRACT_UNAVAILABLE";
    case OfficialGunAttackResponseReason::ObservationUnavailable:
        return "OBSERVATION_UNAVAILABLE";
    case OfficialGunAttackResponseReason::IndeterminateFormKeepsBreak:
        return "INDETERMINATE_FORM_KEEPS_BREAK";
    case OfficialGunAttackResponseReason::TrackingGeometryUnavailable:
        return "TRACKING_GEOMETRY_UNAVAILABLE";
    case OfficialGunAttackResponseReason::TrackingJinkSelected:
        return "TRACKING_JINK_SELECTED";
    case OfficialGunAttackResponseReason::SnapshotCapabilityUnavailable:
        return "SNAPSHOT_CAPABILITY_UNAVAILABLE";
    case OfficialGunAttackResponseReason::SnapshotGeometryUnavailable:
        return "SNAPSHOT_GEOMETRY_UNAVAILABLE";
    case OfficialGunAttackResponseReason::SnapshotReferenceUnavailable:
        return "SNAPSHOT_REFERENCE_UNAVAILABLE";
    case OfficialGunAttackResponseReason::SnapshotPlaneChangeSelected:
        return "SNAPSHOT_PLANE_CHANGE_SELECTED";
    case OfficialGunAttackResponseReason::TrackingFormKeepsBreak:
        return "TRACKING_FORM_KEEPS_BREAK";
    default:
        return "UNKNOWN";
    }
}

OfficialGunAttackResponsePolicy::OfficialGunAttackResponsePolicy() noexcept
{
    Reset();
}

void OfficialGunAttackResponsePolicy::Reset() noexcept
{
    rng_state_.high = PcgSeed17StateHigh;
    rng_state_.low = PcgSeed17StateLow;
    rng_uint32_cached_ = false;
    rng_cached_uint32_ = 0U;
    ClearResponseState();
}

void OfficialGunAttackResponsePolicy::ClearEpisode() noexcept
{
    ClearResponseState();
}

void OfficialGunAttackResponsePolicy::ClearSnapshotState() noexcept
{
    snapshot_target_lift_valid_ = false;
    snapshot_target_lift_ned_ = Vector3{};
    snapshot_reference_valid_ = false;
    snapshot_reference_ = OfficialGunSnapshotReference{};
    snapshot_direction_valid_ = false;
    snapshot_direction_sign_ = 0;
    snapshot_capture_resolution_valid_ = false;
    snapshot_capture_resolution_rad_ = 0.0;
    snapshot_force_feedback_primed_ = false;
    snapshot_previous_force_valid_ = false;
    snapshot_previous_force_target_bank_rad_ = 0.0;
    snapshot_previous_force_bank_rad_ = 0.0;
    snapshot_force_target_crossed_ = false;
}

void OfficialGunAttackResponsePolicy::ClearResponseState() noexcept
{
    held_jink_direction_valid_ = false;
    held_jink_direction_ned_ = Vector3{};
    ClearSnapshotState();
    previous_form_ = GunAttackForm::Indeterminate;
}

std::uint64_t OfficialGunAttackResponsePolicy::NextRaw64() noexcept
{
    const Wide64Product low_product = Multiply64(
        rng_state_.low,
        PcgMultiplierLow);
    const std::uint64_t cross_low = rng_state_.low * PcgMultiplierHigh
        + rng_state_.high * PcgMultiplierLow;
    const std::uint64_t next_low_without_increment = low_product.low;
    const std::uint64_t next_high_without_increment =
        low_product.high + cross_low;
    const std::uint64_t next_low =
        next_low_without_increment + PcgIncrementLow;
    const std::uint64_t carry =
        next_low < next_low_without_increment ? 1U : 0U;
    const std::uint64_t next_high =
        next_high_without_increment + PcgIncrementHigh + carry;
    rng_state_.high = next_high;
    rng_state_.low = next_low;

    return RotateRight64(
        rng_state_.high ^ rng_state_.low,
        static_cast<std::uint32_t>(rng_state_.high >> 58U));
}

std::uint32_t OfficialGunAttackResponsePolicy::NextUint32() noexcept
{
    if (rng_uint32_cached_)
    {
        rng_uint32_cached_ = false;
        return rng_cached_uint32_;
    }
    const std::uint64_t raw = NextRaw64();
    rng_cached_uint32_ = static_cast<std::uint32_t>(raw >> 32U);
    rng_uint32_cached_ = true;
    return static_cast<std::uint32_t>(raw);
}

std::uint32_t OfficialGunAttackResponsePolicy::NextBoundedIndex(
    const std::uint32_t upper_bound) noexcept
{
    if (upper_bound <= 1U)
    {
        return 0U;
    }
    const std::uint32_t threshold =
        static_cast<std::uint32_t>(0U - upper_bound) % upper_bound;
    // The validated ON policy calls this only with bounds 2..4. Sixty-four
    // bounded rejection attempts make termination structural. If the exact
    // NumPy rejection predicate were to reject all of them, selecting zero is
    // still a valid seeded geometric candidate and cannot remove BREAK.
    for (std::uint32_t attempt = 0U; attempt < 64U; ++attempt)
    {
        const std::uint32_t random = NextUint32();
        const std::uint64_t product =
            static_cast<std::uint64_t>(random) * upper_bound;
        const std::uint32_t low = static_cast<std::uint32_t>(product);
        if (low >= threshold)
        {
            return static_cast<std::uint32_t>(product >> 32U);
        }
    }
    return 0U;
}

void OfficialGunAttackResponsePolicy::Evaluate(
    const DogfightGeometryFrame& frame,
    const bool root_official_gun_owner_selected,
    const OfficialGunAttackResponseActivation& activation,
    const GunAttackFormObservation* observation,
    const bool response_horizon_available,
    const double response_horizon_s,
    const bool response_cone_available,
    const double response_cone_rad,
    const bool load_capability_admitted,
    const double load_capability_g,
    const OfficialGunForceFeedback* previous_feedback,
    const ControlIntent& base_break,
    OfficialGunAttackResponseReceipt& receipt,
    ControlIntent& output,
    Status& status) noexcept
{
    receipt = OfficialGunAttackResponseReceipt{};
    receipt.frame_identity = frame.frame_identity;
    receipt.evaluated = true;
    output = base_break;
    status = Status{};

    if (!ValidBaseBreak(frame, base_break))
    {
        receipt.reason =
            OfficialGunAttackResponseReason::BaseBreakContractRejected;
        receipt.declared_ready_contract_contradiction = true;
        status.code = StatusCode::InvalidConfiguration;
        ClearEpisode();
        return;
    }

    if (!root_official_gun_owner_selected)
    {
        receipt.reason = OfficialGunAttackResponseReason::NonOwner;
        ClearEpisode();
        return;
    }
    if (!activation.enabled)
    {
        receipt.reason = OfficialGunAttackResponseReason::FeatureDisabled;
        receipt.declared_ready_contract_contradiction =
            activation.exact_provenance
            || activation.seed != 0U
            || activation.tracking_jink_enabled;
        ClearEpisode();
        return;
    }
    if (!activation.exact_provenance
        || activation.seed != OfficialGunAttackResponseProductionSeed)
    {
        receipt.reason =
            OfficialGunAttackResponseReason::ActivationContractRejected;
        receipt.declared_ready_contract_contradiction = true;
        ClearEpisode();
        return;
    }
    if (!response_horizon_available
        || !std::isfinite(response_horizon_s)
        || response_horizon_s <= 0.0
        || !response_cone_available
        || !std::isfinite(response_cone_rad)
        || response_cone_rad <= 0.0
        || response_cone_rad >= constants::Pi / 2.0)
    {
        receipt.reason =
            OfficialGunAttackResponseReason::ResponseContractUnavailable;
        ClearEpisode();
        return;
    }
    if (observation == nullptr || !ValidObservation(frame, *observation))
    {
        receipt.reason =
            OfficialGunAttackResponseReason::ObservationUnavailable;
        ClearEpisode();
        return;
    }

    receipt.observation_consumed = true;
    receipt.observation_form = observation->attack_form;
    if (observation->attack_form == GunAttackForm::Indeterminate)
    {
        receipt.reason = OfficialGunAttackResponseReason::
            IndeterminateFormKeepsBreak;
        ClearEpisode();
        return;
    }

    if (observation->attack_form == GunAttackForm::Tracking)
    {
        if (!activation.tracking_jink_enabled)
        {
            receipt.reason = OfficialGunAttackResponseReason::
                TrackingFormKeepsBreak;
            ClearEpisode();
            return;
        }
        if (previous_form_ != GunAttackForm::Tracking)
        {
            held_jink_direction_valid_ = false;
            held_jink_direction_ned_ = Vector3{};
        }
        ClearSnapshotState();

        Vector3 relative_position{};
        Vector3 los{};
        JinkCandidates unique_candidates{};
        if (!SafeSubtract(
                frame.opponent.position_ned_m,
                frame.own.position_ned_m,
                relative_position)
            || !::UnitVector(relative_position, los)
            || !BuildJinkCandidates(
                frame,
                base_break,
                los,
                unique_candidates))
        {
            receipt.reason = OfficialGunAttackResponseReason::
                TrackingGeometryUnavailable;
            ClearEpisode();
            return;
        }

        Vector3 los_motion_raw{};
        Vector3 los_motion{};
        const bool los_motion_valid = ::SafeCross(
                observation->geometry.los_rate_world_radps,
                los,
                los_motion_raw)
            && ::UnitVector(los_motion_raw, los_motion);

        Vector3 projected_held{};
        const bool projected_held_valid = held_jink_direction_valid_
            && ProjectNormalTo(held_jink_direction_ned_, los, projected_held);
        double visible_projection = 0.0;
        const bool response_visible = projected_held_valid
            && los_motion_valid
            && SafeDot(los_motion, projected_held, visible_projection)
            && visible_projection > 0.0;

        std::uint32_t chosen_index = 0U;
        std::uint32_t eligible_count = 0U;
        if (!projected_held_valid || response_visible)
        {
            std::array<Vector3, 4U> eligible{};
            if (los_motion_valid)
            {
                for (std::uint32_t index = 0U;
                     index < unique_candidates.count;
                     ++index)
                {
                    double relation = 0.0;
                    if (!SafeDot(
                            unique_candidates.value[index],
                            los_motion,
                            relation))
                    {
                        receipt.reason = OfficialGunAttackResponseReason::
                            TrackingGeometryUnavailable;
                        ClearEpisode();
                        return;
                    }
                    if (relation <= 0.0)
                    {
                        eligible[eligible_count] =
                            unique_candidates.value[index];
                        ++eligible_count;
                    }
                }
            }
            if (eligible_count == 0U)
            {
                eligible = unique_candidates.value;
                eligible_count = unique_candidates.count;
            }

            if (held_jink_direction_valid_ && eligible_count > 1U)
            {
                std::array<Vector3, 4U> alternatives{};
                std::uint32_t alternative_count = 0U;
                for (std::uint32_t index = 0U;
                     index < eligible_count;
                     ++index)
                {
                    if (!SameDirection(
                            eligible[index],
                            held_jink_direction_ned_))
                    {
                        alternatives[alternative_count] = eligible[index];
                        ++alternative_count;
                    }
                }
                if (alternative_count > 0U)
                {
                    eligible = alternatives;
                    eligible_count = alternative_count;
                }
            }
            chosen_index = NextBoundedIndex(eligible_count);
            held_jink_direction_ned_ = eligible[chosen_index];
            held_jink_direction_valid_ = true;
        }
        else
        {
            held_jink_direction_ned_ = projected_held;
            held_jink_direction_valid_ = true;
            eligible_count = unique_candidates.count;
        }

        const double range_m = frame.enemy_offense.range_m;
        Vector3 ranged_direction{};
        Vector3 replacement_aim{};
        if (!std::isfinite(range_m)
            || range_m <= 0.0
            || !held_jink_direction_valid_
            || !SafeScale(
                held_jink_direction_ned_,
                range_m,
                ranged_direction)
            || !SafeAdd(
                frame.own.position_ned_m,
                ranged_direction,
                replacement_aim))
        {
            receipt.reason = OfficialGunAttackResponseReason::
                TrackingGeometryUnavailable;
            ClearEpisode();
            output = base_break;
            return;
        }
        output.aim_point_m = replacement_aim;
        Status replacement_status{};
        output.Validate(replacement_status);
        if (!replacement_status.sample_valid())
        {
            receipt.reason = OfficialGunAttackResponseReason::
                TrackingGeometryUnavailable;
            ClearEpisode();
            output = base_break;
            return;
        }

        previous_form_ = GunAttackForm::Tracking;
        receipt.selected_branch =
            OfficialGunAttackResponseBranch::TrackingJink;
        receipt.reason =
            OfficialGunAttackResponseReason::TrackingJinkSelected;
        receipt.base_break_preserved = false;
        receipt.replacement_available = true;
        receipt.replace_aim_point = true;
        receipt.replacement_aim_point_ned_m = replacement_aim;
        receipt.tracking_candidate_count = eligible_count;
        receipt.tracking_selected_index = chosen_index;
        receipt.tracking_direction_held = projected_held_valid
            && !response_visible;
        receipt.tracking_response_visible = response_visible;
        return;
    }

    if (observation->attack_form != GunAttackForm::Snapshot)
    {
        receipt.reason = OfficialGunAttackResponseReason::
            IndeterminateFormKeepsBreak;
        ClearEpisode();
        return;
    }

    if (previous_form_ != GunAttackForm::Snapshot)
    {
        ClearSnapshotState();
    }
    held_jink_direction_valid_ = false;
    held_jink_direction_ned_ = Vector3{};

    if (!load_capability_admitted
        || !std::isfinite(load_capability_g)
        || load_capability_g < 1.0)
    {
        receipt.reason = OfficialGunAttackResponseReason::
            SnapshotCapabilityUnavailable;
        ClearEpisode();
        return;
    }

    Vector3 velocity{};
    Vector3 negative_down{};
    Vector3 actual_lift{};
    const GunAttackOptionalDouble& lift_resolution =
        observation->geometry.own_lift_direction_resolution_rad;
    std::int32_t tie_side = 0;
    if (!::UnitVector(frame.own.velocity_ned_mps, velocity)
        || !SafeScale(frame.own.down_ned, -1.0, negative_down)
        || !ProjectNormalTo(negative_down, velocity, actual_lift)
        || !lift_resolution.has_value
        || !std::isfinite(lift_resolution.value)
        || lift_resolution.value <= 0.0
        || !BaselineTurnSense(frame, base_break, tie_side))
    {
        receipt.reason = OfficialGunAttackResponseReason::
            SnapshotGeometryUnavailable;
        ClearEpisode();
        return;
    }

    OfficialGunSnapshotReference solved_reference{};
    if (!SnapshotWezEscapeReference(
            frame,
            response_horizon_s,
            response_cone_rad,
            load_capability_g,
            tie_side,
            solved_reference))
    {
        receipt.reason = OfficialGunAttackResponseReason::
            SnapshotReferenceUnavailable;
        ClearEpisode();
        return;
    }

    Vector3 held{};
    const bool held_valid = snapshot_target_lift_valid_
        && ProjectNormalTo(snapshot_target_lift_ned_, velocity, held);
    double attitude_error = 0.0;
    const bool attitude_captured = held_valid
        && snapshot_capture_resolution_valid_
        && SignedAngleAboutAxis(
            actual_lift,
            held,
            velocity,
            attitude_error)
        && std::fabs(attitude_error) <= snapshot_capture_resolution_rad_;

    double target_force_bank = 0.0;
    double observed_force_bank = 0.0;
    const bool force_sample_valid = ValidForceFeedback(
        previous_feedback,
        target_force_bank,
        observed_force_bank);
    if (force_sample_valid && !snapshot_force_feedback_primed_)
    {
        snapshot_force_feedback_primed_ = true;
        snapshot_previous_force_valid_ = false;
        snapshot_previous_force_target_bank_rad_ = 0.0;
        snapshot_previous_force_bank_rad_ = 0.0;
    }
    else if (force_sample_valid)
    {
        if (snapshot_previous_force_valid_
            && snapshot_direction_valid_)
        {
            const double previous_relative_force = WrappedDifference(
                snapshot_previous_force_bank_rad_,
                snapshot_previous_force_target_bank_rad_);
            const double current_relative_force = WrappedDifference(
                observed_force_bank,
                target_force_bank);
            const double relative_force_motion = WrappedDifference(
                current_relative_force,
                previous_relative_force);
            const double direction =
                static_cast<double>(snapshot_direction_sign_);
            if (std::isfinite(previous_relative_force)
                && std::isfinite(current_relative_force)
                && std::isfinite(relative_force_motion)
                && direction * previous_relative_force < 0.0
                && direction * current_relative_force >= 0.0
                && direction * relative_force_motion > 0.0)
            {
                snapshot_force_target_crossed_ = true;
            }
        }
        snapshot_previous_force_valid_ = true;
        snapshot_previous_force_target_bank_rad_ = target_force_bank;
        snapshot_previous_force_bank_rad_ = observed_force_bank;
    }
    else
    {
        snapshot_force_feedback_primed_ = false;
        snapshot_previous_force_valid_ = false;
        snapshot_previous_force_target_bank_rad_ = 0.0;
        snapshot_previous_force_bank_rad_ = 0.0;
    }

    const bool target_captured =
        attitude_captured && snapshot_force_target_crossed_;
    const bool line_of_fire_clear =
        solved_reference.coast_predicted_ata_rad >= response_cone_rad;
    bool reversal_admitted = false;

    if (held_valid && !target_captured)
    {
        snapshot_target_lift_ned_ = held;
        snapshot_target_lift_valid_ = true;
    }
    else
    {
        const std::int32_t proposed_direction = solved_reference.side_sign;
        const bool direction_change_admitted =
            !snapshot_direction_valid_
            || proposed_direction == snapshot_direction_sign_
            || line_of_fire_clear;
        if (held_valid && !direction_change_admitted)
        {
            snapshot_target_lift_ned_ = held;
            snapshot_target_lift_valid_ = true;
        }
        else
        {
            reversal_admitted = held_valid
                && snapshot_direction_valid_
                && proposed_direction == -snapshot_direction_sign_
                && direction_change_admitted;
            double target_change = 0.0;
            const bool same_target_epoch = held_valid
                && snapshot_direction_valid_
                && proposed_direction == snapshot_direction_sign_
                && SignedAngleAboutAxis(
                    held,
                    solved_reference.target_lift_direction_ned,
                    velocity,
                    target_change)
                && std::fabs(target_change) <= lift_resolution.value;
            if (same_target_epoch)
            {
                snapshot_reference_ = solved_reference;
                snapshot_reference_valid_ = true;
                snapshot_target_lift_ned_ = held;
                snapshot_target_lift_valid_ = true;
            }
            else
            {
                snapshot_target_lift_ned_ =
                    solved_reference.target_lift_direction_ned;
                snapshot_target_lift_valid_ = true;
                snapshot_reference_ = solved_reference;
                snapshot_reference_valid_ = true;
                snapshot_direction_sign_ = proposed_direction;
                snapshot_direction_valid_ = true;
                snapshot_capture_resolution_rad_ = lift_resolution.value;
                snapshot_capture_resolution_valid_ = true;
                snapshot_force_feedback_primed_ = false;
                snapshot_previous_force_valid_ = false;
                snapshot_previous_force_target_bank_rad_ = 0.0;
                snapshot_previous_force_bank_rad_ = 0.0;
                snapshot_force_target_crossed_ = false;
            }
        }
    }

    if (!snapshot_target_lift_valid_ || !snapshot_reference_valid_)
    {
        receipt.reason = OfficialGunAttackResponseReason::
            SnapshotReferenceUnavailable;
        ClearEpisode();
        return;
    }

    previous_form_ = GunAttackForm::Snapshot;
    receipt.selected_branch =
        OfficialGunAttackResponseBranch::SnapshotPlaneChange;
    receipt.reason =
        OfficialGunAttackResponseReason::SnapshotPlaneChangeSelected;
    receipt.base_break_preserved = true;
    receipt.replacement_available = true;
    receipt.snapshot_reference = snapshot_reference_;
    receipt.snapshot_reference.target_lift_direction_ned =
        snapshot_target_lift_ned_;
    receipt.snapshot_force_feedback_primed =
        snapshot_force_feedback_primed_;
    receipt.snapshot_force_target_crossed =
        snapshot_force_target_crossed_;
    receipt.snapshot_reversal_admitted_this_tick = reversal_admitted;
}

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
