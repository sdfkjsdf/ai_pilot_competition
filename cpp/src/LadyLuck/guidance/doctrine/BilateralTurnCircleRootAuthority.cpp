#include "LadyLuck/guidance/doctrine/BilateralTurnCircleRootAuthority.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{
namespace
{

struct FixedTurnCircleCapabilitySupplier
{
    const ManualTurnCircleCapabilityReceipt* receipt = nullptr;

    void operator()(
        double,
        double,
        ManualTurnCircleCapabilityReceipt& supplied,
        Status& status) const noexcept
    {
        status = Status{};
        if (receipt == nullptr)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        supplied = *receipt;
    }
};

struct ManualTurnCircleCapabilitySupplier
{
    const ManualTurnCircleCapabilityProvider* provider = nullptr;

    void operator()(
        const double speed_mps,
        const double altitude_m,
        ManualTurnCircleCapabilityReceipt& supplied,
        Status& status) const noexcept
    {
        status = Status{};
        if (provider == nullptr)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        provider->Observe(speed_mps, altitude_m, supplied, status);
    }
};

double Dot(const Vector3& lhs, const Vector3& rhs) noexcept
{
    return lhs[0] * rhs[0]
        + lhs[1] * rhs[1]
        + lhs[2] * rhs[2];
}

Vector3 Subtract(const Vector3& lhs, const Vector3& rhs) noexcept
{
    return Vector3{{
        lhs[0] - rhs[0],
        lhs[1] - rhs[1],
        lhs[2] - rhs[2]}};
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
}

double Norm(const Vector3& value) noexcept
{
    return std::sqrt(Dot(value, value));
}

double EvaluateQuarticUnitInterval(
    const std::array<double, 5U>& polynomial,
    const double x) noexcept
{
    return polynomial[0]
        + x * (polynomial[1]
            + x * (polynomial[2]
                + x * (polynomial[3] + x * polynomial[4])));
}

bool Finite(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double ClipUnit(const double value) noexcept
{
    return (std::max)(-1.0, (std::min)(1.0, value));
}

void AppendRoot(
    std::array<double, 3U>& roots,
    std::size_t& count,
    const double root) noexcept
{
    if (count < roots.size() && std::isfinite(root))
    {
        roots[count] = root;
        ++count;
    }
}

void RealRootsAtMostCubic(
    const std::array<double, 4U>& coefficients,
    std::array<double, 3U>& roots,
    std::size_t& count) noexcept
{
    roots = std::array<double, 3U>{{0.0, 0.0, 0.0}};
    count = 0U;
    const double epsilon = std::numeric_limits<double>::epsilon();
    if (coefficients[3] == 0.0)
    {
        if (coefficients[2] == 0.0)
        {
            if (coefficients[1] != 0.0)
            {
                AppendRoot(
                    roots,
                    count,
                    -coefficients[0] / coefficients[1]);
            }
            return;
        }
        const double discriminant = coefficients[1] * coefficients[1]
            - 4.0 * coefficients[2] * coefficients[0];
        const double discriminant_scale = (std::max)(
            1.0,
            (std::max)(
                std::abs(coefficients[1] * coefficients[1]),
                std::abs(4.0 * coefficients[2] * coefficients[0])));
        const double tolerance = 256.0 * epsilon * discriminant_scale;
        if (discriminant < -tolerance)
        {
            return;
        }
        const double square_root = std::sqrt((std::max)(0.0, discriminant));
        const double denominator = 2.0 * coefficients[2];
        AppendRoot(
            roots,
            count,
            (-coefficients[1] - square_root) / denominator);
        if (square_root != 0.0)
        {
            AppendRoot(
                roots,
                count,
                (-coefficients[1] + square_root) / denominator);
        }
        return;
    }

    // x^3 + a*x^2 + b*x + c = 0, followed by x = y - a/3.
    // This is an allocation-free real-root equivalent of numpy.polyroots for
    // the derivative polynomial used by the d90 monotone-interval search.
    const double a = coefficients[2] / coefficients[3];
    const double b = coefficients[1] / coefficients[3];
    const double c = coefficients[0] / coefficients[3];
    const double p = b - a * a / 3.0;
    const double q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;
    const double half_q = 0.5 * q;
    const double third_p = p / 3.0;
    const double discriminant = half_q * half_q
        + third_p * third_p * third_p;
    const double discriminant_scale = (std::max)(
        1.0,
        (std::max)(
            std::abs(half_q * half_q),
            std::abs(third_p * third_p * third_p)));
    // d90 admits a polyroots result as real when its imaginary part is no
    // larger than 256 eps times its real-root scale.
    const double tolerance = 256.0 * epsilon * discriminant_scale;
    const double offset = a / 3.0;
    if (discriminant > tolerance)
    {
        const double square_root = std::sqrt(discriminant);
        const double u = std::cbrt(-half_q + square_root);
        const double v = std::cbrt(-half_q - square_root);
        AppendRoot(roots, count, u + v - offset);
        return;
    }
    if (discriminant >= -tolerance)
    {
        const double u = std::cbrt(-half_q);
        AppendRoot(roots, count, 2.0 * u - offset);
        if (u != 0.0)
        {
            AppendRoot(roots, count, -u - offset);
        }
        return;
    }

    const double radial_square = -third_p;
    if (!(radial_square > 0.0))
    {
        AppendRoot(roots, count, -offset);
        return;
    }
    const double radial = std::sqrt(radial_square);
    const double denominator = radial * radial * radial;
    const double angle = std::acos(ClipUnit(-half_q / denominator));
    const double magnitude = 2.0 * radial;
    AppendRoot(roots, count, magnitude * std::cos(angle / 3.0) - offset);
    AppendRoot(
        roots,
        count,
        magnitude * std::cos(
            (angle + 2.0 * constants::Pi) / 3.0) - offset);
    AppendRoot(
        roots,
        count,
        magnitude * std::cos(
            (angle + 4.0 * constants::Pi) / 3.0) - offset);
}

double TurnCircleTubeMarginM(
    const Vector3& relative_position_m,
    const Vector3& defender_velocity_mps,
    const double turn_radius_m) noexcept
{
    const double defender_speed_mps = Norm(defender_velocity_mps);
    const Vector3 tangent = Scale(
        defender_velocity_mps,
        1.0 / defender_speed_mps);
    const double along_m = Dot(relative_position_m, tangent);
    const Vector3 transverse_m = Subtract(
        relative_position_m,
        Scale(tangent, along_m));
    const double cross_track_m = Norm(transverse_m);
    return std::hypot(along_m, cross_track_m - turn_radius_m)
        - turn_radius_m;
}

bool FirstConstantVelocityTurnCircleReachTimeS(
    const Vector3& relative_position_m,
    const Vector3& attacker_velocity_mps,
    const Vector3& defender_velocity_mps,
    const double turn_radius_m,
    const double max_horizon_s,
    double& reach_time_s) noexcept
{
    reach_time_s = 0.0;
    const double defender_speed_mps = Norm(defender_velocity_mps);
    if (TurnCircleTubeMarginM(
            relative_position_m,
            defender_velocity_mps,
            turn_radius_m) <= 0.0)
    {
        return true;
    }
    if (max_horizon_s == 0.0 || Norm(attacker_velocity_mps) == 0.0)
    {
        return false;
    }

    const Vector3 tangent = Scale(
        defender_velocity_mps,
        1.0 / defender_speed_mps);
    const double initial_along_m = Dot(relative_position_m, tangent);
    const double velocity_along_mps = Dot(attacker_velocity_mps, tangent);
    const Vector3 initial_transverse_m = Subtract(
        relative_position_m,
        Scale(tangent, initial_along_m));
    const Vector3 velocity_transverse_mps = Subtract(
        attacker_velocity_mps,
        Scale(tangent, velocity_along_mps));
    const double axial_scale_m = (std::max)(
        1.0,
        (std::max)(
            turn_radius_m,
            (std::max)(
                Norm(relative_position_m),
                Norm(attacker_velocity_mps) * max_horizon_s)));
    const double axial_tolerance_m = 64.0
        * std::numeric_limits<double>::epsilon()
        * axial_scale_m;
    if (Norm(initial_transverse_m) <= axial_tolerance_m
        && Norm(velocity_transverse_mps) <= axial_tolerance_m
        && velocity_along_mps != 0.0)
    {
        const double axial_reach_s =
            -initial_along_m / velocity_along_mps;
        if (axial_reach_s >= 0.0 && axial_reach_s <= max_horizon_s)
        {
            reach_time_s = axial_reach_s;
            return true;
        }
        return false;
    }

    const double length_scale_m = (std::max)(
        1.0,
        (std::max)(
            turn_radius_m,
            (std::max)(
                Norm(relative_position_m),
                Norm(attacker_velocity_mps) * max_horizon_s)));
    const Vector3 r0 = Scale(relative_position_m, 1.0 / length_scale_m);
    const Vector3 rv = Scale(
        attacker_velocity_mps,
        max_horizon_s / length_scale_m);
    const double scaled_radius_m = turn_radius_m / length_scale_m;
    const double s0 = Dot(r0, tangent);
    const double sv = Dot(rv, tangent);
    const Vector3 u0 = Subtract(r0, Scale(tangent, s0));
    const Vector3 uv = Subtract(rv, Scale(tangent, sv));
    const double q0 = Dot(r0, r0);
    const double q1 = 2.0 * Dot(r0, rv);
    const double q2 = Dot(rv, rv);
    const double p0 = Dot(u0, u0);
    const double p1 = 2.0 * Dot(u0, uv);
    const double p2 = Dot(uv, uv);
    const double radius_squared = scaled_radius_m * scaled_radius_m;
    std::array<double, 5U> polynomial{{
        q0 * q0 - 4.0 * radius_squared * p0,
        2.0 * q0 * q1 - 4.0 * radius_squared * p1,
        q1 * q1 + 2.0 * q0 * q2 - 4.0 * radius_squared * p2,
        2.0 * q1 * q2,
        q2 * q2}};
    double coefficient_scale = 1.0;
    for (const double value : polynomial)
    {
        coefficient_scale = (std::max)(coefficient_scale, std::abs(value));
    }
    for (double& value : polynomial)
    {
        value /= coefficient_scale;
    }

    const std::array<double, 4U> derivative{{
        polynomial[1],
        2.0 * polynomial[2],
        3.0 * polynomial[3],
        4.0 * polynomial[4]}};
    std::array<double, 3U> roots{};
    std::size_t root_count = 0U;
    RealRootsAtMostCubic(derivative, roots, root_count);
    std::sort(roots.begin(), roots.begin() + root_count);

    std::array<double, 5U> boundaries{{0.0, 0.0, 0.0, 0.0, 0.0}};
    std::size_t boundary_count = 1U;
    double previous_stationary = 0.0;
    bool have_previous_stationary = false;
    for (std::size_t index = 0U; index < root_count; ++index)
    {
        const double root = roots[index];
        if (root > 0.0 && root <= 1.0
            && (!have_previous_stationary || root != previous_stationary))
        {
            boundaries[boundary_count] = root;
            ++boundary_count;
            previous_stationary = root;
            have_previous_stationary = true;
        }
    }
    if (boundary_count == 1U || boundaries[boundary_count - 1U] < 1.0)
    {
        boundaries[boundary_count] = 1.0;
        ++boundary_count;
    }

    double sum_absolute_coefficients = 0.0;
    for (const double value : polynomial)
    {
        sum_absolute_coefficients += std::abs(value);
    }
    const double value_tolerance = 512.0
        * std::numeric_limits<double>::epsilon()
        * (std::max)(1.0, sum_absolute_coefficients);
    double previous_x = boundaries[0];
    double previous_value = EvaluateQuarticUnitInterval(
        polynomial,
        previous_x);
    for (std::size_t index = 1U; index < boundary_count; ++index)
    {
        const double current_x = boundaries[index];
        const double current_value = EvaluateQuarticUnitInterval(
            polynomial,
            current_x);
        if (current_value <= value_tolerance)
        {
            if (current_value >= -value_tolerance)
            {
                reach_time_s = current_x * max_horizon_s;
                return true;
            }
            double lower = previous_x;
            double upper = current_x;
            if (previous_value <= value_tolerance)
            {
                reach_time_s = lower * max_horizon_s;
                return true;
            }
            for (std::size_t iteration = 0U; iteration < 64U; ++iteration)
            {
                const double midpoint = 0.5 * (lower + upper);
                if (EvaluateQuarticUnitInterval(polynomial, midpoint) > 0.0)
                {
                    lower = midpoint;
                }
                else
                {
                    upper = midpoint;
                }
            }
            reach_time_s = upper * max_horizon_s;
            return true;
        }
        previous_x = current_x;
        previous_value = current_value;
    }
    return false;
}

void ValidateSuppliedCapability(
    const ManualTurnCircleCapabilityReceipt& capability,
    Status& status) noexcept
{
    status = Status{};
    if (capability.capability_g.has_value
        && !std::isfinite(capability.capability_g.value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (capability.admitted && !capability.capability_g.has_value)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (capability.physical_authority && capability.fixed_command_bound)
    {
        status.code = StatusCode::InvalidConfiguration;
    }
}

template <typename CapabilitySupplier>
void EvaluateDirectional(
    const Vector3& attacker_position_m,
    const Vector3& attacker_velocity_mps,
    const Vector3& defender_position_m,
    const Vector3& defender_velocity_mps,
    const double defender_altitude_m,
    const double attacker_ata_rad,
    CapabilitySupplier capability_supplier,
    DirectionalDoctrineTurnCircleReceipt& output,
    Status& status) noexcept
{
    output = DirectionalDoctrineTurnCircleReceipt{};
    status = Status{};
    if (!Finite(attacker_position_m)
        || !Finite(attacker_velocity_mps)
        || !Finite(defender_position_m)
        || !Finite(defender_velocity_mps)
        || !std::isfinite(defender_altitude_m)
        || !std::isfinite(attacker_ata_rad))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const Vector3 relative_position_m = Subtract(
        attacker_position_m,
        defender_position_m);
    const double defender_speed_mps = Norm(defender_velocity_mps);
    const double separation_m = Norm(relative_position_m);
    output.attacker_rear_halfspace =
        Dot(relative_position_m, defender_velocity_mps) < 0.0;
    // ATA is already produced by the same-frame WEZ geometry.  The strict
    // forward half-space is geometric, not a new tactical cone or WEZ gate.
    output.attacker_nose_toward =
        std::fabs(attacker_ata_rad) < 0.5 * constants::Pi;
    if (defender_speed_mps <= 0.0 || separation_m <= 0.0)
    {
        output.reason = DirectionalTurnCircleReason::
            TurnCircleKinematicsUnavailable;
        return;
    }

    capability_supplier(
        defender_speed_mps,
        defender_altitude_m,
        output.capability,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    ValidateSuppliedCapability(output.capability, status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!output.capability.admitted
        || !output.capability.capability_g.has_value
        || output.capability.capability_g.value <= 1.0)
    {
        output.capability.admitted = false;
        output.reason = DirectionalTurnCircleReason::CapabilityNotAdmitted;
        return;
    }

    const double capability_g = output.capability.capability_g.value;
    const double turn_denominator = constants::StandardGravityMps2
        * std::sqrt(capability_g * capability_g - 1.0);
    const double turn_radius_m = defender_speed_mps * defender_speed_mps
        / turn_denominator;
    const double maximum_turn_rate_rad_s =
        defender_speed_mps / turn_radius_m;
    const double cosine = Dot(
        Scale(defender_velocity_mps, 1.0 / defender_speed_mps),
        Scale(relative_position_m, 1.0 / separation_m));
    const double initial_face_angle_rad = std::acos(ClipUnit(cosine));
    const double face_time_s = initial_face_angle_rad
        / maximum_turn_rate_rad_s;
    if (!std::isfinite(turn_radius_m)
        || turn_radius_m <= 0.0
        || !std::isfinite(maximum_turn_rate_rad_s)
        || maximum_turn_rate_rad_s <= 0.0
        || !std::isfinite(face_time_s)
        || face_time_s < 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    double reach_time_s = 0.0;
    const bool reach_available = FirstConstantVelocityTurnCircleReachTimeS(
        relative_position_m,
        attacker_velocity_mps,
        defender_velocity_mps,
        turn_radius_m,
        face_time_s,
        reach_time_s);
    DoctrineTurnCircleEvents events{};
    events.valid = true;
    events.face_time_s = OptionalTurnCircleScalar{true, face_time_s};
    events.turn_radius_m = turn_radius_m;
    events.maximum_turn_rate_rad_s = maximum_turn_rate_rad_s;
    events.initial_face_angle_rad = initial_face_angle_rad;
    if (reach_available)
    {
        events.reach_time_s = OptionalTurnCircleScalar{true, reach_time_s};
    }
    else
    {
        events.reach_not_observed_through_s =
            OptionalTurnCircleScalar{true, face_time_s};
    }
    events.event_order = reach_available && reach_time_s < face_time_s
        ? TurnCircleEventOrder::ReachFirst
        : TurnCircleEventOrder::FaceFirstOrEqual;
    output.events_available = true;
    output.events = events;
    output.reason = events.event_order == TurnCircleEventOrder::ReachFirst
        ? DirectionalTurnCircleReason::ReachFirst
        : DirectionalTurnCircleReason::FaceFirstOrEqual;
}

template <typename OwnCapabilitySupplier, typename OpponentCapabilitySupplier>
void EvaluateBilateral(
    const DogfightGeometryFrame& frame,
    OwnCapabilitySupplier own_capability_supplier,
    OpponentCapabilitySupplier opponent_capability_supplier,
    const bool production_authority,
    BilateralDoctrineTurnCircleReceipt& output,
    Status& status) noexcept
{
    output = BilateralDoctrineTurnCircleReceipt{};
    status = Status{};
    // own_attack means ownship attacking an opponent defender, hence the
    // opponent's capability supplier is consumed first, exactly as in d90.
    EvaluateDirectional(
        frame.own.position_ned_m,
        frame.own.velocity_ned_mps,
        frame.opponent.position_ned_m,
        frame.opponent.velocity_ned_mps,
        -frame.opponent.position_ned_m[2],
        frame.own_offense.ata_rad,
        opponent_capability_supplier,
        output.own_attack,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    EvaluateDirectional(
        frame.opponent.position_ned_m,
        frame.opponent.velocity_ned_mps,
        frame.own.position_ned_m,
        frame.own.velocity_ned_mps,
        -frame.own.position_ned_m[2],
        frame.enemy_offense.ata_rad,
        own_capability_supplier,
        output.opponent_attack,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    const bool own_reach_proven = output.own_attack.events_available
        && output.own_attack.events.event_order
            == TurnCircleEventOrder::ReachFirst;
    const bool opponent_reach_proven = output.opponent_attack.events_available
        && output.opponent_attack.events.event_order
            == TurnCircleEventOrder::ReachFirst;
    const bool own_proven = own_reach_proven
        && output.own_attack.attacker_rear_halfspace
        && output.own_attack.attacker_nose_toward;
    const bool opponent_proven = opponent_reach_proven
        && output.opponent_attack.attacker_rear_halfspace
        && output.opponent_attack.attacker_nose_toward;
    const bool own_ruled_out = output.own_attack.events_available
        && output.own_attack.events.event_order
            == TurnCircleEventOrder::FaceFirstOrEqual;
    const bool opponent_ruled_out = output.opponent_attack.events_available
        && output.opponent_attack.events.event_order
            == TurnCircleEventOrder::FaceFirstOrEqual;
    if (own_proven && !opponent_proven)
    {
        output.candidate_mode = TacticalMode::Obfm;
        output.dominance_status =
            BilateralTurnCircleDominanceStatus::OwnAdvantageProven;
    }
    else if (opponent_proven && !own_proven)
    {
        output.candidate_mode = TacticalMode::Dbfm;
        output.dominance_status =
            BilateralTurnCircleDominanceStatus::AdversaryAdvantageProven;
    }
    else if (own_ruled_out && opponent_ruled_out)
    {
        output.candidate_mode = TacticalMode::Habfm;
        output.dominance_status =
            BilateralTurnCircleDominanceStatus::NeutralProven;
    }
    else
    {
        output.candidate_mode = TacticalMode::Habfm;
        output.dominance_status =
            BilateralTurnCircleDominanceStatus::HabfmFallback;
    }
    output.valid = true;
    output.evaluated = true;
    output.tactical_mode_authority = production_authority;
    output.production_authority = production_authority;
}

} // namespace

void ManualTurnCircleCapabilityProvider::Observe(
    const double speed_mps,
    const double altitude_m,
    ManualTurnCircleCapabilityReceipt& output,
    Status& status) const noexcept
{
    output = ManualTurnCircleCapabilityReceipt{};
    status = Status{};
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (speed_mps <= 0.0)
    {
        // A finite pre-start/low-information sample is a normal tactical
        // non-admission.  The bilateral classifier maps it to HABFM.
        return;
    }

    envelope_.ObserveCharacterizedRawN(
        speed_mps,
        altitude_m,
        false,
        output.raw_lookup);
    if (!output.raw_lookup.lookup_valid())
    {
        // Missing physical E-M authority withholds directional dominance.
        // The Root classifier is nevertheless total and maps this case to
        // HABFM; command-containment bounds belong only to the downstream FCS.
        return;
    }
    const double raw_load_g = output.raw_lookup.load_factor_g.value;
    output.capability_g = OptionalTurnCircleScalar{true, raw_load_g};
    if (output.raw_lookup.trusted && raw_load_g > 1.0)
    {
        output.capability_g.value = (std::min)(
            TurnCircleMaximumCommandLoadFactorG,
            TurnCirclePositiveCapabilityScale * raw_load_g);
        output.admitted = true;
        output.physical_authority = true;
        return;
    }
}

void EvaluateBilateralManualTurnCircleFromCapabilities(
    const DogfightGeometryFrame& frame,
    const ManualTurnCircleCapabilityReceipt& own_capability,
    const ManualTurnCircleCapabilityReceipt& opponent_capability,
    const bool production_authority,
    BilateralDoctrineTurnCircleReceipt& output,
    Status& status) noexcept
{
    FixedTurnCircleCapabilitySupplier own_supplier{};
    own_supplier.receipt = &own_capability;
    FixedTurnCircleCapabilitySupplier opponent_supplier{};
    opponent_supplier.receipt = &opponent_capability;
    EvaluateBilateral(
        frame,
        own_supplier,
        opponent_supplier,
        production_authority,
        output,
        status);
}

void BilateralTurnCircleRootAuthority::Observe(
    const DogfightGeometryFrame& frame,
    BilateralDoctrineTurnCircleReceipt& output,
    Status& status) const noexcept
{
    ManualTurnCircleCapabilitySupplier supplier{};
    supplier.provider = &capability_provider_;
    EvaluateBilateral(
        frame,
        supplier,
        supplier,
        true,
        output,
        status);
}

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
