#include "LadyLuck/geometry/WezGeometry.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>

namespace
{
bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

LadyLuck::Vector3 Add(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

LadyLuck::Vector3 Subtract(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

LadyLuck::Vector3 Scale(
    const LadyLuck::Vector3& vector,
    const double scalar) noexcept
{
    return LadyLuck::Vector3{{
        vector[0] * scalar,
        vector[1] * scalar,
        vector[2] * scalar}};
}

bool FiniteGeometry(const LadyLuck::WezGeometry& geometry) noexcept
{
    return std::isfinite(geometry.t_sec)
        && std::isfinite(geometry.tau_sec)
        && FiniteVector(geometry.attacker_pos_m)
        && FiniteVector(geometry.attacker_vel_ned_mps)
        && FiniteVector(geometry.attacker_nose_ned)
        && FiniteVector(geometry.target_pos_m)
        && FiniteVector(geometry.target_vel_ned_mps)
        && FiniteVector(geometry.target_nose_ned)
        && FiniteVector(geometry.target_pred_pos_m)
        && FiniteVector(geometry.target_pred_vel_ned_mps)
        && FiniteVector(geometry.attacker_pred_pos_m)
        && FiniteVector(geometry.capture_point_m)
        && std::isfinite(geometry.capture_range_des_m)
        && FiniteVector(geometry.los_hat_ned)
        && std::isfinite(geometry.range_m)
        && std::isfinite(geometry.ata_rad)
        && std::isfinite(geometry.aspect_rad)
        && std::isfinite(geometry.hca_rad)
        && std::isfinite(geometry.damage_rate)
        && std::isfinite(geometry.soft_potential);
}
}

namespace LadyLuck
{
Result<Vector3> UnitVector(
    const Vector3& vector,
    const Vector3& fallback) noexcept
{
    Result<Vector3> result{};
    if (!FiniteVector(vector) || !FiniteVector(fallback))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const double norm = VectorNorm(vector);
    if (!std::isfinite(norm))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    if (norm < constants::Tiny)
    {
        result.value = fallback;
        return result;
    }
    result.value = Scale(vector, 1.0 / norm);
    return result;
}

Result<double> AngleBetween(
    const Vector3& left,
    const Vector3& right) noexcept
{
    Result<double> result{};
    const Result<Vector3> left_hat = UnitVector(left);
    if (!left_hat.ok())
    {
        result.status = left_hat.status;
        return result;
    }
    const Result<Vector3> right_hat = UnitVector(right, left_hat.value);
    if (!right_hat.ok())
    {
        result.status = right_hat.status;
        return result;
    }
    result.value = std::acos(std::max(
        -1.0,
        std::min(1.0, Dot(left_hat.value, right_hat.value))));
    if (!std::isfinite(result.value))
    {
        result.status.code = StatusCode::NonFiniteInput;
    }
    return result;
}

Result<WezLimits> ActiveWezLimits(const double t_sec) noexcept
{
    Result<WezLimits> result{};
    const Result<WezPhase> phase = ActiveWezPhase(t_sec);
    if (!phase.ok())
    {
        result.status = phase.status;
        return result;
    }
    result.value.angle_rad = phase.value.angle_rad;
    result.value.min_range_m = phase.value.min_range_m;
    result.value.max_range_m = phase.value.max_range_m;
    return result;
}

Result<WezPhaseMatch> MatchWezPhase(
    const double range_m,
    const double ata_rad,
    const double t_sec) noexcept
{
    Result<WezPhaseMatch> result{};
    if (!std::isfinite(range_m)
        || !std::isfinite(ata_rad)
        || !std::isfinite(t_sec))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    const double ata_deg = std::fabs(ata_rad) * RadiansToDegrees;
    for (std::size_t index = 0U; index < OfficialWezPhaseCount; ++index)
    {
        const Result<WezPhase> phase = OfficialWezPhaseAt(index);
        if (!phase.ok())
        {
            result.status = phase.status;
            return result;
        }
        if (t_sec >= phase.value.start_sec
            && ata_deg < phase.value.angle_deg
            && range_m >= phase.value.min_range_m
            && range_m <= phase.value.max_range_m)
        {
            const double span = std::max(
                constants::Epsilon,
                phase.value.max_range_m - phase.value.min_range_m);
            const double falloff = std::max(
                0.0,
                std::min(
                    1.0,
                    (phase.value.max_range_m - range_m) / span));
            result.value.matched = true;
            result.value.phase = phase.value;
            result.value.damage_rate = phase.value.coeff * falloff;
            return result;
        }
    }

    const Result<WezPhase> active = ActiveWezPhase(t_sec);
    if (!active.ok())
    {
        result.status = active.status;
        return result;
    }
    result.value.phase = active.value;
    return result;
}

Result<WezDamageScores> ComputeWezDamageScores(
    const double los_deg,
    const double range_m,
    const double t_sec,
    const double soft_sigma_deg) noexcept
{
    Result<WezDamageScores> result{};
    const Result<double> hard = OfficialDamageCoeffMeters(
        los_deg,
        range_m,
        t_sec);
    if (!hard.ok())
    {
        result.status = hard.status;
        return result;
    }
    const Result<double> soft = SoftOffensePotential(
        los_deg,
        range_m,
        t_sec,
        soft_sigma_deg);
    if (!soft.ok())
    {
        result.status = soft.status;
        return result;
    }
    result.value.hard = hard.value;
    result.value.soft = soft.value;
    return result;
}

Result<WezGeometry> BuildWezGeometry(
    const WezSnapshotInput& input) noexcept
{
    Result<WezGeometry> result{};
    const bool target_nose_finite = !input.has_target_nose
        || FiniteVector(input.target_nose_ned);
    const bool target_accel_finite = !input.has_target_accel
        || FiniteVector(input.target_accel_ned_mps2);
    if (!FiniteVector(input.attacker_pos_m)
        || !FiniteVector(input.attacker_vel_ned_mps)
        || !FiniteVector(input.attacker_nose_ned)
        || !FiniteVector(input.target_pos_m)
        || !FiniteVector(input.target_vel_ned_mps)
        || !target_nose_finite
        || !target_accel_finite
        || !std::isfinite(input.t_sec)
        || !std::isfinite(input.tau_sec)
        || !std::isfinite(input.capture_range_des_m)
        || !std::isfinite(input.soft_sigma_deg))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }

    const Vector3 zero{{0.0, 0.0, 0.0}};
    const Vector3 x_axis{{1.0, 0.0, 0.0}};
    const Vector3 target_accel = input.has_target_accel
        ? input.target_accel_ned_mps2
        : zero;

    const Result<Vector3> attacker_nose = UnitVector(
        input.attacker_nose_ned,
        input.attacker_vel_ned_mps);
    if (!attacker_nose.ok())
    {
        result.status = attacker_nose.status;
        return result;
    }
    const Vector3 target_nose_source = input.has_target_nose
        ? input.target_nose_ned
        : input.target_vel_ned_mps;
    const Result<Vector3> target_nose = UnitVector(
        target_nose_source,
        input.target_vel_ned_mps);
    if (!target_nose.ok())
    {
        result.status = target_nose.status;
        return result;
    }

    // Preserve NumPy's left-to-right operation order in
    // p + v*t + 0.5*a*t*t rather than algebraically folding t*t.
    const Vector3 acceleration_displacement = Scale(
        Scale(
            Scale(target_accel, 0.5),
            input.tau_sec),
        input.tau_sec);
    const Vector3 target_pred_pos = Add(
        Add(
            input.target_pos_m,
            Scale(input.target_vel_ned_mps, input.tau_sec)),
        acceleration_displacement);
    const Vector3 target_pred_vel = Add(
        input.target_vel_ned_mps,
        Scale(target_accel, input.tau_sec));
    const Vector3 attacker_pred_pos = Add(
        input.attacker_pos_m,
        Scale(input.attacker_vel_ned_mps, input.tau_sec));

    const Result<Vector3> normalized_target_fallback = UnitVector(
        input.target_vel_ned_mps,
        x_axis);
    if (!normalized_target_fallback.ok())
    {
        result.status = normalized_target_fallback.status;
        return result;
    }
    const Result<Vector3> target_velocity_hat = UnitVector(
        target_pred_vel,
        normalized_target_fallback.value);
    if (!target_velocity_hat.ok())
    {
        result.status = target_velocity_hat.status;
        return result;
    }
    const Vector3 capture_point = Subtract(
        target_pred_pos,
        Scale(target_velocity_hat.value, input.capture_range_des_m));

    const Vector3 rho = Subtract(target_pred_pos, attacker_pred_pos);
    const double range_m = std::max(VectorNorm(rho), constants::Tiny);
    if (!std::isfinite(range_m))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    const Vector3 los_hat = Scale(rho, 1.0 / range_m);

    const Result<double> ata = AngleBetween(attacker_nose.value, los_hat);
    const Result<double> aspect = AngleBetween(target_nose.value, los_hat);
    if (!ata.ok() || !aspect.ok())
    {
        result.status = !ata.ok() ? ata.status : aspect.status;
        return result;
    }
    const Result<Vector3> attacker_velocity_hat = UnitVector(
        input.attacker_vel_ned_mps,
        attacker_nose.value);
    const Result<Vector3> target_pred_velocity_hat = UnitVector(
        target_pred_vel,
        target_nose.value);
    if (!attacker_velocity_hat.ok() || !target_pred_velocity_hat.ok())
    {
        result.status = !attacker_velocity_hat.ok()
            ? attacker_velocity_hat.status
            : target_pred_velocity_hat.status;
        return result;
    }
    const Result<double> hca = AngleBetween(
        attacker_velocity_hat.value,
        target_pred_velocity_hat.value);
    if (!hca.ok())
    {
        result.status = hca.status;
        return result;
    }

    const double t_future = input.t_sec + input.tau_sec;
    const Result<WezPhase> phase = ActiveWezPhase(t_future);
    const Result<WezDamageScores> damage = ComputeWezDamageScores(
        ata.value * RadiansToDegrees,
        range_m,
        t_future,
        input.soft_sigma_deg);
    if (!phase.ok() || !damage.ok())
    {
        result.status = !phase.ok() ? phase.status : damage.status;
        return result;
    }

    WezGeometry& output = result.value;
    output.phase = phase.value;
    output.t_sec = t_future;
    output.tau_sec = input.tau_sec;
    output.attacker_pos_m = input.attacker_pos_m;
    output.attacker_vel_ned_mps = input.attacker_vel_ned_mps;
    output.attacker_nose_ned = attacker_nose.value;
    output.target_pos_m = input.target_pos_m;
    output.target_vel_ned_mps = input.target_vel_ned_mps;
    output.target_nose_ned = target_nose.value;
    output.target_pred_pos_m = target_pred_pos;
    output.target_pred_vel_ned_mps = target_pred_vel;
    output.attacker_pred_pos_m = attacker_pred_pos;
    output.capture_point_m = capture_point;
    output.capture_range_des_m = input.capture_range_des_m;
    output.los_hat_ned = los_hat;
    output.range_m = range_m;
    output.ata_rad = ata.value;
    output.aspect_rad = aspect.value;
    output.hca_rad = hca.value;
    output.damage_rate = damage.value.hard;
    output.soft_potential = damage.value.soft;
    if (!FiniteGeometry(output))
    {
        result.status.code = StatusCode::NonFiniteInput;
        result.value = WezGeometry{};
    }
    return result;
}
}
