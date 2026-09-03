#include "LadyLuck/guidance/obfm/ObfmLongitudinalReferenceProvider.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

double Norm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(Dot(value, value));
}

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
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
    const LadyLuck::Vector3& value,
    const double scalar) noexcept
{
    return LadyLuck::Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
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

void Reject(
    LadyLuck::ObfmLongitudinalProviderReceipt& output,
    const LadyLuck::ObfmLongitudinalProviderStatus reason) noexcept
{
    output.status = reason;
    output.reference.admitted = false;
    output.reference.desired_speed_mps =
        LadyLuck::IntentOptionalValue<double>{};
    output.reference.desired_speed_rate_mps2 =
        LadyLuck::IntentOptionalValue<double>{};
}

} // namespace

namespace LadyLuck
{

void ShapeObfmBumplessSpeedReference(
    const ObfmBumplessSpeedInput& input,
    ObfmBumplessSpeedReceipt& output,
    Status& status) noexcept
{
    output = ObfmBumplessSpeedReceipt{};
    status = Status{};
    output.evaluated = true;
    output.frame_identity = input.frame_identity;
    output.raw_desired_speed_mps =
        std::isfinite(input.raw_desired_speed_mps)
        ? input.raw_desired_speed_mps
        : 0.0;

    if (!input.raw_reference_admitted)
    {
        output.status = ObfmBumplessSpeedStatus::RawReferenceUnavailable;
        return;
    }
    if (!IsValidControlFrameIdentity(input.frame_identity)
        || !std::isfinite(input.raw_desired_speed_mps)
        || input.raw_desired_speed_mps <= 0.0
        || !std::isfinite(input.raw_desired_speed_rate_mps2))
    {
        output.status = ObfmBumplessSpeedStatus::RawReferenceUnavailable;
        return;
    }

    // Only the already-published backend slew bounds govern the transition.
    // Gains, controller mode, and unrelated longitudinal metadata are not
    // re-admitted here.
    const double rate_min =
        input.rate_authority.speed_command_rate_min_mps2;
    const double rate_max =
        input.rate_authority.speed_command_rate_max_mps2;
    if (!input.rate_authority.valid
        || !std::isfinite(rate_min)
        || !std::isfinite(rate_max)
        || rate_min > rate_max
        || rate_min > 0.0
        || rate_max < 0.0)
    {
        const bool prior_valid = input.prior_published_speed_valid
            && std::isfinite(input.prior_published_speed_mps)
            && input.prior_published_speed_mps > 0.0;
        output.admitted = true;
        if (prior_valid)
        {
            // Slew metadata is optional command-shaping evidence. Preserve
            // the last actually-published v_cmd instead of replacing it with
            // measured own speed; the downstream TECS still clamps rate.
            output.desired_speed_mps = input.prior_published_speed_mps;
            output.desired_speed_rate_mps2 = 0.0;
            output.rate_limited =
                input.prior_published_speed_mps
                    != input.raw_desired_speed_mps
                || input.raw_desired_speed_rate_mps2 != 0.0;
            output.status = ObfmBumplessSpeedStatus::
                AdmittedPriorHoldWithoutRateAuthority;
        }
        else
        {
            // With no valid causal prior, retain the finite raw tactical
            // reference. TECS owns the existing speed-rate clamp.
            output.desired_speed_mps = input.raw_desired_speed_mps;
            output.desired_speed_rate_mps2 =
                input.raw_desired_speed_rate_mps2;
            output.status = ObfmBumplessSpeedStatus::
                AdmittedRawWithoutRateAuthority;
        }
        return;
    }

    if (!input.prior_published_speed_valid)
    {
        const double raw_rate = input.raw_desired_speed_rate_mps2;
        const double shaped_rate = raw_rate < rate_min
            ? rate_min
            : (raw_rate > rate_max ? rate_max : raw_rate);
        output.admitted = true;
        output.desired_speed_mps = input.raw_desired_speed_mps;
        output.desired_speed_rate_mps2 = shaped_rate;
        output.rate_limited = shaped_rate != raw_rate;
        output.status = ObfmBumplessSpeedStatus::AdmittedWithoutPrior;
        return;
    }

    if (!std::isfinite(input.prior_published_speed_mps)
        || input.prior_published_speed_mps <= 0.0)
    {
        output.status =
            ObfmBumplessSpeedStatus::PriorPublishedReferenceInvalid;
        return;
    }
    if (!std::isfinite(input.dt_s) || input.dt_s <= 0.0)
    {
        output.status = ObfmBumplessSpeedStatus::RateAuthorityUnavailable;
        return;
    }

    const double raw_delta = input.raw_desired_speed_mps
        - input.prior_published_speed_mps;
    const double minimum_delta = rate_min * input.dt_s;
    const double maximum_delta = rate_max * input.dt_s;
    if (!std::isfinite(raw_delta)
        || !std::isfinite(minimum_delta)
        || !std::isfinite(maximum_delta))
    {
        output.status = ObfmBumplessSpeedStatus::RateAuthorityUnavailable;
        return;
    }

    const double shaped_delta = raw_delta < minimum_delta
        ? minimum_delta
        : (raw_delta > maximum_delta ? maximum_delta : raw_delta);
    const double shaped_speed = input.prior_published_speed_mps
        + shaped_delta;
    const double shaped_rate = shaped_delta / input.dt_s;
    if (!std::isfinite(shaped_speed)
        || shaped_speed <= 0.0
        || !std::isfinite(shaped_rate))
    {
        output.status = ObfmBumplessSpeedStatus::RateAuthorityUnavailable;
        return;
    }

    output.admitted = true;
    output.rate_limited = shaped_delta != raw_delta;
    output.desired_speed_mps = shaped_speed;
    output.desired_speed_rate_mps2 = shaped_rate;
    output.status = output.rate_limited
        ? ObfmBumplessSpeedStatus::AdmittedRateLimited
        : ObfmBumplessSpeedStatus::AdmittedUnchanged;
}

void ObfmLongitudinalReferenceProvider::Evaluate(
    const ObfmLagGuidancePreparation& preparation,
    const runtime::TacticalCommandBuildInput& tactical_input,
    ObfmLongitudinalProviderReceipt& output,
    Status& status) const noexcept
{
    output = ObfmLongitudinalProviderReceipt{};
    status = Status{};
    if (!preparation.valid
        || !tactical_input.valid
        || !IsValidControlFrameIdentity(preparation.frame_identity)
        || !SameControlFrameIdentity(
            preparation.frame_identity,
            tactical_input.frame.frame_identity))
    {
        output.status =
            ObfmLongitudinalProviderStatus::FrameOrPreparationInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.reference.frame_identity = preparation.frame_identity;
    output.reference.evaluated = true;
    output.reference.source_authoritative = true;
    output.reference.same_reference_episode =
        preparation.same_reference_episode;
    if (!preparation.same_reference_episode)
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::
                ReferenceEpisodePrimeOrDiscontinuous);
        return;
    }
    if (!preparation.transported_reference_point_valid
        || !FiniteVector(preparation.previous_reference_point_ned_m)
        || !FiniteVector(preparation.transported_reference_point_ned_m)
        || !FiniteVector(preparation.current_reference_point_ned_m)
        || !FiniteVector(preparation.current_own_position_ned_m)
        || !FiniteVector(preparation.current_own_velocity_ned_mps)
        || !FiniteVector(preparation.current_target_velocity_ned_mps)
        || !std::isfinite(preparation.dt_s)
        || !std::isfinite(preparation.official_max_range_m))
    {
        output.status =
            ObfmLongitudinalProviderStatus::FrameOrPreparationInvalid;
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const double own_speed_mps =
        Norm(preparation.current_own_velocity_ned_mps);
    const double target_speed_mps =
        Norm(preparation.current_target_velocity_ned_mps);
    const double target_horizontal_speed_mps = std::sqrt(
        preparation.current_target_velocity_ned_mps[0]
            * preparation.current_target_velocity_ned_mps[0]
        + preparation.current_target_velocity_ned_mps[1]
            * preparation.current_target_velocity_ned_mps[1]);
    output.point.evaluated = true;
    output.point.current_speed_mps = std::isfinite(own_speed_mps)
        ? own_speed_mps
        : 0.0;
    output.point.target_speed_mps = std::isfinite(target_speed_mps)
        ? target_speed_mps
        : 0.0;
    if (!std::isfinite(own_speed_mps)
        || own_speed_mps <= constants::Tiny)
    {
        Reject(output, ObfmLongitudinalProviderStatus::OwnSpeedNotPositive);
        return;
    }
    if (!std::isfinite(target_speed_mps)
        || target_speed_mps <= constants::Tiny)
    {
        Reject(output, ObfmLongitudinalProviderStatus::TargetSpeedNotPositive);
        return;
    }
    if (!std::isfinite(target_horizontal_speed_mps)
        || target_horizontal_speed_mps <= constants::Tiny)
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::TargetHorizontalSpeedNotPositive);
        return;
    }
    if (preparation.dt_s <= 0.0)
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::GuidanceIntervalNotPositive);
        return;
    }
    if (preparation.official_max_range_m <= 0.0)
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::OfficialMaximumRangeNotPositive);
        return;
    }

    const Vector3 reference_velocity = Scale(
        Subtract(
            preparation.transported_reference_point_ned_m,
            preparation.previous_reference_point_ned_m),
        1.0 / preparation.dt_s);
    const Vector3 capture_error = Subtract(
        preparation.current_reference_point_ned_m,
        preparation.current_own_position_ned_m);
    const double structural_rate = target_horizontal_speed_mps
        / preparation.official_max_range_m;
    const Vector3 required_velocity = Add(
        reference_velocity,
        Scale(capture_error, structural_rate));
    const Vector3 path_direction = Scale(
        preparation.current_own_velocity_ned_mps,
        1.0 / own_speed_mps);
    const double raw_speed_mps = Dot(path_direction, required_velocity);
    const Vector3 perpendicular = Subtract(
        required_velocity,
        Scale(path_direction, raw_speed_mps));
    const double perpendicular_speed_mps = Norm(perpendicular);
    if (!std::isfinite(raw_speed_mps)
        || !std::isfinite(structural_rate)
        || !std::isfinite(perpendicular_speed_mps)
        || !FiniteVector(reference_velocity)
        || !FiniteVector(capture_error)
        || !FiniteVector(required_velocity)
        || !FiniteVector(perpendicular))
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::PointSpeedGeometryNonfinite);
        return;
    }

    output.point.raw_speed_mps = raw_speed_mps;
    output.point.structural_rate_per_s = structural_rate;
    output.point.reference_velocity_ned_mps = reference_velocity;
    output.point.capture_error_ned_m = capture_error;
    output.point.required_velocity_ned_mps = required_velocity;
    output.point.perpendicular_velocity_ned_mps = perpendicular;
    output.point.perpendicular_speed_mps = perpendicular_speed_mps;
    if (raw_speed_mps <= 0.0)
    {
        Reject(
            output,
            ObfmLongitudinalProviderStatus::
                ReferenceNotAcquirableAlongCurrentPath);
        return;
    }
    output.point.admitted = true;
    // The moving-point geometry is the speed command.  Previous-frame energy
    // telemetry may characterize tracking, but it does not own admission of
    // this current-frame reference.  TECS/CIS performs the existing command
    // shaping and actuator saturation downstream.
    output.reference.admitted = true;
    output.reference.desired_speed_mps.has_value = true;
    output.reference.desired_speed_mps.value = raw_speed_mps;
    output.reference.desired_speed_rate_mps2.has_value = true;
    output.reference.desired_speed_rate_mps2.value = 0.0;
    output.status = ObfmLongitudinalProviderStatus::ReferenceAdmitted;
}

} // namespace LadyLuck
