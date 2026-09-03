#pragma once

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace control
{
namespace route5
{

// Exact fixed-profile sources:
//   guidance/pursuit_manuver/vpp_los_bandwidth_limiter.py
//   guidance/pursuit_manuver/vpp2n_cmd_mu_cmd.py, Yang et al. Eq.(1)
// The physical gains and LOS profile are frozen authority, not tunable Route-5
// admission knobs.  The caller supplies only Route5Guidance's existing
// elevation limit.
enum class AimPointVectorGuidanceReason : std::int32_t
{
    Applied = 0,
    Initialized = 1,
    HeldInvalidTimestep = 2,
    HeldInvalidInput = 3,
    HeldInvalidRange = 4,
    MissingHeldLos = 5,
    InvalidVelocity = 6,
    InvalidElevationLimit = 7,
    InternalGeometryFault = 8,
    AppliedShortRange = 9
};

struct AimPointVectorGuidanceOutput
{
    bool valid = false;
    bool observation_valid = false;
    bool initialized = false;
    bool rate_limited = false;
    AimPointVectorGuidanceReason reason =
        AimPointVectorGuidanceReason::MissingHeldLos;
    Vector3 commanded_los_n{};
    Vector3 applied_los_rate_n_radps{};
    Vector3 acceleration_ned_mps2{};
    double raw_range_m = 0.0;
    double raw_command_angle_rad = 0.0;
    double applied_angle_rad = 0.0;
};

struct AimPointVectorGuidanceSnapshot
{
    bool initialized = false;
    bool last_axis_valid = false;
    bool previous_los_valid = false;
    Vector3 commanded_los_n{};
    Vector3 last_axis_n{};
};

namespace aim_point_vector_detail
{

inline bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

inline double Dot(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

inline Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

inline double Norm(const Vector3& value) noexcept
{
    return std::sqrt(Dot(value, value));
}

inline bool Normalize(
    const Vector3& value,
    const double epsilon,
    Vector3& output) noexcept
{
    output = Vector3{};
    if (!FiniteVector(value))
    {
        return false;
    }
    const double norm = Norm(value);
    if (!std::isfinite(norm) || norm <= epsilon)
    {
        return false;
    }
    output = Vector3{{
        value[0] / norm,
        value[1] / norm,
        value[2] / norm}};
    return FiniteVector(output);
}

inline bool TangentAxis(
    const Vector3& candidate,
    const Vector3& los_n,
    Vector3& output) noexcept
{
    constexpr double vector_epsilon = 1.0e-12;
    const double projection = Dot(candidate, los_n);
    const Vector3 tangent{{
        candidate[0] - projection * los_n[0],
        candidate[1] - projection * los_n[1],
        candidate[2] - projection * los_n[2]}};
    return Normalize(tangent, vector_epsilon, output);
}

inline Vector3 LeastAlignedBasis(const Vector3& los_n) noexcept
{
    const double absolute_x = std::fabs(los_n[0]);
    const double absolute_y = std::fabs(los_n[1]);
    const double absolute_z = std::fabs(los_n[2]);
    if (absolute_x <= absolute_y && absolute_x <= absolute_z)
    {
        return Vector3{{1.0, 0.0, 0.0}};
    }
    if (absolute_y <= absolute_z)
    {
        return Vector3{{0.0, 1.0, 0.0}};
    }
    return Vector3{{0.0, 0.0, 1.0}};
}

inline bool FallbackAxis(
    const Vector3& los_n,
    const bool last_axis_valid,
    const Vector3& last_axis_n,
    Vector3& output) noexcept
{
    if (last_axis_valid && TangentAxis(last_axis_n, los_n, output))
    {
        return true;
    }
    const Vector3 frozen_fallback_axis_n{{0.0, 0.0, 1.0}};
    if (TangentAxis(frozen_fallback_axis_n, los_n, output))
    {
        return true;
    }
    return TangentAxis(LeastAlignedBasis(los_n), los_n, output);
}

inline bool RodriguesRotate(
    const Vector3& vector,
    const Vector3& axis,
    const double angle_rad,
    Vector3& output) noexcept
{
    constexpr double vector_epsilon = 1.0e-12;
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    const double projection = Dot(axis, vector);
    const Vector3 cross = Cross(axis, vector);
    const Vector3 rotated{{
        vector[0] * cosine + cross[0] * sine
            + axis[0] * projection * (1.0 - cosine),
        vector[1] * cosine + cross[1] * sine
            + axis[1] * projection * (1.0 - cosine),
        vector[2] * cosine + cross[2] * sine
            + axis[2] * projection * (1.0 - cosine)}};
    return Normalize(rotated, vector_epsilon, output);
}

inline bool ElevationLimitedLos(
    const Vector3& raw_los_n,
    const Vector3& velocity_ned_mps,
    const double elevation_limit_rad,
    Vector3& output) noexcept
{
    constexpr double vector_epsilon = 1.0e-12;
    if (!std::isfinite(elevation_limit_rad)
        || elevation_limit_rad < 0.0)
    {
        output = Vector3{};
        return false;
    }
    Vector3 los_n{};
    if (!Normalize(raw_los_n, vector_epsilon, los_n))
    {
        output = Vector3{};
        return false;
    }
    const double horizontal = std::hypot(los_n[0], los_n[1]);
    const double velocity_course = std::atan2(
        velocity_ned_mps[1],
        velocity_ned_mps[0]);
    const double azimuth = horizontal <= vector_epsilon
        ? velocity_course
        : std::atan2(los_n[1], los_n[0]);
    const double raw_elevation = std::atan2(-los_n[2], horizontal);
    const double geometric_elevation_limit = std::min(
        elevation_limit_rad,
        0.5 * constants::Pi);
    const double elevation = std::max(
        -geometric_elevation_limit,
        std::min(geometric_elevation_limit, raw_elevation));
    output = Vector3{{
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        -std::sin(elevation)}};
    return FiniteVector(output);
}

inline bool Eq1Acceleration(
    const Vector3& commanded_los_n,
    const Vector3& applied_los_rate_n_radps,
    const Vector3& velocity_ned_mps,
    Vector3& output) noexcept
{
    constexpr double k_pg_per_s = 2.0;
    constexpr double navigation_constant = 0.5;
    constexpr double vector_epsilon = 1.0e-12;
    Vector3 los_n{};
    Vector3 velocity_direction{};
    if (!Normalize(commanded_los_n, vector_epsilon, los_n)
        || !Normalize(
            velocity_ned_mps,
            constants::Epsilon,
            velocity_direction)
        || !FiniteVector(applied_los_rate_n_radps))
    {
        output = Vector3{};
        return false;
    }
    const Vector3 cross_velocity_los = Cross(velocity_direction, los_n);
    const double sine = Norm(cross_velocity_los);
    const double cosine = std::max(
        -1.0,
        std::min(1.0, Dot(velocity_direction, los_n)));
    const double angle = std::atan2(sine, cosine);
    Vector3 mu_vector{};
    if (sine > constants::Tiny)
    {
        const double scale = angle / sine;
        mu_vector = Vector3{{
            scale * cross_velocity_los[0],
            scale * cross_velocity_los[1],
            scale * cross_velocity_los[2]}};
    }
    else if (cosine < 0.0)
    {
        // Eq.(1)'s velocity/LOS rotation is axis-indeterminate only at the
        // exact antipode.  Preserve full pi-angle authority with the same
        // deterministic tangent-axis rule as the spherical LOS limiter.
        Vector3 antipodal_axis{};
        const Vector3 no_last_axis{};
        if (!FallbackAxis(
                velocity_direction,
                false,
                no_last_axis,
                antipodal_axis))
        {
            output = Vector3{};
            return false;
        }
        mu_vector = Vector3{{
            angle * antipodal_axis[0],
            angle * antipodal_axis[1],
            angle * antipodal_axis[2]}};
    }
    const Vector3 proportional = Cross(mu_vector, velocity_ned_mps);
    const Vector3 navigation = Cross(
        applied_los_rate_n_radps,
        velocity_ned_mps);
    output = Vector3{{
        k_pg_per_s * proportional[0]
            + navigation_constant * navigation[0],
        k_pg_per_s * proportional[1]
            + navigation_constant * navigation[1],
        k_pg_per_s * proportional[2]
            + navigation_constant * navigation[2]}};
    return FiniteVector(output);
}

} // namespace aim_point_vector_detail

class AimPointVectorGuidance final
{
public:
    void Reset() noexcept
    {
        initialized_ = false;
        last_axis_valid_ = false;
        previous_los_valid_ = false;
        commanded_los_n_ = Vector3{};
        last_axis_n_ = Vector3{};
    }

    void CopySnapshot(AimPointVectorGuidanceSnapshot& output) const noexcept
    {
        output = AimPointVectorGuidanceSnapshot{};
        output.initialized = initialized_;
        output.last_axis_valid = last_axis_valid_;
        output.previous_los_valid = previous_los_valid_;
        output.commanded_los_n = commanded_los_n_;
        output.last_axis_n = last_axis_n_;
    }

    void Step(
        const Vector3& aircraft_position_ned_m,
        const Vector3& raw_aim_point_ned_m,
        const Vector3& velocity_ned_mps,
        const double elevation_limit_rad,
        const double dt_s,
        AimPointVectorGuidanceOutput& output) noexcept
    {
        constexpr double omega_max_radps =
            constants::Pi / 6.0;
        constexpr double minimum_range_m = 50.0;
        constexpr double same_direction_epsilon_rad = 1.0e-6;
        constexpr double maximum_valid_dt_s = 0.05;
        constexpr double antipodal_sine_epsilon = 1.0e-8;
        constexpr double vector_epsilon = 1.0e-12;

        output = AimPointVectorGuidanceOutput{};
        output.initialized = initialized_;
        if (!std::isfinite(dt_s) || dt_s <= 0.0)
        {
            previous_los_valid_ = false;
            output.reason = AimPointVectorGuidanceReason::HeldInvalidTimestep;
            return;
        }
        if (!aim_point_vector_detail::FiniteVector(aircraft_position_ned_m)
            || !aim_point_vector_detail::FiniteVector(raw_aim_point_ned_m)
            || !aim_point_vector_detail::FiniteVector(velocity_ned_mps))
        {
            previous_los_valid_ = false;
            output.reason = AimPointVectorGuidanceReason::HeldInvalidInput;
            return;
        }
        if (!std::isfinite(elevation_limit_rad)
            || elevation_limit_rad < 0.0)
        {
            previous_los_valid_ = false;
            output.reason = AimPointVectorGuidanceReason::InvalidElevationLimit;
            return;
        }
        if (dt_s > maximum_valid_dt_s)
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::HeldInvalidTimestep,
                velocity_ned_mps,
                output);
            return;
        }

        const Vector3 relative{{
            raw_aim_point_ned_m[0] - aircraft_position_ned_m[0],
            raw_aim_point_ned_m[1] - aircraft_position_ned_m[1],
            raw_aim_point_ned_m[2] - aircraft_position_ned_m[2]}};
        const double raw_range_m = aim_point_vector_detail::Norm(relative);
        output.raw_range_m = raw_range_m;
        if (!std::isfinite(raw_range_m))
        {
            previous_los_valid_ = false;
            output.reason = AimPointVectorGuidanceReason::HeldInvalidInput;
            return;
        }
        if (raw_range_m <= vector_epsilon)
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::HeldInvalidRange,
                velocity_ned_mps,
                output);
            return;
        }
        const bool short_range = raw_range_m < minimum_range_m;

        Vector3 raw_los_n{};
        if (!aim_point_vector_detail::ElevationLimitedLos(
                relative,
                velocity_ned_mps,
                elevation_limit_rad,
                raw_los_n))
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::InternalGeometryFault,
                velocity_ned_mps,
                output);
            return;
        }
        if (!initialized_)
        {
            commanded_los_n_ = raw_los_n;
            initialized_ = true;
            previous_los_valid_ = true;
            output.observation_valid = true;
            output.initialized = true;
            output.reason = short_range
                ? AimPointVectorGuidanceReason::AppliedShortRange
                : AimPointVectorGuidanceReason::Initialized;
            CompleteOutput(velocity_ned_mps, output);
            return;
        }

        Vector3 previous{};
        if (!aim_point_vector_detail::Normalize(
                commanded_los_n_,
                vector_epsilon,
                previous))
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::InternalGeometryFault,
                velocity_ned_mps,
                output);
            return;
        }
        const Vector3 cross = aim_point_vector_detail::Cross(
            previous,
            raw_los_n);
        const double sine = aim_point_vector_detail::Norm(cross);
        const double cosine = std::max(
            -1.0,
            std::min(1.0, aim_point_vector_detail::Dot(previous, raw_los_n)));
        const double theta = std::atan2(sine, cosine);
        const double maximum_step = omega_max_radps * dt_s;
        Vector3 axis{};
        if (sine > antipodal_sine_epsilon)
        {
            axis = Vector3{{
                cross[0] / sine,
                cross[1] / sine,
                cross[2] / sine}};
        }
        else if (cosine < 0.0)
        {
            if (!aim_point_vector_detail::FallbackAxis(
                    previous,
                    last_axis_valid_,
                    last_axis_n_,
                    axis))
            {
                previous_los_valid_ = false;
                HoldOrSeedVelocity(
                    AimPointVectorGuidanceReason::InternalGeometryFault,
                    velocity_ned_mps,
                    output);
                return;
            }
        }
        else if (sine > vector_epsilon)
        {
            axis = Vector3{{
                cross[0] / sine,
                cross[1] / sine,
                cross[2] / sine}};
        }
        else if (!aim_point_vector_detail::FallbackAxis(
                previous,
                last_axis_valid_,
                last_axis_n_,
                axis))
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::InternalGeometryFault,
                velocity_ned_mps,
                output);
            return;
        }

        const bool can_snap = theta <= same_direction_epsilon_rad
            && theta <= maximum_step;
        const double applied_angle = can_snap
            ? theta
            : std::min(theta, maximum_step);
        Vector3 commanded{};
        if (can_snap)
        {
            commanded = raw_los_n;
        }
        else if (applied_angle <= vector_epsilon)
        {
            commanded = previous;
        }
        else if (!aim_point_vector_detail::RodriguesRotate(
                previous,
                axis,
                applied_angle,
                commanded))
        {
            previous_los_valid_ = false;
            HoldOrSeedVelocity(
                AimPointVectorGuidanceReason::InternalGeometryFault,
                velocity_ned_mps,
                output);
            return;
        }

        commanded_los_n_ = commanded;
        if (applied_angle > vector_epsilon)
        {
            last_axis_n_ = axis;
            last_axis_valid_ = true;
            const double applied_rate = applied_angle / dt_s;
            output.applied_los_rate_n_radps = Vector3{{
                applied_rate * axis[0],
                applied_rate * axis[1],
                applied_rate * axis[2]}};
        }
        previous_los_valid_ = true;
        output.observation_valid = true;
        output.initialized = true;
        output.rate_limited = theta > maximum_step;
        output.reason = short_range
            ? AimPointVectorGuidanceReason::AppliedShortRange
            : AimPointVectorGuidanceReason::Applied;
        output.raw_command_angle_rad = theta;
        output.applied_angle_rad = applied_angle;
        CompleteOutput(velocity_ned_mps, output);
    }

private:
    void HoldOrSeedVelocity(
        const AimPointVectorGuidanceReason reason,
        const Vector3& velocity_ned_mps,
        AimPointVectorGuidanceOutput& output) noexcept
    {
        if (!initialized_)
        {
            Vector3 velocity_direction{};
            if (!aim_point_vector_detail::Normalize(
                    velocity_ned_mps,
                    constants::Epsilon,
                    velocity_direction))
            {
                output.reason = AimPointVectorGuidanceReason::InvalidVelocity;
                return;
            }
            commanded_los_n_ = velocity_direction;
            initialized_ = true;
        }
        HeldOutput(reason, velocity_ned_mps, output);
    }

    void HeldOutput(
        const AimPointVectorGuidanceReason reason,
        const Vector3& velocity_ned_mps,
        AimPointVectorGuidanceOutput& output) const noexcept
    {
        output.reason = initialized_
            ? reason
            : AimPointVectorGuidanceReason::MissingHeldLos;
        output.initialized = initialized_;
        output.commanded_los_n = commanded_los_n_;
        output.valid = initialized_
            && aim_point_vector_detail::Eq1Acceleration(
                commanded_los_n_,
                output.applied_los_rate_n_radps,
                velocity_ned_mps,
                output.acceleration_ned_mps2);
    }

    void CompleteOutput(
        const Vector3& velocity_ned_mps,
        AimPointVectorGuidanceOutput& output) const noexcept
    {
        output.commanded_los_n = commanded_los_n_;
        output.valid = aim_point_vector_detail::Eq1Acceleration(
            commanded_los_n_,
            output.applied_los_rate_n_radps,
            velocity_ned_mps,
            output.acceleration_ned_mps2);
        if (!output.valid)
        {
            output.reason = AimPointVectorGuidanceReason::InvalidVelocity;
        }
    }

    bool initialized_ = false;
    bool last_axis_valid_ = false;
    bool previous_los_valid_ = false;
    Vector3 commanded_los_n_{};
    Vector3 last_axis_n_{};
};

} // namespace route5
} // namespace control
} // namespace LadyLuck

static_assert(
    std::is_trivially_copyable<
        LadyLuck::control::route5::AimPointVectorGuidance>::value,
    "AimPoint vector guidance must remain fixed-storage value state");
static_assert(
    std::is_nothrow_copy_constructible<
        LadyLuck::control::route5::AimPointVectorGuidance>::value,
    "AimPoint vector guidance projection must remain noexcept-copyable");
