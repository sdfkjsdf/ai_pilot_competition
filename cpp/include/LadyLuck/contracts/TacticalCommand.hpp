#pragma once

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace LadyLuck
{
template <typename T>
struct OptionalValue
{
    bool has_value = false;
    T value{};
};

// Raw tactical guidance reference.  This is not a body-rate, load-factor,
// control-surface, thrust, or observed-aircraft-response contract.
//
// Field order and defaults mirror guidance/tactical_command.py at the frozen
// add/main 4de76fa7 authority.  In particular, absence remains distinct from
// a present zero value on every optional field.
struct TacticalCommand
{
    Vector3 aim_point_m{};                                             //  1
    double desired_speed_mps = 0.0;                                   //  2
    OptionalValue<Vector3> aim_point_velocity_mps{};                   //  3
    double desired_speed_rate_mps2 = 0.0;                             //  4
    double specific_energy_rate_bias_m2ps3 = 0.0;                     //  5
    OptionalValue<bool> path_inversion_allowed{};                     //  6
    double capture_range_des_m = 650.0;                               //  7
    double aim_blend = 0.0;                                           //  8
    double lead_time_tau_sec = 0.0;                                   //  9
    double lateral_offset_m = 0.0;                                    // 10
    double vertical_offset_m = 0.0;                                   // 11
    double vertical_yoyo_scale = 0.0;                                 // 12
    OptionalValue<double> k_roll{};                                   // 13
    OptionalValue<double> k_pitch{};                                  // 14
    OptionalValue<double> throttle_bias{};                            // 15
    OptionalValue<double> total_load_factor_limit_g{};                // 16
    OptionalValue<double> direct_p_cmd_radps{};                       // 17
    OptionalValue<double> direct_nz_cmd_g{};                          // 18
    OptionalValue<double> direct_beta_cmd_rad{};                      // 19
    OptionalValue<double> direct_accel_cmd_mps2{};                    // 20
    OptionalValue<Vector3> direct_acceleration_ned_mps2{};            // 21
    // This is a velocity-bank direction-effect reference.  It is not a
    // direct body-p command; the direct-NED allocator remains the sole owner
    // of the resulting p/q/r/Nz references.
    OptionalValue<double> direct_acceleration_roll_rate_reference_radps{}; // 22
    bool direct_acceleration_tracking_enabled = false;                // 23
    bool direct_acceleration_tracking_observation_only = false;       // 24
    bool direct_acceleration_magnitude_tracking_enabled = false;      // 25
    bool direct_acceleration_loaded_roll_enabled = false;             // 26
    bool direct_acceleration_load_component_compensation_enabled = false; // 27
    bool direct_acceleration_yaw_coordination_enabled = false;        // 28
    bool direct_acceleration_roll_priority_yaw_enabled = false;       // 29
    OptionalValue<double> direct_bank_cmd_rad{};                      // 30
    OptionalValue<double> direct_turn_rate_cmd_radps{};               // 31
    std::string behavior_label = "tracking";                          // 32
    std::string mode_label = "CONTROL_ZONE";                          // 33
    OptionalValue<Vector3> direct_load_vector_acceleration_ned_mps2{}; // 34
};

namespace tactical_command_detail
{
inline bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

inline Status Failure(const StatusCode code) noexcept
{
    Status status{};
    status.code = code;
    return status;
}

// numpy.isclose(a, b, rtol=1e-12, atol=0) uses b as its reference magnitude.
inline bool IsCloseH09(const double left, const double right) noexcept
{
    if (left == right)
    {
        return true;
    }
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    return std::fabs(left - right) <= 1.0e-12 * std::fabs(right);
}
}

// Exact structural validation performed by TacticalCommand.__post_init__.
// Required aim/speed finiteness is deliberately not added here: the Python
// constructor also leaves that later guidance-capture check to its owner.
inline Status ValidateTacticalCommandStructure(
    const TacticalCommand& command) noexcept
{
    if (command.direct_acceleration_ned_mps2.has_value)
    {
        if (command.direct_p_cmd_radps.has_value
            || command.direct_nz_cmd_g.has_value
            || command.direct_bank_cmd_rad.has_value
            || command.direct_load_vector_acceleration_ned_mps2.has_value)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        if (!tactical_command_detail::FiniteVector(
                command.direct_acceleration_ned_mps2.value))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
    }

    if (command.direct_acceleration_roll_rate_reference_radps.has_value)
    {
        const double roll_rate =
            command.direct_acceleration_roll_rate_reference_radps.value;
        if (!std::isfinite(roll_rate))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
        if (!command.direct_acceleration_ned_mps2.has_value)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
    }

    if (command.direct_acceleration_tracking_enabled
        && !command.direct_acceleration_ned_mps2.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_tracking_observation_only
        && !(command.direct_acceleration_tracking_enabled
            && command.direct_acceleration_ned_mps2.has_value))
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_magnitude_tracking_enabled
        && !(command.direct_acceleration_tracking_enabled
            && command.direct_acceleration_ned_mps2.has_value))
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_loaded_roll_enabled
        && !command.direct_acceleration_ned_mps2.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_load_component_compensation_enabled
        && !command.direct_acceleration_ned_mps2.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_yaw_coordination_enabled
        && !command.direct_acceleration_ned_mps2.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_roll_priority_yaw_enabled
        && !command.direct_acceleration_ned_mps2.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }
    if (command.direct_acceleration_roll_priority_yaw_enabled
        && command.direct_acceleration_yaw_coordination_enabled)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }

    if (command.total_load_factor_limit_g.has_value)
    {
        const double limit = command.total_load_factor_limit_g.value;
        if (!std::isfinite(limit))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
        if (limit <= 0.0)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
    }

    if (command.aim_point_velocity_mps.has_value
        && !tactical_command_detail::FiniteVector(
            command.aim_point_velocity_mps.value))
    {
        return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
    }

    constexpr double HalfPi = 0.5 * constants::Pi;
    if (command.direct_bank_cmd_rad.has_value)
    {
        const double bank = command.direct_bank_cmd_rad.value;
        if (!std::isfinite(bank))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
        if (std::fabs(bank) >= HalfPi)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        if (!command.direct_turn_rate_cmd_radps.has_value)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        const double turn_rate = command.direct_turn_rate_cmd_radps.value;
        if (!std::isfinite(turn_rate))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
        if (turn_rate <= 0.0)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        if (!command.total_load_factor_limit_g.has_value)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        const double speed = command.desired_speed_mps;
        if (!std::isfinite(speed))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
        if (speed <= 0.0)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }

        const double load = command.total_load_factor_limit_g.value;
        const double lateral_from_bank = std::tan(std::fabs(bank));
        const double lateral_from_rate =
            speed * turn_rate / constants::StandardGravityMps2;
        if (!tactical_command_detail::IsCloseH09(
                lateral_from_bank,
                lateral_from_rate))
        {
            return tactical_command_detail::Failure(
                StatusCode::InvalidConfiguration);
        }
        const double load_from_bank = 1.0 / std::cos(std::fabs(bank));
        if (!tactical_command_detail::IsCloseH09(load_from_bank, load))
        {
            return tactical_command_detail::Failure(
                StatusCode::InvalidConfiguration);
        }
    }
    else if (command.direct_turn_rate_cmd_radps.has_value)
    {
        return tactical_command_detail::Failure(StatusCode::InvalidArgument);
    }

    if (command.direct_load_vector_acceleration_ned_mps2.has_value)
    {
        if (command.direct_p_cmd_radps.has_value
            || command.direct_nz_cmd_g.has_value
            || command.direct_beta_cmd_rad.has_value
            || command.direct_bank_cmd_rad.has_value
            || command.direct_acceleration_ned_mps2.has_value
            || command.direct_acceleration_roll_rate_reference_radps.has_value)
        {
            return tactical_command_detail::Failure(StatusCode::InvalidArgument);
        }
        if (!tactical_command_detail::FiniteVector(
                command.direct_load_vector_acceleration_ned_mps2.value))
        {
            return tactical_command_detail::Failure(StatusCode::NonFiniteInput);
        }
    }

    return Status{};
}

// On failure the returned value is not authoritative and must be ignored.
inline Result<TacticalCommand> MakeValidatedTacticalCommand(
    const TacticalCommand& candidate) noexcept
{
    Result<TacticalCommand> result{};
    result.status = ValidateTacticalCommandStructure(candidate);
    if (!result.status.ok())
    {
        return result;
    }
    try
    {
        result.value = candidate;
    }
    catch (...)
    {
        Result<TacticalCommand> failed{};
        failed.status.code = StatusCode::InvalidConfiguration;
        return failed;
    }
    return result;
}
}
