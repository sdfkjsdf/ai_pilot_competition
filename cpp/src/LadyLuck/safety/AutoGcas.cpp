#include "LadyLuck/safety/AutoGcas.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace
{
using LadyLuck::ControlFrameIdentity;
using LadyLuck::PlaneState;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::safety::AutoGcasConfig;
using LadyLuck::safety::AutoGcasEntryInput;
using LadyLuck::safety::AutoGcasEntryReceipt;
using LadyLuck::safety::AutoGcasPhase;
using LadyLuck::safety::AutoGcasReceipt;
using LadyLuck::safety::NormalizedControlCommand;

constexpr double RollGainPerSecond = 4.0;
constexpr double TriggerEqualityToleranceM = 1.0e-9;
constexpr double ClimbRateSpeedToleranceMps = 1.0e-9;
constexpr double NominalPipelineDtS = 1.0 / 60.0;
constexpr std::uint32_t MaximumPredictionSamples = 600U;

struct PredictionReceipt
{
    bool valid = false;
    bool predicted_bottom_valid = false;
    double predicted_bottom_altitude_m = 0.0;
    bool recovery_established = false;
    bool horizon_exhausted = false;
};

struct EntryReceipt
{
    bool available = false;
    bool should_activate = false;
    bool boundary_breached = false;
    bool recoverable = false;
    bool effective_nz_valid = false;
    double effective_nz_g = 0.0;
    PredictionReceipt prediction{};
};

Status Failure(const StatusCode code) noexcept
{
    Status status{};
    status.code = code;
    return status;
}

double Clamp(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::min(upper, std::max(lower, value));
}

double WrapRadians(const double value) noexcept
{
    return std::atan2(std::sin(value), std::cos(value));
}

bool FiniteControl(const NormalizedControlCommand& command) noexcept
{
    return std::isfinite(command.aileron)
        && std::isfinite(command.elevator)
        && std::isfinite(command.rudder)
        && std::isfinite(command.throttle);
}

bool ControlInDomain(const NormalizedControlCommand& command) noexcept
{
    return command.aileron >= -1.0 && command.aileron <= 1.0
        && command.elevator >= -1.0 && command.elevator <= 1.0
        && command.rudder >= -1.0 && command.rudder <= 1.0
        && command.throttle >= 0.0 && command.throttle <= 1.0;
}

bool FiniteOwnship(const PlaneState& state) noexcept
{
    return std::isfinite(state.position_ned_m[0])
        && std::isfinite(state.position_ned_m[1])
        && std::isfinite(state.position_ned_m[2])
        && std::isfinite(state.rpy_rad[0])
        && std::isfinite(state.rpy_rad[1])
        && std::isfinite(state.rpy_rad[2])
        && std::isfinite(state.velocity_body_mps[0])
        && std::isfinite(state.velocity_body_mps[1])
        && std::isfinite(state.velocity_body_mps[2])
        && std::isfinite(state.speed_mps)
        && std::isfinite(state.alpha_rad)
        && std::isfinite(state.beta_rad);
}

bool ConfigurationValid(const AutoGcasConfig& config) noexcept
{
    const double values[] = {
        config.crash_floor_m,
        config.min_altitude_m,
        config.entry_effective_roll_rate_radps,
        config.prediction_margin,
        config.onset_rate_gps,
        config.release_altitude_m,
        config.wings_level_gate_rad,
        config.settled_roll_rate_gate_radps,
        config.minimum_pull_time_s,
        config.pull_n_g,
        config.maximum_roll_rate_radps,
        config.maximum_prediction_horizon_s,
        config.settle_gamma_rad,
        config.minimum_settle_time_s,
        config.settle_nz_tolerance_g,
        config.settle_vertical_speed_gain_g_per_mps,
        config.settle_min_nz_g};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return config.crash_floor_m >= 0.0
        && config.min_altitude_m > config.crash_floor_m
        && config.entry_effective_roll_rate_radps > 0.0
        && config.prediction_margin >= 1.0
        && config.onset_rate_gps > 0.0
        && config.wings_level_gate_rad >= 0.0
        && config.settled_roll_rate_gate_radps >= 0.0
        && config.minimum_pull_time_s >= 0.0
        && config.pull_n_g > 1.0
        && config.maximum_roll_rate_radps > 0.0
        && config.maximum_prediction_horizon_s >= NominalPipelineDtS
        && config.settle_gamma_rad > 0.0
        && config.settle_gamma_rad < 0.5 * LadyLuck::constants::Pi
        && config.minimum_settle_time_s >= NominalPipelineDtS
        && config.settle_nz_tolerance_g > 0.0
        && config.settle_nz_tolerance_g < 1.0
        && config.settle_vertical_speed_gain_g_per_mps > 0.0
        && config.settle_min_nz_g >= 0.0
        && config.settle_min_nz_g < 1.0;
}

double ClimbRate(const PlaneState& state) noexcept
{
    const double roll = state.rpy_rad[0];
    const double pitch = state.rpy_rad[1];
    const double u = state.velocity_body_mps[0];
    const double v = state.velocity_body_mps[1];
    const double w = state.velocity_body_mps[2];
    const double sine_pitch = std::sin(pitch);
    const double cosine_pitch = std::cos(pitch);
    const double sine_roll = std::sin(roll);
    const double cosine_roll = std::cos(roll);
    return u * sine_pitch
        - v * sine_roll * cosine_pitch
        - w * cosine_roll * cosine_pitch;
}

bool RecoveryReference(
    const AutoGcasConfig& config,
    const AutoGcasPhase phase,
    const double bank_rad,
    const double climb_rate_mps,
    const double pull_nz_g,
    double& nz_cmd_g,
    double& p_cmd_radps) noexcept
{
    nz_cmd_g = 0.0;
    p_cmd_radps = 0.0;
    if (!std::isfinite(bank_rad)
        || !std::isfinite(climb_rate_mps)
        || !std::isfinite(pull_nz_g)
        || pull_nz_g <= 1.0)
    {
        return false;
    }
    p_cmd_radps = Clamp(
        RollGainPerSecond * WrapRadians(-bank_rad),
        -config.maximum_roll_rate_radps,
        config.maximum_roll_rate_radps);
    switch (phase)
    {
    case AutoGcasPhase::Roll:
        nz_cmd_g = std::max(std::cos(bank_rad), 0.0);
        break;
    case AutoGcasPhase::Pull:
        nz_cmd_g = pull_nz_g;
        break;
    case AutoGcasPhase::Level:
        nz_cmd_g = 1.0
            - config.settle_vertical_speed_gain_g_per_mps
                * climb_rate_mps;
        nz_cmd_g = Clamp(nz_cmd_g, config.settle_min_nz_g, pull_nz_g);
        break;
    default:
        return false;
    }
    return std::isfinite(nz_cmd_g) && std::isfinite(p_cmd_radps);
}

bool SurfaceCommandFromReference(
    const AutoGcasConfig& config,
    const AutoGcasPhase phase,
    const double nz_cmd_g,
    const double p_cmd_radps,
    NormalizedControlCommand& output) noexcept
{
    output = NormalizedControlCommand{};
    if (!std::isfinite(nz_cmd_g) || !std::isfinite(p_cmd_radps))
    {
        return false;
    }
    const double aileron = Clamp(
        p_cmd_radps / std::max(1.0e-6, config.maximum_roll_rate_radps),
        -1.0,
        1.0);
    const double normalized_nz = (nz_cmd_g - 1.0)
        / std::max(1.0, config.pull_n_g - 1.0);
    double elevator = 0.0;
    if (phase == AutoGcasPhase::Pull)
    {
        elevator = -Clamp(normalized_nz, 0.25, 1.0);
    }
    else if (phase == AutoGcasPhase::Roll || phase == AutoGcasPhase::Level)
    {
        elevator = -Clamp(normalized_nz, -1.0, 1.0);
    }
    else
    {
        return false;
    }
    output = NormalizedControlCommand{
        aileron,
        elevator,
        0.0,
        1.0,
        true};
    return FiniteControl(output) && ControlInDomain(output);
}

void PredictRecovery(
    const AutoGcasConfig& config,
    const double altitude_m,
    const double speed_mps,
    const double gamma_rad,
    const double bank_rad,
    const double roll_rate_radps,
    const bool roll_rate_valid,
    const double measured_nz_g,
    const bool measured_nz_valid,
    const double available_nz_g,
    const double dt_s,
    PredictionReceipt& output) noexcept
{
    output = PredictionReceipt{};
    const double values[] = {
        altitude_m,
        speed_mps,
        gamma_rad,
        bank_rad,
        roll_rate_radps,
        measured_nz_g,
        available_nz_g,
        dt_s};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return;
        }
    }
    if (!roll_rate_valid || speed_mps <= 0.0 || available_nz_g <= 1.0
        || dt_s <= 0.0 || dt_s > config.maximum_prediction_horizon_s)
    {
        return;
    }
    const double step_count_value = std::ceil(
        config.maximum_prediction_horizon_s / dt_s);
    if (!std::isfinite(step_count_value)
        || step_count_value < 1.0
        || step_count_value
            > static_cast<double>(MaximumPredictionSamples))
    {
        // The production 60 Hz/gap path is bounded by 600 samples over 10 s.
        // A smaller-than-production dt cannot expand work in the 60 Hz thread;
        // an invalid prediction keeps GCAS authority instead.
        return;
    }
    const std::uint32_t max_steps = static_cast<std::uint32_t>(
        step_count_value);
    const double pull_nz_g = std::min(available_nz_g, config.pull_n_g);
    double altitude = altitude_m;
    double gamma = gamma_rad;
    double bank = WrapRadians(bank_rad);
    double achieved_nz = measured_nz_valid
        ? Clamp(measured_nz_g, 0.0, pull_nz_g)
        : 0.0;
    const double latency_gamma_dot = LadyLuck::constants::StandardGravityMps2
        / speed_mps
        * (achieved_nz * std::cos(bank) - std::cos(gamma));
    const double latency_next_gamma = gamma + latency_gamma_dot * dt_s;
    altitude += speed_mps * 0.5
        * (std::sin(gamma) + std::sin(latency_next_gamma)) * dt_s;
    bank = WrapRadians(bank + roll_rate_radps * dt_s);
    gamma = latency_next_gamma;
    if (!std::isfinite(altitude)
        || !std::isfinite(bank)
        || !std::isfinite(gamma))
    {
        return;
    }

    double minimum_altitude = std::min(altitude_m, altitude);
    AutoGcasPhase phase = std::fabs(bank) <= 0.5 * LadyLuck::constants::Pi
        ? AutoGcasPhase::Pull
        : AutoGcasPhase::Roll;
    std::uint32_t samples = 1U;
    bool recovery_established = false;

    while (samples < max_steps)
    {
        const double climb_rate_mps = speed_mps * std::sin(gamma);
        double nz_cmd_g = 0.0;
        double p_cmd_radps = 0.0;
        if (!RecoveryReference(
                config,
                phase,
                bank,
                climb_rate_mps,
                pull_nz_g,
                nz_cmd_g,
                p_cmd_radps))
        {
            return;
        }
        const double achieved_p_radps = Clamp(
            p_cmd_radps,
            -config.entry_effective_roll_rate_radps,
            config.entry_effective_roll_rate_radps);
        if (phase == AutoGcasPhase::Roll
            && std::fabs(bank) <= 0.5 * LadyLuck::constants::Pi)
        {
            phase = AutoGcasPhase::Pull;
            if (!RecoveryReference(
                    config,
                    phase,
                    bank,
                    climb_rate_mps,
                    pull_nz_g,
                    nz_cmd_g,
                    p_cmd_radps))
            {
                return;
            }
        }

        bank = WrapRadians(bank + achieved_p_radps * dt_s);
        if (phase == AutoGcasPhase::Roll)
        {
            achieved_nz = nz_cmd_g;
        }
        else
        {
            const double nz_step = config.onset_rate_gps * dt_s;
            achieved_nz += Clamp(nz_cmd_g - achieved_nz, -nz_step, nz_step);
        }
        const double gamma_dot = LadyLuck::constants::StandardGravityMps2
            / speed_mps
            * (achieved_nz * std::cos(bank) - std::cos(gamma));
        const double next_gamma = gamma + gamma_dot * dt_s;
        altitude += speed_mps * 0.5
            * (std::sin(gamma) + std::sin(next_gamma)) * dt_s;
        gamma = next_gamma;
        ++samples;
        if (!std::isfinite(altitude)
            || !std::isfinite(bank)
            || !std::isfinite(gamma)
            || !std::isfinite(achieved_nz))
        {
            return;
        }
        if (altitude < minimum_altitude)
        {
            minimum_altitude = altitude;
        }
        else if (phase == AutoGcasPhase::Pull
            && altitude > minimum_altitude
            && gamma >= 0.0)
        {
            recovery_established = true;
            break;
        }
    }

    const double conservative_minimum = altitude_m
        - config.prediction_margin * (altitude_m - minimum_altitude);
    if (!std::isfinite(conservative_minimum))
    {
        return;
    }
    output.valid = true;
    output.predicted_bottom_valid = true;
    output.predicted_bottom_altitude_m = conservative_minimum;
    output.recovery_established = recovery_established;
    output.horizon_exhausted = !recovery_established;
}

void EvaluateEntryModel(
    const AutoGcasConfig& config,
    const AutoGcasEntryInput& input,
    const double altitude_m,
    const double climb_rate_mps,
    EntryReceipt& output) noexcept
{
    output = EntryReceipt{};
    if (!input.available_nz_valid)
    {
        return;
    }

    const double effective_nz_g = std::min(
        input.available_nz_g,
        config.pull_n_g);
    output.available = true;
    output.effective_nz_valid = true;
    output.effective_nz_g = effective_nz_g;
    output.boundary_breached = altitude_m <= config.crash_floor_m;
    if (effective_nz_g <= 1.0)
    {
        return;
    }
    output.recoverable = true;

    // A breached altitude boundary is an immediate recovery demand, not a
    // reason to discard the measured Nz authority.  Keep the entry on the
    // normal recovery path so RecoveryReference clamps the requested load to
    // effective_nz_g instead of manufacturing the configured maximum pull.
    if (output.boundary_breached)
    {
        output.should_activate = true;
        return;
    }

    const double descent_rate_mps = std::max(-climb_rate_mps, 0.0);
    if (descent_rate_mps == 0.0)
    {
        output.prediction.valid = true;
        output.prediction.predicted_bottom_valid = true;
        output.prediction.predicted_bottom_altitude_m = altitude_m;
        output.prediction.recovery_established = true;
        output.prediction.horizon_exhausted = false;
        output.should_activate = altitude_m <= config.min_altitude_m;
        return;
    }
    if (descent_rate_mps > input.ownship.speed_mps
            + ClimbRateSpeedToleranceMps)
    {
        output = EntryReceipt{};
        return;
    }

    const double sin_gamma = descent_rate_mps / input.ownship.speed_mps;
    const double cosine_gamma = std::sqrt(std::max(
        1.0 - sin_gamma * sin_gamma,
        0.0));
    const double latency_loss_m = descent_rate_mps * input.dt_s;
    const double roll_loss_m = descent_rate_mps
        * std::fabs(WrapRadians(input.ownship.rpy_rad[0]))
        / config.entry_effective_roll_rate_radps;
    const double onset_loss_m = descent_rate_mps
        * (effective_nz_g - 1.0) / config.onset_rate_gps;
    const double pull_loss_m = input.ownship.speed_mps
        * input.ownship.speed_mps * (1.0 - cosine_gamma)
        / (LadyLuck::constants::StandardGravityMps2
            * (effective_nz_g - 1.0));
    const double required_clearance_m = config.prediction_margin
        * (latency_loss_m + roll_loss_m + onset_loss_m + pull_loss_m);
    if (!std::isfinite(required_clearance_m))
    {
        output = EntryReceipt{};
        return;
    }

    PredictRecovery(
        config,
        altitude_m,
        input.ownship.speed_mps,
        -std::asin(sin_gamma),
        input.ownship.rpy_rad[0],
        input.roll_rate_endpoint_radps,
        input.roll_rate_endpoint_valid,
        input.measured_nz_g,
        input.measured_nz_valid,
        effective_nz_g,
        input.dt_s,
        output.prediction);
    const bool bottom_at_or_below_trigger =
        output.prediction.predicted_bottom_valid
        && (output.prediction.predicted_bottom_altitude_m
                <= config.min_altitude_m
            || std::fabs(
                output.prediction.predicted_bottom_altitude_m
                    - config.min_altitude_m)
                <= TriggerEqualityToleranceM);
    // Horizon exhaustion is diagnostic uncertainty, not terrain evidence.
    // Only an observed boundary breach (handled above) or a resolved predicted
    // bottom at/below the trigger may acquire the top-level recovery owner.
    output.should_activate = output.prediction.valid
        && output.prediction.predicted_bottom_valid
        && bottom_at_or_below_trigger;
}

void CopyEntryToReceipt(
    const EntryReceipt& entry,
    AutoGcasReceipt& output) noexcept
{
    output.entry_available = entry.available;
    output.entry_should_activate = entry.should_activate;
    output.entry_boundary_breached = entry.boundary_breached;
    output.entry_recoverable = entry.recoverable;
    output.prediction_valid = entry.prediction.valid;
    output.prediction_recovery_established =
        entry.prediction.recovery_established;
    output.prediction_horizon_exhausted =
        entry.prediction.horizon_exhausted;
    output.predicted_bottom_altitude_valid =
        entry.prediction.predicted_bottom_valid;
    output.predicted_bottom_altitude_m =
        entry.prediction.predicted_bottom_altitude_m;
    if (entry.effective_nz_valid)
    {
        output.recovery_load_factor_g = entry.effective_nz_g;
    }
}

void CopyEntryToPublicReceipt(
    const EntryReceipt& entry,
    AutoGcasEntryReceipt& output) noexcept
{
    output.entry_available = entry.available;
    output.entry_should_activate = entry.should_activate;
    output.entry_boundary_breached = entry.boundary_breached;
    output.entry_recoverable = entry.recoverable;
    output.effective_nz_valid = entry.effective_nz_valid;
    output.effective_nz_g = entry.effective_nz_g;
    output.prediction_valid = entry.prediction.valid;
    output.prediction_recovery_established =
        entry.prediction.recovery_established;
    output.prediction_horizon_exhausted =
        entry.prediction.horizon_exhausted;
    output.predicted_bottom_altitude_valid =
        entry.prediction.predicted_bottom_valid;
    output.predicted_bottom_altitude_m =
        entry.prediction.predicted_bottom_altitude_m;
}

void CopyPublicReceiptToEntry(
    const AutoGcasEntryReceipt& input,
    EntryReceipt& output) noexcept
{
    output = EntryReceipt{};
    output.available = input.entry_available;
    output.should_activate = input.entry_should_activate;
    output.boundary_breached = input.entry_boundary_breached;
    output.recoverable = input.entry_recoverable;
    output.effective_nz_valid = input.effective_nz_valid;
    output.effective_nz_g = input.effective_nz_g;
    output.prediction.valid = input.prediction_valid;
    output.prediction.recovery_established =
        input.prediction_recovery_established;
    output.prediction.horizon_exhausted =
        input.prediction_horizon_exhausted;
    output.prediction.predicted_bottom_valid =
        input.predicted_bottom_altitude_valid;
    output.prediction.predicted_bottom_altitude_m =
        input.predicted_bottom_altitude_m;
}

bool FiniteEntryInput(const AutoGcasEntryInput& input) noexcept
{
    return std::isfinite(input.t_sec)
        && std::isfinite(input.dt_s)
        && (!input.roll_rate_endpoint_valid
            || std::isfinite(input.roll_rate_endpoint_radps))
        && (!input.measured_nz_valid
            || std::isfinite(input.measured_nz_g))
        && (!input.available_nz_valid
            || std::isfinite(input.available_nz_g))
        && FiniteOwnship(input.ownship);
}

} // namespace

namespace LadyLuck
{
namespace safety
{

AutoGcas::AutoGcas() noexcept
    : AutoGcas(AutoGcasConfig{})
{
}

AutoGcas::AutoGcas(const AutoGcasConfig& config) noexcept
    : config_(config),
      configuration_valid_(ConfigurationValid(config))
{
    Reset();
}

void AutoGcas::CopyConfigurationValid(bool& output) const noexcept
{
    output = configuration_valid_;
}

void AutoGcas::Reset() noexcept
{
    state_ = AutoGcasSnapshot{};
}

void AutoGcas::EvaluateEntry(
    const AutoGcasEntryInput& input,
    AutoGcasEntryReceipt& output,
    Status& status) const noexcept
{
    output = AutoGcasEntryReceipt{};
    status = Status{};
    if (!configuration_valid_)
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }
    if (!FiniteEntryInput(input))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }
    if (input.t_sec < 0.0
        || input.dt_s <= 0.0
        || input.ownship.speed_mps <= 0.0)
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }

    const double climb_rate_mps = ClimbRate(input.ownship);
    const double altitude_m = -input.ownship.position_ned_m[2];
    if (!std::isfinite(climb_rate_mps) || !std::isfinite(altitude_m))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }

    EntryReceipt entry{};
    EvaluateEntryModel(
        config_,
        input,
        altitude_m,
        climb_rate_mps,
        entry);
    output.valid = true;
    output.frame_identity = input.estimator_frame_identity;
    output.evaluated_input = input;
    output.climb_rate_mps = climb_rate_mps;
    CopyEntryToPublicReceipt(entry, output);
}

void AutoGcas::BuildRecoveryCommand(
    const AutoGcasEntryReceipt& entry_receipt,
    AutoGcasReceipt& output,
    Status& status) noexcept
{
    output = AutoGcasReceipt{};
    status = Status{};
    if (!configuration_valid_)
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }
    output.recovery_load_factor_g = 0.0;
    output.phase = AutoGcasPhase::Inactive;
    output.frame_identity = entry_receipt.frame_identity;
    output.climb_rate_mps = entry_receipt.climb_rate_mps;
    if (!entry_receipt.valid)
    {
        // Missing optional prediction evidence cannot acquire top-level
        // terrain-recovery command authority.
        Reset();
        return;
    }
    EntryReceipt entry{};
    CopyPublicReceiptToEntry(entry_receipt, entry);
    CopyEntryToReceipt(entry, output);
    if (!config_.enabled)
    {
        return;
    }

    if (!entry.available || !entry.recoverable)
    {
        // Missing or physically insufficient current Nz authority is
        // command-neutral.  Neither condition proves that a 9-g pull is
        // achievable.  Publish no recovery command and release any stale
        // Auto-GCAS latch.  With usable authority, including
        // at the crash-floor boundary, the normal path below clamps recovery
        // to entry.effective_nz_g.
        Reset();
        return;
    }

    const AutoGcasEntryInput& evaluated = entry_receipt.evaluated_input;
    state_.latest_climb_rate_mps = entry_receipt.climb_rate_mps;
    const double wrapped_bank = WrapRadians(evaluated.ownship.rpy_rad[0]);
    const double gamma_rad = std::asin(Clamp(
        entry_receipt.climb_rate_mps / evaluated.ownship.speed_mps,
        -1.0,
        1.0));
    const bool wings_settled =
        std::fabs(wrapped_bank) <= config_.wings_level_gate_rad
        && evaluated.roll_rate_endpoint_valid
        && std::fabs(evaluated.roll_rate_endpoint_radps)
            <= config_.settled_roll_rate_gate_radps;
    const bool load_settled = evaluated.measured_nz_valid
        && std::fabs(evaluated.measured_nz_g - 1.0)
            <= config_.settle_nz_tolerance_g;
    const bool prediction_safe_to_release = entry.prediction.valid
        && entry.prediction.recovery_established
        && !entry.prediction.horizon_exhausted
        && entry.prediction.predicted_bottom_valid
        && entry.prediction.predicted_bottom_altitude_m
            > config_.min_altitude_m
        && -evaluated.ownship.position_ned_m[2]
            > config_.min_altitude_m;

    bool released_after_level_command = false;
    AutoGcasPhase command_phase = state_.phase;
    if (state_.active)
    {
        if (state_.phase == AutoGcasPhase::Pull
            && std::fabs(wrapped_bank) > 0.5 * constants::Pi
            && entry_receipt.climb_rate_mps < 0.0)
        {
            state_.phase = AutoGcasPhase::Roll;
            state_.has_pull_start_time = false;
            state_.pull_start_time_s = 0.0;
            state_.has_settle_candidate_time = false;
            state_.settle_candidate_time_s = 0.0;
        }
        if (state_.phase == AutoGcasPhase::Pull
            && state_.has_pull_start_time
            && evaluated.t_sec - state_.pull_start_time_s
                >= config_.minimum_pull_time_s
            && entry_receipt.climb_rate_mps >= 0.0)
        {
            state_.phase = AutoGcasPhase::Level;
            state_.has_settle_candidate_time = false;
            state_.settle_candidate_time_s = 0.0;
        }
        else if (state_.phase == AutoGcasPhase::Level)
        {
            if (gamma_rad < -config_.settle_gamma_rad)
            {
                state_.phase = AutoGcasPhase::Pull;
                state_.has_pull_start_time = true;
                state_.pull_start_time_s = evaluated.t_sec;
                state_.has_settle_candidate_time = false;
                state_.settle_candidate_time_s = 0.0;
            }
            else if (std::fabs(gamma_rad) <= config_.settle_gamma_rad
                && wings_settled
                && load_settled
                && prediction_safe_to_release)
            {
                if (!state_.has_settle_candidate_time)
                {
                    state_.has_settle_candidate_time = true;
                    state_.settle_candidate_time_s = evaluated.t_sec;
                }
                else if (evaluated.t_sec - state_.settle_candidate_time_s
                    >= config_.minimum_settle_time_s)
                {
                    command_phase = AutoGcasPhase::Level;
                    released_after_level_command = true;
                    state_.active = false;
                    state_.phase = AutoGcasPhase::Roll;
                    state_.has_pull_start_time = false;
                    state_.pull_start_time_s = 0.0;
                    state_.has_settle_candidate_time = false;
                    state_.settle_candidate_time_s = 0.0;
                }
            }
            else
            {
                state_.has_settle_candidate_time = false;
                state_.settle_candidate_time_s = 0.0;
            }
        }
    }
    else if (entry.should_activate)
    {
        state_.active = true;
        state_.phase = std::fabs(wrapped_bank) <= 0.5 * constants::Pi
            ? AutoGcasPhase::Pull
            : AutoGcasPhase::Roll;
        state_.has_pull_start_time = state_.phase == AutoGcasPhase::Pull;
        state_.pull_start_time_s = state_.has_pull_start_time
            ? evaluated.t_sec
            : 0.0;
        state_.has_settle_candidate_time = false;
        state_.settle_candidate_time_s = 0.0;
    }
    if (state_.active
        && state_.phase == AutoGcasPhase::Roll
        && std::fabs(wrapped_bank) <= 0.5 * constants::Pi)
    {
        state_.phase = AutoGcasPhase::Pull;
        state_.has_pull_start_time = true;
        state_.pull_start_time_s = evaluated.t_sec;
    }

    const bool command_required = state_.active || released_after_level_command;
    if (state_.active)
    {
        command_phase = state_.phase;
    }
    output.override_active = state_.active;
    output.phase = command_required ? command_phase : AutoGcasPhase::Inactive;
    if (!command_required)
    {
        return;
    }

    double nz_cmd_g = 0.0;
    double p_cmd_radps = 0.0;
    if (!RecoveryReference(
            config_,
            command_phase,
            evaluated.ownship.rpy_rad[0],
            state_.latest_climb_rate_mps,
            entry.effective_nz_g,
            nz_cmd_g,
            p_cmd_radps)
        || !SurfaceCommandFromReference(
            config_,
            command_phase,
            nz_cmd_g,
            p_cmd_radps,
            output.post_command))
    {
        status = Failure(StatusCode::InvalidConfiguration);
        return;
    }
    output.recovery_load_factor_g = nz_cmd_g;
}

void AutoGcas::CopySnapshot(AutoGcasSnapshot& output) const noexcept
{
    output = state_;
}

} // namespace safety
} // namespace LadyLuck
