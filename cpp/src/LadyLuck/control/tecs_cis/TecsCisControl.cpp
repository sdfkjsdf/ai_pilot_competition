#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"

#include "LadyLuck/common/Numerics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace LadyLuck
{
namespace control
{
namespace tecs_cis
{

namespace
{

constexpr double G = 9.80665;
constexpr double MetresPerFoot = 0.3048;
constexpr double PoundsForceToNewtons = 4.4482216;
constexpr double MilThrustN = 17800.0 * PoundsForceToNewtons;
constexpr double MaximumThrustN = 29000.0 * PoundsForceToNewtons;
constexpr double WingAreaM2 = 300.0 * MetresPerFoot * MetresPerFoot;
constexpr double GearDragCoefficient = 0.0270;
constexpr double ThrustArmM = -0.0553;
constexpr double PitchMomentSlopeResolution = 1.0e-3;
constexpr double RollRateNorm = 0.31821;
constexpr double MetresToFeet = 3.280839895;

#include "tables/TecsCisTables.inc"

// Every interpolator below reads both endpoints of one cell.  Make a
// generator/schema regression a compile-time failure instead of allowing a
// one-point table to reach count - 1 or lower + 1 on the control path.
static_assert(PitchNzCount >= 2U, "pitch Nz axis requires one cell");
static_assert(PitchMachCount >= 2U, "pitch Mach axis requires one cell");
static_assert(PitchAltitudeCount >= 2U, "pitch altitude axis requires one cell");
static_assert(CorrectionNzCount >= 2U, "correction Nz axis requires one cell");
static_assert(CorrectionMachCount >= 2U, "correction Mach axis requires one cell");
static_assert(CorrectionAltitudeCount >= 2U, "correction altitude axis requires one cell");
static_assert(EngineMachCount >= 2U, "engine Mach axis requires one cell");
static_assert(EngineIdleMachCount >= 2U, "idle Mach axis requires one cell");
static_assert(EngineMilMachCount >= 2U, "mil Mach axis requires one cell");
static_assert(EngineAltitudeCount >= 2U, "engine altitude axis requires one cell");
static_assert(DragAlphaCount >= 2U, "drag alpha axis requires one cell");
static_assert(DragElevatorCount >= 2U, "drag elevator axis requires one cell");
static_assert(DragMachCount >= 2U, "drag Mach axis requires one cell");

struct Cell
{
    std::size_t lower = 0U;
    double fraction = 0.0;
};

double Clip(const double value, const double lower, const double upper) noexcept
{
    return std::min(upper, std::max(lower, value));
}

Cell Locate(const double* const axis, const std::size_t count, const double raw) noexcept
{
    const double value = Clip(raw, axis[0], axis[count - 1U]);
    std::size_t upper = 1U;
    while (upper < count && axis[upper] < value)
    {
        ++upper;
    }
    if (upper >= count)
    {
        upper = count - 1U;
    }
    const std::size_t lower = upper - 1U;
    const double denominator = axis[upper] - axis[lower];
    Cell cell;
    cell.lower = lower;
    cell.fraction = denominator > 0.0
        ? Clip((value - axis[lower]) / denominator, 0.0, 1.0)
        : 0.0;
    return cell;
}

double Interpolate1d(
    const double* const axis,
    const double* const values,
    const std::size_t count,
    const double x) noexcept
{
    const Cell cell = Locate(axis, count, x);
    // np.interp evaluates y0 + t*(y1-y0), not a weighted two-term sum.
    return values[cell.lower] + cell.fraction
        * (values[cell.lower + 1U] - values[cell.lower]);
}

double Interpolate2d(
    const double* const row_axis,
    const std::size_t row_count,
    const double* const column_axis,
    const std::size_t column_count,
    const double* const values,
    const double row_value,
    const double column_value) noexcept
{
    const Cell row = Locate(row_axis, row_count, row_value);
    const Cell column = Locate(column_axis, column_count, column_value);
    const std::size_t r0 = row.lower;
    const std::size_t r1 = r0 + 1U;
    const std::size_t c0 = column.lower;
    const std::size_t c1 = c0 + 1U;
    const double d00 = values[r0 * column_count + c0];
    const double d10 = values[r1 * column_count + c0];
    const double d01 = values[r0 * column_count + c1];
    const double d11 = values[r1 * column_count + c1];
    const double one_minus_row = 1.0 - row.fraction;
    const double one_minus_column = 1.0 - column.fraction;
    return d00 * one_minus_row * one_minus_column
        + d10 * row.fraction * one_minus_column
        + d01 * one_minus_row * column.fraction
        + d11 * row.fraction * column.fraction;
}

double PitchTableValue(
    const std::array<double, PitchCellCount>& table,
    const std::size_t i,
    const std::size_t j,
    const std::size_t k) noexcept
{
    return table[(i * PitchMachCount + j) * PitchAltitudeCount + k];
}

double CorrectionTableValue(
    const std::size_t i,
    const std::size_t j,
    const std::size_t k) noexcept
{
    return CorrectionElevator[
        (i * CorrectionMachCount + j) * CorrectionAltitudeCount + k];
}

double PitchValue(
    const std::array<double, PitchCellCount>& table,
    const Cell& nz,
    const Cell& mach,
    const Cell& altitude) noexcept
{
    const std::size_t i = nz.lower;
    const std::size_t j = mach.lower;
    const std::size_t k = altitude.lower;
    const double i00 = PitchTableValue(table, i, j, k)
            * (1.0 - nz.fraction)
        + PitchTableValue(table, i + 1U, j, k) * nz.fraction;
    const double i01 = PitchTableValue(table, i, j, k + 1U)
            * (1.0 - nz.fraction)
        + PitchTableValue(table, i + 1U, j, k + 1U) * nz.fraction;
    const double i10 = PitchTableValue(table, i, j + 1U, k)
            * (1.0 - nz.fraction)
        + PitchTableValue(table, i + 1U, j + 1U, k) * nz.fraction;
    const double i11 = PitchTableValue(table, i, j + 1U, k + 1U)
            * (1.0 - nz.fraction)
        + PitchTableValue(table, i + 1U, j + 1U, k + 1U)
            * nz.fraction;
    const double j0 = i00 * (1.0 - mach.fraction) + i10 * mach.fraction;
    const double j1 = i01 * (1.0 - mach.fraction) + i11 * mach.fraction;
    return j0 * (1.0 - altitude.fraction) + j1 * altitude.fraction;
}

double CorrectionValue(
    const double nz_value,
    const double mach_value,
    const double altitude_value) noexcept
{
    const Cell nz = Locate(CorrectionNzAxis.data(), CorrectionNzCount, nz_value);
    const Cell mach = Locate(CorrectionMachAxis.data(), CorrectionMachCount, mach_value);
    const Cell altitude = Locate(
        CorrectionAltitudeAxis.data(), CorrectionAltitudeCount, altitude_value);
    const std::size_t i = nz.lower;
    const std::size_t j = mach.lower;
    const std::size_t k = altitude.lower;
    const double i00 = CorrectionTableValue(i, j, k)
            * (1.0 - nz.fraction)
        + CorrectionTableValue(i + 1U, j, k) * nz.fraction;
    const double i01 = CorrectionTableValue(i, j, k + 1U)
            * (1.0 - nz.fraction)
        + CorrectionTableValue(i + 1U, j, k + 1U) * nz.fraction;
    const double i10 = CorrectionTableValue(i, j + 1U, k)
            * (1.0 - nz.fraction)
        + CorrectionTableValue(i + 1U, j + 1U, k) * nz.fraction;
    const double i11 = CorrectionTableValue(i, j + 1U, k + 1U)
            * (1.0 - nz.fraction)
        + CorrectionTableValue(i + 1U, j + 1U, k + 1U)
            * nz.fraction;
    const double j0 = i00 * (1.0 - mach.fraction) + i10 * mach.fraction;
    const double j1 = i01 * (1.0 - mach.fraction) + i11 * mach.fraction;
    return j0 * (1.0 - altitude.fraction) + j1 * altitude.fraction;
}

struct ThrustBounds
{
    double idle = 0.0;
    double mil = 0.0;
    double maximum = 0.0;
};

ThrustBounds EngineThrustBounds(const double mach, const double altitude_m) noexcept
{
    const double altitude_ft = altitude_m / MetresPerFoot;
    const double idle_coefficient = Interpolate2d(
        EngineMachAxis.data(), EngineIdleMachCount,
        EngineAltitudeAxisFt.data(), EngineAltitudeCount,
        EngineIdle.data(), mach, altitude_ft);
    const double mil_coefficient = Interpolate2d(
        EngineMachAxis.data(), EngineMilMachCount,
        EngineAltitudeAxisFt.data(), EngineAltitudeCount,
        EngineMil.data(), mach, altitude_ft);
    const double augmented_coefficient = Interpolate2d(
        EngineMachAxis.data(), EngineMachCount,
        EngineAltitudeAxisFt.data(), EngineAltitudeCount,
        EngineAugmented.data(), mach, altitude_ft);
    ThrustBounds bounds;
    bounds.idle = MilThrustN * idle_coefficient;
    bounds.mil = bounds.idle + (MilThrustN - bounds.idle) * mil_coefficient;
    bounds.maximum = MaximumThrustN * augmented_coefficient;
    return bounds;
}

double StaticThrustFromInternalThrottle(
    const double f_cis,
    const double mach,
    const double altitude_m) noexcept
{
    const ThrustBounds bounds = EngineThrustBounds(mach, altitude_m);
    const double position = Clip((f_cis - 0.5) / 0.25, 0.0, 2.0);
    if (position <= 1.0)
    {
        return bounds.idle
            + position * position * (bounds.mil - bounds.idle);
    }
    return bounds.mil
        + (position - 1.0) * (bounds.maximum - bounds.mil);
}

double InternalThrottleFromThrust(
    const double thrust_n,
    const ThrustBounds& bounds) noexcept
{
    const double limited = Clip(thrust_n, bounds.idle, bounds.maximum);
    double position = 0.0;
    if (limited <= bounds.mil)
    {
        position = bounds.mil > bounds.idle
            ? std::sqrt(Clip(
                (limited - bounds.idle) / (bounds.mil - bounds.idle),
                0.0,
                1.0))
            : 0.0;
    }
    else
    {
        const double augmentation = bounds.maximum > bounds.mil
            ? Clip(
                (limited - bounds.mil) / (bounds.maximum - bounds.mil),
                0.0,
                1.0)
            : 0.0;
        position = 1.0 + augmentation;
    }
    return Clip((position + 2.0) / 4.0, 0.5, 1.0);
}

double DragCoefficient(
    const double alpha_rad,
    const double mach,
    const double elevator_rad,
    const double gear_pos_norm) noexcept
{
    const double base = Interpolate2d(
        DragAlphaAxis.data(), DragAlphaCount,
        DragElevatorAxis.data(), DragElevatorCount,
        DragBase.data(), alpha_rad, elevator_rad);
    const double wave = Interpolate1d(
        DragMachAxis.data(), DragMach.data(), DragMachCount, mach);
    return base + wave + GearDragCoefficient * gear_pos_norm;
}

double AtmosphereTemperature(const double altitude_m) noexcept
{
    const double altitude = std::max(altitude_m, 0.0);
    return altitude <= 11000.0
        ? 288.15 - 0.0065 * altitude
        : 216.65;
}

double SpeedOfSound(const double temperature_k) noexcept
{
    return std::sqrt(1.4 * 287.05 * temperature_k);
}

double NzMaximum(
    const Cell& mach,
    const Cell& altitude) noexcept
{
    const std::size_t j = mach.lower;
    const std::size_t k = altitude.lower;
    const double v00 = PitchNzMaximum[j * PitchAltitudeCount + k];
    const double v10 = PitchNzMaximum[(j + 1U) * PitchAltitudeCount + k];
    const double v01 = PitchNzMaximum[j * PitchAltitudeCount + k + 1U];
    const double v11 = PitchNzMaximum[(j + 1U) * PitchAltitudeCount + k + 1U];
    return v00 * (1.0 - mach.fraction) * (1.0 - altitude.fraction)
        + v10 * mach.fraction * (1.0 - altitude.fraction)
        + v01 * (1.0 - mach.fraction) * altitude.fraction
        + v11 * mach.fraction * altitude.fraction;
}

struct PitchLookup
{
    double mach = 0.0;
    double alpha = 0.0;
    double elevator = 0.0;
};

PitchLookup LookupPitch(
    const double nz_cmd_g,
    const double speed_mps,
    const double altitude_m,
    const double mass_kg,
    const double f_cis) noexcept
{
    PitchLookup output;
    output.mach = speed_mps / SpeedOfSound(AtmosphereTemperature(altitude_m));
    const Cell mach = Locate(PitchMachAxis.data(), PitchMachCount, output.mach);
    const Cell altitude = Locate(
        PitchAltitudeAxis.data(), PitchAltitudeCount, altitude_m);
    double nz_effective = nz_cmd_g * mass_kg / PitchMassReferenceKg;
    nz_effective = std::min(nz_effective, NzMaximum(mach, altitude));
    const Cell nz = Locate(PitchNzAxis.data(), PitchNzCount, nz_effective);
    output.alpha = PitchValue(PitchAlpha, nz, mach, altitude);
    output.elevator = PitchValue(PitchElevator, nz, mach, altitude);
    const double derivative = PitchValue(
        PitchMomentDerivative, nz, mach, altitude);
    const double reference_thrust = Interpolate2d(
        PitchMachAxis.data(), PitchMachCount,
        PitchAltitudeAxis.data(), PitchAltitudeCount,
        PitchReferenceThrust.data(), output.mach, altitude_m);
    const double thrust = StaticThrustFromInternalThrottle(
        f_cis, output.mach, altitude_m);
    output.elevator += ThrustArmM * (reference_thrust - thrust)
        * numerics::RegularizedSignedInverse(
            derivative,
            PitchMomentSlopeResolution);
    output.elevator += CorrectionValue(
        nz_effective, output.mach, altitude_m);
    return output;
}

double SchedulerGain(const double alpha_rad) noexcept
{
    constexpr double axis[5] = {-0.5236, -0.5, 0.0, 0.5, 0.5236};
    constexpr double values[5] = {0.0, 0.11, 1.0, 0.11, 0.0};
    return Interpolate1d(axis, values, 5U, alpha_rad);
}

double YawScheduler(const double ground_speed_fps) noexcept
{
    constexpr double axis[3] = {80.0, 100.0, 150.0};
    constexpr double values[3] = {0.0, 15.0, 100.0};
    return Interpolate1d(axis, values, 3U, ground_speed_fps);
}

NormalizedControlCommand Float32Command(
    const double aileron,
    const double elevator,
    const double rudder,
    const double throttle) noexcept
{
    NormalizedControlCommand output;
    output.aileron = static_cast<double>(static_cast<float>(aileron));
    output.elevator = static_cast<double>(static_cast<float>(elevator));
    output.rudder = static_cast<double>(static_cast<float>(rudder));
    output.throttle = static_cast<double>(static_cast<float>(throttle));
    return output;
}

bool CommandFinite(const NormalizedControlCommand& command) noexcept
{
    return std::isfinite(command.aileron)
        && std::isfinite(command.elevator)
        && std::isfinite(command.rudder)
        && std::isfinite(command.throttle);
}

bool EnvelopeContractValid(
    const control::route5::CommandEnvelope& envelope) noexcept
{
    return control::route5::CommandEnvelopeSourceProvidesBounds(
            envelope.source)
        && std::isfinite(envelope.nz_feasible_g)
        && std::isfinite(envelope.nz_min_g)
        && std::isfinite(envelope.p_max_radps)
        && envelope.nz_feasible_g > 0.0
        && envelope.nz_min_g <= envelope.nz_feasible_g
        && envelope.p_max_radps > 0.0;
}

bool ConfigValid(const TecsCisConfig& config) noexcept
{
    const double values[] = {
        config.energy_error_gain_per_s,
        config.energy_integral_gain_per_s2,
        config.energy_rate_feedback_gain,
        config.antiwindup_gain_per_s,
        config.integral_error_limit_m2ps,
        config.minimum_speed_mps,
        config.minimum_thrust_velocity_projection,
        config.speed_command_rate_min_mps2,
        config.speed_command_rate_max_mps2,
        config.energy_rate_bias_min_m2ps3,
        config.energy_rate_bias_max_m2ps3,
        config.p_command_limit_radps,
        config.nz_command_min_g,
        config.nz_command_max_g,
        config.stateless_flaperon_pitch_gain_per_g,
        config.stateless_flaperon_pitch_bias_limit
    };
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return config.energy_error_gain_per_s >= 0.0
        && config.energy_integral_gain_per_s2 >= 0.0
        && config.energy_rate_feedback_gain >= 0.0
        && config.antiwindup_gain_per_s >= 0.0
        && config.integral_error_limit_m2ps >= 0.0
        && config.minimum_speed_mps > 0.0
        && config.minimum_thrust_velocity_projection > 0.0
        && config.minimum_thrust_velocity_projection <= 1.0
        && config.speed_command_rate_min_mps2
            <= config.speed_command_rate_max_mps2
        && config.energy_rate_bias_min_m2ps3
            <= config.energy_rate_bias_max_m2ps3
        && config.p_command_limit_radps > 0.0
        && config.nz_command_min_g <= config.nz_command_max_g
        && config.stateless_flaperon_pitch_bias_limit >= 0.0;
}

} // namespace

TecsCisControl::TecsCisControl() noexcept
    : TecsCisControl(TecsCisConfig())
{
}

TecsCisControl::TecsCisControl(const TecsCisConfig& config) noexcept
    : config_(config),
      configuration_valid_(ConfigValid(config))
{
}

void TecsCisControl::CopyConfigurationValid(bool& output) const noexcept
{
    output = configuration_valid_;
}

void TecsCisControl::CopyLongitudinalAuthorityConfiguration(
    TecsCisLongitudinalAuthorityConfiguration& output,
    Status& status) const noexcept
{
    output = TecsCisLongitudinalAuthorityConfiguration{};
    status = Status{};
    if (!configuration_valid_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.valid = true;
    output.continuous_total_energy_controller = true;
    output.energy_error_gain_per_s = config_.energy_error_gain_per_s;
    output.energy_integral_gain_per_s2 = config_.energy_integral_gain_per_s2;
    output.energy_rate_feedback_gain = config_.energy_rate_feedback_gain;
    output.minimum_speed_mps = config_.minimum_speed_mps;
    output.speed_command_rate_min_mps2 =
        config_.speed_command_rate_min_mps2;
    output.speed_command_rate_max_mps2 =
        config_.speed_command_rate_max_mps2;
}

void TecsCisControl::Reset() noexcept
{
    energy_integral_error_m2ps_ = 0.0;
    accepted_step_count_ = 0U;
}

void TecsCisControl::CopySnapshot(TecsCisSnapshot& output) const noexcept
{
    output = TecsCisSnapshot{};
    output.energy_integral_error_m2ps = energy_integral_error_m2ps_;
    output.accepted_step_count = accepted_step_count_;
}

void TecsCisControl::Preview(
    const BodyRateLoadEnergyCommand& reference,
    const EstimatorOutputV6& estimate,
    const control::route5::CommandEnvelope& envelope,
    const double dt_s,
    TecsCisOutput& output,
    TecsCisSnapshot& next_snapshot,
    Status& status) const noexcept
{
    TecsCisControl projected(*this);
    projected.Step(
        reference,
        estimate,
        envelope,
        dt_s,
        output,
        status);
    projected.CopySnapshot(next_snapshot);
}

void TecsCisControl::Step(
    const BodyRateLoadEnergyCommand& reference,
    const EstimatorOutputV6& estimate,
    const control::route5::CommandEnvelope& envelope,
    const double dt_s,
    TecsCisOutput& output,
    Status& status) noexcept
{
    output = TecsCisOutput{};
    status = Status{};
    if (!configuration_valid_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!reference.valid)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!EnvelopeContractValid(envelope))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!std::isfinite(dt_s) || dt_s <= 0.0)
    {
        status.code = StatusCode::InvalidDt;
        return;
    }
    const double reference_values[] = {
        reference.p_cmd_radps,
        reference.q_cmd_radps,
        reference.r_cmd_radps,
        reference.nz_cmd_g,
        reference.desired_speed_mps,
        reference.desired_speed_rate_mps2,
        reference.flight_path_angle_cmd_rad,
        reference.specific_energy_rate_bias_m2ps3
    };
    const double estimate_values[] = {
        estimate.V, estimate.alt, estimate.mass, estimate.u, estimate.v,
        estimate.w, estimate.roll, estimate.pitch, estimate.alpha,
        estimate.beta, estimate.mach, estimate.qbar, estimate.gear_pos_norm
    };
    for (const double value : reference_values)
    {
        if (!std::isfinite(value))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    for (const double value : estimate_values)
    {
        if (!std::isfinite(value))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    const double envelope_values[] = {
        envelope.nz_feasible_g,
        envelope.nz_min_g,
        envelope.p_max_radps};
    for (const double value : envelope_values)
    {
        if (!std::isfinite(value))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    const double p_command_limit = std::min(
        config_.p_command_limit_radps,
        envelope.p_max_radps);
    const double nz_command_min = std::max(
        config_.nz_command_min_g,
        envelope.nz_min_g);
    const double nz_command_max = std::min(
        config_.nz_command_max_g,
        envelope.nz_feasible_g);
    if (!(p_command_limit > 0.0)
        || nz_command_min > nz_command_max)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (reference.desired_speed_mps < 0.0
        || estimate.V <= 0.0 || estimate.mass <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const double speed = std::max(
        estimate.V,
        numerics::CisPairLongitudinalSpeedRegularizationMps);
    const double elevator_rad = estimate.elevator_valid
            && std::isfinite(estimate.elevator_rad)
        ? estimate.elevator_rad
        : 0.0;
    // Gear position is a bounded physical actuator fraction.  A finite
    // estimator overshoot must saturate at the actuator boundary instead of
    // erasing an otherwise finite current-frame FCS command.
    const double gear_position_norm = Clip(estimate.gear_pos_norm, 0.0, 1.0);
    const double drag = std::max(
        0.0,
        estimate.qbar * WingAreaM2 * DragCoefficient(
            estimate.alpha,
            estimate.mach,
            elevator_rad,
            gear_position_norm));
    const double projection_raw = std::cos(estimate.alpha)
        * std::cos(estimate.beta);
    const double projection = Clip(
        projection_raw,
        config_.minimum_thrust_velocity_projection,
        1.0);
    const ThrustBounds bounds = EngineThrustBounds(estimate.mach, estimate.alt);
    if (!std::isfinite(drag) || !std::isfinite(projection)
        || !std::isfinite(bounds.idle) || !std::isfinite(bounds.maximum)
        || bounds.idle > bounds.maximum)
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const double sin_pitch = std::sin(estimate.pitch);
    const double cos_pitch = std::cos(estimate.pitch);
    const double sin_roll = std::sin(estimate.roll);
    const double cos_roll = std::cos(estimate.roll);
    const double altitude_rate = estimate.u * sin_pitch
        - estimate.v * sin_roll * cos_pitch
        - estimate.w * cos_roll * cos_pitch;
    const double gamma = std::asin(Clip(altitude_rate / speed, -1.0, 1.0));
    const double speed_rate_cmd = Clip(
        reference.desired_speed_rate_mps2,
        config_.speed_command_rate_min_mps2,
        config_.speed_command_rate_max_mps2);
    const double altitude_rate_cmd = speed
        * std::sin(reference.flight_path_angle_cmd_rad);
    const double rate_bias = Clip(
        reference.specific_energy_rate_bias_m2ps3,
        config_.energy_rate_bias_min_m2ps3,
        config_.energy_rate_bias_max_m2ps3);
    const double kinetic = 0.5 * speed * speed;
    const double potential = G * estimate.alt;
    const double kinetic_cmd = 0.5 * reference.desired_speed_mps
        * reference.desired_speed_mps;
    const double potential_cmd = G * estimate.alt;
    const double total = kinetic + potential;
    const double total_cmd = kinetic_cmd + potential_cmd;
    const double total_error = total_cmd - total;
    const double rate_reference = reference.desired_speed_mps * speed_rate_cmd
        + G * altitude_rate_cmd
        + rate_bias;
    const bool rate_measurement_valid = estimate.thrust_valid
        && std::isfinite(estimate.thrust);
    const double rate_measured_raw = rate_measurement_valid
        ? speed * (projection * estimate.thrust - drag)
            / std::max(estimate.mass, 1.0)
        : rate_reference;
    const double rate_error = rate_reference - rate_measured_raw;
    const double integral_before = energy_integral_error_m2ps_;
    const double rate_command = rate_reference
        + config_.energy_error_gain_per_s * total_error
        + config_.energy_integral_gain_per_s2 * integral_before
        + config_.energy_rate_feedback_gain * rate_error;
    const double speed_for_inverse = std::max(speed, config_.minimum_speed_mps);
    const double thrust_raw = (drag
        + estimate.mass * rate_command / speed_for_inverse) / projection;
    const double thrust_limited = Clip(thrust_raw, bounds.idle, bounds.maximum);
    const bool lower_saturated = thrust_raw < bounds.idle;
    const bool upper_saturated = thrust_raw > bounds.maximum;
    if (!std::isfinite(rate_reference) || !std::isfinite(rate_measured_raw)
        || !std::isfinite(rate_command) || !std::isfinite(thrust_raw)
        || !std::isfinite(thrust_limited))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const bool integral_drive_blocked =
        (upper_saturated && total_error > 0.0)
        || (lower_saturated && total_error < 0.0);
    double integral_dot = reference.integrator_hold || integral_drive_blocked
        ? 0.0
        : total_error;
    const bool integral_unload = config_.antiwindup_gain_per_s > 0.0
        && !reference.integrator_hold
        && ((upper_saturated && integral_before > 0.0)
            || (lower_saturated && integral_before < 0.0));
    if (integral_unload)
    {
        integral_dot -= config_.antiwindup_gain_per_s * integral_before;
    }
    const double integral_after = Clip(
        integral_before + dt_s * integral_dot,
        -config_.integral_error_limit_m2ps,
        config_.integral_error_limit_m2ps);
    if (!std::isfinite(integral_after))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    // Python commits TECS before pitch inversion and final safety filtering.
    // Preserve that staged ownership: a late numerical fallback keeps this
    // integral update instead of rolling the whole control tick back.
    energy_integral_error_m2ps_ = integral_after;

    const double f_cis = InternalThrottleFromThrust(thrust_limited, bounds);
    const double nz_cmd = Clip(
        reference.nz_cmd_g,
        nz_command_min,
        nz_command_max);
    const PitchLookup pitch = LookupPitch(
        nz_cmd,
        speed,
        estimate.alt,
        estimate.mass,
        f_cis);
    const double scheduler = SchedulerGain(pitch.alpha);
    const double gravity_correction = std::cos(estimate.pitch)
        * std::cos(estimate.roll);
    double pitch_inverse = 0.0;
    if (scheduler < 0.05)
    {
        pitch_inverse = nz_cmd > 1.0 ? -1.0 : 0.0;
    }
    else
    {
        pitch_inverse = (
            pitch.elevator / 0.436
            - 1.0472 * pitch.alpha
            - 1.86 * reference.q_cmd_radps
            + 0.006 * (-nz_cmd - gravity_correction)
        ) / (1.3 * scheduler);
    }
    pitch_inverse = Clip(pitch_inverse, -1.0, 0.44);
    double flaperon_bias = 0.0;
    if (estimate.nz_flaperon_valid && std::isfinite(estimate.nz_flaperon))
    {
        flaperon_bias = Clip(
            config_.stateless_flaperon_pitch_gain_per_g
                * estimate.nz_flaperon,
            -config_.stateless_flaperon_pitch_bias_limit,
            config_.stateless_flaperon_pitch_bias_limit);
    }
    const double pitch_unclipped = pitch_inverse + flaperon_bias;
    const double p_cmd = Clip(
        reference.p_cmd_radps,
        -p_command_limit,
        p_command_limit);
    const double ground_speed = std::isfinite(estimate.ground_speed_horizontal_mps)
        ? estimate.ground_speed_horizontal_mps
        : speed;
    const double aileron = Clip(RollRateNorm * p_cmd, -1.0, 1.0);
    const double elevator = Clip(pitch_unclipped, -1.0, 0.44);
    const double rudder = Clip(
        -(YawScheduler(ground_speed * MetresToFeet)
            * reference.r_cmd_radps),
        -1.0,
        1.0);
    const double throttle = Clip(2.0 * f_cis - 1.0, -1.0, 1.0);
    NormalizedControlCommand command = Float32Command(
        aileron, elevator, rudder, throttle);

    const double final_values[] = {
        drag,
        projection,
        gamma,
        rate_reference,
        rate_measured_raw,
        rate_command,
        thrust_raw,
        thrust_limited,
        f_cis,
        integral_before,
        integral_after,
        pitch.mach,
        pitch.alpha,
        pitch.elevator,
        scheduler,
        flaperon_bias,
        pitch_unclipped};
    bool final_finite = CommandFinite(command);
    for (const double value : final_values)
    {
        final_finite = final_finite && std::isfinite(value);
    }
    if (!final_finite)
    {
        output = TecsCisOutput{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    command.valid = true;
    output.frame_identity = reference.frame_identity;
    output.valid = true;
    output.command = command;
    output.diagnostics.drag_estimate_n = drag;
    output.diagnostics.thrust_velocity_projection = projection;
    output.diagnostics.flight_path_angle_rad = gamma;
    output.diagnostics.specific_energy_rate_reference_m2ps3 = rate_reference;
    output.diagnostics.specific_energy_rate_measured_m2ps3 = rate_measured_raw;
    output.diagnostics.specific_energy_rate_command_m2ps3 = rate_command;
    output.diagnostics.thrust_cmd_raw_n = thrust_raw;
    output.diagnostics.thrust_cmd_limited_n = thrust_limited;
    output.diagnostics.internal_throttle_f_cis = f_cis;
    output.diagnostics.energy_integral_before_m2ps = integral_before;
    output.diagnostics.energy_integral_after_m2ps = integral_after;
    output.diagnostics.trim_mach = pitch.mach;
    output.diagnostics.alpha_required_rad = pitch.alpha;
    output.diagnostics.elevator_required_rad = pitch.elevator;
    output.diagnostics.scheduler_gain = scheduler;
    output.diagnostics.stateless_flaperon_pitch_bias = flaperon_bias;
    output.diagnostics.pitch_command_unclipped = pitch_unclipped;
    output.diagnostics.lower_thrust_saturated = lower_saturated;
    output.diagnostics.upper_thrust_saturated = upper_saturated;
    output.diagnostics.rate_measurement_valid = rate_measurement_valid;
    output.completed_energy_authority.source_frame_identity =
        reference.frame_identity;
    output.completed_energy_authority.valid = true;
    output.completed_energy_authority.continuous_total_energy_controller =
        true;
    output.completed_energy_authority.rate_measurement_valid =
        rate_measurement_valid;
    output.completed_energy_authority.controller_configuration_available =
        configuration_valid_;
    output.completed_energy_authority.energy_error_gain_per_s =
        config_.energy_error_gain_per_s;
    output.completed_energy_authority.energy_integral_gain_per_s2 =
        config_.energy_integral_gain_per_s2;
    output.completed_energy_authority.energy_rate_feedback_gain =
        config_.energy_rate_feedback_gain;
    output.completed_energy_authority.total_energy_error_m2ps2 = total_error;
    output.completed_energy_authority.energy_integral_error_m2ps =
        integral_after;
    output.completed_energy_authority.specific_energy_rate_measured_m2ps3 =
        rate_measured_raw;
    output.completed_energy_authority.speed_mps = speed;
    output.completed_energy_authority.minimum_speed_mps =
        config_.minimum_speed_mps;
    output.completed_energy_authority.mass_kg = estimate.mass;
    output.completed_energy_authority.drag_estimate_n = drag;
    output.completed_energy_authority.thrust_velocity_projection = projection;
    output.completed_energy_authority.thrust_min_n = bounds.idle;
    output.completed_energy_authority.thrust_max_n = bounds.maximum;
    ++accepted_step_count_;
    status.code = StatusCode::Ok;
}

} // namespace tecs_cis
} // namespace control
} // namespace LadyLuck
