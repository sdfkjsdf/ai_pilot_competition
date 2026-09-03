#include "LadyLuck/guidance/obfm/ObfmLongitudinalReferenceProvider.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{

constexpr double AirGasConstantJpkgK = 287.05;
constexpr double AirGamma = 1.4;
constexpr double GovernorReferenceMassKg = 11159.0;
constexpr double EmConflictAltitude0M = 2500.0;
constexpr double EmConflictAltitude1M = 8500.0;
constexpr double EmConflictAltitude2M = 12000.0;

// Both generated files are immutable d90 table translations.  Including them
// in this translation unit retains their internal linkage and avoids a second
// hand-copied tactical data source.
#include "../../control/route5/envelope/ReferenceGovernorTable.inc"
#include "../em/EnergyManeuverTables.generated.inc"

static_assert(
    ReferenceGovernorMachCount >= 2U
        && ReferenceGovernorAltitudeCount >= 2U,
    "Governor interpolation axes require at least two nodes.");
static_assert(
    ReferenceGovernorMachAxis.size() == ReferenceGovernorMachCount
        && ReferenceGovernorAltitudeAxis.size()
            == ReferenceGovernorAltitudeCount
        && ReferenceGovernorNzFeasible.size()
            == ReferenceGovernorMachCount
                * ReferenceGovernorAltitudeCount,
    "Governor table dimensions must match the frozen schema.");
static_assert(
    kEmSpeedCount >= 2U && kEmAltitudeCount >= 2U,
    "E-M interpolation axes require at least two nodes.");
static_assert(
    kEmGridCount == kEmSpeedCount * kEmAltitudeCount
        && sizeof(kEmSpeedAxisMps) / sizeof(kEmSpeedAxisMps[0U])
            == kEmSpeedCount
        && sizeof(kEmAltitudeAxisM) / sizeof(kEmAltitudeAxisM[0U])
            == kEmAltitudeCount
        && sizeof(kEmNInstPubG) / sizeof(kEmNInstPubG[0U])
            == kEmGridCount
        && sizeof(kEmCornerInstantaneousMps)
                / sizeof(kEmCornerInstantaneousMps[0U])
            == kEmAltitudeCount
        && sizeof(kEmCornerSustainedMps)
                / sizeof(kEmCornerSustainedMps[0U])
            == kEmAltitudeCount,
    "E-M table dimensions must match the frozen schema.");

bool Finite(const double value) noexcept
{
    return std::isfinite(value);
}

bool PositiveFinite(const double value) noexcept
{
    return Finite(value) && value > 0.0;
}

double Clamp(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::min(upper, std::max(lower, value));
}

bool StrictlyIncreasingFiniteAxis(
    const double* const axis,
    const std::size_t count) noexcept
{
    if (axis == nullptr || count < 2U || !Finite(axis[0U]))
    {
        return false;
    }
    for (std::size_t index = 1U; index < count; ++index)
    {
        if (!Finite(axis[index]) || !(axis[index - 1U] < axis[index]))
        {
            return false;
        }
    }
    return true;
}

bool LeftCellIndex(
    const double* axis,
    const std::size_t count,
    const double value,
    std::size_t& output) noexcept
{
    output = 0U;
    if (!Finite(value) || !StrictlyIncreasingFiniteAxis(axis, count))
    {
        return false;
    }
    const double* const position = std::lower_bound(
        axis,
        axis + count,
        value);
    if (position == axis)
    {
        return true;
    }
    if (position == axis + count)
    {
        output = count - 2U;
        return true;
    }
    output = static_cast<std::size_t>(position - axis - 1);
    return true;
}

bool RightBracketLowerIndex(
    const double* axis,
    const std::size_t count,
    const double value,
    std::size_t& output) noexcept
{
    output = 0U;
    if (!Finite(value) || !StrictlyIncreasingFiniteAxis(axis, count))
    {
        return false;
    }
    const double* const position = std::upper_bound(
        axis,
        axis + count,
        value);
    if (position == axis)
    {
        return true;
    }
    const std::size_t raw = static_cast<std::size_t>(position - axis - 1);
    output = std::min(raw, count - 1U);
    return true;
}

bool Bilinear(
    const double* speed_axis,
    const std::size_t speed_count,
    const double* altitude_axis,
    const std::size_t altitude_count,
    const double* field,
    const double requested_speed,
    const double requested_altitude,
    double& output) noexcept
{
    output = 0.0;
    if (field == nullptr
        || !Finite(requested_speed)
        || !Finite(requested_altitude)
        || !StrictlyIncreasingFiniteAxis(speed_axis, speed_count)
        || !StrictlyIncreasingFiniteAxis(altitude_axis, altitude_count))
    {
        return false;
    }
    const double speed = Clamp(
        requested_speed,
        speed_axis[0U],
        speed_axis[speed_count - 1U]);
    const double altitude = Clamp(
        requested_altitude,
        altitude_axis[0U],
        altitude_axis[altitude_count - 1U]);
    std::size_t i = 0U;
    std::size_t j = 0U;
    if (!LeftCellIndex(speed_axis, speed_count, speed, i)
        || !LeftCellIndex(
            altitude_axis,
            altitude_count,
            altitude,
            j))
    {
        return false;
    }
    const double tv = (speed - speed_axis[i])
        / (speed_axis[i + 1U] - speed_axis[i]);
    const double th = (altitude - altitude_axis[j])
        / (altitude_axis[j + 1U] - altitude_axis[j]);
    const std::size_t row0 = i * altitude_count;
    const std::size_t row1 = (i + 1U) * altitude_count;
    const double corners[] = {
        field[row0 + j],
        field[row1 + j],
        field[row0 + j + 1U],
        field[row1 + j + 1U]};
    for (const double corner : corners)
    {
        if (!Finite(corner))
        {
            return false;
        }
    }
    output =
        field[row0 + j] * (1.0 - tv) * (1.0 - th)
        + field[row1 + j] * tv * (1.0 - th)
        + field[row0 + j + 1U] * (1.0 - tv) * th
        + field[row1 + j + 1U] * tv * th;
    return Finite(output);
}

bool LinearInterpolate(
    const double* axis,
    const double* field,
    const std::size_t count,
    const double value,
    double& output) noexcept
{
    output = 0.0;
    if (field == nullptr
        || !Finite(value)
        || !StrictlyIncreasingFiniteAxis(axis, count))
    {
        return false;
    }
    if (value <= axis[0U])
    {
        output = field[0U];
        return Finite(output);
    }
    if (value >= axis[count - 1U])
    {
        output = field[count - 1U];
        return Finite(output);
    }
    const double* const right = std::lower_bound(
        axis,
        axis + count,
        value);
    const std::size_t right_index =
        static_cast<std::size_t>(right - axis);
    if (*right == value)
    {
        output = field[right_index];
        return Finite(output);
    }
    const std::size_t left_index = right_index - 1U;
    if (!Finite(field[left_index]) || !Finite(field[right_index]))
    {
        return false;
    }
    const double slope =
        (field[right_index] - field[left_index])
        / (axis[right_index] - axis[left_index]);
    output = slope * (value - axis[left_index]) + field[left_index];
    return Finite(output);
}

double AtmosphereTemperatureK(const double altitude_m) noexcept
{
    return altitude_m <= 11000.0
        ? 288.15 - 0.0065 * altitude_m
        : 216.65;
}

double SoundSpeedMps(const double altitude_m) noexcept
{
    return std::sqrt(
        AirGamma * AirGasConstantJpkgK
        * AtmosphereTemperatureK(altitude_m));
}

bool GovernorNzAtMach(
    const double mach,
    const double altitude_m,
    const double mass_kg,
    double& output) noexcept
{
    output = 0.0;
    double value = 0.0;
    if (!Finite(mass_kg)
        || !Bilinear(
            ReferenceGovernorMachAxis.data(),
            ReferenceGovernorMachCount,
            ReferenceGovernorAltitudeAxis.data(),
            ReferenceGovernorAltitudeCount,
            ReferenceGovernorNzFeasible.data(),
            mach,
            altitude_m,
            value))
    {
        return false;
    }
    // Match ReferenceGovernor.nz_feasible: multiply, then divide by
    // max(mass, 1.0).  Do not fold this into a precomputed scale.
    output = value * GovernorReferenceMassKg / std::max(mass_kg, 1.0);
    return Finite(output);
}

bool GovernorNzAtSpeed(
    const double speed_mps,
    const double altitude_m,
    const double mass_kg,
    double& output) noexcept
{
    output = 0.0;
    const double sound_speed = SoundSpeedMps(altitude_m);
    if (!Finite(speed_mps)
        || !Finite(sound_speed)
        || sound_speed <= 0.0)
    {
        return false;
    }
    const double mach = speed_mps / sound_speed;
    return Finite(mach)
        && GovernorNzAtMach(mach, altitude_m, mass_kg, output);
}

bool Contains(
    const LadyLuck::ObfmFeasibleSpeedInterval& interval,
    const double speed_mps) noexcept
{
    return interval.valid
        && interval.lower_mps <= speed_mps
        && speed_mps <= interval.upper_mps;
}

double BoundaryCrossing(
    const double x0,
    const double x1,
    const double y0,
    const double y1,
    const double target) noexcept
{
    return x0 + (target - y0) * (x1 - x0) / (y1 - y0);
}

bool AppendInterval(
    LadyLuck::ObfmStallFeasibleSetReceipt& output,
    const double lower_mach,
    const double upper_mach,
    const bool lower_resolved,
    const bool upper_resolved,
    const double sound_speed_mps) noexcept
{
    if (output.interval_count
            >= LadyLuck::ObfmMaximumFeasibleSpeedIntervals
        || !Finite(lower_mach)
        || !Finite(upper_mach)
        || !Finite(sound_speed_mps))
    {
        return false;
    }
    LadyLuck::ObfmFeasibleSpeedInterval interval{};
    interval.lower_mps = lower_mach * sound_speed_mps;
    interval.upper_mps = upper_mach * sound_speed_mps;
    interval.lower_boundary_resolved = lower_resolved;
    interval.upper_boundary_resolved = upper_resolved;
    interval.valid = Finite(interval.lower_mps)
        && Finite(interval.upper_mps)
        && interval.lower_mps > 0.0
        && interval.upper_mps > interval.lower_mps;
    if (!interval.valid)
    {
        return false;
    }
    output.intervals[output.interval_count] = interval;
    ++output.interval_count;
    return true;
}

void QueryStallFeasibleSet(
    const double altitude_m,
    const double mass_kg,
    const double required_load_g,
    LadyLuck::ObfmStallFeasibleSetReceipt& output) noexcept
{
    output = LadyLuck::ObfmStallFeasibleSetReceipt{};
    output.altitude_m = altitude_m;
    output.mass_kg = mass_kg;
    output.required_load_g = required_load_g;
    if (!StrictlyIncreasingFiniteAxis(
            ReferenceGovernorMachAxis.data(),
            ReferenceGovernorMachCount)
        || !StrictlyIncreasingFiniteAxis(
            ReferenceGovernorAltitudeAxis.data(),
            ReferenceGovernorAltitudeCount))
    {
        output.status = LadyLuck::ObfmStallFeasibleSetStatus::
            SourceNumericsInvalid;
        return;
    }
    output.mach_domain_lower = ReferenceGovernorMachAxis[0U];
    output.mach_domain_upper =
        ReferenceGovernorMachAxis[ReferenceGovernorMachCount - 1U];
    if (mass_kg <= 0.0 || required_load_g <= 0.0)
    {
        output.status = LadyLuck::ObfmStallFeasibleSetStatus::
            NonpositiveMassOrRequiredLoad;
        return;
    }
    if (altitude_m < ReferenceGovernorAltitudeAxis[0U]
        || altitude_m > ReferenceGovernorAltitudeAxis[
            ReferenceGovernorAltitudeCount - 1U])
    {
        output.status = LadyLuck::ObfmStallFeasibleSetStatus::
            AltitudeOutsideTrimTableDomain;
        return;
    }

    const double sound_speed = SoundSpeedMps(altitude_m);
    std::array<double, ReferenceGovernorMachCount> values{};
    for (std::size_t index = 0U;
         index < ReferenceGovernorMachCount;
         ++index)
    {
        // Python evaluates nz_feasible(mach_node * a, ...), whose governor
        // recomputes Mach by dividing by a.  Preserve that binary64 round trip.
        if (!GovernorNzAtSpeed(
                ReferenceGovernorMachAxis[index] * sound_speed,
                altitude_m,
                mass_kg,
                values[index]))
        {
            output.status = LadyLuck::ObfmStallFeasibleSetStatus::
                SourceNumericsInvalid;
            return;
        }
    }

    bool has_start = values[0U] >= required_load_g;
    double start = has_start ? ReferenceGovernorMachAxis[0U] : 0.0;
    bool start_resolved = false;
    for (std::size_t index = 1U;
         index < ReferenceGovernorMachCount;
         ++index)
    {
        const bool was_feasible = values[index - 1U] >= required_load_g;
        const bool is_feasible = values[index] >= required_load_g;
        if (!was_feasible && is_feasible)
        {
            start = BoundaryCrossing(
                ReferenceGovernorMachAxis[index - 1U],
                ReferenceGovernorMachAxis[index],
                values[index - 1U],
                values[index],
                required_load_g);
            has_start = true;
            start_resolved = true;
        }
        else if (was_feasible && !is_feasible)
        {
            const double end = BoundaryCrossing(
                ReferenceGovernorMachAxis[index - 1U],
                ReferenceGovernorMachAxis[index],
                values[index - 1U],
                values[index],
                required_load_g);
            if (!has_start || !AppendInterval(
                    output,
                    start,
                    end,
                    start_resolved,
                    true,
                    sound_speed))
            {
                output = LadyLuck::ObfmStallFeasibleSetReceipt{};
                output.status = LadyLuck::ObfmStallFeasibleSetStatus::
                    SourceNumericsInvalid;
                return;
            }
            has_start = false;
            start_resolved = false;
        }
    }
    if (has_start && !AppendInterval(
            output,
            start,
            ReferenceGovernorMachAxis[ReferenceGovernorMachCount - 1U],
            start_resolved,
            false,
            sound_speed))
    {
        output = LadyLuck::ObfmStallFeasibleSetReceipt{};
        output.status =
            LadyLuck::ObfmStallFeasibleSetStatus::SourceNumericsInvalid;
        return;
    }
    output.admitted = output.interval_count > 0U;
    output.status = output.admitted
        ? LadyLuck::ObfmStallFeasibleSetStatus::IntervalsAvailable
        : LadyLuck::ObfmStallFeasibleSetStatus::RequiredLoadUnreachable;
}

bool ConflictAltitude(const double altitude_m) noexcept
{
    return altitude_m == EmConflictAltitude0M
        || altitude_m == EmConflictAltitude1M
        || altitude_m == EmConflictAltitude2M;
}

void QueryEmCorner(
    const double altitude_m,
    LadyLuck::ObfmEmCornerReceipt& output) noexcept
{
    output = LadyLuck::ObfmEmCornerReceipt{};
    output.altitude_m = altitude_m;
    if (!StrictlyIncreasingFiniteAxis(
            kEmAltitudeAxisM,
            kEmAltitudeCount))
    {
        output.status = LadyLuck::ObfmEmCornerStatus::SourceNumericsInvalid;
        return;
    }
    if (altitude_m < kEmAltitudeAxisM[0U]
        || altitude_m > kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        output.status = LadyLuck::ObfmEmCornerStatus::
            AltitudeOutsideEmTableDomain;
        return;
    }
    if (!LinearInterpolate(
            kEmAltitudeAxisM,
            kEmCornerInstantaneousMps,
            kEmAltitudeCount,
            altitude_m,
            output.instantaneous_mps)
        || !LinearInterpolate(
            kEmAltitudeAxisM,
            kEmCornerSustainedMps,
            kEmAltitudeCount,
            altitude_m,
            output.sustained_mps))
    {
        output.status = LadyLuck::ObfmEmCornerStatus::SourceNumericsInvalid;
        return;
    }
    const LadyLuck::guidance::em::StrictEnergyManeuverEnvelope envelope{};
    LadyLuck::guidance::em::EmCellTrustReceipt instantaneous_trust{};
    LadyLuck::guidance::em::EmCellTrustReceipt sustained_trust{};
    envelope.ObserveCellTrust(
        output.instantaneous_mps,
        altitude_m,
        instantaneous_trust);
    envelope.ObserveCellTrust(
        output.sustained_mps,
        altitude_m,
        sustained_trust);
    if (!instantaneous_trust.lookup_valid()
        || !sustained_trust.lookup_valid())
    {
        output.status = LadyLuck::ObfmEmCornerStatus::SourceNumericsInvalid;
        return;
    }
    output.instantaneous_lookup_trusted =
        instantaneous_trust.cell.trusted;
    output.sustained_lookup_trusted = sustained_trust.cell.trusted;

    std::size_t lower = 0U;
    if (!RightBracketLowerIndex(
            kEmAltitudeAxisM,
            kEmAltitudeCount,
            altitude_m,
            lower))
    {
        output = LadyLuck::ObfmEmCornerReceipt{};
        output.altitude_m = altitude_m;
        output.status = LadyLuck::ObfmEmCornerStatus::SourceNumericsInvalid;
        return;
    }
    const std::size_t upper = std::min(
        lower + 1U,
        kEmAltitudeCount - 1U);
    output.external_conflict_bracketed =
        ConflictAltitude(kEmAltitudeAxisM[lower])
        || ConflictAltitude(kEmAltitudeAxisM[upper]);
    output.admitted = output.instantaneous_lookup_trusted
        && output.sustained_lookup_trusted
        && !output.external_conflict_bracketed;
    if (output.external_conflict_bracketed)
    {
        output.status =
            LadyLuck::ObfmEmCornerStatus::ExternalConflictBracketed;
    }
    else if (!output.instantaneous_lookup_trusted
        || !output.sustained_lookup_trusted)
    {
        output.status =
            LadyLuck::ObfmEmCornerStatus::SoundnessMaskUntrusted;
    }
    else
    {
        output.status = LadyLuck::ObfmEmCornerStatus::CornerAvailable;
    }
}

bool PublishedInstantaneousLoad(
    const double speed_mps,
    const double altitude_m,
    const double mass_kg,
    double& load_g,
    bool& trusted) noexcept
{
    load_g = 0.0;
    trusted = false;
    double value = 0.0;
    if (!Bilinear(
            kEmSpeedAxisMps,
            kEmSpeedCount,
            kEmAltitudeAxisM,
            kEmAltitudeCount,
            kEmNInstPubG,
            speed_mps,
            altitude_m,
            value))
    {
        return false;
    }
    const double mass_reference_kg =
        kEmReferenceWeightN / LadyLuck::constants::StandardGravityMps2;
    if (!PositiveFinite(mass_kg)
        || !PositiveFinite(mass_reference_kg))
    {
        return false;
    }
    const double lambda = mass_kg / mass_reference_kg;
    if (!Finite(lambda)
        || lambda <= 0.0
        || (lambda < 1.0
            && std::abs(value)
                >= std::numeric_limits<double>::max() * lambda))
    {
        return false;
    }
    const double candidate_load_g = value / lambda;
    if (!Finite(candidate_load_g))
    {
        return false;
    }
    load_g = candidate_load_g;
    const LadyLuck::guidance::em::StrictEnergyManeuverEnvelope envelope{};
    LadyLuck::guidance::em::EmCellTrustReceipt cell_trust{};
    envelope.ObserveCellTrust(speed_mps, altitude_m, cell_trust);
    trusted = cell_trust.lookup_valid() && cell_trust.cell.trusted_n;
    return true;
}

} // namespace

namespace LadyLuck
{

void ObfmLongitudinalAuthority::Observe(
    const ObfmLongitudinalAuthorityInput& input,
    ObfmLongitudinalAuthorityReceipt& output,
    Status& status) const noexcept
{
    output = ObfmLongitudinalAuthorityReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(input.current_frame_identity)
        || !IsValidControlFrameIdentity(
            input.previous_energy.source_frame_identity))
    {
        output.previous_energy.status =
            ObfmEnergyRateAuthorityStatus::FrameIdentityInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.evaluated = true;
    output.current_frame_identity = input.current_frame_identity;
    output.previous_energy.evaluated = true;
    output.previous_energy.source_frame_identity =
        input.previous_energy.source_frame_identity;
    output.speed_rate_bounds_available =
        input.speed_rate_bounds_available;
    output.speed_rate_bounds_source_valid =
        input.speed_rate_bounds_source_valid;
    output.speed_rate_lower_mps2 = input.speed_rate_lower_mps2;
    output.speed_rate_upper_mps2 = input.speed_rate_upper_mps2;
    output.flight_path_gamma_limit_rad =
        input.flight_path_gamma_limit_rad;

    if (!input.previous_energy.cis_v4_backend)
    {
        output.previous_energy.status =
            ObfmEnergyRateAuthorityStatus::BackendUnavailable;
        return;
    }
    if (!input.previous_energy.energy_receipt_available)
    {
        output.previous_energy.status =
            ObfmEnergyRateAuthorityStatus::EnergyReceiptUnavailable;
        return;
    }
    if (!input.previous_energy.continuous_total_energy_controller)
    {
        output.previous_energy.status = ObfmEnergyRateAuthorityStatus::
            ControllerNotContinuousTotalEnergy;
        return;
    }
    if (!input.previous_energy.rate_measurement_valid)
    {
        output.previous_energy.status = ObfmEnergyRateAuthorityStatus::
            RateMeasurementUnavailable;
        return;
    }
    if (!input.previous_energy.controller_configuration_available)
    {
        output.previous_energy.status = ObfmEnergyRateAuthorityStatus::
            ControllerConfigurationMissing;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    const double numeric_values[] = {
        input.previous_energy.energy_error_gain_per_s,
        input.previous_energy.energy_integral_gain_per_s2,
        input.previous_energy.energy_rate_feedback_gain,
        input.previous_energy.total_energy_error_m2ps2,
        input.previous_energy.energy_integral_error_m2ps,
        input.previous_energy.specific_energy_rate_measured_m2ps3,
        input.previous_energy.speed_mps,
        input.previous_energy.minimum_speed_mps,
        input.previous_energy.mass_kg,
        input.previous_energy.drag_estimate_n,
        input.previous_energy.thrust_velocity_projection,
        input.previous_energy.thrust_min_n,
        input.previous_energy.thrust_max_n,
        input.speed_rate_lower_mps2,
        input.speed_rate_upper_mps2,
        input.flight_path_gamma_limit_rad};
    for (const double value : numeric_values)
    {
        if (!Finite(value))
        {
            output = ObfmLongitudinalAuthorityReceipt{};
            output.previous_energy.status =
                ObfmEnergyRateAuthorityStatus::InputNonfinite;
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }

    ObfmEnergyRateAuthorityObservation& energy = output.previous_energy;
    energy.energy_error_gain_per_s =
        input.previous_energy.energy_error_gain_per_s;
    energy.energy_integral_gain_per_s2 =
        input.previous_energy.energy_integral_gain_per_s2;
    energy.energy_rate_feedback_gain =
        input.previous_energy.energy_rate_feedback_gain;
    energy.total_energy_error_m2ps2 =
        input.previous_energy.total_energy_error_m2ps2;
    energy.energy_integral_error_m2ps =
        input.previous_energy.energy_integral_error_m2ps;
    energy.specific_energy_rate_measured_m2ps3 =
        input.previous_energy.specific_energy_rate_measured_m2ps3;
    energy.mass_kg = input.previous_energy.mass_kg;
    energy.drag_estimate_n = input.previous_energy.drag_estimate_n;
    energy.thrust_velocity_projection =
        input.previous_energy.thrust_velocity_projection;
    energy.thrust_min_n = input.previous_energy.thrust_min_n;
    energy.thrust_max_n = input.previous_energy.thrust_max_n;

    if (energy.energy_error_gain_per_s < 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::EnergyErrorGainNegative;
        return;
    }
    if (energy.energy_integral_gain_per_s2 < 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::EnergyIntegralGainNegative;
        return;
    }
    if (energy.energy_rate_feedback_gain < 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::EnergyRateFeedbackGainNegative;
        return;
    }
    if (input.previous_energy.minimum_speed_mps <= 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::MinimumSpeedNotPositive;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    energy.speed_for_inverse_mps = std::max(
        input.previous_energy.speed_mps,
        input.previous_energy.minimum_speed_mps);
    if (energy.speed_for_inverse_mps <= 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::InverseSpeedNotPositive;
        return;
    }
    if (energy.mass_kg <= 0.0)
    {
        energy.status = ObfmEnergyRateAuthorityStatus::MassNotPositive;
        return;
    }
    if (energy.drag_estimate_n < 0.0)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::DragEstimateNegative;
        return;
    }
    if (energy.thrust_velocity_projection <= 0.0
        || energy.thrust_velocity_projection > 1.0)
    {
        energy.status = ObfmEnergyRateAuthorityStatus::
            ThrustProjectionOutOfRange;
        return;
    }
    if (energy.thrust_min_n > energy.thrust_max_n)
    {
        energy.status =
            ObfmEnergyRateAuthorityStatus::ThrustBoundsReversed;
        return;
    }

    energy.reference_denominator =
        1.0 + energy.energy_rate_feedback_gain;
    energy.controller_offset_m2ps3 =
        energy.energy_error_gain_per_s * energy.total_energy_error_m2ps2
        + energy.energy_integral_gain_per_s2
            * energy.energy_integral_error_m2ps
        - energy.energy_rate_feedback_gain
            * energy.specific_energy_rate_measured_m2ps3;
    energy.rate_command_min_m2ps3 = energy.speed_for_inverse_mps
        * (energy.thrust_velocity_projection * energy.thrust_min_n
            - energy.drag_estimate_n)
        / energy.mass_kg;
    energy.rate_command_max_m2ps3 = energy.speed_for_inverse_mps
        * (energy.thrust_velocity_projection * energy.thrust_max_n
            - energy.drag_estimate_n)
        / energy.mass_kg;
    energy.reference_min_m2ps3 =
        (energy.rate_command_min_m2ps3
            - energy.controller_offset_m2ps3)
        / energy.reference_denominator;
    energy.reference_max_m2ps3 =
        (energy.rate_command_max_m2ps3
            - energy.controller_offset_m2ps3)
        / energy.reference_denominator;
    const double derived[] = {
        energy.reference_denominator,
        energy.controller_offset_m2ps3,
        energy.rate_command_min_m2ps3,
        energy.rate_command_max_m2ps3,
        energy.reference_min_m2ps3,
        energy.reference_max_m2ps3};
    for (const double value : derived)
    {
        if (!Finite(value))
        {
            energy.reference_denominator = 0.0;
            energy.controller_offset_m2ps3 = 0.0;
            energy.rate_command_min_m2ps3 = 0.0;
            energy.rate_command_max_m2ps3 = 0.0;
            energy.reference_min_m2ps3 = 0.0;
            energy.reference_max_m2ps3 = 0.0;
            energy.status =
                ObfmEnergyRateAuthorityStatus::DerivedAuthorityNonfinite;
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    if (energy.rate_command_min_m2ps3
            > energy.rate_command_max_m2ps3
        || energy.reference_min_m2ps3 > energy.reference_max_m2ps3)
    {
        energy.reference_denominator = 0.0;
        energy.controller_offset_m2ps3 = 0.0;
        energy.rate_command_min_m2ps3 = 0.0;
        energy.rate_command_max_m2ps3 = 0.0;
        energy.reference_min_m2ps3 = 0.0;
        energy.reference_max_m2ps3 = 0.0;
        energy.status =
            ObfmEnergyRateAuthorityStatus::DerivedAuthorityReversed;
        return;
    }
    energy.valid = true;
    energy.status = ObfmEnergyRateAuthorityStatus::AuthorityAvailable;
}

namespace
{

void BuildC1SpeedSource(
    const ObfmLongitudinalAdmissionInput& input,
    ObfmC1SpeedSourceReceipt& output) noexcept
{
    output = ObfmC1SpeedSourceReceipt{};
    QueryEmCorner(input.altitude_m, output.em_corner);
    if (!output.em_corner.admitted)
    {
        output.status = ObfmC1SpeedSourceStatus::EmCornerNotAdmitted;
        return;
    }

    bool load_trusted = false;
    if (!PublishedInstantaneousLoad(
            output.em_corner.instantaneous_mps,
            input.altitude_m,
            input.mass_kg,
            output.required_load_g,
            load_trusted))
    {
        output.status = ObfmC1SpeedSourceStatus::SourceNumericsInvalid;
        return;
    }
    if (!load_trusted)
    {
        output.status =
            ObfmC1SpeedSourceStatus::PublishedCornerLoadUntrusted;
        return;
    }
    if (output.required_load_g <= 0.0)
    {
        output.required_load_g = 0.0;
        output.status =
            ObfmC1SpeedSourceStatus::PublishedCornerLoadNotPositive;
        return;
    }
    QueryStallFeasibleSet(
        input.altitude_m,
        input.mass_kg,
        output.required_load_g,
        output.stall_set);

    // wez_c1_vertical_speed_reference begins here.  Keep its contract checks
    // after the corner-load and stall queries, matching the Python caller.
    if (input.current_range_m <= 0.0
        || input.target_speed_mps <= 0.0
        || input.acquisition_speed_mps <= 0.0
        || input.official_min_range_m <= 0.0
        || input.official_max_range_m <= input.official_min_range_m)
    {
        output.status = ObfmC1SpeedSourceStatus::
            SpeedOrOfficialRangeContractInvalid;
        return;
    }
    if (!output.stall_set.admitted)
    {
        output.status = ObfmC1SpeedSourceStatus::StallSetNotAdmitted;
        return;
    }

    bool interval_found = false;
    for (std::uint8_t index = 0U;
         index < output.stall_set.interval_count;
         ++index)
    {
        if (Contains(
                output.stall_set.intervals[index],
                output.em_corner.instantaneous_mps))
        {
            output.feasible_interval = output.stall_set.intervals[index];
            interval_found = true;
            break;
        }
    }
    if (!interval_found)
    {
        output.status = ObfmC1SpeedSourceStatus::
            EmCornerOutsideRequiredLoadSet;
        return;
    }

    output.acquisition_speed_admitted_mps = Clamp(
        input.acquisition_speed_mps,
        output.feasible_interval.lower_mps,
        output.feasible_interval.upper_mps);
    output.range_hold_gain_per_s =
        input.target_speed_mps / input.official_min_range_m;
    output.range_hold_speed_raw_mps = input.target_speed_mps
        + output.range_hold_gain_per_s
            * (input.current_range_m - input.official_min_range_m);
    const double hold_upper = std::min(
        output.feasible_interval.upper_mps,
        output.em_corner.instantaneous_mps);
    if (hold_upper < output.feasible_interval.lower_mps)
    {
        output.status = ObfmC1SpeedSourceStatus::
            EmCornerBelowRequiredLoadFloor;
        return;
    }
    output.range_hold_speed_admitted_mps = Clamp(
        output.range_hold_speed_raw_mps,
        output.feasible_interval.lower_mps,
        hold_upper);
    const double normalized =
        (input.official_max_range_m - input.current_range_m)
        / (input.official_max_range_m - input.official_min_range_m);
    const double x = Clamp(normalized, 0.0, 1.0);
    output.blend_weight = x * x * (3.0 - 2.0 * x);
    output.speed_command_mps =
        (1.0 - output.blend_weight)
            * output.acquisition_speed_admitted_mps
        + output.blend_weight * output.range_hold_speed_admitted_mps;

    const double x_rate = x > 0.0 && x < 1.0
        ? -input.range_rate_mps
            / (input.official_max_range_m - input.official_min_range_m)
        : 0.0;
    const double weight_rate = 6.0 * x * (1.0 - x) * x_rate;
    const double hold_rate =
        output.range_hold_speed_raw_mps
                > output.feasible_interval.lower_mps
            && output.range_hold_speed_raw_mps < hold_upper
        ? output.range_hold_gain_per_s * input.range_rate_mps
        : 0.0;
    output.range_only_speed_rate_diagnostic_mps2 =
        weight_rate
            * (output.range_hold_speed_admitted_mps
                - output.acquisition_speed_admitted_mps)
        + output.blend_weight * hold_rate;

    const double derived[] = {
        output.required_load_g,
        output.acquisition_speed_admitted_mps,
        output.range_hold_gain_per_s,
        output.range_hold_speed_raw_mps,
        output.range_hold_speed_admitted_mps,
        output.blend_weight,
        output.speed_command_mps,
        output.range_only_speed_rate_diagnostic_mps2};
    for (const double value : derived)
    {
        if (!Finite(value))
        {
            output = ObfmC1SpeedSourceReceipt{};
            output.status = ObfmC1SpeedSourceStatus::SourceNumericsInvalid;
            return;
        }
    }
    output.admitted = true;
    output.status = ObfmC1SpeedSourceStatus::SourceAvailable;
}

void BuildTransientLoadBridge(
    const ObfmLongitudinalAdmissionInput& input,
    const double terminal_speed_mps,
    const double terminal_required_load_g,
    ObfmTransientLoadBridgeReceipt& output) noexcept
{
    output = ObfmTransientLoadBridgeReceipt{};
    output.previous_governed_load_g = input.previous_governed_load_g;
    output.previous_measured_load_g = input.previous_measured_load_g;
    output.terminal_required_load_g = terminal_required_load_g > 0.0
        ? terminal_required_load_g
        : 0.0;
    if (input.mass_kg <= 0.0
        || input.current_speed_mps <= 0.0
        || input.previous_speed_command_mps <= 0.0
        || terminal_speed_mps <= 0.0)
    {
        output.status =
            ObfmTransientLoadBridgeStatus::MassOrSpeedNotPositive;
        return;
    }
    if (input.previous_governed_load_g <= 0.0
        || input.previous_measured_load_g <= 0.0)
    {
        output.status = ObfmTransientLoadBridgeStatus::
            PreviousPositiveLoadRegimeNotEstablished;
        return;
    }
    if (terminal_required_load_g <= 0.0)
    {
        output.status = ObfmTransientLoadBridgeStatus::
            TerminalRequiredLoadNotPositive;
        return;
    }
    if (!GovernorNzAtSpeed(
            input.current_speed_mps,
            input.altitude_m,
            input.mass_kg,
            output.current_capability_load_g))
    {
        output.status =
            ObfmTransientLoadBridgeStatus::SourceNumericsInvalid;
        return;
    }
    if (output.current_capability_load_g <= 0.0)
    {
        output.status = ObfmTransientLoadBridgeStatus::
            CurrentCapabilityNotPositive;
        return;
    }
    const double previous_obligation = std::max(
        1.0,
        std::max(
            input.previous_governed_load_g,
            input.previous_measured_load_g));
    output.bridge_required_load_g = std::nextafter(
        previous_obligation,
        -std::numeric_limits<double>::infinity());
    if (!Finite(output.bridge_required_load_g)
        || output.bridge_required_load_g <= 0.0)
    {
        output.bridge_required_load_g = 0.0;
        output.status = ObfmTransientLoadBridgeStatus::
            BridgeRequiredLoadNotPositive;
        return;
    }
    QueryStallFeasibleSet(
        input.altitude_m,
        input.mass_kg,
        output.bridge_required_load_g,
        output.stall_set);
    if (!output.stall_set.admitted)
    {
        output.status = ObfmTransientLoadBridgeStatus::
            BridgeStallSetNotAdmitted;
        return;
    }
    std::uint8_t matching_count = 0U;
    for (std::uint8_t index = 0U;
         index < output.stall_set.interval_count;
         ++index)
    {
        const ObfmFeasibleSpeedInterval& interval =
            output.stall_set.intervals[index];
        if (Contains(interval, input.current_speed_mps)
            && Contains(interval, input.previous_speed_command_mps))
        {
            output.feasible_interval = interval;
            ++matching_count;
        }
    }
    if (matching_count == 0U)
    {
        output.feasible_interval = ObfmFeasibleSpeedInterval{};
        output.status = ObfmTransientLoadBridgeStatus::
            CurrentAndPreviousNotInOneConnectedInterval;
        return;
    }
    if (matching_count != 1U)
    {
        output.feasible_interval = ObfmFeasibleSpeedInterval{};
        output.status = ObfmTransientLoadBridgeStatus::
            ConnectedIntervalNotUnique;
        return;
    }
    output.admitted = true;
    output.status = ObfmTransientLoadBridgeStatus::BridgeAvailable;
}

bool CheckedAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double maximum = std::numeric_limits<double>::max();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!Finite(output))
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
    return CheckedAdd(left, -right, output);
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!Finite(left) || !Finite(right))
    {
        return false;
    }
    const double absolute_left = std::abs(left);
    const double absolute_right = std::abs(right);
    if ((absolute_left > 1.0
            && absolute_right
                >= std::numeric_limits<double>::max() / absolute_left)
        || (absolute_right > 1.0
            && absolute_left
                >= std::numeric_limits<double>::max() / absolute_right))
    {
        return false;
    }
    output = left * right;
    if (!Finite(output))
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
    if (!Finite(numerator)
        || !Finite(denominator)
        || denominator == 0.0)
    {
        return false;
    }
    const double absolute_numerator = std::abs(numerator);
    const double absolute_denominator = std::abs(denominator);
    if (absolute_numerator != 0.0
        && absolute_denominator < 1.0
        && absolute_numerator
            >= std::numeric_limits<double>::max()
                * absolute_denominator)
    {
        return false;
    }
    output = numerator / denominator;
    if (!Finite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool RateCommandForSpeed(
    const double speed_command_mps,
    const double previous_speed_command_mps,
    const double dt_s,
    const double own_speed_mps,
    const double altitude_rate_cmd_mps,
    const ObfmEnergyRateAuthorityObservation& authority,
    double& output) noexcept
{
    output = 0.0;
    double speed_delta = 0.0;
    double speed_rate = 0.0;
    double reference_speed_term = 0.0;
    double reference_altitude_term = 0.0;
    double reference = 0.0;
    double speed_squared = 0.0;
    double own_speed_squared = 0.0;
    double energy_square_difference = 0.0;
    double energy_error = 0.0;
    double reference_gain = 0.0;
    double proportional_term = 0.0;
    double integral_term = 0.0;
    double feedback_term = 0.0;
    double accumulated = 0.0;
    if (!CheckedSubtract(
            speed_command_mps,
            previous_speed_command_mps,
            speed_delta)
        || !CheckedDivide(speed_delta, dt_s, speed_rate)
        || !CheckedMultiply(
            speed_command_mps,
            speed_rate,
            reference_speed_term)
        || !CheckedMultiply(
            constants::StandardGravityMps2,
            altitude_rate_cmd_mps,
            reference_altitude_term)
        || !CheckedAdd(
            reference_speed_term,
            reference_altitude_term,
            reference)
        || !CheckedMultiply(
            speed_command_mps,
            speed_command_mps,
            speed_squared)
        || !CheckedMultiply(
            own_speed_mps,
            own_speed_mps,
            own_speed_squared)
        || !CheckedSubtract(
            speed_squared,
            own_speed_squared,
            energy_square_difference)
        || !CheckedMultiply(
            0.5,
            energy_square_difference,
            energy_error)
        || !CheckedAdd(
            1.0,
            authority.energy_rate_feedback_gain,
            reference_gain)
        || !CheckedMultiply(reference_gain, reference, accumulated)
        || !CheckedMultiply(
            authority.energy_error_gain_per_s,
            energy_error,
            proportional_term)
        || !CheckedAdd(accumulated, proportional_term, accumulated)
        || !CheckedMultiply(
            authority.energy_integral_gain_per_s2,
            authority.energy_integral_error_m2ps,
            integral_term)
        || !CheckedAdd(accumulated, integral_term, accumulated)
        || !CheckedMultiply(
            authority.energy_rate_feedback_gain,
            authority.specific_energy_rate_measured_m2ps3,
            feedback_term)
        || !CheckedSubtract(accumulated, feedback_term, output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool BuildRateCommandQuadratic(
    const ObfmLongitudinalAdmissionInput& input,
    double& coefficient_a,
    double& coefficient_b,
    double& coefficient_c) noexcept
{
    coefficient_a = 0.0;
    coefficient_b = 0.0;
    coefficient_c = 0.0;
    const ObfmEnergyRateAuthorityObservation& authority =
        input.energy_authority;
    double reference_gain = 0.0;
    double reference_rate_gain = 0.0;
    double half_proportional_gain = 0.0;
    double previous_speed_term = 0.0;
    double altitude_reference_term = 0.0;
    double altitude_command_term = 0.0;
    double own_speed_term = 0.0;
    double own_energy_term = 0.0;
    double integral_term = 0.0;
    double feedback_term = 0.0;
    double accumulated = 0.0;
    if (!CheckedAdd(
            1.0,
            authority.energy_rate_feedback_gain,
            reference_gain)
        || !CheckedDivide(
            reference_gain,
            input.dt_s,
            reference_rate_gain)
        || !CheckedMultiply(
            0.5,
            authority.energy_error_gain_per_s,
            half_proportional_gain)
        || !CheckedAdd(
            reference_rate_gain,
            half_proportional_gain,
            coefficient_a)
        || !CheckedMultiply(
            reference_gain,
            input.previous_speed_command_mps,
            previous_speed_term)
        || !CheckedDivide(
            previous_speed_term,
            input.dt_s,
            previous_speed_term)
        || !CheckedMultiply(-1.0, previous_speed_term, coefficient_b)
        || !CheckedMultiply(
            reference_gain,
            constants::StandardGravityMps2,
            altitude_reference_term)
        || !CheckedMultiply(
            altitude_reference_term,
            input.altitude_rate_cmd_mps,
            altitude_command_term)
        || !CheckedMultiply(
            half_proportional_gain,
            input.current_speed_mps,
            own_speed_term)
        || !CheckedMultiply(
            own_speed_term,
            input.current_speed_mps,
            own_energy_term)
        || !CheckedMultiply(
            authority.energy_integral_gain_per_s2,
            authority.energy_integral_error_m2ps,
            integral_term)
        || !CheckedMultiply(
            authority.energy_rate_feedback_gain,
            authority.specific_energy_rate_measured_m2ps3,
            feedback_term)
        || !CheckedSubtract(
            altitude_command_term,
            own_energy_term,
            accumulated)
        || !CheckedAdd(accumulated, integral_term, accumulated)
        || !CheckedSubtract(accumulated, feedback_term, coefficient_c))
    {
        coefficient_a = 0.0;
        coefficient_b = 0.0;
        coefficient_c = 0.0;
        return false;
    }
    return true;
}

bool NonnegativeFiniteBits(
    const double value,
    std::uint64_t& bits) noexcept
{
    if (!Finite(value) || value < 0.0)
    {
        return false;
    }
    const double normalized = value == 0.0 ? 0.0 : value;
    std::memcpy(&bits, &normalized, sizeof(bits));
    return (bits >> 63U) == 0U;
}

double NonnegativeDoubleFromBits(const std::uint64_t bits) noexcept
{
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool InwardPositiveRoot(
    const double rate_command_bound,
    const bool lower_boundary,
    const double coefficient_a,
    const double coefficient_b,
    const double coefficient_c,
    const ObfmLongitudinalAdmissionInput& input,
    double& output,
    std::size_t* const evaluation_count = nullptr) noexcept
{
    output = 0.0;
    if (evaluation_count != nullptr)
    {
        *evaluation_count = 0U;
    }
    double coefficient_c_offset = 0.0;
    double coefficient_b_squared = 0.0;
    double four_coefficient_a = 0.0;
    double discriminant_product = 0.0;
    double discriminant = 0.0;
    if (!Finite(rate_command_bound)
        || !Finite(coefficient_a)
        || !Finite(coefficient_b)
        || !Finite(coefficient_c)
        || coefficient_a <= 0.0
        || coefficient_b >= 0.0
        || !CheckedSubtract(
            coefficient_c,
            rate_command_bound,
            coefficient_c_offset)
        || !CheckedMultiply(
            coefficient_b,
            coefficient_b,
            coefficient_b_squared)
        || !CheckedMultiply(4.0, coefficient_a, four_coefficient_a)
        || !CheckedMultiply(
            four_coefficient_a,
            coefficient_c_offset,
            discriminant_product)
        || !CheckedSubtract(
            coefficient_b_squared,
            discriminant_product,
            discriminant)
        || discriminant < 0.0)
    {
        return false;
    }
    const double square_root = std::sqrt(discriminant);
    double numerator = 0.0;
    double denominator = 0.0;
    double root = 0.0;
    if (!Finite(square_root)
        || !CheckedAdd(-coefficient_b, square_root, numerator)
        || !CheckedMultiply(2.0, coefficient_a, denominator)
        || !CheckedDivide(numerator, denominator, root)
        || root <= 0.0)
    {
        return false;
    }
    const double direction = lower_boundary
        ? std::numeric_limits<double>::infinity()
        : -std::numeric_limits<double>::infinity();
    const double root_neighbor = std::nextafter(root, direction);
    std::uint64_t neighbor_bits = 0U;
    if (!NonnegativeFiniteBits(root_neighbor, neighbor_bits)
        || root_neighbor <= 0.0)
    {
        return false;
    }

    double neighbor_command = 0.0;
    if (evaluation_count != nullptr)
    {
        ++(*evaluation_count);
    }
    if (!RateCommandForSpeed(
            root_neighbor,
            input.previous_speed_command_mps,
            input.dt_s,
            input.current_speed_mps,
            input.altitude_rate_cmd_mps,
            input.energy_authority,
            neighbor_command))
    {
        return false;
    }
    const bool neighbor_inside = lower_boundary
        ? neighbor_command >= rate_command_bound
        : neighbor_command <= rate_command_bound;
    if (neighbor_inside)
    {
        output = root_neighbor;
        return true;
    }

    std::uint64_t lower_bits = 0U;
    std::uint64_t upper_bits = 0U;
    if (lower_boundary)
    {
        lower_bits = neighbor_bits;
        const double maximum = std::numeric_limits<double>::max();
        if (!NonnegativeFiniteBits(maximum, upper_bits)
            || lower_bits >= upper_bits)
        {
            return false;
        }
    }
    else
    {
        double half_negative_b = 0.0;
        double vertex = 0.0;
        if (!CheckedMultiply(-0.5, coefficient_b, half_negative_b)
            || !CheckedDivide(
                half_negative_b,
                coefficient_a,
                vertex)
            || vertex < 0.0)
        {
            return false;
        }
        double vertex_command = 0.0;
        if (evaluation_count != nullptr)
        {
            ++(*evaluation_count);
        }
        if (!RateCommandForSpeed(
                vertex,
                input.previous_speed_command_mps,
                input.dt_s,
                input.current_speed_mps,
                input.altitude_rate_cmd_mps,
                input.energy_authority,
                vertex_command)
            || vertex_command > rate_command_bound
            || !NonnegativeFiniteBits(vertex, lower_bits)
            || lower_bits >= neighbor_bits)
        {
            return false;
        }
        upper_bits = neighbor_bits;
    }

    // Non-negative finite binary64 bit patterns have numeric order.  The
    // complete representable bracket therefore closes in at most 64 steps;
    // no tolerance, ULP walk, or tactical margin is involved.  Convexity makes
    // the exact controller law nondecreasing on the positive-root branch.  The
    // ordered binary64 reconstruction is the search predicate and the returned
    // endpoint is checked again.  Monotonic rounding on this branch is needed
    // only for completeness; if it or finite reconstruction cannot be
    // established, the caller receives normal typed unavailability.  A
    // non-finite value is never returned as command authority.
    for (std::size_t iteration = 0U;
        iteration < std::numeric_limits<std::uint64_t>::digits
            && upper_bits - lower_bits > 1U;
        ++iteration)
    {
        const std::uint64_t middle_bits = lower_bits
            + (upper_bits - lower_bits) / 2U;
        const double candidate = NonnegativeDoubleFromBits(middle_bits);
        double command_value = 0.0;
        if (evaluation_count != nullptr)
        {
            ++(*evaluation_count);
        }
        const bool command_available = RateCommandForSpeed(
            candidate,
            input.previous_speed_command_mps,
            input.dt_s,
            input.current_speed_mps,
            input.altitude_rate_cmd_mps,
            input.energy_authority,
            command_value);
        if (lower_boundary)
        {
            if (command_available
                && command_value < rate_command_bound)
            {
                lower_bits = middle_bits;
            }
            else
            {
                upper_bits = middle_bits;
            }
        }
        else
        {
            if (!command_available)
            {
                return false;
            }
            if (command_value <= rate_command_bound)
            {
                lower_bits = middle_bits;
            }
            else
            {
                upper_bits = middle_bits;
            }
        }
    }
    if (upper_bits - lower_bits > 1U)
    {
        return false;
    }

    const std::uint64_t admitted_bits = lower_boundary
        ? upper_bits
        : lower_bits;
    const double admitted = NonnegativeDoubleFromBits(admitted_bits);
    double admitted_command = 0.0;
    if (evaluation_count != nullptr)
    {
        ++(*evaluation_count);
    }
    if (!RateCommandForSpeed(
            admitted,
            input.previous_speed_command_mps,
            input.dt_s,
            input.current_speed_mps,
            input.altitude_rate_cmd_mps,
            input.energy_authority,
            admitted_command)
        || (lower_boundary
            ? admitted_command < rate_command_bound
            : admitted_command > rate_command_bound))
    {
        return false;
    }
    output = admitted;
    return true;
}

std::uint8_t ActiveLowerSources(
    const double lower,
    const double nzfeas,
    const double backend,
    const double tecs) noexcept
{
    std::uint8_t output = ObfmCausalBoundSourceNone;
    if (nzfeas == lower)
    {
        output = static_cast<std::uint8_t>(
            output | ObfmCausalBoundSourceNzfeas);
    }
    if (backend == lower)
    {
        output = static_cast<std::uint8_t>(
            output | ObfmCausalBoundSourceBackendVdot);
    }
    if (tecs == lower)
    {
        output = static_cast<std::uint8_t>(
            output | ObfmCausalBoundSourceTecsThrust);
    }
    return output;
}

void AdmitCausalSpeed(
    const ObfmLongitudinalAdmissionInput& input,
    const double raw_speed_mps,
    const ObfmFeasibleSpeedInterval& feasible_interval,
    ObfmCausalSpeedAdmissionReceipt& output) noexcept
{
    output = ObfmCausalSpeedAdmissionReceipt{};
    output.raw_speed_mps = raw_speed_mps;
    output.previous_speed_command_mps = input.previous_speed_command_mps;
    if (raw_speed_mps <= 0.0
        || input.previous_speed_command_mps <= 0.0
        || input.current_speed_mps <= 0.0
        || input.dt_s <= 0.0)
    {
        output.status =
            ObfmCausalSpeedAdmissionStatus::SpeedOrDtNotPositive;
        return;
    }
    if (!feasible_interval.valid || !input.energy_authority.valid)
    {
        output.status = ObfmCausalSpeedAdmissionStatus::AuthorityInvalid;
        return;
    }
    if (input.speed_rate_lower_mps2 > input.speed_rate_upper_mps2)
    {
        output.status =
            ObfmCausalSpeedAdmissionStatus::SpeedRateBoundsReversed;
        return;
    }
    const double command_min =
        input.energy_authority.rate_command_min_m2ps3;
    const double command_max =
        input.energy_authority.rate_command_max_m2ps3;
    double coefficient_a = 0.0;
    double coefficient_b = 0.0;
    double coefficient_c = 0.0;
    if (!BuildRateCommandQuadratic(
            input,
            coefficient_a,
            coefficient_b,
            coefficient_c))
    {
        output.status =
            ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
        return;
    }
    if (coefficient_a <= 0.0)
    {
        output.status =
            ObfmCausalSpeedAdmissionStatus::EnergyQuadraticNotConvex;
        return;
    }

    if (!InwardPositiveRoot(
            command_min,
            true,
            coefficient_a,
            coefficient_b,
            coefficient_c,
            input,
            output.energy_authority_lower_mps)
        || !InwardPositiveRoot(
            command_max,
            false,
            coefficient_a,
            coefficient_b,
            coefficient_c,
            input,
            output.energy_authority_upper_mps))
    {
        output.energy_authority_lower_mps = 0.0;
        output.energy_authority_upper_mps = 0.0;
        output.status = ObfmCausalSpeedAdmissionStatus::
            EnergyAuthorityRootUnavailable;
        return;
    }
    output.feasible_speed_lower_mps = feasible_interval.lower_mps;
    output.feasible_speed_upper_mps = feasible_interval.upper_mps;
    double backend_lower_delta = 0.0;
    double backend_upper_delta = 0.0;
    if (!CheckedMultiply(
            input.dt_s,
            input.speed_rate_lower_mps2,
            backend_lower_delta)
        || !CheckedAdd(
            input.previous_speed_command_mps,
            backend_lower_delta,
            output.backend_reachable_lower_mps)
        || !CheckedMultiply(
            input.dt_s,
            input.speed_rate_upper_mps2,
            backend_upper_delta)
        || !CheckedAdd(
            input.previous_speed_command_mps,
            backend_upper_delta,
            output.backend_reachable_upper_mps))
    {
        output = ObfmCausalSpeedAdmissionReceipt{};
        output.status =
            ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
        return;
    }
    output.intersection_lower_mps = std::max(
        output.feasible_speed_lower_mps,
        std::max(
            output.backend_reachable_lower_mps,
            output.energy_authority_lower_mps));
    output.intersection_upper_mps = std::min(
        output.feasible_speed_upper_mps,
        std::min(
            output.backend_reachable_upper_mps,
            output.energy_authority_upper_mps));
    double twice_coefficient_a = 0.0;
    double derivative_at_lower = 0.0;
    if (!Finite(output.intersection_lower_mps)
        || !Finite(output.intersection_upper_mps)
        || !CheckedMultiply(
            2.0,
            coefficient_a,
            twice_coefficient_a)
        || !CheckedMultiply(
            twice_coefficient_a,
            output.intersection_lower_mps,
            derivative_at_lower)
        || !CheckedAdd(
            derivative_at_lower,
            coefficient_b,
            derivative_at_lower))
    {
        output = ObfmCausalSpeedAdmissionReceipt{};
        output.status =
            ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
        return;
    }
    if (derivative_at_lower <= 0.0)
    {
        output.status = ObfmCausalSpeedAdmissionStatus::
            EnergyCommandNotMonotone;
        return;
    }
    if (output.intersection_lower_mps > output.intersection_upper_mps)
    {
        output.status = ObfmCausalSpeedAdmissionStatus::
            CausalSpeedIntersectionEmpty;
        return;
    }
    output.admitted_speed_mps = Clamp(
        raw_speed_mps,
        output.intersection_lower_mps,
        output.intersection_upper_mps);
    double raw_speed_delta = 0.0;
    double admitted_speed_delta = 0.0;
    double speed_reference_term = 0.0;
    double altitude_reference_term = 0.0;
    double admitted_speed_squared = 0.0;
    double current_speed_squared = 0.0;
    double energy_square_difference = 0.0;
    if (!CheckedSubtract(
            raw_speed_mps,
            input.previous_speed_command_mps,
            raw_speed_delta)
        || !CheckedDivide(
            raw_speed_delta,
            input.dt_s,
            output.raw_speed_rate_mps2)
        || !CheckedSubtract(
            output.admitted_speed_mps,
            input.previous_speed_command_mps,
            admitted_speed_delta)
        || !CheckedDivide(
            admitted_speed_delta,
            input.dt_s,
            output.admitted_speed_rate_mps2)
        || !CheckedMultiply(
            output.admitted_speed_mps,
            output.admitted_speed_rate_mps2,
            speed_reference_term)
        || !CheckedMultiply(
            constants::StandardGravityMps2,
            input.altitude_rate_cmd_mps,
            altitude_reference_term)
        || !CheckedAdd(
            speed_reference_term,
            altitude_reference_term,
            output.specific_energy_rate_reference_m2ps3)
        || !CheckedMultiply(
            output.admitted_speed_mps,
            output.admitted_speed_mps,
            admitted_speed_squared)
        || !CheckedMultiply(
            input.current_speed_mps,
            input.current_speed_mps,
            current_speed_squared)
        || !CheckedSubtract(
            admitted_speed_squared,
            current_speed_squared,
            energy_square_difference)
        || !CheckedMultiply(
            0.5,
            energy_square_difference,
            output.specific_total_energy_error_m2ps2)
        || !RateCommandForSpeed(
            output.admitted_speed_mps,
            input.previous_speed_command_mps,
            input.dt_s,
            input.current_speed_mps,
            input.altitude_rate_cmd_mps,
            input.energy_authority,
            output.specific_energy_rate_command_m2ps3))
    {
        output = ObfmCausalSpeedAdmissionReceipt{};
        output.status =
            ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
        return;
    }
    const double derived[] = {
        output.admitted_speed_mps,
        output.raw_speed_rate_mps2,
        output.admitted_speed_rate_mps2,
        output.specific_energy_rate_reference_m2ps3,
        output.specific_total_energy_error_m2ps2,
        output.specific_energy_rate_command_m2ps3};
    for (const double value : derived)
    {
        if (!Finite(value))
        {
            output = ObfmCausalSpeedAdmissionReceipt{};
            output.status =
                ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
            return;
        }
    }
    if (!(command_min <= output.specific_energy_rate_command_m2ps3
            && output.specific_energy_rate_command_m2ps3 <= command_max))
    {
        output.status = ObfmCausalSpeedAdmissionStatus::
            EnergyAuthorityNumericalIdentityFailed;
        return;
    }
    output.lower_active_sources = ActiveLowerSources(
        output.intersection_lower_mps,
        output.feasible_speed_lower_mps,
        output.backend_reachable_lower_mps,
        output.energy_authority_lower_mps);
    output.upper_active_sources = ActiveLowerSources(
        output.intersection_upper_mps,
        output.feasible_speed_upper_mps,
        output.backend_reachable_upper_mps,
        output.energy_authority_upper_mps);
    output.lower_limited = raw_speed_mps < output.intersection_lower_mps;
    output.upper_limited = raw_speed_mps > output.intersection_upper_mps;
    output.admitted = true;
    output.status = output.lower_limited || output.upper_limited
        ? ObfmCausalSpeedAdmissionStatus::ProjectedToCausalIntersection
        : ObfmCausalSpeedAdmissionStatus::WithinCausalIntersection;
}

} // namespace

void ObfmLongitudinalAuthority::Admit(
    const ObfmLongitudinalAdmissionInput& input,
    ObfmLongitudinalAdmissionReceipt& output,
    Status& status) const noexcept
{
    output = ObfmLongitudinalAdmissionReceipt{};
    status = Status{};
    const double numeric_values[] = {
        input.altitude_m,
        input.mass_kg,
        input.current_range_m,
        input.range_rate_mps,
        input.target_speed_mps,
        input.acquisition_speed_mps,
        input.official_min_range_m,
        input.official_max_range_m,
        input.current_speed_mps,
        input.previous_speed_command_mps,
        input.previous_governed_load_g,
        input.previous_measured_load_g,
        input.altitude_rate_cmd_mps,
        input.dt_s,
        input.speed_rate_lower_mps2,
        input.speed_rate_upper_mps2};
    for (const double value : numeric_values)
    {
        if (!Finite(value))
        {
            output.status = ObfmLongitudinalAdmissionStatus::InputNonfinite;
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }

    BuildC1SpeedSource(input, output.c1_source);
    if (!output.c1_source.admitted)
    {
        output.status =
            ObfmLongitudinalAdmissionStatus::C1SourceNotAdmitted;
        return;
    }
    const double terminal_speed = output.c1_source.speed_command_mps;
    if (!Contains(output.c1_source.feasible_interval, terminal_speed))
    {
        output.status = ObfmLongitudinalAdmissionStatus::
            TerminalSpeedOutsideQualifiedInterval;
        return;
    }
    BuildTransientLoadBridge(
        input,
        terminal_speed,
        output.c1_source.required_load_g,
        output.transient_bridge);
    if (!output.transient_bridge.admitted)
    {
        output.status = ObfmLongitudinalAdmissionStatus::
            TransientLoadBridgeNotAdmitted;
        return;
    }
    if (input.energy_authority.valid)
    {
        const double authority_values[] = {
            input.energy_authority.energy_error_gain_per_s,
            input.energy_authority.energy_integral_gain_per_s2,
            input.energy_authority.energy_rate_feedback_gain,
            input.energy_authority.energy_integral_error_m2ps,
            input.energy_authority.specific_energy_rate_measured_m2ps3,
            input.energy_authority.rate_command_min_m2ps3,
            input.energy_authority.rate_command_max_m2ps3};
        for (const double value : authority_values)
        {
            if (!Finite(value))
            {
                output.status =
                    ObfmLongitudinalAdmissionStatus::InputNonfinite;
                status.code = StatusCode::NonFiniteInput;
                return;
            }
        }
    }
    AdmitCausalSpeed(
        input,
        terminal_speed,
        output.transient_bridge.feasible_interval,
        output.command_admission);
    if (!output.command_admission.admitted)
    {
        output.status = ObfmLongitudinalAdmissionStatus::
            CausalCommandNotAdmitted;
        return;
    }
    output.desired_speed_mps =
        output.command_admission.admitted_speed_mps;
    output.desired_speed_rate_mps2 =
        output.command_admission.admitted_speed_rate_mps2;
    output.admitted = true;
    output.status = ObfmLongitudinalAdmissionStatus::ReferenceAvailable;
}

void ObfmLongitudinalReferenceProvider::Evaluate(
    const ObfmLagGuidancePreparation& preparation,
    const runtime::TacticalCommandBuildInput& tactical_input,
    const ObfmLongitudinalAuthorityReceipt& authority,
    ObfmLongitudinalProviderReceipt& output,
    Status& status) const noexcept
{
    // Energy authority is diagnostic here. The current moving-point geometry
    // owns v_cmd; TECS/CIS owns downstream shaping and saturation.
    static_cast<void>(authority);
    Evaluate(preparation, tactical_input, output, status);
}

} // namespace LadyLuck
