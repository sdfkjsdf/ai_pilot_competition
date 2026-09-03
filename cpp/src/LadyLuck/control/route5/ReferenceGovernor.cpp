#include "LadyLuck/control/route5/ReferenceGovernor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace
{
constexpr double AirGasConstantJpkgK = 287.05;
constexpr double AirGamma = 1.4;
constexpr double ReferenceMassKg = 11159.0;
constexpr double ArtifactGearPosition = 0.0;
constexpr double NzBackstopG = 21.0;
constexpr double FixedFallbackNzMinG = -1.0;
constexpr double FixedFallbackNzMaxG = 9.0;
constexpr double PlantNzMinG = -4.0;
constexpr double PMaxRadps = 1.0 / 0.31821;

#include "envelope/ReferenceGovernorTable.inc"

template <std::size_t Size>
std::size_t CellIndex(
    const std::array<double, Size>& axis,
    const double value) noexcept
{
    const auto position = std::lower_bound(axis.begin(), axis.end(), value);
    if (position == axis.begin())
    {
        return 0U;
    }
    if (position == axis.end())
    {
        return Size - 2U;
    }
    const std::size_t index = static_cast<std::size_t>(
        std::distance(axis.begin(), position) - 1);
    return std::min(index, Size - 2U);
}

double CellFraction(
    const double low,
    const double high,
    const double value) noexcept
{
    const double raw = (value - low) / (high - low);
    return std::min(1.0, std::max(0.0, raw));
}

double AtmosphereTemperatureK(const double altitude_m) noexcept
{
    const double altitude = std::max(altitude_m, 0.0);
    return altitude <= 11000.0
        ? 288.15 - 0.0065 * altitude
        : 216.65;
}

double BilinearNzFeasible(
    const double mach,
    const double altitude_m) noexcept
{
    const std::size_t mach_index = CellIndex(
        ReferenceGovernorMachAxis,
        mach);
    const std::size_t altitude_index = CellIndex(
        ReferenceGovernorAltitudeAxis,
        altitude_m);
    const double mach_fraction = CellFraction(
        ReferenceGovernorMachAxis[mach_index],
        ReferenceGovernorMachAxis[mach_index + 1U],
        mach);
    const double altitude_fraction = CellFraction(
        ReferenceGovernorAltitudeAxis[altitude_index],
        ReferenceGovernorAltitudeAxis[altitude_index + 1U],
        altitude_m);
    const double one_minus_mach = 1.0 - mach_fraction;
    const double one_minus_altitude = 1.0 - altitude_fraction;
    const std::size_t row0 = mach_index * ReferenceGovernorAltitudeCount;
    const std::size_t row1 = (mach_index + 1U) * ReferenceGovernorAltitudeCount;
    return
        ReferenceGovernorNzFeasible[row0 + altitude_index]
            * one_minus_mach * one_minus_altitude
        + ReferenceGovernorNzFeasible[row1 + altitude_index]
            * mach_fraction * one_minus_altitude
        + ReferenceGovernorNzFeasible[row0 + altitude_index + 1U]
            * one_minus_mach * altitude_fraction
        + ReferenceGovernorNzFeasible[row1 + altitude_index + 1U]
            * mach_fraction * altitude_fraction;
}

double NzFeasibleAtMachAltitude(
    const double mach,
    const double altitude_m,
    const double mass_kg) noexcept
{
    // d90 stall_speed_boundary computes this scale once before multiplying
    // each Mach-node sample. Preserve that binary64 operation order.
    const double scale = ReferenceMassKg / mass_kg;
    return BilinearNzFeasible(mach, altitude_m) * scale;
}

double FiniteOrZero(const double value) noexcept
{
    return std::isfinite(value) ? value : 0.0;
}

bool HasAcceptedFrameIdentity(
    const LadyLuck::EstimatorOutputV6& estimate) noexcept
{
    return estimate.measurement_reset_epoch.has_value
        && estimate.measurement_frame_index.has_value
        && estimate.accepted_sample_t_sec.has_value
        && std::isfinite(estimate.accepted_sample_t_sec.value)
        && estimate.accepted_sample_t_sec.value >= 0.0;
}

void CopyAcceptedFrameIdentity(
    const LadyLuck::EstimatorOutputV6& estimate,
    LadyLuck::ControlFrameIdentity& output) noexcept
{
    output = LadyLuck::ControlFrameIdentity{};
    output.valid = true;
    output.episode_epoch = estimate.measurement_reset_epoch.value;
    output.frame_index = estimate.measurement_frame_index.value;
    output.source_time_s = estimate.accepted_sample_t_sec.value;
}

double SafeMach(const double speed_mps, const double altitude_m) noexcept
{
    const double temperature_k = AtmosphereTemperatureK(altitude_m);
    const double sound_speed_squared =
        AirGamma * AirGasConstantJpkgK * temperature_k;
    if (!(sound_speed_squared > 0.0)
        || !std::isfinite(sound_speed_squared))
    {
        return 0.0;
    }
    const double mach = speed_mps / std::sqrt(sound_speed_squared);
    return std::isfinite(mach) ? mach : 0.0;
}

void FixedFallback(
    const LadyLuck::EstimatorOutputV6& estimate,
    const LadyLuck::control::route5::CommandEnvelopeSource source,
    LadyLuck::control::route5::CommandEnvelope& output) noexcept
{
    using LadyLuck::control::route5::CommandEnvelope;

    output = CommandEnvelope{};
    CopyAcceptedFrameIdentity(estimate, output.frame_identity);
    output.valid = true;
    output.nz_feasible_g = FixedFallbackNzMaxG;
    output.nz_min_g = FixedFallbackNzMinG;
    output.p_max_radps = PMaxRadps;
    output.nz_feasible_roll_g = FixedFallbackNzMaxG;
    output.speed_mps = FiniteOrZero(estimate.V);
    output.altitude_m = FiniteOrZero(estimate.alt);
    output.mach = SafeMach(output.speed_mps, output.altitude_m);
    output.mass_kg = FiniteOrZero(estimate.mass);
    output.enabled = true;
    output.fallback = true;
    output.gear_pos_norm = FiniteOrZero(estimate.gear_pos_norm);
    output.source = source;
    output.command_containment_authority = true;
    output.physical_authority = false;
}
}

namespace LadyLuck
{
namespace control
{
namespace route5
{

void ReferenceGovernor::EnvelopeFrom(
    const EstimatorOutputV6& estimate,
    CommandEnvelope& output,
    Status& status) const noexcept
{
    output = CommandEnvelope{};
    status = Status{};
    if (!HasAcceptedFrameIdentity(estimate))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    const double values[] = {
        estimate.V,
        estimate.alt,
        estimate.mass,
        estimate.gear_pos_norm};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            // Fixed containment is only a finite-domain fallback.  Replacing
            // NaN/Inf state with numeric bounds would hide estimator/wire
            // corruption and let an unrelated maneuver appear admissible.
            status.code = StatusCode::NonFiniteInput;
            return;
        }
    }
    if (estimate.V <= 0.0 || estimate.mass <= 0.0)
    {
        FixedFallback(
            estimate,
            CommandEnvelopeSource::FixedBoundsNonPositiveInput,
            output);
        return;
    }
    if (std::fabs(estimate.gear_pos_norm - ArtifactGearPosition) > 1.0e-12)
    {
        FixedFallback(
            estimate,
            CommandEnvelopeSource::FixedBoundsGearMismatch,
            output);
        return;
    }

    const double temperature_k = AtmosphereTemperatureK(estimate.alt);
    const double speed_of_sound_squared =
        AirGamma * AirGasConstantJpkgK * temperature_k;
    if (!(speed_of_sound_squared > 0.0)
        || !std::isfinite(speed_of_sound_squared))
    {
        FixedFallback(
            estimate,
            CommandEnvelopeSource::FixedBoundsPitchTrimDomain,
            output);
        return;
    }
    const double speed_of_sound_mps = std::sqrt(speed_of_sound_squared);
    const double mach = estimate.V / speed_of_sound_mps;
    if (!std::isfinite(mach)
        || mach < ReferenceGovernorMachAxis.front()
        || mach > ReferenceGovernorMachAxis.back()
        || estimate.alt < ReferenceGovernorAltitudeAxis.front()
        || estimate.alt > ReferenceGovernorAltitudeAxis.back())
    {
        FixedFallback(
            estimate,
            CommandEnvelopeSource::FixedBoundsPitchTrimDomain,
            output);
        return;
    }
    const double nz_feasible_g = std::min(
        NzBackstopG,
        BilinearNzFeasible(mach, estimate.alt)
            * ReferenceMassKg / std::max(estimate.mass, 1.0));
    if (!std::isfinite(nz_feasible_g)
        || nz_feasible_g <= 0.0)
    {
        FixedFallback(
            estimate,
            CommandEnvelopeSource::FixedBoundsArithmeticInvalid,
            output);
        return;
    }

    output = CommandEnvelope{};
    CopyAcceptedFrameIdentity(estimate, output.frame_identity);
    output.valid = true;
    output.nz_feasible_g = nz_feasible_g;
    output.nz_min_g = PlantNzMinG;
    output.p_max_radps = PMaxRadps;
    // Python estimator attach uses p_cmd=0.  At exactly zero roll reference,
    // nz_feasible_rolling returns the pure-pull value without consulting the
    // unported nonzero-p collapse table.
    output.nz_feasible_roll_g = nz_feasible_g;
    output.roll_reference_p_cmd_radps = 0.0;
    output.roll_reference_p_cmd_valid = true;
    output.speed_mps = estimate.V;
    output.altitude_m = estimate.alt;
    output.mach = mach;
    output.mass_kg = estimate.mass;
    output.enabled = true;
    output.fallback = false;
    output.gear_pos_norm = estimate.gear_pos_norm;
    output.source = CommandEnvelopeSource::PitchTrimArtifactCurrentNz;
    output.command_containment_authority = true;
    output.physical_authority = true;
    output.current_nz_authoritative = true;
    output.roll_nz_authoritative = true;
    output.gamma_authoritative = false;
    StallSpeedBoundary stall{};
    Status stall_status{};
    StallSpeedBoundaryFrom(estimate, 1.0, stall, stall_status);
    if (stall_status.code != StatusCode::Ok || !stall.valid
        || !std::isfinite(stall.speed_mps) || stall.speed_mps <= 0.0
        || stall.source == StallSpeedBoundarySource::Unavailable)
    {
        // Stall boundary is optional tactical evidence, not a prerequisite
        // for the already valid current-Nz command envelope.  Leave the
        // explicit stall validity fields false so dependent maneuvers
        // nonadmit without erasing ordinary guidance/FCS authority.
        return;
    }
    output.stall_speed_mps = stall.speed_mps;
    output.stall_speed_valid = true;
    output.stall_speed_resolved = stall.resolved;
    output.stall_speed_source = stall.source;
    output.stall_speed_authoritative = stall.resolved;
}

void ReferenceGovernor::StallSpeedBoundaryFrom(
    const EstimatorOutputV6& estimate,
    const double nz_target_g,
    StallSpeedBoundary& output,
    Status& status) const noexcept
{
    output = StallSpeedBoundary{};
    status = Status{};
    if (!HasAcceptedFrameIdentity(estimate))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const bool altitude_in_domain = std::isfinite(estimate.alt)
        && estimate.alt >= ReferenceGovernorAltitudeAxis.front()
        && estimate.alt <= ReferenceGovernorAltitudeAxis.back();
    double sample_altitude_m = ReferenceGovernorAltitudeAxis.front();
    if (std::isfinite(estimate.alt))
    {
        sample_altitude_m = std::min(
            ReferenceGovernorAltitudeAxis.back(),
            std::max(ReferenceGovernorAltitudeAxis.front(), estimate.alt));
    }
    const double sound_speed_mps = std::sqrt(
        AirGamma * AirGasConstantJpkgK
            * AtmosphereTemperatureK(sample_altitude_m));
    const double domain_speed_min_mps =
        ReferenceGovernorMachAxis.front() * sound_speed_mps;
    const double domain_speed_max_mps =
        ReferenceGovernorMachAxis.back() * sound_speed_mps;
    if (!std::isfinite(sound_speed_mps)
        || !std::isfinite(domain_speed_min_mps)
        || !std::isfinite(domain_speed_max_mps))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    output.valid = true;
    output.speed_mps = domain_speed_min_mps;
    output.resolved = false;
    output.source = StallSpeedBoundarySource::
        PitchTrimArtifactMachDomainFloorUnresolved;
    if (!altitude_in_domain
        || !std::isfinite(estimate.mass)
        || estimate.mass <= 0.0
        || !std::isfinite(nz_target_g))
    {
        return;
    }

    const double target_g = std::max(nz_target_g, 0.2);
    const double first_nz_g = NzFeasibleAtMachAltitude(
        ReferenceGovernorMachAxis.front(),
        estimate.alt,
        estimate.mass);
    if (!std::isfinite(first_nz_g))
    {
        output = StallSpeedBoundary{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (first_nz_g >= target_g)
    {
        return;
    }

    double nz_low_g = first_nz_g;
    for (std::size_t index = 0U;
        index + 1U < ReferenceGovernorMachCount;
        ++index)
    {
        const double nz_high_g = NzFeasibleAtMachAltitude(
            ReferenceGovernorMachAxis[index + 1U],
            estimate.alt,
            estimate.mass);
        if (!std::isfinite(nz_high_g))
        {
            output = StallSpeedBoundary{};
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        if (nz_low_g < target_g && target_g <= nz_high_g)
        {
            const double fraction = (target_g - nz_low_g)
                / (nz_high_g - nz_low_g);
            const double crossing_mach = ReferenceGovernorMachAxis[index]
                + fraction * (ReferenceGovernorMachAxis[index + 1U]
                    - ReferenceGovernorMachAxis[index]);
            const double crossing_speed_mps =
                crossing_mach * sound_speed_mps;
            if (!std::isfinite(crossing_speed_mps))
            {
                output = StallSpeedBoundary{};
                status.code = StatusCode::NonFiniteInput;
                return;
            }
            output.speed_mps = crossing_speed_mps;
            output.resolved = true;
            output.source = StallSpeedBoundarySource::
                PitchTrimArtifactGridFirstCrossing;
            return;
        }
        nz_low_g = nz_high_g;
    }

    output.speed_mps = domain_speed_max_mps;
    output.resolved = false;
    output.source = StallSpeedBoundarySource::
        PitchTrimArtifactMachDomainCeilingUnresolved;
}

void ReferenceGovernor::CopyNzfeasAuthorityReceipt(
    const CommandEnvelope& envelope,
    ReferenceGovernorNzfeasAuthorityReceipt& output,
    Status& status) const noexcept
{
    output = ReferenceGovernorNzfeasAuthorityReceipt{};
    status = Status{};
    output.frame_identity = envelope.frame_identity;
    output.evaluated = true;
    output.valid = true;
    output.envelope_source = envelope.source;
    output.artifact_gear_position = ArtifactGearPosition;
    output.artifact_reference_mass_kg = ReferenceMassKg;
    output.canonical_table_authority =
        IsPhysicalNzCommandEnvelopeSource(envelope.source);
    output.artifact_source = output.canonical_table_authority
        ? ReferenceGovernorNzfeasArtifactSource::CorrectedGearUpD90
        : ReferenceGovernorNzfeasArtifactSource::Unavailable;
}

} // namespace route5
} // namespace control
} // namespace LadyLuck
