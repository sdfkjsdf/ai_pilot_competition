#include "LadyLuck/geometry/WezRule.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
using LadyLuck::WezPhase;
using LadyLuck::WezPhaseId;

const std::array<WezPhase, LadyLuck::OfficialWezPhaseCount> Phases{{
    WezPhase{
        WezPhaseId::P1,
        1U,
        0.0,
        1.0 * LadyLuck::DegreesToRadians,
        1.0,
        500.0 * LadyLuck::constants::FeetToMeters,
        3000.0 * LadyLuck::constants::FeetToMeters,
        1.0},
    WezPhase{
        WezPhaseId::P2,
        2U,
        100.0,
        2.0 * LadyLuck::DegreesToRadians,
        2.0,
        500.0 * LadyLuck::constants::FeetToMeters,
        3500.0 * LadyLuck::constants::FeetToMeters,
        0.3},
    WezPhase{
        WezPhaseId::P3,
        3U,
        150.0,
        3.0 * LadyLuck::DegreesToRadians,
        3.0,
        500.0 * LadyLuck::constants::FeetToMeters,
        4000.0 * LadyLuck::constants::FeetToMeters,
        0.1}}};

const std::array<double, LadyLuck::OfficialWezPhaseCount> MinRangesFeet{{
    500.0,
    500.0,
    500.0}};
const std::array<double, LadyLuck::OfficialWezPhaseCount> MaxRangesFeet{{
    3000.0,
    3500.0,
    4000.0}};

bool Finite3(
    const double first,
    const double second,
    const double third) noexcept
{
    return std::isfinite(first)
        && std::isfinite(second)
        && std::isfinite(third);
}

LadyLuck::Result<double> InvalidScalarResult() noexcept
{
    LadyLuck::Result<double> result{};
    result.status.code = LadyLuck::StatusCode::NonFiniteInput;
    return result;
}

double Sigmoid(const double x) noexcept
{
    if (x >= 0.0)
    {
        const double z = std::exp(-x);
        return 1.0 / (1.0 + z);
    }
    const double z = std::exp(x);
    return z / (1.0 + z);
}

double AngularPotential(
    const double los_deg,
    const double los_max_deg,
    const double sigma_deg) noexcept
{
    const double over = std::max(0.0, std::fabs(los_deg) - los_max_deg);
    return std::exp(-over / std::max(1.0e-6, sigma_deg));
}

double RangeBand(
    const double distance_ft,
    const double min_range_ft,
    const double max_range_ft) noexcept
{
    constexpr double softness_ft = 120.0;
    return Sigmoid((distance_ft - min_range_ft) / softness_ft)
        * Sigmoid((max_range_ft - distance_ft) / softness_ft);
}

LadyLuck::Result<double> OfficialDamageInPhaseUnits(
    const double los_deg,
    const double distance,
    const double t_sec,
    const bool use_meters) noexcept
{
    if (!Finite3(los_deg, distance, t_sec))
    {
        return InvalidScalarResult();
    }

    LadyLuck::Result<double> result{};
    const double los = std::fabs(los_deg);
    for (std::size_t index = 0U; index < Phases.size(); ++index)
    {
        const WezPhase& phase = Phases[index];
        const double min_range = use_meters
            ? phase.min_range_m
            : MinRangesFeet[index];
        const double max_range = use_meters
            ? phase.max_range_m
            : MaxRangesFeet[index];
        if (t_sec >= phase.start_sec
            && los < phase.angle_deg
            && distance >= min_range
            && distance <= max_range)
        {
            const double falloff =
                (max_range - distance) / (max_range - min_range);
            result.value = phase.coeff * falloff;
            return result;
        }
    }
    return result;
}
}

namespace LadyLuck
{
Result<WezPhase> OfficialWezPhaseAt(const std::size_t index) noexcept
{
    Result<WezPhase> result{};
    if (index >= Phases.size())
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    result.value = Phases[index];
    return result;
}

Result<WezPhase> ActiveWezPhase(const double t_sec) noexcept
{
    Result<WezPhase> result{};
    if (!std::isfinite(t_sec))
    {
        result.status.code = StatusCode::NonFiniteInput;
        return result;
    }
    result.value = Phases[0U];
    for (const WezPhase& phase : Phases)
    {
        if (t_sec >= phase.start_sec)
        {
            result.value = phase;
        }
    }
    return result;
}

Result<double> OfficialDamageCoeffFeet(
    const double los_deg,
    const double distance_ft,
    const double t_sec) noexcept
{
    return OfficialDamageInPhaseUnits(los_deg, distance_ft, t_sec, false);
}

Result<double> OfficialDamageCoeffMeters(
    const double los_deg,
    const double distance_m,
    const double t_sec) noexcept
{
    // Compare in meters directly. This intentionally avoids a meter -> feet
    // round trip at the official inclusive 500 ft boundary.
    return OfficialDamageInPhaseUnits(los_deg, distance_m, t_sec, true);
}

Result<double> SoftOffensePotential(
    const double los_deg,
    const double distance_m,
    const double t_sec,
    const double sigma_deg) noexcept
{
    if (!Finite3(los_deg, distance_m, t_sec) || !std::isfinite(sigma_deg))
    {
        return InvalidScalarResult();
    }

    Result<double> result{};
    const double los = std::fabs(los_deg);
    const double distance_ft = distance_m * constants::MetersToFeet;
    for (std::size_t index = 0U; index < Phases.size(); ++index)
    {
        const WezPhase& phase = Phases[index];
        if (t_sec < phase.start_sec)
        {
            continue;
        }
        const double min_range_ft = MinRangesFeet[index];
        const double max_range_ft = MaxRangesFeet[index];
        const double falloff = std::max(
            0.0,
            std::min(
                1.0,
                (max_range_ft - distance_ft)
                    / (max_range_ft - min_range_ft)));
        const double value = phase.coeff
            * falloff
            * AngularPotential(los, phase.angle_deg, sigma_deg)
            * RangeBand(distance_ft, min_range_ft, max_range_ft);
        result.value = std::max(result.value, value);
    }
    if (!std::isfinite(result.value))
    {
        return InvalidScalarResult();
    }
    return result;
}

Result<double> TightConePotential(
    const double los_deg,
    const double distance_m,
    const double sigma_deg) noexcept
{
    if (!Finite3(los_deg, distance_m, sigma_deg))
    {
        return InvalidScalarResult();
    }
    Result<double> result{};
    const double distance_ft = distance_m * constants::MetersToFeet;
    const double falloff = std::max(
        0.0,
        std::min(1.0, (3000.0 - distance_ft) / 2500.0));
    result.value = AngularPotential(los_deg, 1.0, sigma_deg)
        * RangeBand(distance_ft, 500.0, 3000.0)
        * falloff;
    if (!std::isfinite(result.value))
    {
        return InvalidScalarResult();
    }
    return result;
}
}
