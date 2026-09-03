#include "LadyLuck/plant/dynamics/EngineModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace LadyLuck
{
namespace plant
{
namespace dynamics
{

namespace
{

constexpr double PoundsForceToNewtons = 4.4482216;
constexpr double MilThrustN = 17800.0 * PoundsForceToNewtons;
constexpr double MaximumThrustN = 29000.0 * PoundsForceToNewtons;
constexpr double MetresPerFoot = 0.3048;
constexpr double IdleN2 = 53.0;
constexpr double MaximumN2 = 100.0;
constexpr double N2Factor = MaximumN2 - IdleN2;
constexpr double Tsfc = 0.74;
constexpr double AugmentedTsfc = 2.05;
constexpr double BypassRatio = 0.4;
constexpr double SpoolDelayUp = 1.0 * 90.0 / (BypassRatio + 3.0);
constexpr double SpoolDelayDown = 3.0 * 90.0 / (BypassRatio + 3.0);
constexpr double RankineReference = 389.7;
constexpr double SeaLevelDensity = 1.225;
constexpr double IdleThrottle = 0.5;
constexpr double FullMilThrottle = 0.75;

constexpr double MachAxis[14] = {
    0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2,
    1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6
};
constexpr double IdleMachAxis[6] = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
constexpr double MilMachAxis[8] = {
    0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4
};
constexpr double AltitudeAxisFt[8] = {
    -10000.0, 0.0, 10000.0, 20000.0,
    30000.0, 40000.0, 50000.0, 60000.0
};

constexpr double IdleTable[6][8] = {
    {0.0430, 0.0488, 0.0528, 0.0694, 0.0899, 0.1183, 0.1467, 0.0},
    {0.0500, 0.0501, 0.0335, 0.0544, 0.0797, 0.1049, 0.1342, 0.0},
    {0.0040, 0.0047, 0.0020, 0.0272, 0.0595, 0.0891, 0.1203, 0.0},
    {-0.0804, -0.0804, -0.0560, -0.0237, 0.0276, 0.0718, 0.1073, 0.0},
    {-0.2129, -0.2129, -0.1498, -0.1025, 0.0474, 0.0868, 0.0900, 0.0},
    {-0.2839, -0.2839, -0.1104, -0.0469, -0.0270, 0.0552, 0.0800, 0.0}
};

constexpr double MilTable[8][8] = {
    {1.2600, 1.0000, 0.7400, 0.5340, 0.3720, 0.2410, 0.1490, 0.0},
    {1.1710, 0.9340, 0.6970, 0.5060, 0.3550, 0.2310, 0.1430, 0.0},
    {1.1500, 0.9210, 0.6920, 0.5060, 0.3570, 0.2330, 0.1450, 0.0},
    {1.1810, 0.9510, 0.7210, 0.5320, 0.3780, 0.2480, 0.1540, 0.0},
    {1.2580, 1.0200, 0.7820, 0.5820, 0.4170, 0.2750, 0.1700, 0.0},
    {1.3690, 1.1200, 0.8710, 0.6510, 0.4750, 0.3150, 0.1950, 0.0},
    {1.4850, 1.2300, 0.9750, 0.7440, 0.5450, 0.3640, 0.2250, 0.0},
    {1.5941, 1.3400, 1.0860, 0.8450, 0.6280, 0.4240, 0.2630, 0.0}
};

constexpr double AugmentedTable[14][8] = {
    {1.1816, 1.0000, 0.8184, 0.6627, 0.5280, 0.3756, 0.2327, 0.0},
    {1.1308, 0.9599, 0.7890, 0.6406, 0.5116, 0.3645, 0.2258, 0.0},
    {1.1150, 0.9474, 0.7798, 0.6340, 0.5070, 0.3615, 0.2240, 0.0},
    {1.1284, 0.9589, 0.7894, 0.6420, 0.5134, 0.3661, 0.2268, 0.0},
    {1.1707, 0.9942, 0.8177, 0.6647, 0.5309, 0.3784, 0.2345, 0.0},
    {1.2411, 1.0529, 0.8648, 0.7017, 0.5596, 0.3983, 0.2467, 0.0},
    {1.3287, 1.1254, 0.9221, 0.7462, 0.5936, 0.4219, 0.2614, 0.0},
    {1.4365, 1.2149, 0.9933, 0.8021, 0.6360, 0.4509, 0.2794, 0.0},
    {1.5711, 1.3260, 1.0809, 0.8700, 0.6874, 0.4860, 0.3011, 0.0},
    {1.7301, 1.4579, 1.1857, 0.9512, 0.7495, 0.5289, 0.3277, 0.0},
    {1.8314, 1.5700, 1.3086, 1.0474, 0.8216, 0.5786, 0.3585, 0.0},
    {1.9700, 1.6900, 1.4100, 1.2400, 0.9100, 0.6359, 0.3940, 0.0},
    {2.0700, 1.8000, 1.5300, 1.3400, 1.0000, 0.7200, 0.4600, 0.0},
    {2.2000, 1.9200, 1.6400, 1.4400, 1.1000, 0.8000, 0.5200, 0.0}
};

struct EngineCoefficients
{
    double idle = 0.0;
    double mil = 0.0;
    double augmented = 0.0;
};

template <std::size_t Rows, std::size_t Columns>
double Interpolate2dClamped(
    const double (&row_axis)[Rows],
    const double (&column_axis)[Columns],
    const double (&table)[Rows][Columns],
    const double row_value,
    const double column_value) noexcept
{
    const double x = std::min(std::max(row_value, row_axis[0]), row_axis[Rows - 1U]);
    const double y = std::min(
        std::max(column_value, column_axis[0]),
        column_axis[Columns - 1U]);

    std::size_t row_upper = 1U;
    while (row_upper < Rows && row_axis[row_upper] < x)
    {
        ++row_upper;
    }
    std::size_t column_upper = 1U;
    while (column_upper < Columns && column_axis[column_upper] < y)
    {
        ++column_upper;
    }
    if (row_upper >= Rows)
    {
        row_upper = Rows - 1U;
    }
    if (column_upper >= Columns)
    {
        column_upper = Columns - 1U;
    }
    const std::size_t row_lower = row_upper - 1U;
    const std::size_t column_lower = column_upper - 1U;
    const double x0 = row_axis[row_lower];
    const double x1 = row_axis[row_upper];
    const double y0 = column_axis[column_lower];
    const double y1 = column_axis[column_upper];
    const double tx = x1 > x0 ? (x - x0) / (x1 - x0) : 0.0;
    const double ty = y1 > y0 ? (y - y0) / (y1 - y0) : 0.0;
    const double d00 = table[row_lower][column_lower];
    const double d10 = table[row_upper][column_lower];
    const double d01 = table[row_lower][column_upper];
    const double d11 = table[row_upper][column_upper];
    return d00 * (1.0 - tx) * (1.0 - ty)
        + d10 * tx * (1.0 - ty)
        + d01 * (1.0 - tx) * ty
        + d11 * tx * ty;
}

EngineCoefficients Coefficients(
    const double mach,
    const double altitude_m) noexcept
{
    const double altitude_ft = altitude_m / MetresPerFoot;
    EngineCoefficients coefficients;
    coefficients.idle = Interpolate2dClamped(
        IdleMachAxis,
        AltitudeAxisFt,
        IdleTable,
        mach,
        altitude_ft);
    coefficients.mil = Interpolate2dClamped(
        MilMachAxis,
        AltitudeAxisFt,
        MilTable,
        mach,
        altitude_ft);
    coefficients.augmented = Interpolate2dClamped(
        MachAxis,
        AltitudeAxisFt,
        AugmentedTable,
        mach,
        altitude_ft);
    return coefficients;
}

double SpoolRate(
    const double n2_normalized,
    const double density_ratio,
    const double delay) noexcept
{
    const double n = std::min(1.0, n2_normalized + 0.1);
    return delay / (1.0 + 3.0 * std::pow(1.0 - n, 3.0)
        + (1.0 - density_ratio));
}

double Seek(
    double value,
    const double target,
    const double acceleration,
    const double deceleration,
    const double dt_s) noexcept
{
    if (value > target)
    {
        value -= dt_s * deceleration;
        if (value < target)
        {
            value = target;
        }
    }
    else if (value < target)
    {
        value += dt_s * acceleration;
        if (value > target)
        {
            value = target;
        }
    }
    return value;
}

double IdleFuelFlowPph() noexcept
{
    return std::pow(MilThrustN / PoundsForceToNewtons, 0.2) * 107.0;
}

} // namespace

EngineModel::EngineModel() noexcept
    : n2_percent_(MaximumN2),
      fuel_flow_pph_(IdleFuelFlowPph())
{
}

PlantStatus EngineModel::Reset() noexcept
{
    n2_percent_ = MaximumN2;
    fuel_flow_pph_ = IdleFuelFlowPph();
    return PlantStatus::Success();
}

double EngineModel::ThrottlePosition(const double throttle_command) noexcept
{
    return std::min(
        std::max(
            (throttle_command - IdleThrottle)
                / (FullMilThrottle - IdleThrottle),
            0.0),
        2.0);
}

PlantResult<AtmosphereState> EngineModel::Atmosphere(
    const double altitude_m) noexcept
{
    PlantResult<AtmosphereState> result;
    if (!std::isfinite(altitude_m))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "engine atmosphere altitude must be finite");
        return result;
    }
    const double altitude = std::max(altitude_m, 0.0);
    double temperature = 0.0;
    double pressure = 0.0;
    if (altitude <= 11000.0)
    {
        temperature = 288.15 - 0.0065 * altitude;
        pressure = 101325.0 * std::pow(temperature / 288.15, 5.2559);
    }
    else
    {
        temperature = 216.65;
        pressure = 22632.0 * std::exp(-(altitude - 11000.0) / 6341.6);
    }
    const double density = pressure / (287.05 * temperature);
    result.value.temperature_k = temperature;
    result.value.density_kg_m3 = density;
    result.value.density_ratio = density / SeaLevelDensity;
    if (!std::isfinite(density) || density <= 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "engine atmosphere produced an invalid density");
        return result;
    }
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<double> EngineModel::StaticFuelFlowLbPerSecond(
    const double throttle_command,
    const double mach,
    const double altitude_m) noexcept
{
    PlantResult<double> result;
    if (!std::isfinite(throttle_command) || !std::isfinite(mach)
        || !std::isfinite(altitude_m))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "static engine input must be finite");
        return result;
    }
    const EngineCoefficients coefficients = Coefficients(mach, altitude_m);
    const double idle_lbf = MilThrustN * coefficients.idle
        / PoundsForceToNewtons;
    const double mil_delta_lbf = (MilThrustN / PoundsForceToNewtons - idle_lbf)
        * coefficients.mil;
    const double mil_full_lbf = idle_lbf + mil_delta_lbf;
    const double maximum_static_lbf = MaximumThrustN
        * coefficients.augmented / PoundsForceToNewtons;
    const double throttle_position = ThrottlePosition(throttle_command);
    if (throttle_position <= 1.0)
    {
        const double n2_normalized = std::min(
            std::max(throttle_position, 0.0),
            1.0);
        const double thrust_lbf = idle_lbf
            + mil_delta_lbf * n2_normalized * n2_normalized;
        const PlantResult<AtmosphereState> atmosphere = Atmosphere(altitude_m);
        if (!atmosphere.ok())
        {
            result.status = atmosphere.status;
            return result;
        }
        const double corrected_tsfc = Tsfc * std::sqrt(
            atmosphere.value.temperature_k * 1.8 / RankineReference)
            * (0.84 + std::pow(1.0 - n2_normalized, 2.0));
        result.value = std::max(
            IdleFuelFlowPph(),
            thrust_lbf * corrected_tsfc) / 3600.0;
    }
    else
    {
        const double augmentation = std::min(throttle_position - 1.0, 1.0);
        const double augmented_thrust_lbf = mil_full_lbf
            + (maximum_static_lbf - mil_full_lbf) * augmentation;
        result.value = AugmentedTsfc * augmented_thrust_lbf / 3600.0;
    }
    if (!std::isfinite(result.value) || result.value < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "static fuel-flow model produced an invalid result");
        return result;
    }
    result.status = PlantStatus::Success();
    return result;
}

PlantResult<double> EngineModel::SeedStaticFuelFlow(
    const double throttle_command,
    const double mach,
    const double altitude_m) noexcept
{
    PlantResult<double> result = StaticFuelFlowLbPerSecond(
        throttle_command,
        mach,
        altitude_m);
    if (!result.ok())
    {
        return result;
    }
    fuel_flow_pph_ = result.value * 3600.0;
    return result;
}

PlantResult<EngineStepOutput> EngineModel::ThrustAndFuel(
    const double throttle_command,
    const double mach,
    const double altitude_m,
    const double dt_s) noexcept
{
    PlantResult<EngineStepOutput> result;
    if (!std::isfinite(throttle_command) || !std::isfinite(mach)
        || !std::isfinite(altitude_m) || !std::isfinite(dt_s)
        || dt_s <= 0.0 || !std::isfinite(n2_percent_)
        || !std::isfinite(fuel_flow_pph_))
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::InvalidArgument,
            "dynamic engine input/state must be finite and dt positive");
        return result;
    }

    const double throttle_position = ThrottlePosition(throttle_command);
    const PlantResult<AtmosphereState> atmosphere = Atmosphere(altitude_m);
    if (!atmosphere.ok())
    {
        result.status = atmosphere.status;
        return result;
    }

    double candidate_n2 = n2_percent_;
    double candidate_fuel_flow_pph = fuel_flow_pph_;
    const double n2_command = IdleN2
        + std::min(throttle_position, 1.0) * N2Factor;
    const double current_n2_normalized = std::min(
        std::max((candidate_n2 - IdleN2) / N2Factor, 0.0),
        1.0);
    if (n2_command > candidate_n2)
    {
        const double acceleration = SpoolRate(
            current_n2_normalized,
            atmosphere.value.density_ratio,
            SpoolDelayUp);
        candidate_n2 = Seek(
            candidate_n2,
            n2_command,
            acceleration,
            1.0e9,
            dt_s);
    }
    else if (n2_command < candidate_n2)
    {
        const double deceleration = SpoolRate(
            current_n2_normalized,
            atmosphere.value.density_ratio,
            SpoolDelayDown);
        candidate_n2 = Seek(
            candidate_n2,
            n2_command,
            1.0e9,
            deceleration,
            dt_s);
    }
    const double n2_normalized = std::min(
        std::max((candidate_n2 - IdleN2) / N2Factor, 0.0),
        1.0);
    const EngineCoefficients coefficients = Coefficients(mach, altitude_m);
    const double idle_thrust = MilThrustN * coefficients.idle;
    const double mil_delta = (MilThrustN - idle_thrust) * coefficients.mil;
    const double mil_thrust = idle_thrust
        + mil_delta * n2_normalized * n2_normalized;
    double thrust = mil_thrust;
    const double augmentation = std::min(
        std::max(throttle_position - 1.0, 0.0),
        1.0);
    if (augmentation > 0.0)
    {
        const double maximum_thrust = MaximumThrustN * coefficients.augmented;
        thrust = mil_thrust + (maximum_thrust - mil_thrust) * augmentation;
        const double target_fuel_flow =
            (thrust / PoundsForceToNewtons) * AugmentedTsfc;
        candidate_fuel_flow_pph = Seek(
            candidate_fuel_flow_pph,
            target_fuel_flow,
            5000.0,
            10000.0,
            dt_s);
    }
    else
    {
        const double temperature_rankine = atmosphere.value.temperature_k * 1.8;
        const double corrected_tsfc = Tsfc
            * std::sqrt(temperature_rankine / RankineReference)
            * (0.84 + std::pow(1.0 - n2_normalized, 2.0));
        const double target_fuel_flow =
            (mil_thrust / PoundsForceToNewtons) * corrected_tsfc;
        candidate_fuel_flow_pph = Seek(
            candidate_fuel_flow_pph,
            target_fuel_flow,
            1000.0,
            10000.0,
            dt_s);
    }

    // JSBSim FGTurbine applies the idle-flow floor after both dry and
    // augmented fuel-flow updates.  At low spool, a negative idle-thrust
    // table coefficient can make the transient augmented target negative;
    // that aerodynamic windmilling term must not become negative fuel burn.
    candidate_fuel_flow_pph = std::max(
        candidate_fuel_flow_pph,
        IdleFuelFlowPph());

    if (!std::isfinite(candidate_n2) || !std::isfinite(candidate_fuel_flow_pph)
        || !std::isfinite(thrust) || candidate_fuel_flow_pph < 0.0)
    {
        result.status = PlantStatus::Failure(
            PlantStatusCode::NonFiniteResult,
            "dynamic engine produced an invalid result");
        return result;
    }

    n2_percent_ = candidate_n2;
    fuel_flow_pph_ = candidate_fuel_flow_pph;
    result.value.thrust_n = thrust;
    result.value.fuel_flow_lb_s = candidate_fuel_flow_pph / 3600.0;
    result.status = PlantStatus::Success();
    return result;
}

double EngineModel::n2_percent() const noexcept
{
    return n2_percent_;
}

double EngineModel::fuel_flow_pph() const noexcept
{
    return fuel_flow_pph_;
}

} // namespace dynamics
} // namespace plant
} // namespace LadyLuck
