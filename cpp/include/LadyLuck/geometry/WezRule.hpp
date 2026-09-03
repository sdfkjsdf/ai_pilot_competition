#pragma once

#include "LadyLuck/contracts/Status.hpp"

#include <cstddef>
#include <cstdint>

namespace LadyLuck
{
enum class WezPhaseId : std::uint8_t
{
    P1 = 1U,
    P2 = 2U,
    P3 = 3U
};

struct WezPhase
{
    WezPhaseId id = WezPhaseId::P1;
    std::uint8_t index = 1U;
    double start_sec = 0.0;
    double angle_rad = 0.0;
    double angle_deg = 0.0;
    double min_range_m = 0.0;
    double max_range_m = 0.0;
    double coeff = 0.0;
};

constexpr std::size_t OfficialWezPhaseCount = 3U;

Result<WezPhase> OfficialWezPhaseAt(std::size_t index) noexcept;
Result<WezPhase> ActiveWezPhase(double t_sec) noexcept;

// The hard-damage functions preserve the official P1 -> P2 -> P3 early-return
// order. Cone bounds are strict and range bounds are inclusive.
Result<double> OfficialDamageCoeffFeet(
    double los_deg,
    double distance_ft,
    double t_sec) noexcept;
Result<double> OfficialDamageCoeffMeters(
    double los_deg,
    double distance_m,
    double t_sec) noexcept;

Result<double> SoftOffensePotential(
    double los_deg,
    double distance_m,
    double t_sec,
    double sigma_deg = 8.0) noexcept;
Result<double> TightConePotential(
    double los_deg,
    double distance_m,
    double sigma_deg = 8.0) noexcept;
}
