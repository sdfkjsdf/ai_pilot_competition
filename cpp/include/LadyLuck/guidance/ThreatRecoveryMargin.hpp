#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <type_traits>

namespace LadyLuck
{
namespace guidance
{

// Command-neutral comparison between the time remaining before the opponent's
// official WEZ and the ownship's admitted 180-degree level-turn recovery time.
// Missing capability or unrepresentable arithmetic is ordinary unevaluated
// evidence; a non-positive closure is a resolved infinite WEZ margin.
struct ThreatRecoveryMarginReceipt
{
    bool evaluated = false;
    double closing_speed_mps = 0.0;
    bool time_to_enemy_wez_valid = false;
    double time_to_enemy_wez_s = 0.0;
    bool own_reversal_time_valid = false;
    double own_reversal_time_s = 0.0;
    bool exhausted = false;
};

void EvaluateThreatRecoveryMargin(
    const DogfightGeometryFrame& frame,
    bool own_turn_capability_admitted,
    double own_turn_capability_g,
    ThreatRecoveryMarginReceipt& output) noexcept;

static_assert(
    std::is_trivially_copyable<ThreatRecoveryMarginReceipt>::value,
    "Threat recovery margin evidence must remain allocation-free.");

} // namespace guidance
} // namespace LadyLuck
