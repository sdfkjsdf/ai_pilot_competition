#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeedControlIntent.hpp"

namespace LadyLuck
{
struct DbfmCornerSpeedEvidence
{
    OptionalValue<double> instantaneous_upper_mps{};
    bool instantaneous_admitted = false;
    OptionalValue<double> sustained_upper_mps{};
    bool sustained_admitted = false;
};

// Apply the admitted defensive-corner speed reference while preserving every
// other raw-guidance field.
Result<TacticalCommand> ApplyDbfmDefenseSpeed(
    const TacticalCommand& command,
    const DbfmCornerSpeedEvidence& evidence);

}
