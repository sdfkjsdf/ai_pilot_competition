#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/dbfm/DbfmHardTurnControlIntent.hpp"

#include <cstdint>

namespace LadyLuck
{
// Build the raw DBFM HARD_TURN aim and speed reference.  Conversion to body
// rates, load factor, control surfaces, and thrust remains downstream-owned.
Result<TacticalCommand> BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame);
}
