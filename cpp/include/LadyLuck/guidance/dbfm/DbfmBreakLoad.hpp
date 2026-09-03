#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/dbfm/DbfmBreakLoadControlIntent.hpp"

namespace LadyLuck
{
Result<DbfmBreakLoadReference> BuildDbfmBreakLoadReference(
    const TacticalCommand& command,
    const DbfmBreakLoadKinematics& own,
    double instantaneous_load_limit_g) noexcept;

// Apply the active BREAK direct-NED shaping while preserving all longitudinal
// fields. Out-of-domain labels, unavailable evidence, and rejected finite
// geometry are production passthroughs. The Python authority assumes valid
// geometry and raises outside that domain; this C++ boundary deliberately
// retains the already-valid base BREAK so an optional overlay cannot create a
// frame-level command gap.
Result<TacticalCommand> ApplyDbfmBreakLoad(
    const TacticalCommand& command,
    const DbfmBreakLoadKinematics& own,
    const DbfmBreakLoadEvidence& evidence,
    const DbfmBreakLoadConfig& config);

}
