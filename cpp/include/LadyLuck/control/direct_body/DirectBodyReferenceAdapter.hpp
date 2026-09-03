#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"

namespace LadyLuck
{
namespace control
{
namespace direct_body
{

// Exact allocation-free C++ port of the d90 load-factor route used when a
// TacticalCommand carries direct p/Nz references.  The resulting p/q/r/Nz and
// energy references are still inputs to TECS/CIS; they are not surface or
// aircraft-response measurements.
void BuildDirectBodyReference(
    const ControlIntent& command,
    const PlaneState& ownship,
    const EstimatorOutputV6& estimate,
    const route5::CommandEnvelope& envelope,
    double gamma_command_limit_rad,
    tecs_cis::BodyRateLoadEnergyCommand& output,
    Status& status) noexcept;

} // namespace direct_body
} // namespace control
} // namespace LadyLuck
