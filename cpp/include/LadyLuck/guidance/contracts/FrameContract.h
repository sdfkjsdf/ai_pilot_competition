#pragma once

#include "LadyLuck/runtime/AIPilotABI.h"

namespace AIP_Guidance
{
bool BuildFrameContractV1(
    const KinematicObservationInputV1& input,
    FrameContractDiagnosticsV1& output) noexcept;
}
