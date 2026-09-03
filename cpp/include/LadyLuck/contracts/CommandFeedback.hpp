#pragma once

#include "LadyLuck/contracts/Enums.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"

#include <array>
#include <cstdint>

namespace LadyLuck
{
using Command4Wire = std::array<float, 4>;
using Command4Estimator = std::array<double, 4>;

struct CommandFeedback
{
    // ResetSeed is an estimator-local double literal, matching
    // ControlCommand.neutral().as_array() before any Provider float32 wire
    // payload exists. For PreviousTransmittedAssumption the integration owner
    // must promote each exact wire float to estimator_command_u_dll.
    Command4Estimator estimator_command_u_dll{{0.0, 0.0, 0.0, 0.65}};
    bool has_transmitted_wire_payload = false;
    Command4Wire transmitted_wire_payload{};
    ActionFeedbackKind kind = ActionFeedbackKind::ResetSeed;
    OptionalFrameIndex source_frame_index{};
    OptionalSeconds source_t_sec{};
    std::uint32_t delay_frames = 1U;
};
}
