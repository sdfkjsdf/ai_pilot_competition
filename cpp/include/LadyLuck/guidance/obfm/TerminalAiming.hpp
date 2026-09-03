#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Same-frame output of d90 tracking_attitude_direct_command().  Exact-zero
// range or speed is a finite non-admission; malformed state is returned
// through Status and never replaced with a level-flight command.
struct TerminalTrackingReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    double p_cmd_radps = 0.0;
    double nz_cmd_g = 0.0;
};

void EvaluateTerminalTracking(
    const DogfightGeometryFrame& frame,
    bool enabled,
    bool eligible_obfm_pursuit_behavior,
    TerminalTrackingReceipt& output,
    Status& status) noexcept;

// Pure post-command overlay.  The caller owns the visible BT selection of
// LAG/FOLLOW/EMPLOY; this function only applies an admitted receipt and keeps
// every longitudinal/aim/reference field of the upstream intent unchanged.
void ApplyTerminalTracking(
    const ControlIntent& upstream,
    const TerminalTrackingReceipt& receipt,
    ControlIntent& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<TerminalTrackingReceipt>::value,
    "Terminal-tracking receipts must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
