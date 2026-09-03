#include "LadyLuck/contracts/FrameContext.hpp"

#include <cmath>

namespace LadyLuck
{

bool IsValidControlFrameIdentity(
    const ControlFrameIdentity& identity) noexcept
{
    return identity.valid
        && std::isfinite(identity.source_time_s)
        && identity.source_time_s >= 0.0;
}

bool SameControlFrameIdentity(
    const ControlFrameIdentity& left,
    const ControlFrameIdentity& right) noexcept
{
    // Source time is derived from the accepted frame index. It is telemetry,
    // not a second chronology authority.
    return IsValidControlFrameIdentity(left)
        && IsValidControlFrameIdentity(right)
        && left.episode_epoch == right.episode_epoch
        && left.frame_index == right.frame_index;
}

} // namespace LadyLuck
