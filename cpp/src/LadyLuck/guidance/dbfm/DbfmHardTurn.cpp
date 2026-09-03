#include "LadyLuck/guidance/dbfm/DbfmHardTurn.hpp"

namespace LadyLuck
{
Result<TacticalCommand> BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame)
{
    Result<TacticalCommand> result{};
    DbfmHardTurnReference reference{};
    BuildDbfmHardTurnReference(frame, reference, result.status);
    if (result.status.code != StatusCode::Ok)
    {
        return result;
    }

    TacticalCommand candidate{};
    candidate.aim_point_m = reference.aim_point_m;
    candidate.desired_speed_mps = reference.desired_speed_mps;
    candidate.capture_range_des_m = reference.capture_range_des_m;
    try
    {
        candidate.behavior_label = "HARD_TURN";
        candidate.mode_label = "DBFM";
    }
    catch (...)
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }
    return MakeValidatedTacticalCommand(candidate);
}
}
