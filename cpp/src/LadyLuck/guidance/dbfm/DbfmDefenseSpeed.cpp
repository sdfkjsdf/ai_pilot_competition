#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeed.hpp"

#include <cmath>

namespace
{
bool AdmittedUpper(
    const LadyLuck::OptionalValue<double>& upper_mps,
    const bool admitted,
    double& value) noexcept
{
    if (!admitted || !upper_mps.has_value)
    {
        return false;
    }
    if (!std::isfinite(upper_mps.value) || upper_mps.value <= 0.0)
    {
        return false;
    }
    value = upper_mps.value;
    return true;
}
}

namespace LadyLuck
{
Result<TacticalCommand> ApplyDbfmDefenseSpeed(
    const TacticalCommand& command,
    const DbfmCornerSpeedEvidence& evidence)
{
    const OptionalValue<double>* selected_upper = nullptr;
    bool selected_admitted = false;
    if (command.behavior_label == "BREAK"
        || command.behavior_label == "GUN_DEFENSE_HORIZONTAL_BREAK"
        || command.behavior_label == "GUN_DEFENSE_TRACKING_JINK")
    {
        selected_upper = &evidence.instantaneous_upper_mps;
        selected_admitted = evidence.instantaneous_admitted;
    }
    else if (command.behavior_label == "HARD_TURN")
    {
        selected_upper = &evidence.sustained_upper_mps;
        selected_admitted = evidence.sustained_admitted;
    }

    double speed_reference_mps = 0.0;
    if (selected_upper == nullptr
        || !AdmittedUpper(
            *selected_upper,
            selected_admitted,
            speed_reference_mps))
    {
        Result<TacticalCommand> passthrough{};
        try
        {
            passthrough.value = command;
        }
        catch (...)
        {
            passthrough.status.code = StatusCode::InvalidConfiguration;
        }
        return passthrough;
    }

    Result<TacticalCommand> result{};
    TacticalCommand candidate{};
    try
    {
        candidate = command;
    }
    catch (...)
    {
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }
    candidate.desired_speed_mps = speed_reference_mps;
    candidate.desired_speed_rate_mps2 = 0.0;
    return MakeValidatedTacticalCommand(candidate);
}

}
