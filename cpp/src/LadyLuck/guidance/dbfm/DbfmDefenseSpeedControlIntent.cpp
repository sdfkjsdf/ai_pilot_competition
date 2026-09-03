#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeedControlIntent.hpp"

#include <cmath>

namespace
{
bool AdmittedUpper(
    const LadyLuck::IntentOptionalValue<double>& upper_mps,
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
void ApplyDbfmDefenseSpeed(
    const ControlIntent& command,
    const DbfmCornerSpeedControlEvidence& evidence,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    command.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }
    if (!std::isfinite(evidence.instantaneous_upper_mps.value)
        || !std::isfinite(evidence.sustained_upper_mps.value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    const IntentOptionalValue<double>* selected_upper = nullptr;
    bool selected_admitted = false;
    if (command.behavior_id
            == DoctrineBehaviorId::GunDefenseHorizontalBreak
        || command.behavior_id == DoctrineBehaviorId::DbfmBreak)
    {
        selected_upper = &evidence.instantaneous_upper_mps;
        selected_admitted = evidence.instantaneous_admitted;
    }
    else if (command.behavior_id == DoctrineBehaviorId::DbfmHardTurn)
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
        output = command;
        status = Status{};
        return;
    }

    ControlIntent candidate = command;
    candidate.desired_speed_mps = speed_reference_mps;
    candidate.desired_speed_rate_mps2 = 0.0;
    candidate.Validate(status);
    if (status.code != StatusCode::Ok)
    {
        output.Clear();
        return;
    }
    output = candidate;
}
}
