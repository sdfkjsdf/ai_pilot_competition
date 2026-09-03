#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"

namespace LadyLuck
{
struct DbfmCornerSpeedControlEvidence
{
    IntentOptionalValue<double> instantaneous_upper_mps{};
    bool instantaneous_admitted = false;
    IntentOptionalValue<double> sustained_upper_mps{};
    bool sustained_admitted = false;
};

void ApplyDbfmDefenseSpeed(
    const ControlIntent& command,
    const DbfmCornerSpeedControlEvidence& evidence,
    ControlIntent& output,
    Status& status) noexcept;
}

