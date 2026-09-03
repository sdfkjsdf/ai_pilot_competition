#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/guidance/dbfm/RollGV2.hpp"

namespace LadyLuck
{
struct DbfmBreakLoadKinematics
{
    Vector3 position_ned_m{};
    Vector3 velocity_body_mps{};
    // This independently supplied world-NED velocity owns the BREAK lateral
    // axis. The raw path law reconstructs its own NED velocity from body state.
    Vector3 velocity_world_ned_mps{};
    Vector3 rpy_rad{};
};

struct DbfmBreakLoadReference
{
    Vector3 raw_acceleration_ned_mps2{};
    Vector3 admitted_acceleration_ned_mps2{};
    double instantaneous_load_limit_g = 0.0;
    RollGV2Result roll_pull{};
};

struct DbfmBreakLoadEvidence
{
    bool capability_admitted = false;
    IntentOptionalValue<double> instantaneous_load_limit_g{};
};

struct DbfmBreakLoadConfig
{
    bool enabled = false;
    bool magnitude_tracking_enabled = false;
    bool loaded_roll_enabled = false;
};

// Allocation-free production counterparts. They apply the same d90 BREAK
// geometry to ControlIntent without constructing or copying the legacy
// string-backed TacticalCommand type.
void BuildDbfmBreakLoadReference(
    const ControlIntent& command,
    const DbfmBreakLoadKinematics& own,
    double instantaneous_load_limit_g,
    DbfmBreakLoadReference& output,
    Status& status) noexcept;

void ApplyDbfmBreakLoad(
    const ControlIntent& command,
    const DbfmBreakLoadKinematics& own,
    const DbfmBreakLoadEvidence& evidence,
    const DbfmBreakLoadConfig& config,
    ControlIntent& output,
    Status& status) noexcept;
}
