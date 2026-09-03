#pragma once

#include <array>

namespace LadyLuck
{
namespace plant
{

struct PidState
{
    double integral = 0.0;
    double input_previous = 0.0;
    double input_previous2 = 0.0;
};

struct KinematicState
{
    double position = 0.0;
};

struct RollChannelState
{
    PidState pid{};
};

struct PitchChannelState
{
    PidState pid{};
    KinematicState elevator{};
};

struct YawChannelState
{
    PidState pid{};
};

struct FlaperonState
{
    KinematicState aileron{};
    KinematicState trailing_edge_flap{};
};

// This is a value-owned candidate state. It never crosses the DLL ABI.
struct AuxState
{
    double mass_kg = 0.0;
    double previous_force_z_n = 0.0;
    double previous_force_y_n = 0.0;
    RollChannelState roll{};
    PitchChannelState pitch{};
    YawChannelState yaw{};
    FlaperonState flaperon{};
    std::array<double, 4> tanks_lb{{3486.0, 3486.0, 0.0, 0.0}};
};

} // namespace plant
} // namespace LadyLuck
