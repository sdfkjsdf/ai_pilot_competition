#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

#include <type_traits>

namespace LadyLuck
{
namespace control
{
namespace route5
{

// Exact, allocation-free front end of d90 NMuFilter.step_acceleration_ned.
// The stateful lateral LPF, continuous-mu unwrap, and LoadVector2Cis servo stay
// owned by Route5Guidance so AimPoint and direct-load-vector commands share one
// causal controller history, as they do in Python.
struct DirectLoadVectorNMuInput
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double course_rad = 0.0;
    double flight_path_angle_rad = 0.0;
    Vector3 acceleration_wind_g{};
    Vector3 gravity_wind_g{};
    Vector3 specific_force_wind_g{};
};

static_assert(
    std::is_trivially_copyable<DirectLoadVectorNMuInput>::value,
    "DirectLoadVectorNMuInput must remain allocation-free");

class DirectLoadVectorNMuAdapter final
{
public:
    DirectLoadVectorNMuAdapter() noexcept = default;

    void Prepare(
        const ControlIntent& command,
        const Vector3& velocity_ned_mps,
        DirectLoadVectorNMuInput& output,
        Status& status) const noexcept;
};

} // namespace route5
} // namespace control
} // namespace LadyLuck
