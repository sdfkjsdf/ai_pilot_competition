#pragma once

#include "LadyLuck/control/direct_ned/DirectNedLoadVector.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{

// Numeric longitudinal fields retained by a direct-NED tactical materializer.
// The caller resolves tactical defaults, G17 speed floors, and GCAS handoff
// ownership before constructing this reference.
struct DirectNedLongitudinalReference
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double flight_path_angle_cmd_rad = 0.0;
    double specific_energy_rate_bias_m2ps3 = 0.0;
    bool integrator_hold = false;
};

// Stateless d90 direct-NED backend boundary.  It applies the current physical
// body-rate envelope and command-envelope upper load limit, then
// restores the q/Nz kinematic pair before the later CIS full Nz clip and
// surface/thrust inversion.
class DirectNedReferenceShaper final
{
public:
    DirectNedReferenceShaper() noexcept = default;

    void Shape(
        const DirectNedLoadVectorOutput& raw_reference,
        const DirectNedLoadVectorState& state,
        const route5::CommandEnvelope& envelope,
        const DirectNedLongitudinalReference& longitudinal,
        bool loaded_roll_enabled,
        tecs_cis::BodyRateLoadEnergyCommand& output,
        Status& status) const noexcept;
};

} // namespace direct_ned
} // namespace control
} // namespace LadyLuck
