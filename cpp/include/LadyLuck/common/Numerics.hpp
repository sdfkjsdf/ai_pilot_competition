#pragma once

namespace LadyLuck
{
namespace numerics
{

// add/main d90e929b: dimensioned resolution used by the paired
// longitudinal-speed CIS mappings.  This is numerical regularization, not a
// flight-control gain or a minimum commanded airspeed.
extern const double CisPairLongitudinalSpeedRegularizationMps;
extern const double CisPairCosineRegularization;

double RegularizedSignedInverse(
    double value,
    double resolution) noexcept;

} // namespace numerics
} // namespace LadyLuck
