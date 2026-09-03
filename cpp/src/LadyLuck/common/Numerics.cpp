#include "LadyLuck/common/Numerics.hpp"

namespace LadyLuck
{
namespace numerics
{

const double CisPairLongitudinalSpeedRegularizationMps = 1.0e-6;
const double CisPairCosineRegularization = 1.0e-6;

double RegularizedSignedInverse(
    const double value,
    const double resolution) noexcept
{
    return value / (value * value + resolution * resolution);
}

} // namespace numerics
} // namespace LadyLuck
