#include "LadyLuck/plant/dynamics/MassBalance.hpp"

#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace dynamics
{

PlantResult<double> ComputeLoadedMassKg(
    const std::array<double, 4>& tanks_lb) noexcept
{
    PlantResult<double> result;
    double total_lb = 17400.0 + 230.0;
    for (const double tank_lb : tanks_lb)
    {
        if (!std::isfinite(tank_lb) || tank_lb < 0.0)
        {
            result.status = PlantStatus::Failure(
                PlantStatusCode::InvalidArgument,
                "tank quantity must be finite and nonnegative");
            return result;
        }
        total_lb += tank_lb;
    }
    result.value = total_lb * 0.45359237;
    result.status = PlantStatus::Success();
    return result;
}

} // namespace dynamics
} // namespace plant
} // namespace LadyLuck
