#include "LadyLuck/guidance/obfm/BarrelRollCounter.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/geometry/WezRule.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;

double PositiveInfinity() noexcept
{
    return (std::numeric_limits<double>::infinity)();
}

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

struct DoubleLength
{
    double hi = 0.0;
    double lo = 0.0;
};

DoubleLength DoubleLengthFastSum(
    const double a,
    const double b) noexcept
{
    const double sum = a + b;
    return DoubleLength{sum, (a - sum) + b};
}

DoubleLength DoubleLengthMultiply(
    const double left,
    const double right) noexcept
{
    const double product = left * right;
    return DoubleLength{product, std::fma(left, right, -product)};
}

double MathHypot3(const Vector3& value) noexcept
{
    Vector3 coordinates{{
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])}};
    const double maximum = (std::max)(
        coordinates[0],
        (std::max)(coordinates[1], coordinates[2]));
    if (std::isinf(maximum))
    {
        return maximum;
    }
    if (!FiniteVector(coordinates))
    {
        return (std::numeric_limits<double>::quiet_NaN)();
    }
    if (maximum == 0.0)
    {
        return maximum;
    }
    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    const double minimum_normal = (std::numeric_limits<double>::min)();
    if (maximum_exponent < -1023)
    {
        coordinates[0] /= minimum_normal;
        coordinates[1] /= minimum_normal;
        coordinates[2] /= minimum_normal;
        return minimum_normal * MathHypot3(coordinates);
    }
    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const DoubleLength product = DoubleLengthMultiply(scaled, scaled);
        const DoubleLength sum = DoubleLengthFastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const DoubleLength negative_square = DoubleLengthMultiply(-result, result);
    const DoubleLength corrected_sum = DoubleLengthFastSum(
        compensated_sum,
        negative_square.hi);
    compensated_sum = corrected_sum.hi;
    fraction_one += negative_square.lo;
    fraction_two += corrected_sum.lo;
    const double correction =
        compensated_sum - 1.0 + (fraction_one + fraction_two);
    result += correction / (2.0 * result);
    return result / scale;
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const float float_infinity =
        (std::numeric_limits<float>::infinity)();
    Vector3 residual{};
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float represented_f32 = static_cast<float>(value[index]);
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(
            std::nextafter(represented_f32, float_infinity));
        const double lower = static_cast<double>(
            std::nextafter(represented_f32, -float_infinity));
        if (!std::isfinite(represented)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return false;
        }
        residual[index] = value[index] - represented;
        full_cell[index] = (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }
    const double residual_norm = std::nextafter(
        MathHypot3(residual),
        PositiveInfinity());
    const double cell_norm = std::nextafter(
        MathHypot3(full_cell),
        PositiveInfinity());
    output = std::nextafter(
        residual_norm + cell_norm,
        PositiveInfinity());
    return std::isfinite(output);
}

bool OfficialOutermostRangeM(double& output) noexcept
{
    output = 0.0;
    for (std::size_t index = 0U;
         index < LadyLuck::OfficialWezPhaseCount;
         ++index)
    {
        const LadyLuck::Result<LadyLuck::WezPhase> phase =
            LadyLuck::OfficialWezPhaseAt(index);
        if (!phase.sample_valid()
            || !std::isfinite(phase.value.max_range_m)
            || phase.value.max_range_m <= 0.0)
        {
            return false;
        }
        output = (std::max)(output, phase.value.max_range_m);
    }
    return output > 0.0;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void BarrelRollCounterInEngagementBand(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    if (!FiniteVector(own_position) || !FiniteVector(adversary_position))
    {
        return;
    }
    double maximum_range_m = 0.0;
    if (!OfficialOutermostRangeM(maximum_range_m))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double horizontal_m = std::hypot(
        own_position[0] - adversary_position[0],
        own_position[1] - adversary_position[1]);
    output = horizontal_m <= maximum_range_m;
}

void BarrelRollCounterApexReached(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = true;
    status = Status{};
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_velocity) || !FiniteVector(adversary_velocity))
    {
        return;
    }
    double own_bound = 0.0;
    double adversary_bound = 0.0;
    if (!Float32WireVectorErrorBound(own_velocity, own_bound)
        || !Float32WireVectorErrorBound(
            adversary_velocity,
            adversary_bound))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    output = !(NumpyNorm3(own_velocity) - own_bound
        > NumpyNorm3(adversary_velocity) + adversary_bound);
}

void BarrelRollCounterWithinReach(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    if (observation == nullptr || !observation->valid)
    {
        return;
    }
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    if (!FiniteVector(own_position) || !FiniteVector(adversary_position))
    {
        return;
    }
    double own_bound = 0.0;
    double adversary_bound = 0.0;
    if (!Float32WireVectorErrorBound(own_position, own_bound)
        || !Float32WireVectorErrorBound(
            adversary_position,
            adversary_bound))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    const double own_altitude = -own_position[2];
    const double adversary_altitude = -adversary_position[2];
    const double envelope = (std::max)(
        0.0,
        observation->roll_envelope_m);
    output = own_altitude - own_bound
        <= adversary_altitude + adversary_bound + envelope;
}

void BuildBarrelRollCounterHoldOverlay(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    BarrelRollCounterHoldOverlay& output,
    Status& status) noexcept
{
    output = BarrelRollCounterHoldOverlay{};
    status = Status{};
    if (observation == nullptr
        || !observation->valid
        || observation->phase_count < 3U)
    {
        return;
    }
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity))
    {
        output.reason = BarrelRollCounterSelectionReason::StateNotFinite;
        return;
    }
    double adversary_velocity_bound = 0.0;
    if (!Float32WireVectorErrorBound(
            adversary_velocity,
            adversary_velocity_bound))
    {
        output.reason = BarrelRollCounterSelectionReason::ContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    const double adversary_speed = NumpyNorm3(adversary_velocity);
    if (!(adversary_speed - adversary_velocity_bound > 0.0))
    {
        output.reason =
            BarrelRollCounterSelectionReason::AdversarySpeedUnresolved;
        return;
    }
    const double own_altitude = -own_position[2];
    const double own_speed = NumpyNorm3(own_velocity);
    const double speed_square_difference =
        own_speed * own_speed - adversary_speed * adversary_speed;
    const double zoom_budget_m = (std::max)(
        0.0,
        speed_square_difference
            / (2.0 * constants::StandardGravityMps2));
    output.selected = true;
    output.reason = BarrelRollCounterSelectionReason::HoldSelected;
    output.aim_point_m = Vector3{{
        adversary_position[0],
        adversary_position[1],
        -(own_altitude + zoom_budget_m)}};
    output.desired_speed_mps = adversary_speed;
    output.desired_speed_rate_mps2 = 0.0;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
