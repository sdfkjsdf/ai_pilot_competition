#include "LadyLuck/guidance/obfm/ObfmStationHoldService.hpp"

#include <cmath>

namespace
{

double Norm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

} // namespace

namespace LadyLuck
{

void ObfmStationHoldService::Evaluate(
    const ObfmStationHoldServiceInput& input,
    ObfmStationHoldServiceReceipt& output,
    Status& status) const noexcept
{
    output = ObfmStationHoldServiceReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(input.frame_identity))
    {
        output.status = ObfmStationHoldStatus::FrameIdentityInvalid;
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.reference.frame_identity = input.frame_identity;
    output.reference.evaluated = true;
    if (!input.station_hold_enabled)
    {
        output.status = ObfmStationHoldStatus::FeatureDisabled;
        return;
    }
    if (input.station_hold_owner_split_enabled)
    {
        output.status = ObfmStationHoldStatus::LagOwnerSplitEnabled;
        return;
    }

    const double range_m = input.range_m;
    const double minimum_m = input.official_min_range_m;
    const double maximum_m = input.official_max_range_m;
    const double target_speed_mps = Norm(input.target_velocity_ned_mps);
    output.target_speed_mps = std::isfinite(target_speed_mps)
        ? target_speed_mps
        : 0.0;
    if (!std::isfinite(range_m)
        || !std::isfinite(minimum_m)
        || !std::isfinite(maximum_m)
        || !std::isfinite(target_speed_mps))
    {
        output.status = ObfmStationHoldStatus::GeometryNonfinite;
        return;
    }
    if (maximum_m <= 0.0
        || minimum_m < 0.0
        || minimum_m >= maximum_m)
    {
        output.status = ObfmStationHoldStatus::OfficialRangeContractInvalid;
        return;
    }
    if (target_speed_mps <= 0.0)
    {
        output.status = ObfmStationHoldStatus::TargetSpeedNotPositive;
        return;
    }
    if (range_m <= 0.0 || range_m >= maximum_m)
    {
        output.status = ObfmStationHoldStatus::OutsideOfficialBand;
        return;
    }

    const double station_range_m = 0.5 * (minimum_m + maximum_m);
    double desired_speed_mps = target_speed_mps
        * (1.0 + (range_m - station_range_m) / maximum_m);
    output.station_range_m = station_range_m;
    if (input.stall_speed_1g_mps.has_value)
    {
        const double floor_mps = input.stall_speed_1g_mps.value;
        if (std::isfinite(floor_mps)
            && floor_mps > 0.0
            && desired_speed_mps < floor_mps)
        {
            desired_speed_mps = floor_mps;
        }
    }
    if (!std::isfinite(desired_speed_mps))
    {
        output.status = ObfmStationHoldStatus::GeometryNonfinite;
        return;
    }
    if (desired_speed_mps <= 0.0)
    {
        output.status = ObfmStationHoldStatus::ResultNotPositive;
        return;
    }

    output.reference.desired_speed_mps.has_value = true;
    output.reference.desired_speed_mps.value = desired_speed_mps;
    output.status = ObfmStationHoldStatus::SpeedAvailable;
}

} // namespace LadyLuck
