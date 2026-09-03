#pragma once

#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

enum class ObfmStationHoldStatus : std::uint8_t
{
    SpeedAvailable = 0U,
    FeatureDisabled = 1U,
    LagOwnerSplitEnabled = 2U,
    GeometryNonfinite = 3U,
    OfficialRangeContractInvalid = 4U,
    TargetSpeedNotPositive = 5U,
    OutsideOfficialBand = 6U,
    ResultNotPositive = 7U,
    FrameIdentityInvalid = 8U
};

// Exact inputs of d90 station_hold_lag_observation.  The optional stall floor
// is deliberately not an authority gate: Python applies it only when it is
// finite and positive, and otherwise retains the unfloored station law.
struct ObfmStationHoldServiceInput
{
    ControlFrameIdentity frame_identity{};
    bool station_hold_enabled = false;
    bool station_hold_owner_split_enabled = false;
    double range_m = 0.0;
    double official_min_range_m = 0.0;
    double official_max_range_m = 0.0;
    Vector3 target_velocity_ned_mps{};
    IntentOptionalValue<double> stall_speed_1g_mps{};
};

struct ObfmStationHoldServiceReceipt
{
    ObfmStationHoldStatus status =
        ObfmStationHoldStatus::FrameIdentityInvalid;
    ObfmLagStationHoldReference reference{};
    double station_range_m = 0.0;
    double target_speed_mps = 0.0;
};

// Stateless, allocation-free Service evaluator.  It publishes only the
// longitudinal observation consumed by the selected LAG Task; it is never a
// command writer and never changes the LAG aim point or lifecycle.
class ObfmStationHoldService final
{
public:
    void Evaluate(
        const ObfmStationHoldServiceInput& input,
        ObfmStationHoldServiceReceipt& output,
        Status& status) const noexcept;
};

static_assert(
    std::is_trivially_copyable<ObfmStationHoldServiceInput>::value,
    "OBFM station-hold input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmStationHoldServiceReceipt>::value,
    "OBFM station-hold receipt must stay allocation-free.");

} // namespace LadyLuck
