#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Exact offline evaluation authority. These values are not tuning knobs.
constexpr double ObfmNearStationCarrotLookaheadM = 400.0;
constexpr double ObfmNearStationCarrotBlendFarM = 500.0;
constexpr double ObfmNearStationCarrotBlendNearM = 150.0;
constexpr double ObfmNearStationCarrotTrackNormMinimumMps = 1.0e-6;

// This shaper operates on a selected virtual AimPoint. It must not be used as
// physical opponent LOS, threat/weapon evidence, or terminal direct control.
enum class ObfmNearStationCarrotReason : std::uint8_t
{
    NotOwner = 0,
    BaseIntentInvalid = 1,
    RouteNotAimPoint = 2,
    OwnHorizontalPositionUnavailable = 3,
    TargetTrackUnavailable = 4,
    TargetTrackNonFinite = 5,
    TargetHorizontalTrackUndefined = 6,
    StationGeometryUnavailable = 7,
    InactiveAtOrBeyondFarBand = 8,
    ArithmeticUnavailable = 9,
    Applied = 10
};

struct ObfmNearStationCarrotInput
{
    bool owner_selected = false;
    bool target_track_velocity_available = false;
    Vector3 own_position_ned_m{};
    Vector3 target_track_velocity_ned_mps{};
};

struct ObfmNearStationCarrotReceipt
{
    bool evaluated = false;
    bool base_validated = false;
    bool applied = false;
    bool base_preserved = true;
    double horizontal_station_distance_m = 0.0;
    double normalized_near_station_blend = 0.0;
    double station_weight = 0.0;
    double horizontal_track_norm_mps = 0.0;
    ObfmNearStationCarrotReason reason =
        ObfmNearStationCarrotReason::NotOwner;
};

// Typed AimPoint shaping seam used only by the selected production Entry
// owner immediately before final intent validation.  Its focused standalone
// tests provide arithmetic/semantic evidence, not FCS or aircraft tracking.
//
// The base intent is copied byte-for-byte before any ownership or evidence
// checks. Once the base validates, every unavailable/degenerate optional-track
// path is a successful bounded hold of that base intent, never a negative
// status. Only the AimPoint north/east coordinates may be modified.
void ShapeObfmNearStationCarrotAimPoint(
    const ControlIntent& base_intent,
    const ObfmNearStationCarrotInput& input,
    ObfmNearStationCarrotReceipt& receipt,
    ControlIntent& output_intent,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<ObfmNearStationCarrotInput>::value,
    "Near-station carrot input must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmNearStationCarrotReceipt>::value,
    "Near-station carrot receipt must remain allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
