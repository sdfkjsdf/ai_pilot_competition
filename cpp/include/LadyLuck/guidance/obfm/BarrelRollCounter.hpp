#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/obfm/RollDefenseObserver.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class BarrelRollCounterSelectionReason : std::uint8_t
{
    ObservationUnavailable = 0U,
    StateNotFinite = 1U,
    AdversarySpeedUnresolved = 2U,
    HoldSelected = 3U,
    ContractRejected = 4U
};

struct BarrelRollCounterHoldOverlay
{
    bool selected = false;
    BarrelRollCounterSelectionReason reason =
        BarrelRollCounterSelectionReason::ObservationUnavailable;
    Vector3 aim_point_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
};

// These are command-neutral Condition/Service receipts. The later selected BT
// Task alone owns applying the overlay to a ControlIntent.
void BarrelRollCounterInEngagementBand(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

void BarrelRollCounterApexReached(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept;

void BarrelRollCounterWithinReach(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    bool& output,
    Status& status) noexcept;

void BuildBarrelRollCounterHoldOverlay(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* observation,
    BarrelRollCounterHoldOverlay& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<BarrelRollCounterHoldOverlay>::value,
    "barrel-roll counter overlay must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
