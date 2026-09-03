#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
enum class DbfmHardTurnGeometryUnavailability : std::uint8_t
{
    NotObserved = 0U,
    None = 1U,
    OwnNoseHorizontalUnobservable = 2U,
    AttackerLosHorizontalUnobservable = 3U
};

struct DbfmHardTurnGeometryReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool command_geometry_available = false;
    DbfmHardTurnGeometryUnavailability unavailability =
        DbfmHardTurnGeometryUnavailability::NotObserved;
};

// Allocation-free numerical reference shared by the production ControlIntent
// materializer and the legacy TacticalCommand parity adapter.  This is raw
// aim/speed guidance only; downstream control owns p/q/r, Nz, surfaces and
// thrust.
struct DbfmHardTurnReference
{
    Vector3 aim_point_m{};
    double desired_speed_mps = 0.0;
    double capture_range_des_m = 0.0;
};

void BuildDbfmHardTurnReference(
    const DogfightGeometryFrame& frame,
    DbfmHardTurnReference& output,
    Status& status) noexcept;

void ObserveDbfmHardTurnGeometry(
    const DogfightGeometryFrame& frame,
    DbfmHardTurnGeometryReceipt& output,
    Status& status) noexcept;

void BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame,
    ControlIntent& output,
    Status& status) noexcept;

void BuildDbfmHardTurnCommand(
    const DogfightGeometryFrame& frame,
    const DbfmHardTurnGeometryReceipt& geometry,
    ControlIntent& output,
    bool& command_ready,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<DbfmHardTurnGeometryReceipt>::value,
    "DBFM HARD_TURN geometry receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<DbfmHardTurnReference>::value,
    "DBFM HARD_TURN reference must remain allocation-free.");
}
