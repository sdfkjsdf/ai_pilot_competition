#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

// The two Python production gates are represented without carrying dynamic
// provenance strings through the 60 Hz path.  Only {false,false} (ordinary
// disabled) and {true,true} (the frozen 45abc production contract) are valid.
struct DbfmAltitudeSeparatedActivation
{
    bool enabled = false;
    bool exact_vertical_threat_provenance = false;
};

constexpr DbfmAltitudeSeparatedActivation
    DbfmAltitudeSeparatedProductionActivation{true, true};

enum class DbfmAltitudeSeparatedReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    ProductionDisabled = 2U,
    SustainedCornerUnavailable = 3U,
    FrameStateUnavailable = 4U,
    NotSection4Situation = 5U,
    HorizontalAwayUndefined = 6U,
    ReferenceRangeUnavailable = 7U,
    CaptureRangeUnavailable = 8U,
    OwnSpeedUnavailable = 9U,
    ArithmeticUnavailable = 10U,
    Selected = 11U
};

// Command-neutral copy of dbfm_vertical_threat.VerticalThreatSample.  These
// fields are kinematic/energy evidence only; they are not body-rate, Nz,
// actuator, thrust, estimator, or aircraft-response receipts.
struct DbfmVerticalThreatObservation
{
    bool evaluated = false;
    bool admitted = false;
    bool enemy_above_proven = false;
    bool energy_standing_holds = false;
    bool unfaceable = false;
    bool altitude_gap_available = false;
    double altitude_gap_m = 0.0;
    bool climb_budget_available = false;
    double climb_budget_m = 0.0;
};

// Raw guidance candidate for the ALTITUDE_SEPARATED leaf.  It requests an
// away-from-attacker horizontal path while holding current speed.  Escape
// energy shaping, flight-control allocation, and actual tracking remain
// downstream-owned.
struct DbfmAltitudeSeparatedReceipt
{
    ControlFrameIdentity frame_identity{};
    DbfmAltitudeSeparatedReason reason =
        DbfmAltitudeSeparatedReason::NotEvaluated;
    DbfmVerticalThreatObservation vertical_threat{};
    bool candidate_available = false;
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double capture_range_des_m = 0.0;
};

void EvaluateDbfmAltitudeSeparated(
    const DogfightGeometryFrame& frame,
    bool dbfm_owner_selected,
    const DbfmAltitudeSeparatedActivation& activation,
    const em::MergeCornerInterval& sustained_corner,
    DbfmAltitudeSeparatedReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<DbfmAltitudeSeparatedReceipt>::value,
    "DBFM altitude-separated receipt must remain allocation-free.");

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
