#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

struct DbfmEscapeEnergyActivation
{
    bool enabled = false;
    bool exact_escape_energy_provenance = false;
};

constexpr DbfmEscapeEnergyActivation DbfmEscapeEnergyProductionActivation{
    true,
    true};

// Exact Python overlay label domain. None is the canonical ordinary non-owner
// value; any future/out-of-domain value is also an ordinary passthrough.
enum class DbfmEscapeEnergyBehavior : std::uint8_t
{
    None = 0U,
    Escape = 1U,
    AltitudeSeparated = 2U
};

enum class DbfmEscapeEnergyReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    ProductionDisabled = 2U,
    BaseReferenceUnavailable = 3U,
    StateUnavailable = 4U,
    HorizontalAimUndefined = 5U,
    AppliedLevelGcasUnavailable = 6U,
    AppliedLevelCapabilityUnavailable = 7U,
    AppliedLevelRecoveryLimited = 8U,
    AppliedDive = 9U
};

// Minimal raw-guidance input copied from the selected TacticalCommand before
// this overlay.  No direct p/q/r/Nz, load-vector, surface, or thrust field is
// present, so the overlay cannot acquire flight-control authority.
struct DbfmEscapeEnergyBaseReference
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
};

// When candidate_available is false the caller preserves its already-valid
// base command.  A true candidate replaces only aim point, desired speed, and
// desired speed rate, exactly like Python dataclasses.replace().
struct DbfmEscapeEnergyReceipt
{
    ControlFrameIdentity frame_identity{};
    DbfmEscapeEnergyReason reason = DbfmEscapeEnergyReason::NotEvaluated;
    bool candidate_available = false;
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    bool dive_admitted = false;
    double dive_depression_rad = 0.0;
};

// Allocation-free C++ counterpart of Python
// maximum_power_speed_reference_mps().  The returned EmValue is absent for a
// non-finite/unusable altitude and otherwise carries the NZFEAS table-domain
// Mach edge converted by the frozen ISA model.  It owns speed guidance only.
em::EmValue DbfmMaximumPowerSpeedReferenceMps(double altitude_m) noexcept;

void ApplyDbfmEscapeEnergy(
    const DogfightGeometryFrame& frame,
    DbfmEscapeEnergyBehavior behavior,
    const DbfmEscapeEnergyActivation& activation,
    const DbfmEscapeEnergyBaseReference& base_reference,
    const em::EnergyManeuverCapability& capability,
    const safety::AutoGcasEntryReceipt& current_auto_gcas_entry,
    DbfmEscapeEnergyReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<DbfmEscapeEnergyBaseReference>::value,
    "DBFM escape-energy base reference must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<DbfmEscapeEnergyReceipt>::value,
    "DBFM escape-energy receipt must remain allocation-free.");

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
