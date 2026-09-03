#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

struct GunAttackOptionalDouble
{
    bool has_value = false;
    double value = 0.0;
};

struct GunAttackOptionalBool
{
    bool has_value = false;
    bool value = false;
};

// Exact allocation-free representation of Python's
// GunAttackGeometryObservation. Optional values preserve Python None.
struct GunAttackGeometryObservation
{
    bool valid = false;
    double t_sec = 0.0;
    Vector3 los_rate_world_radps{};
    double los_rate_radps = 0.0;
    GunAttackOptionalDouble relative_maneuver_plane_angle_rad{};
    GunAttackOptionalDouble maneuver_plane_alignment_cosine{};
    GunAttackOptionalDouble maneuver_plane_separation_rad{};
    GunAttackOptionalDouble maneuver_plane_resolution_rad{};
    GunAttackOptionalDouble own_lift_direction_resolution_rad{};
    GunAttackOptionalDouble own_los_plane_incidence_rad{};
    GunAttackOptionalDouble opponent_los_plane_incidence_rad{};
    GunAttackOptionalDouble los_plane_resolution_rad{};
    bool maneuver_plane_observable = false;
    GunAttackOptionalBool same_maneuver_plane_within_resolution{};
    GunAttackOptionalBool attacker_in_tail_tracking_region{};
    double attacker_aim_error_rad = 0.0;
    double active_gun_cone_rad = 0.0;
    bool attacker_aim_inside_active_cone = false;
};

enum class GunAttackForm : std::uint8_t
{
    Indeterminate = 0U,
    Snapshot = 1U,
    Tracking = 2U
};

const char* GunAttackFormLabel(GunAttackForm form) noexcept;

enum class GunAttackFormReason : std::uint8_t
{
    ManeuverPlaneUnobservable = 0U,
    ManualManeuverPlanesResolvablyDiffer = 1U,
    ManualHighCrossingOrNonTailPass = 2U,
    TailTrackingRegionUnresolved = 3U,
    ManualFixedLosSamePlaneTailTracking = 4U,
    CausalAimContinuityNotYetObserved = 5U,
    ManualContinuousAimSamePlaneTailTracking = 6U,
    ManualMomentaryAimSolutionPassage = 7U,
    ManualContinuouslyRetainedOfficialAimSolution = 8U
};

const char* GunAttackFormReasonLabel(
    GunAttackFormReason reason) noexcept;

struct GunAttackFormObservation
{
    bool valid = false;
    GunAttackGeometryObservation geometry{};
    GunAttackForm attack_form = GunAttackForm::Indeterminate;
    GunAttackOptionalBool same_maneuver_plane_endpoint{};
    bool fixed_los_endpoint = false;
    bool continuous_aim_solution = false;
    bool momentary_aim_solution = false;
    GunAttackFormReason reason =
        GunAttackFormReason::ManeuverPlaneUnobservable;
};

// Names consumed by the already-ported prefire response selector.
using PrefireGunAttackForm = GunAttackForm;
using PrefireGunAttackFormObservation = GunAttackFormObservation;

struct GunAttackFormObserverConfig
{
    // false is Python None and keeps the retention override fully inert.
    bool retention_authority_radps_valid = false;
    double retention_authority_radps = 0.0;
};

void ObserveGunAttackGeometry(
    const DogfightGeometryFrame& frame,
    GunAttackGeometryObservation& output,
    Status& status) noexcept;

void ClassifyManualAttackFormEndpoints(
    const GunAttackGeometryObservation& geometry,
    GunAttackFormObservation& output,
    Status& status) noexcept;

class GunAttackFormObserver final
{
public:
    GunAttackFormObserver() noexcept;

    // Mirrors Python construction. A successful call replaces the optional
    // retention authority and resets only the observer's mutable history.
    void Configure(
        const GunAttackFormObserverConfig& config,
        Status& status) noexcept;
    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        GunAttackFormObservation& output,
        Status& status) noexcept;

private:
    bool retention_authority_radps_valid_ = false;
    double retention_authority_radps_ = 0.0;
    bool previous_t_sec_valid_ = false;
    double previous_t_sec_ = 0.0;
    bool previous_aim_inside_ = false;
    bool previous_same_plane_ = false;
    bool previous_tail_tracking_ = false;
    bool retention_first_t_valid_ = false;
    double retention_first_t_ = 0.0;
    bool retention_previous_t_valid_ = false;
    double retention_previous_t_ = 0.0;
};

static_assert(
    std::is_trivially_copyable<GunAttackGeometryObservation>::value,
    "gun-attack geometry receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<GunAttackFormObservation>::value,
    "gun-attack form receipt must remain allocation-free");

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
