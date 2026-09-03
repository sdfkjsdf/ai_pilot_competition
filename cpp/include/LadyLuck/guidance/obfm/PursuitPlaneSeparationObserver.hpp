#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Command-neutral path-plane evidence.  This observer does not request bank,
// load, body rate, surface, or thrust; it only proves whether two measured
// velocity histories resolve distinct maneuver planes.
enum class PursuitPathPlaneGate : std::uint8_t
{
    Unavailable = 0U,
    TwoTurnChordsNotInitialized = 1U,
    PathSampleTimeLineageDiscontinuous = 2U,
    PathSampleNotObservable = 3U,
    TwoTurnChordsNotResolvedOutsideDirectionError = 4U,
    ConsecutiveTurnPlaneConesDisjoint = 5U,
    PathPlaneLiftAxisNotObservable = 6U,
    TwoIntervalPathPlaneEstablished = 7U
};

enum class RollingScissorsPlaneRelation : std::uint8_t
{
    NotObservable = 0U,
    WithinObservationResolution = 1U,
    ResolvablySeparated = 2U
};

// Reduced production receipt consumed by rolling-scissors admission.  The
// optional Python scalars use finite backing plus separation_valid; no NaN is
// used as an availability sentinel.
struct RollingScissorsPlaneSeparationReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    PursuitPathPlaneGate own_path_gate =
        PursuitPathPlaneGate::Unavailable;
    PursuitPathPlaneGate target_path_gate =
        PursuitPathPlaneGate::Unavailable;
    bool own_path_plane_valid = false;
    bool target_path_plane_valid = false;
    RollingScissorsPlaneRelation relation =
        RollingScissorsPlaneRelation::NotObservable;
    bool separation_valid = false;
    double plane_separation_rad = 0.0;
    double plane_separation_resolution_bound_rad = 0.0;
};

// Exact reduced port of add/main d90
// guidance/behavior_tree/obfm_pursuit_state.py's two causal
// CausalPathPlaneObserver instances and _principal_plane_separation().
class PursuitPlaneSeparationObserver final
{
public:
    PursuitPlaneSeparationObserver() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const DogfightGeometryFrame& frame,
        double sample_dt_s,
        RollingScissorsPlaneSeparationReceipt& output,
        Status& status) noexcept;

private:
    std::array<double, 3U> own_sample_times_s_{};
    std::array<Vector3, 3U> own_sample_directions_ned_{};
    std::array<double, 3U> own_sample_direction_bounds_rad_{};
    std::size_t own_sample_count_ = 0U;
    std::array<double, 3U> target_sample_times_s_{};
    std::array<Vector3, 3U> target_sample_directions_ned_{};
    std::array<double, 3U> target_sample_direction_bounds_rad_{};
    std::size_t target_sample_count_ = 0U;
};

static_assert(
    std::is_trivially_copyable<
        RollingScissorsPlaneSeparationReceipt>::value,
    "Rolling-scissors plane receipt must remain allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
