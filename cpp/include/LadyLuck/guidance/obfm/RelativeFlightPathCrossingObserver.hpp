#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class RelativeFlightPathCrossingReason : std::uint8_t
{
    FirstSampleNoHistory = 0U,
    TrackLineageMismatch = 1U,
    SourceEpochMismatch = 2U,
    SampleIndexNotExactlyConsecutive = 3U,
    SampleTimeNotStrictlyIncreasing = 4U,
    DerivedLateralIntervalNotFinite = 5U,
    LateralSideUnresolvedWithinErrorBounds = 6U,
    FrozenCourseAxesDisagree = 7U,
    NoRelativeFlightPathCrossing = 8U,
    RelativeFlightPathCrossingResolved = 9U,
    HorizontalCourseUnobservableWithinBounds = 10U,
    CrossingSampleContractRejected = 11U
};

const char* RelativeFlightPathCrossingReasonLabel(
    RelativeFlightPathCrossingReason reason) noexcept;

struct RelativeFlightPathSignedLateralInterval
{
    bool valid = false;
    double nominal_m = 0.0;
    double lower_m = 0.0;
    double upper_m = 0.0;
    std::int32_t resolved_sign = 0;
};

// Command-neutral d90 receipt. A resolved event may be appended to the
// next-tick G3-S crossing history, but this observer never owns guidance,
// flight-control, or production command authority.
struct RelativeFlightPathCrossingReceipt
{
    bool evaluated = false;
    RelativeFlightPathCrossingReason reason =
        RelativeFlightPathCrossingReason::FirstSampleNoHistory;

    bool previous_sample_present = false;
    std::int32_t previous_own_plane_id = -1;
    std::int32_t previous_target_plane_id = -1;
    std::uint64_t previous_episode_epoch = 0U;
    std::uint64_t previous_sample_index = 0U;
    double previous_t_sec = 0.0;

    std::int32_t current_own_plane_id = -1;
    std::int32_t current_target_plane_id = -1;
    std::uint64_t current_episode_epoch = 0U;
    std::uint64_t current_sample_index = 0U;
    double current_t_sec = 0.0;

    bool sample_dt_valid = false;
    double sample_dt_s = 0.0;
    bool geometry_evaluable = false;
    bool endpoint_sides_resolved = false;
    bool dual_frozen_axis_crossing_resolved = false;
    RelativeFlightPathSignedLateralInterval
        previous_axis_previous_position{};
    RelativeFlightPathSignedLateralInterval
        previous_axis_current_position{};
    RelativeFlightPathSignedLateralInterval
        current_axis_previous_position{};
    RelativeFlightPathSignedLateralInterval
        current_axis_current_position{};

    bool crossing_event_valid = false;
    double crossing_event_t_sec = 0.0;

    bool overshoot_timing_authority = false;
    bool side_orientation_authority = false;
    bool tactical_command_authority = false;
    bool production_authority = false;
};

class RelativeFlightPathCrossingObserver final
{
public:
    RelativeFlightPathCrossingObserver() noexcept = default;

    // Public only so translation-unit numerical kernels remain ordinary
    // allocation-free functions. Observer history remains private.
    struct HorizontalFlightPathSample
    {
        std::int32_t own_plane_id = -1;
        std::int32_t target_plane_id = -1;
        std::uint64_t episode_epoch = 0U;
        std::uint64_t sample_index = 0U;
        double t_sec = 0.0;
        Vector3 relative_position_ned_m{};
        Vector3 own_horizontal_course_ned{};
        double relative_position_error_bound_m = 0.0;
        double own_course_error_bound_rad = 0.0;
        double own_course_normalization_error_bound_l1 = 0.0;
    };

    void Reset() noexcept;
    void Observe(
        const DogfightGeometryFrame& frame,
        RelativeFlightPathCrossingReceipt& output,
        Status& status) noexcept;

private:
    bool previous_sample_valid_ = false;
    HorizontalFlightPathSample previous_sample_{};
};

static_assert(
    std::is_trivially_copyable<RelativeFlightPathCrossingReceipt>::value,
    "relative-flight-path receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RelativeFlightPathCrossingObserver>::value,
    "relative-flight-path observer must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
