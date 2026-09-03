#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

constexpr std::size_t G16HighObservationHistoryCapacity = 8192U;

enum class G16EstablishedTurnReason : std::uint8_t
{
    Reset = 0U,
    RecentOfficialChordUnavailable = 1U,
    OlderOfficialChordUnavailable = 2U,
    TurnRateIntervalsDisjoint = 3U,
    TurnPlaneConesDisjoint = 4U,
    CircleUnobservable = 5U,
    TwoOfficialChordsConsistent = 6U,
    HistoryCapacityExceeded = 7U
};

enum class G16VerticalPhase : std::uint8_t
{
    Climbing = 0U,
    Descending = 1U,
    UnresolvedZeroInterval = 2U
};

enum class G16ApexReason : std::uint8_t
{
    Reset = 0U,
    FirstSampleNoHistory = 1U,
    ResolvedClimbArmed = 2U,
    ZeroIntervalRetainsClimbArm = 3U,
    ZeroIntervalWithoutPriorResolvedClimb = 4U,
    ResolvedClimbToDescentApexCrossing = 5U,
    ResolvedDescentWithoutPriorResolvedClimb = 6U
};

enum class G16HighRollReason : std::uint8_t
{
    Reset = 0U,
    AttitudePlaneNotObservable = 1U,
    DefenderDirectionNotObservable = 2U,
    ActualBankTowardDefenderInResolvedDescent = 3U,
    TowardDefenderRollOrDescentNotComplete = 4U,
    RollDirectionOrVerticalPhaseUnresolved = 5U
};

struct G16TurnChordReceipt
{
    bool valid = false;
    double duration_s = 0.0;
    double rotation_rad = 0.0;
    double endpoint_direction_error_bound_rad = 0.0;
    Vector3 plane_normal_ned{};
    double mean_turn_rate_radps = 0.0;
    double turn_rate_lower_radps = 0.0;
    double turn_rate_upper_radps = 0.0;
    double plane_axis_error_bound_rad = 0.0;
};

// Command-neutral proof that the defender has maintained two compatible
// official-angle path-turn chords.  This is not a High command and is not
// evidence of the aircraft response to a High command.
struct G16EstablishedTurnCircleReceipt
{
    bool evaluated = false;
    bool admitted = false;
    G16EstablishedTurnReason reason = G16EstablishedTurnReason::Reset;
    G16TurnChordReceipt older_chord{};
    G16TurnChordReceipt recent_chord{};
    Vector3 plane_normal_ned{};
    Vector3 centre_direction_ned{};
    Vector3 circle_centre_ned_m{};
    double radius_m = 0.0;
    double target_speed_mps = 0.0;
    double target_speed_error_bound_mps = 0.0;
    double normal_turn_rate_radps = 0.0;
    double observer_rate_resolution_radps = 0.0;
};

struct G16ApexObservationReceipt
{
    bool evaluated = false;
    bool apex_crossed = false;
    bool climb_armed = false;
    G16VerticalPhase vertical_phase =
        G16VerticalPhase::UnresolvedZeroInterval;
    G16ApexReason reason = G16ApexReason::Reset;
    double vertical_velocity_down_mps = 0.0;
    double vertical_velocity_lower_mps = 0.0;
    double vertical_velocity_upper_mps = 0.0;
};

struct G16HighRollObservationReceipt
{
    bool evaluated = false;
    G16HighRollReason reason = G16HighRollReason::Reset;
    bool turning_toward_defender_resolved = false;
    bool turning_toward_defender = false;
    bool descending_resolved = false;
    bool descending = false;
    bool high_roll_in_complete_resolved = false;
    bool high_roll_in_complete = false;
    Vector3 flight_direction_ned{};
    Vector3 maneuver_plane_normal_ned{};
    Vector3 lift_axis_ned{};
    double flight_direction_error_bound_rad = 0.0;
    double maneuver_plane_error_bound_rad = 0.0;
    double lift_axis_error_bound_rad = 0.0;
    double defender_direction_error_bound_rad = 0.0;
    double nominal_alignment_angle_rad = 0.0;
    double alignment_angle_lower_rad = 0.0;
    double alignment_angle_upper_rad = 0.0;
};

struct G16HighObservationReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool source_simultaneous = false;
    bool identity_restarted = false;
    G16EstablishedTurnCircleReceipt established_turn{};
    G16ApexObservationReceipt apex{};
    G16HighRollObservationReceipt roll_in{};
};

// Allocation-free observer bundle used by the visible G16-P High Service.
// One accepted source identity mutates the bundle at most once; duplicate
// calls return the cached receipt.  A lineage, index, or time discontinuity
// resets both the target-turn and apex histories before seeding the current
// sample, matching the production Python wrapper rather than the standalone
// apex leaf's looser gap policy.
class G16HighObservation final
{
public:
    G16HighObservation() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const runtime::TacticalCommandBuildInput& input,
        G16HighObservationReceipt& output,
        Status& status) noexcept;

private:
    struct TurnSupportSample
    {
        double time_s = 0.0;
        Vector3 direction_ned{};
        double body_direction_error_bound_rad = 0.0;
    };

    void ResetPhysicalHistory() noexcept;
    void AppendTurnSample(
        const TurnSupportSample& sample,
        bool& appended) noexcept;
    void BuildSupportChord(
        std::size_t end_offset,
        std::size_t& anchor_offset,
        G16TurnChordReceipt& output) const noexcept;
    void ObserveEstablishedTurn(
        const runtime::TacticalCommandBuildInput& input,
        double body_speed_error_bound_mps,
        double body_direction_error_bound_rad,
        G16EstablishedTurnCircleReceipt& output,
        Status& status) noexcept;
    void ObserveApex(
        const runtime::TacticalCommandBuildInput& input,
        double own_world_velocity_error_bound_mps,
        G16ApexObservationReceipt& output,
        Status& status) noexcept;
    void ObserveRollIn(
        const runtime::TacticalCommandBuildInput& input,
        const G16ApexObservationReceipt& apex,
        G16HighRollObservationReceipt& output,
        Status& status) const noexcept;

    std::array<TurnSupportSample, G16HighObservationHistoryCapacity>
        turn_history_{};
    std::size_t turn_history_head_ = 0U;
    std::size_t turn_history_count_ = 0U;
    double turn_observer_time_s_ = 0.0;
    bool apex_previous_sample_valid_ = false;
    bool apex_climb_armed_ = false;
    bool cached_identity_valid_ = false;
    ControlFrameIdentity cached_identity_{};
    std::int32_t cached_own_plane_id_ = -1;
    std::int32_t cached_target_plane_id_ = -1;
    G16HighObservationReceipt cached_receipt_{};
    StatusCode cached_status_code_ = StatusCode::Seeded;
};

static_assert(
    std::is_trivially_copyable<G16TurnChordReceipt>::value,
    "G16 High chord receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16EstablishedTurnCircleReceipt>::value,
    "G16 High circle receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16ApexObservationReceipt>::value,
    "G16 apex receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16HighRollObservationReceipt>::value,
    "G16 High roll receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16HighObservationReceipt>::value,
    "G16 High observation receipt must remain allocation-free.");

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
