#pragma once

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

// Exact production provenance enabled by add/main@45abc9f6.  Entry Setup
// itself is enabled directly in my_submission.py; only its longitudinal
// companion is generated from the immutable competition profile.
constexpr const char* kObfmEntryLongitudinalReferenceProvenance =
    "OBFM_ENTRY_LONGITUDINAL_REFERENCE_V1";

// These are BattleServer wire-resolution identities, not fitted guidance
// thresholds.  The path-rate observer requires the measured interval rotation
// rate to exceed twice its complete endpoint direction-error bound.
constexpr double kBattleServerRpyQuantumRad =
    1.7453292519943296e-5; // radians(0.001 degree)
constexpr double kBattleServerBodyVelocityQuantumMps = 3.048e-4;

enum class ObfmEntryWindowReason : std::uint8_t
{
    Reset = 0U,
    SelectorServiceNotReached = 1U,
    FrameEvidenceUnavailable = 2U,
    DtUnavailable = 3U,
    TargetWorldVelocityUnavailable = 4U,
    TargetBodyVelocityUnavailable = 5U,
    TargetPathRateInit = 6U,
    TargetPathRateUnderResolution = 7U,
    TargetPathRotationAxisUnavailable = 8U,
    TargetCircleUnavailable = 9U,
    OwnWorldVelocityUnavailable = 10U,
    OwnCourseProjectionUnavailable = 11U,
    EntryWindowGeometryUnavailable = 12U,
    EntryWindowNotAhead = 13U,
    EntryPointVelocityInit = 14U,
    EntryWindowAhead = 15U,
    WindowFrameOrientationDiscontinuous = 16U,
    NoForwardRelativeProgress = 17U,
    NoTangentCoordinateCrossing = 18U,
    ProjectedWindowPassage = 19U,
    TangentCrossingOutsideWindowSegment = 20U,
    FeatureDisabled = 21U,
    SpacingOwnerDependencyDisabled = 22U,
    SafetyEvidenceUnavailable = 23U,
    SafetyNotAdmitted = 24U,
    SpacingHandoffDeferredCurrentEnergy = 25U,
    EntrySetupCompletedProjectedPassage = 26U,
    EntrySetupCompletedOutsideSegmentDevelopment = 27U,
    EntrySetupSelected = 28U,
    EntrySetupEntered = 29U,
    EntrySetupContinued = 30U,
    EntryObservationLost = 31U,
    SpacingOwnerHandoff = 32U,
    OfficialEmployPreemption = 33U,
    EntrySetupTreePreempted = 34U,
    EntrySetupReleased = 35U,
    EntrySetupInactive = 36U,
    OwnerContractUnavailable = 37U,
    DeclaredReadyFrameIdentityInvalid = 38U,
    DeclaredReadyFrameNonfinite = 39U,
    ServiceReceiptContradiction = 40U,
    TaskLifecycleContradiction = 41U,
    TurnCircleNotEstablished = 42U
};

const char* ObfmEntryWindowReasonLabel(ObfmEntryWindowReason reason) noexcept;

// Command-neutral classification for the two official-angle path-turn chords
// required by REQ-OBFM-10.  These are evidence reasons, not guidance modes or
// observed aircraft-response classifications.
enum class ObfmEntryEstablishedTurnReason : std::uint8_t
{
    Reset = 0U,
    FrameEvidenceUnavailable = 1U,
    RecentOfficialChordUnavailable = 2U,
    OlderOfficialChordUnavailable = 3U,
    TurnRateIntervalsDisjoint = 4U,
    TurnPlaneConesDisjoint = 5U,
    CircleUnobservable = 6U,
    TwoOfficialChordsConsistent = 7U,
    HistoryCapacityExceeded = 8U,
    OfficialRuleUnavailable = 9U,
    DeclaredReadyFrameIdentityInvalid = 10U,
    DeclaredReadyFrameNonfinite = 11U
};

const char* ObfmEntryEstablishedTurnReasonLabel(
    ObfmEntryEstablishedTurnReason reason) noexcept;

struct ObfmEntryTargetTurnCircle
{
    Vector3 target_position_ned_m{};
    Vector3 target_velocity_ned_mps{};
    Vector3 target_omega_ned_rad_s{};
    Vector3 plane_normal_ned{};
    Vector3 centre_direction_ned{};
    Vector3 circle_centre_ned_m{};
    double radius_m = 0.0;
    double speed_mps = 0.0;
    double normal_turn_rate_rad_s = 0.0;
    double observer_rate_resolution_rad_s = 0.0;
};

struct ObfmEntryTurnChordReceipt
{
    bool valid = false;
    double duration_s = 0.0;
    double rotation_rad = 0.0;
    double endpoint_direction_error_bound_rad = 0.0;
    Vector3 plane_normal_ned{};
    double mean_turn_rate_rad_s = 0.0;
    double turn_rate_lower_rad_s = 0.0;
    double turn_rate_upper_rad_s = 0.0;
    double plane_axis_error_bound_rad = 0.0;
};

struct ObfmEntryEstablishedTurnReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    bool identity_restarted = false;
    ObfmEntryEstablishedTurnReason reason =
        ObfmEntryEstablishedTurnReason::Reset;
    ObfmEntryTurnChordReceipt older_chord{};
    ObfmEntryTurnChordReceipt recent_chord{};
    ObfmEntryTargetTurnCircle circle{};
};

// Allocation capacity only; the physical history is pruned by the official
// P1-to-P2 rule epoch and never by an invented dwell or fitted threshold.
constexpr std::size_t ObfmEntryEstablishedTurnHistoryCapacity = 8192U;

struct ObfmEntryWindowObservationInput;

// Global, command-neutral observer for the established target turn assumed by
// the Entry Window doctrine.  Owner halt deliberately does not reach this
// object.  Episode/target/frame discontinuity resets its physical history so
// no chord can bridge unrelated evidence.
class ObfmEntryEstablishedTurnObserver final
{
public:
    ObfmEntryEstablishedTurnObserver() noexcept;

    void Reset() noexcept;
    void Observe(
        const DogfightGeometryFrame& frame,
        const ObfmEntryWindowObservationInput& input,
        ObfmEntryEstablishedTurnReceipt& output,
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
        double official_support_angle_rad,
        std::size_t& anchor_offset,
        ObfmEntryTurnChordReceipt& output) const noexcept;

    std::array<
        TurnSupportSample,
        ObfmEntryEstablishedTurnHistoryCapacity> turn_history_{};
    std::size_t turn_history_head_ = 0U;
    std::size_t turn_history_count_ = 0U;
    double turn_observer_time_s_ = 0.0;
    bool cached_identity_valid_ = false;
    ControlFrameIdentity cached_identity_{};
    std::int32_t cached_own_plane_id_ = -1;
    std::int32_t cached_target_plane_id_ = -1;
    ObfmEntryEstablishedTurnReceipt cached_receipt_{};
    StatusCode cached_status_code_ = StatusCode::Seeded;
};

struct ObfmEntryWindowGeometry
{
    ObfmEntryTargetTurnCircle circle{};
    Vector3 own_position_ned_m{};
    Vector3 own_velocity_ned_mps{};
    Vector3 own_projected_course_ned{};
    Vector3 entry_radius_direction_ned{};
    Vector3 entry_point_ned_m{};
    Vector3 entry_vector_ned_m{};
    double along_course_distance_m = 0.0;
    double entry_distance_m = 0.0;
    bool ahead = false;
};

struct ObfmEntryWindowPassageSample
{
    bool available = false;
    Vector3 window_axis_ned{};
    Vector3 tangent_axis_ned{};
    Vector3 own_plane_projection_ned_m{};
    double plane_offset_m = 0.0;
    double signed_tangent_distance_m = 0.0;
    double radial_fraction = 0.0;
    Vector3 course_intersection_ned_m{};
    double course_intersection_distance_m = 0.0;
    bool course_intersection_ahead = false;
    bool course_intersection_on_segment = false;
};

struct ObfmEntryWindowPassageEvent
{
    bool evaluated = false;
    bool available = false;
    bool projected_passage = false;
    bool interpolation_available = false;
    double interpolation_fraction = 0.0;
    double radial_fraction_at_crossing = 0.0;
    double plane_offset_m_at_crossing = 0.0;
    Vector3 crossing_point_ned_m{};
    ObfmEntryWindowReason reason =
        ObfmEntryWindowReason::NoTangentCoordinateCrossing;
};

struct ObfmEntryWindowObservationInput
{
    // False is ordinary evidence unavailability.  If true, every numeric field
    // consumed from the immutable frame is still checked before arithmetic.
    bool frame_evidence_available = false;
    double dt_s = 0.0;
};

struct ObfmEntryWindowObservationReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    ObfmEntryWindowReason reason = ObfmEntryWindowReason::Reset;
    bool target_path_rate_valid = false;
    bool target_path_rate_feature_ready = false;
    double target_path_relative_rotation_rad = 0.0;
    double target_path_rate_rad_s = 0.0;
    double admission_rate_resolution_rad_s = 0.0;
    Vector3 target_path_omega_ned_rad_s{};
    bool geometry_available = false;
    ObfmEntryWindowGeometry geometry{};
    bool entry_point_velocity_available = false;
    Vector3 entry_point_velocity_ned_mps{};
    ObfmEntryWindowPassageSample passage_sample{};
    ObfmEntryWindowPassageEvent passage_event{};
};

// Re-evaluate the current target circle with the previous ownship state.  The
// resulting point retains target translation/turn/plane/radius changes while
// excluding current-ownship motion.  It is longitudinal evidence only and
// must never replace the current lateral Entry point or its BEM velocity.
bool BuildObfmEntryTargetOnlyTransportedPoint(
    const ObfmEntryTargetTurnCircle& current_circle,
    const Vector3& previous_own_position_ned_m,
    const Vector3& previous_own_velocity_ned_mps,
    Vector3& transported_entry_point_ned_m,
    ObfmEntryWindowReason& reason) noexcept;

enum class ObfmEntrySetupCompletionKind : std::uint8_t
{
    None = 0U,
    ProjectedSegmentPassage = 1U,
    OutsideSegmentDevelopmentEvent = 2U
};

struct ObfmEntrySetupServiceInput
{
    bool selector_service_reached = false;
    bool feature_enabled = false;
    bool spacing_owner_enabled = false;
    bool safety_evidence_available = false;
    bool safety_admitted = false;
    // The higher-priority SPACING preview ran but current energy authority did
    // not admit its first command.  An already-active Entry owner may retain
    // the still-valid current geometry and retry on the next sample.
    bool spacing_handoff_deferred_current_energy = false;
};

struct ObfmEntrySetupServiceReceipt
{
    ControlFrameIdentity frame_identity{};
    bool service_evaluated = false;
    bool enabled_result = false;
    bool selected_result = false;
    bool owner_was_active = false;
    bool completed_this_tick = false;
    ObfmEntrySetupCompletionKind completion_kind =
        ObfmEntrySetupCompletionKind::None;
    ObfmEntryWindowReason reason =
        ObfmEntryWindowReason::SelectorServiceNotReached;
};

struct ObfmEntrySetupTaskReceipt
{
    ControlFrameIdentity frame_identity{};
    bool task_active = false;
    bool producer_ready = false;
    std::uint32_t producer_count = 0U;
    ObfmEntryWindowReason reason =
        ObfmEntryWindowReason::EntrySetupInactive;
};

enum class ObfmEntrySetupHaltCause : std::uint8_t
{
    OtherPreemption = 0U,
    SpacingOwnerActive = 1U,
    CompletedThisTick = 2U,
    OfficialEmployActive = 3U
};

struct ObfmEntrySetupHaltReceipt
{
    bool valid = false;
    bool was_active = false;
    bool released_this_tick = false;
    bool append_abort_reason = false;
    // A successor may have already become the sole command writer before the
    // previous Stateful Task is halted.
    bool clear_command_only_if_still_owner = true;
    ObfmEntryWindowReason reason =
        ObfmEntryWindowReason::EntrySetupInactive;
};

// Standalone port of the current production observer and Entry Setup
// Service -> Condition -> Stateful Task lifecycle.  This stage publishes no
// guidance command and owns no fallback.  ResetOwnerBranch deliberately keeps
// the global target-path observer; ResetEpisode clears both.
class ObfmEntryWindowAdmission final
{
public:
    ObfmEntryWindowAdmission() noexcept;

    void ResetEpisode() noexcept;
    void ResetOwnerBranch() noexcept;

    void Observe(
        const DogfightGeometryFrame& frame,
        const ObfmEntryWindowObservationInput& input,
        ObfmEntryWindowObservationReceipt& output,
        Status& status) noexcept;
    // Canonical production path: consumes the single established-turn
    // observer receipt instead of maintaining a second one-tick target-turn
    // history inside Entry. RuntimeContext must evaluate that observer first.
    void ObserveFromEstablishedTurn(
        const DogfightGeometryFrame& frame,
        const ObfmEntryWindowObservationInput& input,
        const ObfmEntryEstablishedTurnReceipt& established_turn,
        ObfmEntryWindowObservationReceipt& output,
        Status& status) noexcept;

    void EvaluateService(
        const ObfmEntrySetupServiceInput& input,
        const ObfmEntryWindowObservationReceipt& observation,
        ObfmEntrySetupServiceReceipt& output,
        Status& status) noexcept;

    void EnterOwner(
        const ObfmEntrySetupServiceReceipt& service,
        Status& status) noexcept;

    void TickOwner(
        const ObfmEntrySetupServiceReceipt& service,
        const ObfmEntryWindowObservationReceipt& observation,
        ObfmEntrySetupTaskReceipt& output,
        Status& status) noexcept;

    void HaltOwner(
        ObfmEntrySetupHaltCause cause,
        ObfmEntrySetupHaltReceipt& output) noexcept;

    bool owner_active() const noexcept;
    ObfmEntryWindowReason pending_halt_reason() const noexcept;

private:
    void ResetObservation() noexcept;
    void ClearOwner() noexcept;

    bool previous_target_direction_available_ = false;
    Vector3 previous_target_direction_ned_{};
    bool previous_body_direction_resolution_available_ = false;
    double previous_body_direction_resolution_rad_ = 0.0;
    bool previous_entry_point_available_ = false;
    Vector3 previous_entry_point_ned_m_{};
    bool previous_passage_sample_available_ = false;
    ObfmEntryWindowPassageSample previous_passage_sample_{};

    bool owner_active_ = false;
    bool completed_this_tick_ = false;
    ObfmEntrySetupCompletionKind completion_kind_ =
        ObfmEntrySetupCompletionKind::None;
    ObfmEntryWindowReason pending_halt_reason_ =
        ObfmEntryWindowReason::EntrySetupTreePreempted;
};

static_assert(
    std::is_trivially_copyable<ObfmEntryWindowObservationReceipt>::value,
    "Entry observation receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntryTurnChordReceipt>::value,
    "Entry turn chord receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntryEstablishedTurnReceipt>::value,
    "Entry established-turn receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntrySetupServiceReceipt>::value,
    "Entry Service receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntrySetupTaskReceipt>::value,
    "Entry Task receipt must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
