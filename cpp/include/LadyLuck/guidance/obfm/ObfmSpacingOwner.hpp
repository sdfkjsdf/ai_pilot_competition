#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/obfm/ObfmLongitudinalAuthority.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

struct ObfmSpacingAtomicFixtureAccess;

// Exact add/main@da8b640e production Spacing lifecycle.  These are raw guidance
// phases only.  Route-5/CIS later converts the aim/speed/energy request into
// p/q/r, Nz, actuator and thrust commands; this owner makes no tracking claim.
enum class ObfmSpacingOwnerPhase : std::uint8_t
{
    Inactive = 0U,
    PathEnergyExchange = 1U,
    LevelRecovery = 2U,
    WezReacquire = 3U,
    PostHitRminArrest = 4U
};

enum class ObfmSpacingCompletionPhase : std::uint8_t
{
    Unprimed = 0U,
    Primed = 1U,
    ClosureArrestLatched = 2U,
    Completed = 3U,
    Failed = 4U
};

enum class ObfmSpacingRecoveryPhase : std::uint8_t
{
    Unprimed = 0U,
    Tracking = 1U,
    LevelRecovery = 2U,
    VerticalTurnaroundLatched = 3U,
    Completed = 4U,
    Failed = 5U
};

// The strict surface admits a new owner (and every post-hit arrest sample).
// Once Path/Level/WEZ is committed, only the fault-only running surface is
// consumed; ordinary limiter/saturation feedback must not abort the maneuver.
enum class ObfmSpacingSafetyGrade : std::uint8_t
{
    None = 0U,
    StrictEntry = 1U,
    RunningFaultOnly = 2U
};

// Ordinary finite nonselection/release reasons retain StatusCode::Ok,
// including the arrival-feasibility reasons appended below.  Only the named
// DeclaredReady*, TaskLifecycle, and ServiceReceipt reasons are contract
// contradictions.
enum class ObfmSpacingOwnerReason : std::uint8_t
{
    SelectorBranchNotReached = 0U,
    FeatureDisabled = 1U,
    FrameEvidenceUnavailable = 2U,
    SafetySampleUnavailable = 3U,
    SafetyNotAdmitted = 4U,
    EnergyAuthorityUnavailable = 5U,
    EnergyAuthorityDoesNotAdmitDissipation = 6U,
    NeutralEnergyAuthorityUnavailable = 7U,
    OfficialEmployAvailable = 8U,
    OfficialEpochPrimed = 9U,
    OfficialEpochChanged = 10U,
    TimeNotIncreasing = 11U,
    TargetHorizontalCourseUndefined = 12U,
    TargetCourseUndefined = 13U,
    OwnSpeedNotPositive = 14U,
    OfficialRangeInvalid = 15U,
    CoincidentGeometry = 16U,
    ArrestHorizontalReferenceUndefined = 17U,
    StationNotAhead = 18U,
    ClimbAllocationUnavailable = 19U,
    NoExcessProjectedClosure = 20U,
    CoordinateClosureNotExcess = 21U,
    DualClosureExcessLatched = 22U,
    LatchedActive = 23U,
    CurrentEnergyProjectionRequired = 24U,
    CurrentEnergyProjectionUnavailable = 25U,
    CurrentEnergyProjectionRejected = 26U,
    DecoratorNotReached = 27U,
    DecoratorNotAdmitted = 28U,
    DecoratorSelected = 29U,
    CommittedActive = 30U,
    PostHitPending = 31U,
    CompletionPrimed = 32U,
    AwaitingClosureArrest = 33U,
    ClosureArrestLatched = 34U,
    AwaitingStationTurnaround = 35U,
    ArrestConfirmed = 36U,
    StationPassedBeforeCompletion = 37U,
    RecoveryTracking = 38U,
    RecoveryActivated = 39U,
    AwaitingVerticalTurnaround = 40U,
    VerticalTurnaroundLatched = 41U,
    AwaitingRecoveryEndpoint = 42U,
    RecoveryCompleted = 43U,
    StationPassedDuringRecovery = 44U,
    OfficialEpochChangedDuringRecovery = 45U,
    WezRecoveryEvidenceUnavailable = 46U,
    WezEpochChanged = 47U,
    OfficialMinimumRangeReached = 48U,
    OfficialMidrangeStationPassed = 49U,
    ReacquireHorizontalCourseUndefined = 50U,
    ReacquireStationVelocityUndefined = 51U,
    TargetPathExceedsAuthority = 52U,
    BoresightGeometryUnavailable = 53U,
    BoresightTargetNotForward = 54U,
    CommandGeometryUnavailable = 55U,
    CommandFloat32DomainUnavailable = 56U,
    CommandReady = 57U,
    TaskCompleted = 58U,
    ReleaseToLowerFallback = 59U,
    TreePreempted = 60U,
    OfficialEmployPreemption = 61U,
    EpisodeOrTargetChanged = 62U,
    DeclaredReadyFrameIdentityInvalid = 63U,
    DeclaredReadyFrameNonfinite = 64U,
    DeclaredReadyEnergyAuthorityContradiction = 65U,
    DeclaredReadyProjectionContradiction = 66U,
    TaskLifecycleContradiction = 67U,
    ServiceReceiptContradiction = 68U,
    RecoveryVelocityBoundUnavailable = 69U,
    DeclaredReadySafetyReceiptContradiction = 70U,
    DeclaredReadyRecoveryVelocityBoundContradiction = 71U,
    DualClosureArrivalInfeasibleLatched = 72U,
    DualArrivalClosureNotPositive = 73U,
    ArrivalArrestDecelerationUnavailable = 74U,
    ArrivalArrestDistanceAvailable = 75U,
    ArrivalArrestArithmeticUnavailable = 76U
};

const char* ObfmSpacingOwnerReasonLabel(
    ObfmSpacingOwnerReason reason) noexcept;

// Focused binary64-domain gate used by the native production smoke.  It does
// not select a tactic or expose a tuning surface.
bool ObfmSpacingArithmeticBoundarySafeForTesting() noexcept;

struct ObfmSpacingGeometry
{
    bool valid = false;
    Vector3 target_axis_ned{};
    Vector3 station_error_horizontal_m{};
    Vector3 cross_track_error_horizontal_m{};
    Vector3 target_horizontal_velocity_ned_mps{};
    Vector3 arrest_horizontal_velocity_ned_mps{};
    double signed_station_spacing_m = 0.0;
    double projected_closure_mps = 0.0;
    double structural_station_closure_mps = 0.0;
    double target_horizontal_speed_mps = 0.0;
    double arrest_horizontal_speed_mps = 0.0;
    double own_speed_mps = 0.0;
    double official_range_m = 0.0;
};

// Shared 15a5e23 path allocation.  Entry feasibility and command
// materialization consume this one operation-ordered result.
struct ObfmSpacingArrestPathAllocation
{
    bool valid = false;
    double raw_gamma_rad = 0.0;
    double admitted_gamma_rad = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double target_axis_reference_deceleration_mps2 = 0.0;
};

struct ObfmSpacingArrivalFeasibilityReceipt
{
    bool evaluated = false;
    bool admitted = false;
    ObfmSpacingArrestPathAllocation path_allocation{};
    bool stopping_distances_available = false;
    double projected_stopping_distance_m = 0.0;
    double coordinate_stopping_distance_m = 0.0;
    ObfmSpacingOwnerReason reason =
        ObfmSpacingOwnerReason::ArrivalArrestDistanceAvailable;
};

struct ObfmSpacingReacquireGeometry
{
    bool valid = false;
    Vector3 target_course_ned{};
    Vector3 target_velocity_ned_mps{};
    Vector3 own_velocity_ned_mps{};
    Vector3 los_direction_ned{};
    Vector3 station_error_ned_m{};
    Vector3 station_velocity_ned_mps{};
    double signed_station_spacing_m = 0.0;
    double slant_range_m = 0.0;
    double target_speed_mps = 0.0;
    double station_horizontal_speed_mps = 0.0;
    double station_speed_mps = 0.0;
    double own_speed_mps = 0.0;
    double structural_rate_per_s = 0.0;
    double capture_range_m = 0.0;
    double official_min_range_m = 0.0;
    double official_max_range_m = 0.0;
};

// Raw TacticalCommand-equivalent guidance candidate.  It intentionally omits
// all body-rate/load/surface/thrust channels.
struct ObfmSpacingGuidanceCommand
{
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double specific_energy_rate_bias_m2ps3 = 0.0;
    bool path_inversion_allowed = false;
    double capture_range_des_m = 0.0;
};

// Same-frame CIS-v4 preview result supplied by the existing backend adapter.
// The backend is the sole authority for this projection.  Spacing verifies
// that only its explicit energy bias changed; it does not reproduce TECS.
struct ObfmSpacingCurrentEnergyProjection
{
    bool evaluated = false;
    bool admitted = false;
    ControlFrameIdentity frame_identity{};
    bool all_nonenergy_fields_unchanged = false;
    // raw_bias is diagnostic telemetry from the projection provider.  The
    // provider's projected command, mapped to admitted_bias below, is the
    // authority; consumers must not re-prove raw binary64 equality.
    double raw_bias_m2ps3 = 0.0;
    double admitted_bias_m2ps3 = 0.0;
};

// Single-producer, same-frame admission evidence.  An admitted bit without
// its evaluated bit, or evaluated evidence from another frame, is a declared
// contract contradiction.  A required grade that was not evaluated, or was
// evaluated but not admitted, is an ordinary lower-branch fallback.
struct ObfmSpacingSafetyReceipt
{
    ControlFrameIdentity frame_identity{};
    bool strict_entry_evaluated = false;
    bool strict_entry_admitted = false;
    bool running_fault_only_evaluated = false;
    bool running_fault_only_admitted = false;
};

// The upstream frame adapter owns the raw-world velocity uncertainty and the
// exact moving-frame tilt calculation.  Spacing only consumes their finite,
// nonnegative same-frame sum; it does not invent another threshold.
struct ObfmSpacingRecoveryVelocityBoundReceipt
{
    bool evaluated = false;
    ControlFrameIdentity frame_identity{};
    double own_down_velocity_error_bound_mps = 0.0;
};

struct ObfmSpacingOwnerServiceInput
{
    // False means a higher-priority selector sibling owns this tick.  No
    // Spacing evidence, geometry or state is evaluated in that case.
    bool selector_branch_reached = false;
    bool frame_evidence_declared_ready = false;
    bool feature_enabled = false;
    ObfmSpacingSafetyReceipt safety{};
    bool flight_path_gamma_limit_available = false;
    double flight_path_gamma_limit_rad = 0.0;
    ObfmEnergyRateAuthorityObservation previous_energy_authority{};
    bool current_energy_projection_required = false;
};

struct ObfmSpacingOwnerServiceReceipt
{
    ControlFrameIdentity frame_identity{};
    bool service_evaluated = false;
    bool feature_enabled = false;
    ObfmSpacingOwnerPhase safety_phase = ObfmSpacingOwnerPhase::Inactive;
    ObfmSpacingSafetyGrade safety_grade_required =
        ObfmSpacingSafetyGrade::None;
    ObfmSpacingSafetyReceipt safety{};
    bool safety_grade_evaluated = false;
    bool safety_grade_admitted = false;
    bool entry_latched = false;
    bool projection_required = false;
    bool selection_finalized = false;
    bool selected_result = false;
    std::uint32_t selected_count = 0U;
    ObfmSpacingGeometry geometry{};
    bool coordinate_closure_available = false;
    double coordinate_closure_mps = 0.0;
    ObfmSpacingArrivalFeasibilityReceipt arrival_feasibility{};
    ObfmSpacingOwnerReason entry_latch_reason =
        ObfmSpacingOwnerReason::SelectorBranchNotReached;
    bool preprojected_candidate_valid = false;
    ObfmSpacingGuidanceCommand preprojected_candidate{};
    bool current_projection_valid = false;
    // `valid` means the optional projection decision was resolved into a
    // finite command. `admitted` distinguishes a projected bias from the raw
    // finite-command fallback used on ordinary projection non-admission.
    bool current_projection_admitted = false;
    double current_projection_raw_bias_m2ps3 = 0.0;
    double current_projection_admitted_bias_m2ps3 = 0.0;
    ObfmSpacingOwnerReason reason =
        ObfmSpacingOwnerReason::SelectorBranchNotReached;
};

struct ObfmSpacingOwnerSelection
{
    ControlFrameIdentity frame_identity{};
    bool branch_reached = false;
    bool selected = false;
    std::uint32_t selection_count = 0U;
    ObfmSpacingOwnerReason reason =
        ObfmSpacingOwnerReason::DecoratorNotReached;
};

struct ObfmSpacingOwnerTaskInput
{
    bool flight_path_gamma_limit_available = false;
    double flight_path_gamma_limit_rad = 0.0;
    ObfmEnergyRateAuthorityObservation previous_energy_authority{};
    ObfmSpacingCurrentEnergyProjection current_energy_projection{};
    ObfmSpacingRecoveryVelocityBoundReceipt recovery_velocity_bound{};
};

struct ObfmSpacingOwnerTaskReceipt
{
    ControlFrameIdentity frame_identity{};
    bool task_active = false;
    ObfmSpacingOwnerPhase phase = ObfmSpacingOwnerPhase::Inactive;
    ObfmSpacingOwnerPhase safety_phase = ObfmSpacingOwnerPhase::Inactive;
    ObfmSpacingSafetyGrade safety_grade_consumed =
        ObfmSpacingSafetyGrade::None;
    ControlFrameIdentity safety_frame_identity{};
    bool safety_admitted = false;
    bool recovery_velocity_bound_consumed = false;
    ObfmSpacingRecoveryVelocityBoundReceipt recovery_velocity_bound{};
    ObfmSpacingCompletionPhase completion_phase =
        ObfmSpacingCompletionPhase::Unprimed;
    ObfmSpacingRecoveryPhase recovery_phase =
        ObfmSpacingRecoveryPhase::Unprimed;
    bool phase_changed = false;
    bool arrest_confirmed_this_tick = false;
    bool recovery_completed_this_tick = false;
    bool task_completed = false;
    bool release_required = false;
    // A projection-required PATH tick first returns this raw, typed preview
    // only when its caller deliberately omits the projection.  It is not a
    // command candidate and carries no selection/publication authority.
    bool projection_required = false;
    bool preprojected_candidate_valid = false;
    ObfmSpacingGuidanceCommand preprojected_candidate{};
    bool candidate_valid = false;
    std::uint32_t candidate_count = 0U;
    ObfmSpacingGuidanceCommand candidate{};
    ObfmSpacingOwnerReason reason =
        ObfmSpacingOwnerReason::CommandGeometryUnavailable;
};

struct ObfmSpacingOwnerHaltReceipt
{
    bool valid = false;
    bool was_active = false;
    bool official_employ_preemption = false;
    bool post_hit_pending = false;
    bool clear_command_only_if_still_owner = true;
};

class ObfmSpacingOwner final
{
public:
    ObfmSpacingOwner() noexcept;

    void ResetEpisode() noexcept;
    void ObserveService(
        const DogfightGeometryFrame& frame,
        const ObfmSpacingOwnerServiceInput& input,
        ObfmSpacingOwnerServiceReceipt& output,
        Status& status) noexcept;
    void FinalizeServiceProjection(
        const ObfmSpacingOwnerServiceReceipt& preliminary,
        const ObfmSpacingCurrentEnergyProjection& projection,
        ObfmSpacingOwnerServiceReceipt& output,
        Status& status) const noexcept;
    void EvaluateDecorator(
        bool branch_reached,
        const ObfmSpacingOwnerServiceReceipt& service,
        ObfmSpacingOwnerSelection& output,
        Status& status) const noexcept;
    void EnterTask(
        const ObfmSpacingOwnerServiceReceipt& service,
        const ObfmSpacingOwnerSelection& selection,
        Status& status) noexcept;
    void TickTask(
        const DogfightGeometryFrame& frame,
        const ObfmSpacingOwnerServiceReceipt& service,
        const ObfmSpacingOwnerTaskInput& input,
        ObfmSpacingOwnerTaskReceipt& output,
        Status& status) noexcept;
    void HaltTask(
        bool official_employ_preemption,
        ObfmSpacingOwnerHaltReceipt& output,
        Status& status) noexcept;

private:
    friend struct ObfmSpacingAtomicFixtureAccess;

    void ClearLifecycle(bool clear_post_hit) noexcept;
    void ResetEntry() noexcept;
    void ResetCompletion() noexcept;
    void ResetRecovery() noexcept;

    bool entry_epoch_valid_ = false;
    WezPhaseId entry_phase_id_ = WezPhaseId::P1;
    double entry_official_range_m_ = 0.0;
    double entry_previous_time_s_ = 0.0;
    double entry_previous_spacing_m_ = 0.0;
    bool entry_latched_ = false;
    bool entry_last_frame_valid_ = false;
    ControlFrameIdentity entry_last_frame_identity_{};

    ObfmSpacingCompletionPhase completion_phase_ =
        ObfmSpacingCompletionPhase::Unprimed;
    bool completion_epoch_valid_ = false;
    WezPhaseId completion_phase_id_ = WezPhaseId::P1;
    double completion_official_range_m_ = 0.0;
    double completion_previous_time_s_ = 0.0;
    double completion_previous_closure_mps_ = 0.0;
    double completion_previous_spacing_m_ = 0.0;

    ObfmSpacingRecoveryPhase recovery_phase_ =
        ObfmSpacingRecoveryPhase::Unprimed;
    bool recovery_epoch_valid_ = false;
    WezPhaseId recovery_phase_id_ = WezPhaseId::P1;
    double recovery_official_range_m_ = 0.0;
    double recovery_previous_time_s_ = 0.0;
    double recovery_previous_down_velocity_mps_ = 0.0;
    double recovery_previous_altitude_m_ = 0.0;
    double recovery_previous_spacing_m_ = 0.0;
    bool recovery_climb_history_observed_ = false;
    bool recovery_post_turnaround_spacing_increase_observed_ = false;

    bool task_active_ = false;
    bool post_hit_pending_ = false;
    bool employ_preemption_pending_ = false;
    bool current_projection_required_ = false;
    bool entry_projection_valid_ = false;
    bool entry_projection_admitted_ = false;
    ControlFrameIdentity entry_projection_identity_{};
    double entry_projection_raw_bias_m2ps3_ = 0.0;
    double entry_projection_admitted_bias_m2ps3_ = 0.0;
    ControlFrameIdentity task_entry_frame_identity_{};
    ObfmSpacingOwnerPhase phase_ = ObfmSpacingOwnerPhase::Inactive;
    bool frozen_wez_epoch_valid_ = false;
    WezPhaseId frozen_wez_phase_id_ = WezPhaseId::P1;
    double frozen_wez_official_range_m_ = 0.0;
    bool owner_identity_valid_ = false;
    std::uint64_t owner_episode_epoch_ = 0U;
    std::int32_t owner_own_plane_id_ = -1;
    std::int32_t owner_target_plane_id_ = -1;
};

static_assert(
    std::is_trivially_copyable<ObfmSpacingOwnerServiceInput>::value,
    "OBFM Spacing Service input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmSpacingOwnerServiceReceipt>::value,
    "OBFM Spacing Service receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmSpacingOwnerTaskReceipt>::value,
    "OBFM Spacing Task receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmSpacingOwner>::value,
    "OBFM Spacing owner shadow copies must stay allocation-free.");
static_assert(
    std::is_nothrow_copy_assignable<ObfmSpacingOwner>::value,
    "OBFM Spacing owner shadow commits must stay noexcept.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
