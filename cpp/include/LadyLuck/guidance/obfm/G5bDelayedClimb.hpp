#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/committed/G16CommittedOwner.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// The visible G5b Task is the seventh production ControlIntent writer.  The
// parent BehaviorTree owns selection; this module cannot select itself or
// publish from its Service/Condition-facing methods.
constexpr std::uint32_t ControlIntentWriterG5bDelayedClimb = 7U;

enum class G5bDelayedClimbPhase : std::uint8_t
{
    Extend = 0U,
    ZoomClimb = 1U,
    Complete = 2U,
    Release = 3U
};

enum class G5bVerticalPhase : std::uint8_t
{
    Climbing = 0U,
    UnresolvedZeroInterval = 1U,
    Descending = 2U
};

enum class G5bTurnCueReason : std::uint8_t
{
    AdversaryNoseOffOwnHalfSpace = 0U,
    AdversaryNoseLaggingOwnFlightPath = 1U,
    AdversaryNoseAimed = 2U,
    Unobservable = 3U
};

enum class G5bFeedbackFreshness : std::uint8_t
{
    Missing = 0U,
    Fresh = 1U,
    Stale = 2U,
    IndexMismatch = 3U
};

enum class G5bControlBackend : std::uint8_t
{
    Unavailable = 0U,
    TecsCisV4 = 1U,
    Other = 2U
};

enum class G5bSafetyAdmissionReason : std::uint8_t
{
    Admitted = 0U,
    SafetyObservationMissing = 1U,
    SafetyStateTimestampMismatch = 2U,
    CommandEnvelopeInvalid = 3U,
    HardDeckSourceMissing = 4U,
    StallSourceMissing = 5U,
    FlightPathGammaLimitSourceMissing = 6U,
    FlightPathGammaLimitInvalid = 7U,
    HardDeckMarginNotPositive = 8U,
    StallBoundaryInvalid = 9U,
    OneGStallMarginNotPositive = 10U,
    ControlFeedbackMissing = 11U,
    ControlFeedbackNotFresh = 12U,
    ControlBackendNotTecsCisV4 = 13U,
    ControlFeedbackFieldsMissing = 14U,
    PreviousCisNanGuard = 15U,
    PreviousCisCommandClipped = 16U,
    PreviousCisFallback = 17U,
    PreviousEnergyLowerSaturated = 18U,
    PreviousEnergyUpperSaturated = 19U,
    PreviousEnergyRateInvalid = 20U,
    PreviousGcasStateInvalid = 21U,
    PreviousGcasActive = 22U
};

// This comparison authority is intentionally explicit.  The competition
// pairing supplies the same F-16 type and reference-mass E-M publication for
// both aircraft; no heterogeneous-aircraft specific-energy comparison is
// admitted by this module.
enum class G5bEnergyComparisonAuthority : std::uint8_t
{
    Unavailable = 0U,
    CompetitionSameF16TypeAndMass = 1U
};

enum class G5bBoomZoomEligibilityReason : std::uint8_t
{
    Admitted = 0U,
    StrictOutwardCompletionLineageUnresolved = 1U,
    EstimatorChronologyUnresolved = 2U,
    EnergyComparisonAuthorityUnavailable = 3U,
    OwnSpeedFloorUnavailable = 4U,
    OpponentSpeedFloorUnavailable = 5U,
    EnergyBoundsUnresolved = 6U,
    OwnEnergyAdvantageNotProven = 7U,
    StrictCurrentWezClearUnresolved = 8U,
    ZoomEnergyBudgetUnavailable = 9U,
    OpponentVerticalFollowNotInsufficient = 10U,
    EnemyFireOpportunityUnresolved = 11U,
    EnemyFireOpportunityActive = 12U,
    OpponentTurnNotAimed = 13U,
    SafetyNotAdmitted = 14U,
    ThreatRecoveryMarginUnresolved = 15U,
    ThreatRecoveryMarginExhausted = 16U,
    ExtendRangeIncreaseUnresolved = 17U
};

enum class G5bObservationReason : std::uint8_t
{
    MeasuredZoomComplete = 0U,
    ZoomSpeedFloorWithoutMeasuredClimb = 1U,
    ZoomClimbRunning = 2U,
    ReleasedToObfm = 3U,
    DelayedClimbEntryAdmitted = 4U,
    AwaitingStrictWezClear = 5U,
    AwaitingOpponentTurnCompletion = 6U,
    AwaitingEntrySafetyAdmission = 7U,
    SpeedFloorUnavailable = 8U,
    AwaitingResolvedSpeedAboveTacticalFloor = 9U,
    AwaitingResolvedZoomAltitudeBudget = 10U,
    AwaitingEnergyAdvantageProof = 11U,
    AwaitingThreatRecoveryMargin = 12U,
    AwaitingPositiveThreatRecoveryMargin = 13U,
    AwaitingBoomZoomEligibility = 14U
};

enum class G5bReleaseReason : std::uint8_t
{
    None = 0U,
    ExtensionWezReentry = 1U,
    ExtensionRunningSafetyNotAdmitted = 2U,
    ExtensionSpeedFloorUnavailable = 3U,
    ZoomClimbWezReentry = 4U,
    ZoomClimbRunningSafetyNotAdmitted = 5U,
    ZoomClimbSpeedFloorUnavailable = 6U,
    ZoomClimbNoMeasuredAltitudeGain = 7U,
    TreePreempted = 8U,
    ExtensionThreatRecoveryMarginExhausted = 9U,
    ZoomClimbThreatRecoveryMarginExhausted = 10U,
    ZoomClimbEnemyFireOpportunity = 11U,
    ZoomClimbVerticalFollowCapability = 12U,
    ZoomClimbEnergyAdvantageLost = 13U,
    ZoomClimbEstimatorChronologyLost = 14U,
    ZoomClimbStrictCompletionLineageLost = 15U
};

enum class G5bSelectedBranch : std::uint8_t
{
    Invalid = 0U,
    Complete = 1U,
    Release = 2U,
    ZoomEntry = 3U,
    Extend = 4U,
    ZoomClimb = 5U
};

// Current physical margins and completed age-1 control evidence corresponding
// exactly to Python DoctrineSafetyObservation fields consumed by G5b.  All
// optional booleans retain explicit presence bits; a missing field is never
// interpreted as benign False.
struct G5bSafetyEvidence
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    double state_sample_t_sec = 0.0;
    bool envelope_valid = false;
    bool hard_deck_source_present = false;
    double hard_deck_margin_m = 0.0;
    bool stall_source_present = false;
    double stall_speed_1g_mps = 0.0;
    bool flight_path_gamma_limit_source_present = false;
    double flight_path_gamma_limit_rad = 0.0;
    bool previous_control_feedback_present = false;
    G5bFeedbackFreshness feedback_freshness = G5bFeedbackFreshness::Missing;
    G5bControlBackend command_backend = G5bControlBackend::Unavailable;
    bool cis_nan_guard_present = false;
    bool cis_nan_guard = false;
    bool cis_clipped_present = false;
    bool cis_clipped = false;
    bool cis_fallback_present = false;
    bool cis_fallback = false;
    bool energy_lower_saturated_present = false;
    bool energy_lower_saturated = false;
    bool energy_upper_saturated_present = false;
    bool energy_upper_saturated = false;
    bool energy_rate_measurement_valid_present = false;
    bool energy_rate_measurement_valid = false;
    bool auto_gcas_active = false;
    bool auto_gcas_state_valid = false;
    // Same-frame, command-neutral Root gun/prefire facts.  Positive threat is
    // resolved immediately; absence is resolved only when both official and
    // predictive authorities completed on this frame.
    bool boom_zoom_evidence_valid = false;
    ControlFrameIdentity boom_zoom_frame_identity{};
    G5bEnergyComparisonAuthority energy_comparison_authority =
        G5bEnergyComparisonAuthority::Unavailable;
    bool same_f16_type_and_mass_assumed = false;
    bool official_enemy_gun_receipt_current = false;
    bool predictive_prefire_receipt_current = false;
    bool predictive_prefire_absence_resolved = false;
    bool enemy_fire_opportunity_evaluated = false;
    bool enemy_fire_opportunity_active = false;
};

struct G5bSpeedFloorEvidence
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    TacticalSpeedFloorSample sample{};
    bool opponent_valid = false;
    TacticalSpeedFloorSample opponent_sample{};
};

// Service-owned routing evidence only.  It cannot publish guidance or own a
// phase.  All bounds are conservative with respect to the accepted float32
// state transport; unresolved arithmetic or authority is ordinary
// nonadmission.
struct G5bBoomZoomEligibilityReceipt
{
    bool evaluated = false;
    ControlFrameIdentity frame_identity{};
    ControlFrameIdentity strict_outward_completion_identity{};
    bool strict_outward_wez_complete = false;
    bool estimator_chronology_consistent = false;
    IntentOptionalValue<double> extend_start_range_upper_m{};
    IntentOptionalValue<double> current_range_lower_m{};
    bool extend_range_increase_observed = false;
    G5bEnergyComparisonAuthority energy_comparison_authority =
        G5bEnergyComparisonAuthority::Unavailable;
    bool same_f16_type_and_mass_assumed = false;
    bool comparison_authority_available = false;
    bool own_speed_floor_admitted = false;
    bool opponent_speed_floor_admitted = false;
    double own_tactical_speed_floor_mps = 0.0;
    double opponent_tactical_speed_floor_mps = 0.0;
    double own_speed_lower_mps = 0.0;
    double opponent_speed_upper_mps = 0.0;
    double own_altitude_lower_m = 0.0;
    double opponent_altitude_upper_m = 0.0;
    bool energy_bounds_evaluated = false;
    double own_specific_energy_lower_m = 0.0;
    double opponent_specific_energy_upper_m = 0.0;
    bool own_energy_advantage_resolved = false;
    bool strict_current_wez_clear = false;
    bool zoom_energy_budget_available = false;
    double own_zoom_target_lower_m = 0.0;
    double opponent_follow_ceiling_upper_m = 0.0;
    bool opponent_vertical_follow_insufficient = false;
    bool official_enemy_gun_receipt_current = false;
    bool predictive_prefire_receipt_current = false;
    bool predictive_prefire_absence_resolved = false;
    bool enemy_fire_opportunity_evaluated = false;
    bool enemy_fire_opportunity_absent = false;
    bool safety_admitted = false;
    bool zoom_admitted = false;
    G5bBoomZoomEligibilityReason reason =
        G5bBoomZoomEligibilityReason::
            StrictOutwardCompletionLineageUnresolved;
};

struct G5bSafetyAdmissionReceipt
{
    bool evaluated = false;
    bool admitted = false;
    G5bSafetyAdmissionReason reason =
        G5bSafetyAdmissionReason::SafetyObservationMissing;
};

// Shared OBFM 3-D safety surface. Current kinematics, finite command bounds,
// and the actual hard-deck margin own admission. Optional physical capability
// metadata and age-1 telemetry never suppress a valid same-frame command.
G5bSafetyAdmissionReceipt EvaluateG5bSafetyAdmission(
    const DogfightGeometryFrame& frame,
    const G5bSafetyEvidence& safety,
    bool completed_command_saturation_is_expected,
    bool running) noexcept;

// Service-owned, same-frame observation.  It classifies admission and actual
// measured climb only; it has no guidance, p/q/r/Nz, surface, thrust, or
// production command authority.
struct G5bDelayedClimbObservation
{
    bool evaluated = false;
    ControlFrameIdentity frame_identity{};
    G5bDelayedClimbPhase phase = G5bDelayedClimbPhase::Extend;
    bool wez_clear = false;
    // Same-frame specific-energy evidence.  A surplus is proven only by the
    // strict mirror of the shared HABFM deficit test:
    // delta_specific_energy_m > energy_evidence_band_m.  Missing or
    // unrepresentable evidence is ordinary fail-closed nonadmission.
    bool energy_standing_evaluated = false;
    double delta_specific_energy_m = 0.0;
    double energy_evidence_band_m = 0.0;
    bool energy_advantage_proven = false;
    // Predictive official-WEZ margin.  A finite non-positive closure is a
    // resolved infinite margin: evaluated remains true, time_to_enemy_wez_s
    // remains absent, and threat_recovery_margin_exhausted remains false.
    // Missing capability/arithmetic blocks ZoomEntry but never manufactures
    // a release for an already-running phase.
    bool threat_recovery_margin_evaluated = false;
    double closing_speed_mps = 0.0;
    IntentOptionalValue<double> time_to_enemy_wez_s{};
    IntentOptionalValue<double> own_reversal_time_s{};
    bool threat_recovery_margin_exhausted = false;
    bool opponent_turn_aimed = false;
    G5bTurnCueReason opponent_turn_cue_reason =
        G5bTurnCueReason::AdversaryNoseAimed;
    G5bSafetyAdmissionReceipt entry_safety{};
    G5bSafetyAdmissionReceipt running_safety{};
    bool speed_floor_admitted = false;
    double own_speed_mps = 0.0;
    double own_speed_lower_mps = 0.0;
    IntentOptionalValue<double> tactical_speed_floor_mps{};
    double altitude_m = 0.0;
    double altitude_lower_m = 0.0;
    double altitude_upper_m = 0.0;
    G5bVerticalPhase vertical_phase =
        G5bVerticalPhase::UnresolvedZeroInterval;
    bool climb_observed = false;
    IntentOptionalValue<double> climb_start_altitude_upper_m{};
    bool altitude_gain_observed = false;
    IntentOptionalValue<double> target_altitude_m{};
    bool target_altitude_reached = false;
    bool speed_floor_guard_reached = false;
    bool climb_entry_admitted = false;
    G5bBoomZoomEligibilityReceipt boom_zoom_eligibility{};
    G5bObservationReason reason =
        G5bObservationReason::AwaitingStrictWezClear;
    bool behavior_authority = false;
    bool tactical_command_authority = false;
    bool flight_control_authority = false;
    bool production_authority = false;
};

struct G5bDelayedClimbSelection
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    G5bDelayedClimbPhase observed_phase = G5bDelayedClimbPhase::Extend;
    G5bSelectedBranch selected_branch = G5bSelectedBranch::Invalid;
    G5bReleaseReason release_reason = G5bReleaseReason::None;
    bool command_task = false;
};

struct G5bDelayedClimbTaskReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    G5bSelectedBranch branch = G5bSelectedBranch::Invalid;
    // Candidate construction only.  The visible BT Task still must publish
    // this candidate through DoctrineBtContract before it has command authority.
    bool command_ready = false;
    bool completed_this_sample = false;
    bool released_this_sample = false;
    G5bReleaseReason release_reason = G5bReleaseReason::None;
};

struct G5bDelayedClimbHaltReceipt
{
    bool valid = false;
    bool was_active = false;
    bool terminal = false;
    bool completed = false;
    bool preempted = false;
    G5bReleaseReason reason = G5bReleaseReason::None;
};

struct G5bDelayedClimbSnapshot
{
    bool active = false;
    G5bDelayedClimbPhase phase = G5bDelayedClimbPhase::Extend;
    bool horizontal_direction_valid = false;
    Vector3 horizontal_direction_ned{};
    IntentOptionalValue<double> target_altitude_m{};
    IntentOptionalValue<double> climb_start_altitude_upper_m{};
    bool climb_observed = false;
    bool strict_outward_completion_lineage_valid = false;
    ControlFrameIdentity strict_outward_completion_identity{};
    IntentOptionalValue<double> extend_start_range_upper_m{};
    bool last_observation_identity_valid = false;
    ControlFrameIdentity last_observation_identity{};
};

// Allocation-free stateful owner matching d90 G5bDelayedClimbOwnerTree.  The
// visible parent BT calls Observe() from its Service, Select() from read-only
// Conditions, and exactly one named *Task method from the selected Task.
class G5bDelayedClimb final
{
public:
    G5bDelayedClimb() noexcept;

    void Reset() noexcept;
    void CopySnapshot(G5bDelayedClimbSnapshot& output) const noexcept;
    void Enter(
        const committed::G16G5bCompletionHandoff& handoff,
        Status& status) noexcept;
    void Observe(
        const committed::G16ProductionEvidenceReceipt& current_evidence,
        const G5bSafetyEvidence& safety,
        const G5bSpeedFloorEvidence& speed_floor,
        G5bDelayedClimbObservation& output,
        Status& status) noexcept;
    void Select(
        const G5bDelayedClimbObservation& observation,
        G5bDelayedClimbSelection& output,
        Status& status) const noexcept;

    // Named Task-only writers.  Service and Condition methods never call
    // these functions and never receive a ControlIntent output parameter.
    void BuildExtendTask(
        const committed::G16ProductionEvidenceReceipt& current_evidence,
        const G5bDelayedClimbObservation& observation,
        const G5bDelayedClimbSelection& selection,
        ControlIntent& output,
        G5bDelayedClimbTaskReceipt& task,
        Status& status) const noexcept;
    void BuildZoomEntryTask(
        const committed::G16ProductionEvidenceReceipt& current_evidence,
        const G5bSafetyEvidence& safety,
        const G5bDelayedClimbObservation& observation,
        const G5bDelayedClimbSelection& selection,
        ControlIntent& output,
        G5bDelayedClimbTaskReceipt& task,
        Status& status) noexcept;
    void BuildZoomClimbTask(
        const committed::G16ProductionEvidenceReceipt& current_evidence,
        const G5bSafetyEvidence& safety,
        const G5bDelayedClimbObservation& observation,
        const G5bDelayedClimbSelection& selection,
        ControlIntent& output,
        G5bDelayedClimbTaskReceipt& task,
        Status& status) const noexcept;
    void CompleteTask(
        const G5bDelayedClimbObservation& observation,
        const G5bDelayedClimbSelection& selection,
        G5bDelayedClimbTaskReceipt& task,
        Status& status) noexcept;
    void ReleaseTask(
        const G5bDelayedClimbObservation& observation,
        const G5bDelayedClimbSelection& selection,
        G5bDelayedClimbTaskReceipt& task,
        Status& status) noexcept;

    // Halt clears a running sequence as preempted.  COMPLETE/RELEASE are
    // terminal cleanup and therefore do not acquire a synthetic abort reason.
    void Halt(G5bDelayedClimbHaltReceipt& output) noexcept;
    void Exit() noexcept;

private:
    void ClearState() noexcept;
    void BuildZoomIntent(
        const committed::G16ProductionEvidenceReceipt& current_evidence,
        const G5bSafetyEvidence& safety,
        const G5bDelayedClimbSnapshot& snapshot,
        ControlIntent& output,
        Status& status) const noexcept;

    G5bDelayedClimbSnapshot snapshot_{};
    bool cached_observation_valid_ = false;
    G5bDelayedClimbObservation cached_observation_{};
};

static_assert(
    std::is_trivially_copyable<G5bSafetyEvidence>::value,
    "G5b safety evidence must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G5bDelayedClimbObservation>::value,
    "G5b observation must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G5bBoomZoomEligibilityReceipt>::value,
    "G5b BoomZoom eligibility must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G5bDelayedClimbSnapshot>::value,
    "G5b state must remain allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
