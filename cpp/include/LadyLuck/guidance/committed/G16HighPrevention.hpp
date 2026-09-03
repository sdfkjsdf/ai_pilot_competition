#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/committed/G16HighObservation.hpp"
#include "LadyLuck/guidance/committed/G16ProductionEvidence.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

enum class G16HighPhase : std::uint8_t
{
    Idle = 0U,
    SharedClimb = 1U,
    SharedClimbReferenceWait = 2U,
    HighRollInCapabilityWait = 3U,
    HighPostApexRollInCapabilityWait = 4U,
    HighRollIn = 5U,
    HighPostApexRollIn = 6U,
    HighObfmLagHandoff = 7U,
    G16EHandoff = 8U,
    UnsupportedDisplacement = 9U
};

enum class G16HighCandidateMask : std::uint8_t
{
    None = 0U,
    High = 1U,
    Displacement = 2U
};

enum class G16HighReferenceRole : std::uint8_t
{
    None = 0U,
    SharedClimb = 1U,
    HighRollIn = 2U,
    ObfmLagHandoff = 3U,
    G16EHandoff = 4U,
    Unsupported = 5U
};

enum class G16HighReason : std::uint8_t
{
    Reset = 0U,
    TransactionUnavailable = 1U,
    ObservationSeeded = 2U,
    EntrySafetyUnavailable = 3U,
    EntryGeometryUnavailable = 4U,
    EffectiveLoadAuthorityUnavailable = 5U,
    ManualSelectionNotHigh = 6U,
    SharedClimbReferenceUnavailable = 7U,
    SharedClimbCommand = 8U,
    HighRollInReferenceUnavailable = 9U,
    HighRollInCommand = 10U,
    HighToLagHandoff = 11U,
    G16EHandoff = 12U,
    DisplacementOwnerNotInThisModule = 13U,
    IdentityRestarted = 14U,
    InternalContractInvalid = 15U
};

// Command-neutral speed authority selected by the ordinary precision-tracking
// channel before G16-P admission.  The receipt carries the same final speed
// reference that writer 5 would use: station hold when explicitly available,
// otherwise the moving-point longitudinal reference.  Current-speed echo is
// diagnostic only and cannot start a High-Yo-Yo.
enum class G16PrecisionSpeedSource : std::uint8_t
{
    Unavailable = 0U,
    CurrentSpeedEcho = 1U,
    PhaseLongitudinal = 2U,
    StationHold = 3U
};

struct G16PrecisionSpeedReceipt
{
    bool evaluated = false;
    bool admitted = false;
    ControlFrameIdentity frame_identity{};
    G16PrecisionSpeedSource source =
        G16PrecisionSpeedSource::Unavailable;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
};

struct G16HighGeometryReceipt
{
    bool evaluated = false;
    bool turn_path_room_available = false;
    bool direct_lag_reentry_resolved = false;
    bool direct_lag_reentry_admissible = false;
    bool entry_point_ahead = false;
    bool course_intersection_ahead = false;
    bool course_intersection_on_window_segment = false;
    bool radius_advantage_resolved = false;
    double radius_advantage_lower_m = 0.0;
    double plane_offset_m = 0.0;
    Vector3 entry_point_ned_m{};
    Vector3 course_intersection_ned_m{};
};

struct G16HighReferenceReceipt
{
    bool evaluated = false;
    bool available = false;
    bool physically_infeasible = false;
    Vector3 aim_point_ned_m{};
    Vector3 acceleration_ned_mps2{};
    Vector3 transverse_specific_force_ned_mps2{};
    double requested_load_g = 0.0;
    double effective_load_limit_g = 0.0;
    double climb_axis_raw_required_accel_mps2 = 0.0;
    double climb_axis_required_accel_mps2 = 0.0;
    double body39_required_accel_mps2 = 0.0;
};

struct G16HighToLagHandoff
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool selected_this_sample = false;
    bool consumed = false;
    G16ProductionEvidenceReceipt production_evidence{};
};

// Command-neutral Service receipt.  The acceleration is a raw guidance
// request; it is not a body-rate/Nz, surface, thrust, estimator, or aircraft-
// response receipt.
struct G16HighPreventionReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool source_simultaneous = false;
    G16HighPhase phase_before = G16HighPhase::Idle;
    G16HighPhase phase_after = G16HighPhase::Idle;
    G16HighCandidateMask committed_candidates = G16HighCandidateMask::None;
    G16HighReferenceRole reference_role = G16HighReferenceRole::None;
    G16HighReason reason = G16HighReason::Reset;
    bool selection_committed = false;
    bool selected = false;
    bool command_ready = false;
    bool effective_nz_valid = false;
    double effective_nz_limit_g = 0.0;
    bool previous_high_command_applied = false;
    G16PrecisionSpeedReceipt precision_speed{};
    G16HighObservationReceipt observation{};
    G16HighGeometryReceipt geometry{};
    G16HighReferenceReceipt shared_climb{};
    G16HighReferenceReceipt roll_in{};
    G16HighToLagHandoff high_to_lag{};
    ControlIntent candidate{};
};

struct G16HighSelection
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool command_selected = false;
    bool high_to_lag_selected = false;
    bool g16_e_handoff = false;
    std::uint32_t writer_id = ControlIntentWriterNone;
    G16HighPhase phase = G16HighPhase::Idle;
};

// Fixed transaction state owned by command-candidate evaluation and
// preparation.  The command-neutral G16HighObservation owner and its cached
// observation receipt are deliberately excluded: aborting or committing a
// candidate must not rewind physical target-turn/apex history already
// accepted by the pre-root observation Service.
struct G16HighPreventionTransactionState
{
    G16HighPhase phase = G16HighPhase::Idle;
    G16HighCandidateMask committed_candidates =
        G16HighCandidateMask::None;
    bool selection_committed = false;
    bool high_to_lag_consumed = false;
    bool shared_climb_plane_valid = false;
    Vector3 shared_climb_plane_normal_ned{};
    bool previous_target_velocity_valid = false;
    Vector3 previous_target_velocity_ned_mps{};
    bool previous_target_time_valid = false;
    double previous_target_time_s = 0.0;
    bool cached_receipt_valid = false;
    G16HighPreventionReceipt cached_receipt{};
};

// Production G16-P High owner.  ObserveKinematics is the global, command-
// neutral target-turn/apex Service.  Evaluate owns the latched High lifecycle
// and candidate construction.  BuildCandidate is a pure selected Task writer.
class G16HighPrevention final
{
public:
    G16HighPrevention() noexcept = default;

    void Reset() noexcept;
    void ResetForSafetyPreemption() noexcept;
    void HaltExecutionPreservingObservation() noexcept;
    void ObserveKinematics(
        const runtime::TacticalCommandBuildInput& input,
        G16HighObservationReceipt& output,
        Status& status) noexcept;
    void Evaluate(
        const runtime::TacticalCommandBuildInput& input,
        const G16ProductionEvidenceReceipt& evidence,
        const G16PrecisionSpeedReceipt& precision_speed,
        G16HighPreventionReceipt& output,
        Status& status) noexcept;
    void CopySelection(
        const ControlFrameIdentity& current_identity,
        G16HighSelection& output,
        Status& status) const noexcept;
    void BuildCandidate(
        const ControlFrameIdentity& current_identity,
        ControlIntent& output,
        Status& status) const noexcept;
    void ConsumeHighToLagHandoff(
        const ControlFrameIdentity& current_identity,
        G16HighToLagHandoff& output,
        Status& status) noexcept;
    void CaptureTransactionState(
        G16HighPreventionTransactionState& output) const noexcept;
    void RestoreTransactionState(
        const G16HighPreventionTransactionState& input) noexcept;

private:
    void ResetExecution() noexcept;
    void BuildEffectiveLoadAuthority(
        const runtime::TacticalCommandBuildInput& input,
        double& output,
        bool& valid) noexcept;
    void BuildGeometry(
        const runtime::TacticalCommandBuildInput& input,
        double effective_nz_g,
        G16HighGeometryReceipt& output) const noexcept;
    void BuildSharedClimb(
        const runtime::TacticalCommandBuildInput& input,
        const G16PrecisionSpeedReceipt& precision_speed,
        double effective_nz_g,
        G16HighReferenceReceipt& output) noexcept;
    void BuildRollIn(
        const runtime::TacticalCommandBuildInput& input,
        double effective_nz_g,
        G16HighReferenceReceipt& output) const noexcept;
    void BuildIntent(
        const runtime::TacticalCommandBuildInput& input,
        const G16PrecisionSpeedReceipt& precision_speed,
        const G16HighReferenceReceipt& reference,
        bool inversion_allowed,
        ControlIntent& output,
        Status& status) const noexcept;
    void AdvanceLifecycle(
        const G16ProductionEvidenceReceipt& evidence,
        G16HighCandidateMask current_candidates,
        bool selection_resolved,
        bool current_speed_excess_resolved,
        bool current_speed_excess,
        const G16HighGeometryReceipt& geometry,
        const G16HighReferenceReceipt& shared_climb,
        const G16HighReferenceReceipt& roll_in,
        bool apex_resolved,
        bool apex_crossed,
        bool roll_complete_resolved,
        bool roll_complete,
        G16HighReason& reason) noexcept;

    G16HighObservation observation_owner_{};
    G16HighPhase phase_ = G16HighPhase::Idle;
    G16HighCandidateMask committed_candidates_ = G16HighCandidateMask::None;
    bool selection_committed_ = false;
    bool high_to_lag_consumed_ = false;
    bool shared_climb_plane_valid_ = false;
    Vector3 shared_climb_plane_normal_ned_{};
    bool previous_target_velocity_valid_ = false;
    Vector3 previous_target_velocity_ned_mps_{};
    bool previous_target_time_valid_ = false;
    double previous_target_time_s_ = 0.0;
    bool cached_observation_valid_ = false;
    G16HighObservationReceipt cached_observation_{};
    bool cached_receipt_valid_ = false;
    G16HighPreventionReceipt cached_receipt_{};
};

static_assert(
    std::is_trivially_copyable<G16PrecisionSpeedReceipt>::value,
    "G16 precision-speed receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16HighPreventionReceipt>::value,
    "G16 High Service receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16HighSelection>::value,
    "G16 High selection must remain allocation-free.");
static_assert(
    std::is_standard_layout<G16HighPreventionTransactionState>::value,
    "G16 High transaction state must have fixed standard layout.");
static_assert(
    std::is_trivially_copyable<
        G16HighPreventionTransactionState>::value,
    "G16 High transaction state must remain allocation-free bitwise state.");
static_assert(sizeof(G16HighPreventionTransactionState) == 3576U,
    "G16 High transaction state x64 ABI size changed.");
static_assert(alignof(G16HighPreventionTransactionState) == 8U,
    "G16 High transaction state x64 ABI alignment changed.");
static_assert(offsetof(G16HighPreventionTransactionState, phase) == 0U,
    "G16 High transaction phase offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState, committed_candidates) == 1U,
    "G16 High transaction candidate-mask offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState, selection_committed) == 2U,
    "G16 High transaction selection latch offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState, high_to_lag_consumed) == 3U,
    "G16 High transaction handoff-consumed offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             shared_climb_plane_valid) == 4U,
    "G16 High transaction climb-plane validity offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             shared_climb_plane_normal_ned) == 8U,
    "G16 High transaction climb-plane offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             previous_target_velocity_valid) == 32U,
    "G16 High transaction target-velocity validity offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             previous_target_velocity_ned_mps) == 40U,
    "G16 High transaction target-velocity offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             previous_target_time_valid) == 64U,
    "G16 High transaction target-time validity offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             previous_target_time_s) == 72U,
    "G16 High transaction target-time offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState,
             cached_receipt_valid) == 80U,
    "G16 High transaction receipt validity offset changed.");
static_assert(
    offsetof(G16HighPreventionTransactionState, cached_receipt) == 88U,
    "G16 High transaction receipt offset changed.");
static_assert(
    sizeof(G16HighPreventionTransactionState) < sizeof(G16HighObservation),
    "G16 High transaction state must exclude physical observation history.");

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
