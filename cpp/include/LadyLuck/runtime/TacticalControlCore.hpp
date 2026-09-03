#pragma once

#include "LadyLuck/control/direct_ned/ForceVectorTracking.hpp"
#include "LadyLuck/control/direct_ned/DirectNedLoadVector.hpp"
#include "LadyLuck/control/direct_ned/DirectNedReferenceShaper.hpp"
#include "LadyLuck/control/route5/ReferenceGovernor.hpp"
#include "LadyLuck/control/route5/Route5Guidance.hpp"
#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"
#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/estimator/PlaneStateEstimator.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/runtime/CurrentCisV4EnergyProjectionPort.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"
#include "LadyLuck/runtime/AIPilotABI.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

class ITacticalCommandProvider
{
public:
    virtual ~ITacticalCommandProvider() noexcept = default;
    virtual void Reset() noexcept = 0;
    virtual void Build(
        const TacticalCommandBuildInput& input,
        ControlIntent& output,
        Status& status) noexcept = 0;

    // Source-compatible synchronous seam.  Existing providers need not
    // override it and retain their exact three-argument Build behavior.
    virtual void BuildWithProjection(
        const TacticalCommandBuildInput& input,
        ICurrentCisV4EnergyProjectionPort& projection_port,
        ControlIntent& output,
        Status& status) noexcept
    {
        static_cast<void>(projection_port);
        Build(input, output, status);
    }

    // Post-FCS/wire transaction seam. Legacy providers already commit during
    // Build, so the source-compatible default is an accepted no-op. A staged
    // provider overrides this method and publishes its prepared owner state
    // only after the current finite command has passed all control validation.
    virtual void CommitPrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t writer_id,
        Status& status) noexcept
    {
        static_cast<void>(frame_identity);
        static_cast<void>(writer_id);
        status = Status{};
    }

    // A transactional provider overrides this to discard staged state while
    // retaining its last committed live state. The default preserves legacy
    // behavior, whose Build path has already committed, by performing Reset.
    virtual void AbortPrepared() noexcept
    {
        Reset();
    }
};

// Typed result of the command-owning path for one Step.  InputRejected is an
// honest command-neutral result; it must never be relabelled as a tactical or
// safety command.  CurrentBase is a freshly built same-frame doctrine terminal
// (OBFM LAG, HABFM terminal owner, or DBFM HARD_TURN), not a generic hold.
enum class ControlCommandOutcome : std::int32_t
{
    CurrentBase = 0,
    Tactical = 1,
    Safety = 2,
    InputRejected = 3
};

// Event edge for CurrentBase ownership.  The outcome itself records continued
// occupancy; this event makes acquisition and release visible without a timer,
// dwell threshold, or previous-command reuse.
enum class CurrentBaseOwnershipEvent : std::int32_t
{
    None = 0,
    Started = 1,
    Ended = 2
};

enum class ControlCoreFaultClass : std::int32_t
{
    None = 0,
    InputRejected = 1,
    EstimatorFrameRejected = 2,
    InternalFault = 3
};

// Diagnostic-only progress marker.  `stage` records current/terminal progress.
// Neither this field nor `origin_failure_stage` carries guidance or actuator
// authority.
enum class ControlCoreStage : std::int32_t
{
    NotStarted = 0,
    Preflight = 1,
    FrameContext = 2,
    EstimatorUpdate = 3,
    EstimatorOutputValidation = 4,
    TargetConversion = 5,
    Geometry = 6,
    Envelope = 7,
    AutoGcasEntryInput = 8,
    AutoGcasEntryEvaluation = 9,
    LongitudinalConfiguration = 10,
    GammaLimit = 11,
    NzfeasAuthority = 12,
    TacticalInput = 13,
    TacticalBuild = 14,
    TacticalValidate = 15,
    Guidance = 16,
    TecsCis = 17,
    FeedbackPrepare = 18,
    AutoGcasApply = 19,
    WireValidation = 20,
    FeedbackComplete = 21,
    Authorized = 22,
    TacticalCommit = 23
};

struct ControlCoreReceipt
{
    Status status{StatusCode::InvalidConfiguration};
    bool prestart_command_neutral = false;
    bool control_authorized = false;
    bool estimator_transaction_committed = false;
    ControlCommandOutcome outcome = ControlCommandOutcome::InputRejected;
    CurrentBaseOwnershipEvent current_base_event =
        CurrentBaseOwnershipEvent::None;
    ControlCoreFaultClass fault_class = ControlCoreFaultClass::InternalFault;
    EstimatorTransactionCode estimator_code =
        EstimatorTransactionCode::InvalidConfiguration;
    EstimatorTransactionCause estimator_cause = EstimatorTransactionCause::None;
    EstimatorFaultClass estimator_fault_class = EstimatorFaultClass::InternalFault;
    ControlCoreStage stage = ControlCoreStage::NotStarted;
    // These fields retain the command-blocking origin for diagnostics only.
    // They never authorize a guidance or control reference.
    StatusCode origin_status_code = StatusCode::Ok;
    ControlCoreStage origin_failure_stage = ControlCoreStage::NotStarted;
    bool auto_gcas_override = false;
    safety::AutoGcasPhase auto_gcas_phase = safety::AutoGcasPhase::Inactive;
    // Reserved fixed-layout diagnostic. Production never presents a
    // previous-frame actuator value as a successful current-frame command.
    bool retained_previous_control = false;
    ControlValue control{};
};

// Frozen MSVC x64 C++14 storage contract for the per-frame control result.
// These constants describe diagnostics and the already-produced wire command;
// they do not admit a reference or change guidance/FCS ownership.
struct ControlCoreReceiptX64LayoutManifest
{
    static constexpr std::size_t Size = 72U;
    static constexpr std::size_t Alignment = 4U;
    static constexpr std::size_t StatusOffset = 0U;
    static constexpr std::size_t PrestartCommandNeutralOffset = 4U;
    static constexpr std::size_t ControlAuthorizedOffset = 5U;
    static constexpr std::size_t EstimatorTransactionCommittedOffset = 6U;
    static constexpr std::size_t OutcomeOffset = 8U;
    static constexpr std::size_t CurrentBaseEventOffset = 12U;
    static constexpr std::size_t FaultClassOffset = 16U;
    static constexpr std::size_t EstimatorCodeOffset = 20U;
    static constexpr std::size_t EstimatorCauseOffset = 24U;
    static constexpr std::size_t EstimatorFaultClassOffset = 28U;
    static constexpr std::size_t StageOffset = 32U;
    static constexpr std::size_t OriginStatusCodeOffset = 36U;
    static constexpr std::size_t OriginFailureStageOffset = 40U;
    static constexpr std::size_t AutoGcasOverrideOffset = 44U;
    static constexpr std::size_t AutoGcasPhaseOffset = 48U;
    static constexpr std::size_t RetainedPreviousControlOffset = 52U;
    static constexpr std::size_t ControlOffset = 53U;
};

static_assert(
    std::is_standard_layout<ControlCoreReceipt>::value,
    "ControlCoreReceipt must remain standard-layout");
static_assert(
    std::is_trivially_copyable<ControlCoreReceipt>::value,
    "ControlCoreReceipt must remain trivially copyable");
static_assert(sizeof(ControlCoreReceipt) ==
                  ControlCoreReceiptX64LayoutManifest::Size,
              "ControlCoreReceipt x64 size changed");
static_assert(alignof(ControlCoreReceipt) ==
                  ControlCoreReceiptX64LayoutManifest::Alignment,
              "ControlCoreReceipt x64 alignment changed");
static_assert(offsetof(ControlCoreReceipt, status) ==
                  ControlCoreReceiptX64LayoutManifest::StatusOffset,
              "ControlCoreReceipt status offset changed");
static_assert(
    offsetof(ControlCoreReceipt, prestart_command_neutral) ==
        ControlCoreReceiptX64LayoutManifest::PrestartCommandNeutralOffset,
    "ControlCoreReceipt prestart flag offset changed");
static_assert(
    offsetof(ControlCoreReceipt, control_authorized) ==
        ControlCoreReceiptX64LayoutManifest::ControlAuthorizedOffset,
    "ControlCoreReceipt authorization offset changed");
static_assert(
    offsetof(ControlCoreReceipt, estimator_transaction_committed) ==
        ControlCoreReceiptX64LayoutManifest::
            EstimatorTransactionCommittedOffset,
    "ControlCoreReceipt estimator commit offset changed");
static_assert(offsetof(ControlCoreReceipt, outcome) ==
                  ControlCoreReceiptX64LayoutManifest::OutcomeOffset,
              "ControlCoreReceipt outcome offset changed");
static_assert(
    offsetof(ControlCoreReceipt, current_base_event) ==
        ControlCoreReceiptX64LayoutManifest::CurrentBaseEventOffset,
    "ControlCoreReceipt CurrentBase event offset changed");
static_assert(offsetof(ControlCoreReceipt, fault_class) ==
                  ControlCoreReceiptX64LayoutManifest::FaultClassOffset,
              "ControlCoreReceipt fault class offset changed");
static_assert(offsetof(ControlCoreReceipt, estimator_code) ==
                  ControlCoreReceiptX64LayoutManifest::EstimatorCodeOffset,
              "ControlCoreReceipt estimator code offset changed");
static_assert(offsetof(ControlCoreReceipt, estimator_cause) ==
                  ControlCoreReceiptX64LayoutManifest::EstimatorCauseOffset,
              "ControlCoreReceipt estimator cause offset changed");
static_assert(
    offsetof(ControlCoreReceipt, estimator_fault_class) ==
        ControlCoreReceiptX64LayoutManifest::EstimatorFaultClassOffset,
    "ControlCoreReceipt estimator fault offset changed");
static_assert(offsetof(ControlCoreReceipt, stage) ==
                  ControlCoreReceiptX64LayoutManifest::StageOffset,
              "ControlCoreReceipt stage offset changed");
static_assert(
    offsetof(ControlCoreReceipt, origin_status_code) ==
        ControlCoreReceiptX64LayoutManifest::OriginStatusCodeOffset,
    "ControlCoreReceipt origin status offset changed");
static_assert(
    offsetof(ControlCoreReceipt, origin_failure_stage) ==
        ControlCoreReceiptX64LayoutManifest::OriginFailureStageOffset,
    "ControlCoreReceipt origin stage offset changed");
static_assert(
    offsetof(ControlCoreReceipt, auto_gcas_override) ==
        ControlCoreReceiptX64LayoutManifest::AutoGcasOverrideOffset,
    "ControlCoreReceipt Auto-GCAS override offset changed");
static_assert(offsetof(ControlCoreReceipt, auto_gcas_phase) ==
                  ControlCoreReceiptX64LayoutManifest::AutoGcasPhaseOffset,
              "ControlCoreReceipt Auto-GCAS phase offset changed");
static_assert(
    offsetof(ControlCoreReceipt, retained_previous_control) ==
        ControlCoreReceiptX64LayoutManifest::RetainedPreviousControlOffset,
    "ControlCoreReceipt retained-control offset changed");
static_assert(offsetof(ControlCoreReceipt, control) ==
                  ControlCoreReceiptX64LayoutManifest::ControlOffset,
              "ControlCoreReceipt wire control offset changed");

struct ControlCorePreflightReceipt
{
    Status status{StatusCode::InvalidConfiguration};
    bool prestart_command_neutral = false;
    bool behavior_tree_tick_allowed = false;
    ControlCoreFaultClass fault_class = ControlCoreFaultClass::InternalFault;
    EstimatorTransactionCode estimator_code =
        EstimatorTransactionCode::InvalidConfiguration;
    EstimatorTransactionCause estimator_cause = EstimatorTransactionCause::None;
    EstimatorFaultClass estimator_fault_class = EstimatorFaultClass::InternalFault;
    bool frame_context_ready = false;
    FrameContext frame_context{};
};

struct TacticalControlCoreSnapshot
{
    bool owner_prepared = false;
    std::int32_t owner_plane_id = -1;
    std::int32_t owner_force_side = 0;
    std::uint64_t generation = 0U;
    std::uint64_t step_call_count = 0U;
    std::uint64_t primary_command_count = 0U;
    // Historical counters retained for snapshot/tool compatibility. Current
    // production does not contain internal faults with a previous command.
    std::uint64_t fail_closed_command_count = 0U;
    std::uint64_t internal_fault_containment_count = 0U;
    std::uint64_t input_rejected_count = 0U;
    std::uint64_t internal_fault_count = 0U;
    std::uint64_t current_base_started_count = 0U;
    std::uint64_t current_base_ended_count = 0U;
    bool current_base_active = false;
    ControlCommandOutcome last_command_outcome =
        ControlCommandOutcome::InputRejected;
    CurrentBaseOwnershipEvent last_current_base_event =
        CurrentBaseOwnershipEvent::None;
    bool has_previous_transmitted = false;
    bool previous_transmitted_auto_gcas_active = false;
    bool gcas_energy_handoff_active = false;
    // The body-rate estimator needs one accepted interval after its Init
    // sample before p_endpoint exists.  This state is driven only by accepted
    // live estimator frames; rejected/pre-start frames never arm it.
    bool initial_rate_interval_pending = false;
    ControlFrameIdentity initial_rate_seed_identity{};
    std::uint64_t previous_command_frame = 0U;
    double previous_command_time_s = 0.0;
    ControlValue previous_transmitted{};
    PlaneStateEstimatorSnapshot estimator{};
    control::direct_ned::DirectNedLoadVectorSnapshot direct_ned{};
    control::direct_ned::ForceVectorTrackingSnapshot direct_force_tracking{};
    control::route5::Route5GuidanceSnapshot route5{};
    control::tecs_cis::TecsCisSnapshot tecs_cis{};
    safety::AutoGcasSnapshot auto_gcas{};
    TacticalAge1ControlFeedback previous_tactical_feedback{};
};

// One candidate command evaluation. This remains a per-Step value; moving the
// type to the contract header makes its fixed contents and rollback boundary
// visible without changing the attempt lifecycle.
struct TacticalControlAttempt
{
    bool valid = false;
    Status status{StatusCode::InvalidConfiguration};
    ControlCoreStage failure_stage = ControlCoreStage::Guidance;
    ControlValue wire_control{};
    bool auto_gcas_override = false;
    safety::AutoGcasPhase auto_gcas_phase = safety::AutoGcasPhase::Inactive;
    TacticalAge1ControlFeedback next_feedback{};
};

// Exact mutable controller state captured before optional tactical evaluation
// and restored when that candidate is not admitted.
struct TacticalControlPipelineState
{
    control::route5::Route5Guidance route5{};
    control::direct_ned::DirectNedLoadVector direct_ned{};
    control::direct_ned::ForceVectorTracking direct_force_tracking{};
    control::tecs_cis::TecsCisControl tecs_cis{};
    safety::AutoGcas auto_gcas{};
    bool gcas_energy_handoff_active = false;
};

static_assert(
    std::is_trivially_copyable<TacticalControlAttempt>::value,
    "Tactical control attempt must remain trivially copyable.");
static_assert(
    std::is_nothrow_copy_constructible<TacticalControlPipelineState>::value,
    "Tactical control pipeline capture must remain non-throwing.");
static_assert(
    std::is_nothrow_copy_assignable<TacticalControlPipelineState>::value,
    "Tactical control pipeline restore must remain non-throwing.");

class TacticalControlCore final
{
public:
    explicit TacticalControlCore(
        ITacticalCommandProvider& command_provider) noexcept;

    void Reset(Status& status) noexcept;
    void PrepareOwner(
        std::int32_t owner_plane_id,
        std::int32_t owner_force_side,
        Status& status) noexcept;
    void ValidateInput(
        const KinematicObservationInputV1& input,
        Status& status) const noexcept;
    void Preflight(
        const KinematicObservationInputV1& input,
        ControlCorePreflightReceipt& output) const noexcept;
    void Step(
        const KinematicObservationInputV1& input,
        ControlCoreReceipt& output) noexcept;
    void StepPrepared(
        const KinematicObservationInputV1& input,
        const ControlCorePreflightReceipt& preflight,
        ControlCoreReceipt& output) noexcept;
    void CopySnapshot(
        TacticalControlCoreSnapshot& output) const noexcept;

private:
    void BuildFrameContextRequest(
        const KinematicObservationInputV1& input,
        FrameContextRequest& output) const noexcept;
    void StepPrimary(
        const KinematicObservationInputV1& input,
        const FrameContext& context,
        ControlCoreReceipt& output) noexcept;
    TacticalControlPipelineState CapturePipelineState() const noexcept;
    void RestorePipelineState(
        const TacticalControlPipelineState& snapshot) noexcept;
    void ExecuteIntent(
        const ControlIntent& intent,
        const Result<DogfightGeometryFrame>& frame,
        const EstimatorUpdateResult& estimate,
        const control::route5::CommandEnvelope& envelope,
        const FrameContext& context,
        const safety::AutoGcasEntryReceipt& gcas_entry,
        TacticalControlAttempt& attempt) noexcept;
    void BuildControlReference(
        const ControlIntent& tactical,
        const DogfightGeometryFrame& frame,
        const PlaneState& ownship,
        const EstimatorOutputV6& estimate,
        const control::route5::CommandEnvelope& envelope,
        double dt_s,
        control::tecs_cis::BodyRateLoadEnergyCommand& output,
        TacticalCompletedTotalLoadReceipt& completed_total_load,
        Status& status) noexcept;
    void BuildAutoGcasEntryInput(
        const FrameContext& context,
        const DogfightGeometryFrame& frame,
        const EstimatorUpdateResult& estimate,
        const control::route5::CommandEnvelope& envelope,
        safety::AutoGcasEntryInput& output) noexcept;
    void UpdateEnergyIntegratorHold(
        const EstimatorOutputV6& estimate,
        double flight_path_angle_cmd_rad,
        bool& output,
        Status& status) noexcept;
    void CommitTransmitted(
        const ControlValue& control,
        std::uint64_t command_frame,
        double command_time_s,
        bool auto_gcas_active) noexcept;
    void RefreshSnapshots() noexcept;
    void RecordCommandOutcome(ControlCoreReceipt& output) noexcept;

    ITacticalCommandProvider& command_provider_;
    PlaneStateEstimator estimator_{};
    control::route5::ReferenceGovernor governor_{};
    control::route5::Route5Guidance route5_{};
    control::direct_ned::DirectNedLoadVector direct_ned_{};
    control::direct_ned::ForceVectorTracking direct_force_tracking_{};
    control::direct_ned::DirectNedReferenceShaper direct_ned_shaper_{};
    control::tecs_cis::TecsCisControl tecs_cis_{};
    safety::AutoGcas auto_gcas_{};
    TacticalCommandBuildInputBuilder tactical_input_builder_{};
    TacticalCompletedTotalLoadReceiptBuilder total_load_builder_{};
    TacticalControlCoreSnapshot state_{};
};

} // namespace runtime
} // namespace LadyLuck
