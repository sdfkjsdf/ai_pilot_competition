#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/ThreatRecoveryMargin.hpp"
#include "LadyLuck/guidance/committed/G16ProductionEvidence.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

enum class G16CommitPhase : std::uint8_t
{
    Idle = 0U,
    Committed = 1U,
    BlowThrough = 2U,
    Complete = 3U,
    Failed = 4U,
    Released = 5U
};

enum class G16CommitEvent : std::uint8_t
{
    HeldIdle = 0U,
    Entered = 1U,
    OvershootRecoveryEntered = 2U,
    Retained = 3U,
    Body39Crossed = 4U,
    Completed = 5U,
    CompletionRetained = 6U,
    FailureRetained = 7U,
    SampleRejected = 8U,
    CompletedAlreadyOutsideMaintained = 9U,
    ReleasedThreatRecoveryMarginExhausted = 10U,
    ReleaseRetained = 11U
};

// Exact handoff seam reserved for the adjacent G5b owner. G5b must consume
// the current transaction and latch its own current horizontal speed; G16
// entry speed is deliberately absent.
struct G16G5bCompletionHandoff
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool completed_this_sample = false;
    G16ProductionEvidenceReceipt production_evidence{};
};

struct G16CommittedOwnerReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    G16CommitPhase phase_before = G16CommitPhase::Idle;
    G16CommitPhase phase_after = G16CommitPhase::Idle;
    G16CommitEvent event = G16CommitEvent::HeldIdle;
    bool sample_accepted = false;
    bool entered_this_sample = false;
    bool completed_this_sample = false;
    bool released_this_sample = false;
    bool body_39_crossing_observed = false;
    bool wez_outward_crossing_observed = false;
    bool wez_outside_maintained_observed = false;
    bool command_owner_active = false;
    bool latched_side_resolved = false;
    std::int32_t latched_side_sign = 0;
    bool horizontal_egress_fallback_latched = false;
    ThreatRecoveryMarginReceipt threat_recovery_margin{};
    G16G5bCompletionHandoff g5b_handoff{};
};

struct G16CommittedSelection
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool selected = false;
    bool command_ready = false;
    G16CommitPhase phase = G16CommitPhase::Idle;
    std::uint32_t writer_id = ControlIntentWriterNone;
};

class G16CommittedOwner final
{
public:
    G16CommittedOwner() noexcept = default;

    void Reset() noexcept;
    void ResetForSafetyPreemption() noexcept;
    void HaltExecutionPreservingLifecycle() noexcept;

    // Command-neutral state update. Repeating the same identity returns the
    // cached receipt and never advances lifecycle twice.
    void Observe(
        const G16ProductionEvidenceReceipt& evidence,
        G16CommittedOwnerReceipt& output,
        Status& status) noexcept;
    void CopySelection(
        const ControlFrameIdentity& current_identity,
        G16CommittedSelection& output,
        Status& status) const noexcept;
    // Pure Task writer. Lifecycle and direction latches are never mutated.
    void BuildCandidate(
        const G16ProductionEvidenceReceipt& evidence,
        ControlIntent& output,
        Status& status) const noexcept;

private:
    void BuildReceipt(
        const G16ProductionEvidenceReceipt& evidence,
        G16CommitPhase before,
        G16CommitEvent event,
        bool sample_accepted,
        bool crossing,
        bool wez_crossing,
        bool wez_outside_maintained,
        const ThreatRecoveryMarginReceipt& threat_margin,
        G16CommittedOwnerReceipt& output) noexcept;
    void ResolveDirectionLatch(
        const G16ProductionEvidenceReceipt& evidence,
        Status& status) noexcept;

    G16CommitPhase phase_ = G16CommitPhase::Idle;
    bool handoff_stream_active_ = false;
    bool episode_identity_valid_ = false;
    std::uint64_t episode_epoch_ = 0U;
    std::uint64_t last_frame_index_ = 0U;
    double last_source_time_s_ = 0.0;
    double previous_committed_margin_m_ = 0.0;
    bool previous_committed_margin_valid_ = false;
    double previous_idle_margin_m_ = 0.0;
    bool previous_idle_margin_valid_ = false;
    double previous_enemy_range_m_ = 0.0;
    bool previous_enemy_range_valid_ = false;
    bool previous_blowthrough_robust_outside_ = false;
    bool previous_blowthrough_robust_outside_valid_ = false;
    double enemy_outer_wez_range_m_ = 0.0;
    bool enemy_outer_wez_range_valid_ = false;
    bool g19_open_continuous_ = false;
    bool g19_open_continuous_valid_ = false;
    bool latched_side_resolved_ = false;
    std::int32_t latched_side_sign_ = 0;
    double latched_entry_speed_mps_ = 0.0;
    bool latched_entry_speed_valid_ = false;
    Vector3 latched_turn_direction_ned_{};
    double latched_turn_direction_resolution_rad_ = 0.0;
    bool latched_turn_direction_valid_ = false;
    Vector3 latched_egress_direction_ned_{};
    bool latched_egress_direction_valid_ = false;
    bool latched_horizontal_egress_fallback_ = false;
    G16ProductionEvidenceReceipt cached_evidence_{};
    G16CommittedOwnerReceipt cached_receipt_{};
    bool cached_receipt_valid_ = false;
};

static_assert(
    std::is_trivially_copyable<G16G5bCompletionHandoff>::value,
    "G16-to-G5b handoff must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G16CommittedOwnerReceipt>::value,
    "G16 owner receipt must remain allocation-free.");

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
