#pragma once

#include "LadyLuck/guidance/prefire/RootPrefireThreatObservation.hpp"
#include "LadyLuck/runtime/RootTacticalCommandWriters.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

// Command-neutral result of the existing ImmediateGunDefense observation
// path. This owner derives it once from the same-frame official damage receipt
// and the existing staged Root prefire consumer. It does not add a threat
// threshold or infer a break side outside the existing toward-side receipt.
struct StaticImmediateGunDefenseAdmissionReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool immediate_defense_required = false;
    bool official_damage_active = false;
    bool predictive_prefire_active = false;
    bool predictive_check_extend_hold = false;
    bool entry_side_sign_valid = false;
    std::int32_t entry_side_sign = 1;
};

enum class StaticSafetyGunDisposition : std::uint8_t
{
    NotApplicable = 0U,
    AutoGcasPrepared = 1U,
    GunBreakPrepared = 2U,
    Fault = 3U
};

// Per-frame preparation evidence only. The ControlIntent is still a raw
// guidance request; this receipt is not FCS output or aircraft response.
struct StaticSafetyGunPreparedReceipt
{
    bool prepare_attempted = false;
    ControlFrameIdentity frame_identity{};
    bool safety_current_required = false;
    bool safety_feedback_latched = false;
    bool safety_required = false;
    StaticImmediateGunDefenseAdmissionReceipt gun_admission{};
    runtime::RootGunPreTaskEvidence root_gun_evidence{};
    StatusCode root_gun_evidence_status = StatusCode::Ok;
    bool prefire_safety_veto = false;
    bool prefire_observation_attempted = false;
    bool prefire_observation_ready = false;
    StatusCode prefire_observation_status = StatusCode::Ok;
    guidance::prefire::RootPrefireThreatShadowReceipt
        prefire_threat_shadow{};
    bool prefire_consumer_attempted = false;
    bool prefire_consumer_ready = false;
    StatusCode prefire_consumer_status = StatusCode::Ok;
    guidance::prefire::RootPrefireThreatConsumerDecision
        prefire_consumer{};
    bool toward_side_observation_attempted = false;
    bool toward_side_observation_ready = false;
    StatusCode toward_side_observation_status = StatusCode::Ok;
    guidance::prefire::RootGunTowardSideShadowReceipt toward_side{};
    bool optional_evidence_fault = false;
    StaticSafetyGunDisposition disposition =
        StaticSafetyGunDisposition::NotApplicable;
    std::uint32_t prepared_writer_id = ControlIntentWriterNone;
    std::uint8_t candidate_count = 0U;
    bool observation_state_staged = false;
    bool command_candidate_staged = false;
    bool state_committed = false;
    bool state_aborted = false;
    StatusCode status_code = StatusCode::Ok;
};

// Fixed-storage owner for the two highest-priority static branches only:
// current-frame Auto-GCAS (writer 1) and base Gun BREAK (writer 2). It does
// not contain G4, phase-graded, OfficialGun Tracking (writer 26), Snapshot
// (writer 27), or any tactical threshold/gain. It only permits the separate
// fixed ImmediateGun response owner to finalize an already-prepared writer 2
// as writer 14, 30, 27, 24, 3, or the unchanged writer 2. All mutable Root
// writer state is prepared on a copy and becomes live only after the selected
// finite wire transaction.
class StaticSafetyGunStagedOwner final
{
public:
    StaticSafetyGunStagedOwner() noexcept;

    void Reset() noexcept;

    void Prepare(
        const runtime::TacticalCommandBuildInput& input,
        ControlIntent& output,
        StaticSafetyGunPreparedReceipt& receipt,
        Status& status) noexcept;

    // If this owner prepared writer 1, published_writer_id must match it. A
    // prepared base writer 2 may complete as writer 2 or as the same-frame G4
    // (writers 14/30), production Snapshot Plane Change (writer 27), or
    // side-preserving HardTurn (writer 3). Tracking writer 26 and all
    // other replacement writers are rejected.
    // For NotApplicable, the call commits command-neutral Gun observation state
    // after some other final writer has completed the same frame successfully.
    void ValidatePrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) const noexcept;

    void CommitPrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) noexcept;

    // Discards only the candidate copy and preserves the last committed owner
    // state. It is safe to call when no frame is staged.
    void AbortPrepared() noexcept;

    void CopySnapshot(
        StaticSafetyGunPreparedReceipt& output) const noexcept;

    // Predictive DBFM BREAK reads the next side of the same committed Gun
    // episode without activating or advancing that episode.  The eventual
    // official Gun entry remains the sole state-transition owner.
    std::int32_t NextGunSideSign() const noexcept
    {
        return committed_writers_.NextGunSideSign();
    }

private:
    void DiscardStagedState() noexcept;
    void RejectPrepared(
        StatusCode code,
        ControlIntent& output,
        Status& status) noexcept;

    runtime::RootTacticalCommandWriters committed_writers_{};
    runtime::RootTacticalCommandWriters staged_writers_{};
    guidance::prefire::RootPrefireThreatObserver
        committed_prefire_observer_{};
    guidance::prefire::RootPrefireThreatObserver
        staged_prefire_observer_{};
    guidance::prefire::RootPrefireThreatConsumer
        committed_prefire_consumer_{};
    guidance::prefire::RootPrefireThreatConsumer
        staged_prefire_consumer_{};
    bool staged_ready_ = false;
    ControlFrameIdentity staged_frame_identity_{};
    std::uint32_t staged_writer_id_ = ControlIntentWriterNone;
    StaticSafetyGunPreparedReceipt snapshot_{};
};

static_assert(
    std::is_trivially_copyable<
        StaticImmediateGunDefenseAdmissionReceipt>::value,
    "Static ImmediateGunDefense admission must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<StaticSafetyGunPreparedReceipt>::value,
    "Static safety/Gun prepared receipt must remain allocation-free.");
static_assert(
    std::is_nothrow_copy_assignable<
        runtime::RootTacticalCommandWriters>::value,
    "Root tactical writer state must support non-throwing staged copies.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::prefire::RootPrefireThreatObserver>::value,
    "Root prefire observer state must support non-throwing staged copies.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::prefire::RootPrefireThreatConsumer>::value,
    "Root prefire consumer state must support non-throwing staged copies.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
