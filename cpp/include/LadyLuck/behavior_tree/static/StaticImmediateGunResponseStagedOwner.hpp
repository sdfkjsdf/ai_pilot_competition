#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtResult.hpp"
#include "LadyLuck/behavior_tree/static/StaticGunSnapshotStagedOwner.hpp"
#include "LadyLuck/guidance/dbfm/DbfmPhaseGradedResponse.hpp"
#include "LadyLuck/guidance/g4/HighGBarrelOwner.hpp"
#include "LadyLuck/runtime/RootGunPreTaskEvidenceProvider.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class StaticImmediateGunResponseDisposition : std::uint8_t
{
    NotEvaluated = 0U,
    BaseBreakRetained = 1U,
    G4HighGPrepared = 3U,
    SnapshotPrepared = 4U,
    PhaseGradedHardTurnPrepared = 5U,
    InputContractFault = 6U
};

enum class StaticImmediateGunResponseReason : std::uint8_t
{
    NotEvaluated = 0U,
    BaseBreakNotApplicable = 1U,
    G4HighGSelected = 3U,
    SnapshotSelected = 4U,
    PhaseGradedHardTurnSelected = 5U,
    OptionalResponseFaultContained = 6U,
    BaseBreakContractFault = 7U,
    SameFrameContractFault = 8U,
    HardWezBaseBreakRetained = 9U
};

// Fixed same-frame inputs copied from an already-prepared
// StaticSafetyGunPreparedReceipt.  writer2_same_frame_admitted is the sole Gun
// admission authority; the response owner does not re-observe official threat
// or add another WEZ/capability gate.
struct StaticImmediateGunResponseStagedInput
{
    runtime::TacticalCommandBuildInput tactical_input{};
    bool writer2_same_frame_admitted = false;
    ControlFrameIdentity safety_gun_frame_identity{};
    bool entry_side_sign_valid = false;
    std::int32_t entry_side_sign = 1;
    runtime::RootGunPreTaskEvidence root_gun_evidence{};
    ControlIntent base_break{};
};

// Raw-guidance transaction evidence only.  The selected ControlIntent remains
// subject to Route-5, FCS, surface/thrust limiting and wire validation before
// any state below may become live.
struct StaticImmediateGunResponsePreparedReceipt
{
    bool prepare_attempted = false;
    ControlFrameIdentity frame_identity{};
    bool base_writer2_same_frame_admitted = false;

    guidance::g4::HighGBarrelSelectionReceipt g4_high_g_selection{};
    guidance::g4::HighGBarrelTaskReceipt g4_high_g_task{};
    bool g4_high_g_release_staged = false;

    bool snapshot_attempted = false;
    bool snapshot_transaction_staged = false;
    StaticGunSnapshotPreparedReceipt snapshot{};

    bool phase_graded_attempted = false;
    GunDefenseSnapshot phase_gun_episode{};
    guidance::dbfm::DbfmPhaseGradedResponseReceipt phase_graded{};
    StatusCode phase_graded_status_code = StatusCode::Ok;
    BtTickResult selector_result{};

    StaticImmediateGunResponseDisposition disposition =
        StaticImmediateGunResponseDisposition::NotEvaluated;
    StaticImmediateGunResponseReason reason =
        StaticImmediateGunResponseReason::NotEvaluated;
    std::uint32_t prepared_writer_id = ControlIntentWriterNone;
    std::uint8_t candidate_count = 0U;
    bool state_staged = false;
    bool state_committed = false;
    bool state_aborted = false;
    std::uint64_t captured_generation = 0U;
    bool optional_response_fault_contained = false;
    StatusCode diagnostic_status_code = StatusCode::Ok;
};

// One fixed-storage owner for the ImmediateGunDefense response priority:
// G4 High-G, Snapshot writer27, side-preserving HardTurn, then the
// already-valid writer2 base.  Tracking writer26 is intentionally absent.
class StaticImmediateGunResponseStagedOwner final
{
public:
    StaticImmediateGunResponseStagedOwner() noexcept;

    void Reset() noexcept;

    void Prepare(
        const StaticImmediateGunResponseStagedInput& input,
        ControlIntent& output,
        StaticImmediateGunResponsePreparedReceipt& receipt,
        Status& status) noexcept;

    void ValidatePrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) const noexcept;

    void CommitPrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t published_writer_id,
        Status& status) noexcept;

    void AbortPrepared() noexcept;

    void CopySnapshot(
        StaticImmediateGunResponsePreparedReceipt& output) const noexcept;

private:
    void DiscardStagedState() noexcept;
    void RetainBaseBreak(
        StaticImmediateGunResponseReason reason,
        StatusCode diagnostic_status_code,
        const ControlIntent& base_break,
        ControlIntent& output,
        StaticImmediateGunResponsePreparedReceipt& receipt,
        Status& status) noexcept;
    void RejectInput(
        StaticImmediateGunResponseReason reason,
        StatusCode code,
        ControlIntent& output,
        StaticImmediateGunResponsePreparedReceipt& receipt,
        Status& status) noexcept;

    guidance::g4::HighGBarrelOwner committed_high_g_owner_{};
    guidance::g4::HighGBarrelOwner staged_high_g_owner_{};
    StaticGunSnapshotStagedOwner committed_snapshot_owner_{};
    StaticGunSnapshotStagedOwner staged_snapshot_owner_{};
    guidance::dbfm::DbfmPhaseGradedResponseProvider
        committed_phase_graded_owner_{};
    guidance::dbfm::DbfmPhaseGradedResponseProvider
        staged_phase_graded_owner_{};

    bool staged_ready_ = false;
    ControlFrameIdentity staged_frame_identity_{};
    std::uint32_t staged_writer_id_ = ControlIntentWriterNone;
    std::uint64_t generation_ = 0U;
    std::uint64_t staged_generation_ = 0U;
    StaticImmediateGunResponsePreparedReceipt snapshot_{};
};

static_assert(
    std::is_trivially_copyable<
        StaticImmediateGunResponseStagedInput>::value,
    "Static ImmediateGun response input must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<
        StaticImmediateGunResponsePreparedReceipt>::value,
    "Static ImmediateGun response receipt must remain allocation-free.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::g4::HighGBarrelOwner>::value,
    "High-G owner must support staged copies.");
static_assert(
    std::is_nothrow_copy_assignable<
        StaticGunSnapshotStagedOwner>::value,
    "Snapshot owner must support staged copies.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::dbfm::DbfmPhaseGradedResponseProvider>::value,
    "Phase-graded owner must support staged copies.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
