#pragma once

#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/guidance/prefire/OfficialGunAttackResponse.hpp"
#include "LadyLuck/guidance/prefire/RootPrefireResponseSelection.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class StaticGunSnapshotDisposition : std::uint8_t
{
    NotEvaluated = 0U,
    BaseBreakRetainedNotApplicable = 1U,
    SnapshotPrepared = 2U,
    BaseBreakRetainedInternalFault = 3U,
    InputContractFault = 4U
};

enum class StaticGunSnapshotReason : std::uint8_t
{
    NotEvaluated = 0U,
    ResponseNotApplicable = 1U,
    SnapshotPrepared = 2U,
    BaseBreakContractFault = 3U,
    FrameOrEnvelopeContractFault = 4U,
    ObserverConfigurationFault = 5U,
    PolicyInternalFault = 6U,
    TrackingBranchForbidden = 7U,
    SnapshotOverlayContractFault = 8U,
    SnapshotIntentContractFault = 9U,
    ObservationInputFault = 10U
};

// Fixed same-frame inputs. The caller supplies the already-selected and
// validated base Gun BREAK that proves writer-2 admission, plus the current
// command envelope used later by Route-5/FCS. This owner performs no second
// threat/capability admission.
struct StaticGunSnapshotStagedInput
{
    DogfightGeometryFrame frame{};
    control::route5::CommandEnvelope current_envelope{};
    ControlIntent base_break{};
};

// Raw-guidance transaction receipt only. The selected intent has not passed
// Route-5/FCS/wire validation and is not evidence of aircraft response.
struct StaticGunSnapshotPreparedReceipt
{
    bool prepare_attempted = false;
    ControlFrameIdentity frame_identity{};
    bool base_writer2_same_frame_admitted = false;
    bool response_horizon_available = false;
    double response_horizon_s = 0.0;
    bool response_cone_available = false;
    double response_cone_rad = 0.0;
    bool response_contract_ready = false;
    bool observation_attempted = false;
    bool observation_ready = false;
    guidance::prefire::GunAttackFormObservation observation{};
    StatusCode observation_status_code = StatusCode::Ok;
    bool load_limit_available = false;
    double load_limit_g = 0.0;
    guidance::prefire::OfficialGunAttackResponseReceipt response{};
    StatusCode response_status_code = StatusCode::Ok;
    bool snapshot_overlay_ready = false;
    StaticGunSnapshotDisposition disposition =
        StaticGunSnapshotDisposition::NotEvaluated;
    StaticGunSnapshotReason reason =
        StaticGunSnapshotReason::NotEvaluated;
    std::uint32_t prepared_writer_id = ControlIntentWriterNone;
    std::uint8_t candidate_count = 0U;
    bool state_staged = false;
    bool state_committed = false;
    bool state_aborted = false;
    std::uint64_t captured_generation = 0U;
    StatusCode diagnostic_status_code = StatusCode::Ok;
};

// Optional production Snapshot Plane Change response for an already-selected
// base Gun BREAK. Tracking Jink is excluded by the frozen production
// activation and by the explicit branch contract in Prepare().
class StaticGunSnapshotStagedOwner final
{
public:
    StaticGunSnapshotStagedOwner() noexcept;

    void Reset() noexcept;

    void Prepare(
        const StaticGunSnapshotStagedInput& input,
        ControlIntent& output,
        StaticGunSnapshotPreparedReceipt& receipt,
        Status& status) noexcept;

    // A retained writer2 may be finalized unchanged or by the next fixed
    // side-preserving HardTurn response (writer3). A prepared Snapshot itself must
    // still finalize as writer27. This keeps observer/policy state staged until
    // the outer post-FCS transaction selects its one final writer.
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
        StaticGunSnapshotPreparedReceipt& output) const noexcept;

private:
    void DiscardStagedState() noexcept;
    void RetainBaseBreak(
        StaticGunSnapshotDisposition disposition,
        StaticGunSnapshotReason reason,
        StatusCode diagnostic_status_code,
        const ControlIntent& base_break,
        ControlIntent& output,
        StaticGunSnapshotPreparedReceipt& receipt,
        Status& status) noexcept;

    guidance::prefire::GunAttackFormObserver committed_observer_{};
    guidance::prefire::GunAttackFormObserver staged_observer_{};
    guidance::prefire::OfficialGunAttackResponsePolicy committed_policy_{};
    guidance::prefire::OfficialGunAttackResponsePolicy staged_policy_{};
    bool observer_configured_ = false;
    bool staged_ready_ = false;
    ControlFrameIdentity staged_frame_identity_{};
    std::uint32_t staged_writer_id_ = ControlIntentWriterNone;
    std::uint64_t generation_ = 0U;
    std::uint64_t staged_generation_ = 0U;
    StaticGunSnapshotPreparedReceipt snapshot_{};
};

static_assert(
    !guidance::prefire::OfficialGunAttackResponseProductionActivation
        .tracking_jink_enabled,
    "Production Static Gun Snapshot owner must exclude Tracking writer 26.");
static_assert(
    std::is_trivially_copyable<StaticGunSnapshotStagedInput>::value,
    "Static Gun Snapshot input must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<StaticGunSnapshotPreparedReceipt>::value,
    "Static Gun Snapshot receipt must remain allocation-free.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::prefire::GunAttackFormObserver>::value,
    "Gun attack-form observer must support staged copies.");
static_assert(
    std::is_nothrow_copy_assignable<
        guidance::prefire::OfficialGunAttackResponsePolicy>::value,
    "Official Gun policy must support staged copies.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
