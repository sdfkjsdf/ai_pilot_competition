#include "LadyLuck/behavior_tree/static/StaticGunSnapshotStagedOwner.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/geometry/WezRule.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

// Existing production OfficialGun attack-form observer value. It is not a
// new admission threshold.
constexpr double OfficialGunRetentionRollAuthorityRadps = 2.0;

void BuildOfficialGunResponseContract(
    const LadyLuck::DogfightGeometryFrame& frame,
    bool& horizon_available,
    double& horizon_s,
    bool& cone_available,
    double& cone_rad,
    LadyLuck::Status& status) noexcept
{
    horizon_available = false;
    horizon_s = 0.0;
    cone_available = false;
    cone_rad = 0.0;
    status = LadyLuck::Status{};
    if (!std::isfinite(frame.t_sec)
        || !std::isfinite(frame.tau_sec)
        || !std::isfinite(frame.enemy_offense.range_m)
        || !std::isfinite(frame.enemy_offense.ata_rad))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }
    if (frame.t_sec < 0.0
        || frame.tau_sec <= 0.0
        || frame.enemy_offense.range_m <= 0.0)
    {
        // A finite non-positive prediction horizon or separation has no
        // Snapshot solution. The already-admitted base BREAK remains valid.
        return;
    }

    const LadyLuck::Result<LadyLuck::WezPhase> phase =
        LadyLuck::ActiveWezPhase(frame.t_sec);
    if (!phase.ok())
    {
        status = phase.status;
        return;
    }
    if (!std::isfinite(phase.value.angle_rad)
        || phase.value.angle_rad <= 0.0
        || phase.value.angle_rad >= LadyLuck::constants::Pi / 2.0)
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return;
    }

    horizon_available = true;
    horizon_s = frame.tau_sec;
    cone_available = true;
    cone_rad = phase.value.angle_rad;
}

bool BuildOfficialGunSnapshotOverlay(
    const LadyLuck::guidance::prefire::OfficialGunSnapshotReference& reference,
    LadyLuck::guidance::prefire::PrefireSnapshotCommandOverlay& output)
    noexcept
{
    output = LadyLuck::guidance::prefire::PrefireSnapshotCommandOverlay{};
    const double gravity = LadyLuck::constants::StandardGravityMps2;
    const double maximum = (std::numeric_limits<double>::max)();
    if (!reference.valid
        || !std::isfinite(reference.target_load_factor_g)
        || reference.target_load_factor_g < 1.0
        || reference.target_load_factor_g > maximum / gravity)
    {
        return false;
    }

    const double specific_force_magnitude =
        reference.target_load_factor_g * gravity;
    LadyLuck::Vector3 total_acceleration{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double direction =
            reference.target_lift_direction_ned[index];
        const double absolute_direction = std::fabs(direction);
        if (!std::isfinite(direction)
            || (absolute_direction > 1.0
                && specific_force_magnitude
                    > maximum / absolute_direction))
        {
            return false;
        }
        total_acceleration[index] = direction * specific_force_magnitude;
        if (!std::isfinite(total_acceleration[index]))
        {
            return false;
        }
    }
    if (total_acceleration[2] > maximum - gravity)
    {
        return false;
    }
    total_acceleration[2] += gravity;
    if (!std::isfinite(total_acceleration[2]))
    {
        return false;
    }

    output.valid = true;
    output.direct_load_vector_acceleration_ned_mps2 = total_acceleration;
    return true;
}

void ValidateBaseBreak(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::ControlIntent& base_break,
    LadyLuck::Status& status) noexcept
{
    base_break.Validate(status);
    if (!status.sample_valid())
    {
        return;
    }
    if (!LadyLuck::SameControlFrameIdentity(
            frame.frame_identity,
            base_break.frame_identity)
        || base_break.route_kind != LadyLuck::ControlRouteKind::AimPoint
        || base_break.behavior_id
            != LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak
        || base_break.mode_id != LadyLuck::DoctrineModeId::Dbfm
        || base_break.writer_id
            != LadyLuck::ControlIntentWriterGunDefenseHorizontalBreak)
    {
        status.code = LadyLuck::StatusCode::InvalidConfiguration;
    }
}

} // namespace

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

StaticGunSnapshotStagedOwner::StaticGunSnapshotStagedOwner() noexcept
{
    Reset();
}

void StaticGunSnapshotStagedOwner::Reset() noexcept
{
    committed_observer_ = guidance::prefire::GunAttackFormObserver{};
    guidance::prefire::GunAttackFormObserverConfig observer_config{};
    observer_config.retention_authority_radps_valid = true;
    observer_config.retention_authority_radps =
        OfficialGunRetentionRollAuthorityRadps;
    Status observer_status{};
    committed_observer_.Configure(observer_config, observer_status);
    observer_configured_ = observer_status.sample_valid();

    committed_policy_ =
        guidance::prefire::OfficialGunAttackResponsePolicy{};
    committed_policy_.Reset();
    staged_observer_ = committed_observer_;
    staged_policy_ = committed_policy_;
    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
    staged_generation_ = 0U;
    ++generation_;
    snapshot_ = StaticGunSnapshotPreparedReceipt{};
}

void StaticGunSnapshotStagedOwner::DiscardStagedState() noexcept
{
    staged_observer_ = committed_observer_;
    staged_policy_ = committed_policy_;
    staged_ready_ = false;
    staged_frame_identity_ = ControlFrameIdentity{};
    staged_writer_id_ = ControlIntentWriterNone;
    staged_generation_ = 0U;
}

void StaticGunSnapshotStagedOwner::RetainBaseBreak(
    const StaticGunSnapshotDisposition disposition,
    const StaticGunSnapshotReason reason,
    const StatusCode diagnostic_status_code,
    const ControlIntent& base_break,
    ControlIntent& output,
    StaticGunSnapshotPreparedReceipt& receipt,
    Status& status) noexcept
{
    output = base_break;
    staged_writer_id_ = ControlIntentWriterGunDefenseHorizontalBreak;
    snapshot_.disposition = disposition;
    snapshot_.reason = reason;
    snapshot_.prepared_writer_id = staged_writer_id_;
    snapshot_.candidate_count = 1U;
    snapshot_.state_staged = staged_ready_;
    snapshot_.state_committed = false;
    snapshot_.state_aborted = false;
    snapshot_.diagnostic_status_code = diagnostic_status_code;
    receipt = snapshot_;
    status = Status{};
}

void StaticGunSnapshotStagedOwner::Prepare(
    const StaticGunSnapshotStagedInput& input,
    ControlIntent& output,
    StaticGunSnapshotPreparedReceipt& receipt,
    Status& status) noexcept
{
    output.Clear();
    receipt = StaticGunSnapshotPreparedReceipt{};
    status = Status{};
    DiscardStagedState();
    snapshot_ = StaticGunSnapshotPreparedReceipt{};
    snapshot_.prepare_attempted = true;
    snapshot_.frame_identity = input.frame.frame_identity;

    Status base_status{};
    ValidateBaseBreak(input.frame, input.base_break, base_status);
    if (!base_status.sample_valid())
    {
        snapshot_.disposition =
            StaticGunSnapshotDisposition::InputContractFault;
        snapshot_.reason =
            StaticGunSnapshotReason::BaseBreakContractFault;
        snapshot_.diagnostic_status_code = base_status.code;
        receipt = snapshot_;
        status = base_status;
        return;
    }
    snapshot_.base_writer2_same_frame_admitted = true;
    output = input.base_break;

    staged_observer_ = committed_observer_;
    staged_policy_ = committed_policy_;
    staged_ready_ = true;
    staged_frame_identity_ = input.frame.frame_identity;
    staged_writer_id_ = ControlIntentWriterGunDefenseHorizontalBreak;
    staged_generation_ = generation_;
    snapshot_.captured_generation = staged_generation_;
    snapshot_.state_staged = true;

    if (!IsValidControlFrameIdentity(
            input.current_envelope.frame_identity)
        || !SameControlFrameIdentity(
            input.current_envelope.frame_identity,
            input.frame.frame_identity)
        || !std::isfinite(input.current_envelope.nz_feasible_g))
    {
        staged_observer_.Reset();
        staged_policy_.ClearEpisode();
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::FrameOrEnvelopeContractFault,
            std::isfinite(input.current_envelope.nz_feasible_g)
                ? StatusCode::InvalidConfiguration
                : StatusCode::NonFiniteInput,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    if (!observer_configured_)
    {
        staged_observer_.Reset();
        staged_policy_.ClearEpisode();
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::ObserverConfigurationFault,
            StatusCode::InvalidConfiguration,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    Status response_contract_status{};
    BuildOfficialGunResponseContract(
        input.frame,
        snapshot_.response_horizon_available,
        snapshot_.response_horizon_s,
        snapshot_.response_cone_available,
        snapshot_.response_cone_rad,
        response_contract_status);
    if (!response_contract_status.ok())
    {
        staged_observer_.Reset();
        staged_policy_.ClearEpisode();
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::FrameOrEnvelopeContractFault,
            response_contract_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }
    snapshot_.response_contract_ready =
        snapshot_.response_horizon_available
        && snapshot_.response_cone_available;

    const guidance::prefire::GunAttackFormObservation* observation = nullptr;
    if (snapshot_.response_contract_ready)
    {
        snapshot_.observation_attempted = true;
        Status observation_status{};
        staged_observer_.Update(
            input.frame,
            snapshot_.observation,
            observation_status);
        snapshot_.observation_status_code = observation_status.code;
        if (!observation_status.ok()
            && observation_status.code == StatusCode::NonFiniteInput)
        {
            staged_observer_.Reset();
            staged_policy_.ClearEpisode();
            RetainBaseBreak(
                StaticGunSnapshotDisposition::
                    BaseBreakRetainedInternalFault,
                StaticGunSnapshotReason::ObservationInputFault,
                observation_status.code,
                input.base_break,
                output,
                receipt,
            status);
            return;
        }
        if (observation_status.ok()
            && snapshot_.observation.valid
            && snapshot_.observation.geometry.valid)
        {
            snapshot_.observation_ready = true;
            observation = &snapshot_.observation;
        }
        else
        {
            // Finite zero-range/axis/plane singularities are ordinary
            // response non-applicability. They cannot remove writer 2 or be
            // promoted to an internal production fault.
            staged_observer_.Reset();
        }
    }
    else
    {
        staged_observer_.Reset();
    }

    snapshot_.load_limit_available =
        input.current_envelope.nz_feasible_g >= 1.0;
    snapshot_.load_limit_g = snapshot_.load_limit_available
        ? input.current_envelope.nz_feasible_g
        : 0.0;
    if (!snapshot_.load_limit_available)
    {
        // A finite envelope below level-support load cannot realize the
        // Snapshot plane change. This is physical non-applicability, not an
        // evidence-contract fault; preserve the same-frame base BREAK.
        staged_observer_.Reset();
        staged_policy_.ClearEpisode();
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedNotApplicable,
            StaticGunSnapshotReason::ResponseNotApplicable,
            StatusCode::Ok,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    ControlIntent raw_response{};
    Status response_status{};
    staged_policy_.Evaluate(
        input.frame,
        true,
        guidance::prefire::
            OfficialGunAttackResponseProductionActivation,
        observation,
        snapshot_.response_horizon_available,
        snapshot_.response_horizon_s,
        snapshot_.response_cone_available,
        snapshot_.response_cone_rad,
        true,
        snapshot_.load_limit_g,
        nullptr,
        input.base_break,
        snapshot_.response,
        raw_response,
        response_status);
    snapshot_.response_status_code = response_status.code;
    if (!response_status.ok()
        || snapshot_.response.declared_ready_contract_contradiction)
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::PolicyInternalFault,
            response_status.ok()
                ? StatusCode::InvalidConfiguration
                : response_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    if (snapshot_.response.selected_branch
            == guidance::prefire::OfficialGunAttackResponseBranch::
                TrackingJink
        || snapshot_.response.tracking_candidate_count != 0U)
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::TrackingBranchForbidden,
            StatusCode::InvalidConfiguration,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    if (!snapshot_.response.replacement_available)
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedNotApplicable,
            StaticGunSnapshotReason::ResponseNotApplicable,
            StatusCode::Ok,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    if (snapshot_.response.selected_branch
        != guidance::prefire::OfficialGunAttackResponseBranch::
            SnapshotPlaneChange)
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::TrackingBranchForbidden,
            StatusCode::InvalidConfiguration,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    guidance::prefire::PrefireSnapshotCommandOverlay overlay{};
    if (!BuildOfficialGunSnapshotOverlay(
            snapshot_.response.snapshot_reference,
            overlay))
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::SnapshotOverlayContractFault,
            StatusCode::InvalidConfiguration,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }
    snapshot_.snapshot_overlay_ready = true;

    ControlIntent candidate{};
    Status overlay_status{};
    guidance::prefire::ApplyPrefireSnapshotCommandOverlay(
        raw_response,
        overlay,
        DoctrineBehaviorId::OfficialGunSnapshotPlaneChange,
        ControlIntentWriterOfficialGunSnapshotPlaneChange,
        candidate,
        overlay_status);
    if (!overlay_status.ok())
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::SnapshotOverlayContractFault,
            overlay_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    Status candidate_status{};
    candidate.Validate(candidate_status);
    if (!candidate_status.sample_valid()
        || !SameControlFrameIdentity(
            candidate.frame_identity,
            input.frame.frame_identity)
        || candidate.writer_id
            != ControlIntentWriterOfficialGunSnapshotPlaneChange
        || candidate.behavior_id
            != DoctrineBehaviorId::OfficialGunSnapshotPlaneChange
        || candidate.mode_id != DoctrineModeId::Dbfm
        || candidate.route_kind
            != ControlRouteKind::DirectLoadVectorAcceleration)
    {
        RetainBaseBreak(
            StaticGunSnapshotDisposition::
                BaseBreakRetainedInternalFault,
            StaticGunSnapshotReason::SnapshotIntentContractFault,
            candidate_status.sample_valid()
                ? StatusCode::InvalidConfiguration
                : candidate_status.code,
            input.base_break,
            output,
            receipt,
            status);
        return;
    }

    output = candidate;
    staged_writer_id_ = ControlIntentWriterOfficialGunSnapshotPlaneChange;
    snapshot_.disposition =
        StaticGunSnapshotDisposition::SnapshotPrepared;
    snapshot_.reason = StaticGunSnapshotReason::SnapshotPrepared;
    snapshot_.prepared_writer_id = staged_writer_id_;
    snapshot_.candidate_count = 1U;
    snapshot_.state_staged = true;
    snapshot_.state_committed = false;
    snapshot_.state_aborted = false;
    snapshot_.diagnostic_status_code = StatusCode::Ok;
    receipt = snapshot_;
    status = Status{};
}

void StaticGunSnapshotStagedOwner::ValidatePrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) const noexcept
{
    status = Status{};
    const bool published_writer_matches =
        staged_writer_id_ == published_writer_id
        || (staged_writer_id_
                == ControlIntentWriterGunDefenseHorizontalBreak
            && published_writer_id
                == ControlIntentWriterDbfmHardTurn);
    if (!staged_ready_
        || !snapshot_.state_staged
        || snapshot_.state_committed
        || snapshot_.state_aborted
        || snapshot_.candidate_count != 1U
        || staged_generation_ != generation_
        || snapshot_.captured_generation != staged_generation_
        || !IsValidControlFrameIdentity(frame_identity)
        || !SameControlFrameIdentity(
            staged_frame_identity_,
            frame_identity)
        || !SameControlFrameIdentity(
            snapshot_.frame_identity,
            frame_identity)
        || !published_writer_matches
        || snapshot_.prepared_writer_id != staged_writer_id_
        || (published_writer_id
                != ControlIntentWriterGunDefenseHorizontalBreak
            && published_writer_id
                != ControlIntentWriterOfficialGunSnapshotPlaneChange
            && published_writer_id != ControlIntentWriterDbfmHardTurn))
    {
        status.code = StatusCode::InvalidConfiguration;
    }
}

void StaticGunSnapshotStagedOwner::CommitPrepared(
    const ControlFrameIdentity& frame_identity,
    const std::uint32_t published_writer_id,
    Status& status) noexcept
{
    ValidatePrepared(frame_identity, published_writer_id, status);
    if (!status.ok())
    {
        return;
    }
    committed_observer_ = staged_observer_;
    committed_policy_ = staged_policy_;
    snapshot_.state_committed = true;
    snapshot_.state_aborted = false;
    snapshot_.diagnostic_status_code =
        snapshot_.disposition
                == StaticGunSnapshotDisposition::
                    BaseBreakRetainedInternalFault
            ? snapshot_.diagnostic_status_code
            : StatusCode::Ok;
    ++generation_;
    DiscardStagedState();
}

void StaticGunSnapshotStagedOwner::AbortPrepared() noexcept
{
    if (staged_ready_)
    {
        snapshot_.state_committed = false;
        snapshot_.state_aborted = true;
    }
    DiscardStagedState();
}

void StaticGunSnapshotStagedOwner::CopySnapshot(
    StaticGunSnapshotPreparedReceipt& output) const noexcept
{
    output = snapshot_;
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
