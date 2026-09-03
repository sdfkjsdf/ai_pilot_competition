#include "LadyLuck/guidance/obfm/ObfmPursuitClassificationObserver.hpp"

#include <cmath>

namespace
{

using LadyLuck::DogfightGeometryFrame;
using LadyLuck::guidance::g10::G10PursuitClassificationReceipt;
using LadyLuck::guidance::g10::G10PursuitState;
using LadyLuck::guidance::obfm::ObfmLeadDisciplineClassification;
using LadyLuck::guidance::obfm::ObfmLeadDisciplinePursuitState;

ObfmLeadDisciplinePursuitState MapState(
    const G10PursuitState state) noexcept
{
    switch (state)
    {
    case G10PursuitState::Lag:
        return ObfmLeadDisciplinePursuitState::Lag;
    case G10PursuitState::Pure:
        return ObfmLeadDisciplinePursuitState::Pure;
    case G10PursuitState::Lead:
        return ObfmLeadDisciplinePursuitState::Lead;
    case G10PursuitState::NotObservable:
        return ObfmLeadDisciplinePursuitState::NotObservable;
    }
    return ObfmLeadDisciplinePursuitState::NotObservable;
}

ObfmLeadDisciplineClassification MapClassification(
    const G10PursuitClassificationReceipt& source) noexcept
{
    ObfmLeadDisciplineClassification output{};
    output.valid = source.valid;
    output.state = MapState(source.state);
    return output;
}

bool ClassifiedState(
    const ObfmLeadDisciplineClassification& value) noexcept
{
    if (!value.valid)
    {
        return false;
    }
    switch (value.state)
    {
    case ObfmLeadDisciplinePursuitState::Lag:
    case ObfmLeadDisciplinePursuitState::Pure:
    case ObfmLeadDisciplinePursuitState::Lead:
        return true;
    case ObfmLeadDisciplinePursuitState::NotObservable:
        return false;
    }
    return false;
}

bool SynchronizedSampleContract(
    const DogfightGeometryFrame& frame,
    const double sample_dt_s) noexcept
{
    return IsValidControlFrameIdentity(frame.frame_identity)
        && std::isfinite(sample_dt_s)
        && sample_dt_s > 0.0
        && std::isfinite(frame.t_sec)
        && frame.t_sec >= 0.0
        && frame.target_same_index
        && frame.target_frame_index == frame.frame_identity.frame_index;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* ObfmPursuitClassificationObserverReasonLabel(
    const ObfmPursuitClassificationObserverReason reason) noexcept
{
    switch (reason)
    {
    case ObfmPursuitClassificationObserverReason::NotOwner:
        return "not_owner";
    case ObfmPursuitClassificationObserverReason::FrameEvidenceUnavailable:
        return "frame_evidence_unavailable";
    case ObfmPursuitClassificationObserverReason::
            DeclaredReadySampleContractRejected:
        return "declared_ready_sample_contract_rejected";
    case ObfmPursuitClassificationObserverReason::OwnerEpochSeeded:
        return "owner_epoch_seeded";
    case ObfmPursuitClassificationObserverReason::
            FiniteObservationUnavailable:
        return "finite_observation_unavailable";
    case ObfmPursuitClassificationObserverReason::ObservationReady:
        return "observation_ready";
    case ObfmPursuitClassificationObserverReason::ProviderContractRejected:
        return "provider_contract_rejected";
    case ObfmPursuitClassificationObserverReason::
            ProviderReceiptContradiction:
        return "provider_receipt_contradiction";
    }
    return "unknown";
}

void ObfmPursuitClassificationObserver::Reset() noexcept
{
    provider_.Reset();
    owner_identity_available_ = false;
    episode_epoch_ = 0U;
    own_plane_id_ = -1;
    target_plane_id_ = -1;
}

void ObfmPursuitClassificationObserver::Update(
    const DogfightGeometryFrame& frame,
    const double sample_dt_s,
    const ObfmPursuitClassificationObserverInput& input,
    ObfmPursuitClassificationObserverReceipt& output) noexcept
{
    output = ObfmPursuitClassificationObserverReceipt{};
    output.frame_identity = frame.frame_identity;
    output.pursuit_owner_active = input.pursuit_owner_active;

    // A non-owner frame never inspects pursuit evidence.  Reset prevents a
    // later owner from inheriting stale maneuver-plane curvature.
    if (!input.pursuit_owner_active)
    {
        output.lifecycle_reset = owner_identity_available_;
        Reset();
        output.reason = ObfmPursuitClassificationObserverReason::NotOwner;
        return;
    }
    if (!input.frame_evidence_declared_ready)
    {
        output.lifecycle_reset = owner_identity_available_;
        Reset();
        output.reason = ObfmPursuitClassificationObserverReason::
            FrameEvidenceUnavailable;
        return;
    }
    if (!SynchronizedSampleContract(frame, sample_dt_s))
    {
        Reset();
        output.lifecycle_reset = true;
        output.producer_contract_contradiction = true;
        output.reason = ObfmPursuitClassificationObserverReason::
            DeclaredReadySampleContractRejected;
        return;
    }

    const bool identity_changed = !owner_identity_available_
        || episode_epoch_ != frame.frame_identity.episode_epoch
        || own_plane_id_ != frame.own_plane_id
        || target_plane_id_ != frame.target_plane_id;
    if (identity_changed)
    {
        provider_.Reset();
        owner_identity_available_ = true;
        episode_epoch_ = frame.frame_identity.episode_epoch;
        own_plane_id_ = frame.own_plane_id;
        target_plane_id_ = frame.target_plane_id;
        output.lifecycle_reset = true;
    }

    guidance::g10::G10SecondUseLagReacquisitionInput provider_input{};
    provider_input.owner_selected = true;
    provider_input.owner_phase = guidance::g10::
        G10SecondUseOwnerPhase::DescendingLagReacquire;
    // The G10 post-command projection is intentionally unreachable here.
    // All pursuit observation fields are completed before this flag is read.
    provider_input.descending_lag_command_applied_before_state = false;
    guidance::g10::G10SecondUseLagReacquisitionReceipt provider_output{};
    Status provider_status{};
    provider_.Update(
        frame,
        sample_dt_s,
        provider_input,
        provider_output,
        provider_status);
    output.sample_evaluated = true;
    output.lifecycle_reset = output.lifecycle_reset
        || provider_output.pursuit_epoch_reset;
    if (!provider_status.ok() || !provider_output.valid)
    {
        Reset();
        output.lifecycle_reset = true;
        output.producer_contract_contradiction = true;
        output.reason = ObfmPursuitClassificationObserverReason::
            ProviderContractRejected;
        return;
    }

    output.completed_observation_present = true;
    output.behavior_switch_admitted =
        provider_output.behavior_switch_admitted;
    output.lift_source_disagreement =
        provider_output.lift_source_disagreement;
    output.own_path_gate = provider_output.own_path_gate;
    output.target_path_gate = provider_output.target_path_gate;
    output.plane_relation = provider_output.plane_relation;
    output.lead_discipline_input_for_next_frame.
        previous_observation_present = true;
    output.lead_discipline_input_for_next_frame.
        behavior_switch_admitted = provider_output.behavior_switch_admitted;
    output.lead_discipline_input_for_next_frame.lift_rule =
        MapClassification(
            provider_output.resolved_separated_plane_classification);
    output.lead_discipline_input_for_next_frame.nose_rule =
        MapClassification(provider_output.nose_ray_classification);

    const bool contradictory = provider_output.behavior_switch_admitted
        && (!ClassifiedState(
                output.lead_discipline_input_for_next_frame.lift_rule)
            || !ClassifiedState(
                output.lead_discipline_input_for_next_frame.nose_rule));
    if (contradictory)
    {
        output.producer_contract_contradiction = true;
        output.reason = ObfmPursuitClassificationObserverReason::
            ProviderReceiptContradiction;
        return;
    }
    if (output.lifecycle_reset)
    {
        output.reason =
            ObfmPursuitClassificationObserverReason::OwnerEpochSeeded;
        return;
    }
    output.reason = provider_output.behavior_switch_admitted
        ? ObfmPursuitClassificationObserverReason::ObservationReady
        : ObfmPursuitClassificationObserverReason::
            FiniteObservationUnavailable;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
