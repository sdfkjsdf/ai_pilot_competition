#pragma once

#include "LadyLuck/guidance/g10/G10SecondUseLagReacquisitionProvider.hpp"
#include "LadyLuck/guidance/obfm/ObfmLeadDiscipline.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Command-neutral completion status for the production OBFM pursuit
// observation.  FiniteObservationUnavailable is an ordinary completed sample:
// the next frame's G11 gate preserves terminal tracking.
enum class ObfmPursuitClassificationObserverReason : std::uint8_t
{
    NotOwner = 0U,
    FrameEvidenceUnavailable = 1U,
    DeclaredReadySampleContractRejected = 2U,
    OwnerEpochSeeded = 3U,
    FiniteObservationUnavailable = 4U,
    ObservationReady = 5U,
    ProviderContractRejected = 6U,
    ProviderReceiptContradiction = 7U
};

const char* ObfmPursuitClassificationObserverReasonLabel(
    ObfmPursuitClassificationObserverReason reason) noexcept;

struct ObfmPursuitClassificationObserverInput
{
    // True only after an OBFM LAG/FOLLOW/EMPLOY command was actually
    // published.  Other owners reset the causal path-plane history.
    bool pursuit_owner_active = false;
    // False is normal typed absence and starts a fresh causal epoch when
    // evidence returns.  True declares the synchronized frame contract.
    bool frame_evidence_declared_ready = false;
};

struct ObfmPursuitClassificationObserverReceipt
{
    ControlFrameIdentity frame_identity{};
    bool pursuit_owner_active = false;
    bool sample_evaluated = false;
    bool lifecycle_reset = false;
    bool completed_observation_present = false;
    bool behavior_switch_admitted = false;
    bool lift_source_disagreement = false;
    PursuitPathPlaneGate own_path_gate =
        PursuitPathPlaneGate::Unavailable;
    PursuitPathPlaneGate target_path_gate =
        PursuitPathPlaneGate::Unavailable;
    RollingScissorsPlaneRelation plane_relation =
        RollingScissorsPlaneRelation::NotObservable;
    // Store this completed current-frame value and expose it to G11 only on
    // the next frame.  The observer itself never applies terminal tracking.
    ObfmLeadDisciplineInput lead_discipline_input_for_next_frame{};
    // Diagnostic only.  No command is removed when a declared-ready producer
    // contract is rejected or internally contradictory.
    bool producer_contract_contradiction = false;
    ObfmPursuitClassificationObserverReason reason =
        ObfmPursuitClassificationObserverReason::NotOwner;
};

// Exact observation math is reused from the already-ported
// G10SecondUseLagReacquisitionProvider.  This wrapper supplies a continuous
// OBFM owner epoch and discards every G10 maneuver/command-authority result;
// only the behavior-neutral pursuit classifications leave the class.
class ObfmPursuitClassificationObserver final
{
public:
    ObfmPursuitClassificationObserver() noexcept = default;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        double sample_dt_s,
        const ObfmPursuitClassificationObserverInput& input,
        ObfmPursuitClassificationObserverReceipt& output) noexcept;

private:
    guidance::g10::G10SecondUseLagReacquisitionProvider provider_{};
    bool owner_identity_available_ = false;
    std::uint64_t episode_epoch_ = 0U;
    std::int32_t own_plane_id_ = -1;
    std::int32_t target_plane_id_ = -1;
};

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
