#pragma once

#include "LadyLuck/geometry/WezRule.hpp"
#include "LadyLuck/guidance/prefire/RootGunTowardSideObservation.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

struct PrefireOptionalDouble
{
    bool has_value = false;
    double value = 0.0;
};

enum class RootPrefireMarginReason : std::uint8_t
{
    CapabilityEvidenceNotAdmitted = 0U,
    OfficialRangeEntryNotProven = 1U,
    ConeSweepMemoryNotEstablished = 2U,
    AttackerConeEntryNotProven = 3U,
    SolutionEntryBeforeOwnTurn = 4U,
    OwnTurnCompletesBeforeSolutionEntry = 5U
};

const char* RootPrefireMarginReasonLabel(
    RootPrefireMarginReason reason) noexcept;

struct RootPrefireBreakMarginSample
{
    bool admitted = false;
    bool margin_break = false;
    double scoring_gap_m = 0.0;
    PrefireOptionalDouble time_to_score_s{};
    PrefireOptionalDouble time_to_face_s{};
    RootPrefireMarginReason reason =
        RootPrefireMarginReason::CapabilityEvidenceNotAdmitted;
};

struct RootPrefirePhaseObservation
{
    WezPhase phase{};
    bool range_satisfied = false;
    bool range_closing_proven = false;
    bool cone_satisfied = false;
    bool cone_closing_proven = false;
    PrefireOptionalDouble time_to_range_s{};
    PrefireOptionalDouble time_to_cone_s{};
    PrefireOptionalDouble time_to_solution_s{};
    RootPrefireBreakMarginSample margin{};
};

enum class RootPrefireThreatShadowReason : std::uint8_t
{
    OfficialGunThreatAlreadyActive = 0U,
    NoElapsedNonScratchPhase = 1U,
    CapabilityEvidenceNotAdmitted = 2U,
    NoPrefireBreakCandidate = 3U,
    PrefireBreakCandidateObserved = 4U,
    PrefireThreatObserverContractRejected = 5U,
    FiniteKinematicsUnavailable = 6U
};

const char* RootPrefireThreatShadowReasonLabel(
    RootPrefireThreatShadowReason reason) noexcept;

constexpr std::size_t RootPrefireNonScratchPhaseCount = 2U;

struct RootPrefireThreatShadowReceipt
{
    bool evaluated = false;
    bool admitted = false;
    bool prefire_break_candidate = false;
    RootPrefireThreatShadowReason reason =
        RootPrefireThreatShadowReason::PrefireThreatObserverContractRejected;
    bool candidate_phase_valid = false;
    WezPhaseId candidate_phase = WezPhaseId::P1;
    std::array<RootPrefirePhaseObservation,
        RootPrefireNonScratchPhaseCount> phase_observations{};
    std::size_t phase_observation_count = 0U;

    bool root_owner_authority = false;
    bool break_side_authority = false;
    bool tactical_reference_authority = false;
    bool tactical_command_authority = false;
    bool production_authority = false;
    std::uint32_t formal_credit = 0U;
};

// Stateful port of RootPrefireThreatObserver. State is only the previous
// admitted (time, attacker ATA) pair, and it is updated before the official-
// threat early return exactly as in d90.  The production BT additionally maps
// finite zero-separation/zero-speed singularities to an authority-free typed
// non-observation so a command-neutral observer cannot suppress Root fallback.
class RootPrefireThreatObserver final
{
public:
    RootPrefireThreatObserver() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        const PrefireOptionalDouble& capability_n_max_g,
        bool capability_n_max_admitted,
        RootPrefireThreatShadowReceipt& output,
        Status& status) noexcept;

private:
    bool previous_valid_ = false;
    double previous_t_s_ = 0.0;
    double previous_ata_rad_ = 0.0;
};

// Exact fields read by `_prefire_break_control_response_admission`. The shared
// runtime maps its completed age-1 safety/control evidence into this object.
struct RootPrefireControlPathEvidence
{
    bool safety_observation_available = false;
    runtime::TacticalFeedbackFreshness feedback_freshness =
        runtime::TacticalFeedbackFreshness::Missing;
    bool previous_control_feedback_available = false;
    runtime::TacticalControlBackendId command_backend_id =
        runtime::TacticalControlBackendId::Unavailable;
};

enum class RootPrefireControlAdmissionReason : std::uint8_t
{
    PrefireControlFeedbackUnavailable = 0U,
    PrefireControlFeedbackNotFresh = 1U,
    PrefireControlBackendUntrusted = 2U,
    PrefireControlPathAdmitted = 3U
};

const char* RootPrefireControlAdmissionReasonLabel(
    RootPrefireControlAdmissionReason reason) noexcept;

enum class RootPrefireThreatConsumerReason : std::uint8_t
{
    NoPrefireBreakDemand = 0U,
    CheckExtendRelease = 1U,
    PrefireMarginCleared = 2U,
    PrefireControlFeedbackUnavailable = 3U,
    PrefireControlFeedbackNotFresh = 4U,
    PrefireControlBackendUntrusted = 5U,
    PrefireEntrySideNotAdmitted = 6U,
    PrefireBreakEnteredControlPathAdmitted = 7U,
    OfficialThreatActive = 8U,
    PrefireBreakContinued = 9U,
    CheckExtendObservationRejected = 10U,
    CheckExtendHold = 11U,
    PrefireCandidateNotRearmed = 12U,
    PrefireThreatConsumerContractRejected = 13U
};

const char* RootPrefireThreatConsumerReasonLabel(
    RootPrefireThreatConsumerReason reason) noexcept;

struct RootPrefireThreatConsumerReceipt
{
    bool active = false;
    bool entered = false;
    bool cleared = false;
    bool official_threat = false;
    bool prefire_candidate = false;
    bool official_threat_seen = false;
    bool check_extend_hold = false;
    RootPrefireThreatConsumerReason reason =
        RootPrefireThreatConsumerReason::NoPrefireBreakDemand;
};

// The side observation is returned separately because Python's consumer
// receipt does not own side fields. It lets the explicit Root Gun entry Task
// pass the same resolved side into GunDefensePolicy without a second formula.
struct RootPrefireThreatConsumerDecision
{
    RootPrefireThreatConsumerReceipt receipt{};
    bool entry_side_observation_attempted = false;
    RootGunTowardSideShadowReceipt entry_side_observation{};
};

class RootPrefireThreatConsumer final
{
public:
    RootPrefireThreatConsumer() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        const RootPrefireThreatShadowReceipt& shadow,
        const SameIndexGeometryFrameEnvelope* envelope,
        bool official_threat,
        const RootPrefireControlPathEvidence& control_evidence,
        RootPrefireThreatConsumerDecision& output,
        Status& status) noexcept;

private:
    bool break_active_ = false;
    bool official_seen_ = false;
    bool check_extend_hold_ = false;
    bool entry_armed_ = true;
};

static_assert(
    std::is_trivially_copyable<RootPrefireThreatShadowReceipt>::value,
    "prefire shadow receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<RootPrefireThreatConsumerDecision>::value,
    "prefire consumer decision must remain allocation-free");

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
