#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseOwner.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseSelectionSupply.hpp"
#include "LadyLuck/guidance/obfm/PursuitPlaneSeparationObserver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

// Command-neutral manual pursuit classification.  This is measured aircraft
// state; it is not a body-rate, load-factor, surface, thrust, or truth receipt.
enum class G10PursuitState : std::uint8_t
{
    Lag = 0U,
    Pure = 1U,
    Lead = 2U,
    NotObservable = 3U
};

enum class G10PursuitClassificationReason : std::uint8_t
{
    NotObserved = 0U,
    NoseOrTargetTangentZeroMagnitude = 1U,
    SampleOrObservationResolutionInvalid = 2U,
    NoseRayNearParallelToTargetPath = 3U,
    CrossingNotProvedAheadOfAttacker = 4U,
    OffsetBeyondTargetPathLocality = 5U,
    OffsetIntervalStraddlesPureBand = 6U,
    NoseRayPathCrossingAdmitted = 7U,
    AttitudeNoseOrTargetDirectionUnavailable = 8U,
    FlightLiftOrTargetTangentZeroMagnitude = 9U,
    FlightAndLiftDoNotResolveManeuverPlane = 10U,
    OrientedMeridianLiftAxisNotResolved = 11U,
    TargetTangentPlaneCrossingNotResolved = 12U,
    TargetCrossingNotProvenOnPositiveLiftSide = 13U,
    TargetCrossingNotProvenAheadOfAttacker = 14U,
    TargetTangentExtrapolationOutsideLocalityDomain = 15U,
    TargetPathStateBoundaryNotResolved = 16U,
    OrientedLiftMeridianClassified = 17U,
    EstablishedPathLiftAxisOrTargetDirectionUnavailable = 18U,
    AttitudeLiftAxisOrTargetDirectionUnavailable = 19U,
    ManualLiftCriterionRequiresResolvedPlaneSeparation = 20U
};

const char* G10PursuitClassificationReasonLabel(
    G10PursuitClassificationReason reason) noexcept;

struct G10OptionalInterval
{
    bool has_value = false;
    double lower = 0.0;
    double upper = 0.0;
};

struct G10PursuitClassificationReceipt
{
    bool valid = false;
    G10PursuitClassificationReason reason =
        G10PursuitClassificationReason::NotObserved;
    G10PursuitState state = G10PursuitState::NotObservable;
    G10OptionalDouble target_path_offset_m{};
    G10OptionalInterval target_path_offset_interval_m{};
    G10OptionalDouble forward_distance_m{};
    G10OptionalInterval forward_distance_interval_m{};
    G10OptionalDouble lift_side_distance_m{};
    G10OptionalInterval lift_side_distance_interval_m{};
    G10OptionalDouble target_path_locality_bound_m{};
    G10OptionalDouble pure_sampling_resolution_m{};
};

enum class G10PursuitSwitchReason : std::uint8_t
{
    PlaneSeparationNotResolved = 0U,
    LiftProxySourcesDisagree = 1U,
    ResolvedClassificationInvalid = 2U,
    ClassificationTrustworthyNoManualBoundaryClaim = 3U
};

const char* G10PursuitSwitchReasonLabel(
    G10PursuitSwitchReason reason) noexcept;

enum class G10LagReacquisitionReason : std::uint8_t
{
    NotUpdated = 0U,
    DescendingLagOwnerInactive = 1U,
    DescendingLagOwnerTransitionSeeded = 2U,
    DescendingLagCommandNotAppliedBeforeState = 3U,
    ManeuverPlaneUnresolvedNotObservable = 4U,
    SimilarPlaneLiteralNoseUnresolved = 5U,
    SimilarPlaneLiteralNoseLag = 6U,
    SimilarPlaneLiteralNosePure = 7U,
    SimilarPlaneLiteralNoseLead = 8U,
    SeparatedPlaneUnresolved = 9U,
    SeparatedPlaneDirectedLiftMeridianUnresolved = 10U,
    SeparatedPlaneDirectedLiftMeridianLag = 11U,
    SeparatedPlaneDirectedLiftMeridianPure = 12U,
    SeparatedPlaneDirectedLiftMeridianLead = 13U,
    ContractRejected = 14U
};

const char* G10LagReacquisitionReasonLabel(
    G10LagReacquisitionReason reason) noexcept;

struct G10SecondUseLagReacquisitionInput
{
    bool owner_selected = false;
    G10SecondUseOwnerPhase owner_phase = G10SecondUseOwnerPhase::Idle;
    // True only when the completed previous-tick G10 command was the
    // descending-Lag command and was actually published before this state.
    bool descending_lag_command_applied_before_state = false;
};

struct G10SecondUseLagReacquisitionReceipt
{
    bool valid = false;
    bool evaluated = false;
    ControlFrameIdentity frame_identity{};
    G10LagReacquisitionReason reason =
        G10LagReacquisitionReason::NotUpdated;
    bool descending_lag_owner_active = false;
    bool pursuit_epoch_reset = false;
    bool descending_lag_command_applied_before_state = false;
    obfm::PursuitPathPlaneGate own_path_gate =
        obfm::PursuitPathPlaneGate::Unavailable;
    obfm::PursuitPathPlaneGate target_path_gate =
        obfm::PursuitPathPlaneGate::Unavailable;
    obfm::RollingScissorsPlaneRelation plane_relation =
        obfm::RollingScissorsPlaneRelation::NotObservable;
    G10OptionalDouble plane_separation_rad{};
    G10OptionalDouble plane_separation_resolution_bound_rad{};
    G10PursuitClassificationReceipt nose_ray_classification{};
    G10PursuitClassificationReceipt path_lift_classification{};
    G10PursuitClassificationReceipt attitude_lift_classification{};
    G10PursuitClassificationReceipt
        resolved_separated_plane_classification{};
    bool lift_source_disagreement = false;
    bool behavior_switch_admitted = false;
    G10PursuitSwitchReason behavior_switch_reason =
        G10PursuitSwitchReason::PlaneSeparationNotResolved;
    G10OptionalBool lag_reacquired{};
    bool behavior_authority = false;
    bool tactical_command_authority = false;
    bool flight_control_authority = false;
    bool production_authority = false;
};

class G10SecondUseLagReacquisitionProvider final
{
public:
    G10SecondUseLagReacquisitionProvider() noexcept = default;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        double sample_dt_s,
        const G10SecondUseLagReacquisitionInput& input,
        G10SecondUseLagReacquisitionReceipt& output,
        Status& status) noexcept;

private:
    struct PathHistory
    {
        std::array<double, 3U> times_s{};
        std::array<Vector3, 3U> directions_ned{};
        std::array<double, 3U> direction_bounds_rad{};
        std::size_t count = 0U;
    };

    void ResetPursuitHistory() noexcept;

    PathHistory own_path_{};
    PathHistory target_path_{};
    bool descending_owner_active_ = false;
};

static_assert(
    std::is_trivially_copyable<
        G10SecondUseLagReacquisitionInput>::value,
    "G10 lag-reacquisition input must remain allocation-free");
static_assert(
    std::is_trivially_copyable<
        G10SecondUseLagReacquisitionReceipt>::value,
    "G10 lag-reacquisition receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<
        G10SecondUseLagReacquisitionProvider>::value,
    "G10 lag-reacquisition provider must remain allocation-free");

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
