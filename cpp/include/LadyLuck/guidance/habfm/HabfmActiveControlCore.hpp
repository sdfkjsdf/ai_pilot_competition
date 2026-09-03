#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"
#include "LadyLuck/guidance/habfm/HabfmVerticalRoom.hpp"

#include <cstdint>

namespace LadyLuck
{
enum class HabfmCircleProfile : std::uint8_t
{
    OneCircle = 0U,
    TwoCircle = 1U
};

enum class HabfmMergeEnergyState : std::uint8_t
{
    AdvantageProven = 0U,
    DeficitProven = 1U,
    Indistinguishable = 2U
};

struct HabfmMergeProfileSelection
{
    HabfmCircleProfile profile = HabfmCircleProfile::TwoCircle;
    double delta_specific_energy_m = 0.0;
    double evidence_band_m = 0.0;
    double delta_speed_mps = 0.0;
    double speed_band_mps = 0.0;
    HabfmMergeEnergyState energy_state =
        HabfmMergeEnergyState::Indistinguishable;
};

enum class HabfmSelectorTransitionReason : std::uint8_t
{
    SingleUncontradictedAxis = 0U,
    ConflictLatchHeld = 1U,
    UnprovenLatchHeld = 2U
};

struct HabfmSelectorTransitionGuardReceipt
{
    HabfmCircleProfile proposed_profile = HabfmCircleProfile::TwoCircle;
    HabfmCircleProfile latched_profile = HabfmCircleProfile::TwoCircle;
    HabfmCircleProfile resolved_profile = HabfmCircleProfile::TwoCircle;
    bool energy_advantage_proven = false;
    bool speed_deficit_proven = false;
    bool transition_allowed = false;
    HabfmSelectorTransitionReason reason =
        HabfmSelectorTransitionReason::UnprovenLatchHeld;
};

enum class HabfmLeadTurnReason : std::uint8_t
{
    OpeningGeometryNoPendingMerge = 0U,
    TurnRateEvidenceAbsentHoldApproach = 1U,
    LosRateReachedAvailableTurnRate = 2U,
    LosRateBelowAvailableTurnRateHoldApproach = 3U
};

struct HabfmLeadTurnEvidence
{
    double los_rate_radps = 0.0;
    double range_rate_mps = 0.0;
    bool closing = false;
    HabfmOptionalScalar available_turn_rate_radps{};
    bool evidence_admitted = false;
    bool initiate = true;
    HabfmLeadTurnReason reason =
        HabfmLeadTurnReason::TurnRateEvidenceAbsentHoldApproach;

    bool hold_approach() const noexcept
    {
        return !initiate;
    }
};

struct HabfmSpeedFloorSupply
{
    bool admitted = false;
    double floor_speed_mps = 0.0;
};

struct HabfmSeparationCompressionSupply
{
    bool admitted = false;
    bool compress = false;
};

struct HabfmFrontalPassSupply
{
    bool admitted = false;
    double safe_abeam_m = 0.0;
    double compressed_abeam_m = 0.0;
    std::int32_t side_sign = 0;
};

struct HabfmActiveCoreInputs
{
    HabfmOptionalScalar capability_n_max_g{};
    bool capability_n_max_admitted = false;
    HabfmMergeIntentEvidence merge_intent{};
    HabfmSpeedFloorSupply merge_speed_floor{};
    HabfmSeparationCompressionSupply merge_separation_policy{};
    HabfmFrontalPassSupply frontal_pass{};
    HabfmVerticalRoomReceipt merge_vertical_room{};
    bool far_flee_approach_enabled = false;
};

enum class HabfmActiveBranch : std::uint8_t
{
    None = 0U,
    MergeApproach = 1U,
    EnergyFight = 2U,
    OneCircle = 3U,
    TwoCircle = 4U
};

enum class HabfmActiveCoreLegStatus : std::uint8_t
{
    Running = 0U,
    MergePass = 1U
};

enum class HabfmCommandGeometryReason : std::uint8_t
{
    Available = 0U,
    OwnSpeedUnobservable = 1U,
    OwnHorizontalCourseUnobservable = 2U,
    OwnHorizontalNoseUnobservable = 3U,
    HorizontalLineOfSightUnobservable = 4U,
    WeaponRangeUnavailable = 5U
};

struct HabfmCommandGeometryReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool available = false;
    bool three_dimensional_merge_required = false;
    HabfmCommandGeometryReason reason =
        HabfmCommandGeometryReason::OwnSpeedUnobservable;
    double own_speed_mps = 0.0;
    double own_horizontal_speed_mps = 0.0;
    double own_horizontal_nose_norm = 0.0;
    double horizontal_line_of_sight_m = 0.0;
};

enum class HabfmFarFleeApproachReason : std::uint8_t
{
    FeatureDisabled = 0U,
    EvidenceUnavailable = 1U,
    InsideOrAtOfficialReach = 2U,
    OutsideButNotOpening = 3U,
    OutsideOpeningArmed = 4U,
    OutsideLatchHeld = 5U
};

struct HabfmFarFleeApproachState
{
    bool latched = false;
    HabfmOptionalScalar anchor_altitude_m{};
};

struct HabfmFarFleeApproachReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool evidence_available = false;
    bool selected = false;
    bool armed = false;
    bool released = false;
    HabfmFarFleeApproachReason reason =
        HabfmFarFleeApproachReason::FeatureDisabled;
    double separation_m = 0.0;
    double official_reach_boundary_m = 0.0;
    HabfmOptionalScalar range_rate_mps{};
    HabfmFarFleeApproachState resolved_state{};
};

struct HabfmActiveControlOutput
{
    HabfmActiveCoreLegStatus leg_status = HabfmActiveCoreLegStatus::Running;
    HabfmActiveBranch branch = HabfmActiveBranch::None;
    bool intent_present = false;
    ControlIntent intent{};
    HabfmLeadTurnEvidence lead_turn{};
    HabfmFarFleeApproachReceipt far_flee_approach{};
    IntentOptionalValue<HabfmMergeProfileSelection> profile_selection{};
    IntentOptionalValue<HabfmSelectorTransitionGuardReceipt> transition_guard{};
    IntentOptionalValue<HabfmCircleProfile> selected_profile{};
    IntentOptionalValue<HabfmCheckpointCueEvidence> checkpoint_cue{};
    IntentOptionalValue<std::int32_t> turn_side_sign{};
    double turn_progress_rad = 0.0;
    std::uint64_t neutral_cue_streak = 0U;
    bool merge_pass = false;
    bool mode_recheck = false;
};

struct HabfmActiveControlCoreSnapshot
{
    HabfmActiveBranch active_branch = HabfmActiveBranch::None;
    IntentOptionalValue<HabfmMergeProfileSelection> leg_selection{};
    IntentOptionalValue<HabfmCircleProfile> guard_latched_profile{};
    IntentOptionalValue<std::int32_t> side_sign{};
    IntentOptionalValue<double> previous_heading_rad{};
    double progress_rad = 0.0;
    IntentOptionalValue<bool> previous_closing{};
    HabfmFarFleeApproachState far_flee_approach{};
};

void EvaluateHabfmCommandGeometry(
    const DogfightGeometryFrame& frame,
    HabfmCommandGeometryReceipt& output,
    Status& status) noexcept;

void EvaluateHabfmFarFleeApproach(
    const DogfightGeometryFrame& frame,
    bool feature_enabled,
    const HabfmFarFleeApproachState& previous_state,
    HabfmFarFleeApproachReceipt& output,
    Status& status) noexcept;

Result<HabfmMergeProfileSelection> SelectHabfmMergeProfile(
    const DogfightGeometryFrame& frame) noexcept;

Result<HabfmLeadTurnEvidence> EvaluateHabfmLeadTurn(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& n_max_g,
    bool n_max_admitted) noexcept;

void BuildHabfmMergeApproachIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    ControlIntent& output,
    Status& status) noexcept;

void BuildHabfmThreeDimensionalMergeIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m,
    ControlIntent& output,
    Status& status) noexcept;

void BuildHabfmMergeApproachIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m,
    ControlIntent& output,
    Status& status) noexcept;

void BuildHabfmEnergyFightIntent(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmFrontalPassSupply& frontal_pass,
    ControlIntent& output,
    Status& status) noexcept;

void BuildHabfmTwoCircleIntent(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign,
    ControlIntent& output,
    Status& status) noexcept;

class HabfmActiveControlCore
{
public:
    HabfmActiveControlCore() noexcept;

    void ResetEpisode() noexcept;
    void ResetLeg() noexcept;
    HabfmActiveControlCoreSnapshot Snapshot() const noexcept;

    void StepControlIntent(
        const DogfightGeometryFrame& frame,
        const HabfmActiveCoreInputs& inputs,
        HabfmActiveControlOutput& output,
        Status& status,
        std::uint64_t blackboard_neutral_cue_streak = 0U) noexcept;

protected:
    void ResetLegState() noexcept;

    HabfmActiveBranch active_branch_ = HabfmActiveBranch::None;
    bool leg_selection_valid_ = false;
    HabfmMergeProfileSelection leg_selection_{};
    bool guard_latched_profile_valid_ = false;
    HabfmCircleProfile guard_latched_profile_ = HabfmCircleProfile::TwoCircle;
    bool side_sign_valid_ = false;
    std::int32_t side_sign_ = 0;
    bool previous_heading_valid_ = false;
    double previous_heading_rad_ = 0.0;
    double progress_rad_ = 0.0;
    bool previous_closing_valid_ = false;
    bool previous_closing_ = false;
    HabfmFarFleeApproachState far_flee_approach_state_{};
};
}
