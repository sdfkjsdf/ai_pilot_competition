#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"

namespace LadyLuck
{
// Legacy adapter result retained for TacticalCommand parity tests and callers.
// The production Doctrine context owns HabfmActiveControlCore instead.
struct HabfmActiveCoreOutput
{
    HabfmActiveCoreLegStatus leg_status = HabfmActiveCoreLegStatus::Running;
    HabfmActiveBranch branch = HabfmActiveBranch::None;
    OptionalValue<TacticalCommand> command{};
    HabfmLeadTurnEvidence lead_turn{};
    HabfmFarFleeApproachReceipt far_flee_approach{};
    OptionalValue<HabfmMergeProfileSelection> profile_selection{};
    OptionalValue<HabfmSelectorTransitionGuardReceipt> transition_guard{};
    OptionalValue<HabfmCircleProfile> selected_profile{};
    OptionalValue<HabfmCheckpointCueEvidence> checkpoint_cue{};
    OptionalValue<std::int32_t> turn_side_sign{};
    double turn_progress_rad = 0.0;
    std::uint64_t neutral_cue_streak = 0U;
    bool merge_pass = false;
    bool mode_recheck = false;
};

struct HabfmActiveCoreSnapshot
{
    HabfmActiveBranch active_branch = HabfmActiveBranch::None;
    OptionalValue<HabfmMergeProfileSelection> leg_selection{};
    OptionalValue<HabfmCircleProfile> guard_latched_profile{};
    OptionalValue<std::int32_t> side_sign{};
    OptionalValue<double> previous_heading_rad{};
    double progress_rad = 0.0;
    OptionalValue<bool> previous_closing{};
    HabfmFarFleeApproachState far_flee_approach{};
};

Result<TacticalCommand> BuildHabfmMergeApproachCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room);

Result<TacticalCommand> BuildHabfmMergeApproachCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m);

Result<TacticalCommand> BuildHabfmEnergyFightCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmFrontalPassSupply& frontal_pass);

Result<TacticalCommand> BuildHabfmTwoCircleCommand(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign);

class HabfmActiveCore : public HabfmActiveControlCore
{
public:
    HabfmActiveCore() noexcept = default;

    HabfmActiveCoreSnapshot Snapshot() const noexcept;

    Result<HabfmActiveCoreOutput> Step(
        const DogfightGeometryFrame& frame,
        const HabfmActiveCoreInputs& inputs,
        std::uint64_t blackboard_neutral_cue_streak = 0U);
};
}
