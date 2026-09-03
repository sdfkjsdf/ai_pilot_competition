#include "LadyLuck/guidance/habfm/HabfmActiveCore.hpp"

#include "LadyLuck/maneuver/HabfmFixedOneCircle.hpp"

#include <cmath>

namespace
{
template <typename T>
void CopyOptionalValue(
    const LadyLuck::IntentOptionalValue<T>& source,
    LadyLuck::OptionalValue<T>& destination) noexcept
{
    destination.has_value = source.has_value;
    destination.value = source.value;
}

LadyLuck::Result<LadyLuck::TacticalCommand> BuildLegacyCommand(
    const LadyLuck::ControlIntent& intent) noexcept
{
    LadyLuck::Result<LadyLuck::TacticalCommand> result{};
    LadyLuck::TacticalCommand candidate{};
    candidate.aim_point_m = intent.aim_point_m;
    candidate.desired_speed_mps = intent.desired_speed_mps;
    CopyOptionalValue(
        intent.aim_point_velocity_mps,
        candidate.aim_point_velocity_mps);
    candidate.desired_speed_rate_mps2 = intent.desired_speed_rate_mps2;
    candidate.specific_energy_rate_bias_m2ps3 =
        intent.specific_energy_rate_bias_m2ps3;
    CopyOptionalValue(
        intent.path_inversion_allowed,
        candidate.path_inversion_allowed);
    candidate.capture_range_des_m = intent.capture_range_des_m;
    candidate.aim_blend = intent.aim_blend;
    candidate.lead_time_tau_sec = intent.lead_time_tau_sec;
    candidate.lateral_offset_m = intent.lateral_offset_m;
    candidate.vertical_offset_m = intent.vertical_offset_m;
    candidate.vertical_yoyo_scale = intent.vertical_yoyo_scale;
    CopyOptionalValue(intent.k_roll, candidate.k_roll);
    CopyOptionalValue(intent.k_pitch, candidate.k_pitch);
    CopyOptionalValue(intent.throttle_bias, candidate.throttle_bias);
    CopyOptionalValue(
        intent.total_load_factor_limit_g,
        candidate.total_load_factor_limit_g);
    CopyOptionalValue(intent.direct_p_cmd_radps, candidate.direct_p_cmd_radps);
    CopyOptionalValue(intent.direct_nz_cmd_g, candidate.direct_nz_cmd_g);
    CopyOptionalValue(
        intent.direct_beta_cmd_rad,
        candidate.direct_beta_cmd_rad);
    CopyOptionalValue(
        intent.direct_accel_cmd_mps2,
        candidate.direct_accel_cmd_mps2);
    CopyOptionalValue(
        intent.direct_acceleration_ned_mps2,
        candidate.direct_acceleration_ned_mps2);
    CopyOptionalValue(
        intent.direct_acceleration_roll_rate_reference_radps,
        candidate.direct_acceleration_roll_rate_reference_radps);
    candidate.direct_acceleration_tracking_enabled =
        intent.direct_acceleration_tracking_enabled;
    candidate.direct_acceleration_tracking_observation_only =
        intent.direct_acceleration_tracking_observation_only;
    candidate.direct_acceleration_magnitude_tracking_enabled =
        intent.direct_acceleration_magnitude_tracking_enabled;
    candidate.direct_acceleration_loaded_roll_enabled =
        intent.direct_acceleration_loaded_roll_enabled;
    candidate.direct_acceleration_load_component_compensation_enabled =
        intent.direct_acceleration_load_component_compensation_enabled;
    candidate.direct_acceleration_yaw_coordination_enabled =
        intent.direct_acceleration_yaw_coordination_enabled;
    candidate.direct_acceleration_roll_priority_yaw_enabled =
        intent.direct_acceleration_roll_priority_yaw_enabled;
    CopyOptionalValue(
        intent.direct_bank_cmd_rad,
        candidate.direct_bank_cmd_rad);
    CopyOptionalValue(
        intent.direct_turn_rate_cmd_radps,
        candidate.direct_turn_rate_cmd_radps);
    CopyOptionalValue(
        intent.direct_load_vector_acceleration_ned_mps2,
        candidate.direct_load_vector_acceleration_ned_mps2);

    try
    {
        switch (intent.behavior_id)
        {
        case LadyLuck::DoctrineBehaviorId::HabfmMergeApproach:
            candidate.behavior_label = "MERGE_APPROACH";
            break;
        case LadyLuck::DoctrineBehaviorId::HabfmEnergyFight:
            candidate.behavior_label = "ENERGY_FIGHT";
            break;
        case LadyLuck::DoctrineBehaviorId::HabfmOneCircle:
            candidate.behavior_label = "ONE_CIRCLE";
            break;
        case LadyLuck::DoctrineBehaviorId::HabfmTwoCircle:
            candidate.behavior_label = "TWO_CIRCLE";
            break;
        default:
            result.status.code = LadyLuck::StatusCode::InvalidConfiguration;
            return result;
        }
        if (intent.mode_id != LadyLuck::DoctrineModeId::Habfm)
        {
            result.status.code = LadyLuck::StatusCode::InvalidConfiguration;
            return result;
        }
        candidate.mode_label = "HABFM";
    }
    catch (...)
    {
        result.status.code = LadyLuck::StatusCode::InvalidConfiguration;
        return result;
    }
    return LadyLuck::MakeValidatedTacticalCommand(candidate);
}

void CopyOutputMetadata(
    const LadyLuck::HabfmActiveControlOutput& source,
    LadyLuck::HabfmActiveCoreOutput& destination) noexcept
{
    destination.leg_status = source.leg_status;
    destination.branch = source.branch;
    destination.lead_turn = source.lead_turn;
    destination.far_flee_approach = source.far_flee_approach;
    destination.profile_selection.has_value =
        source.profile_selection.has_value;
    destination.profile_selection.value = source.profile_selection.value;
    destination.transition_guard.has_value =
        source.transition_guard.has_value;
    destination.transition_guard.value = source.transition_guard.value;
    destination.selected_profile.has_value = source.selected_profile.has_value;
    destination.selected_profile.value = source.selected_profile.value;
    destination.checkpoint_cue.has_value = source.checkpoint_cue.has_value;
    destination.checkpoint_cue.value = source.checkpoint_cue.value;
    destination.turn_side_sign.has_value = source.turn_side_sign.has_value;
    destination.turn_side_sign.value = source.turn_side_sign.value;
    destination.turn_progress_rad = source.turn_progress_rad;
    destination.neutral_cue_streak = source.neutral_cue_streak;
    destination.merge_pass = source.merge_pass;
    destination.mode_recheck = source.mode_recheck;
}
}

namespace LadyLuck
{
Result<TacticalCommand> BuildHabfmMergeApproachCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room)
{
    return BuildHabfmMergeApproachCommand(
        frame,
        speed_floor,
        compress_separation_aim,
        frontal_pass,
        vertical_room,
        HabfmOptionalScalar{});
}

Result<TacticalCommand> BuildHabfmMergeApproachCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const bool compress_separation_aim,
    const HabfmFrontalPassSupply& frontal_pass,
    const HabfmVerticalRoomReceipt& vertical_room,
    const HabfmOptionalScalar& far_flee_anchor_altitude_m)
{
    DogfightGeometryFrame intent_frame = frame;
    const bool legacy_capture_fallback =
        !std::isfinite(frame.own_offense.phase.max_range_m)
        || frame.own_offense.phase.max_range_m <= 0.0;
    if (legacy_capture_fallback)
    {
        intent_frame.own_offense.phase.max_range_m =
            frame.own_offense.range_m;
    }

    ControlIntent intent{};
    Status status{};
    BuildHabfmMergeApproachIntent(
        intent_frame,
        speed_floor,
        compress_separation_aim,
        frontal_pass,
        vertical_room,
        far_flee_anchor_altitude_m,
        intent,
        status);
    if (!status.ok())
    {
        Result<TacticalCommand> result{};
        result.status = status;
        return result;
    }
    if (legacy_capture_fallback)
    {
        intent.capture_range_des_m = frame.own_offense.range_m;
    }
    return BuildLegacyCommand(intent);
}

Result<TacticalCommand> BuildHabfmEnergyFightCommand(
    const DogfightGeometryFrame& frame,
    const HabfmSpeedFloorSupply& speed_floor,
    const HabfmFrontalPassSupply& frontal_pass)
{
    ControlIntent intent{};
    Status status{};
    BuildHabfmEnergyFightIntent(
        frame, speed_floor, frontal_pass, intent, status);
    if (!status.ok())
    {
        Result<TacticalCommand> result{};
        result.status = status;
        return result;
    }
    return BuildLegacyCommand(intent);
}

Result<TacticalCommand> BuildHabfmTwoCircleCommand(
    const DogfightGeometryFrame& frame,
    const std::int32_t side_sign)
{
    ControlIntent intent{};
    Status status{};
    BuildHabfmTwoCircleIntent(frame, side_sign, intent, status);
    if (!status.ok())
    {
        Result<TacticalCommand> result{};
        result.status = status;
        return result;
    }
    return BuildLegacyCommand(intent);
}

HabfmActiveCoreSnapshot HabfmActiveCore::Snapshot() const noexcept
{
    const HabfmActiveControlCoreSnapshot production =
        HabfmActiveControlCore::Snapshot();
    HabfmActiveCoreSnapshot snapshot{};
    snapshot.active_branch = production.active_branch;
    snapshot.leg_selection.has_value = production.leg_selection.has_value;
    snapshot.leg_selection.value = production.leg_selection.value;
    snapshot.guard_latched_profile.has_value =
        production.guard_latched_profile.has_value;
    snapshot.guard_latched_profile.value =
        production.guard_latched_profile.value;
    snapshot.side_sign.has_value = production.side_sign.has_value;
    snapshot.side_sign.value = production.side_sign.value;
    snapshot.previous_heading_rad.has_value =
        production.previous_heading_rad.has_value;
    snapshot.previous_heading_rad.value =
        production.previous_heading_rad.value;
    snapshot.progress_rad = production.progress_rad;
    snapshot.previous_closing.has_value = production.previous_closing.has_value;
    snapshot.previous_closing.value = production.previous_closing.value;
    snapshot.far_flee_approach = production.far_flee_approach;
    return snapshot;
}

Result<HabfmActiveCoreOutput> HabfmActiveCore::Step(
    const DogfightGeometryFrame& frame,
    const HabfmActiveCoreInputs& inputs,
    const std::uint64_t blackboard_neutral_cue_streak)
{
    Result<HabfmActiveCoreOutput> result{};
    HabfmActiveCore candidate = *this;
    HabfmActiveControlOutput control_output{};
    candidate.StepControlIntent(
        frame,
        inputs,
        control_output,
        result.status,
        blackboard_neutral_cue_streak);
    if (!result.status.ok())
    {
        return result;
    }

    HabfmActiveCoreOutput legacy_output{};
    CopyOutputMetadata(control_output, legacy_output);
    if (control_output.intent_present)
    {
        const Result<TacticalCommand> command =
            BuildLegacyCommand(control_output.intent);
        if (!command.ok())
        {
            result.status = command.status;
            return result;
        }
        try
        {
            legacy_output.command.value = command.value;
            legacy_output.command.has_value = true;
        }
        catch (...)
        {
            result.status.code = StatusCode::InvalidConfiguration;
            return result;
        }
    }

    try
    {
        result.value = legacy_output;
    }
    catch (...)
    {
        result = Result<HabfmActiveCoreOutput>{};
        result.status.code = StatusCode::InvalidConfiguration;
        return result;
    }
    *this = candidate;
    return result;
}
}
