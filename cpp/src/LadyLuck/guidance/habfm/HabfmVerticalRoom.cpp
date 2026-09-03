#include "LadyLuck/guidance/habfm/HabfmVerticalRoom.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{
bool FiniteBacking(const LadyLuck::HabfmOptionalScalar& value) noexcept
{
    return std::isfinite(value.value);
}

void Level(
    const LadyLuck::HabfmVerticalRoomReason reason,
    LadyLuck::HabfmVerticalRoomReceipt& output,
    const bool identity_valid = false,
    const double identity_m = 0.0,
    const bool radius_valid = false,
    const double radius_m = 0.0) noexcept
{
    output = LadyLuck::HabfmVerticalRoomReceipt{};
    output.reason = reason;
    output.identity_depth_m.has_value = identity_valid;
    output.identity_depth_m.value = identity_valid ? identity_m : 0.0;
    output.radius_clamp_m.has_value = radius_valid;
    output.radius_clamp_m.value = radius_valid ? radius_m : 0.0;
}
}

namespace LadyLuck
{
void EvaluateHabfmVerticalRoom(
    const HabfmVerticalRoomInputs& inputs,
    HabfmVerticalRoomReceipt& output,
    Status& status) noexcept
{
    output = HabfmVerticalRoomReceipt{};
    status.code = StatusCode::Ok;

    if ((inputs.gate.enabled
            && (!inputs.gate.provenance_present
                || !inputs.gate.provenance_matches))
        || (!inputs.gate.enabled && inputs.gate.provenance_present))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!FiniteBacking(inputs.corner_speed_mps)
        || !FiniteBacking(inputs.turn_radius_m)
        || !FiniteBacking(inputs.floor_speed_mps)
        || !std::isfinite(inputs.evidence.own_speed_mps)
        || !FiniteBacking(inputs.evidence.corner_speed_mps)
        || !FiniteBacking(inputs.evidence.excess_mps)
        || !std::isfinite(inputs.evidence.speed_error_bound_mps)
        || !std::isfinite(inputs.hard_deck_margin_m)
        || !std::isfinite(inputs.horizontal_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    if (!inputs.gate.enabled)
    {
        Level(HabfmVerticalRoomReason::GateDisabled, output);
        return;
    }
    if (!inputs.evidence_present || !inputs.evidence.evidence_admitted)
    {
        Level(HabfmVerticalRoomReason::VerticalEvidenceNotAdmitted, output);
        return;
    }
    if (inputs.evidence.provenance
        != HabfmObservationProvenance::VerticalExcess)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const bool corner_valid = inputs.corner_admitted
        && inputs.corner_speed_mps.has_value
        && inputs.corner_speed_mps.value > 0.0;
    if (!corner_valid)
    {
        Level(HabfmVerticalRoomReason::CornerSpeedNotAdmitted, output);
        return;
    }
    const double corner_mps = inputs.corner_speed_mps.value;

    if (!inputs.horizontal_range_finite || inputs.horizontal_range_m <= 0.0)
    {
        Level(HabfmVerticalRoomReason::HorizontalRangeUnresolved, output);
        return;
    }
    if (inputs.evidence.state == HabfmVerticalExcessState::WithinResolution)
    {
        Level(HabfmVerticalRoomReason::WithinResolutionLevelRoom, output);
        return;
    }
    if (inputs.evidence.state != HabfmVerticalExcessState::AboveCornerProven
        && inputs.evidence.state != HabfmVerticalExcessState::BelowCornerProven)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    double target_mps = corner_mps;
    if (inputs.evidence.state == HabfmVerticalExcessState::AboveCornerProven)
    {
        if (inputs.floor_admitted
            && inputs.floor_speed_mps.has_value
            && inputs.floor_speed_mps.value > 0.0)
        {
            target_mps = (std::max)(corner_mps, inputs.floor_speed_mps.value);
        }
        if (inputs.evidence.own_speed_mps <= target_mps)
        {
            Level(HabfmVerticalRoomReason::WithinFloorBandLevelRoom, output);
            return;
        }
    }

    const bool radius_valid = inputs.turn_radius_admitted
        && inputs.turn_radius_m.has_value
        && inputs.turn_radius_m.value > 0.0;
    if (!radius_valid)
    {
        Level(HabfmVerticalRoomReason::TurnRadiusNotAdmitted, output);
        return;
    }
    const double radius_m = inputs.turn_radius_m.value;

    const double own_mps = inputs.evidence.own_speed_mps;
    if (own_mps <= 0.0 || target_mps <= 0.0)
    {
        Level(
            HabfmVerticalRoomReason::IdentityNotResolvable,
            output,
            false,
            0.0,
            true,
            radius_m);
        return;
    }
    const double identity_m = std::fabs(
        own_mps * own_mps - target_mps * target_mps)
        / (2.0 * constants::StandardGravityMps2);
    if (!std::isfinite(identity_m) || identity_m <= 0.0)
    {
        Level(
            HabfmVerticalRoomReason::IdentityNotResolvable,
            output,
            false,
            0.0,
            true,
            radius_m);
        return;
    }

    const double magnitude_m = (std::min)(identity_m, radius_m);
    const bool clamp_active = radius_m < identity_m;
    double offset_m = magnitude_m;
    HabfmVerticalRoomReason reason =
        HabfmVerticalRoomReason::VerticalRoomAboveCorner;
    if (inputs.evidence.state == HabfmVerticalExcessState::BelowCornerProven)
    {
        if (!inputs.hard_deck_margin_finite
            || inputs.hard_deck_margin_m <= 0.0)
        {
            Level(
                HabfmVerticalRoomReason::HardDeckMarginNotPositive,
                output,
                true,
                identity_m,
                true,
                radius_m);
            return;
        }
        if (magnitude_m >= inputs.hard_deck_margin_m)
        {
            Level(
                HabfmVerticalRoomReason::DiveDepthExceedsHardDeckMargin,
                output,
                true,
                identity_m,
                true,
                radius_m);
            return;
        }
        offset_m = -magnitude_m;
        reason = HabfmVerticalRoomReason::VerticalRoomBelowCorner;
    }

    const double angle_rad = std::atan2(offset_m, inputs.horizontal_range_m);
    if (!std::isfinite(offset_m) || offset_m == 0.0
        || !std::isfinite(angle_rad))
    {
        status.code = StatusCode::NonFiniteInput;
        output = HabfmVerticalRoomReceipt{};
        return;
    }
    output.admitted = true;
    output.reason = reason;
    output.vertical_offset_m = offset_m;
    output.identity_depth_m.has_value = true;
    output.identity_depth_m.value = identity_m;
    output.radius_clamp_m.has_value = true;
    output.radius_clamp_m.value = radius_m;
    output.clamp_active_valid = true;
    output.clamp_active = clamp_active;
    output.room_angle_rad.has_value = true;
    output.room_angle_rad.value = angle_rad;
}
}
