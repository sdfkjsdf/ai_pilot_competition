#include "LadyLuck/guidance/habfm/HabfmEngageDecision.hpp"

#include <algorithm>
#include <cmath>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

bool EnergyDeficitProven(
    const LadyLuck::HabfmEngageDecisionInput& input) noexcept
{
    return input.merge_selection_present
        && input.merge_selection.energy_state
            == LadyLuck::HabfmMergeEnergyState::DeficitProven;
}

bool AdmittedCorridor(
    const LadyLuck::HabfmFrontalPassSupply& supply,
    double& abeam_m,
    std::int32_t& side_sign) noexcept
{
    abeam_m = 0.0;
    side_sign = 0;
    if (!supply.admitted
        || !std::isfinite(supply.compressed_abeam_m)
        || supply.compressed_abeam_m <= 0.0
        || (supply.side_sign != -1 && supply.side_sign != 1))
    {
        return false;
    }
    abeam_m = supply.compressed_abeam_m;
    side_sign = supply.side_sign;
    return true;
}

bool AdmittedFloor(
    const LadyLuck::TacticalSpeedFloorSample& sample,
    double& floor_mps) noexcept
{
    floor_mps = 0.0;
    if (!sample.admitted()
        || !std::isfinite(sample.floor_mps.value)
        || sample.floor_mps.value <= 0.0)
    {
        return false;
    }
    floor_mps = sample.floor_mps.value;
    return true;
}

void PublishEngage(
    const LadyLuck::HabfmEngageDecisionInput& input,
    const LadyLuck::HabfmEngageDecisionReason reason,
    LadyLuck::HabfmEngageDecisionReceipt& output) noexcept
{
    output = LadyLuck::HabfmEngageDecisionReceipt{};
    output.frame_identity = input.frame_identity;
    output.valid = true;
    output.decision = LadyLuck::HabfmEngageDecisionState::Engage;
    output.reason = reason;
}

void PublishAvoid(
    const LadyLuck::HabfmEngageDecisionInput& input,
    const LadyLuck::HabfmEngageDecisionReason reason,
    LadyLuck::HabfmEngageDecisionReceipt& output) noexcept
{
    output = LadyLuck::HabfmEngageDecisionReceipt{};
    output.frame_identity = input.frame_identity;
    output.valid = true;
    output.active = true;
    output.decision = LadyLuck::HabfmEngageDecisionState::AvoidPass;
    output.reason = reason;
    output.pass_corridor_present = AdmittedCorridor(
        input.frontal_pass, output.pass_abeam_m, output.pass_side_sign);
    output.speed_floor_present = AdmittedFloor(
        input.speed_floor, output.speed_floor_mps);
    output.hold_current_course = output.speed_floor_present;
    if (!output.pass_corridor_present)
    {
        output.reference_reason = output.speed_floor_present
            ? LadyLuck::HabfmEngageReferenceReason::CorridorPending
            : LadyLuck::HabfmEngageReferenceReason::CorridorAndFloorPending;
    }
    else
    {
        output.reference_reason = output.speed_floor_present
            ? LadyLuck::HabfmEngageReferenceReason::Complete
            : LadyLuck::HabfmEngageReferenceReason::FloorPending;
    }
}

bool EgressComplete(
    const LadyLuck::HabfmEngageDecisionInput& input,
    bool& evaluated,
    LadyLuck::Status& status) noexcept
{
    evaluated = false;
    if (!std::isfinite(input.closing_speed_mps))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return false;
    }
    if (input.closing_speed_mps > 0.0)
    {
        evaluated = true;
        return false;
    }
    if (!std::isfinite(input.adversary_range_m)
        || !std::isfinite(input.reengage_range_m)
        || input.reengage_range_m <= 0.0)
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return false;
    }
    evaluated = true;
    return input.adversary_range_m > input.reengage_range_m;
}

void FailOverlay(
    LadyLuck::HabfmAvoidPassOverlayReceipt& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::HabfmAvoidPassOverlayReceipt{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{

void HabfmEngageDecisionLatch::Reset() noexcept
{
    decision_ = HabfmEngageDecisionState::Engage;
}

void HabfmEngageDecisionLatch::Update(
    const HabfmEngageDecisionInput& input,
    HabfmEngageDecisionReceipt& output,
    Status& status) noexcept
{
    output = HabfmEngageDecisionReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(input.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!input.gate_enabled)
    {
        PublishEngage(input, HabfmEngageDecisionReason::GateDisabled, output);
        decision_ = HabfmEngageDecisionState::Engage;
        return;
    }
    if (decision_ == HabfmEngageDecisionState::Engage)
    {
        if (!EnergyDeficitProven(input))
        {
            PublishEngage(
                input,
                HabfmEngageDecisionReason::EnergyDeficitNotProven,
                output);
            return;
        }
        if (!std::isfinite(input.closing_speed_mps))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        if (input.closing_speed_mps <= 0.0)
        {
            PublishEngage(
                input,
                HabfmEngageDecisionReason::ClosingNotProven,
                output);
            return;
        }
        PublishAvoid(
            input,
            HabfmEngageDecisionReason::AvoidPassClosingDeficit,
            output);
        decision_ = HabfmEngageDecisionState::AvoidPass;
        return;
    }

    bool evaluated = false;
    const bool complete = EgressComplete(input, evaluated, status);
    static_cast<void>(evaluated);
    if (!status.ok())
    {
        return;
    }
    if (complete)
    {
        PublishEngage(
            input,
            HabfmEngageDecisionReason::EgressRangeCompleteReengage,
            output);
        decision_ = HabfmEngageDecisionState::Engage;
        return;
    }
    PublishAvoid(
        input,
        HabfmEngageDecisionReason::AvoidPassLatched,
        output);
    decision_ = HabfmEngageDecisionState::AvoidPass;
}

void BuildHabfmAvoidPassOverlay(
    const HabfmAvoidPassOverlayInput& input,
    HabfmAvoidPassOverlayReceipt& output,
    Status& status) noexcept
{
    output = HabfmAvoidPassOverlayReceipt{};
    status = Status{};
    if (input.modifier_writer_id == ControlIntentWriterNone
        || !input.decision.valid
        || !IsValidControlFrameIdentity(input.frame.frame_identity)
        || !SameControlFrameIdentity(
            input.upstream_intent.frame_identity,
            input.frame.frame_identity)
        || !SameControlFrameIdentity(
            input.decision.frame_identity,
            input.frame.frame_identity))
    {
        FailOverlay(output, status, StatusCode::InvalidArgument);
        return;
    }
    Status intent_status{};
    input.upstream_intent.Validate(intent_status);
    if (!intent_status.ok())
    {
        FailOverlay(output, status, intent_status.code);
        return;
    }
    output.frame_identity = input.frame.frame_identity;
    output.valid = true;
    output.upstream_writer_id = input.upstream_intent.writer_id;
    output.candidate = input.upstream_intent;
    if (!input.decision.active
        || input.decision.decision != HabfmEngageDecisionState::AvoidPass)
    {
        return;
    }
    output.applicable = true;
    if (!std::isfinite(input.frame.closing_speed_mps))
    {
        FailOverlay(output, status, StatusCode::NonFiniteInput);
        return;
    }

    if (input.frame.closing_speed_mps > 0.0)
    {
        if (!input.decision.pass_corridor_present)
        {
            return;
        }
        if (!FiniteVector(input.frame.own.position_ned_m)
            || !FiniteVector(input.frame.opponent.position_ned_m))
        {
            FailOverlay(output, status, StatusCode::NonFiniteInput);
            return;
        }
        const double north_m = input.frame.opponent.position_ned_m[0]
            - input.frame.own.position_ned_m[0];
        const double east_m = input.frame.opponent.position_ned_m[1]
            - input.frame.own.position_ned_m[1];
        const double horizontal_m = std::sqrt(
            north_m * north_m + east_m * east_m);
        if (!std::isfinite(horizontal_m) || horizontal_m <= 0.0
            || !std::isfinite(input.decision.pass_abeam_m)
            || input.decision.pass_abeam_m <= 0.0
            || (input.decision.pass_side_sign != -1
                && input.decision.pass_side_sign != 1))
        {
            FailOverlay(output, status, StatusCode::InvalidArgument);
            return;
        }
        const double los_north = north_m / horizontal_m;
        const double los_east = east_m / horizontal_m;
        const double offset =
            static_cast<double>(input.decision.pass_side_sign)
            * input.decision.pass_abeam_m;
        output.candidate.aim_point_m = Vector3{{
            input.frame.opponent.position_ned_m[0] - offset * los_east,
            input.frame.opponent.position_ned_m[1] + offset * los_north,
            input.frame.own.position_ned_m[2]}};
        if (input.decision.speed_floor_present)
        {
            if (!FiniteVector(input.frame.own.velocity_ned_mps)
                || !std::isfinite(input.decision.speed_floor_mps)
                || input.decision.speed_floor_mps <= 0.0)
            {
                FailOverlay(output, status, StatusCode::NonFiniteInput);
                return;
            }
            const double speed_mps = NumpyNorm3(
                input.frame.own.velocity_ned_mps);
            if (!std::isfinite(speed_mps))
            {
                FailOverlay(output, status, StatusCode::NonFiniteInput);
                return;
            }
            output.candidate.desired_speed_mps = (std::max)(
                speed_mps, input.decision.speed_floor_mps);
        }
        output.candidate.behavior_id =
            DoctrineBehaviorId::HabfmAvoidPassCross;
        output.leg = HabfmAvoidPassLeg::Cross;
    }
    else
    {
        if (!FiniteVector(input.frame.own.position_ned_m)
            || !FiniteVector(input.frame.own.velocity_ned_mps)
            || !std::isfinite(input.frame.own_offense.range_m)
            || input.frame.own_offense.range_m <= 0.0)
        {
            FailOverlay(output, status, StatusCode::NonFiniteInput);
            return;
        }
        const double horizontal_m = std::sqrt(
            input.frame.own.velocity_ned_mps[0]
                * input.frame.own.velocity_ned_mps[0]
            + input.frame.own.velocity_ned_mps[1]
                * input.frame.own.velocity_ned_mps[1]);
        const double speed_mps = NumpyNorm3(input.frame.own.velocity_ned_mps);
        if (!std::isfinite(horizontal_m) || horizontal_m <= 0.0
            || !std::isfinite(speed_mps) || speed_mps <= 0.0)
        {
            FailOverlay(output, status, StatusCode::InvalidArgument);
            return;
        }
        const double course_north =
            input.frame.own.velocity_ned_mps[0] / horizontal_m;
        const double course_east =
            input.frame.own.velocity_ned_mps[1] / horizontal_m;
        output.candidate.aim_point_m = Vector3{{
            input.frame.own.position_ned_m[0]
                + input.frame.own_offense.range_m * course_north,
            input.frame.own.position_ned_m[1]
                + input.frame.own_offense.range_m * course_east,
            input.frame.own.position_ned_m[2]}};
        output.candidate.desired_speed_mps =
            input.decision.speed_floor_present
            ? (std::max)(speed_mps, input.decision.speed_floor_mps)
            : speed_mps;
        output.candidate.behavior_id =
            DoctrineBehaviorId::HabfmAvoidPassExtend;
        output.leg = HabfmAvoidPassLeg::Extend;
    }

    output.candidate.writer_id = input.modifier_writer_id;
    output.candidate.Validate(intent_status);
    if (!intent_status.ok())
    {
        FailOverlay(output, status, intent_status.code);
        return;
    }
    output.modified = true;
    output.published_writer_id = input.modifier_writer_id;
}

} // namespace LadyLuck
