#include "LadyLuck/guidance/g10/G10SecondUseSelectionSupply.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{

using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g10::G10OptionalBool;
using LadyLuck::guidance::g10::G10OptionalDouble;
using LadyLuck::guidance::g10::G10SecondUseSelectionBinding;
using LadyLuck::guidance::g10::G10SecondUseSelectionReason;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3NumpyAssociation(
    const Vector3& left,
    const Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3NumpyAssociation(value, value));
}

void SetOptional(G10OptionalDouble& value, const double scalar) noexcept
{
    value.has_value = true;
    value.value = scalar;
}

void SetOptional(G10OptionalBool& value, const bool scalar) noexcept
{
    value.has_value = true;
    value.value = scalar;
}

void RefuseSelection(
    G10SecondUseSelectionBinding& output,
    const G10SecondUseSelectionReason reason) noexcept
{
    output.bound = false;
    output.reason = reason;
    output.selection_available = false;
    output.barrel_roll_attack_family = false;
}

double DefenderAspectDeg(
    const Vector3& own_position,
    const Vector3& defender_position,
    const Vector3& defender_velocity,
    const double range_m,
    const double defender_speed_mps) noexcept
{
    const Vector3 line_of_sight{{
        own_position[0] - defender_position[0],
        own_position[1] - defender_position[1],
        own_position[2] - defender_position[2]}};
    const Vector3 line_of_sight_unit{{
        line_of_sight[0] / range_m,
        line_of_sight[1] / range_m,
        line_of_sight[2] / range_m}};
    const Vector3 defender_tail_unit{{
        -defender_velocity[0] / defender_speed_mps,
        -defender_velocity[1] / defender_speed_mps,
        -defender_velocity[2] / defender_speed_mps}};
    const double nominal = Dot3NumpyAssociation(
        line_of_sight_unit,
        defender_tail_unit);
    const double clipped = (std::max)(-1.0, (std::min)(1.0, nominal));
    return std::acos(clipped) * (180.0 / LadyLuck::constants::Pi);
}

bool OvershootAlreadyRealized(
    const LadyLuck::guidance::g10::G10PursuitOvershootForecastReceipt&
        forecast) noexcept
{
    using LadyLuck::guidance::g10::G10PursuitOvershootForecastReason;
    using LadyLuck::guidance::g10::G10PursuitOvershootForecastStatus;
    return forecast.status == G10PursuitOvershootForecastStatus::NotForced
        && forecast.reason
            == G10PursuitOvershootForecastReason::OwnAlreadyAhead;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

const char* G10SecondUseSelectionReasonLabel(
    const G10SecondUseSelectionReason reason) noexcept
{
    switch (reason)
    {
    case G10SecondUseSelectionReason::AspectUnresolved:
        return "aspect_unresolved";
    case G10SecondUseSelectionReason::SpeedDumpBindingRefused:
        return "speed_dump_binding_refused";
    case G10SecondUseSelectionReason::ManualSelectionNotBarrelFamily:
        return "manual_selection_not_barrel_family";
    case G10SecondUseSelectionReason::SecondUseSelectionBound:
        return "second_use_selection_bound";
    case G10SecondUseSelectionReason::ContractRejected:
        return "g10_second_use_selection_contract_rejected";
    case G10SecondUseSelectionReason::BridgeReceiptNotAdmitted:
    default:
        return "bridge_receipt_not_admitted";
    }
}

void BindG10SecondUseSelection(
    const G10SecondUseBridgeAdmissionReceipt& bridge,
    const DogfightGeometryFrame& frame,
    const G10BarrelSpeedDumpDecisionReceipt& speed_dump,
    G10SecondUseSelectionBinding& output,
    Status& status) noexcept
{
    output = G10SecondUseSelectionBinding{};
    status = Status{};
    static_cast<void>(frame);
    static_cast<void>(speed_dump);
    if (!bridge.valid)
    {
        RefuseSelection(
            output,
            G10SecondUseSelectionReason::ContractRejected);
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (!bridge.admitted)
    {
        RefuseSelection(
            output,
            G10SecondUseSelectionReason::BridgeReceiptNotAdmitted);
        return;
    }

    if (bridge.adversary_post_reversal_turn_sign != -1
        && bridge.adversary_post_reversal_turn_sign != 1)
    {
        RefuseSelection(
            output,
            G10SecondUseSelectionReason::ContractRejected);
        status.code = StatusCode::InvalidArgument;
        return;
    }
    // The causal bridge already selected the second-use family.  Aspect and a
    // separate manual speed-dump proof were duplicate tactical admission
    // gates and are intentionally not evaluated here.
    output.bound = true;
    output.reason = G10SecondUseSelectionReason::SecondUseSelectionBound;
    output.selection_available = true;
    output.barrel_roll_attack_family = true;
}

const char* G10SecondUseSupplyReasonLabel(
    const G10SecondUseSupplyReason reason) noexcept
{
    switch (reason)
    {
    case G10SecondUseSupplyReason::ForecastUnavailable:
        return "supply_withheld:forecast_unavailable";
    case G10SecondUseSupplyReason::FlightPathGammaUnavailable:
        return "supply_withheld:flight_path_gamma_unavailable";
    case G10SecondUseSupplyReason::SupplyPublished:
        return "supply_published";
    case G10SecondUseSupplyReason::ContractRejected:
        return "g10_second_use_supply_contract_rejected";
    case G10SecondUseSupplyReason::NotUpdated:
    default:
        return "not_updated";
    }
}

const char* G10ApprovedMinimumSourceSha256() noexcept
{
    return "45E3C7CA852DE70EFFC29B2E24F239D78823BD074AFBF7F121D871CBF7104F7F";
}

const char* G10ApprovedMinimumComparisonId() noexcept
{
    return "g16-barrel-high-aspect-v2-continuous-entry-phase-reachability-001";
}

const char* G10SecondUseSupplyLineageId() noexcept
{
    return "doctrine-bt-g10-second-use-gate-001";
}

const char* G10SecondUseSupplySourceEpoch() noexcept
{
    return "doctrine-bt-g10-second-use-gate-epoch-001";
}

void BuildG10TrackedApprovedReceipts(
    G10BarrelSpeedDumpDecisionReceipt& speed_dump,
    G10BarrelLoadSelectionReceipt& load_selection,
    double& roll_rate_limit_radps,
    Status& status) noexcept
{
    speed_dump = G10BarrelSpeedDumpDecisionReceipt{};
    speed_dump.valid = true;
    speed_dump.status = G10BarrelSpeedDumpStatus::BarrelEffectRequired;
    speed_dump.barrel_manual_cell_applicable = true;
    speed_dump.eligible_non_barrel_candidate_count = 0U;
    speed_dump.maximum_non_barrel_upper_reduction_mps = 0.0;
    SetOptional(speed_dump.substantial_speed_dump_required, true);
    speed_dump.positive_basis =
        G10BarrelSpeedDumpBasis::ManualCellNoNonBarrelAlternative;

    load_selection = G10BarrelLoadSelectionReceipt{};
    load_selection.valid = true;
    load_selection.status = G10BarrelLoadSelectionStatus::Selected;
    load_selection.selected_load_magnitude_g = 5.04884908375921;
    load_selection.selected_roll_rate_magnitude_radps = 1.9069;
    load_selection.selected_constructive_reduction_lower_mps =
        -4.496831526434519;
    load_selection.selected_effect_upper_mps = -4.341631171642561;
    load_selection.selected_causal_effect_time_s = 3.033333333333333;
    load_selection.diagnostic_structural_requirement_upper_mps =
        204.05677679889064;
    load_selection.effect_horizon_s = 3.9499999999999997;
    load_selection.effective_load_limit_g = 5.814582568147313;
    load_selection.effective_roll_rate_limit_radps = 3.1416;
    load_selection.candidate_count = 1U;
    roll_rate_limit_radps = 3.1416;
    status = Status{};
}

G10SecondUseSupplyProvider::G10SecondUseSupplyProvider() noexcept
{
    Status status{};
    BuildG10TrackedApprovedReceipts(
        speed_dump_,
        load_selection_,
        roll_rate_limit_radps_,
        status);
    Reset();
}

void G10SecondUseSupplyProvider::Reset() noexcept
{
    overshoot_realized_t_sec_ = G10OptionalDouble{};
    last_reason_ = G10SecondUseSupplyReason::NotUpdated;
}

G10SecondUseSupplyReason
G10SecondUseSupplyProvider::LastReason() const noexcept
{
    return last_reason_;
}

G10OptionalDouble
G10SecondUseSupplyProvider::OvershootRealizedT() const noexcept
{
    return overshoot_realized_t_sec_;
}

void G10SecondUseSupplyProvider::Update(
    const DogfightGeometryFrame& frame,
    const G10PursuitOvershootForecastReceipt* const forecast,
    const G10FlightPathGammaLimitReceipt* const gamma_limit,
    G10SecondUseSupply& output,
    Status& status) noexcept
{
    const G10SecondUseCausalSupplyInput no_causal_receipts{};
    Update(
        frame,
        forecast,
        gamma_limit,
        no_causal_receipts,
        output,
        status);
}

void G10SecondUseSupplyProvider::Update(
    const DogfightGeometryFrame& frame,
    const G10PursuitOvershootForecastReceipt* const forecast,
    const G10FlightPathGammaLimitReceipt* const gamma_limit,
    const G10SecondUseCausalSupplyInput& causal_input,
    G10SecondUseSupply& output,
    Status& status) noexcept
{
    output = G10SecondUseSupply{};
    status = Status{};
    if (forecast == nullptr)
    {
        last_reason_ = G10SecondUseSupplyReason::ForecastUnavailable;
        return;
    }
    if (gamma_limit == nullptr || !gamma_limit->available)
    {
        last_reason_ = G10SecondUseSupplyReason::FlightPathGammaUnavailable;
        return;
    }
    const double gamma = gamma_limit->value_rad;
    if (!std::isfinite(gamma))
    {
        if (!gamma_limit->source_nonempty)
        {
            last_reason_ =
                G10SecondUseSupplyReason::FlightPathGammaUnavailable;
            return;
        }
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (gamma <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    const double time_s = frame.t_sec;
    if (!std::isfinite(time_s))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (!overshoot_realized_t_sec_.has_value
        && OvershootAlreadyRealized(*forecast))
    {
        SetOptional(overshoot_realized_t_sec_, time_s);
    }

    const Vector3& station_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(station_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& opponent_position = frame.opponent.position_ned_m;
    const Vector3& opponent_nose = frame.opponent.nose_ned;
    const Vector3 relative{{
        own_position[0] - opponent_position[0],
        own_position[1] - opponent_position[1],
        own_position[2] - opponent_position[2]}};
    if (!FiniteVector(relative) || !FiniteVector(opponent_nose))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const bool body_39_crossed =
        -Dot3NumpyAssociation(relative, opponent_nose) < 0.0;

    output.valid = true;
    output.overshoot_realized_t_sec = overshoot_realized_t_sec_;
    output.speed_dump_decision = speed_dump_;
    output.load_selection = load_selection_;
    output.roll_rate_limit_radps = roll_rate_limit_radps_;
    output.station_velocity_mps = station_velocity;
    output.flight_path_gamma_limit_rad = gamma;
    output.moving_body_3_9_crossed = body_39_crossed;
    output.descending_lag_command_applied_before_state =
        causal_input.completed_k_minus_1_descending_lag_publication;
    output.prevention_failure_handoff_required =
        causal_input.current_prevention_failure_handoff_required;
    output.prevention_egress_handoff_available =
        causal_input.current_prevention_failure_handoff_required.has_value;
    last_reason_ = G10SecondUseSupplyReason::SupplyPublished;
}

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
