#include "LadyLuck/guidance/g10/G10SecondUseAdmissionProvider.hpp"

#include <cmath>

namespace
{

using LadyLuck::guidance::g10::G10OptionalBool;
using LadyLuck::guidance::g10::G10SecondUseAdmissionReason;
using LadyLuck::guidance::g10::G10SecondUseAdmissionReceipt;

void SetOptional(G10OptionalBool& value, const bool scalar) noexcept
{
    value.has_value = true;
    value.value = scalar;
}

void BeginReceipt(
    G10SecondUseAdmissionReceipt& output,
    const G10SecondUseAdmissionReason reason) noexcept
{
    output.valid = true;
    output.reason = reason;
    output.bridge.valid = true;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

const char* G10SecondUseAdmissionReasonLabel(
    const G10SecondUseAdmissionReason reason) noexcept
{
    switch (reason)
    {
    case G10SecondUseAdmissionReason::GateDisabled:
        return "gate_disabled";
    case G10SecondUseAdmissionReason::ScissorsResponseOwns:
        return "scissors_response_owns";
    case G10SecondUseAdmissionReason::ForecastUnavailable:
        return "forecast_unavailable";
    case G10SecondUseAdmissionReason::OvershootNotRealized:
        return "overshoot_not_realized";
    case G10SecondUseAdmissionReason::OvershootInstantUnavailable:
        return "overshoot_instant_unavailable";
    case G10SecondUseAdmissionReason::AdversaryReversalNotCurrent:
        return "adversary_reversal_not_current";
    case G10SecondUseAdmissionReason::ReversalNotAfterOvershoot:
        return "reversal_not_after_overshoot";
    case G10SecondUseAdmissionReason::SecondUseAdmitted:
        return "second_use_admitted";
    case G10SecondUseAdmissionReason::NotUpdated:
    default:
        return "not_updated";
    }
}

G10SecondUseAdmissionProvider::G10SecondUseAdmissionProvider() noexcept =
    default;

void G10SecondUseAdmissionProvider::Reset() noexcept
{
}

void G10SecondUseAdmissionProvider::Update(
    const DogfightGeometryFrame& frame,
    const bool gate_enabled,
    const obfm::PursuitOvershootForecast* const completed_forecast,
    const obfm::AdversaryReversalObservation* const completed_reversal,
    const G10OptionalDouble& overshoot_realized_t_sec,
    const bool scissors_response_engaged,
    G10SecondUseAdmissionReceipt& output,
    Status& status) noexcept
{
    output = G10SecondUseAdmissionReceipt{};
    status = Status{};
    static_cast<void>(frame);

    if (!gate_enabled)
    {
        BeginReceipt(output, G10SecondUseAdmissionReason::GateDisabled);
        return;
    }
    if (scissors_response_engaged)
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::ScissorsResponseOwns);
        return;
    }
    if (completed_forecast == nullptr)
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::ForecastUnavailable);
        return;
    }

    const bool overshoot_realized =
        completed_forecast->status
            == obfm::PursuitOvershootForecastStatus::NotForced
        && completed_forecast->reason
            == obfm::PursuitOvershootForecastReason::OwnAlreadyAhead;
    SetOptional(output.overshoot_realized, overshoot_realized);
    if (!overshoot_realized)
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::OvershootNotRealized);
        return;
    }
    if (!overshoot_realized_t_sec.has_value
        || !std::isfinite(overshoot_realized_t_sec.value))
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::OvershootInstantUnavailable);
        return;
    }

    const bool reversal_current =
        obfm::ScissorsSituationResolved(completed_reversal);
    SetOptional(output.adversary_reversal_current, reversal_current);
    if (!reversal_current)
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::AdversaryReversalNotCurrent);
        return;
    }

    const bool reversal_after_overshoot =
        completed_reversal->current_run_start_t.has_value
        && std::isfinite(completed_reversal->current_run_start_t.value)
        && completed_reversal->current_run_start_t.value
            >= overshoot_realized_t_sec.value;
    SetOptional(
        output.reversal_after_overshoot,
        reversal_after_overshoot);
    output.bridge.adversary_post_reversal_turn_sign =
        completed_reversal->current_sign;
    if (!reversal_after_overshoot)
    {
        BeginReceipt(
            output,
            G10SecondUseAdmissionReason::ReversalNotAfterOvershoot);
        return;
    }

    // HABFM selection and higher-priority Auto-GCAS/gun-defense ownership are
    // already resolved by the production selector. G10 therefore answers
    // only its causal question: overshoot followed by opponent reversal.
    BeginReceipt(output, G10SecondUseAdmissionReason::SecondUseAdmitted);
    output.admitted = true;
    output.entry_family_available = true;
    output.barrel_roll_attack_family = true;
    output.bridge.admitted = true;
}

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
