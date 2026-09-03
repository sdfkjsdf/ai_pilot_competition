#include "LadyLuck/guidance/obfm/ObfmLeadDiscipline.hpp"

namespace
{

using LadyLuck::guidance::obfm::ObfmLeadDisciplinePursuitState;

bool IsClassifiedState(
    const ObfmLeadDisciplinePursuitState state) noexcept
{
    switch (state)
    {
    case ObfmLeadDisciplinePursuitState::Lag:
    case ObfmLeadDisciplinePursuitState::Pure:
    case ObfmLeadDisciplinePursuitState::Lead:
        return true;
    case ObfmLeadDisciplinePursuitState::NotObservable:
        return false;
    }
    // Totalize malformed receipt bytes without inventing command authority.
    return false;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* ObfmLeadDisciplineReasonLabel(
    const ObfmLeadDisciplineReason reason) noexcept
{
    switch (reason)
    {
    case ObfmLeadDisciplineReason::PreviousObservationMissing:
        return "previous_observation_missing";
    case ObfmLeadDisciplineReason::BehaviorSwitchNotAdmitted:
        return "behavior_switch_not_admitted";
    case ObfmLeadDisciplineReason::LiftClassificationInvalid:
        return "lift_classification_invalid";
    case ObfmLeadDisciplineReason::NoseClassificationInvalid:
        return "nose_classification_invalid";
    case ObfmLeadDisciplineReason::ClassificationStateUnavailable:
        return "classification_state_unavailable";
    case ObfmLeadDisciplineReason::RulesAgree:
        return "rules_agree";
    case ObfmLeadDisciplineReason::LiftPureAllowsTerminalPull:
        return "lift_pure_allows_terminal_pull";
    case ObfmLeadDisciplineReason::LiftLeadAllowsTerminalPull:
        return "lift_lead_allows_terminal_pull";
    case ObfmLeadDisciplineReason::
            LiftLagDisagreementWithholdsTerminalPull:
        return "lift_lag_disagreement_withholds_terminal_pull";
    }
    return "unknown";
}

void EvaluateObfmLeadDiscipline(
    const ObfmLeadDisciplineInput& input,
    ObfmLeadDisciplineReceipt& output) noexcept
{
    output = ObfmLeadDisciplineReceipt{};
    output.previous_observation_present =
        input.previous_observation_present;

    if (!input.previous_observation_present)
    {
        output.reason =
            ObfmLeadDisciplineReason::PreviousObservationMissing;
        return;
    }
    if (!input.behavior_switch_admitted)
    {
        output.reason =
            ObfmLeadDisciplineReason::BehaviorSwitchNotAdmitted;
        return;
    }

    output.evidence_evaluated = true;
    if (!input.lift_rule.valid)
    {
        // behavior_switch_admitted is produced only after the resolved lift
        // classification is valid.  Preserve the command and diagnose this
        // impossible producer combination without returning a fault status.
        output.producer_contract_contradiction = true;
        output.reason =
            ObfmLeadDisciplineReason::LiftClassificationInvalid;
        return;
    }
    if (!input.nose_rule.valid)
    {
        output.reason =
            ObfmLeadDisciplineReason::NoseClassificationInvalid;
        return;
    }
    if (!IsClassifiedState(input.lift_rule.state)
        || !IsClassifiedState(input.nose_rule.state))
    {
        // Both classification valid bits were declared.  Their producer can
        // publish only LAG/PURE/LEAD in that state; NOT_OBSERVABLE or an
        // unknown enum byte is therefore diagnostic contract damage.
        output.producer_contract_contradiction = true;
        output.reason =
            ObfmLeadDisciplineReason::ClassificationStateUnavailable;
        return;
    }
    if (input.lift_rule.state == input.nose_rule.state)
    {
        output.reason = ObfmLeadDisciplineReason::RulesAgree;
        return;
    }

    if (input.lift_rule.state
        == ObfmLeadDisciplinePursuitState::Lag)
    {
        output.preserve_terminal_tracking = false;
        output.withhold_terminal_pull = true;
        output.reason = ObfmLeadDisciplineReason::
            LiftLagDisagreementWithholdsTerminalPull;
        return;
    }
    output.reason = input.lift_rule.state
        == ObfmLeadDisciplinePursuitState::Pure
        ? ObfmLeadDisciplineReason::LiftPureAllowsTerminalPull
        : ObfmLeadDisciplineReason::LiftLeadAllowsTerminalPull;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
