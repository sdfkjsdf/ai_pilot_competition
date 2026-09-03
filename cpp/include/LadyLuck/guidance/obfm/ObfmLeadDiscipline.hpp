#pragma once

#include <cstdint>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// G11 consumes pursuit geometry only.  It never creates body-rate, load,
// surface, thrust, or raw-guidance commands.  When it withholds the terminal
// pull, the already-produced base LAG/PURE pursuit guidance remains the sole
// command candidate for the frame.
enum class ObfmLeadDisciplinePursuitState : std::uint8_t
{
    Lag = 0U,
    Pure = 1U,
    Lead = 2U,
    NotObservable = 3U
};

struct ObfmLeadDisciplineClassification
{
    bool valid = false;
    ObfmLeadDisciplinePursuitState state =
        ObfmLeadDisciplinePursuitState::NotObservable;
};

// The caller supplies the PREVIOUS tick's completed pursuit observation.
// This mirrors add/main@45abc9f6 exactly: the observer runs at the root-tick
// tail, so the current command may consume only the stored prior result.
struct ObfmLeadDisciplineInput
{
    bool previous_observation_present = false;
    bool behavior_switch_admitted = false;
    ObfmLeadDisciplineClassification lift_rule{};
    ObfmLeadDisciplineClassification nose_rule{};
};

enum class ObfmLeadDisciplineReason : std::uint8_t
{
    PreviousObservationMissing = 0U,
    BehaviorSwitchNotAdmitted = 1U,
    LiftClassificationInvalid = 2U,
    NoseClassificationInvalid = 3U,
    ClassificationStateUnavailable = 4U,
    RulesAgree = 5U,
    LiftPureAllowsTerminalPull = 6U,
    LiftLeadAllowsTerminalPull = 7U,
    LiftLagDisagreementWithholdsTerminalPull = 8U
};

const char* ObfmLeadDisciplineReasonLabel(
    ObfmLeadDisciplineReason reason) noexcept;

struct ObfmLeadDisciplineReceipt
{
    bool previous_observation_present = false;
    bool evidence_evaluated = false;
    // These are complementary policy choices, not command writers.  Exactly
    // one is true on every input, including normal missing/untrusted evidence.
    bool preserve_terminal_tracking = true;
    bool withhold_terminal_pull = false;
    std::uint32_t decision_count = 1U;
    // Diagnostic only: a producer-declared admitted/valid receipt contradicted
    // its own classification contract.  It never suppresses the bounded base
    // or terminal command selected by the caller.
    bool producer_contract_contradiction = false;
    ObfmLeadDisciplineReason reason =
        ObfmLeadDisciplineReason::PreviousObservationMissing;
};

// Exact production-on G11 consumer gate.  All uncertainty preserves today's
// terminal-tracking behavior; only admitted, valid rule disagreement with the
// lift rule reading LAG withholds the terminal pull.
void EvaluateObfmLeadDiscipline(
    const ObfmLeadDisciplineInput& input,
    ObfmLeadDisciplineReceipt& output) noexcept;

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
