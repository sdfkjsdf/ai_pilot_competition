#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

// Dynamic provenance strings are excluded from the 60 Hz path.  Each frozen
// feature is valid only as an OFF/OFF or ON/exact-provenance pair.
struct DbfmBreakMarginActivation
{
    bool break_margin_enabled = false;
    bool exact_break_margin_provenance = false;
    bool phase_graded_enabled = false;
    bool exact_phase_graded_provenance = false;
};

// Verified my_submission.py configuration at add/main@45abc9f6.
constexpr DbfmBreakMarginActivation DbfmBreakMarginProductionActivation{
    true, true, true, true};

struct DbfmBreakMarginTurnCapabilityReceipt
{
    ControlFrameIdentity frame_identity{};
    bool admitted = false;
    bool n_max_available = false;
    double n_max_g = 0.0;
    bool physical_authority = false;
    bool fixed_command_bound = false;
};

enum class DbfmBreakOfficialThreatReason : std::uint8_t
{
    NotEvaluated = 0U,
    DamageEvidenceUnavailable = 1U,
    OfficialThreatClear = 2U,
    OfficialThreatNotScratch = 3U,
    OfficialScratchDemotedByPhaseGrading = 4U,
    OfficialThreatSelected = 5U
};

struct DbfmBreakOfficialThreatObservation
{
    bool evaluated = false;
    bool damage_evidence_available = false;
    bool official_gun_threat = false;
    bool scratch_evaluated = false;
    bool official_scratch = false;
    bool phase_graded_demoted = false;
    DbfmBreakOfficialThreatReason reason =
        DbfmBreakOfficialThreatReason::NotEvaluated;
};

enum class DbfmBreakMarginReason : std::uint8_t
{
    NotEvaluated = 0U,
    CapabilityUnavailable = 1U,
    RangeUnavailable = 2U,
    ScoringRangeUnavailable = 3U,
    OwnSpeedUnavailable = 4U,
    ArithmeticUnavailable = 5U,
    AttackerInsideScoringRange = 6U,
    AttackerNotClosing = 7U,
    SeparationUnavailable = 8U,
    ScoringCrossingBeforeOwnTurn = 9U,
    OwnTurnCompletesBeforeScoring = 10U
};

// Command-neutral copy of BreakMarginSample with typed availability.  The
// observation can select the existing BREAK leaf but cannot publish an aim,
// Nz/body-rate, surface, thrust, estimator, or aircraft-response value.
struct DbfmBreakMarginObservation
{
    bool evaluated = false;
    bool admitted = false;
    bool margin_break = false;
    DbfmBreakMarginReason reason = DbfmBreakMarginReason::NotEvaluated;
    bool scoring_gap_available = false;
    double scoring_gap_m = 0.0;
    bool own_speed_available = false;
    double own_speed_mps = 0.0;
    bool required_turn_available = false;
    double required_turn_rad = 0.0;
    bool available_turn_rate_available = false;
    double available_turn_rate_rad_s = 0.0;
    bool time_to_score_available = false;
    double time_to_score_s = 0.0;
    bool time_to_face_available = false;
    double time_to_face_s = 0.0;
};

enum class DbfmBreakSelectionSource : std::uint8_t
{
    None = 0U,
    OfficialGunThreat = 1U,
    PredictiveBreakMargin = 2U
};

enum class DbfmBreakMarginDecisionReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    HigherPriorityRootGun = 2U,
    OfficialEvidenceUnavailable = 3U,
    OfficialGunThreatSelected = 4U,
    BreakMarginDisabled = 5U,
    BreakMarginNotSelected = 6U,
    BreakMarginSelected = 7U
};

// Single typed Service receipt for the DBFM BREAK Condition.  Safety and the
// Root Gun owner remain structurally senior.  Within DBFM this Condition is
// evaluated before ALTITUDE_SEPARATED, ESCAPE, EXTEND, and HARD_TURN.
struct DbfmBreakMarginReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool break_selected = false;
    DbfmBreakSelectionSource selection_source =
        DbfmBreakSelectionSource::None;
    DbfmBreakMarginDecisionReason reason =
        DbfmBreakMarginDecisionReason::NotEvaluated;
    DbfmBreakOfficialThreatObservation official_threat{};
    DbfmBreakMarginObservation margin{};
};

// `higher_priority_root_gun_selected` models Root official/prefire preemption.
// It short-circuits before activation or frame evidence is inspected.  Passing
// false permits focused characterization of DbfmCoreTree's local Service,
// including its exact phase-graded scratch interaction.
void EvaluateDbfmBreakMargin(
    const DogfightGeometryFrame& frame,
    bool dbfm_owner_selected,
    bool higher_priority_root_gun_selected,
    const DbfmBreakMarginActivation& activation,
    const DbfmBreakMarginTurnCapabilityReceipt& capability,
    DbfmBreakMarginReceipt& output,
    Status& status) noexcept;

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
