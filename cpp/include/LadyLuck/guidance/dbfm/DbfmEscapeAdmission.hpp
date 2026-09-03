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

struct DbfmEscapeAdmissionActivation
{
    bool phase_graded_enabled = false;
    bool exact_phase_graded_provenance = false;
    bool entry_forecast_enabled = false;
    bool exact_entry_forecast_provenance = false;
};

// Verified add/main@45abc runtime configuration: phase grading is promoted,
// while DoctrineBtIntegration does not pass EscapeEntryForecastConfig and the
// stateful pre-damage producer therefore remains at its default OFF setting.
constexpr DbfmEscapeAdmissionActivation
    DbfmEscapeAdmissionProductionActivation{true, true, false, false};

// Explicit characterization activation for the implemented but production-OFF
// entry producer.  This is not a production-promotion claim.
constexpr DbfmEscapeAdmissionActivation
    DbfmEscapeAdmissionEntryCharacterizationActivation{
        true, true, true, true};

struct DbfmEscapeTurnCapabilityReceipt
{
    ControlFrameIdentity frame_identity{};
    bool admitted = false;
    bool n_max_available = false;
    double n_max_g = 0.0;
    bool physical_authority = false;
    bool fixed_command_bound = false;
};

enum class DbfmOfficialScratchReason : std::uint8_t
{
    NotEvaluated = 0U,
    DamageNotPositive = 1U,
    DamageValueUnavailable = 2U,
    NoOfficialPhaseMatch = 3U,
    HigherPriorityGunBand = 4U,
    ScratchBandMatched = 5U
};

struct DbfmOfficialScratchReceipt
{
    bool evaluated = false;
    bool scratch_matched = false;
    DbfmOfficialScratchReason reason =
        DbfmOfficialScratchReason::NotEvaluated;
};

enum class DbfmDefenseUrgencyReason : std::uint8_t
{
    NotEvaluated = 0U,
    CapabilityUnavailable = 1U,
    OwnSpeedUnavailable = 2U,
    OpponentSpeedUnavailable = 3U,
    RangeUnavailable = 4U,
    SeparationUnavailable = 5U,
    ArithmeticUnavailable = 6U,
    AttackerInsideOwnTurnCircle = 7U,
    AttackerOutsideOwnTurnCircle = 8U
};

struct DbfmDefenseUrgencyReceipt
{
    bool evaluated = false;
    bool admitted = false;
    bool urgent_available = false;
    bool urgent = false;
    DbfmDefenseUrgencyReason reason =
        DbfmDefenseUrgencyReason::NotEvaluated;
    bool required_turn_available = false;
    double required_turn_rad = 0.0;
    bool time_to_face_available = false;
    double time_to_face_s = 0.0;
    bool attacker_reach_time_available = false;
    double attacker_reach_time_s = 0.0;
    bool own_speed_available = false;
    double own_speed_mps = 0.0;
    bool opponent_speed_available = false;
    double opponent_speed_mps = 0.0;
};

enum class DbfmEscapeEligibilityReason : std::uint8_t
{
    NotEvaluated = 0U,
    CapabilityOrKinematicsUnavailable = 1U,
    DefenseUrgent = 2U,
    SpeedBandArithmeticUnavailable = 3U,
    SpeedAdvantageNotProven = 4U,
    Selected = 5U
};

struct DbfmEscapeEligibilityReceipt
{
    bool evaluated = false;
    bool selected = false;
    DbfmEscapeEligibilityReason reason =
        DbfmEscapeEligibilityReason::NotEvaluated;
    DbfmDefenseUrgencyReceipt urgency{};
    bool speed_evidence_available = false;
    double own_speed_mps = 0.0;
    double opponent_speed_mps = 0.0;
    double speed_delta_mps = 0.0;
    double speed_evidence_band_mps = 0.0;
};

enum class DbfmScratchEntryForecastReason : std::uint8_t
{
    NotEvaluated = 0U,
    ProductionDisabled = 1U,
    FrameStateUnavailable = 2U,
    ScratchPhaseNotActive = 3U,
    DegenerateSeparation = 4U,
    ArithmeticUnavailable = 5U,
    ConeSweepMemoryNotEstablished = 6U,
    NonMonotonicTime = 7U,
    EntryForming = 8U,
    EntryNotForming = 9U
};

struct DbfmScratchEntryForecastReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    bool entry_forming = false;
    bool scratch_active = false;
    bool cone_satisfied = false;
    bool cone_closing_proven = false;
    bool range_satisfied = false;
    bool range_closing_proven = false;
    bool time_to_cone_available = false;
    double time_to_cone_s = 0.0;
    bool time_to_range_available = false;
    double time_to_range_s = 0.0;
    bool causal_history_reset = false;
    DbfmScratchEntryForecastReason reason =
        DbfmScratchEntryForecastReason::NotEvaluated;
};

enum class DbfmEscapeSelectionSource : std::uint8_t
{
    None = 0U,
    DamagePositiveScratch = 1U,
    PreDamageEntryForecast = 2U
};

enum class DbfmEscapeAdmissionReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    HigherPriorityBranch = 2U,
    ProductionDisabled = 3U,
    NotDamagePositiveScratch = 4U,
    DamageScratchNotEligible = 5U,
    DamageScratchSelected = 6U,
    EntryForecastNotForming = 7U,
    EntryForecastNotEligible = 8U,
    EntryForecastSelected = 9U
};

// Single typed Service receipt deciding the ESCAPE Condition.  It is tactical
// admission evidence only; it contains no aim, speed command, body-rate/Nz,
// surface, thrust, estimator-truth, or aircraft-response authority.  The
// existing DbfmEscapeEnergy module remains a post-selection guidance overlay.
struct DbfmEscapeAdmissionReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool selected = false;
    DbfmEscapeSelectionSource selection_source =
        DbfmEscapeSelectionSource::None;
    DbfmEscapeAdmissionReason reason =
        DbfmEscapeAdmissionReason::NotEvaluated;
    DbfmOfficialScratchReceipt official_scratch{};
    DbfmEscapeEligibilityReceipt damage_scratch_eligibility{};
    DbfmScratchEntryForecastReceipt entry_forecast{};
    DbfmEscapeEligibilityReceipt entry_eligibility{};
};

class DbfmEscapeAdmissionEvaluator
{
public:
    void Reset() noexcept;

    // Exact selector ownership seam:
    // - !dbfm_owner_selected resets state like Python mode_state Service;
    // - DBFM owner but !escape_branch_reached preserves state and does not
    //   inspect evidence because BREAK/ALTITUDE_SEPARATED already won;
    // - only a reached ESCAPE branch evaluates this receipt.
    void Evaluate(
        const DogfightGeometryFrame& frame,
        bool dbfm_owner_selected,
        bool escape_branch_reached,
        const DbfmEscapeAdmissionActivation& activation,
        const DbfmEscapeTurnCapabilityReceipt& capability,
        DbfmEscapeAdmissionReceipt& output,
        Status& status) noexcept;

private:
    bool history_valid_ = false;
    ControlFrameIdentity history_frame_identity_{};
    std::int32_t history_own_plane_id_ = -1;
    std::int32_t history_target_plane_id_ = -1;
    std::uint64_t history_target_frame_index_ = 0U;
    double previous_t_s_ = 0.0;
    double previous_ata_rad_ = 0.0;

    void UpdateEntryForecast(
        const DogfightGeometryFrame& frame,
        DbfmScratchEntryForecastReceipt& output,
        Status& status) noexcept;
};

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
