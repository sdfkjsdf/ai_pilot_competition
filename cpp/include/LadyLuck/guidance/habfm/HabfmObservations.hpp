#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>

namespace LadyLuck
{
// Value-only optional used at the admitted E-M chart seam.  A stored value is
// deliberately ignored while has_value is false, matching Python's None and
// boolean short-circuit behavior.
struct HabfmOptionalScalar
{
    bool has_value = false;
    double value = 0.0;
};

enum class HabfmObservationProvenance : std::uint8_t
{
    MergeIntent = 0U,
    PursuitCourse = 1U,
    VerticalExcess = 2U,
    CheckpointCue = 3U
};

enum class HabfmMergeIntentState : std::uint8_t
{
    EnergyFightProven = 0U,
    NotProven = 1U
};

enum class HabfmMergeIntentReason : std::uint8_t
{
    CornerSpeedEvidenceAbsentDefaultTurnFightPosture = 0U,
    ObservedSpeedExceedsCornerIntervalAtEveryMass = 1U,
    ObservedSpeedWithinOrBelowCornerInterval = 2U
};

struct HabfmMergeIntentEvidence
{
    double adversary_speed_mps = 0.0;
    double speed_error_bound_mps = 0.0;
    HabfmOptionalScalar corner_speed_lower_mps{};
    HabfmOptionalScalar corner_speed_upper_mps{};
    bool evidence_admitted = false;
    HabfmMergeIntentState intent = HabfmMergeIntentState::NotProven;
    HabfmMergeIntentReason reason =
        HabfmMergeIntentReason::CornerSpeedEvidenceAbsentDefaultTurnFightPosture;
    HabfmObservationProvenance provenance =
        HabfmObservationProvenance::MergeIntent;
};

enum class HabfmPursuitCourseState : std::uint8_t
{
    LeadProven = 0U,
    LagProven = 1U,
    WithinResolution = 2U
};

struct HabfmPursuitCourseEvidence
{
    double lead_metric = 0.0;
    double resolution_bound = 0.0;
    HabfmPursuitCourseState state =
        HabfmPursuitCourseState::WithinResolution;
    bool transverse_defined = false;
    HabfmObservationProvenance provenance =
        HabfmObservationProvenance::PursuitCourse;
};

enum class HabfmVerticalExcessState : std::uint8_t
{
    AboveCornerProven = 0U,
    BelowCornerProven = 1U,
    WithinResolution = 2U
};

struct HabfmVerticalExcessEvidence
{
    double own_speed_mps = 0.0;
    HabfmOptionalScalar corner_speed_mps{};
    HabfmOptionalScalar excess_mps{};
    double speed_error_bound_mps = 0.0;
    bool evidence_admitted = false;
    HabfmVerticalExcessState state =
        HabfmVerticalExcessState::WithinResolution;
    HabfmObservationProvenance provenance =
        HabfmObservationProvenance::VerticalExcess;
};

enum class HabfmCheckpointCueState : std::uint8_t
{
    Winning = 0U,
    Neutral = 1U,
    Losing = 2U
};

struct HabfmCheckpointCueEvidence
{
    HabfmCheckpointCueState cue = HabfmCheckpointCueState::Neutral;
    bool angle_favourable = false;
    bool angle_unfavourable = false;
    double delta_specific_energy_m = 0.0;
    double evidence_band_m = 0.0;
    bool energy_deficit_proven = false;
    HabfmObservationProvenance provenance =
        HabfmObservationProvenance::CheckpointCue;
};

struct HabfmObservationInputs
{
    HabfmOptionalScalar adversary_corner_speed_lower_mps{};
    HabfmOptionalScalar adversary_corner_speed_upper_mps{};
    bool adversary_corner_interval_admitted = false;
    HabfmOptionalScalar own_corner_speed_upper_mps{};
    bool own_corner_interval_admitted = false;
};

struct HabfmPreTaskObservations
{
    HabfmMergeIntentEvidence merge_intent{};
    HabfmPursuitCourseEvidence pursuit_course{};
    HabfmVerticalExcessEvidence vertical_excess{};
};

Result<double> SpecificEnergyM(
    double altitude_m,
    double speed_mps) noexcept;

Result<double> EnergyEvidenceBandM(
    double own_altitude_m,
    double own_speed_mps,
    double adversary_altitude_m,
    double adversary_speed_mps) noexcept;

Result<HabfmMergeIntentEvidence> EvaluateMergeIntent(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& corner_speed_lower_mps,
    const HabfmOptionalScalar& corner_speed_upper_mps,
    bool corner_interval_admitted) noexcept;

Result<HabfmPursuitCourseEvidence> EvaluatePursuitCourse(
    const DogfightGeometryFrame& frame) noexcept;

Result<HabfmVerticalExcessEvidence> EvaluateVerticalExcess(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& corner_speed_mps,
    bool corner_admitted) noexcept;

// Preserve the active Python caller order: merge -> pursuit -> vertical.  The
// bundle is published only after all three value-only observations succeed.
Result<HabfmPreTaskObservations> EvaluateHabfmPreTaskObservations(
    const DogfightGeometryFrame& frame,
    const HabfmObservationInputs& inputs) noexcept;

// The cue remains a per-leg continuous observation and is intentionally not
// folded into the pre-task bundle.
Result<HabfmCheckpointCueEvidence> EvaluateCheckpointCue(
    const DogfightGeometryFrame& frame) noexcept;
}
