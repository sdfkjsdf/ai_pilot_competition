#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"

#include <cstdint>

namespace LadyLuck
{
enum class HabfmVerticalRoomReason : std::uint8_t
{
    GateDisabled = 0U,
    VerticalEvidenceNotAdmitted = 1U,
    CornerLineageMismatch = 2U,
    CornerSpeedNotAdmitted = 3U,
    HorizontalRangeUnresolved = 4U,
    WithinResolutionLevelRoom = 5U,
    WithinFloorBandLevelRoom = 6U,
    TurnRadiusNotAdmitted = 7U,
    HardDeckMarginNotPositive = 8U,
    DiveDepthExceedsHardDeckMargin = 9U,
    IdentityNotResolvable = 10U,
    VerticalRoomAboveCorner = 11U,
    VerticalRoomBelowCorner = 12U
};

// Allocation-free representation of the Python provenance gate.  The runtime
// configuration layer resolves the exact provenance string once; the 60 Hz
// evaluator consumes only the resulting presence/match receipt.
struct HabfmVerticalRoomGate
{
    bool enabled = false;
    bool provenance_present = false;
    bool provenance_matches = false;
};

struct HabfmVerticalRoomInputs
{
    HabfmVerticalRoomGate gate{};
    bool evidence_present = false;
    HabfmVerticalExcessEvidence evidence{};
    HabfmOptionalScalar corner_speed_mps{};
    bool corner_admitted = false;
    HabfmOptionalScalar turn_radius_m{};
    bool turn_radius_admitted = false;
    double hard_deck_margin_m = 0.0;
    bool hard_deck_margin_finite = false;
    double horizontal_range_m = 0.0;
    bool horizontal_range_finite = false;
    HabfmOptionalScalar floor_speed_mps{};
    bool floor_admitted = false;
};

struct HabfmVerticalRoomReceipt
{
    bool admitted = false;
    HabfmVerticalRoomReason reason = HabfmVerticalRoomReason::GateDisabled;
    double vertical_offset_m = 0.0;
    HabfmOptionalScalar identity_depth_m{};
    HabfmOptionalScalar radius_clamp_m{};
    bool clamp_active_valid = false;
    bool clamp_active = false;
    HabfmOptionalScalar room_angle_rad{};
};

// Exact semantic port of habfm_vertical_room.merge_vertical_room.  Ordinary
// evidence refusal is an OK, unadmitted level-room receipt.  Only a malformed
// typed/configuration contract returns a negative Status.
void EvaluateHabfmVerticalRoom(
    const HabfmVerticalRoomInputs& inputs,
    HabfmVerticalRoomReceipt& output,
    Status& status) noexcept;
}
