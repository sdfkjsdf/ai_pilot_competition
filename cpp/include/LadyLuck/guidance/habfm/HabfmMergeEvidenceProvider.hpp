#pragma once

#include "LadyLuck/guidance/habfm/FrontalPassObserver.hpp"
#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"

#include <cstdint>

namespace LadyLuck
{

// Caller-resolved evidence entering the d90 HABFM merge-supply row. This
// provider does not own chart, envelope, safety, or BehaviorTree admission.
struct HabfmMergeEvidenceProviderInputs
{
    HabfmOptionalScalar capability_n_max_g{};
    bool capability_n_max_admitted = false;
    HabfmMergeIntentEvidence merge_intent{};
    HabfmSpeedFloorSupply merge_speed_floor{};
    bool three_dimensional_merge_required = false;

    // Exact _habfm_entry_side_fallback result: the active HABFM turn-side
    // latch when present, otherwise the caller's frame-derived entry side.
    std::int32_t frontal_pass_fallback_side_sign = 0;

    bool vertical_excess_present = false;
    HabfmVerticalExcessEvidence vertical_excess{};
    HabfmOptionalScalar corner_speed_mps{};
    bool corner_admitted = false;
    HabfmOptionalScalar turn_radius_m{};
    bool turn_radius_admitted = false;
    HabfmOptionalScalar hard_deck_margin_m{};
    HabfmVerticalRoomGate vertical_room_gate{};
};

// Allocation-free owner of the maneuver-persistent FrontalPass side latch.
// It composes existing evidence modules in the Python supplier's order and
// publishes value-only HabfmActiveCoreInputs; it never emits a command.
class HabfmMergeEvidenceProvider final
{
public:
    HabfmMergeEvidenceProvider() noexcept;

    void Reset() noexcept;
    void Build(
        const DogfightGeometryFrame& frame,
        const HabfmMergeEvidenceProviderInputs& inputs,
        HabfmActiveCoreInputs& output,
        Status& status) noexcept;

private:
    FrontalPassTracker frontal_pass_tracker_{};
};

} // namespace LadyLuck
