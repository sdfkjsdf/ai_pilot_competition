#include "LadyLuck/guidance/habfm/HabfmMergeEvidenceProvider.hpp"

#include <cmath>

namespace
{

bool FiniteOptional(
    const LadyLuck::HabfmOptionalScalar& value) noexcept
{
    return std::isfinite(value.value);
}

bool FiniteMergeIntent(
    const LadyLuck::HabfmMergeIntentEvidence& evidence) noexcept
{
    return std::isfinite(evidence.adversary_speed_mps)
        && std::isfinite(evidence.speed_error_bound_mps)
        && FiniteOptional(evidence.corner_speed_lower_mps)
        && FiniteOptional(evidence.corner_speed_upper_mps);
}

void Fail(
    LadyLuck::HabfmActiveCoreInputs& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::HabfmActiveCoreInputs{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{

HabfmMergeEvidenceProvider::HabfmMergeEvidenceProvider() noexcept
{
    Reset();
}

void HabfmMergeEvidenceProvider::Reset() noexcept
{
    frontal_pass_tracker_.Reset();
}

void HabfmMergeEvidenceProvider::Build(
    const DogfightGeometryFrame& frame,
    const HabfmMergeEvidenceProviderInputs& inputs,
    HabfmActiveCoreInputs& output,
    Status& status) noexcept
{
    output = HabfmActiveCoreInputs{};
    status = Status{};

    // These rows were already produced before _habfm_supply_merge_evidence in
    // Python. Reject a non-finite typed carrier before composing later rows so
    // NaN cannot become an availability sentinel in HabfmActiveCoreInputs.
    if (!FiniteOptional(inputs.capability_n_max_g)
        || !FiniteMergeIntent(inputs.merge_intent)
        || !std::isfinite(inputs.merge_speed_floor.floor_speed_mps))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    HabfmActiveCoreInputs candidate{};
    candidate.capability_n_max_g = inputs.capability_n_max_g;
    candidate.capability_n_max_admitted =
        inputs.capability_n_max_admitted;
    candidate.merge_intent = inputs.merge_intent;

    // Python order 1: publish the caller-resolved G17 floor unchanged.
    candidate.merge_speed_floor = inputs.merge_speed_floor;

    // Python order 2: the pure merge profile owns only the separation-policy
    // compression bit; this provider does not latch or reinterpret it.
    const Result<HabfmMergeProfileSelection> selection =
        SelectHabfmMergeProfile(frame);
    if (!selection.ok())
    {
        Fail(output, status, selection.status.code);
        return;
    }
    candidate.merge_separation_policy.admitted = true;
    candidate.merge_separation_policy.compress =
        selection.value.profile == HabfmCircleProfile::OneCircle;

    // Python order 3: update the maneuver-persistent frontal-pass side latch
    // only when the horizontal merge geometry owns this frame.  The existing
    // 3-D MergeApproach owner consumes neither entry side nor FrontalPass; a
    // singular horizontal geometry therefore has the same release semantics
    // as a normal FrontalPass non-admission and must not invent a side.
    if (inputs.three_dimensional_merge_required)
    {
        frontal_pass_tracker_.Reset();
    }
    else
    {
        FrontalPassEvidence frontal_pass{};
        Status frontal_pass_status{};
        frontal_pass_tracker_.Update(
            frame,
            inputs.frontal_pass_fallback_side_sign,
            frontal_pass,
            frontal_pass_status);
        if (!frontal_pass_status.ok())
        {
            Fail(output, status, frontal_pass_status.code);
            return;
        }
        if (frontal_pass.admitted)
        {
            candidate.frontal_pass.admitted = true;
            candidate.frontal_pass.safe_abeam_m = frontal_pass.safe_abeam_m;
            candidate.frontal_pass.compressed_abeam_m =
                frontal_pass.compressed_abeam_m;
            candidate.frontal_pass.side_sign = frontal_pass.side_sign;
        }
    }

    // Python order 4 for the supplies represented by HabfmActiveCoreInputs:
    // VerticalRoom is evaluated after FrontalPass and reads the same corner,
    // radius, hard-deck, offense range, and already-published speed floor.
    HabfmVerticalRoomInputs vertical_room_inputs{};
    vertical_room_inputs.gate = inputs.vertical_room_gate;
    vertical_room_inputs.evidence_present = inputs.vertical_excess_present;
    vertical_room_inputs.evidence = inputs.vertical_excess;
    vertical_room_inputs.corner_speed_mps = inputs.corner_speed_mps;
    vertical_room_inputs.corner_admitted = inputs.corner_admitted;
    vertical_room_inputs.turn_radius_m = inputs.turn_radius_m;
    vertical_room_inputs.turn_radius_admitted = inputs.turn_radius_admitted;
    vertical_room_inputs.hard_deck_margin_m =
        inputs.hard_deck_margin_m.has_value
            ? inputs.hard_deck_margin_m.value
            : 0.0;
    vertical_room_inputs.hard_deck_margin_finite =
        inputs.hard_deck_margin_m.has_value
        && std::isfinite(inputs.hard_deck_margin_m.value);
    vertical_room_inputs.horizontal_range_m = frame.own_offense.range_m;
    vertical_room_inputs.horizontal_range_finite =
        std::isfinite(frame.own_offense.range_m);
    vertical_room_inputs.floor_speed_mps.has_value =
        inputs.merge_speed_floor.admitted;
    vertical_room_inputs.floor_speed_mps.value =
        inputs.merge_speed_floor.admitted
            ? inputs.merge_speed_floor.floor_speed_mps
            : 0.0;
    vertical_room_inputs.floor_admitted =
        inputs.merge_speed_floor.admitted;

    Status vertical_room_status{};
    EvaluateHabfmVerticalRoom(
        vertical_room_inputs,
        candidate.merge_vertical_room,
        vertical_room_status);
    if (!vertical_room_status.ok())
    {
        // Do not roll back FrontalPassTracker: Python updates that owner before
        // this later helper call, and an integration fault preserves that order.
        Fail(output, status, vertical_room_status.code);
        return;
    }

    output = candidate;
}

} // namespace LadyLuck
