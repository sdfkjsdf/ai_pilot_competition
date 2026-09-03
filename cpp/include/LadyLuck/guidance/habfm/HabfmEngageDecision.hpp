#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"

#include <cstdint>

namespace LadyLuck
{

enum class HabfmEngageDecisionState : std::uint8_t
{
    Engage = 0U,
    AvoidPass = 1U
};

enum class HabfmEngageDecisionReason : std::uint8_t
{
    GateDisabled = 0U,
    EnergyDeficitNotProven = 1U,
    ClosingNotProven = 2U,
    EgressRangeCompleteReengage = 3U,
    AvoidPassClosingDeficit = 4U,
    AvoidPassLatched = 5U
};

enum class HabfmEngageReferenceReason : std::uint8_t
{
    None = 0U,
    Complete = 1U,
    CorridorPending = 2U,
    FloorPending = 3U,
    CorridorAndFloorPending = 4U
};

struct HabfmEngageDecisionInput
{
    ControlFrameIdentity frame_identity{};
    bool gate_enabled = false;
    double adversary_range_m = 0.0;
    double reengage_range_m = 0.0;
    double closing_speed_mps = 0.0;
    bool merge_selection_present = false;
    HabfmMergeProfileSelection merge_selection{};
    HabfmFrontalPassSupply frontal_pass{};
    TacticalSpeedFloorSample speed_floor{};
};

// Command-neutral one-tick receipt. The selected post-root Task is the sole
// command writer; this object only carries the Python latch decision and its
// independently optional CROSS and EXTEND references.
struct HabfmEngageDecisionReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool active = false;
    HabfmEngageDecisionState decision = HabfmEngageDecisionState::Engage;
    HabfmEngageDecisionReason reason =
        HabfmEngageDecisionReason::GateDisabled;
    bool pass_corridor_present = false;
    double pass_abeam_m = 0.0;
    std::int32_t pass_side_sign = 0;
    bool hold_current_course = false;
    bool speed_floor_present = false;
    double speed_floor_mps = 0.0;
    HabfmEngageReferenceReason reference_reason =
        HabfmEngageReferenceReason::None;
};

class HabfmEngageDecisionLatch final
{
public:
    HabfmEngageDecisionLatch() noexcept = default;

    void Reset() noexcept;
    void Update(
        const HabfmEngageDecisionInput& input,
        HabfmEngageDecisionReceipt& output,
        Status& status) noexcept;

private:
    HabfmEngageDecisionState decision_ = HabfmEngageDecisionState::Engage;
};

enum class HabfmAvoidPassLeg : std::uint8_t
{
    None = 0U,
    Cross = 1U,
    Extend = 2U
};

struct HabfmAvoidPassOverlayInput
{
    DogfightGeometryFrame frame{};
    ControlIntent upstream_intent{};
    HabfmEngageDecisionReceipt decision{};
    std::uint32_t modifier_writer_id = ControlIntentWriterNone;
};

struct HabfmAvoidPassOverlayReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool applicable = false;
    bool modified = false;
    HabfmAvoidPassLeg leg = HabfmAvoidPassLeg::None;
    std::uint32_t upstream_writer_id = ControlIntentWriterNone;
    std::uint32_t published_writer_id = ControlIntentWriterNone;
    ControlIntent candidate{};
};

// Pure d90 _apply_engage_decision_overlay materializer. It changes raw aim and
// speed guidance only; Route5, TECS/CIS and AutoGCAS remain downstream owners.
void BuildHabfmAvoidPassOverlay(
    const HabfmAvoidPassOverlayInput& input,
    HabfmAvoidPassOverlayReceipt& output,
    Status& status) noexcept;

} // namespace LadyLuck
