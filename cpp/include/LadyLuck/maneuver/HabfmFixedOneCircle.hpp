#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"
#include "LadyLuck/maneuver/HabfmFixedOneCircleControlIntent.hpp"

#include <cstdint>
#include <string>

namespace LadyLuck
{
enum class HabfmFixedOneCircleLegStatus : std::uint8_t
{
    Running = 0U,
    MergePass = 1U
};

struct HabfmFixedOneCircleOutput
{
    HabfmFixedOneCircleLegStatus leg_status =
        HabfmFixedOneCircleLegStatus::Running;
    OptionalValue<TacticalCommand> command{};
    bool entered = false;
    bool merge_pass = false;
    std::int32_t side_sign = 0;
    double turn_progress_rad = 0.0;
    bool mode_recheck = false;
    OptionalValue<std::string> transition_reason{};
    std::uint64_t neutral_cue_streak = 0U;
    HabfmCheckpointCueEvidence checkpoint_cue{};
};

// Complete private lifecycle state.  A merge-pass return has already executed
// the Python Task on_exit equivalent, so Snapshot() is reset-seeded while the
// returned output retains the just-completed leg evidence.
struct HabfmFixedOneCircleSnapshot
{
    bool active = false;
    OptionalValue<std::int32_t> side_sign{};
    OptionalValue<double> previous_heading_rad{};
    double progress_rad = 0.0;
    OptionalValue<bool> previous_closing{};
};

// Fixed production ONE_CIRCLE raw reference: a latched horizontal lateral aim
// at current range, current measured speed, and the current NED-Down plane.
// FCS shaping, p/q/r, Nz, surfaces, thrust, envelope protection, and actual
// aircraft response remain downstream responsibilities.
Result<TacticalCommand> BuildHabfmFixedOneCircleCommand(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign);

class HabfmFixedOneCirclePolicy
{
public:
    HabfmFixedOneCirclePolicy() noexcept;

    void Reset() noexcept;
    HabfmFixedOneCircleSnapshot Snapshot() const noexcept;

    // This method owns only one fixed ONE_CIRCLE Task leg.  A caller-owned
    // Service must first run EvaluateHabfmPreTaskObservations and publish its
    // situation fields.  StepLeg does not choose a tactical branch, infer
    // HABFM admission, publish a Blackboard command, or perform same-frame
    // re-entry.  The caller/BT owns those operations.  The neutral-cue streak
    // is Blackboard-owned and therefore enters/leaves the Task explicitly;
    // it is not part of the private turn-memory Snapshot.  Not calling
    // StepLeg preserves all private state.
    Result<HabfmFixedOneCircleOutput> StepLeg(
        const DogfightGeometryFrame& frame,
        std::uint64_t blackboard_neutral_cue_streak = 0U);

private:
    Result<HabfmFixedOneCircleOutput> FailAndReset(
        const Status& status);

    bool active_ = false;
    std::int32_t side_sign_ = 0;
    bool previous_heading_valid_ = false;
    double previous_heading_rad_ = 0.0;
    double progress_rad_ = 0.0;
    bool previous_closing_valid_ = false;
    bool previous_closing_ = false;
};
}
