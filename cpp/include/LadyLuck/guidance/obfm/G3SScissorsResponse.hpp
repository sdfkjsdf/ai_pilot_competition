#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class G3ScissorsPhase : std::uint8_t
{
    None = 0U,
    GapBuild = 1U,
    Egress = 2U
};

enum class G3ScissorsReleaseReason : std::uint8_t
{
    None = 0U,
    OutsideEngagementBand = 1U,
    EgressNoLongerOpening = 2U
};

// Minimal fixed-state owner for a measured horizontal scissors reversal.
// A reversal is an opposing target-course run that sweeps the smallest
// official gun-cone angle. Once selected, writer 16 alone owns GAP_BUILD and
// EGRESS. It consumes no overshoot forecast, escape-window model, energy
// branch, or embedded GCAS predictor.
struct G3ScissorsState
{
    bool identity_ready = false;
    std::uint64_t episode_epoch = 0U;
    std::int32_t target_plane_id = -1;
    bool frame_seen = false;
    std::uint64_t last_frame_index = 0U;

    bool previous_course_ready = false;
    double previous_course_rad = 0.0;
    std::int32_t current_turn_sign = 0;
    double current_turn_sweep_rad = 0.0;
    bool previous_run_qualified = false;
    bool current_reversal_reported = false;
    G3ScissorsPhase phase = G3ScissorsPhase::None;
};

struct G3ScissorsReceipt
{
    bool evaluated = false;
    bool selected = false;
    bool entered = false;
    bool transitioned_to_egress = false;
    bool released = false;
    bool reversal_observed = false;
    bool own_ahead = false;
    G3ScissorsPhase phase = G3ScissorsPhase::None;
    G3ScissorsReleaseReason release_reason = G3ScissorsReleaseReason::None;
    std::uint32_t writer_id = ControlIntentWriterNone;
};

class G3ScissorsOwner final
{
public:
    G3ScissorsOwner() noexcept;

    void Reset() noexcept;
    void Evaluate(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3ScissorsReceipt& receipt) noexcept;
    void Halt() noexcept;

private:
    void ResetObservation(const DogfightGeometryFrame& frame) noexcept;
    bool ObserveReversal(const DogfightGeometryFrame& frame) noexcept;
    bool OwnAhead(const DogfightGeometryFrame& frame) const noexcept;
    bool InEngagementBand(const DogfightGeometryFrame& frame) const noexcept;
    bool CanBuildGap(const DogfightGeometryFrame& frame) const noexcept;
    void BuildGapCommand(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3ScissorsReceipt& receipt) const noexcept;
    void BuildEgressCommand(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3ScissorsReceipt& receipt) const noexcept;

    G3ScissorsState state_{};
};

static_assert(std::is_standard_layout<G3ScissorsState>::value,
              "G3 scissors state must remain standard-layout.");
static_assert(std::is_trivially_copyable<G3ScissorsState>::value,
              "G3 scissors state must remain fixed-storage.");
static_assert(std::is_standard_layout<G3ScissorsReceipt>::value,
              "G3 scissors receipt must remain standard-layout.");
static_assert(std::is_trivially_copyable<G3ScissorsReceipt>::value,
              "G3 scissors receipt must remain fixed-storage.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
