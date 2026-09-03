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

enum class G3RollCounterKind : std::uint8_t
{
    None = 0U,
    CounterBarrel = 1U
};

enum class G3RollCounterReleaseReason : std::uint8_t
{
    None = 0U,
    VerticalPhaseChanged = 1U,
    RangeOpening = 2U
};

// Minimal fixed-state owner for the observable opponent roll event.  It owns
// only: three alternating vertical phases, one current AimPoint command, and
// release on the next phase reversal or opening range.  Safety, gun defence,
// situation classification and downstream FCS authority remain upstream.
struct G3RollCounterState
{
    bool identity_ready = false;
    std::uint64_t episode_epoch = 0U;
    std::int32_t target_plane_id = -1;
    bool frame_seen = false;
    std::uint64_t last_frame_index = 0U;

    bool previous_course_ready = false;
    double previous_course_rad = 0.0;
    std::int32_t vertical_phase_sign = 0;
    std::uint8_t vertical_phase_count = 0U;
    std::uint8_t horizontal_turn_bits = 0U;
    double phase_start_altitude_m = 0.0;
    double last_vertical_excursion_m = 0.0;
    bool active = false;
    std::int32_t active_phase_sign = 0;
    G3RollCounterKind active_kind = G3RollCounterKind::None;
    bool rolling_handoff_ready = false;
    bool rolling_active = false;
    bool rolling_release_pending = false;
    std::int32_t rolling_active_phase_sign = 0;
};

struct G3RollCounterReceipt
{
    bool evaluated = false;
    bool selected = false;
    bool released = false;
    bool new_vertical_phase = false;
    G3RollCounterReleaseReason release_reason =
        G3RollCounterReleaseReason::None;
    G3RollCounterKind kind = G3RollCounterKind::None;
    std::uint8_t vertical_phase_count = 0U;
    std::uint8_t horizontal_turn_bits = 0U;
    std::uint32_t writer_id = ControlIntentWriterNone;
};

struct G3CounterRollingScissorsReceipt
{
    bool evaluated = false;
    bool selected = false;
    bool released = false;
    bool handoff_consumed = false;
    bool own_pushed_ahead = false;
    double vertical_excursion_m = 0.0;
    std::uint32_t writer_id = ControlIntentWriterNone;
};

class G3RollCounterOwner final
{
public:
    G3RollCounterOwner() noexcept;

    void Reset() noexcept;
    void Evaluate(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3RollCounterReceipt& receipt) noexcept;
    void EvaluateCounterRollingScissors(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3CounterRollingScissorsReceipt& receipt) noexcept;
    void HaltCounterBarrel() noexcept;
    void HaltCounterRollingScissors() noexcept;
    void Halt() noexcept;

private:
    void ResetObservation(
        const DogfightGeometryFrame& frame,
        std::int32_t initial_vertical_sign) noexcept;
    void BuildCommand(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3RollCounterReceipt& receipt) const noexcept;
    void BuildRollingScissorsCommand(
        const DogfightGeometryFrame& frame,
        ControlIntent& command,
        G3CounterRollingScissorsReceipt& receipt) const noexcept;
    bool OwnPushedAhead(const DogfightGeometryFrame& frame) const noexcept;

    G3RollCounterState state_{};
};

static_assert(std::is_standard_layout<G3RollCounterState>::value,
              "G3 roll-counter state must remain standard-layout.");
static_assert(std::is_trivially_copyable<G3RollCounterState>::value,
              "G3 roll-counter state must remain fixed-storage.");
static_assert(std::is_standard_layout<G3RollCounterReceipt>::value,
              "G3 roll-counter receipt must remain standard-layout.");
static_assert(std::is_trivially_copyable<G3RollCounterReceipt>::value,
              "G3 roll-counter receipt must remain fixed-storage.");
static_assert(
    std::is_trivially_copyable<G3CounterRollingScissorsReceipt>::value,
    "G3 counter-rolling-scissors receipt must remain fixed-storage.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
