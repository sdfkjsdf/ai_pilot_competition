#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class RollDefenseObservationReason : std::uint8_t
{
    StateNotObservable = 0U,
    AlternationNotEstablished = 1U,
    ReversalNotResolved = 2U,
    ForwardSpeedDropNotResolved = 3U,
    EnergyStandingNotResolved = 4U,
    RollingDefenseSignatureResolved = 5U,
    ObserverContractRejected = 6U
};

const char* RollDefenseObservationReasonLabel(
    RollDefenseObservationReason reason) noexcept;

struct RollDefenseObservation
{
    bool valid = false;
    RollDefenseObservationReason reason =
        RollDefenseObservationReason::StateNotObservable;
    bool admitted = false;
    std::int32_t phase_sign = 0;
    std::uint32_t phase_count = 0U;
    bool standing_resolved = false;
    double roll_envelope_m = 0.0;
    bool envelope_mature = false;
    double sweep_low_m = 0.0;
    double sweep_high_m = 0.0;
    bool s3_relative = false;
    bool s2_per_phase = false;
};

// Exact allocation-free port of d90 obfm_barrel_roll_counter.py's
// command-neutral RollDefenseObserver. It owns no ControlIntent writer.
class RollDefenseObserver final
{
public:
    RollDefenseObserver() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        RollDefenseObservation& output,
        Status& status) noexcept;

private:
    static constexpr std::size_t AlternationMinimum = 3U;

    void ResetEpisode(std::int32_t sign) noexcept;

    std::array<std::int32_t, AlternationMinimum> phase_signs_{};
    std::array<double, AlternationMinimum> phase_start_alt_m_{};
    std::array<double, AlternationMinimum> phase_start_forward_mps_{};
    std::array<double, AlternationMinimum> phase_start_forward_bound_mps_{};
    // Bit 0 => +1 turn observed, bit 1 => -1 turn observed.
    std::array<std::uint8_t, AlternationMinimum> phase_turn_bits_{};
    std::size_t phase_count_ = 0U;

    std::array<double, AlternationMinimum> completed_excursions_m_{};
    std::size_t completed_excursion_count_ = 0U;

    bool previous_course_present_ = false;
    double previous_course_rad_ = 0.0;
    double previous_course_bound_rad_ = 0.0;

    std::uint64_t tick_ = 0U;
    std::uint64_t phase_start_tick_ = 0U;
    double longest_excursion_m_ = 0.0;
    std::uint64_t longest_phase_ticks_ = 0U;
    std::int32_t quarantine_sign_ = 0;
};

bool BarrelRollCounterWithholdsPull(
    const RollDefenseObservation* observation) noexcept;

static_assert(
    std::is_trivially_copyable<RollDefenseObservation>::value,
    "roll-defense observation must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
