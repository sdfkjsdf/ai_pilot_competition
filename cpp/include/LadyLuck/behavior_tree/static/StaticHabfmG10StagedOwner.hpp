#pragma once

#include "LadyLuck/guidance/g10/G10SecondUseAdmissionProvider.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseOwner.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseSelectionSupply.hpp"
#include "LadyLuck/guidance/habfm/HabfmH09AltitudeStorage.hpp"
#include "LadyLuck/guidance/habfm/HabfmTerminalControlIntentOwner.hpp"
#include "LadyLuck/guidance/obfm/G3SAdversaryCourseReversal.hpp"
#include "LadyLuck/guidance/obfm/PursuitOvershootForecast.hpp"
#include "LadyLuck/runtime/TacticalControlCore.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class StaticHabfmG10Disposition : std::uint8_t
{
    NotEvaluated = 0U,
    HabfmBaseSelected = 1U,
    G10SecondUseSelected = 2U,
    HabfmAvoidPassSelected = 3U
};

// All state that can advance while evaluating the optional G10 lifecycle.
// Prepare operates on a copy; only CommitPublished installs the copy.
struct StaticHabfmG10State
{
    guidance::g10::G10SecondUseSupplyProvider supply_provider{};
    guidance::g10::G10SecondUseAdmissionProvider admission_provider{};
    guidance::g10::G10SecondUseOwner owner{};
    guidance::obfm::AdversaryCourseReversalObserver reversal_observer{};
    guidance::obfm::PursuitOvershootForecast completed_forecast{};
    guidance::obfm::AdversaryReversalObservation completed_reversal{};
    bool completed_forecast_available = false;
    bool completed_reversal_available = false;
    bool last_descending_publication_available = false;
    ControlFrameIdentity last_descending_publication_identity{};
};

struct StaticHabfmG10Prepared
{
    ControlFrameIdentity frame_identity{};
    std::uint64_t captured_generation = 0U;
    bool prepare_attempted = false;
    bool next_state_ready = false;
    bool committed = false;
    StaticHabfmG10Disposition disposition =
        StaticHabfmG10Disposition::NotEvaluated;
    StatusCode optional_g10_status_code = StatusCode::Ok;
    StatusCode optional_h09_status_code = StatusCode::Ok;
    StatusCode optional_avoid_status_code = StatusCode::Ok;
    HabfmTerminalPreparedControlIntent habfm_base{};
    HabfmAvoidPassOverlayReceipt avoid_overlay{};
    guidance::g10::G10SecondUseSupply supply{};
    guidance::g10::G10SecondUseAdmissionReceipt admission{};
    guidance::g10::G10SecondUseOwnerReceipt owner{};
    guidance::habfm::HabfmH09AltitudeStorageAdmission h09_storage{};
    runtime::CurrentCisV4EnergyProjectionReceipt h09_projection{};
    guidance::habfm::HabfmH09ResidualClimbAllocation h09_allocation{};
    bool h09_projection_attempted = false;
    bool h09_active = false;
    ControlIntent selected_intent{};
    StaticHabfmG10State next_state{};
};

// HABFM-local fixed selector. It stages writer 4 first, then evaluates the
// existing AvoidPass latch, G10 lifecycle, and H09 energy-storage modifier on
// copied state. Optional nonselection never removes the current writer-4
// reference. No runtime XML, Blackboard, allocation, exception, or hidden
// publisher exists in this owner.
class StaticHabfmG10StagedOwner final
{
public:
    StaticHabfmG10StagedOwner() noexcept;

    void Reset() noexcept;
    void Observe(
        const HabfmTerminalControlIntentInput& input,
        HabfmTerminalControlIntentObservation& output,
        Status& status) const noexcept;
    void Prepare(
        const runtime::TacticalCommandBuildInput& tactical_input,
        runtime::ICurrentCisV4EnergyProjectionPort* projection_port,
        const HabfmTerminalControlIntentObservation& observation,
        StaticHabfmG10Prepared& output,
        Status& status) const noexcept;
    void ValidatePublished(
        const StaticHabfmG10Prepared& prepared,
        std::uint32_t writer_id,
        Status& status) const noexcept;
    void CommitPublished(
        StaticHabfmG10Prepared& prepared,
        std::uint32_t writer_id,
        Status& status) noexcept;

private:
    HabfmTerminalControlIntentOwner habfm_owner_{};
    StaticHabfmG10State g10_state_{};
    std::uint64_t generation_ = 0U;
};

static_assert(
    std::is_trivially_copyable<StaticHabfmG10State>::value,
    "Static HABFM/G10 state must remain allocation-free");
static_assert(
    std::is_trivially_copyable<StaticHabfmG10Prepared>::value,
    "Static HABFM/G10 preparation must remain allocation-free");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
