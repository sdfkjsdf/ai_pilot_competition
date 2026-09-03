#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseSelectionSupply.hpp"
#include "LadyLuck/guidance/obfm/G3SAdversaryCourseReversal.hpp"
#include "LadyLuck/guidance/obfm/PursuitOvershootForecast.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

enum class G10SecondUseAdmissionReason : std::uint8_t
{
    NotUpdated = 0U,
    GateDisabled = 1U,
    ScissorsResponseOwns = 2U,
    ForecastUnavailable = 3U,
    OvershootNotRealized = 4U,
    OvershootInstantUnavailable = 5U,
    AdversaryReversalNotCurrent = 6U,
    ReversalNotAfterOvershoot = 7U,
    SecondUseAdmitted = 8U
};

const char* G10SecondUseAdmissionReasonLabel(
    G10SecondUseAdmissionReason reason) noexcept;

// Command-neutral d90 G10 second-use admission receipt. Optional fields mean
// that the published check order stopped before that ingredient was read.
struct G10SecondUseAdmissionReceipt
{
    bool valid = false;
    bool admitted = false;
    G10SecondUseAdmissionReason reason =
        G10SecondUseAdmissionReason::NotUpdated;
    G10OptionalBool overshoot_realized{};
    G10OptionalBool adversary_reversal_current{};
    G10OptionalBool reversal_after_overshoot{};
    bool entry_family_available = false;
    bool barrel_roll_attack_family = false;
    bool behavior_authority = false;
    bool tactical_command_authority = false;
    bool flight_control_authority = false;
    bool production_authority = false;
    G10SecondUseBridgeAdmissionReceipt bridge{};
};

// Command-neutral causal admission. Forecast and reversal pointers are
// caller-owned completed receipts; this class neither updates nor latches
// either upstream observer.
class G10SecondUseAdmissionProvider final
{
public:
    G10SecondUseAdmissionProvider() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        bool gate_enabled,
        const obfm::PursuitOvershootForecast* completed_forecast,
        const obfm::AdversaryReversalObservation* completed_reversal,
        const G10OptionalDouble& overshoot_realized_t_sec,
        bool scissors_response_engaged,
        G10SecondUseAdmissionReceipt& output,
        Status& status) noexcept;

};

static_assert(
    std::is_trivially_copyable<G10SecondUseAdmissionReceipt>::value,
    "G10 admission receipt must remain allocation-free");
static_assert(
    std::is_trivially_copyable<G10SecondUseAdmissionProvider>::value,
    "G10 admission provider must remain allocation-free");

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
