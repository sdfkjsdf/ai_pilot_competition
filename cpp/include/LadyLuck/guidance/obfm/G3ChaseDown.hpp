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

enum class G3ChaseDownObservationReason : std::uint8_t
{
    StateNotObservable = 0U,
    AdversaryDirectionNotResolved = 1U,
    AdversaryCourseNotResolved = 2U,
    InsufficientCausalWindow = 3U,
    TurnNotResolved = 4U,
    TurnSignNotSustained = 5U,
    DescentNotResolved = 6U,
    OwnAboveNotResolved = 7U,
    SustainedDivingTurnBelowCrossingPredicted = 8U,
    ObserverContractRejected = 9U
};

const char* G3ChaseDownObservationReasonLabel(
    G3ChaseDownObservationReason reason) noexcept;

struct G3ChaseDownObservation
{
    bool valid = false;
    G3ChaseDownObservationReason reason =
        G3ChaseDownObservationReason::StateNotObservable;
    bool admitted = false;
    std::int32_t turn_sign = 0;
    bool descent_resolved = false;
    bool own_faster_resolved = false;
    bool own_above_resolved = false;
};

class G3ChaseDownObserver final
{
public:
    G3ChaseDownObserver() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        G3ChaseDownObservation& output,
        Status& status) noexcept;

private:
    struct CourseSample
    {
        double course_rad = 0.0;
        double course_bound_rad = 0.0;
        bool descent_resolved = false;
    };

    std::array<CourseSample, 3U> window_{};
    std::size_t window_count_ = 0U;
};

// Exact typed replacement for the remaining pursuit family
// ("LAG", "EMPLOY"). `Other` preserves every other label.
enum class G3ChaseDownPursuitBehavior : std::uint8_t
{
    Lag = 0U,
    Employ = 2U,
    Other = 3U
};

enum class G3ChaseDownSelectionReason : std::uint8_t
{
    ObservationMissingOrUnadmitted = 0U,
    BehaviorOutsidePursuitFamily = 1U,
    StateOrAimNotFinite = 2U,
    AimAlreadyAtOrAboveOwnAltitude = 3U,
    OwnAltitudeFloorSelected = 4U,
    SelectionContractRejected = 5U
};

const char* G3ChaseDownSelectionReasonLabel(
    G3ChaseDownSelectionReason reason) noexcept;

struct G3ChaseDownAimOverlay
{
    bool valid = false;
    Vector3 aim_point_m{};
};

// Decorator/Task handoff. The standalone module grants no ControlIntent writer
// or behavior ID; a later visible selected Task may consume only this overlay.
struct G3ChaseDownSelectionReceipt
{
    bool selected = false;
    G3ChaseDownSelectionReason reason =
        G3ChaseDownSelectionReason::ObservationMissingOrUnadmitted;
    G3ChaseDownAimOverlay overlay{};
};

void EvaluateG3ChaseDown(
    const DogfightGeometryFrame& frame,
    G3ChaseDownPursuitBehavior upstream_behavior,
    const Vector3& upstream_aim_point_m,
    const G3ChaseDownObservation* observation,
    G3ChaseDownSelectionReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<G3ChaseDownObservation>::value,
    "G3 chase-down observation must remain allocation-free");
static_assert(
    std::is_trivially_copyable<G3ChaseDownSelectionReceipt>::value,
    "G3 chase-down selection must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
