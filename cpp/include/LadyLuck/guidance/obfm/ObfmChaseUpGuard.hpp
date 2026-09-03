#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Typed replacement for the Python behavior-label family consumed by G12.
// The visible BT/runtime adapter owns the label-to-enum mapping.
enum class ObfmChaseUpBehavior : std::uint8_t
{
    Lag = 0U,
    Employ = 2U,
    Other = 3U
};

enum class ObfmChaseUpGuardReason : std::uint8_t
{
    BehaviorOutsidePursuitFamily = 0U,
    SustainedCornerUnavailable = 1U,
    SustainedCornerInvalid = 2U,
    StateUnavailable = 3U,
    AimWithinCeilingBand = 4U,
    CeilingClampSelected = 5U
};

const char* ObfmChaseUpGuardReasonLabel(
    ObfmChaseUpGuardReason reason) noexcept;

// Raw-guidance overlay receipt.  It does not contain p/q/r, Nz, surface,
// thrust, estimator, or actual-aircraft tracking evidence.
struct ObfmChaseUpGuardReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool applicable = false;
    bool modified = false;
    ObfmChaseUpGuardReason reason =
        ObfmChaseUpGuardReason::BehaviorOutsidePursuitFamily;
    double sustained_corner_speed_mps = 0.0;
    double own_speed_mps = 0.0;
    double own_altitude_m = 0.0;
    double climb_budget_m = 0.0;
    double ceiling_altitude_m = 0.0;
    double aim_altitude_m = 0.0;
    double float32_band_m = 0.0;
    ControlIntent candidate{};
};

// Exact allocation-free counterpart of chase_up_guarded_command().  Missing
// or physically unobservable corner/state evidence is ordinary non-selection:
// the validated upstream command is retained unchanged and status stays OK.
void EvaluateObfmChaseUpGuard(
    const DogfightGeometryFrame& frame,
    ObfmChaseUpBehavior upstream_behavior,
    const ControlIntent& upstream_intent,
    const em::MergeCornerInterval& sustained_corner,
    ObfmChaseUpGuardReceipt& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<ObfmChaseUpGuardReceipt>::value,
    "OBFM chase-up receipt must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
