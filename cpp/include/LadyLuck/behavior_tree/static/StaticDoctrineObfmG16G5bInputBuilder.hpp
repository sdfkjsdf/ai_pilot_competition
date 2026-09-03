#pragma once

#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bOwner.hpp"
#include "LadyLuck/behavior_tree/static/StaticSafetyGunStagedOwner.hpp"
#include "LadyLuck/guidance/doctrine/BilateralTurnCircleRootAuthority.hpp"
#include "LadyLuck/guidance/obfm/ObfmPursuitClassificationObserver.hpp"

#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

// Fixed receipts available before the OBFM writer-6/8/5/7 traversal. Safety,
// Gun, and pursuit producers remain command-neutral and retain their own
// lifecycle ownership.
struct StaticDoctrineObfmG16G5bInputSources
{
    runtime::TacticalCommandBuildInput tactical_input{};
    bool safety_gun_receipt_available = false;
    StaticSafetyGunPreparedReceipt safety_gun{};
    bool previous_pursuit_receipt_available = false;
    guidance::obfm::ObfmPursuitClassificationObserverReceipt
        previous_pursuit{};
};

// Diagnostic receipt for input assembly only. It contains no guidance,
// control-law, actuator, or aircraft-response authority.
struct StaticDoctrineObfmG16G5bInputBuilderReceipt
{
    bool build_attempted = false;
    ControlFrameIdentity frame_identity{};
    bool root_observation_attempted = false;
    bool root_observation_ready = false;
    guidance::doctrine::BilateralDoctrineTurnCircleReceipt
        root_observation{};
    StatusCode root_status_code = StatusCode::Ok;
    bool root_finite_habfm_fallback = false;
    bool mode_decision_ready = false;
    bool safety_gun_receipt_current = false;
    bool official_gun_receipt_current = false;
    bool predictive_prefire_receipt_current = false;
    bool g5b_safety_ready = false;
    bool own_speed_floor_sampled = false;
    bool opponent_speed_floor_sampled = false;
    bool previous_pursuit_consumed = false;
    ControlFrameIdentity previous_pursuit_frame_identity{};
    bool output_ready = false;
    StatusCode status_code = StatusCode::Ok;
};

// Value-only composition boundary for the static OBFM owner. A finite current
// frame always receives a total OBFM/HABFM/DBFM mode. Missing optional Gun,
// predictive, speed-floor, or prior-pursuit authority remains typed absence;
// G5b therefore withholds entry while the ordinary writer-5 path remains
// available.
class StaticDoctrineObfmG16G5bInputBuilder final
{
public:
    void Build(
        const StaticDoctrineObfmG16G5bInputSources& sources,
        StaticDoctrineObfmG16G5bInput& output,
        StaticDoctrineObfmG16G5bInputBuilderReceipt& receipt,
        Status& status) const noexcept;

private:
    guidance::doctrine::BilateralTurnCircleRootAuthority
        root_authority_{};
};

static_assert(
    std::is_standard_layout<
        StaticDoctrineObfmG16G5bInputSources>::value,
    "Static OBFM input sources must have standard layout.");
static_assert(
    std::is_trivially_copyable<
        StaticDoctrineObfmG16G5bInputSources>::value,
    "Static OBFM input sources must remain fixed-storage.");
static_assert(
    std::is_standard_layout<
        StaticDoctrineObfmG16G5bInputBuilderReceipt>::value,
    "Static OBFM input-builder receipt must have standard layout.");
static_assert(
    std::is_trivially_copyable<
        StaticDoctrineObfmG16G5bInputBuilderReceipt>::value,
    "Static OBFM input-builder receipt must remain fixed-storage.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
