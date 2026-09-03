#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
// Fixed production episode receipt shared with PostRoot phase-graded DBFM.
// It carries no TacticalCommand storage and has no command authority of its
// own; it only preserves the Gun BREAK lifecycle side already selected by the
// legacy policy.
struct GunDefenseSnapshot
{
    bool active = false;
    std::int32_t side_sign = 1;
    std::uint64_t entry_count = 0U;
    bool toward_side_candidate_held = false;
};

enum class HorizontalBreakDirectionSource : std::uint8_t
{
    None = 0U,
    AttackerLos = 1U,
    OwnNose = 2U,
    OwnVelocity = 3U
};

enum class HorizontalBreakReferenceReason : std::uint8_t
{
    NotEvaluated = 0U,
    HorizontalDirectionUnavailable = 1U,
    RangeUnavailable = 2U,
    OwnSpeedUnavailable = 3U,
    CaptureRangeUnavailable = 4U,
    ArithmeticUnavailable = 5U,
    Ready = 6U
};

// Allocation-free raw guidance reference shared by Root/DBFM break
// materializers.  This is still only an aim/speed request; downstream
// guidance and the FCS own body-rate, Nz, surface, and thrust conversion.
struct HorizontalBreakReferenceReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool command_available = false;
    HorizontalBreakReferenceReason reason =
        HorizontalBreakReferenceReason::NotEvaluated;
    HorizontalBreakDirectionSource direction_source =
        HorizontalBreakDirectionSource::None;
    std::int32_t side_sign = 0;
    Vector3 aim_point_m{};
    double desired_speed_mps = 0.0;
    double capture_range_des_m = 0.0;
};

// Totalized add/main@45abc horizontal_break_command materializer.  A finite
// geometric degeneracy is ordinary nonselection; malformed declared evidence
// remains a typed Status fault.
void BuildHorizontalBreakReference(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign,
    HorizontalBreakReferenceReceipt& output,
    Status& status) noexcept;

static_assert(std::is_standard_layout<GunDefenseSnapshot>::value,
              "GunDefenseSnapshot must remain standard-layout");
static_assert(std::is_trivially_copyable<GunDefenseSnapshot>::value,
              "GunDefenseSnapshot must remain allocation-free");
static_assert(sizeof(GunDefenseSnapshot) == 24U,
              "GunDefenseSnapshot x64 ABI size changed");
static_assert(alignof(GunDefenseSnapshot) == 8U,
              "GunDefenseSnapshot x64 ABI alignment changed");
static_assert(offsetof(GunDefenseSnapshot, active) == 0U,
              "GunDefenseSnapshot.active offset changed");
static_assert(offsetof(GunDefenseSnapshot, side_sign) == 4U,
              "GunDefenseSnapshot.side_sign offset changed");
static_assert(offsetof(GunDefenseSnapshot, entry_count) == 8U,
              "GunDefenseSnapshot.entry_count offset changed");
static_assert(
    offsetof(GunDefenseSnapshot, toward_side_candidate_held) == 16U,
    "GunDefenseSnapshot.toward_side_candidate_held offset changed");
}
