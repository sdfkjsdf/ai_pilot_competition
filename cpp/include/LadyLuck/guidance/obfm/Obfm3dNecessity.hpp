#pragma once

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Command-neutral definition-domain result for generic OBFM 3-D.  This is
// not maneuver admission and owns no writer.  It answers only whether the
// current positive closure can be arrested by the available in-plane
// longitudinal deceleration before the measured range is consumed.
enum class Obfm3dNecessityReason : std::uint8_t
{
    InputNotObservable = 0U,
    NoPositiveClosure = 1U,
    InPlaneArrestSufficient = 2U,
    InPlaneArrestInsufficient = 3U
};

struct Obfm3dNecessityInput
{
    double range_m = 0.0;
    double closing_speed_mps = 0.0;
    // Positive magnitude of the currently admitted deceleration authority.
    double available_deceleration_mps2 = 0.0;
};

struct Obfm3dNecessityReceipt
{
    bool observable = false;
    bool positive_closure = false;
    bool in_plane_arrest_sufficient = false;
    bool three_dimensional_needed = false;
    Obfm3dNecessityReason reason =
        Obfm3dNecessityReason::InputNotObservable;
    double range_m = 0.0;
    double closing_speed_mps = 0.0;
    double available_deceleration_mps2 = 0.0;
    double arrestable_closing_speed_mps = 0.0;
};

const char* Obfm3dNecessityReasonLabel(
    Obfm3dNecessityReason reason) noexcept;

// Exact constant-free stopping-distance identity:
//
//   closing^2 <= 2 * a_available * range
//
// Equality is sufficient. Opening/zero closure needs no 3-D diversion.
// Non-finite or non-positive range/authority remains command-neutral and
// not observable; it never becomes evidence for a vertical maneuver.
void EvaluateObfm3dNecessity(
    const Obfm3dNecessityInput& input,
    Obfm3dNecessityReceipt& output) noexcept;

static_assert(
    std::is_trivially_copyable<Obfm3dNecessityInput>::value,
    "OBFM 3-D necessity input must remain allocation-free");
static_assert(
    std::is_trivially_copyable<Obfm3dNecessityReceipt>::value,
    "OBFM 3-D necessity receipt must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
