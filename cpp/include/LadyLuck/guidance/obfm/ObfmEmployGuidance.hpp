#pragma once

#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

constexpr std::uint32_t ControlIntentWriterObfmEmploy = 10U;

enum class ObfmEmployAdmissionReason : std::uint8_t
{
    None = 0U,
    NoCurrentWeaponEffect = 1U
};

// Command-neutral receipt shared by the parent OBFM reachability decision and
// the EMPLOY leaf Condition.  The provider observes the official current
// offensive effect once; it never selects or publishes a writer.
struct ObfmEmployAdmissionReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    bool admitted = false;
    ObfmEmployAdmissionReason reason = ObfmEmployAdmissionReason::None;
};

class ObfmEmployAdmissionProvider final
{
public:
    void Observe(
        const DogfightGeometryFrame& frame,
        ObfmEmployAdmissionReceipt& output,
        Status& status) const noexcept;
};

struct ObfmEmployGuidanceInput
{
    bool selected = false;
    ObfmLagStationHoldReference station_hold{};
};

// Pure pre-publication candidate.  The visible EMPLOY Task maps this directly
// to its ControlIntent behavior/writer identity and publishes it once.
struct ObfmEmployGuidanceCandidate
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double capture_range_des_m = 0.0;
};

void BuildObfmEmployGuidanceCandidate(
    const DogfightGeometryFrame& frame,
    const ObfmEmployGuidanceInput& input,
    ObfmEmployGuidanceCandidate& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<ObfmEmployAdmissionReceipt>::value,
    "OBFM EMPLOY admission receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEmployGuidanceInput>::value,
    "OBFM EMPLOY input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEmployGuidanceCandidate>::value,
    "OBFM EMPLOY candidate must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
