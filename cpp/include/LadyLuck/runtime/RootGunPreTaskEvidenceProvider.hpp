#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

// Command-neutral, current-frame Root Service receipt. It binds the official
// damage observation and the strict E-M N channel used by downstream leaves.
struct RootGunPreTaskEvidence
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool official_gun_threat = false;
    guidance::em::StrictEmInput strict_em_input{};
    guidance::em::EnergyManeuverCapability capability{};
    bool capability_admitted = false;
    guidance::em::MergeCornerInterval instantaneous_corner_interval{};
};

// Allocation-free value provider shared by Root Gun and HABFM. It observes no
// maneuver lifecycle and publishes no ControlIntent.
class RootGunPreTaskEvidenceProvider final
{
public:
    void Observe(
        const DogfightGeometryFrame& frame,
        RootGunPreTaskEvidence& output,
        Status& status) const noexcept;

private:
    guidance::em::StrictEnergyManeuverEnvelope strict_em_{};
    guidance::em::MergeIntentCornerProvider corner_provider_{};
};

static_assert(
    std::is_trivially_copyable<RootGunPreTaskEvidence>::value,
    "Root Gun pre-task evidence must remain allocation-free");

} // namespace runtime
} // namespace LadyLuck
