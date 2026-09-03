#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/g13/G13FlatScissors.hpp"
#include "LadyLuck/guidance/g4/HighGBarrelEvidence.hpp"
#include "LadyLuck/guidance/prefire/GunAttackFormObservation.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

// Command-neutral d90 G4 Service producer.  It evaluates the published corner
// interval first, then advances its private gun-form observer exactly once for
// the current ControlFrameIdentity.  It never selects or publishes guidance.
class HighGBarrelEvidenceProvider final
{
public:
    HighGBarrelEvidenceProvider() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const runtime::TacticalCommandBuildInput& input,
        const guidance::g13::G13FlatScissorsObservation& g13_observation,
        bool g13_sample_valid,
        bool g13_response_engaged,
        const HighGBarrelSafetyEvidence& safety,
        const HighGBarrelLoadedResponseEvidence& loaded_response,
        const HighGBarrelCornerIntervalEvidence& corner_interval,
        HighGBarrelExactEvidence& output,
        Status& status) noexcept;

private:
    guidance::prefire::GunAttackFormObserver attack_form_observer_{};
    bool previous_tracking_t_sec_valid_ = false;
    double previous_tracking_t_sec_ = 0.0;
    bool cached_ = false;
    ControlFrameIdentity cached_frame_identity_{};
    HighGBarrelExactEvidence cached_output_{};
    StatusCode cached_status_code_ = StatusCode::InvalidConfiguration;
};

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
