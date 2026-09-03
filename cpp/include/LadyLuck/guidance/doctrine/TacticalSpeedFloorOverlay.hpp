#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/doctrine/TacticalSpeedFloorObserver.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

enum class TacticalSpeedFloorDomain : std::uint8_t
{
    OutOfDomain = 0U,
    NeutralHandoff = 1U,
    EntryFamily = 2U,
    ApproachLag = 3U,
    InFightLag = 4U
};

enum class TacticalSpeedFloorApplication : std::uint8_t
{
    UnchangedOutOfDomain = 0U,
    UnchangedInFightLag = 1U,
    UnchangedFloorUnadmitted = 2U,
    UnchangedWithinFloat32Band = 3U,
    UnchangedDesiredSpeedAlreadyAtFloor = 4U,
    AppliedSpeedOnly = 5U,
    AppliedNeutralDive = 6U,
    AppliedNeutralLevel = 7U
};

// Command-neutral G17 Service evidence. The floor is the existing exact
// 25-row d90 TacticalSpeedFloorObserver memo. The pull capability is the
// strict E-M N channel used by Python _capability_from_observation().
struct TacticalSpeedFloorEvidence
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    em::EnergyManeuverCapability capability{};
    bool capability_admitted = false;
};

class TacticalSpeedFloorEvidenceProvider final
{
public:
    TacticalSpeedFloorEvidenceProvider() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const DogfightGeometryFrame& frame,
        TacticalSpeedFloorEvidence& output,
        Status& status) noexcept;

private:
    em::StrictEnergyManeuverEnvelope strict_envelope_{};
};

struct TacticalSpeedFloorOverlayInput
{
    DogfightGeometryFrame frame{};
    ControlIntent upstream_intent{};
    TacticalSpeedFloorEvidence evidence{};
    std::uint32_t modifier_writer_id = ControlIntentWriterNone;
};

// Diagnostic/admission receipt only. `candidate` remains raw guidance; it is
// not a shaped reference, FCS command, estimator measurement, or response.
struct TacticalSpeedFloorOverlayReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool applicable = false;
    bool modified = false;
    TacticalSpeedFloorDomain domain =
        TacticalSpeedFloorDomain::OutOfDomain;
    TacticalSpeedFloorApplication application =
        TacticalSpeedFloorApplication::UnchangedOutOfDomain;
    double own_altitude_m = 0.0;
    double own_speed_mps = 0.0;
    double floor_mps = 0.0;
    double float32_band_mps = 0.0;
    double dive_depression_rad = 0.0;
    std::uint32_t upstream_writer_id = ControlIntentWriterNone;
    std::uint32_t published_writer_id = ControlIntentWriterNone;
    ControlIntent candidate{};
};

// Pure d90 _apply_tactical_speed_floor materializer. Selection and
// publication stay in visible BehaviorTree.CPP Condition/Task nodes.
class TacticalSpeedFloorOverlay final
{
public:
    TacticalSpeedFloorOverlay() noexcept;

    void Evaluate(
        const TacticalSpeedFloorOverlayInput& input,
        TacticalSpeedFloorOverlayReceipt& output,
        Status& status) const noexcept;

private:
    bool RecoveryAllowsDivePosture(
        const ControlFrameIdentity& frame_identity,
        double dive_rad,
        double speed_mps,
        double altitude_m,
        double pull_capability_n_g) const noexcept;
    double FloorLimitedDiveRad(
        const ControlFrameIdentity& frame_identity,
        double speed_mps,
        double altitude_m,
        bool capability_admitted,
        double pull_capability_n_g) const noexcept;

    safety::AutoGcas recovery_predictor_;
};

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
