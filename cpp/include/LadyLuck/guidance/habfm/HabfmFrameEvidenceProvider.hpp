#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cstdint>

namespace LadyLuck
{

enum class TacticalSpeedFloorStatus : std::uint8_t
{
    DerivedByHalfLoopBisection = 0U,
    EmPublicationNotAdmitted = 1U,
    AltitudeNotFinite = 2U,
    AltitudeOutsidePublishedAxis = 3U,
    DerivedTrajectoryOutsidePublishedAxis = 4U
};

const char* TacticalSpeedFloorStatusText(
    TacticalSpeedFloorStatus status) noexcept;
const char* TacticalSpeedFloorProvenance() noexcept;

struct TacticalSpeedFloorSample
{
    TacticalSpeedFloorStatus status =
        TacticalSpeedFloorStatus::EmPublicationNotAdmitted;
    guidance::em::EmValue floor_mps{};
    guidance::em::EmValue apex_altitude_m{};
    guidance::em::EmValue cell_altitude_m{};

    bool admitted() const noexcept
    {
        return status == TacticalSpeedFloorStatus::DerivedByHalfLoopBisection
            && floor_mps.has_value
            && apex_altitude_m.has_value
            && cell_altitude_m.has_value;
    }
};

enum class SustainedTurnPointStatus : std::uint8_t
{
    Admitted = 0U,
    AltitudeOutsidePublishedAxis = 1U,
    PublishedSustainedPointNotTrusted = 2U,
    PublishedSustainedLoadNotAboveOneG = 3U,
    PublishedLookupInvalid = 4U
};

const char* SustainedTurnPointStatusText(
    SustainedTurnPointStatus status) noexcept;
const char* SustainedTurnPointProvenance() noexcept;

struct SustainedTurnOperatingPoint
{
    SustainedTurnPointStatus status =
        SustainedTurnPointStatus::PublishedLookupInvalid;
    guidance::em::EmValue speed_mps{};
    guidance::em::EmValue load_factor_g{};
    guidance::em::EmValue turn_rate_radps{};
    guidance::em::EmValue turn_radius_m{};
    guidance::em::EmValue mass_kg{};

    bool admitted() const noexcept
    {
        return status == SustainedTurnPointStatus::Admitted
            && speed_mps.has_value
            && load_factor_g.has_value
            && turn_rate_radps.has_value
            && turn_radius_m.has_value
            && mass_kg.has_value;
    }
};

enum class EnergyResolutionOwner : std::uint8_t
{
    None = 0U,
    Ownship = 1U,
    Adversary = 2U
};

enum class HabfmFrameEvidenceStatus : std::uint8_t
{
    Built = 0U,
    FrameStateNotFinite = 1U,
    SpeedNormNotFinite = 2U
};

struct HabfmFrameEvidence
{
    guidance::em::EmValue own_altitude_m{};
    guidance::em::EmValue adversary_altitude_m{};
    guidance::em::EmValue own_speed_mps{};
    guidance::em::EmValue adversary_speed_mps{};

    guidance::em::MergeCornerInterval adversary_corner_interval{};
    guidance::em::MergeCornerInterval own_corner_interval{};
    guidance::em::MergeCornerInterval own_sustained_corner_interval{};
    SustainedTurnOperatingPoint own_sustained_turn_point{};

    guidance::em::EnergyResolution merge_energy_resolution{};
    EnergyResolutionOwner merge_energy_resolution_owner =
        EnergyResolutionOwner::None;
    TacticalSpeedFloorSample g17_speed_floor{};
};

// Allocation-free frame-only supplier for d90 HABFM chart rows.  It owns no
// tactical selection and publishes no guidance or control command.
class HabfmFrameEvidenceProvider final
{
public:
    HabfmFrameEvidenceProvider() noexcept;

    // Python's G17 altitude memo survives doctrine episode/mode resets.  The
    // C++ representation is an immutable precomputed table, so Reset is a
    // deliberate no-op with the same lifetime semantics.
    void Reset() noexcept;

    void Build(
        const DogfightGeometryFrame& frame,
        HabfmFrameEvidence& output,
        HabfmFrameEvidenceStatus& status) const noexcept;

private:
    void SampleTacticalSpeedFloor(
        double altitude_m,
        TacticalSpeedFloorSample& output) const noexcept;

    guidance::em::MergeIntentCornerProvider corner_provider_{};
    guidance::em::StrictEnergyManeuverEnvelope strict_envelope_{};
};

} // namespace LadyLuck
