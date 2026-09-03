#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>
namespace LadyLuck
{
namespace control
{
namespace route5
{

enum class CommandEnvelopeSource : std::int32_t
{
    Unavailable = 0,
    FixedBoundsNonFiniteInput = 1,
    FixedBoundsNonPositiveInput = 2,
    FixedBoundsGearMismatch = 3,
    FixedBoundsPitchTrimDomain = 4,
    FixedBoundsArithmeticInvalid = 5,
    PitchTrimArtifactCurrentNz = 6
};

inline bool IsFixedBoundsCommandEnvelopeSource(
    const CommandEnvelopeSource source) noexcept
{
    return source == CommandEnvelopeSource::FixedBoundsNonFiniteInput
        || source == CommandEnvelopeSource::FixedBoundsNonPositiveInput
        || source == CommandEnvelopeSource::FixedBoundsGearMismatch
        || source == CommandEnvelopeSource::FixedBoundsPitchTrimDomain
        || source == CommandEnvelopeSource::FixedBoundsArithmeticInvalid;
}

inline bool IsPhysicalNzCommandEnvelopeSource(
    const CommandEnvelopeSource source) noexcept
{
    return source == CommandEnvelopeSource::PitchTrimArtifactCurrentNz;
}

inline bool CommandEnvelopeSourceProvidesBounds(
    const CommandEnvelopeSource source) noexcept
{
    return IsPhysicalNzCommandEnvelopeSource(source)
        || IsFixedBoundsCommandEnvelopeSource(source);
}

enum class StallSpeedBoundarySource : std::int32_t
{
    Unavailable = 0,
    PitchTrimArtifactGridFirstCrossing = 1,
    PitchTrimArtifactMachDomainFloorUnresolved = 2,
    PitchTrimArtifactMachDomainCeilingUnresolved = 3
};

// Finite V-n left-boundary receipt. An unresolved domain edge still carries
// the same finite conservative command-floor value as d90; `resolved` keeps
// that value from being promoted to a measured physical stall boundary.
struct StallSpeedBoundary
{
    bool valid = false;
    double speed_mps = 0.0;
    bool resolved = false;
    StallSpeedBoundarySource source = StallSpeedBoundarySource::Unavailable;
};

// Same-frame reference-governor product.  Every scalar is finite.  Optional
// products use explicit validity flags, never NaN sentinels.  A fixed-bounds
// product is command containment only and must not be cited as aircraft
// capability.
struct CommandEnvelope
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    double nz_feasible_g = 0.0;
    double nz_min_g = 0.0;
    double p_max_radps = 0.0;
    double gamma_max_rad = 0.0;
    bool gamma_max_valid = false;
    double nz_feasible_roll_g = 0.0;
    // envelope_from() is called with the frozen p_cmd=0 default. Keep that
    // causal input explicit so the p=0 roll-aware equality is not mistaken
    // for authority over the unported nonzero-p collapse table.
    double roll_reference_p_cmd_radps = 0.0;
    bool roll_reference_p_cmd_valid = false;
    double stall_speed_mps = 0.0;
    bool stall_speed_valid = false;
    bool stall_speed_resolved = false;
    StallSpeedBoundarySource stall_speed_source =
        StallSpeedBoundarySource::Unavailable;
    double speed_mps = 0.0;
    double altitude_m = 0.0;
    double mach = 0.0;
    double mass_kg = 0.0;
    bool enabled = false;
    bool fallback = false;
    double gear_pos_norm = 0.0;
    CommandEnvelopeSource source = CommandEnvelopeSource::Unavailable;
    bool command_containment_authority = false;
    bool physical_authority = false;
    bool current_nz_authoritative = false;
    bool roll_nz_authoritative = false;
    bool gamma_authoritative = false;
    bool stall_speed_authoritative = false;
};

} // namespace route5
} // namespace control
} // namespace LadyLuck
