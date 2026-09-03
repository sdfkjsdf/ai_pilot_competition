#pragma once

#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"

namespace LadyLuck
{
namespace control
{
namespace route5
{

enum class ReferenceGovernorNzfeasArtifactSource : std::int32_t
{
    Unavailable = 0,
    CorrectedGearUpD90 = 1
};

// Typed source receipt for the exact corrected gear-up NZFEAS artifact used
// by EnvelopeFrom().  `valid` authenticates a coherent governor product;
// `canonical_table_authority` distinguishes its physical table result from a
// finite fixed command-containment fallback.
struct ReferenceGovernorNzfeasAuthorityReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool valid = false;
    bool canonical_table_authority = false;
    ReferenceGovernorNzfeasArtifactSource artifact_source =
        ReferenceGovernorNzfeasArtifactSource::Unavailable;
    CommandEnvelopeSource envelope_source = CommandEnvelopeSource::Unavailable;
    double artifact_gear_position = 0.0;
    double artifact_reference_mass_kg = 0.0;
};

// Allocation-free, filesystem-free current-nz subset of the Python reference
// governor.  Its compiled table is the corrected gear-up NZFEAS product.  The
// d90e929b runtime contract returns finite fixed command bounds whenever live
// physical-envelope authority is unavailable.
class ReferenceGovernor final
{
public:
    ReferenceGovernor() noexcept = default;

    void EnvelopeFrom(
        const EstimatorOutputV6& estimate,
        CommandEnvelope& output,
        Status& status) const noexcept;
    void StallSpeedBoundaryFrom(
        const EstimatorOutputV6& estimate,
        double nz_target_g,
        StallSpeedBoundary& output,
        Status& status) const noexcept;
    void CopyNzfeasAuthorityReceipt(
        const CommandEnvelope& envelope,
        ReferenceGovernorNzfeasAuthorityReceipt& output,
        Status& status) const noexcept;
};

} // namespace route5
} // namespace control
} // namespace LadyLuck
