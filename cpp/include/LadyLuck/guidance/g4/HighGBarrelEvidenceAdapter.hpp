#pragma once

#include "LadyLuck/guidance/g4/HighGBarrelEvidence.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"
#include "LadyLuck/runtime/TacticalCompletedTotalLoadReceipt.hpp"

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

// Pure typed adapter for evidence already owned by the current estimator,
// envelope and completed-controller transaction.  It does not observe
// geometry, mutate maneuver lifecycle or publish a command.
class HighGBarrelEvidenceAdapter final
{
public:
    HighGBarrelEvidenceAdapter() noexcept = default;

    void BuildSafety(
        const runtime::TacticalCommandBuildInput& input,
        bool hard_deck_source_valid,
        double hard_deck_margin_m,
        HighGBarrelSafetyEvidence& output,
        Status& status) const noexcept;

    void BuildLoadedResponse(
        const runtime::TacticalCommandBuildInput& input,
        const runtime::TacticalCompletedTotalLoadReceipt& total_load,
        HighGBarrelLoadedResponseEvidence& output,
        Status& status) const noexcept;
};

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
