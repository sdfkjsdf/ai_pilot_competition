#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtResult.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

// Production-neutral typed seam.  Concrete observation, classification,
// lifecycle and candidate storage is supplied by the eventual production
// wiring; this type neither owns nor allocates those objects.
template <typename ObservationT,
          typename ClassificationT,
          typename LifecycleT,
          typename CandidateT>
struct ProductionBtContext
{
    const ObservationT* observation = nullptr;
    ClassificationT* classification = nullptr;
    LifecycleT* lifecycle = nullptr;
    CandidateT* candidate = nullptr;
    std::uint64_t frame_index = 0U;

    constexpr bool HasRequiredInputs() const noexcept
    {
        return observation != nullptr && classification != nullptr &&
               lifecycle != nullptr && candidate != nullptr;
    }

    constexpr BtTickResult MissingRequiredInputResult(
        const BtNodeId node_id,
        const BtStageId stage_id,
        const BtReasonId reason) const noexcept
    {
        return MakeBtTickResult(BtReturnCode::MissingRequiredInput,
                                node_id,
                                stage_id,
                                frame_index,
                                reason);
    }
};

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck

