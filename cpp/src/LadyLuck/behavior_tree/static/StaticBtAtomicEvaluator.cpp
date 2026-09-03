#include "LadyLuck/behavior_tree/static/StaticBtAtomicEvaluator.hpp"

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

BtTickResult ValidateStaticBtControlIntentCandidate(
    const ControlIntent& candidate,
    const ControlFrameIdentity& current_frame_identity,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index) noexcept
{
    if (!IsValidControlFrameIdentity(current_frame_identity) ||
        current_frame_identity.frame_index != frame_index)
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidInput,
            node_id,
            stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CurrentFrameInvalid));
    }
    if (candidate.writer_id == ControlIntentWriterNone)
    {
        return MakeBtTickResult(
            BtReturnCode::WriterContractFault,
            node_id,
            stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CandidateWriterMissing));
    }

    Status status{};
    candidate.Validate(status);
    if (!status.sample_valid())
    {
        const StaticBtAtomicReason reason =
            status.code == StatusCode::NonFiniteInput
                ? StaticBtAtomicReason::CandidateNonFinite
                : StaticBtAtomicReason::CandidateContractInvalid;
        return MakeBtTickResult(BtReturnCode::InvalidInput,
                                node_id,
                                stage_id,
                                frame_index,
                                ToReasonId(reason));
    }
    if (!SameControlFrameIdentity(candidate.frame_identity,
                                  current_frame_identity))
    {
        return MakeBtTickResult(
            BtReturnCode::InvalidInput,
            node_id,
            stage_id,
            frame_index,
            ToReasonId(StaticBtAtomicReason::CandidateFrameMismatch));
    }

    return MakeBtTickResult(BtReturnCode::Completed,
                            node_id,
                            stage_id,
                            frame_index,
                            BtReasonIdNone);
}

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck

