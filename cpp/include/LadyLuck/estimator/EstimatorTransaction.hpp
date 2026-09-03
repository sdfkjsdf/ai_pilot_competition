#pragma once

#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

#include <cstdint>

namespace LadyLuck
{

// These values describe ownership and lifecycle. They are internal C++
// contracts, not part of the competition DLL ABI.
enum class EstimatorTransactionCode : std::int32_t
{
    Committed = 0,
    ResetCommitted = 1,
    InvalidConfiguration = -1,
    InvalidFrameContext = -2,
    DuplicateFrame = -3,
    ReverseFrame = -4,
    StructuralOwnshipRejected = -5,
    StructuralActionRejected = -6,
    CandidateUpdateRejected = -7,
    CandidateOutputRejected = -8
};

enum class EstimatorTransactionCause : std::int32_t
{
    None = 0,
    InitialConstruction = 1,
    ExplicitReset = 2,
    GapResync = 3,
    CompletedModelFbwTransition = 4,
    FrameContextInvalidReject = 5,
    FrameContextDuplicateReject = 6,
    FrameContextReverseReject = 7,
    StructuralOwnshipReject = 8,
    StructuralActionReject = 9,
    CandidateUpdateReject = 10,
    LateCandidateOutputValidationReject = 11
};

// Flight-state admission and implementation faults have different command
// authority. Only an externally rejected ownship frame may select the explicit
// Auto-GCAS containment path. A malformed previous command, candidate, schema,
// or configuration is an internal fault and must publish no actuator command.
enum class EstimatorFaultClass : std::int32_t
{
    None = 0,
    ExternalFrameRejected = 1,
    InternalFault = 2
};

struct EstimatorTransactionReceipt
{
    EstimatorTransactionCode code =
        EstimatorTransactionCode::InvalidConfiguration;
    EstimatorTransactionCause cause = EstimatorTransactionCause::None;
    EstimatorFaultClass fault_class = EstimatorFaultClass::InternalFault;
    bool state_committed = false;
    std::uint64_t episode_epoch_before = 0U;
    std::uint64_t episode_epoch_after = 0U;
    std::uint64_t estimator_epoch_before = 0U;
    std::uint64_t estimator_epoch_after = 0U;
    OptionalFrameIndex measurement_frame_index{};
    std::uint64_t gap_count = 0U;

    bool ok() const noexcept
    {
        return static_cast<std::int32_t>(code) >= 0;
    }
};

struct EstimatorUpdateResult
{
    EstimatorTransactionReceipt transaction{};
    bool has_output = false;
    PlaneState plane_state{};
    EstimatorOutputV6 output{};

    bool ok() const noexcept
    {
        return transaction.ok() && has_output;
    }
};

struct FrameContextBuildResult
{
    EstimatorTransactionCode code =
        EstimatorTransactionCode::InvalidFrameContext;
    EstimatorTransactionCause cause =
        EstimatorTransactionCause::FrameContextInvalidReject;
    EstimatorFaultClass fault_class = EstimatorFaultClass::InternalFault;
    FrameContext value{};

    bool ok() const noexcept
    {
        return code == EstimatorTransactionCode::Committed;
    }
};

#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
enum class EstimatorCandidateFaultForTest : std::int32_t
{
    None = 0,
    NonFiniteMassAfterCandidateUpdate = 1
};
#endif

} // namespace LadyLuck
