#include "LadyLuck/estimator/PlaneStateEstimator.hpp"

#include <cmath>
#include <limits>

namespace LadyLuck
{
namespace
{

constexpr double ActionDomainTolerance = 1.0e-12;

bool FiniteCoreOutput(const EstimatorOutputV6& output) noexcept
{
    const double values[] = {
        output.nz,
        output.mass,
        output.u,
        output.v,
        output.w,
        output.V,
        output.alt,
        output.roll,
        output.pitch,
        output.yaw,
        output.alpha,
        output.beta,
        output.mu,
        output.mach,
        output.qbar,
        output.elevator_rad,
        output.p,
        output.q,
        output.r,
        output.ground_speed_horizontal_mps,
        output.gear_pos_norm,
        output.nz_model,
        output.nz_flaperon,
        output.nz_fused,
        output.sample_dt_s
    };
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

EstimatorFaultClass ClassifyFault(
    const EstimatorTransactionCode code) noexcept
{
    switch (code)
    {
    case EstimatorTransactionCode::Committed:
    case EstimatorTransactionCode::ResetCommitted:
        return EstimatorFaultClass::None;
    case EstimatorTransactionCode::InvalidFrameContext:
    case EstimatorTransactionCode::DuplicateFrame:
    case EstimatorTransactionCode::ReverseFrame:
    case EstimatorTransactionCode::StructuralOwnshipRejected:
        return EstimatorFaultClass::ExternalFrameRejected;
    case EstimatorTransactionCode::InvalidConfiguration:
    case EstimatorTransactionCode::StructuralActionRejected:
    case EstimatorTransactionCode::CandidateUpdateRejected:
    case EstimatorTransactionCode::CandidateOutputRejected:
        return EstimatorFaultClass::InternalFault;
    }
    return EstimatorFaultClass::InternalFault;
}

} // namespace

PlaneStateEstimator::PlaneStateEstimator() noexcept
    : PlaneStateEstimator(PlaneStateEstimatorConfig{})
{
}

PlaneStateEstimator::PlaneStateEstimator(
    const PlaneStateEstimatorConfig& config) noexcept
    : config_(config)
{
    configuration_valid_ =
        std::isfinite(config_.initial_mass_kg)
        && config_.initial_mass_kg > 0.0
        && std::isfinite(config_.nominal_dt_s)
        && config_.nominal_dt_s > 0.0
        && std::isfinite(config_.gear_pos_norm)
        && config_.gear_pos_norm >= 0.0
        && config_.gear_pos_norm <= 1.0;
    state_.payload = LightNzEstimator(config_.gear_pos_norm);
    configuration_valid_ = configuration_valid_
        && state_.payload.configuration_valid();
}

bool PlaneStateEstimator::configuration_valid() const noexcept
{
    return configuration_valid_;
}

void PlaneStateEstimator::ClassifyChronology(
    const std::uint64_t measurement_frame_index,
    ChronologyDecision& output) const noexcept
{
    output = ChronologyDecision{};
    output.episode_epoch = state_.episode_epoch;
    output.gap_count = 1U;
    output.gap_policy = GapPolicy::FirstSeed;
    if (!configuration_valid_)
    {
        return;
    }
    if (state_.accepted_measurement_frame_index.has_value)
    {
        const std::uint64_t previous =
            state_.accepted_measurement_frame_index.value;
        if (measurement_frame_index == previous)
        {
            output.code = EstimatorTransactionCode::DuplicateFrame;
            output.cause =
                EstimatorTransactionCause::FrameContextDuplicateReject;
            output.fault_class = EstimatorFaultClass::ExternalFrameRejected;
            return;
        }
        if (measurement_frame_index < previous)
        {
            output.code = EstimatorTransactionCode::ReverseFrame;
            output.cause =
                EstimatorTransactionCause::FrameContextReverseReject;
            output.fault_class = EstimatorFaultClass::ExternalFrameRejected;
            return;
        }
        output.gap_count = measurement_frame_index - previous;
        if (output.gap_count == 1U)
        {
            output.gap_policy = GapPolicy::Normal;
        }
        else if (output.gap_count == 2U)
        {
            output.gap_policy = GapPolicy::ObserverReprime;
        }
        else
        {
            if (output.episode_epoch
                == std::numeric_limits<std::uint64_t>::max())
            {
                // Preserve the historical receipt at the impossible resync
                // overflow boundary; this is a frame-context rejection, not
                // a configuration rewrite.
                output.code = EstimatorTransactionCode::InvalidFrameContext;
                return;
            }
            output.gap_policy = GapPolicy::GapResync;
            ++output.episode_epoch;
        }
    }

    output.code = EstimatorTransactionCode::Committed;
    output.cause = EstimatorTransactionCause::None;
    output.fault_class = EstimatorFaultClass::None;
}

void PlaneStateEstimator::MakeFrameContext(
    const FrameContextRequest& request,
    FrameContextBuildResult& output) const noexcept
{
    output = FrameContextBuildResult{};
    if (!configuration_valid_
        || !std::isfinite(request.estimator_soft_budget_s)
        || request.estimator_soft_budget_s <= 0.0)
    {
        return;
    }

    ChronologyDecision chronology{};
    ClassifyChronology(request.measurement_frame_index, chronology);
    output.code = chronology.code;
    output.cause = chronology.cause;
    output.fault_class = chronology.fault_class;
    if (chronology.code != EstimatorTransactionCode::Committed)
    {
        return;
    }

    FrameContext& context = output.value;
    context.episode_epoch = chronology.episode_epoch;
    context.measurement_frame_index = request.measurement_frame_index;
    context.command_frame_index = request.command_frame_index;
    context.source_t_sec = static_cast<double>(
        request.measurement_frame_index) * config_.nominal_dt_s;
    context.source_time_kind = SourceTimeKind::PlaneInfoIndexDerived;
    context.nominal_dt_sec = config_.nominal_dt_s;
    context.sample_dt_sec = static_cast<double>(chronology.gap_count)
        * config_.nominal_dt_s;
    context.gap_count = chronology.gap_count;
    context.gap_policy = chronology.gap_policy;
    context.previous_command_frame = request.previous_command_frame;
    context.estimator_soft_budget_sec = request.estimator_soft_budget_s;
    if (!std::isfinite(context.source_t_sec)
        || !std::isfinite(context.sample_dt_sec)
        || context.sample_dt_sec <= 0.0)
    {
        output = FrameContextBuildResult{};
        return;
    }
    output.code = EstimatorTransactionCode::Committed;
    output.cause = EstimatorTransactionCause::None;
    output.fault_class = EstimatorFaultClass::None;
}

EstimatorTransactionReceipt PlaneStateEstimator::Rejection(
    const EstimatorTransactionCode code,
    const EstimatorTransactionCause cause,
    const FrameContext* const context) const noexcept
{
    EstimatorTransactionReceipt receipt{};
    receipt.code = code;
    receipt.cause = cause;
    receipt.fault_class = ClassifyFault(code);
    receipt.state_committed = false;
    receipt.episode_epoch_before = state_.episode_epoch;
    receipt.episode_epoch_after = state_.episode_epoch;
    receipt.estimator_epoch_before = state_.estimator_epoch;
    receipt.estimator_epoch_after = state_.estimator_epoch;
    if (context != nullptr)
    {
        receipt.measurement_frame_index.has_value = true;
        receipt.measurement_frame_index.value =
            context->measurement_frame_index;
        receipt.gap_count = context->gap_count;
    }
    return receipt;
}

bool PlaneStateEstimator::ValidateFeedbackStructure(
    const CommandFeedback& feedback,
    const FrameContext& context) noexcept
{
    const Command4Estimator& command = feedback.estimator_command_u_dll;
    const double lower[] = {-1.0, -1.0, -1.0, 0.0};
    for (std::size_t index = 0U; index < command.size(); ++index)
    {
        if (!std::isfinite(command[index])
            || command[index] < lower[index] - ActionDomainTolerance
            || command[index] > 1.0 + ActionDomainTolerance)
        {
            return false;
        }
    }
    if (feedback.kind == ActionFeedbackKind::ResetSeed)
    {
        // ResetSeed means that this runtime has never transmitted an actuator
        // command.  That fact remains true after a command-neutral rejected
        // frame or a downstream NoCommand result; it is not restricted to the
        // estimator's very first measurement transaction.
        return !feedback.has_transmitted_wire_payload
            && !feedback.source_frame_index.has_value
            && !feedback.source_t_sec.has_value
            && !context.previous_command_frame.has_value
            && feedback.delay_frames == 1U;
    }
    if (feedback.kind == ActionFeedbackKind::PreviousTransmittedAssumption)
    {
        if (!feedback.has_transmitted_wire_payload
            || !feedback.source_frame_index.has_value
            || !context.previous_command_frame.has_value
            || feedback.source_frame_index.value
                != context.previous_command_frame.value
            || feedback.delay_frames != 1U)
        {
            return false;
        }
        return true;
    }
    // The production path admits only an explicit local reset seed or the
    // previous transmitted payload. Frame index is the chronology authority;
    // duplicated float/time representations are not re-compared.
    return false;
}

EstimatorTransactionCause PlaneStateEstimator::TransactionCause(
    const FrameState& current,
    const FrameContext& context,
    const bool seed_transition) noexcept
{
    if (context.gap_policy == GapPolicy::GapResync)
    {
        return EstimatorTransactionCause::GapResync;
    }
    if (!seed_transition)
    {
        return EstimatorTransactionCause::CompletedModelFbwTransition;
    }
    if (current.episode_epoch == 0U
        && current.estimator_epoch == 0U
        && !current.accepted_measurement_frame_index.has_value)
    {
        return EstimatorTransactionCause::InitialConstruction;
    }
    return EstimatorTransactionCause::ExplicitReset;
}

void PlaneStateEstimator::BindContext(
    EstimatorOutputV6& output,
    const FrameContext& context) noexcept
{
    output.measurement_reset_epoch.has_value = true;
    output.measurement_reset_epoch.value = context.episode_epoch;
    output.measurement_frame_index.has_value = true;
    output.measurement_frame_index.value = context.measurement_frame_index;
    output.measurement_source_t_sec.has_value = true;
    output.measurement_source_t_sec.value = context.source_t_sec;
    output.measurement_source_time_kind = context.source_time_kind;
    output.accepted_sample_t_sec.has_value = true;
    output.accepted_sample_t_sec.value = context.source_t_sec;
}

void PlaneStateEstimator::BindValidityMetadata(
    EstimatorOutputV6& output,
    const EstimatorTransactionCause cause) noexcept
{
    if (cause != EstimatorTransactionCause::CompletedModelFbwTransition)
    {
        output.nz_valid = false;
        output.nz_quality = 0.0;
        output.nz_gate = EstimatorGate::SeedUnvalidated;
        output.nz_source = EstimatorSource::ModelSeedUnvalidated;
    }
    else if (!output.pqr_valid)
    {
        output.nz_valid = false;
        output.nz_quality = 0.0;
        output.nz_gate = EstimatorGate::DependencyPqrIntervalInvalid;
        output.nz_source = EstimatorSource::
            AeroModelInvalidPqrSentinelKinematicFusionDisabled;
    }
    else if (!output.pqr_endpoint_valid)
    {
        output.nz_valid = false;
        output.nz_quality = 0.0;
        output.nz_gate = EstimatorGate::DependencyPqrEndpointInvalid;
        output.nz_source = EstimatorSource::
            AeroModelIntervalFallbackKinematicFusionDisabled;
    }
    else
    {
        output.nz_valid = true;
        output.nz_quality = 1.0;
        output.nz_gate = EstimatorGate::LiveEndpoint;
        output.nz_source = EstimatorSource::
            AeroModelEndpointPqrKinematicFusionDisabled;
    }
    output.nz_flaperon_valid = output.nz_valid;

    output.elevator_quality =
        cause == EstimatorTransactionCause::CompletedModelFbwTransition
            ? 1.0
            : 0.0;
    output.elevator_valid =
        cause == EstimatorTransactionCause::CompletedModelFbwTransition;
    switch (cause)
    {
    case EstimatorTransactionCause::InitialConstruction:
        output.elevator_gate = EstimatorGate::ConstructionSeedUnvalidated;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
        break;
    case EstimatorTransactionCause::ExplicitReset:
        output.elevator_gate = EstimatorGate::ExplicitResetSeedUnvalidated;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
        break;
    case EstimatorTransactionCause::GapResync:
        output.elevator_gate = EstimatorGate::GapResyncSeedUnvalidated;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
        break;
    case EstimatorTransactionCause::CompletedModelFbwTransition:
        output.elevator_gate = EstimatorGate::LiveActuatorReplica;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaLive;
        break;
    default:
        output.elevator_gate = EstimatorGate::Uninitialized;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
        break;
    }
}

bool PlaneStateEstimator::ValidateCandidate(
    const EstimatorOutputV6& output,
    const LightNzSnapshot& candidate,
    const EstimatorTransactionCause cause) noexcept
{
    static_cast<void>(candidate);
    static_cast<void>(cause);
    return FiniteCoreOutput(output)
        && std::isfinite(output.thrust)
        && output.mass > 0.0
        && output.V > 0.0
        && output.gear_pos_norm >= 0.0
        && output.gear_pos_norm <= 1.0;
}

EstimatorUpdateResult PlaneStateEstimator::Update(
    const KinematicObservation& ownship,
    const CommandFeedback& feedback,
    const FrameContext& context) noexcept
{
    EstimatorUpdateResult result{};
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    EstimatorTestSeamState* const test_seam = test_seam_state_;
    if (test_seam != nullptr)
    {
        test_seam->candidate_evidence = EstimatorCandidateEvidenceForTest{};
    }
#endif
    if (!configuration_valid_
        || !std::isfinite(context.source_t_sec)
        || !std::isfinite(context.sample_dt_sec)
        || context.sample_dt_sec <= 0.0)
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::InvalidFrameContext,
            EstimatorTransactionCause::FrameContextInvalidReject,
            &context);
        return result;
    }
    if (ownship.frame_index != context.measurement_frame_index)
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::StructuralOwnshipRejected,
            EstimatorTransactionCause::StructuralOwnshipReject,
            &context);
        return result;
    }

    const Result<PlaneState> converted = ConvertKinematicObservation(ownship);
    if (!converted.ok())
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::StructuralOwnshipRejected,
            EstimatorTransactionCause::StructuralOwnshipReject,
            &context);
        return result;
    }
    if (!ValidateFeedbackStructure(feedback, context))
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::StructuralActionRejected,
            EstimatorTransactionCause::StructuralActionReject,
            &context);
        return result;
    }

    const FrameState current = state_;
    const bool gap_resync = context.gap_policy == GapPolicy::GapResync;
    const bool seed_transition = !current.payload.ready() || gap_resync;
    const EstimatorTransactionCause transaction_cause = TransactionCause(
        current,
        context,
        seed_transition);
    LightNzEstimator candidate = gap_resync
        ? current.payload.FreshCandidate()
        : current.payload.DetachedCopy();
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    if (test_seam != nullptr)
    {
        test_seam->candidate_evidence.available = true;
        test_seam->candidate_evidence.before_update = candidate.Snapshot();
    }
#endif

    LightNzUpdateInput input{};
    input.state = converted.value;
    input.feedback = feedback;
    input.initial_mass_kg = config_.initial_mass_kg;
    input.sample_dt_s = context.sample_dt_sec;
    Result<EstimatorOutputV6> candidate_result = candidate.Step(input);
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    if (test_seam != nullptr)
    {
        test_seam->candidate_evidence.update_completed = candidate_result.ok();
        test_seam->candidate_evidence.after_update = candidate.Snapshot();
    }
#endif
    if (!candidate_result.ok())
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::CandidateUpdateRejected,
            EstimatorTransactionCause::CandidateUpdateReject,
            &context);
        return result;
    }

    EstimatorOutputV6 output = candidate_result.value;
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    if (test_seam != nullptr)
    {
        test_seam->candidate_evidence.has_output_after_update = true;
        test_seam->candidate_evidence.output_after_update = output;
    }
#endif
    BindContext(output, context);
    BindValidityMetadata(output, transaction_cause);
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    if (test_seam != nullptr
        && test_seam->next_fault
            == EstimatorCandidateFaultForTest::NonFiniteMassAfterCandidateUpdate)
    {
        output.mass = std::numeric_limits<double>::quiet_NaN();
        test_seam->candidate_evidence.output_fault_injected = true;
    }
    if (test_seam != nullptr)
    {
        test_seam->next_fault = EstimatorCandidateFaultForTest::None;
    }
#endif
    if (!ValidateCandidate(
            output,
            candidate.Snapshot(),
            transaction_cause))
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::CandidateOutputRejected,
            EstimatorTransactionCause::LateCandidateOutputValidationReject,
            &context);
        return result;
    }
    if (current.estimator_epoch
        == std::numeric_limits<std::uint64_t>::max())
    {
        result.transaction = Rejection(
            EstimatorTransactionCode::CandidateOutputRejected,
            EstimatorTransactionCause::LateCandidateOutputValidationReject,
            &context);
        return result;
    }

    FrameState next = current;
    next.episode_epoch = context.episode_epoch;
    next.estimator_epoch = current.estimator_epoch + 1U;
    next.accepted_measurement_frame_index.has_value = true;
    next.accepted_measurement_frame_index.value =
        context.measurement_frame_index;
    next.accepted_sample_t_s.has_value = true;
    next.accepted_sample_t_s.value = context.source_t_sec;
    next.payload = candidate;
    state_ = next;

    result.transaction.code = EstimatorTransactionCode::Committed;
    result.transaction.cause = transaction_cause;
    result.transaction.fault_class = EstimatorFaultClass::None;
    result.transaction.state_committed = true;
    result.transaction.episode_epoch_before = current.episode_epoch;
    result.transaction.episode_epoch_after = next.episode_epoch;
    result.transaction.estimator_epoch_before = current.estimator_epoch;
    result.transaction.estimator_epoch_after = next.estimator_epoch;
    result.transaction.measurement_frame_index.has_value = true;
    result.transaction.measurement_frame_index.value =
        context.measurement_frame_index;
    result.transaction.gap_count = context.gap_count;
    result.has_output = true;
    result.plane_state = converted.value;
    result.output = output;
    return result;
}

EstimatorTransactionReceipt PlaneStateEstimator::Reset() noexcept
{
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    if (test_seam_state_ != nullptr)
    {
        test_seam_state_->candidate_evidence =
            EstimatorCandidateEvidenceForTest{};
        test_seam_state_->next_fault = EstimatorCandidateFaultForTest::None;
    }
#endif
    if (!configuration_valid_
        || state_.episode_epoch
            == std::numeric_limits<std::uint64_t>::max())
    {
        return Rejection(
            EstimatorTransactionCode::InvalidConfiguration,
            EstimatorTransactionCause::None,
            nullptr);
    }
    const FrameState current = state_;
    FrameState next = current;
    next.episode_epoch = current.episode_epoch + 1U;
    next.accepted_measurement_frame_index = OptionalFrameIndex{};
    next.accepted_sample_t_s = OptionalSeconds{};
    next.payload = current.payload.FreshCandidate();
    state_ = next;

    EstimatorTransactionReceipt receipt{};
    receipt.code = EstimatorTransactionCode::ResetCommitted;
    receipt.cause = EstimatorTransactionCause::ExplicitReset;
    receipt.fault_class = EstimatorFaultClass::None;
    receipt.state_committed = true;
    receipt.episode_epoch_before = current.episode_epoch;
    receipt.episode_epoch_after = next.episode_epoch;
    receipt.estimator_epoch_before = current.estimator_epoch;
    receipt.estimator_epoch_after = next.estimator_epoch;
    return receipt;
}

PlaneStateEstimatorSnapshot PlaneStateEstimator::Snapshot() const noexcept
{
    PlaneStateEstimatorSnapshot snapshot{};
    snapshot.episode_epoch = state_.episode_epoch;
    snapshot.estimator_epoch = state_.estimator_epoch;
    snapshot.accepted_measurement_frame_index =
        state_.accepted_measurement_frame_index;
    snapshot.accepted_sample_t_s = state_.accepted_sample_t_s;
    snapshot.configured_initial_mass_kg = config_.initial_mass_kg;
    snapshot.nominal_dt_s = config_.nominal_dt_s;
    snapshot.reference_governor_enabled =
        config_.reference_governor_enabled;
    snapshot.payload = state_.payload.Snapshot();
    return snapshot;
}

} // namespace LadyLuck
