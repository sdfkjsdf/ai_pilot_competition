#pragma once

#include "LadyLuck/contracts/CommandFeedback.hpp"
#include "LadyLuck/estimator/EstimatorTransaction.hpp"
#include "LadyLuck/estimator/LightNzEstimator.hpp"

namespace LadyLuck
{

#if defined(LADYLUCK_PRODUCTION_BUILD) \
    && defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
#error LADYLUCK_ESTIMATOR_TEST_SEAMS must not be enabled in a production build
#endif

#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
struct EstimatorTestSeamState;
class PlaneStateEstimatorTestAccess;
#endif

struct PlaneStateEstimatorConfig
{
    double initial_mass_kg = 11159.27948674;
    double nominal_dt_s = 1.0 / 60.0;
    bool reference_governor_enabled = true;
    double gear_pos_norm = 0.0;
};

struct FrameContextRequest
{
    std::uint64_t measurement_frame_index = 0U;
    OptionalFrameIndex command_frame_index{};
    OptionalFrameIndex previous_command_frame{};
    double estimator_soft_budget_s = 0.005;
};

struct PlaneStateEstimatorSnapshot
{
    std::uint64_t episode_epoch = 0U;
    std::uint64_t estimator_epoch = 0U;
    OptionalFrameIndex accepted_measurement_frame_index{};
    OptionalSeconds accepted_sample_t_s{};
    double configured_initial_mass_kg = 0.0;
    double nominal_dt_s = 0.0;
    bool reference_governor_enabled = true;
    LightNzSnapshot payload{};
};

#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
struct EstimatorCandidateEvidenceForTest
{
    bool available = false;
    bool update_completed = false;
    bool output_fault_injected = false;
    bool has_output_after_update = false;
    LightNzSnapshot before_update{};
    LightNzSnapshot after_update{};
    EstimatorOutputV6 output_after_update{};
};

struct EstimatorTestSeamState
{
    EstimatorCandidateFaultForTest next_fault =
        EstimatorCandidateFaultForTest::None;
    EstimatorCandidateEvidenceForTest candidate_evidence{};
};
#endif

class PlaneStateEstimator final
{
public:
    PlaneStateEstimator() noexcept;
    explicit PlaneStateEstimator(
        const PlaneStateEstimatorConfig& config) noexcept;

    bool configuration_valid() const noexcept;
    void MakeFrameContext(
        const FrameContextRequest& request,
        FrameContextBuildResult& output) const noexcept;
    EstimatorUpdateResult Update(
        const KinematicObservation& ownship,
        const CommandFeedback& feedback,
        const FrameContext& context) noexcept;
    EstimatorTransactionReceipt Reset() noexcept;

    PlaneStateEstimatorSnapshot Snapshot() const noexcept;

private:
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    friend class PlaneStateEstimatorTestAccess;
#endif

    struct FrameState
    {
        std::uint64_t episode_epoch = 0U;
        std::uint64_t estimator_epoch = 0U;
        OptionalFrameIndex accepted_measurement_frame_index{};
        OptionalSeconds accepted_sample_t_s{};
        LightNzEstimator payload{};
    };

    struct ChronologyDecision
    {
        EstimatorTransactionCode code =
            EstimatorTransactionCode::InvalidConfiguration;
        EstimatorTransactionCause cause =
            EstimatorTransactionCause::FrameContextInvalidReject;
        EstimatorFaultClass fault_class = EstimatorFaultClass::InternalFault;
        std::uint64_t episode_epoch = 0U;
        std::uint64_t gap_count = 0U;
        GapPolicy gap_policy = GapPolicy::FirstSeed;
    };

    EstimatorTransactionReceipt Rejection(
        EstimatorTransactionCode code,
        EstimatorTransactionCause cause,
        const FrameContext* context) const noexcept;
    void ClassifyChronology(
        std::uint64_t measurement_frame_index,
        ChronologyDecision& output) const noexcept;
    static bool ValidateFeedbackStructure(
        const CommandFeedback& feedback,
        const FrameContext& context) noexcept;
    static EstimatorTransactionCause TransactionCause(
        const FrameState& current,
        const FrameContext& context,
        bool seed_transition) noexcept;
    static void BindContext(
        EstimatorOutputV6& output,
        const FrameContext& context) noexcept;
    static void BindValidityMetadata(
        EstimatorOutputV6& output,
        EstimatorTransactionCause cause) noexcept;
    static bool ValidateCandidate(
        const EstimatorOutputV6& output,
        const LightNzSnapshot& candidate,
        EstimatorTransactionCause cause) noexcept;

    PlaneStateEstimatorConfig config_{};
    bool configuration_valid_ = true;
    FrameState state_{};
#if defined(LADYLUCK_ESTIMATOR_TEST_SEAMS)
    EstimatorTestSeamState* test_seam_state_ = nullptr;
#endif
};

} // namespace LadyLuck
