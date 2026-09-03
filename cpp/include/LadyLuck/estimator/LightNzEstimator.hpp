#pragma once

#include "LadyLuck/contracts/CommandFeedback.hpp"
#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/observers/BodyRateObserver.hpp"
#include "LadyLuck/observers/NedSpecificForceObserver.hpp"
#include "LadyLuck/plant/core/ProcessModel.hpp"
#include "LadyLuck/plant/dynamics/EngineModel.hpp"

namespace LadyLuck
{

// Inputs owned by Python LightNzEstimator.update(). Measurement-frame clocks
// remain the outer PlaneStateEstimator transaction's responsibility.
struct LightNzUpdateInput
{
    PlaneState state{};
    CommandFeedback feedback{};
    double initial_mass_kg = 0.0;
    double sample_dt_s = 0.0;
};

// Complete read-only value snapshot of every transaction-mutable LightNz root
// field. Immutable ProcessModel tables/models are deliberately excluded.
struct LightNzSnapshot
{
    double gear_position_normalized = 0.0;
    bool configuration_valid = false;
    bool ready = false;
    double age_s = 0.0;
    BodyRateObserverSnapshot body_rate_observer{};
    NedSpecificForceObserverSnapshot translational_observer{};
    bool has_auxiliary = false;
    plant::AuxState auxiliary{};
    bool has_engine = false;
    double engine_n2_percent = 0.0;
    double engine_fuel_flow_pph = 0.0;
};

class LightNzEstimator final
{
public:
    // add/main's CANONICAL_GEAR_POS_NORM is 0.0 for the V1 clean profile.
    explicit LightNzEstimator(
        double gear_position_normalized = 0.0) noexcept;

    void Reset() noexcept;
    LightNzEstimator FreshCandidate() const noexcept;
    LightNzEstimator DetachedCopy() const noexcept;

    // Atomic with respect to this estimator root. The input is evaluated on a
    // detached value copy and installed only for accepted (non-negative)
    // receipts, including observer Seeded/FrameGap/ObservationInvalid states.
    Result<EstimatorOutputV6> Step(
        const LightNzUpdateInput& input) noexcept;

    LightNzSnapshot Snapshot() const noexcept;
    bool configuration_valid() const noexcept;
    bool ready() const noexcept;
    double age_s() const noexcept;
    double gear_position_normalized() const noexcept;

private:
    Result<EstimatorOutputV6> StepInPlace(
        const LightNzUpdateInput& input) noexcept;

    plant::core::ProcessModel process_model_{};
    double gear_position_normalized_ = 0.0;
    bool configuration_valid_ = true;
    BodyRateObserver body_rate_observer_{};
    NedSpecificForceObserver translational_observer_{};
    bool ready_ = false;
    double age_s_ = 0.0;
    bool has_auxiliary_ = false;
    plant::AuxState auxiliary_{};
    bool has_engine_ = false;
    plant::dynamics::EngineModel engine_{};
};

} // namespace LadyLuck
