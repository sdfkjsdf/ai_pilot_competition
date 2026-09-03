#pragma once

#include "LadyLuck/contracts/Enums.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

namespace LadyLuck
{
struct NedSpecificForceObserverConfig
{
    double nominal_dt_s = 1.0 / 60.0;
    double dt_min_s = 0.5 / 60.0;
    double dt_max_s = 1.5 / 60.0;
};

struct NedSpecificForceObservation
{
    Vector3 specific_force_body_mps2{};
    double nz_pullup_g = 0.0;
    bool valid = false;
    double age_s = 0.0;
    double quality = 0.0;
    EstimatorSource source = EstimatorSource::NedVelocityDifferenceShadow;
    TranslationalGate gate = TranslationalGate::Reset;
    double sample_dt_s = 0.0;
};

// Read-only value snapshot for estimator transaction/oracle evidence.  It
// mirrors every persistent field owned by NedSpecificForceObserver.
struct NedSpecificForceObserverSnapshot
{
    NedSpecificForceObserverConfig config{};
    bool configuration_valid = false;
    bool has_previous_velocity_ned = false;
    Vector3 previous_velocity_ned_mps{};
    double age_s = 0.0;
    NedSpecificForceObservation last{};
};

class NedSpecificForceObserver final
{
public:
    NedSpecificForceObserver() noexcept;
    explicit NedSpecificForceObserver(
        const NedSpecificForceObserverConfig& config) noexcept;

    void Reset() noexcept;
    Result<NedSpecificForceObservation> Step(
        const Vector3& rpy_rad,
        const Vector3& velocity_body_mps,
        double sample_dt_s) noexcept;

    const NedSpecificForceObservation& last() const noexcept;
    const NedSpecificForceObserverConfig& config() const noexcept;
    bool configuration_valid() const noexcept;
    NedSpecificForceObserverSnapshot Snapshot() const noexcept;

private:
    Result<NedSpecificForceObservation> Invalid(
        TranslationalGate gate,
        StatusCode status,
        double sample_dt_s) noexcept;

    NedSpecificForceObserverConfig config_{};
    bool configuration_valid_ = true;
    bool has_previous_velocity_ned_ = false;
    Vector3 previous_velocity_ned_mps_{};
    double age_s_ = 0.0;
    NedSpecificForceObservation last_{};
};
}
