#pragma once

#include "LadyLuck/contracts/Enums.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"

namespace LadyLuck
{
struct BodyRateObserverConfig
{
    double nominal_dt_s = 1.0 / 60.0;
    double dt_min_s = 0.5 / 60.0;
    double dt_max_s = 1.5 / 60.0;
    double dt_quality_tolerance_fraction = 0.50;
    double near_pi_quality_margin_rad = 0.25;
    double max_quantization_rate_resolution_rad_s = 0.01;
    double min_feature_age_s = 0.05;
    double min_feature_quality = 0.80;
};

struct BodyRateObservation
{
    Vector3 raw_pqr_rad_s{};
    Vector3 pqr_rad_s{};
    Vector3 pqr_endpoint_rad_s{};
    bool endpoint_valid = false;
    bool valid = false;
    double age_s = 0.0;
    BodyRateGate gate = BodyRateGate::Reset;
    double sample_dt_s = 0.0;
    double relative_rotation_rad = 0.0;
    double rate_quantization_resolution_rad_s = 0.0;
    double data_quality = 0.0;
    bool feature_ready = false;
    FeatureGate feature_gate = FeatureGate::Reset;
};

// Read-only value snapshot for estimator transaction/oracle evidence.  It
// mirrors every persistent field owned by BodyRateObserver; restoring or
// mutating observer internals through this type is intentionally unsupported.
struct BodyRateObserverSnapshot
{
    BodyRateObserverConfig config{};
    bool configuration_valid = false;
    bool has_previous_dcm = false;
    Matrix3RowMajor previous_dcm{};
    Matrix3RowMajor estimated_dcm{};
    Vector3 filtered{};
    bool has_previous_interval = false;
    Vector3 previous_interval_rad_s{};
    bool filter_ready = false;
    double age_s = 0.0;
    BodyRateObservation last{};
};

class BodyRateObserver final
{
public:
    BodyRateObserver() noexcept;
    explicit BodyRateObserver(const BodyRateObserverConfig& config) noexcept;

    void Reset() noexcept;
    Result<BodyRateObservation> Step(
        const Vector3& rpy_rad,
        double sample_dt_s) noexcept;

    const BodyRateObservation& last() const noexcept;
    const BodyRateObserverConfig& config() const noexcept;
    bool configuration_valid() const noexcept;
    BodyRateObserverSnapshot Snapshot() const noexcept;

private:
    Result<BodyRateObservation> Invalid(
        const Matrix3RowMajor* current_dcm,
        BodyRateGate gate,
        StatusCode status,
        double sample_dt_s = 0.0) noexcept;
    void ComputeQuality(
        const Vector3& rpy_rad,
        double sample_dt_s,
        double relative_rotation_rad,
        double& quality,
        double& rate_resolution_rad_s) const noexcept;
    void ComputeFeatureGate(
        double quality,
        bool& ready,
        FeatureGate& gate) const noexcept;

    BodyRateObserverConfig config_{};
    bool configuration_valid_ = true;
    bool has_previous_dcm_ = false;
    Matrix3RowMajor previous_dcm_{};
    Matrix3RowMajor estimated_dcm_{};
    Vector3 filtered_{};
    bool has_previous_interval_ = false;
    Vector3 previous_interval_rad_s_{};
    bool filter_ready_ = false;
    double age_s_ = 0.0;
    BodyRateObservation last_{};
};
}
