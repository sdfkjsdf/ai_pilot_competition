#include "LadyLuck/observers/BodyRateObserver.hpp"

#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double Epsilon = 1.0e-9;

double Clamp01(const double value) noexcept
{
    return std::max(0.0, std::min(1.0, value));
}

LadyLuck::FeatureGate FeatureGateFor(
    const LadyLuck::BodyRateGate gate) noexcept
{
    using LadyLuck::BodyRateGate;
    using LadyLuck::FeatureGate;
    switch (gate)
    {
    case BodyRateGate::Uninitialized: return FeatureGate::Reset;
    case BodyRateGate::Reset: return FeatureGate::Reset;
    case BodyRateGate::Init: return FeatureGate::Init;
    case BodyRateGate::Update: return FeatureGate::Update;
    case BodyRateGate::InvalidAttitude: return FeatureGate::InvalidAttitude;
    case BodyRateGate::InvalidDt: return FeatureGate::InvalidDt;
    case BodyRateGate::FrameGap: return FeatureGate::FrameGap;
    case BodyRateGate::AmbiguousRotation: return FeatureGate::AmbiguousRotation;
    }
    return FeatureGate::Reset;
}
}

namespace LadyLuck
{
BodyRateObserver::BodyRateObserver() noexcept
    : BodyRateObserver(BodyRateObserverConfig{})
{
}

BodyRateObserver::BodyRateObserver(
    const BodyRateObserverConfig& config) noexcept
    : config_(config)
{
    configuration_valid_ =
        std::isfinite(config_.nominal_dt_s)
        && std::isfinite(config_.dt_min_s)
        && std::isfinite(config_.dt_max_s)
        && std::isfinite(config_.dt_quality_tolerance_fraction)
        && std::isfinite(config_.near_pi_quality_margin_rad)
        && std::isfinite(config_.max_quantization_rate_resolution_rad_s)
        && std::isfinite(config_.min_feature_age_s)
        && std::isfinite(config_.min_feature_quality)
        && config_.dt_min_s > 0.0
        && config_.dt_min_s < config_.dt_max_s
        && config_.nominal_dt_s >= config_.dt_min_s
        && config_.nominal_dt_s <= config_.dt_max_s
        && config_.dt_quality_tolerance_fraction > 0.0
        && config_.near_pi_quality_margin_rad > 0.0
        && config_.max_quantization_rate_resolution_rad_s > 0.0
        && config_.min_feature_age_s > 0.0
        && config_.min_feature_quality >= 0.0
        && config_.min_feature_quality <= 1.0;
    Reset();
}

void BodyRateObserver::Reset() noexcept
{
    has_previous_dcm_ = false;
    previous_dcm_ = Matrix3RowMajor{};
    estimated_dcm_ = Matrix3RowMajor{};
    filtered_ = Vector3{};
    has_previous_interval_ = false;
    previous_interval_rad_s_ = Vector3{};
    filter_ready_ = false;
    age_s_ = 0.0;
    last_ = BodyRateObservation{};
}

Result<BodyRateObservation> BodyRateObserver::Invalid(
    const Matrix3RowMajor* const current_dcm,
    const BodyRateGate gate,
    const StatusCode status,
    const double sample_dt_s) noexcept
{
    if (current_dcm != nullptr)
    {
        previous_dcm_ = *current_dcm;
        estimated_dcm_ = *current_dcm;
        has_previous_dcm_ = true;
    }
    filtered_ = Vector3{};
    has_previous_interval_ = false;
    previous_interval_rad_s_ = Vector3{};
    filter_ready_ = false;
    age_s_ = 0.0;
    last_ = BodyRateObservation{};
    last_.gate = gate;
    last_.feature_gate = FeatureGateFor(gate);
    last_.sample_dt_s = std::isfinite(sample_dt_s) ? sample_dt_s : 0.0;
    Result<BodyRateObservation> result{};
    result.status.code = status;
    result.value = last_;
    return result;
}

void BodyRateObserver::ComputeQuality(
    const Vector3& rpy_rad,
    const double sample_dt_s,
    const double relative_rotation_rad,
    double& quality,
    double& rate_resolution_rad_s) const noexcept
{
    Vector3 ulp_rad{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float degrees = static_cast<float>(rpy_rad[index] * RadiansToDegrees);
        const float next_degrees = std::nextafter(
            degrees,
            std::numeric_limits<float>::infinity());
        const float delta_degrees = next_degrees - degrees;
        ulp_rad[index] = std::fabs(
            static_cast<double>(delta_degrees) * DegreesToRadians);
    }
    rate_resolution_rad_s = VectorNorm(ulp_rad) / std::max(sample_dt_s, Epsilon);

    const double tolerance =
        config_.dt_quality_tolerance_fraction * config_.nominal_dt_s;
    const double dt_quality = Clamp01(
        1.0 - std::fabs(sample_dt_s - config_.nominal_dt_s)
        / std::max(tolerance, Epsilon));
    const double pi_margin = std::max(0.0, Pi - relative_rotation_rad);
    const double rotation_quality = Clamp01(
        pi_margin / config_.near_pi_quality_margin_rad);
    const double quantization_quality = Clamp01(
        1.0 - rate_resolution_rad_s
        / config_.max_quantization_rate_resolution_rad_s);
    quality = std::min(
        dt_quality,
        std::min(rotation_quality, quantization_quality));
}

void BodyRateObserver::ComputeFeatureGate(
    const double quality,
    bool& ready,
    FeatureGate& gate) const noexcept
{
    if (age_s_ + Epsilon < config_.min_feature_age_s)
    {
        ready = false;
        gate = FeatureGate::Warmup;
        return;
    }
    if (quality + Epsilon < config_.min_feature_quality)
    {
        ready = false;
        gate = FeatureGate::LowDataQuality;
        return;
    }
    ready = true;
    gate = FeatureGate::Ready;
}

Result<BodyRateObservation> BodyRateObserver::Step(
    const Vector3& rpy_rad,
    const double sample_dt_s) noexcept
{
    if (!configuration_valid_)
    {
        return Invalid(
            nullptr,
            BodyRateGate::Reset,
            StatusCode::InvalidConfiguration);
    }
    const Result<Matrix3RowMajor> dcm = RpyToDcmNedToBody(rpy_rad);
    if (!dcm.ok())
    {
        return Invalid(
            nullptr,
            BodyRateGate::InvalidAttitude,
            StatusCode::NonFiniteInput);
    }
    if (!std::isfinite(sample_dt_s))
    {
        return Invalid(
            &dcm.value,
            BodyRateGate::InvalidDt,
            StatusCode::InvalidDt,
            sample_dt_s);
    }
    if (!has_previous_dcm_)
    {
        return Invalid(
            &dcm.value,
            BodyRateGate::Init,
            StatusCode::Seeded,
            sample_dt_s);
    }
    if (sample_dt_s < config_.dt_min_s || sample_dt_s > config_.dt_max_s)
    {
        return Invalid(
            &dcm.value,
            BodyRateGate::FrameGap,
            StatusCode::FrameGap,
            sample_dt_s);
    }

    const Matrix3RowMajor relative = MatrixProduct(
        previous_dcm_,
        MatrixTranspose(dcm.value));
    const Result<Vector3> log_vee = RotationLogVee(relative);
    if (!log_vee.ok())
    {
        return Invalid(
            &dcm.value,
            BodyRateGate::AmbiguousRotation,
            StatusCode::ObservationInvalid,
            sample_dt_s);
    }

    Vector3 raw{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        raw[index] = log_vee.value[index] / sample_dt_s;
    }
    const double relative_rotation_rad = VectorNorm(log_vee.value);
    double quality = 0.0;
    double rate_resolution_rad_s = 0.0;
    ComputeQuality(
        rpy_rad,
        sample_dt_s,
        relative_rotation_rad,
        quality,
        rate_resolution_rad_s);

    // ed757e27 active profile: raw only. No low-pass or alpha-beta branch.
    const Vector3 filtered = raw;
    Vector3 endpoint = filtered;
    const bool endpoint_valid = has_previous_interval_;
    if (has_previous_interval_)
    {
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            endpoint[index] =
                1.5 * filtered[index]
                - 0.5 * previous_interval_rad_s_[index];
        }
    }

    previous_dcm_ = dcm.value;
    estimated_dcm_ = dcm.value;
    has_previous_dcm_ = true;
    previous_interval_rad_s_ = filtered;
    has_previous_interval_ = true;
    filtered_ = filtered;
    filter_ready_ = true;
    age_s_ += sample_dt_s;

    last_ = BodyRateObservation{};
    last_.raw_pqr_rad_s = raw;
    last_.pqr_rad_s = filtered;
    last_.pqr_endpoint_rad_s = endpoint;
    last_.endpoint_valid = endpoint_valid;
    last_.valid = true;
    last_.age_s = age_s_;
    last_.gate = BodyRateGate::Update;
    last_.sample_dt_s = sample_dt_s;
    last_.relative_rotation_rad = relative_rotation_rad;
    last_.rate_quantization_resolution_rad_s = rate_resolution_rad_s;
    last_.data_quality = quality;
    ComputeFeatureGate(quality, last_.feature_ready, last_.feature_gate);

    Result<BodyRateObservation> result{};
    result.value = last_;
    return result;
}

const BodyRateObservation& BodyRateObserver::last() const noexcept
{
    return last_;
}

const BodyRateObserverConfig& BodyRateObserver::config() const noexcept
{
    return config_;
}

bool BodyRateObserver::configuration_valid() const noexcept
{
    return configuration_valid_;
}

BodyRateObserverSnapshot BodyRateObserver::Snapshot() const noexcept
{
    BodyRateObserverSnapshot snapshot;
    snapshot.config = config_;
    snapshot.configuration_valid = configuration_valid_;
    snapshot.has_previous_dcm = has_previous_dcm_;
    snapshot.previous_dcm = previous_dcm_;
    snapshot.estimated_dcm = estimated_dcm_;
    snapshot.filtered = filtered_;
    snapshot.has_previous_interval = has_previous_interval_;
    snapshot.previous_interval_rad_s = previous_interval_rad_s_;
    snapshot.filter_ready = filter_ready_;
    snapshot.age_s = age_s_;
    snapshot.last = last_;
    return snapshot;
}
}
