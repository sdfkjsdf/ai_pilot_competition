#include "LadyLuck/observers/NedSpecificForceObserver.hpp"

#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double GravityMps2 = 9.80665;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}
}

namespace LadyLuck
{
NedSpecificForceObserver::NedSpecificForceObserver() noexcept
    : NedSpecificForceObserver(NedSpecificForceObserverConfig{})
{
}

NedSpecificForceObserver::NedSpecificForceObserver(
    const NedSpecificForceObserverConfig& config) noexcept
    : config_(config)
{
    configuration_valid_ =
        std::isfinite(config_.nominal_dt_s)
        && std::isfinite(config_.dt_min_s)
        && std::isfinite(config_.dt_max_s)
        && config_.dt_min_s > 0.0
        && config_.dt_min_s <= config_.nominal_dt_s
        && config_.nominal_dt_s <= config_.dt_max_s;
    Reset();
}

void NedSpecificForceObserver::Reset() noexcept
{
    has_previous_velocity_ned_ = false;
    previous_velocity_ned_mps_ = Vector3{};
    age_s_ = 0.0;
    last_ = NedSpecificForceObservation{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    last_.specific_force_body_mps2 = Vector3{{nan, nan, nan}};
    last_.nz_pullup_g = nan;
}

Result<NedSpecificForceObservation> NedSpecificForceObserver::Invalid(
    const TranslationalGate gate,
    const StatusCode status,
    const double sample_dt_s) noexcept
{
    last_ = NedSpecificForceObservation{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    last_.specific_force_body_mps2 = Vector3{{nan, nan, nan}};
    last_.nz_pullup_g = nan;
    last_.age_s = age_s_;
    last_.gate = gate;
    last_.sample_dt_s = std::isfinite(sample_dt_s) ? sample_dt_s : 0.0;
    Result<NedSpecificForceObservation> result{};
    result.status.code = status;
    result.value = last_;
    return result;
}

Result<NedSpecificForceObservation> NedSpecificForceObserver::Step(
    const Vector3& rpy_rad,
    const Vector3& velocity_body_mps,
    const double sample_dt_s) noexcept
{
    if (!configuration_valid_)
    {
        return Invalid(
            TranslationalGate::Reset,
            StatusCode::InvalidConfiguration,
            sample_dt_s);
    }
    const Result<Matrix3RowMajor> dcm = RpyToDcmNedToBody(rpy_rad);
    if (!dcm.ok())
    {
        has_previous_velocity_ned_ = false;
        previous_velocity_ned_mps_ = Vector3{};
        age_s_ = 0.0;
        return Invalid(
            TranslationalGate::InvalidSample,
            StatusCode::NonFiniteInput,
            sample_dt_s);
    }
    if (!FiniteVector(velocity_body_mps))
    {
        has_previous_velocity_ned_ = false;
        previous_velocity_ned_mps_ = Vector3{};
        age_s_ = 0.0;
        return Invalid(
            TranslationalGate::InvalidVelocity,
            StatusCode::NonFiniteInput,
            sample_dt_s);
    }

    const Vector3 velocity_ned = TransposeMatrixVectorProduct(
        dcm.value,
        velocity_body_mps);
    if (!std::isfinite(sample_dt_s))
    {
        previous_velocity_ned_mps_ = velocity_ned;
        has_previous_velocity_ned_ = true;
        age_s_ = 0.0;
        return Invalid(
            TranslationalGate::InvalidDt,
            StatusCode::InvalidDt,
            sample_dt_s);
    }
    if (!has_previous_velocity_ned_)
    {
        previous_velocity_ned_mps_ = velocity_ned;
        has_previous_velocity_ned_ = true;
        return Invalid(
            TranslationalGate::Init,
            StatusCode::Seeded,
            sample_dt_s);
    }
    if (sample_dt_s < config_.dt_min_s || sample_dt_s > config_.dt_max_s)
    {
        previous_velocity_ned_mps_ = velocity_ned;
        age_s_ = 0.0;
        return Invalid(
            TranslationalGate::FrameGap,
            StatusCode::FrameGap,
            sample_dt_s);
    }

    Vector3 acceleration_ned{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        acceleration_ned[index] =
            (velocity_ned[index] - previous_velocity_ned_mps_[index])
            / sample_dt_s;
    }
    previous_velocity_ned_mps_ = velocity_ned;
    const Vector3 specific_force_ned{{
        acceleration_ned[0],
        acceleration_ned[1],
        acceleration_ned[2] - GravityMps2}};
    const Vector3 specific_force_body = MatrixVectorProduct(
        dcm.value,
        specific_force_ned);
    age_s_ += sample_dt_s;
    const double dt_error =
        std::fabs(sample_dt_s - config_.nominal_dt_s)
        / std::max(config_.nominal_dt_s, 1.0e-9);

    last_ = NedSpecificForceObservation{};
    last_.specific_force_body_mps2 = specific_force_body;
    last_.nz_pullup_g = -specific_force_body[2] / GravityMps2;
    last_.valid = true;
    last_.age_s = age_s_;
    last_.quality = std::max(0.0, std::min(1.0, 1.0 - dt_error));
    last_.gate = TranslationalGate::Update;
    last_.sample_dt_s = sample_dt_s;
    Result<NedSpecificForceObservation> result{};
    result.value = last_;
    return result;
}

const NedSpecificForceObservation& NedSpecificForceObserver::last() const noexcept
{
    return last_;
}

const NedSpecificForceObserverConfig& NedSpecificForceObserver::config() const noexcept
{
    return config_;
}

bool NedSpecificForceObserver::configuration_valid() const noexcept
{
    return configuration_valid_;
}

NedSpecificForceObserverSnapshot NedSpecificForceObserver::Snapshot() const noexcept
{
    NedSpecificForceObserverSnapshot snapshot;
    snapshot.config = config_;
    snapshot.configuration_valid = configuration_valid_;
    snapshot.has_previous_velocity_ned = has_previous_velocity_ned_;
    snapshot.previous_velocity_ned_mps = previous_velocity_ned_mps_;
    snapshot.age_s = age_s_;
    snapshot.last = last_;
    return snapshot;
}
}
