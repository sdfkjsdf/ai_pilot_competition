#include "LadyLuck/estimator/LightNzEstimator.hpp"

#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double GravityMps2 = 9.80665;
constexpr double ActionDomainTolerance = 1.0e-12;

bool Finite3(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

LadyLuck::Status Failure(const LadyLuck::StatusCode code) noexcept
{
    LadyLuck::Status status;
    status.code = code;
    return status;
}

LadyLuck::Status FromPlantStatus(
    const LadyLuck::plant::PlantStatus& plant_status) noexcept
{
    using LadyLuck::StatusCode;
    using LadyLuck::plant::PlantStatusCode;
    switch (plant_status.code)
    {
    case PlantStatusCode::Ok:
        return Failure(StatusCode::Ok);
    case PlantStatusCode::InvalidArgument:
    case PlantStatusCode::InvalidState:
        return Failure(StatusCode::InvalidArgument);
    case PlantStatusCode::NonFiniteResult:
        return Failure(StatusCode::NonFiniteInput);
    case PlantStatusCode::DependencyFailure:
        return Failure(StatusCode::InvalidConfiguration);
    }
    return Failure(StatusCode::InvalidConfiguration);
}

LadyLuck::StatusCode AcceptedObserverReceipt(
    const LadyLuck::StatusCode body_rate,
    const LadyLuck::StatusCode translational) noexcept
{
    if (body_rate != LadyLuck::StatusCode::Ok)
    {
        return body_rate;
    }
    return translational;
}

bool PublishedAtmosphere(
    const double altitude_m,
    double& density_kg_m3,
    double& speed_of_sound_mps) noexcept
{
    const double altitude = std::max(altitude_m, 0.0);
    double temperature_k = 0.0;
    double pressure_pa = 0.0;
    if (altitude <= 11000.0)
    {
        temperature_k = 288.15 - 0.0065 * altitude;
        pressure_pa = 101325.0 * std::pow(
            temperature_k / 288.15,
            5.2559);
    }
    else
    {
        temperature_k = 216.65;
        pressure_pa = 22632.0 * std::exp(
            -(altitude - 11000.0) / 6341.6);
    }
    density_kg_m3 = pressure_pa / (287.05 * temperature_k);
    // Published mach follows light_nz._atmo_engine's np.sqrt path. The plant
    // and engine path separately retain ProcessModel::Isa's pow(..., 0.5).
    speed_of_sound_mps = std::sqrt(1.4 * 287.05 * temperature_k);
    return std::isfinite(density_kg_m3)
        && density_kg_m3 >= 0.0
        && std::isfinite(speed_of_sound_mps)
        && speed_of_sound_mps > 0.0;
}

LadyLuck::Status ValidateInput(
    const LadyLuck::LightNzUpdateInput& input,
    const bool ready,
    const bool configuration_valid,
    const bool has_auxiliary,
    const bool has_engine) noexcept
{
    using LadyLuck::StatusCode;
    if (!configuration_valid)
    {
        return Failure(StatusCode::InvalidConfiguration);
    }
    if (!std::isfinite(input.sample_dt_s) || input.sample_dt_s <= 0.0)
    {
        return Failure(StatusCode::InvalidDt);
    }
    if (!Finite3(input.state.position_ned_m)
        || !Finite3(input.state.rpy_rad)
        || !Finite3(input.state.velocity_body_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    const LadyLuck::Command4Estimator& action =
        input.feedback.estimator_command_u_dll;
    for (const double value : action)
    {
        if (!std::isfinite(value))
        {
            return Failure(StatusCode::NonFiniteInput);
        }
    }
    const double lower[4] = {-1.0, -1.0, -1.0, 0.0};
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        if (action[index] < lower[index] - ActionDomainTolerance
            || action[index] > 1.0 + ActionDomainTolerance)
        {
            return Failure(StatusCode::InvalidArgument);
        }
    }
    if (input.feedback.source_t_sec.has_value
        && !std::isfinite(input.feedback.source_t_sec.value))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    if (!ready
        && (!std::isfinite(input.initial_mass_kg)
            || input.initial_mass_kg <= 0.0))
    {
        return Failure(
            std::isfinite(input.initial_mass_kg)
                ? StatusCode::InvalidArgument
                : StatusCode::NonFiniteInput);
    }
    if (ready && (!has_auxiliary || !has_engine))
    {
        return Failure(StatusCode::InvalidConfiguration);
    }
    return Failure(StatusCode::Ok);
}

LadyLuck::Status PopulateRecoveredKinematics(
    const LadyLuck::LightNzUpdateInput& input,
    const double gear_position_normalized,
    LadyLuck::EstimatorOutputV6& output) noexcept
{
    using LadyLuck::Matrix3RowMajor;
    using LadyLuck::StatusCode;
    using LadyLuck::Vector3;

    const double u = input.state.velocity_body_mps[0];
    const double v = input.state.velocity_body_mps[1];
    const double w = input.state.velocity_body_mps[2];
    const double altitude_m = -input.state.position_ned_m[2];
    const double speed_squared = u * u + v * v + w * w;
    // Python's published V uses **0.5. Preserve that expression separately
    // from the engine seed/live std::sqrt path below.
    const double speed_mps = std::max(std::pow(speed_squared, 0.5), 1.0e-6);
    if (!std::isfinite(speed_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    const double alpha_rad = std::atan2(w, u);
    const double beta_rad = std::asin(std::max(
        -1.0,
        std::min(1.0, v / speed_mps)));
    double density_kg_m3 = 0.0;
    double speed_of_sound_mps = 0.0;
    if (!PublishedAtmosphere(
            altitude_m,
            density_kg_m3,
            speed_of_sound_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    const double mach = speed_of_sound_mps > 0.0
        ? speed_mps / speed_of_sound_mps
        : 0.0;
    const double qbar_pa = 0.5 * density_kg_m3 * speed_mps * speed_mps;

    const LadyLuck::Result<Matrix3RowMajor> dcm =
        LadyLuck::RpyToDcmNedToBody(input.state.rpy_rad);
    if (!dcm.ok())
    {
        return dcm.status;
    }
    const double cosine_alpha = std::cos(alpha_rad);
    const double sine_alpha = std::sin(alpha_rad);
    const double cosine_beta = std::cos(beta_rad);
    const double sine_beta = std::sin(beta_rad);
    const Matrix3RowMajor dcm_wind_to_body{{
        cosine_alpha * cosine_beta,
        -cosine_alpha * sine_beta,
        -sine_alpha,
        sine_beta,
        cosine_beta,
        0.0,
        sine_alpha * cosine_beta,
        -sine_alpha * sine_beta,
        cosine_alpha}};
    const Matrix3RowMajor dcm_ned_to_wind = LadyLuck::MatrixProduct(
        LadyLuck::MatrixTranspose(dcm_wind_to_body),
        dcm.value);
    const double mu_rad = std::atan2(
        dcm_ned_to_wind[5],
        dcm_ned_to_wind[8]);
    const Vector3 velocity_ned_mps = LadyLuck::TransposeMatrixVectorProduct(
        dcm.value,
        input.state.velocity_body_mps);
    const double ground_speed_horizontal_mps = std::sqrt(
        velocity_ned_mps[0] * velocity_ned_mps[0]
        + velocity_ned_mps[1] * velocity_ned_mps[1]);
    if (!std::isfinite(alpha_rad) || !std::isfinite(beta_rad)
        || !std::isfinite(mach) || !std::isfinite(qbar_pa)
        || !std::isfinite(mu_rad)
        || !std::isfinite(ground_speed_horizontal_mps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }

    output.u = u;
    output.v = v;
    output.w = w;
    output.V = speed_mps;
    output.alt = altitude_m;
    output.roll = input.state.rpy_rad[0];
    output.pitch = input.state.rpy_rad[1];
    output.yaw = input.state.rpy_rad[2];
    output.alpha = alpha_rad;
    output.beta = beta_rad;
    output.mu = mu_rad;
    output.mach = mach;
    output.qbar = qbar_pa;
    output.ground_speed_horizontal_mps = ground_speed_horizontal_mps;
    output.gear_pos_norm = gear_position_normalized;
    return Failure(StatusCode::Ok);
}

void PopulateFeedbackMetadata(
    const LadyLuck::CommandFeedback& feedback,
    const double sample_dt_s,
    LadyLuck::EstimatorOutputV6& output) noexcept
{
    output.action_feedback_kind = feedback.kind;
    output.action_source_frame_index = feedback.source_frame_index;
    output.action_source_t_sec = feedback.source_t_sec;
    output.action_feedback_delay_frames = feedback.delay_frames;
    output.sample_dt_s = sample_dt_s;
}

double ProviderThrottleToInternal(const double provider_throttle) noexcept
{
    return 0.5 * (provider_throttle + 1.0);
}

} // namespace

namespace LadyLuck
{

LightNzEstimator::LightNzEstimator(
    const double gear_position_normalized) noexcept
    : gear_position_normalized_(gear_position_normalized)
{
    configuration_valid_ = std::isfinite(gear_position_normalized_)
        && gear_position_normalized_ >= 0.0
        && gear_position_normalized_ <= 1.0
        && body_rate_observer_.configuration_valid()
        && translational_observer_.configuration_valid();
    Reset();
}

void LightNzEstimator::Reset() noexcept
{
    body_rate_observer_.Reset();
    translational_observer_.Reset();
    ready_ = false;
    age_s_ = 0.0;
    has_auxiliary_ = false;
    auxiliary_ = plant::AuxState{};
    has_engine_ = false;
    engine_ = plant::dynamics::EngineModel{};
}

LightNzEstimator LightNzEstimator::FreshCandidate() const noexcept
{
    return LightNzEstimator(gear_position_normalized_);
}

LightNzEstimator LightNzEstimator::DetachedCopy() const noexcept
{
    return *this;
}

Result<EstimatorOutputV6> LightNzEstimator::Step(
    const LightNzUpdateInput& input) noexcept
{
    LightNzEstimator candidate = *this;
    Result<EstimatorOutputV6> result = candidate.StepInPlace(input);
    if (result.ok())
    {
        *this = candidate;
    }
    return result;
}

Result<EstimatorOutputV6> LightNzEstimator::StepInPlace(
    const LightNzUpdateInput& input) noexcept
{
    Result<EstimatorOutputV6> result{};
    result.status = ValidateInput(
        input,
        ready_,
        configuration_valid_,
        has_auxiliary_,
        has_engine_);
    if (!result.ok())
    {
        return result;
    }

    EstimatorOutputV6 output;
    result.status = PopulateRecoveredKinematics(
        input,
        gear_position_normalized_,
        output);
    if (!result.ok())
    {
        return result;
    }
    PopulateFeedbackMetadata(
        input.feedback,
        input.sample_dt_s,
        output);

    const Result<BodyRateObservation> body_rate = body_rate_observer_.Step(
        input.state.rpy_rad,
        input.sample_dt_s);
    if (!body_rate.ok())
    {
        result.status = body_rate.status;
        return result;
    }
    const Result<NedSpecificForceObservation> translation =
        translational_observer_.Step(
            input.state.rpy_rad,
            input.state.velocity_body_mps,
            input.sample_dt_s);
    if (!translation.ok())
    {
        result.status = translation.status;
        return result;
    }

    const Command4Estimator& action = input.feedback.estimator_command_u_dll;
    if (!ready_)
    {
        auxiliary_ = process_model_.InitialAuxiliaryState(
            input.initial_mass_kg);
        has_auxiliary_ = true;
        engine_ = plant::dynamics::EngineModel{};
        const plant::PlantStatus reset_status = engine_.Reset();
        if (!reset_status.ok())
        {
            result.status = FromPlantStatus(reset_status);
            return result;
        }
        const double speed_mps = std::max(
            std::sqrt(
                input.state.velocity_body_mps[0]
                    * input.state.velocity_body_mps[0]
                + input.state.velocity_body_mps[1]
                    * input.state.velocity_body_mps[1]
                + input.state.velocity_body_mps[2]
                    * input.state.velocity_body_mps[2]),
            1.0e-6);
        const plant::PlantResult<plant::core::IsaState> atmosphere =
            plant::core::ProcessModel::Isa(output.alt);
        if (!atmosphere.ok())
        {
            result.status = FromPlantStatus(atmosphere.status);
            return result;
        }
        const plant::PlantResult<double> seed_fuel =
            engine_.SeedStaticFuelFlow(
                ProviderThrottleToInternal(action[3]),
                speed_mps / atmosphere.value.speed_of_sound_mps,
                output.alt);
        if (!seed_fuel.ok())
        {
            result.status = FromPlantStatus(seed_fuel.status);
            return result;
        }
        has_engine_ = true;
        ready_ = true;

        output.nz = 1.0;
        output.nz_model = 1.0;
        output.nz_flaperon = 0.0;
        output.nz_flaperon_valid = false;
        output.nz_kinematic = translation.value.nz_pullup_g;
        output.nz_fused = 1.0;
        output.nz_valid = false;
        output.nz_age_s = 0.0;
        output.nz_quality = 0.0;
        output.nz_gate = EstimatorGate::SeedUnvalidated;
        output.nz_source = EstimatorSource::ModelSeedUnvalidated;
        output.mass = auxiliary_.mass_kg;
        output.mass_valid = true;
        output.mass_age_s = 0.0;
        output.elevator_rad = 0.0;
        output.elevator_valid = false;
        output.elevator_quality = 0.0;
        output.elevator_gate = EstimatorGate::ConstructionSeedUnvalidated;
        output.elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
        output.p = body_rate.value.pqr_rad_s[0];
        output.q = body_rate.value.pqr_rad_s[1];
        output.r = body_rate.value.pqr_rad_s[2];
        output.pqr_valid = body_rate.value.valid;
        output.pqr_age_s = body_rate.value.age_s;
        output.pqr_quality = body_rate.value.data_quality;
        output.pqr_gate = body_rate.value.gate;
        output.nz_kinematic_valid = translation.value.valid;
        output.nz_kinematic_quality = translation.value.quality;
        output.nz_kinematic_gate = translation.value.gate;
        output.thrust_valid = false;
        result.value = output;
        result.status.code = StatusCode::Seeded;
        return result;
    }

    const Result<QuaternionWxyz> quaternion = Euler321ToQuaternion(
        input.state.rpy_rad);
    if (!quaternion.ok())
    {
        result.status = quaternion.status;
        return result;
    }
    plant::PlantState plant_state;
    plant_state.position_ned_m = input.state.position_ned_m;
    plant_state.quaternion_wxyz = quaternion.value;
    plant_state.velocity_body_mps = input.state.velocity_body_mps;
    plant_state.omega_body_rad_s = body_rate.value.pqr_rad_s;

    const double engine_speed_mps = std::max(
        std::sqrt(
            input.state.velocity_body_mps[0]
                * input.state.velocity_body_mps[0]
            + input.state.velocity_body_mps[1]
                * input.state.velocity_body_mps[1]
            + input.state.velocity_body_mps[2]
                * input.state.velocity_body_mps[2]),
        1.0e-6);
    const plant::PlantResult<plant::core::IsaState> atmosphere =
        plant::core::ProcessModel::Isa(output.alt);
    if (!atmosphere.ok())
    {
        result.status = FromPlantStatus(atmosphere.status);
        return result;
    }
    const plant::PlantResult<plant::dynamics::EngineStepOutput> engine_step =
        engine_.ThrustAndFuel(
            ProviderThrottleToInternal(action[3]),
            engine_speed_mps / atmosphere.value.speed_of_sound_mps,
            output.alt,
            input.sample_dt_s);
    if (!engine_step.ok())
    {
        result.status = FromPlantStatus(engine_step.status);
        return result;
    }

    plant::core::ProcessStepInput process_input;
    process_input.state = plant_state;
    process_input.auxiliary = auxiliary_;
    process_input.command = action;
    process_input.thrust_n = engine_step.value.thrust_n;
    process_input.dt_s = input.sample_dt_s;
    process_input.gear_position_normalized = gear_position_normalized_;
    process_input.fuel_flow_lb_s = engine_step.value.fuel_flow_lb_s;
    if (body_rate.value.endpoint_valid)
    {
        process_input.omega_aero_rad_s.Set(
            body_rate.value.pqr_endpoint_rad_s);
    }
    const plant::PlantResult<plant::core::ProcessStepOutput> process_step =
        process_model_.StepEstimateModern(process_input);
    if (!process_step.ok())
    {
        result.status = FromPlantStatus(process_step.status);
        return result;
    }
    auxiliary_ = process_step.value.next_auxiliary;
    age_s_ += input.sample_dt_s;

    const double nz_model = -process_step.value.force_z_n
        / (process_step.value.mass_kg * GravityMps2);
    const double nz_flaperon = -process_step.value.flaperon_force_z_n
        / (process_step.value.mass_kg * GravityMps2);
    if (!std::isfinite(nz_model) || !std::isfinite(nz_flaperon))
    {
        result.status = Failure(StatusCode::NonFiniteInput);
        return result;
    }

    bool nz_valid = false;
    EstimatorGate nz_gate = EstimatorGate::DependencyPqrIntervalInvalid;
    EstimatorSource nz_source =
        EstimatorSource::AeroModelInvalidPqrSentinelKinematicFusionDisabled;
    if (body_rate.value.valid && !body_rate.value.endpoint_valid)
    {
        nz_gate = EstimatorGate::DependencyPqrEndpointInvalid;
        nz_source =
            EstimatorSource::AeroModelIntervalFallbackKinematicFusionDisabled;
    }
    else if (body_rate.value.valid && body_rate.value.endpoint_valid)
    {
        nz_valid = true;
        nz_gate = EstimatorGate::LiveEndpoint;
        nz_source =
            EstimatorSource::AeroModelEndpointPqrKinematicFusionDisabled;
    }

    output.nz = nz_model;
    output.nz_model = nz_model;
    output.nz_flaperon = nz_flaperon;
    output.nz_flaperon_valid = nz_valid;
    output.nz_kinematic = translation.value.nz_pullup_g;
    output.nz_fused = nz_model;
    output.nz_valid = nz_valid;
    output.nz_age_s = age_s_;
    output.nz_quality = nz_valid ? 1.0 : 0.0;
    output.nz_gate = nz_gate;
    output.nz_source = nz_source;
    output.mass = process_step.value.mass_kg;
    output.mass_valid = true;
    output.mass_age_s = age_s_;
    output.thrust = engine_step.value.thrust_n;
    output.thrust_valid = std::isfinite(output.thrust);
    output.elevator_rad = process_step.value.elevator_rad;
    output.elevator_valid = true;
    output.elevator_quality = 1.0;
    output.elevator_gate = EstimatorGate::LiveActuatorReplica;
    output.elevator_source = EstimatorSource::FbwActuatorReplicaLive;
    output.p = body_rate.value.pqr_rad_s[0];
    output.q = body_rate.value.pqr_rad_s[1];
    output.r = body_rate.value.pqr_rad_s[2];
    output.p_endpoint = body_rate.value.pqr_endpoint_rad_s[0];
    output.q_endpoint = body_rate.value.pqr_endpoint_rad_s[1];
    output.r_endpoint = body_rate.value.pqr_endpoint_rad_s[2];
    output.pqr_endpoint_valid = body_rate.value.endpoint_valid;
    output.pqr_valid = body_rate.value.valid;
    output.pqr_age_s = body_rate.value.age_s;
    output.pqr_quality = body_rate.value.data_quality;
    output.pqr_gate = body_rate.value.gate;
    output.nz_kinematic_valid = translation.value.valid;
    output.nz_kinematic_quality = translation.value.quality;
    output.nz_kinematic_gate = translation.value.gate;

    result.value = output;
    result.status.code = AcceptedObserverReceipt(
        body_rate.status.code,
        translation.status.code);
    return result;
}

LightNzSnapshot LightNzEstimator::Snapshot() const noexcept
{
    LightNzSnapshot snapshot;
    snapshot.gear_position_normalized = gear_position_normalized_;
    snapshot.configuration_valid = configuration_valid_;
    snapshot.ready = ready_;
    snapshot.age_s = age_s_;
    snapshot.body_rate_observer = body_rate_observer_.Snapshot();
    snapshot.translational_observer = translational_observer_.Snapshot();
    snapshot.has_auxiliary = has_auxiliary_;
    snapshot.auxiliary = auxiliary_;
    snapshot.has_engine = has_engine_;
    snapshot.engine_n2_percent = engine_.n2_percent();
    snapshot.engine_fuel_flow_pph = engine_.fuel_flow_pph();
    return snapshot;
}

bool LightNzEstimator::configuration_valid() const noexcept
{
    return configuration_valid_;
}

bool LightNzEstimator::ready() const noexcept
{
    return ready_;
}

double LightNzEstimator::age_s() const noexcept
{
    return age_s_;
}

double LightNzEstimator::gear_position_normalized() const noexcept
{
    return gear_position_normalized_;
}

} // namespace LadyLuck
