#pragma once

#include "LadyLuck/contracts/Enums.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"

#include <cstddef>
#include <cstdint>

namespace LadyLuck
{
constexpr std::size_t EstimatorOutputV6FieldCount = 66U;

// Field-for-field typed representation of Python EstimatorOutput schema v6.
// Python strings are closed enums so this contract remains allocation-free.
// Numeric defaults are finite. Availability is carried only by the adjacent
// validity/gate fields so an uninitialized diagnostic cannot introduce NaN
// into the 60 Hz controller state.
struct EstimatorOutputV6
{
    EstimatorSchemaVersion schema_version = EstimatorSchemaVersion::DeployableEstimatorOutputV6;
    double nz = 0.0;
    double mass = 0.0;
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    double V = 0.0;
    double alt = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double alpha = 0.0;
    double beta = 0.0;
    double mu = 0.0;
    double mach = 0.0;
    double qbar = 0.0;
    double thrust = 0.0;
    double elevator_rad = 0.0;
    bool elevator_valid = false;
    double elevator_quality = 0.0;
    EstimatorGate elevator_gate = EstimatorGate::Uninitialized;
    EstimatorSource elevator_source = EstimatorSource::FbwActuatorReplicaSeed;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;
    double p_endpoint = 0.0;
    double q_endpoint = 0.0;
    double r_endpoint = 0.0;
    bool pqr_endpoint_valid = false;
    double ground_speed_horizontal_mps = 0.0;
    double gear_pos_norm = 0.0;
    double nz_model = 0.0;
    double nz_flaperon = 0.0;
    bool nz_flaperon_valid = false;
    double nz_kinematic = 0.0;
    double nz_fused = 0.0;
    bool nz_valid = false;
    double nz_age_s = 0.0;
    double nz_quality = 0.0;
    EstimatorGate nz_gate = EstimatorGate::Uninitialized;
    EstimatorSource nz_source = EstimatorSource::Uninitialized;
    bool nz_kinematic_valid = false;
    double nz_kinematic_quality = 0.0;
    TranslationalGate nz_kinematic_gate = TranslationalGate::Uninitialized;
    bool thrust_valid = false;
    bool pqr_valid = false;
    double pqr_age_s = 0.0;
    double pqr_quality = 0.0;
    BodyRateGate pqr_gate = BodyRateGate::Uninitialized;
    EstimatorSource pqr_source = EstimatorSource::So3Raw;
    bool mass_valid = false;
    double mass_age_s = 0.0;
    EstimatorSource mass_source = EstimatorSource::ConfigurationPlusFuelDeadReckoning;
    ActionFeedbackKind action_feedback_kind = ActionFeedbackKind::CallerUnspecified;
    OptionalFrameIndex action_source_frame_index{};
    OptionalSeconds action_source_t_sec{};
    std::uint32_t action_feedback_delay_frames = 1U;
    OptionalEpoch measurement_reset_epoch{};
    OptionalFrameIndex measurement_frame_index{};
    OptionalSeconds measurement_source_t_sec{};
    SourceTimeKind measurement_source_time_kind = SourceTimeKind::CallerUnspecified;
    OptionalSeconds accepted_sample_t_sec{};
    double sample_dt_s = 0.0;
    TimeAlignment pqr_time_alignment = TimeAlignment::IntervalAverageReportedAtK;
    TimeAlignment pqr_endpoint_time_alignment = TimeAlignment::EndpointKSecondOrder;
    TimeAlignment nz_kinematic_time_alignment = TimeAlignment::IntervalAverageReportedAtK;
};
}
