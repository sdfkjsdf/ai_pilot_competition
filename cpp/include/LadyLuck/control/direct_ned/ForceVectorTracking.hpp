#pragma once

#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{

enum class ForceDirectionGate : std::uint8_t
{
    Init = 0,
    FrameGap = 1,
    ExcludedForceInvalid = 2,
    VelocityDirectionUndefined = 3,
    ObservationInvalid = 4,
    Update = 5
};

enum class ForceDirectionSource : std::uint8_t
{
    CausalForceDirectionUnavailable = 0,
    NedVelocityDifferenceMinusExplicitForceComponent = 1
};

struct VelocityNormalForceDirectionObservation
{
    Vector3 total_specific_force_ned_g{};
    Vector3 excluded_specific_force_ned_g{};
    Vector3 observed_specific_force_ned_g{};
    Vector3 velocity_normal_specific_force_ned_g{};
    Vector3 direction_ned{};
    double magnitude_g = 0.0;
    double velocity_parallel_g = 0.0;
    double lateral_g = 0.0;
    double support_g = 0.0;
    double bank_rad = 0.0;
};

struct ForceDirectionBiasCompensation
{
    double target_force_bank_rad = 0.0;
    double observed_kinematic_bank_rad = 0.0;
    double observed_force_bank_rad = 0.0;
    double force_to_kinematic_bias_rad = 0.0;
    double force_direction_error_rad = 0.0;
    double compensated_kinematic_reference_rad = 0.0;
};

struct ForceDirectionInput
{
    Vector3 velocity_ned_mps{};
    double dt_s = 0.0;
    Vector3 excluded_specific_force_ned_mps2{};
    Vector3 positive_lateral_direction_ned{};
    Vector3 positive_support_direction_ned{};
    double target_force_bank_rad = 0.0;
    double observed_kinematic_bank_rad = 0.0;
    bool excluded_force_valid = false;
};

struct ForceDirectionOutput
{
    bool valid = false;
    ForceDirectionGate gate = ForceDirectionGate::Init;
    ForceDirectionSource source =
        ForceDirectionSource::NedVelocityDifferenceMinusExplicitForceComponent;
    double sample_dt_s = 0.0;
    bool observation_valid = false;
    VelocityNormalForceDirectionObservation observation{};
    bool compensation_valid = false;
    ForceDirectionBiasCompensation compensation{};
};

struct CausalForceDirectionSnapshot
{
    bool has_previous_velocity = false;
    Vector3 previous_velocity_ned_mps{};
};

// Causal observation of the realized velocity-normal force direction.  The
// class owns only the preceding NED velocity sample and never emits a control
// command.
class CausalForceDirection final
{
public:
    CausalForceDirection() noexcept = default;

    void Reset() noexcept;
    void CopySnapshot(CausalForceDirectionSnapshot& output) const noexcept;
    void Update(
        const ForceDirectionInput& input,
        ForceDirectionOutput& output,
        Status& status) noexcept;

private:
    bool has_previous_velocity_ = false;
    Vector3 previous_velocity_ned_mps_{};
};

enum class ForceVectorTrackingGate : std::uint8_t
{
    Active = 0,
    VelocityDirectionUndefined = 1,
    LateralAxisUndefined = 2,
    SupportAxisUndefined = 3,
    TargetForceDirectionUndefined = 4,
    TargetOutsidePositiveLiftTurnDomain = 5,
    DirectionInit = 6,
    DirectionFrameGap = 7,
    DirectionExcludedForceInvalid = 8,
    DirectionVelocityDirectionUndefined = 9,
    DirectionObservationInvalid = 10,
    ObservedForceOutsideTargetTurnDomain = 11,
    CompensatedReferenceOutsideTargetTurnDomain = 12
};

enum class ForceMagnitudeTrackingGate : std::uint8_t
{
    NotRequested = 0,
    DirectionObservationUnavailable = 1,
    DirectionDomainRejected = 2,
    PreviousTargetUnavailable = 3,
    NonpositiveDeficit = 4,
    PreviousErrorUnavailable = 5,
    DeficitClosing = 6,
    Active = 7,
    NonpositiveCompensatedMagnitude = 8
};

struct ForceTrackingOptionalScalar
{
    bool has_value = false;
    double value = 0.0;
};

struct ForceVectorTrackingInput
{
    Vector3 requested_acceleration_ned_mps2{};
    Vector3 velocity_ned_mps{};
    double dt_s = 0.0;
    double observed_kinematic_bank_rad = 0.0;
    Vector3 excluded_specific_force_ned_mps2{};
    bool excluded_force_valid = false;
    bool magnitude_tracking_enabled = false;
};

// A valid output can still carry tracking_applied=false.  That is the normal
// finite/unavailable path and preserves the requested acceleration unchanged.
struct ForceVectorTrackingOutput
{
    bool valid = false;
    Vector3 requested_acceleration_ned_mps2{};
    Vector3 applied_acceleration_ned_mps2{};
    bool tracking_applied = false;
    ForceVectorTrackingGate gate =
        ForceVectorTrackingGate::VelocityDirectionUndefined;
    ForceDirectionSource observation_source =
        ForceDirectionSource::CausalForceDirectionUnavailable;
    ForceTrackingOptionalScalar target_force_bank_rad{};
    ForceTrackingOptionalScalar observed_force_bank_rad{};
    ForceTrackingOptionalScalar corrected_kinematic_bank_rad{};
    ForceTrackingOptionalScalar target_force_magnitude_g{};
    ForceTrackingOptionalScalar observed_force_magnitude_g{};
    ForceTrackingOptionalScalar force_direction_error_rad{};
    bool magnitude_tracking_applied = false;
    ForceMagnitudeTrackingGate magnitude_tracking_gate =
        ForceMagnitudeTrackingGate::NotRequested;
    ForceTrackingOptionalScalar previous_target_force_magnitude_g{};
    ForceTrackingOptionalScalar compensated_force_magnitude_reference_g{};
    ForceTrackingOptionalScalar force_magnitude_error_g{};
};

struct ForceVectorTrackingSnapshot
{
    CausalForceDirectionSnapshot direction{};
    bool has_previous_target_force_magnitude = false;
    double previous_target_force_magnitude_g = 0.0;
    bool has_previous_force_magnitude_error = false;
    double previous_force_magnitude_error_g = 0.0;
};

// Parameter-free, age-1 direction and optional magnitude compensation for an
// inertial acceleration request.  It owns observation history, not tactical,
// body-rate, surface, thrust, or command-authorization authority.
class ForceVectorTracking final
{
public:
    ForceVectorTracking() noexcept = default;

    void Reset() noexcept;
    void CopySnapshot(ForceVectorTrackingSnapshot& output) const noexcept;
    void Step(
        const ForceVectorTrackingInput& input,
        ForceVectorTrackingOutput& output,
        Status& status) noexcept;

private:
    CausalForceDirection direction_{};
    bool has_previous_target_force_magnitude_ = false;
    double previous_target_force_magnitude_g_ = 0.0;
    bool has_previous_force_magnitude_error_ = false;
    double previous_force_magnitude_error_g_ = 0.0;
};

} // namespace direct_ned
} // namespace control
} // namespace LadyLuck
