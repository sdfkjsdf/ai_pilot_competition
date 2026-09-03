#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/guidance/habfm/HabfmActiveControlCore.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace habfm
{

enum class HabfmH09StorageReason : std::uint8_t
{
    Admitted = 0U,
    NotTwoCircle = 1U,
    CheckpointCueUnavailable = 2U,
    CheckpointCueNotNeutral = 3U,
    SustainedOperatingPointUnavailable = 4U,
    OwnSpeedUnavailable = 5U,
    SpeedBelowSustainedReference = 6U,
    OfficialEnemyGunDamageInvalid = 7U,
    InsideOfficialEnemyGunThreat = 8U,
    ControlFeedbackNotFresh = 9U,
    ControlFeedbackUnavailable = 10U,
    ControlBackendNotCisV4 = 11U,
    AutoGcasOwnsOrInvalidatesSample = 12U,
    PreviousCommandNotTwoCircle = 13U,
    H09TotalLoadEvidenceUnavailable = 14U,
    PreviousTurnDemandNotTrackable = 15U,
    PreviousLoadHeadroomNotPositive = 16U,
    MeasuredLoadUnavailable = 17U,
    EnergyRateMeasurementInvalid = 18U,
    TecsEnergyAuthorityUnavailable = 19U,
    MeasuredSpecificExcessPowerNotPositive = 20U,
    PositiveTecsReferenceNotAuthorized = 21U,
    AdmittedStorageRateNotPositive = 22U,
    ClimbRateNotKinematicallyRepresentable = 23U,
    ClimbPathAngleOutsideCurrentLimit = 24U,
    CurrentProjectionUnavailable = 25U,
    CurrentEnergyMeasurementNotPositive = 26U,
    CurrentThrustPreviewUnavailable = 27U,
    CombinedVectorUnavailable = 28U,
    CombinedLoadExceedsCurrentLimit = 29U,
    FeedbackNotAgeOne = 30U,
    CurrentAutoGcasOwnsSample = 31U,
    SeniorAvoidPassOwnsSample = 32U,
    CurrentPhysicalAuthorityUnavailable = 33U,
    CurrentMassUnavailable = 34U,
    TurnSideUnavailable = 35U,
    SameCurrentEvidenceChanged = 36U,
    FinalIntentUnavailable = 37U
};

// Command-neutral current-frame proposal. It requests no body rate, load,
// surface, thrust, or aircraft response. The synchronous TECS projection seam
// owns current energy measurement and final admission; age-1 telemetry is
// diagnostic only. Non-admission leaves legacy TWO_CIRCLE available.
struct HabfmH09AltitudeStorageAdmission
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    HabfmH09StorageReason reason =
        HabfmH09StorageReason::SustainedOperatingPointUnavailable;
    double own_speed_mps = 0.0;
    double sustained_speed_mps = 0.0;
    double sustained_turn_rate_radps = 0.0;
    double measured_energy_rate_m2ps3 = 0.0;
    double authority_upper_reference_m2ps3 = 0.0;
    double admitted_energy_rate_m2ps3 = 0.0;
    double climb_rate_mps = 0.0;
    double climb_gamma_cmd_rad = 0.0;
    double speed_reference_rate_mps2 = 0.0;
};

void EvaluateHabfmH09AltitudeStorage(
    const runtime::TacticalCommandBuildInput& tactical,
    const HabfmFrameEvidence& evidence,
    const HabfmActiveControlOutput& active,
    HabfmH09AltitudeStorageAdmission& output,
    Status& status) noexcept;

struct HabfmH09ResidualClimbInput
{
    ControlFrameIdentity frame_identity{};
    Vector3 velocity_ned_mps{};
    Vector3 body_forward_ned{};
    std::int32_t side_sign = 0;
    double sustained_speed_mps = 0.0;
    double sustained_course_rate_radps = 0.0;
    double speed_reference_rate_mps2 = 0.0;
    double residual_energy_rate_m2ps3 = 0.0;
    double tecs_thrust_command_n = 0.0;
    double mass_kg = 0.0;
    double instantaneous_load_limit_g = 0.0;
    double flight_path_rate_command_radps = 0.0;
};

// Exact DirectLoadVector route quantity.  Route5 subtracts gravity once from
// this total-inertial-minus-thrust acceleration, yielding the aerodynamic
// specific force.  TECS therefore keeps longitudinal thrust ownership.
struct HabfmH09ResidualClimbAllocation
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    HabfmH09StorageReason reason =
        HabfmH09StorageReason::CombinedVectorUnavailable;
    double climb_rate_mps = 0.0;
    double climb_gamma_cmd_rad = 0.0;
    double course_acceleration_mps2 = 0.0;
    Vector3 required_inertial_acceleration_ned_mps2{};
    Vector3 thrust_acceleration_ned_mps2{};
    Vector3 aerodynamic_specific_force_ned_mps2{};
    Vector3 adapter_inertial_acceleration_ned_mps2{};
    double aerodynamic_load_factor_g = 0.0;
    double instantaneous_load_limit_g = 0.0;
};

void AllocateHabfmH09ResidualClimb(
    const HabfmH09ResidualClimbInput& input,
    HabfmH09ResidualClimbAllocation& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<HabfmH09AltitudeStorageAdmission>::value,
    "H09-B admission must remain allocation-free");
static_assert(
    std::is_trivially_copyable<HabfmH09ResidualClimbAllocation>::value,
    "H09-B allocation must remain allocation-free");

} // namespace habfm
} // namespace guidance
} // namespace LadyLuck
