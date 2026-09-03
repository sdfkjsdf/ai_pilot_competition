#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/obfm/ObfmEntryWindowAdmission.hpp"
#include "LadyLuck/guidance/obfm/ObfmLongitudinalAuthority.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class ObfmEntryLongitudinalApplicationMode : std::uint8_t
{
    None = 0U,
    PrimeCurrentSpeedEcho = 1U,
    PreAdmissionCurrentSpeedEcho = 2U,
    AdmittedReference = 3U,
    PostAdmissionRejectCurrentSpeedEcho = 4U,
    ActiveEpisodeLatchedSpeedHold = 5U
};

enum class ObfmEntryLongitudinalReason : std::uint8_t
{
    Reset = 0U,
    NonOwnerSkip = 1U,
    FeatureDisabled = 2U,
    EntryObservationUnavailable = 3U,
    FrameIdentityUnavailable = 4U,
    CurrentSpeedUnavailable = 5U,
    OfficialMaximumRangeUnavailable = 6U,
    ReferenceEpisodePrimeOrDiscontinuous = 7U,
    TransportedPointUnavailable = 8U,
    TargetCircleSpeedLineageMismatch = 9U,
    TargetSpeedNotPositive = 10U,
    PointSpeedGeometryUnavailable = 11U,
    ReferenceNotAcquirableAlongCurrentPath = 12U,
    GuidanceTimeLineageMismatch = 13U,
    CurrentEnvelopeInvalid = 14U,
    CurrentStateTimeMismatch = 15U,
    ControlFeedbackMissing = 16U,
    ControlFeedbackStale = 17U,
    PreviousFeedbackTimeMismatch = 18U,
    PreviousBackendNotCisV4 = 19U,
    PreviousCisIntegrityNotClean = 20U,
    PreviousAutoGcasInterventionOrFault = 21U,
    PreviousEnergyMeasurementInvalid = 22U,
    PreviousEnergyAuthorityInvalid = 23U,
    PreviousGovernedLoadUnavailable = 24U,
    PreviousMeasuredLoadUnavailable = 25U,
    BackendSpeedRateBoundsUnavailable = 26U,
    ReferenceHorizontalRangeDegenerate = 27U,
    FlightPathGammaLimitUnavailable = 28U,
    CanonicalNzfeasC1Unavailable = 29U,
    TransientLoadBridgeUnavailable = 30U,
    CausalCommandUnavailable = 31U,
    LongitudinalReferenceAdmitted = 32U,
    CommitUnavailable = 33U
};

const char* ObfmEntryLongitudinalReasonLabel(
    ObfmEntryLongitudinalReason reason) noexcept;

struct ObfmEntryPointSpeedReceipt
{
    bool evaluated = false;
    bool admitted = false;
    double raw_speed_mps = 0.0;
    double current_speed_mps = 0.0;
    double target_speed_mps = 0.0;
    double structural_rate_per_s = 0.0;
    Vector3 transported_reference_point_ned_m{};
    Vector3 reference_velocity_ned_mps{};
    Vector3 capture_error_ned_m{};
    Vector3 required_velocity_ned_mps{};
    Vector3 perpendicular_velocity_ned_mps{};
    double perpendicular_speed_mps = 0.0;
};

struct ObfmEntryLongitudinalSnapshot
{
    bool previous_reference_point_valid = false;
    Vector3 previous_reference_point_ned_m{};
    bool previous_own_position_valid = false;
    Vector3 previous_own_position_ned_m{};
    bool previous_own_velocity_valid = false;
    Vector3 previous_own_velocity_ned_mps{};
    bool previous_speed_command_valid = false;
    double previous_speed_command_mps = 0.0;
    bool previous_frame_identity_valid = false;
    ControlFrameIdentity previous_frame_identity{};
    bool previous_time_valid = false;
    double previous_time_s = 0.0;
    // Python's `entry_setup_longitudinal_admitted_once` is overwritten after
    // every published tick; it is not sticky for the complete owner episode.
    bool previous_reference_admitted = false;
};

// Pure current-frame transaction token.  No causal history changes until the
// selected Entry Task publishes and CommitPublished() consumes this token.
struct ObfmEntryLongitudinalPreparation
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    std::uint64_t lifecycle_generation = 0U;
    std::uint64_t base_commit_count = 0U;
    bool same_reference_episode = false;
    bool reference_admitted = false;
    Vector3 current_reference_point_ned_m{};
    Vector3 current_own_position_ned_m{};
    Vector3 current_own_velocity_ned_mps{};
    double current_time_s = 0.0;
};

// Allocation-free command candidate for the future selected Task wiring.
// This is raw guidance: it is not p/q/r, Nz, surface/thrust command, estimator
// measurement, simulator truth, or observed aircraft response.
struct ObfmEntrySetupCommandCandidate
{
    bool valid = false;
    Vector3 aim_point_ned_m{};
    Vector3 aim_point_velocity_ned_mps{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double specific_energy_rate_bias_m2ps3 = 0.0;
    bool path_inversion_allowed = false;
    double capture_range_des_m = 0.0;
};

struct ObfmEntryLongitudinalReceipt
{
    bool evaluated = false;
    bool non_owner_skip = false;
    bool base_fallback_required = false;
    bool producer_ready = false;
    std::uint32_t producer_count = 0U;
    ObfmEntryLongitudinalApplicationMode application_mode =
        ObfmEntryLongitudinalApplicationMode::None;
    ObfmEntryLongitudinalReason reason =
        ObfmEntryLongitudinalReason::Reset;
    ControlFrameIdentity frame_identity{};
    ObfmEntryPointSpeedReceipt point{};
    ObfmLongitudinalAdmissionReceipt admission{};
    ObfmEntrySetupCommandCandidate command{};
};

// Entry-only moving-point speed owner. It transports P*, resolves the current
// full-NED point speed, and owns lifecycle history. Previous energy/load
// receipts are diagnostic; downstream TECS/CIS shapes the admitted v_cmd.
class ObfmEntryLongitudinalReference final
{
public:
    ObfmEntryLongitudinalReference() noexcept;

    void ResetEpisode() noexcept;
    void EnterOwner(
        const ObfmEntrySetupServiceReceipt& service,
        Status& status) noexcept;
    void HaltOwner() noexcept;

    void Prepare(
        bool feature_enabled,
        const ObfmEntrySetupServiceReceipt& service,
        const ObfmEntryWindowObservationReceipt& observation,
        const runtime::TacticalCommandBuildInput& tactical_input,
        ObfmEntryLongitudinalPreparation& preparation,
        ObfmEntryLongitudinalReceipt& output,
        Status& status) noexcept;

    void CommitPublished(
        const ObfmEntryLongitudinalPreparation& preparation,
        bool entry_command_published,
        double published_desired_speed_mps,
        Status& status) noexcept;

    void CopySnapshot(
        ObfmEntryLongitudinalSnapshot& output) const noexcept;
    bool owner_active() const noexcept;

private:
    void ClearHistory() noexcept;
    void AdvanceLifecycle() noexcept;

    bool owner_active_ = false;
    ObfmEntryLongitudinalSnapshot snapshot_{};
    std::uint64_t lifecycle_generation_ = 0U;
    std::uint64_t commit_count_ = 0U;
    ObfmLongitudinalAuthority authority_{};
};

static_assert(
    std::is_trivially_copyable<ObfmEntryPointSpeedReceipt>::value,
    "Entry point-speed receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntryLongitudinalPreparation>::value,
    "Entry longitudinal preparation must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEntryLongitudinalReceipt>::value,
    "Entry longitudinal receipt must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
