#pragma once

#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"
#include "LadyLuck/guidance/obfm/ObfmLongitudinalAuthority.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

// Ordered fail-closed outcomes of the d90 ordinary-OBFM provider.  Entries
// after PreviousEnergyAuthorityUnrepresented document the next exact seam
// work; none may be replaced by a guessed speed, load, gain, or rate bound.
enum class ObfmLongitudinalProviderStatus : std::uint8_t
{
    ReferenceAdmitted = 0U,
    ReferenceEpisodePrimeOrDiscontinuous = 1U,
    TargetSpeedNotPositive = 2U,
    TargetHorizontalSpeedNotPositive = 3U,
    GuidanceIntervalNotPositive = 4U,
    OfficialMaximumRangeNotPositive = 5U,
    PointSpeedGeometryNonfinite = 6U,
    ReferenceNotAcquirableAlongCurrentPath = 7U,
    GuidanceTimeLineageMismatch = 8U,
    CurrentEnvelopeInvalid = 9U,
    CurrentStateTimeMismatch = 10U,
    ControlFeedbackMissing = 11U,
    ControlFeedbackStale = 12U,
    PreviousFeedbackTimeMismatch = 13U,
    PreviousBackendNotCisV4 = 14U,
    PreviousCisIntegrityNotClean = 15U,
    PreviousAutoGcasInterventionOrFault = 16U,
    PreviousEnergyMeasurementInvalid = 17U,
    PreviousEnergyAuthorityUnrepresented = 18U,
    PreviousEnergyAuthorityInvalid = 19U,
    PreviousGovernedLoadUnavailable = 20U,
    PreviousMeasuredLoadUnavailable = 21U,
    BackendSpeedRateBoundsUnrepresented = 22U,
    ReferenceHorizontalRangeDegenerate = 23U,
    FlightPathGammaLimitUnrepresented = 24U,
    ExactCanonicalNzfeasC1Unavailable = 25U,
    ExactTransientLoadBridgeUnavailable = 26U,
    ExactCausalCommandAdmissionUnavailable = 27U,
    FrameOrPreparationInvalid = 28U,
    OwnSpeedNotPositive = 29U
};

struct ObfmPointSpeedReceipt
{
    bool evaluated = false;
    bool admitted = false;
    double raw_speed_mps = 0.0;
    double current_speed_mps = 0.0;
    double target_speed_mps = 0.0;
    double structural_rate_per_s = 0.0;
    Vector3 reference_velocity_ned_mps{};
    Vector3 capture_error_ned_m{};
    Vector3 required_velocity_ned_mps{};
    Vector3 perpendicular_velocity_ned_mps{};
    double perpendicular_speed_mps = 0.0;
};

struct ObfmLongitudinalProviderReceipt
{
    ObfmLongitudinalProviderStatus status =
        ObfmLongitudinalProviderStatus::FrameOrPreparationInvalid;
    ObfmLagLongitudinalReference reference{};
    ObfmPointSpeedReceipt point{};
    ObfmLongitudinalAdmissionReceipt admission{};
};

enum class ObfmBumplessSpeedStatus : std::uint8_t
{
    AdmittedWithoutPrior = 0U,
    AdmittedUnchanged = 1U,
    AdmittedRateLimited = 2U,
    RawReferenceUnavailable = 3U,
    PriorPublishedReferenceInvalid = 4U,
    RateAuthorityUnavailable = 5U,
    AdmittedPriorHoldWithoutRateAuthority = 6U,
    AdmittedRawWithoutRateAuthority = 7U
};

// Shared station/phase transition shaper. The caller supplies only an
// actually published prior v_cmd; a preview or failed publication must not be
// used as causal state. No new gain, dwell, or speed threshold is introduced.
struct ObfmBumplessSpeedInput
{
    ControlFrameIdentity frame_identity{};
    bool raw_reference_admitted = false;
    double raw_desired_speed_mps = 0.0;
    double raw_desired_speed_rate_mps2 = 0.0;
    bool prior_published_speed_valid = false;
    double prior_published_speed_mps = 0.0;
    double dt_s = 0.0;
    control::tecs_cis::TecsCisLongitudinalAuthorityConfiguration
        rate_authority{};
};

struct ObfmBumplessSpeedReceipt
{
    bool evaluated = false;
    bool admitted = false;
    bool rate_limited = false;
    ControlFrameIdentity frame_identity{};
    double raw_desired_speed_mps = 0.0;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    ObfmBumplessSpeedStatus status =
        ObfmBumplessSpeedStatus::RawReferenceUnavailable;
};

void ShapeObfmBumplessSpeedReference(
    const ObfmBumplessSpeedInput& input,
    ObfmBumplessSpeedReceipt& output,
    Status& status) noexcept;

// Stateless provider; all episode history remains in ObfmLagGuidance and is
// committed only after the selected Task publishes.  TacticalCommandBuildInput
// currently lacks the following exact d90 data, so the original production
// overload stops at a typed unavailable outcome before emitting speed/rate.
// The explicit-authority overload accepts these values only through a receipt
// produced by ObfmLongitudinalAuthority; it never substitutes defaults:
//
// * age-1 EnergyRateAuthority: Kp, Ki, Kr, post-update integral, measured
//   specific-energy rate, exact thrust-derived rate-command min/max, mass, and
//   the validity receipt built from inverse speed/drag/projection/thrust bounds;
// * current CIS-v4 desired-speed rate bounds and gamma-command limit with their
//   source receipts;
// * authenticated all-connected gear-up NZFEAS intervals, interpolated shared
//   and N-channel E-M masks, published instantaneous/sustained corners, the
//   validation conflict brackets, and exact C1/transient/causal admission.
//
// `source_authoritative=true` below means this class preserves d90 ordering and
// explicit refusal.  It does not claim that the original overload's absent
// physical authorities are available.
class ObfmLongitudinalReferenceProvider final
{
public:
    void Evaluate(
        const ObfmLagGuidancePreparation& preparation,
        const runtime::TacticalCommandBuildInput& tactical_input,
        ObfmLongitudinalProviderReceipt& output,
        Status& status) const noexcept;

    // Exact-authority overload for the forthcoming completed-CIS/current-
    // safety seam.  The original overload remains the production-safe typed
    // unavailable observation until that seam supplies this receipt.
    void Evaluate(
        const ObfmLagGuidancePreparation& preparation,
        const runtime::TacticalCommandBuildInput& tactical_input,
        const ObfmLongitudinalAuthorityReceipt& authority,
        ObfmLongitudinalProviderReceipt& output,
        Status& status) const noexcept;
};

static_assert(
    std::is_trivially_copyable<ObfmPointSpeedReceipt>::value,
    "OBFM point-speed receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmLongitudinalProviderReceipt>::value,
    "OBFM longitudinal receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmBumplessSpeedReceipt>::value,
    "OBFM bumpless speed receipt must stay allocation-free.");

} // namespace LadyLuck
