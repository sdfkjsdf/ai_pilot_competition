#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

// A native 24-node Mach grid can contain at most twelve disconnected
// feasible runs.  This is a representation bound derived from the canonical
// table width, not a tactical limit.
constexpr std::size_t ObfmMaximumFeasibleSpeedIntervals = 12U;

enum class ObfmEnergyRateAuthorityStatus : std::uint8_t
{
    AuthorityAvailable = 0U,
    BackendUnavailable = 1U,
    EnergyReceiptUnavailable = 2U,
    ControllerNotContinuousTotalEnergy = 3U,
    RateMeasurementUnavailable = 4U,
    ControllerConfigurationMissing = 5U,
    FrameIdentityInvalid = 6U,
    InputNonfinite = 7U,
    EnergyErrorGainNegative = 8U,
    EnergyIntegralGainNegative = 9U,
    EnergyRateFeedbackGainNegative = 10U,
    MinimumSpeedNotPositive = 11U,
    InverseSpeedNotPositive = 12U,
    MassNotPositive = 13U,
    DragEstimateNegative = 14U,
    ThrustProjectionOutOfRange = 15U,
    ThrustBoundsReversed = 16U,
    DerivedAuthorityNonfinite = 17U,
    DerivedAuthorityReversed = 18U
};

// Exact scalar inputs used by d90 completed_cis_v4_energy_rate_authority().
// The caller owns extraction from one completed CIS-v4 sample; no controller
// defaults or thrust limits are substituted here.
struct ObfmEnergyRateAuthorityInput
{
    ControlFrameIdentity source_frame_identity{};
    bool cis_v4_backend = false;
    bool energy_receipt_available = false;
    bool continuous_total_energy_controller = false;
    bool rate_measurement_valid = false;
    bool controller_configuration_available = false;
    double energy_error_gain_per_s = 0.0;
    double energy_integral_gain_per_s2 = 0.0;
    double energy_rate_feedback_gain = 0.0;
    double total_energy_error_m2ps2 = 0.0;
    double energy_integral_error_m2ps = 0.0;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    double speed_mps = 0.0;
    double minimum_speed_mps = 0.0;
    double mass_kg = 0.0;
    double drag_estimate_n = 0.0;
    double thrust_velocity_projection = 0.0;
    double thrust_min_n = 0.0;
    double thrust_max_n = 0.0;
};

struct ObfmEnergyRateAuthorityObservation
{
    bool evaluated = false;
    bool valid = false;
    ObfmEnergyRateAuthorityStatus status =
        ObfmEnergyRateAuthorityStatus::BackendUnavailable;
    ControlFrameIdentity source_frame_identity{};
    double energy_error_gain_per_s = 0.0;
    double energy_integral_gain_per_s2 = 0.0;
    double energy_rate_feedback_gain = 0.0;
    double total_energy_error_m2ps2 = 0.0;
    double energy_integral_error_m2ps = 0.0;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    double speed_for_inverse_mps = 0.0;
    double mass_kg = 0.0;
    double drag_estimate_n = 0.0;
    double thrust_velocity_projection = 0.0;
    double thrust_min_n = 0.0;
    double thrust_max_n = 0.0;
    double reference_denominator = 0.0;
    double controller_offset_m2ps3 = 0.0;
    double rate_command_min_m2ps3 = 0.0;
    double rate_command_max_m2ps3 = 0.0;
    double reference_min_m2ps3 = 0.0;
    double reference_max_m2ps3 = 0.0;
};

// Current-frame controller/safety values are retained separately from the
// age-1 energy observation so the provider can preserve Python's check order.
struct ObfmLongitudinalAuthorityInput
{
    ControlFrameIdentity current_frame_identity{};
    ObfmEnergyRateAuthorityInput previous_energy{};
    bool speed_rate_bounds_available = false;
    bool speed_rate_bounds_source_valid = false;
    double speed_rate_lower_mps2 = 0.0;
    double speed_rate_upper_mps2 = 0.0;
    double flight_path_gamma_limit_rad = 0.0;
};

struct ObfmLongitudinalAuthorityReceipt
{
    bool evaluated = false;
    ControlFrameIdentity current_frame_identity{};
    ObfmEnergyRateAuthorityObservation previous_energy{};
    bool speed_rate_bounds_available = false;
    bool speed_rate_bounds_source_valid = false;
    double speed_rate_lower_mps2 = 0.0;
    double speed_rate_upper_mps2 = 0.0;
    double flight_path_gamma_limit_rad = 0.0;
};

struct ObfmFeasibleSpeedInterval
{
    bool valid = false;
    double lower_mps = 0.0;
    double upper_mps = 0.0;
    bool lower_boundary_resolved = false;
    bool upper_boundary_resolved = false;
};

enum class ObfmStallFeasibleSetStatus : std::uint8_t
{
    IntervalsAvailable = 0U,
    NonpositiveMassOrRequiredLoad = 1U,
    AltitudeOutsideTrimTableDomain = 2U,
    RequiredLoadUnreachable = 3U,
    SourceNumericsInvalid = 4U
};

struct ObfmStallFeasibleSetReceipt
{
    bool admitted = false;
    ObfmStallFeasibleSetStatus status =
        ObfmStallFeasibleSetStatus::SourceNumericsInvalid;
    double altitude_m = 0.0;
    double mass_kg = 0.0;
    double required_load_g = 0.0;
    double mach_domain_lower = 0.0;
    double mach_domain_upper = 0.0;
    std::array<ObfmFeasibleSpeedInterval,
        ObfmMaximumFeasibleSpeedIntervals> intervals{};
    std::uint8_t interval_count = 0U;
};

enum class ObfmEmCornerStatus : std::uint8_t
{
    CornerAvailable = 0U,
    AltitudeOutsideEmTableDomain = 1U,
    ExternalConflictBracketed = 2U,
    SoundnessMaskUntrusted = 3U,
    SourceNumericsInvalid = 4U
};

struct ObfmEmCornerReceipt
{
    bool admitted = false;
    ObfmEmCornerStatus status = ObfmEmCornerStatus::SourceNumericsInvalid;
    double altitude_m = 0.0;
    double instantaneous_mps = 0.0;
    double sustained_mps = 0.0;
    bool instantaneous_lookup_trusted = false;
    bool sustained_lookup_trusted = false;
    bool external_conflict_bracketed = false;
};

enum class ObfmC1SpeedSourceStatus : std::uint8_t
{
    SourceAvailable = 0U,
    EmCornerNotAdmitted = 1U,
    PublishedCornerLoadUntrusted = 2U,
    PublishedCornerLoadNotPositive = 3U,
    SpeedOrOfficialRangeContractInvalid = 4U,
    StallSetNotAdmitted = 5U,
    EmCornerOutsideRequiredLoadSet = 6U,
    EmCornerBelowRequiredLoadFloor = 7U,
    SourceNumericsInvalid = 8U
};

struct ObfmC1SpeedSourceReceipt
{
    bool admitted = false;
    ObfmC1SpeedSourceStatus status =
        ObfmC1SpeedSourceStatus::SourceNumericsInvalid;
    double required_load_g = 0.0;
    ObfmStallFeasibleSetReceipt stall_set{};
    ObfmEmCornerReceipt em_corner{};
    ObfmFeasibleSpeedInterval feasible_interval{};
    double acquisition_speed_admitted_mps = 0.0;
    double range_hold_speed_raw_mps = 0.0;
    double range_hold_speed_admitted_mps = 0.0;
    double blend_weight = 0.0;
    double speed_command_mps = 0.0;
    double range_hold_gain_per_s = 0.0;
    double range_only_speed_rate_diagnostic_mps2 = 0.0;
};

enum class ObfmTransientLoadBridgeStatus : std::uint8_t
{
    BridgeAvailable = 0U,
    MassOrSpeedNotPositive = 1U,
    PreviousPositiveLoadRegimeNotEstablished = 2U,
    TerminalRequiredLoadNotPositive = 3U,
    CurrentCapabilityNotPositive = 4U,
    BridgeRequiredLoadNotPositive = 5U,
    BridgeStallSetNotAdmitted = 6U,
    CurrentAndPreviousNotInOneConnectedInterval = 7U,
    ConnectedIntervalNotUnique = 8U,
    SourceNumericsInvalid = 9U
};

struct ObfmTransientLoadBridgeReceipt
{
    bool admitted = false;
    ObfmTransientLoadBridgeStatus status =
        ObfmTransientLoadBridgeStatus::SourceNumericsInvalid;
    double current_capability_load_g = 0.0;
    double previous_governed_load_g = 0.0;
    double previous_measured_load_g = 0.0;
    double terminal_required_load_g = 0.0;
    double bridge_required_load_g = 0.0;
    ObfmStallFeasibleSetReceipt stall_set{};
    ObfmFeasibleSpeedInterval feasible_interval{};
};

enum class ObfmCausalSpeedAdmissionStatus : std::uint8_t
{
    WithinCausalIntersection = 0U,
    ProjectedToCausalIntersection = 1U,
    SpeedOrDtNotPositive = 2U,
    AuthorityInvalid = 3U,
    SpeedRateBoundsReversed = 4U,
    EnergyQuadraticNotConvex = 5U,
    EnergyAuthorityRootUnavailable = 6U,
    EnergyCommandNotMonotone = 7U,
    CausalSpeedIntersectionEmpty = 8U,
    EnergyAuthorityNumericalIdentityFailed = 9U,
    SourceNumericsInvalid = 10U
};

enum ObfmCausalBoundSource : std::uint8_t
{
    ObfmCausalBoundSourceNone = 0U,
    ObfmCausalBoundSourceNzfeas = 1U << 0U,
    ObfmCausalBoundSourceBackendVdot = 1U << 1U,
    ObfmCausalBoundSourceTecsThrust = 1U << 2U
};

struct ObfmCausalSpeedAdmissionReceipt
{
    bool admitted = false;
    ObfmCausalSpeedAdmissionStatus status =
        ObfmCausalSpeedAdmissionStatus::SourceNumericsInvalid;
    double raw_speed_mps = 0.0;
    double previous_speed_command_mps = 0.0;
    double admitted_speed_mps = 0.0;
    double raw_speed_rate_mps2 = 0.0;
    double admitted_speed_rate_mps2 = 0.0;
    double feasible_speed_lower_mps = 0.0;
    double feasible_speed_upper_mps = 0.0;
    double backend_reachable_lower_mps = 0.0;
    double backend_reachable_upper_mps = 0.0;
    double energy_authority_lower_mps = 0.0;
    double energy_authority_upper_mps = 0.0;
    double intersection_lower_mps = 0.0;
    double intersection_upper_mps = 0.0;
    std::uint8_t lower_active_sources = ObfmCausalBoundSourceNone;
    std::uint8_t upper_active_sources = ObfmCausalBoundSourceNone;
    bool lower_limited = false;
    bool upper_limited = false;
    double specific_energy_rate_reference_m2ps3 = 0.0;
    double specific_total_energy_error_m2ps2 = 0.0;
    double specific_energy_rate_command_m2ps3 = 0.0;
};

enum class ObfmLongitudinalAdmissionStatus : std::uint8_t
{
    ReferenceAvailable = 0U,
    C1SourceNotAdmitted = 1U,
    TerminalSpeedOutsideQualifiedInterval = 2U,
    TransientLoadBridgeNotAdmitted = 3U,
    CausalCommandNotAdmitted = 4U,
    InputNonfinite = 5U
};

struct ObfmLongitudinalAdmissionInput
{
    double altitude_m = 0.0;
    double mass_kg = 0.0;
    double current_range_m = 0.0;
    double range_rate_mps = 0.0;
    double target_speed_mps = 0.0;
    double acquisition_speed_mps = 0.0;
    double official_min_range_m = 0.0;
    double official_max_range_m = 0.0;
    double current_speed_mps = 0.0;
    double previous_speed_command_mps = 0.0;
    double previous_governed_load_g = 0.0;
    double previous_measured_load_g = 0.0;
    double altitude_rate_cmd_mps = 0.0;
    double dt_s = 0.0;
    double speed_rate_lower_mps2 = 0.0;
    double speed_rate_upper_mps2 = 0.0;
    ObfmEnergyRateAuthorityObservation energy_authority{};
};

struct ObfmLongitudinalAdmissionReceipt
{
    bool admitted = false;
    ObfmLongitudinalAdmissionStatus status =
        ObfmLongitudinalAdmissionStatus::InputNonfinite;
    ObfmC1SpeedSourceReceipt c1_source{};
    ObfmTransientLoadBridgeReceipt transient_bridge{};
    ObfmCausalSpeedAdmissionReceipt command_admission{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
};

// Pure, allocation-free producer and resolver.  Observe() constructs the
// exact next-tick TECS authority.  Admit() applies canonical NZFEAS, E-M C1,
// transient-load, backend-Vdot, and thrust-authority intersections in d90
// operation order.  Neither method publishes a guidance or FCS command.
class ObfmLongitudinalAuthority final
{
public:
    void Observe(
        const ObfmLongitudinalAuthorityInput& input,
        ObfmLongitudinalAuthorityReceipt& output,
        Status& status) const noexcept;

    void Admit(
        const ObfmLongitudinalAdmissionInput& input,
        ObfmLongitudinalAdmissionReceipt& output,
        Status& status) const noexcept;
};

static_assert(
    std::is_trivially_copyable<ObfmLongitudinalAuthorityInput>::value,
    "OBFM authority input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmLongitudinalAuthorityReceipt>::value,
    "OBFM authority receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmLongitudinalAdmissionInput>::value,
    "OBFM admission input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmLongitudinalAdmissionReceipt>::value,
    "OBFM admission receipt must stay allocation-free.");

} // namespace LadyLuck
