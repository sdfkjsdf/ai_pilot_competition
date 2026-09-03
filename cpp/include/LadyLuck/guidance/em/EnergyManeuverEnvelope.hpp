#pragma once

#include <cstddef>
#include <cstdint>

namespace LadyLuck
{
namespace guidance
{
namespace em
{
// Value and receipt are separate so a rejected table lookup cannot silently
// become a usable zero load or speed reference.
struct EmValue
{
    bool has_value = false;
    double value = 0.0;
};

enum class StrictEmStatus : std::uint8_t
{
    LookupTrusted = 0U,
    TableIdentityMismatch = 1U,
    TableSchemaOrProvenanceInvalid = 2U,
    StateOrMassInvalid = 3U,
    ConfigurationUnverifiedOrMismatch = 4U,
    OperatingPointOutsideAxes = 5U,
    SoundnessCellNotComplete = 6U,
    PublishedLookupFailed = 7U,
    PublishedLookupNonfinite = 8U,
    PublishedLookupSemanticsInvalid = 9U
};

const char* StrictEmStatusText(StrictEmStatus status) noexcept;

struct EmTableAuthority
{
    bool table_identity_valid = false;
    bool schema_valid = false;
    bool provenance_valid = false;
    const char* table_source = nullptr;
    const char* table_sha256 = nullptr;
    const char* f16_md5 = nullptr;
    const char* baseline_configuration = nullptr;
    const char* publish_policy_v52_sha256 = nullptr;
};

struct StrictEmInput
{
    double speed_mps = 0.0;
    double altitude_m = 0.0;
    double mass_kg = 0.0;
    bool mass_valid = false;
    const char* mass_source = nullptr;
    double nz_g = 0.0;
    bool nz_valid = false;
    const char* nz_source = nullptr;
    double gear_pos_norm = 0.0;
    double speedbrake_pos_norm = 0.0;
    bool speedbrake_valid = false;
    const char* configuration_source = nullptr;
};

struct EmCellEvidence
{
    bool available = false;
    std::size_t speed_index = 0U;
    std::size_t altitude_index = 0U;
    double speed_lower_mps = 0.0;
    double speed_upper_mps = 0.0;
    double altitude_lower_m = 0.0;
    double altitude_upper_m = 0.0;
    std::uint8_t mask_corners[4] = {0U, 0U, 0U, 0U};
    bool trusted = false;
    // v5.2 publishes an additive N-channel trust mask.  The shared-mask
    // receipt above keeps its pre-v5.2 Ps-inclusive meaning.
    bool trusted_n = false;
};

enum class EmCellTrustStatus : std::uint8_t
{
    LookupCompleted = 0U,
    TableUnavailable = 1U,
    OperatingPointNotFinite = 2U,
    OperatingPointOutsideAxes = 3U
};

// A binary soundness mask is a cell contract, not an interpolated physical
// field.  A completed lookup therefore preserves the exact selected cell and
// admits a channel only when all four of that channel's corners are zero.
struct EmCellTrustReceipt
{
    EmCellTrustStatus status = EmCellTrustStatus::TableUnavailable;
    EmCellEvidence cell{};

    bool lookup_valid() const noexcept
    {
        return status == EmCellTrustStatus::LookupCompleted
            && cell.available;
    }
};

// These are quasi-steady table capabilities/evidence.  They are neither raw
// body-rate/Nz commands nor proof that the FCS, actuators, or aircraft tracked
// the corresponding load or energy state.
struct EnergyManeuverCapability
{
    StrictEmStatus status = StrictEmStatus::TableIdentityMismatch;
    bool lookup_trusted = false;
    bool load_comparison_valid = false;
    bool predictive_promotion_valid = false;
    bool table_identity_valid = false;
    bool provenance_valid = false;
    bool configuration_valid = false;
    // This may be true while lookup_trusted is false.  In that case only the
    // N values are admitted; Ps remains absent and the shared refusal status
    // remains SoundnessCellNotComplete.
    bool n_channel_trusted = false;
    EmCellEvidence cell{};
    EmValue ps_min_1g_idle_mps{};
    EmValue ps_max_1g_ab_mps{};
    EmValue n_inst_g{};
    EmValue n_sus_g{};

    bool admitted() const noexcept
    {
        return lookup_trusted && status == StrictEmStatus::LookupTrusted;
    }
};

enum class PublishedSustainedNStatus : std::uint8_t
{
    LookupCompleted = 0U,
    TableUnavailable = 1U,
    StateOrMassInvalid = 2U,
    PublishedLookupNonfinite = 3U
};

const char* PublishedSustainedNStatusText(
    PublishedSustainedNStatus status) noexcept;

// The value and interpolated-mask diagnostic exactly preserve Python
// EMEnvelope.N_pub(..., sustained=True).  Production trust applies the stricter
// all-four-corner contract; callers must consume value and trust atomically.
struct PublishedSustainedNLookup
{
    PublishedSustainedNStatus status =
        PublishedSustainedNStatus::TableUnavailable;
    EmValue load_factor_g{};
    EmValue interpolated_mask_n{};
    bool trusted = false;

    bool lookup_valid() const noexcept
    {
        return status == PublishedSustainedNStatus::LookupCompleted
            && load_factor_g.has_value
            && interpolated_mask_n.has_value;
    }

    bool admitted() const noexcept
    {
        return lookup_valid() && trusted;
    }
};

enum class CharacterizedRawNStatus : std::uint8_t
{
    LookupCompleted = 0U,
    TableUnavailable = 1U,
    OperatingPointNotFinite = 2U,
    LookupNonfinite = 3U
};

// Allocation-free numeric counterpart of EMEnvelope.N_characterized_raw for a
// reference-mass scalar query.  The pre-margin value is preserved even outside
// the table axes, while production `trusted` additionally requires the requested
// operating point to lie inside both axes and all four N-mask cell corners to
// be zero.  The interpolated mask remains a numeric diagnostic only.
// This is opponent maximum-turn characterization evidence, not ownship
// guidance authority or a certified physical upper bound.
struct CharacterizedRawNLookup
{
    CharacterizedRawNStatus status =
        CharacterizedRawNStatus::TableUnavailable;
    EmValue load_factor_g{};
    EmValue interpolated_mask_n{};
    bool in_table_domain = false;
    bool trusted = false;

    bool lookup_valid() const noexcept
    {
        return status == CharacterizedRawNStatus::LookupCompleted
            && load_factor_g.has_value
            && interpolated_mask_n.has_value;
    }

    bool admitted() const noexcept
    {
        return lookup_valid() && trusted;
    }
};

enum class PursuitOvershootEmStatus : std::uint8_t
{
    QueryCompleted = 0U,
    TableUnavailable = 1U,
    OperatingPointNotFinite = 2U,
    PublishedLookupNonfinite = 3U
};

// Composite, allocation-free publication query used by the d90
// PursuitOvershootForecaster.  Each field preserves the corresponding
// EMEnvelope lookup independently: the sustained corner remains usable for
// the branch payload even when one point in the arrest band is untrusted.
struct PursuitOvershootEmQuery
{
    PursuitOvershootEmStatus status =
        PursuitOvershootEmStatus::TableUnavailable;
    EmValue stall_1g_mps{};
    EmValue sustained_corner_mps{};
    EmValue optimistic_arrest_mps2{};
    bool arrest_band_trusted = false;

    bool query_valid() const noexcept
    {
        return status == PursuitOvershootEmStatus::QueryCompleted;
    }
};

class StrictEnergyManeuverEnvelope final
{
public:
    // Authority validates the generated source identity, compiled-table
    // checksum, axes/schema, F-16 model provenance, and clean configuration
    // prefix without consulting a runtime filesystem.
    static EmTableAuthority Authority() noexcept;

    // Strict admission additionally requires finite estimator state, admitted
    // positive mass, verified clean configuration, in-axis operation, and all
    // four corners of the selected bilinear cell to have soundness_mask == 0.
    EnergyManeuverCapability Observe(const StrictEmInput& input) const noexcept;

    // Mass/configuration-independent trust receipt shared by doctrine
    // consumers.  This never interpolates a binary mask and never extrapolates
    // beyond the closed published axes.
    void ObserveCellTrust(
        double speed_mps,
        double altitude_m,
        EmCellTrustReceipt& output) const noexcept;

    // Allocation-free numeric companion for the d90 HABFM sustained operating
    // point.  It preserves numpy.clip, searchsorted(side="left")-1, the
    // source's left-associative bilinear expression, MASS/reference-mass
    // division, and the interpolated mask as a diagnostic.  Admission uses the
    // same exact all-four-corner N trust receipt as every production consumer.
    void ObservePublishedSustainedN(
        double speed_mps,
        double altitude_m,
        double mass_kg,
        PublishedSustainedNLookup& output) const noexcept;

    // Python production constructs ManualTurnCircleCapabilityProvider without
    // a mass argument, so this surface intentionally evaluates only the
    // reference-mass characterized table used by that provider.
    void ObserveCharacterizedRawN(
        double speed_mps,
        double altitude_m,
        bool sustained,
        CharacterizedRawNLookup& output) const noexcept;

    // Numeric composite of EMEnvelope.v_stall(h, 1), V_corner_sus(h), and the
    // supremum of N_pub(..., sustained=False) plus Ps_pub_min deceleration at
    // the current speed, the stall floor, and every strict interior published
    // speed knot. `mass_available == false` is Python mass=None (reference
    // mass).  No canonical table data is copied into the consumer.
    void ObservePursuitOvershootComposite(
        double speed_mps,
        double altitude_m,
        bool mass_available,
        double mass_kg,
        PursuitOvershootEmQuery& output) const noexcept;
};

enum class CornerIntervalStatus : std::uint8_t
{
    Admitted = 0U,
    TableUnavailable = 1U,
    AltitudeNotFinite = 2U,
    AltitudeOutsidePublishedAxis = 3U,
    PublishedCornerNotFinitePositive = 4U
};

const char* CornerIntervalStatusText(CornerIntervalStatus status) noexcept;

struct MergeCornerInterval
{
    CornerIntervalStatus status = CornerIntervalStatus::TableUnavailable;
    EmValue lower_mps{};
    EmValue upper_mps{};

    bool admitted() const noexcept
    {
        return status == CornerIntervalStatus::Admitted
            && lower_mps.has_value
            && upper_mps.has_value;
    }
};

enum class EnergyResolutionStatus : std::uint8_t
{
    Admitted = 0U,
    TableUnavailable = 1U,
    OperatingPointNotFinite = 2U,
    AltitudeOutsidePublishedAxis = 3U,
    SpeedOutsidePublishedAxis = 4U,
    PublishedSpeedAxisDegenerate = 5U,
    SpeedGridGapUnresolved = 6U,
    PublishedGridGapNotFinitePositive = 7U,
    ResolutionNotFinitePositive = 8U
};

// Failure text and admitted provenance are generated from the canonical
// Python EnergyResolution authority.
const char* EnergyResolutionStatusText(
    EnergyResolutionStatus status) noexcept;

struct EnergyResolution
{
    EnergyResolutionStatus status = EnergyResolutionStatus::TableUnavailable;
    EmValue resolution_m{};

    bool admitted() const noexcept
    {
        return status == EnergyResolutionStatus::Admitted
            && resolution_m.has_value;
    }
};

class MergeIntentCornerProvider final
{
public:
    // Deliberately different from Observe(): the active Python merge provider
    // admits each 1-D published corner curve from finite, in-axis altitude and
    // a finite positive result.  It does not apply the 2-D cell soundness mask,
    // current mass/configuration gates, or strict diagnostic provenance gate.
    MergeCornerInterval InstantaneousInterval(double altitude_m) const noexcept;
    MergeCornerInterval SustainedInterval(double altitude_m) const noexcept;

    // E_s = h + V^2/(2g) has local V-axis resolution V*dV/g.  The exact
    // Python bracket search and neighbouring-gap order are preserved.
    EnergyResolution EnergyResolutionAt(
        double altitude_m,
        double speed_mps) const noexcept;
};
}
}
}
