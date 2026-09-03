#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

struct G10OptionalBool
{
    bool has_value = false;
    bool value = false;
};

struct G10OptionalDouble
{
    bool has_value = false;
    double value = 0.0;
};

// Typed admission receipt supplied by the separate G10 bridge. This binder
// consumes only its public admitted verdict, exactly like Python.
struct G10SecondUseBridgeAdmissionReceipt
{
    bool valid = false;
    bool admitted = false;
    std::int32_t adversary_post_reversal_turn_sign = 0;
};

enum class G10BarrelSpeedDumpStatus : std::uint8_t
{
    Unresolved = 0U,
    NoForwardVectorReductionRequired = 1U,
    NonBarrelEffectNotRefuted = 2U,
    BarrelEffectRequired = 3U
};

enum class G10BarrelSpeedDumpBasis : std::uint8_t
{
    None = 0U,
    StrictEffectDominance = 1U,
    ManualCellNoNonBarrelAlternative = 2U
};

struct G10BarrelSpeedDumpDecisionReceipt
{
    bool valid = false;
    G10BarrelSpeedDumpStatus status =
        G10BarrelSpeedDumpStatus::Unresolved;
    bool barrel_manual_cell_applicable = false;
    std::uint32_t eligible_non_barrel_candidate_count = 0U;
    double maximum_non_barrel_upper_reduction_mps = 0.0;
    G10OptionalBool substantial_speed_dump_required{};
    G10BarrelSpeedDumpBasis positive_basis =
        G10BarrelSpeedDumpBasis::None;
};

enum class G10SecondUseSelectionReason : std::uint8_t
{
    BridgeReceiptNotAdmitted = 0U,
    AspectUnresolved = 1U,
    SpeedDumpBindingRefused = 2U,
    ManualSelectionNotBarrelFamily = 3U,
    SecondUseSelectionBound = 4U,
    ContractRejected = 5U
};

const char* G10SecondUseSelectionReasonLabel(
    G10SecondUseSelectionReason reason) noexcept;

struct G10SecondUseSelectionBinding
{
    bool bound = false;
    G10SecondUseSelectionReason reason =
        G10SecondUseSelectionReason::BridgeReceiptNotAdmitted;
    bool selection_available = false;
    bool barrel_roll_attack_family = false;
    G10OptionalDouble aspect_deg{};
    G10OptionalBool excess_closure_present{};
};

// Exact production call path: the caller supplies none of the optional
// low/medium-aspect manual variables, as in g10_post_root_consumer.py.
void BindG10SecondUseSelection(
    const G10SecondUseBridgeAdmissionReceipt& bridge,
    const DogfightGeometryFrame& frame,
    const G10BarrelSpeedDumpDecisionReceipt& speed_dump,
    G10SecondUseSelectionBinding& output,
    Status& status) noexcept;

enum class G10PursuitOvershootForecastStatus : std::uint8_t
{
    Forced = 0U,
    NotForced = 1U,
    Unresolved = 2U
};

enum class G10PursuitOvershootForecastReason : std::uint8_t
{
    Other = 0U,
    OwnAlreadyAhead = 1U
};

struct G10PursuitOvershootForecastReceipt
{
    G10PursuitOvershootForecastStatus status =
        G10PursuitOvershootForecastStatus::Unresolved;
    G10PursuitOvershootForecastReason reason =
        G10PursuitOvershootForecastReason::Other;
};

struct G10FlightPathGammaLimitReceipt
{
    bool available = false;
    double value_rad = 0.0;
    // Python treats a nonfinite value with an absent/blank source as ordinary
    // unavailability, but a present source as a schema fault.
    bool source_nonempty = false;
};

enum class G10BarrelLoadSelectionStatus : std::uint8_t
{
    Unresolved = 0U,
    Selected = 1U,
    NoAdmittedPairWithinControlLimits = 2U
};

struct G10BarrelLoadSelectionReceipt
{
    bool valid = false;
    G10BarrelLoadSelectionStatus status =
        G10BarrelLoadSelectionStatus::Unresolved;
    double selected_load_magnitude_g = 0.0;
    double selected_roll_rate_magnitude_radps = 0.0;
    double selected_constructive_reduction_lower_mps = 0.0;
    double selected_effect_upper_mps = 0.0;
    double selected_causal_effect_time_s = 0.0;
    double diagnostic_structural_requirement_upper_mps = 0.0;
    double effect_horizon_s = 0.0;
    double effective_load_limit_g = 0.0;
    double effective_roll_rate_limit_radps = 0.0;
    std::uint32_t candidate_count = 0U;
};

struct G10SecondUseSupply
{
    bool valid = false;
    G10OptionalDouble overshoot_realized_t_sec{};
    G10BarrelSpeedDumpDecisionReceipt speed_dump_decision{};
    G10BarrelLoadSelectionReceipt load_selection{};
    double roll_rate_limit_radps = 0.0;
    Vector3 station_velocity_mps{};
    double flight_path_gamma_limit_rad = 0.0;
    bool moving_body_3_9_crossed = false;
    bool descending_lag_command_applied_before_state = false;
    // Current typed prevention handoff.  has_value=false preserves Python
    // None and is not equivalent to a resolved false receipt.
    G10OptionalBool prevention_failure_handoff_required{};
    // Compatibility diagnostic only.  New callers consume the typed field.
    bool prevention_egress_handoff_available = false;
};

// Causal inputs owned by the production caller, not inferred by this frozen
// supply provider.  The Lag flag is the completed k-1 publication fact read
// before observing the current aircraft state.
struct G10SecondUseCausalSupplyInput
{
    G10OptionalBool current_prevention_failure_handoff_required{};
    bool completed_k_minus_1_descending_lag_publication = false;
};

enum class G10SecondUseSupplyReason : std::uint8_t
{
    NotUpdated = 0U,
    ForecastUnavailable = 1U,
    FlightPathGammaUnavailable = 2U,
    SupplyPublished = 3U,
    ContractRejected = 4U
};

const char* G10SecondUseSupplyReasonLabel(
    G10SecondUseSupplyReason reason) noexcept;

const char* G10ApprovedMinimumSourceSha256() noexcept;
const char* G10ApprovedMinimumComparisonId() noexcept;
const char* G10SecondUseSupplyLineageId() noexcept;
const char* G10SecondUseSupplySourceEpoch() noexcept;

void BuildG10TrackedApprovedReceipts(
    G10BarrelSpeedDumpDecisionReceipt& speed_dump,
    G10BarrelLoadSelectionReceipt& load_selection,
    double& roll_rate_limit_radps,
    Status& status) noexcept;

class G10SecondUseSupplyProvider final
{
public:
    G10SecondUseSupplyProvider() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        const G10PursuitOvershootForecastReceipt* forecast,
        const G10FlightPathGammaLimitReceipt* gamma_limit,
        G10SecondUseSupply& output,
        Status& status) noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        const G10PursuitOvershootForecastReceipt* forecast,
        const G10FlightPathGammaLimitReceipt* gamma_limit,
        const G10SecondUseCausalSupplyInput& causal_input,
        G10SecondUseSupply& output,
        Status& status) noexcept;

    G10SecondUseSupplyReason LastReason() const noexcept;
    G10OptionalDouble OvershootRealizedT() const noexcept;

private:
    G10BarrelSpeedDumpDecisionReceipt speed_dump_{};
    G10BarrelLoadSelectionReceipt load_selection_{};
    double roll_rate_limit_radps_ = 0.0;
    G10OptionalDouble overshoot_realized_t_sec_{};
    G10SecondUseSupplyReason last_reason_ =
        G10SecondUseSupplyReason::NotUpdated;
};

static_assert(
    std::is_trivially_copyable<G10SecondUseSelectionBinding>::value,
    "G10 selection binding must remain allocation-free");
static_assert(
    std::is_trivially_copyable<G10SecondUseSupply>::value,
    "G10 supply must remain allocation-free");
static_assert(
    std::is_trivially_copyable<G10SecondUseCausalSupplyInput>::value,
    "G10 causal supply input must remain allocation-free");
static_assert(
    std::is_trivially_copyable<G10SecondUseSupplyProvider>::value,
    "G10 supply provider must remain allocation-free");

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
