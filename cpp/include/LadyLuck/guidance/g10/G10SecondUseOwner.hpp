#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/g10/G10SecondUseSelectionSupply.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

// These labels preserve the four Python behavior_label identities until the
// shared ControlIntent registry assigns behavior IDs 39-42 and writer 19.
enum class G10SecondUseCommandLabel : std::uint8_t
{
    Upstream = 0U,
    PitchUp = 1U,
    PositiveLoadedWinding = 2U,
    DescendingLag = 3U
};

struct G10SecondUseCommand
{
    ControlIntent intent{};
    G10SecondUseCommandLabel label = G10SecondUseCommandLabel::Upstream;
};

enum class G10SecondUseOwnerPhase : std::uint8_t
{
    Idle = 0U,
    BarrelPitchUp = 1U,
    BarrelLoadedRoll = 2U,
    DescendingLagReacquire = 3U,
    Complete = 4U,
    G16EHandOff = 5U
};

enum class G10SecondUseReferenceRole : std::uint8_t
{
    None = 0U,
    BarrelRollAttack = 1U,
    DescendingLag = 2U,
    G16EHandOff = 3U
};

enum class G10SecondUseOwnerReason : std::uint8_t
{
    GateDisabled = 0U,
    UpstreamCommandUnavailable = 1U,
    SupplyFault = 2U,
    BridgeNotAdmitted = 3U,
    SelectionNotBound = 4U,
    RollDirectionUnresolved = 5U,
    SampleIdentityUnavailable = 6U,
    MachineSampleSequenceBroken = 7U,
    EnergyAdvantageLost = 8U,
    NoseToTailConversionAchieved = 9U,
    WindingCommandWithheld = 10U,
    CommittedMachineReleased = 11U,
    SecondUseCommandPublished = 12U,
    ContractRejected = 13U
};

enum class G10FrozenPairRuntimeReason : std::uint8_t
{
    PairNotSelected = 0U,
    RuntimeLoadLimitUnavailable = 1U,
    RuntimeRollRateLimitUnavailable = 2U,
    LoadExceedsRuntimeLimit = 3U,
    RollRateExceedsRuntimeLimit = 4U,
    Admitted = 5U,
    ContractRejected = 6U
};

struct G10FrozenPairRuntimeAdmission
{
    bool valid = false;
    bool admitted = false;
    G10FrozenPairRuntimeReason reason =
        G10FrozenPairRuntimeReason::PairNotSelected;
    G10OptionalDouble load_magnitude_g{};
    G10OptionalDouble roll_rate_magnitude_radps{};
    G10OptionalDouble runtime_load_limit_g{};
    G10OptionalDouble runtime_roll_rate_limit_radps{};
    G10OptionalDouble frozen_load_limit_g{};
    G10OptionalDouble frozen_roll_rate_limit_radps{};
    bool clamped = false;
};

struct G10SecondUseRuntimeAuthority
{
    G10OptionalDouble nz_feasible_g{};
    bool nz_feasible_source_nonempty = false;
};

struct G10SecondUseOwnerInput
{
    bool gate_enabled = false;
    bool root_command_available = false;
    G10SecondUseCommand root_command{};
    G10SecondUseBridgeAdmissionReceipt bridge{};
    G10SecondUseSupply supply{};
    G10SecondUseRuntimeAuthority runtime{};
    double sample_dt_s = 0.0;
};

struct G10VelocityBankRollProgress
{
    bool present = false;
    bool valid = false;
    std::int32_t direction_sign = 0;
    Vector3 previous_velocity_hat_ned{};
    Vector3 previous_bank_direction_ned{};
    double previous_observation_time_s = 0.0;
    double progress_rad = 0.0;
    bool nominal_270_observed = false;
};

struct G10SecondUseTargetPathHistory
{
    bool previous_target_velocity_valid = false;
    Vector3 previous_target_velocity_ned_mps{};
    bool previous_time_valid = false;
    double previous_time_s = 0.0;
    bool previous_rear_attack_point_valid = false;
    Vector3 previous_rear_attack_point_ned_m{};
    bool previous_rear_attack_sample_index_valid = false;
    std::uint64_t previous_rear_attack_sample_index = 0U;
    bool previous_own_position_valid = false;
    Vector3 previous_own_position_ned_m{};
    bool previous_speed_command_valid = false;
    double previous_speed_command_mps = 0.0;
};

struct G10SecondUseOwnerSnapshot
{
    bool engaged = false;
    std::uint64_t expected_frame_index = 0U;
    bool last_observation_time_valid = false;
    double last_observation_time_s = 0.0;
    G10SecondUseOwnerPhase phase = G10SecondUseOwnerPhase::Idle;
    std::int32_t committed_roll_sign = 0;
    G10SecondUseBridgeAdmissionReceipt committed_admission{};
    G10SecondUseSelectionBinding committed_binding{};
    G10BarrelLoadSelectionReceipt committed_load_selection{};
    double committed_roll_rate_limit_radps = 0.0;
    bool committed_nz_source_nonempty = false;
    G10FrozenPairRuntimeAdmission committed_runtime_pair{};
    bool previous_commanded_bank_direction_valid = false;
    Vector3 previous_commanded_bank_direction_ned{};
    G10VelocityBankRollProgress roll_progress{};
    bool roll_complete = false;
    bool crossing_rearmed = false;
    bool crossing_seen = false;
    bool descending_lag_applied = false;
    G10SecondUseTargetPathHistory target_path{};
    std::uint64_t pair_readmission_count = 0U;
    std::uint64_t target_path_commit_count = 0U;
};

enum class G10SecondUseOperation : std::uint8_t
{
    None = 0U,
    BindSelection = 1U,
    EntryPairReadmission = 2U,
    EntryStateCommitted = 3U,
    SequenceValidated = 4U,
    CommittedPairReadmission = 5U,
    CrossingStateMutated = 6U,
    EnergyReleaseChecked = 7U,
    RollProgressObserved = 8U,
    LifecycleAdvanced = 9U,
    PitchUpMaterialized = 10U,
    WindingMaterialized = 11U,
    DescendingLagMaterialized = 12U,
    TargetPathCommitted = 13U,
    CommandWithheld = 14U,
    OwnerReleased = 15U,
    CommandPublished = 16U
};

constexpr std::size_t G10SecondUseOperationCapacity = 20U;

struct G10SecondUseOperationTrace
{
    std::array<G10SecondUseOperation, G10SecondUseOperationCapacity> values{};
    std::size_t count = 0U;
};

struct G10SecondUseOwnerReceipt
{
    bool valid = false;
    bool evaluated = false;
    bool engaged = false;
    bool commitment_retained = false;
    bool entered_this_tick = false;
    bool released_this_tick = false;
    bool reference_changed = false;
    G10SecondUseOwnerReason reason =
        G10SecondUseOwnerReason::ContractRejected;
    G10SecondUseSelectionBinding binding{};
    G10FrozenPairRuntimeAdmission runtime_pair{};
    G10SecondUseOwnerPhase phase = G10SecondUseOwnerPhase::Idle;
    G10SecondUseReferenceRole reference_role =
        G10SecondUseReferenceRole::None;
    G10SecondUseCommand command{};
    G10SecondUseOperationTrace operation_trace{};
};

// Provider-call windows are derived from the owner's pre-update phase and the
// same actual-aircraft bank observation used by Update.  They keep raw attitude
// geometry out of the production caller.
struct G10SecondUseProviderWindowReceipt
{
    bool valid = false;
    bool actual_bank_observation_available = false;
    bool barrel_loaded_roll_active = false;
    bool rear_preview_required = false;
};

// Exact standalone port of d90 G10BarrelSecondUsePostRootOwner. The owner
// mutates only while this selected operation runs. Raw guidance requests are
// not p/q/r/Nz, surface, thrust, estimator, or aircraft-response receipts.
class G10SecondUseOwner final
{
public:
    G10SecondUseOwner() noexcept = default;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        const G10SecondUseOwnerInput& input,
        G10SecondUseOwnerReceipt& output,
        Status& status) noexcept;
    void CopySnapshot(G10SecondUseOwnerSnapshot& output) const noexcept;

private:
    G10SecondUseOwnerSnapshot snapshot_{};
};

static_assert(
    std::is_trivially_copyable<G10SecondUseOwnerInput>::value,
    "G10 owner input must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G10SecondUseOwnerSnapshot>::value,
    "G10 owner snapshot must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G10SecondUseOwnerReceipt>::value,
    "G10 owner receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G10SecondUseProviderWindowReceipt>::value,
    "G10 provider-window receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<G10SecondUseOwner>::value,
    "G10 owner preflight copy must remain allocation-free.");

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
