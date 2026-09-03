#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/guidance/prefire/GunAttackFormObservation.hpp"
#include "LadyLuck/guidance/prefire/RootPrefireThreatObservation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

struct SnapshotPlaneChangeState
{
    bool commanded_transverse_direction_valid = false;
    Vector3 commanded_transverse_direction_ned{};
};

enum class SnapshotPlaneChangeReason : std::uint8_t
{
    ShadowDisabledDefaultOff = 0U,
    SolutionTimeInvalid = 1U,
    SpeedNotPositive = 2U,
    LiftAxisDegenerate = 3U,
    CarriedDirectionDegenerate = 4U,
    RotatedDirectionDegenerate = 5U,
    ReachablePlaneChangeStep = 6U,
    SnapshotPlaneChangeContractRejected = 7U,
    TransverseProjectionUnavailable = 8U
};

const char* SnapshotPlaneChangeReasonLabel(
    SnapshotPlaneChangeReason reason) noexcept;

struct SnapshotPlaneChangeReceipt
{
    bool engaged = false;
    SnapshotPlaneChangeReason reason =
        SnapshotPlaneChangeReason::ShadowDisabledDefaultOff;
    bool commanded_transverse_direction_valid = false;
    Vector3 commanded_transverse_direction_ned{};
    PrefireOptionalDouble applied_rotation_rad{};
    PrefireOptionalDouble threat_derived_rate_radps{};
    PrefireOptionalDouble rotation_bound_rad{};
    PrefireOptionalDouble baseline_transverse_magnitude_mps2{};
    PrefireOptionalDouble admitted_transverse_magnitude_mps2{};
    bool a_cmd_total_valid = false;
    Vector3 a_cmd_total_ned_mps2{};
};

enum class PrefireResponseSelected : std::uint8_t
{
    BreakPassthrough = 0U,
    SnapshotPlaneChange = 1U
};

const char* PrefireResponseSelectedLabel(
    PrefireResponseSelected selected) noexcept;

// Exact TacticalCommand.behavior_label written by the selected Python Task.
const char* PrefireSnapshotPlaneChangeBehaviorLabel() noexcept;

enum class PrefireResponseSelectionReason : std::uint8_t
{
    NoIncomingCommand = 0U,
    NonSnapshotFormKeepsBreak = 1U,
    CandidatePhaseUnresolved = 2U,
    ThreatPredictionUnresolved = 3U,
    BaselineDegenerateOwnSpeedMustBePositive = 4U,
    LiftSeedDegenerate = 5U,
    AwaySignDegenerate = 6U,
    PlaneChangeBlocked = 7U,
    SnapshotFormTakesPlaneChange = 8U,
    PrefireResponseSelectionContractRejected = 9U
};

// The standalone module does not allocate or claim a shared writer/behavior
// numeric ID. This typed overlay tells the visible selected Task exactly which
// Python TacticalCommand fields to replace and clear.
struct PrefireSnapshotCommandOverlay
{
    bool valid = false;
    Vector3 direct_load_vector_acceleration_ned_mps2{};
    bool clear_direct_p_cmd = true;
    bool clear_direct_nz_cmd = true;
    bool clear_direct_beta_cmd = true;
    bool clear_direct_bank_cmd = true;
    bool clear_direct_turn_rate_cmd = true;
    bool clear_direct_acceleration_ned = true;
    bool clear_direct_acceleration_roll_rate_reference = true;
    bool direct_acceleration_tracking_enabled = false;
    bool direct_acceleration_tracking_observation_only = false;
    bool direct_acceleration_magnitude_tracking_enabled = false;
    bool direct_acceleration_loaded_roll_enabled = false;
    bool direct_acceleration_load_component_compensation_enabled = false;
    bool direct_acceleration_yaw_coordination_enabled = false;
    bool direct_acceleration_roll_priority_yaw_enabled = false;
};

constexpr std::size_t PrefireResponseReasonLabelCapacity = 96U;

struct PrefireResponseSelectionReceipt
{
    bool engaged = false;
    PrefireResponseSelected selected =
        PrefireResponseSelected::BreakPassthrough;
    PrefireResponseSelectionReason reason =
        PrefireResponseSelectionReason::NoIncomingCommand;
    std::array<char, PrefireResponseReasonLabelCapacity> exact_reason_label{};
    bool attack_form_valid = false;
    PrefireGunAttackForm attack_form =
        PrefireGunAttackForm::Indeterminate;
    PrefireOptionalDouble away_sign{};
    PrefireOptionalDouble baseline_transverse_magnitude_mps2{};
    bool plane_change_receipt_valid = false;
    SnapshotPlaneChangeReceipt plane_change{};
    PrefireSnapshotCommandOverlay command_overlay{};
};

void ResetSnapshotPlaneChangeState(
    SnapshotPlaneChangeState& state) noexcept;

void PathHoldTransverseMagnitudeMps2(
    const Vector3& own_velocity_ned_mps,
    double& output,
    Status& status) noexcept;

void PrefireAwaySign(
    const Vector3& commanded_or_lift_direction_ned,
    const Vector3& own_position_ned_m,
    const Vector3& attacker_position_ned_m,
    const Vector3& own_velocity_ned_mps,
    PrefireOptionalDouble& output,
    Status& status) noexcept;

// Exact d90 selector. `incoming_gun_command_available` preserves Python None;
// all longitudinal/TECS fields remain with the caller's upstream intent.
void SelectPrefireResponse(
    const SnapshotPlaneChangeState& state,
    bool incoming_gun_command_available,
    const PrefireGunAttackFormObservation* attack_form_observation,
    const RootPrefireThreatShadowReceipt* shadow_receipt,
    const Vector3& own_rpy_rad,
    const Vector3& own_position_ned_m,
    const Vector3& own_velocity_ned_mps,
    const Vector3& attacker_position_ned_m,
    double max_roll_rate_radps,
    double dt_s,
    SnapshotPlaneChangeState& next_state,
    PrefireResponseSelectionReceipt& output,
    Status& status) noexcept;

// Apply only the exact dataclass_replace field family. The visible post-root
// Task supplies the shared behavior and writer IDs after those IDs are frozen.
void ApplyPrefireSnapshotCommandOverlay(
    const ControlIntent& upstream,
    const PrefireSnapshotCommandOverlay& overlay,
    DoctrineBehaviorId snapshot_behavior_id,
    std::uint32_t selected_writer_id,
    ControlIntent& output,
    Status& status) noexcept;

static_assert(
    std::is_trivially_copyable<SnapshotPlaneChangeState>::value,
    "snapshot state must remain allocation-free");
static_assert(
    std::is_trivially_copyable<PrefireResponseSelectionReceipt>::value,
    "prefire response receipt must remain allocation-free");

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
