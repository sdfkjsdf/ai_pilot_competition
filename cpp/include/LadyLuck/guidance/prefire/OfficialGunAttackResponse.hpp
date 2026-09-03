#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/prefire/GunAttackFormObservation.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace prefire
{

// Frozen add/main@45abc9f6 ON-characterization seed. Production and explicit
// standalone characterization use the same exact-provenance replay seed.
// Reset replays the NumPy PCG64 stream; ClearEpisode preserves its position.
constexpr std::uint64_t OfficialGunAttackResponseProductionSeed = 17U;

struct OfficialGunAttackResponseActivation
{
    bool enabled = false;
    bool exact_provenance = false;
    std::uint64_t seed = 0U;
    // TRACKING Jink is retained only for standalone characterization.  The
    // production policy keeps the already-admitted Root Gun BREAK instead.
    bool tracking_jink_enabled = false;
};

constexpr OfficialGunAttackResponseActivation
    OfficialGunAttackResponseProductionActivation{
        true,
        true,
        OfficialGunAttackResponseProductionSeed,
        false};

enum class OfficialGunAttackResponseBranch : std::uint8_t
{
    BaseBreak = 0U,
    TrackingJink = 1U,
    SnapshotPlaneChange = 2U
};

enum class OfficialGunAttackResponseReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    FeatureDisabled = 2U,
    ActivationContractRejected = 3U,
    BaseBreakContractRejected = 4U,
    ResponseContractUnavailable = 5U,
    ObservationUnavailable = 6U,
    IndeterminateFormKeepsBreak = 7U,
    TrackingGeometryUnavailable = 8U,
    TrackingJinkSelected = 9U,
    SnapshotCapabilityUnavailable = 10U,
    SnapshotGeometryUnavailable = 11U,
    SnapshotReferenceUnavailable = 12U,
    SnapshotPlaneChangeSelected = 13U,
    TrackingFormKeepsBreak = 14U
};

const char* OfficialGunAttackResponseReasonLabel(
    OfficialGunAttackResponseReason reason) noexcept;

// Completed k-1 CIS-v4 force observation. The future shared adapter supplies
// this only after its own freshness check. Missing or malformed values merely
// prevent a new SNAPSHOT target; they never remove the current Root BREAK.
struct OfficialGunForceFeedback
{
    bool available = false;
    bool fresh = false;
    bool cis_v4_backend = false;
    bool snapshot_behavior = false;
    bool direct_force_tracking_requested = false;
    bool observation_only = false;
    bool target_bank_valid = false;
    double target_bank_rad = 0.0;
    bool observed_bank_valid = false;
    double observed_bank_rad = 0.0;
};

struct OfficialGunSnapshotReference
{
    bool valid = false;
    Vector3 target_lift_direction_ned{};
    double target_bank_rad = 0.0;
    double target_load_factor_g = 0.0;
    double horizon_s = 0.0;
    double cone_angle_rad = 0.0;
    double coast_predicted_ata_rad = 0.0;
    double commanded_predicted_ata_rad = 0.0;
    bool boundary_reachable = false;
    bool load_saturated = false;
    std::int32_t side_sign = 0;
};

// This receipt is raw-guidance selection only. Production TRACKING preserves
// the complete Root BREAK. Standalone characterization may still exercise its
// detached Jink candidate. SNAPSHOT preserves the complete BREAK intent and
// publishes its held lift target for the already-existing NED/FCS path. No
// p/q/r, Nz, actuator, thrust, estimator, or aircraft-response authority is
// claimed here.
struct OfficialGunAttackResponseReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool observation_consumed = false;
    GunAttackForm observation_form = GunAttackForm::Indeterminate;
    OfficialGunAttackResponseBranch selected_branch =
        OfficialGunAttackResponseBranch::BaseBreak;
    OfficialGunAttackResponseReason reason =
        OfficialGunAttackResponseReason::NotEvaluated;
    bool base_break_preserved = true;
    bool replacement_available = false;
    bool replace_aim_point = false;
    Vector3 replacement_aim_point_ned_m{};
    std::uint32_t candidate_count = 1U;
    std::uint32_t tracking_candidate_count = 0U;
    std::uint32_t tracking_selected_index = 0U;
    bool tracking_direction_held = false;
    bool tracking_response_visible = false;
    OfficialGunSnapshotReference snapshot_reference{};
    bool snapshot_force_feedback_primed = false;
    bool snapshot_force_target_crossed = false;
    bool snapshot_reversal_admitted_this_tick = false;
    // Diagnostic only. It never suppresses a valid same-frame base BREAK.
    bool declared_ready_contract_contradiction = false;
};

class OfficialGunAttackResponsePolicy final
{
public:
    OfficialGunAttackResponsePolicy() noexcept;

    // Complete-run reset: replay the frozen seed-17 NumPy PCG64 stream.
    void Reset() noexcept;
    // Response-episode reset: clear latches but continue the seeded stream.
    void ClearEpisode() noexcept;

    // output begins as base_break. Every finite missing/no-real/degenerate
    // response condition returns Status::Ok and that one validated command.
    // Only a contradictory base BREAK contract can return a negative Status.
    void Evaluate(
        const DogfightGeometryFrame& frame,
        bool root_official_gun_owner_selected,
        const OfficialGunAttackResponseActivation& activation,
        const GunAttackFormObservation* observation,
        bool response_horizon_available,
        double response_horizon_s,
        bool response_cone_available,
        double response_cone_rad,
        bool load_capability_admitted,
        double load_capability_g,
        const OfficialGunForceFeedback* previous_feedback,
        const ControlIntent& base_break,
        OfficialGunAttackResponseReceipt& receipt,
        ControlIntent& output,
        Status& status) noexcept;

private:
    struct Pcg128
    {
        std::uint64_t high = 0U;
        std::uint64_t low = 0U;
    };

    std::uint32_t NextBoundedIndex(std::uint32_t upper_bound) noexcept;
    std::uint32_t NextUint32() noexcept;
    std::uint64_t NextRaw64() noexcept;
    void ClearResponseState() noexcept;
    void ClearSnapshotState() noexcept;

    Pcg128 rng_state_{};
    bool rng_uint32_cached_ = false;
    std::uint32_t rng_cached_uint32_ = 0U;
    bool held_jink_direction_valid_ = false;
    Vector3 held_jink_direction_ned_{};
    bool snapshot_target_lift_valid_ = false;
    Vector3 snapshot_target_lift_ned_{};
    bool snapshot_reference_valid_ = false;
    OfficialGunSnapshotReference snapshot_reference_{};
    bool snapshot_direction_valid_ = false;
    std::int32_t snapshot_direction_sign_ = 0;
    bool snapshot_capture_resolution_valid_ = false;
    double snapshot_capture_resolution_rad_ = 0.0;
    bool snapshot_force_feedback_primed_ = false;
    bool snapshot_previous_force_valid_ = false;
    double snapshot_previous_force_target_bank_rad_ = 0.0;
    double snapshot_previous_force_bank_rad_ = 0.0;
    bool snapshot_force_target_crossed_ = false;
    GunAttackForm previous_form_ = GunAttackForm::Indeterminate;
};

static_assert(
    std::is_trivially_copyable<OfficialGunForceFeedback>::value,
    "official-Gun force feedback must remain allocation-free");
static_assert(
    std::is_trivially_copyable<OfficialGunAttackResponseReceipt>::value,
    "official-Gun response receipt must remain allocation-free");

} // namespace prefire
} // namespace guidance
} // namespace LadyLuck
