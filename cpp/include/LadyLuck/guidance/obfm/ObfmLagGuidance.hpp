#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

// Local command-writer identity for the ordinary OBFM LAG leaf. Selection
// remains owned by the visible doctrine BehaviorTree; this module cannot
// preempt a committed owner or any higher-priority OBFM sibling.
constexpr std::uint32_t ControlIntentWriterObfmLag = 5U;

// Result of the visible, production-enabled longitudinal-reference provider.
// The caller evaluates that provider from Prepare() before BuildCandidate();
// this leaf only applies its admitted result in the Python
// command-construction order.
struct ObfmLagLongitudinalReference
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool source_authoritative = false;
    bool same_reference_episode = false;
    bool admitted = false;
    IntentOptionalValue<double> desired_speed_mps{};
    IntentOptionalValue<double> desired_speed_rate_mps2{};
};

// Result of the station_hold_lag_observation Service attached to the selected
// LAG leaf. An evaluated Service may legitimately publish no speed outside its
// official range band.
struct ObfmLagStationHoldReference
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    IntentOptionalValue<double> desired_speed_mps{};
    // Optional because the raw station observer has no derivative. The
    // RuntimeContext fills this only after the shared station/phase bumpless
    // shaper has selected the final writer-5/G16 speed reference.
    IntentOptionalValue<double> desired_speed_rate_mps2{};
};

// The visible BT chooses exactly one longitudinal speed authority before the
// single LAG writer builds its candidate.  No blending or last-write-wins
// replacement is permitted inside ObfmLagGuidance.
enum class ObfmLagSpeedAuthority : std::uint8_t
{
    Unavailable = 0U,
    PhaseLongitudinal = 1U,
    StationHold = 2U
};

struct ObfmLagGuidanceInput
{
    // True only after the visible OBFM selector has rejected every preceding
    // sibling and selected lag_station_owner. This class is not a selector.
    bool ordinary_fallback_selected = false;
    // The visible selected Task supplies the behavior/writer identity; no
    // hidden selector lives in this provider.
    DoctrineBehaviorId behavior_id = DoctrineBehaviorId::Lag;
    std::uint32_t writer_id = ControlIntentWriterObfmLag;
    ObfmLagSpeedAuthority speed_authority =
        ObfmLagSpeedAuthority::Unavailable;
    ObfmLagLongitudinalReference longitudinal{};
    ObfmLagStationHoldReference station_hold{};
};

// Causal state required by the external longitudinal-reference Service. A
// A LAG task halt does not clear this history in d90; Reset is the episode
// boundary and clears every field.
struct ObfmLagGuidanceSnapshot
{
    bool previous_adversary_velocity_valid = false;
    Vector3 previous_adversary_velocity_ned_mps{};
    bool previous_time_valid = false;
    double previous_time_s = 0.0;
    bool previous_reference_point_valid = false;
    Vector3 previous_reference_point_ned_m{};
    bool previous_own_position_valid = false;
    Vector3 previous_own_position_ned_m{};
    bool previous_speed_command_valid = false;
    double previous_speed_command_mps = 0.0;
    OptionalFrameIndex previous_frame_index{};
    bool longitudinal_admitted_once = false;
};

// Pure, token-bound output of the kinematic preparation step.  The external
// longitudinal source consumes these exact d90 arguments; no command or causal
// history is published by Prepare().
struct ObfmLagGuidancePreparation
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    std::uint64_t lifecycle_generation = 0U;
    std::uint64_t base_commit_count = 0U;
    OptionalFrameIndex safety_frame_index{};
    Vector3 current_reference_point_ned_m{};
    Vector3 previous_reference_point_ned_m{};
    bool transported_reference_point_valid = false;
    Vector3 transported_reference_point_ned_m{};
    bool same_reference_episode = false;
    Vector3 current_own_position_ned_m{};
    Vector3 current_own_velocity_ned_mps{};
    Vector3 current_target_velocity_ned_mps{};
    double dt_s = 0.0;
    double current_time_s = 0.0;
    double previous_time_s = 0.0;
    double current_range_m = 0.0;
    double range_rate_mps = 0.0;
    double official_min_range_m = 0.0;
    double official_max_range_m = 0.0;
    double previous_speed_command_mps = 0.0;
};

// Candidate next history produced before doctrine publication.  It has no
// authority until the selected Task publishes its ControlIntent and then calls
// CommitPublished() in the same serialized transaction.
struct ObfmLagGuidanceCommit
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    std::uint64_t lifecycle_generation = 0U;
    std::uint64_t base_commit_count = 0U;
    ObfmLagGuidanceSnapshot next_snapshot{};
};

// EMPLOY clears the LAG/FOLLOW longitudinal episode before building its pure
// pursuit command, but Python still commits the current adversary velocity and
// time after that selected command is published.
struct ObfmEmployHistoryCommit
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    std::uint64_t lifecycle_generation = 0U;
    std::uint64_t base_commit_count = 0U;
    Vector3 adversary_velocity_ned_mps{};
    double time_s = 0.0;
};

// Exact pre-overlay guidance writer for the d90 production-effective ordinary
// OBFM LAG leaf. It requests a concentric point behind the target path and a
// longitudinal speed; Route-5/TECS/CIS and post-root owners remain downstream.
class ObfmLagGuidance final
{
public:
    ObfmLagGuidance() noexcept;

    void Reset() noexcept;
    // EMPLOY clears only the concentric longitudinal episode.  Target
    // velocity/time history remains available exactly as in d90.
    void ClearLongitudinalStateForEmploy() noexcept;
    void PrepareEmployHistoryCommit(
        const DogfightGeometryFrame& frame,
        ObfmEmployHistoryCommit& output,
        Status& status) const noexcept;
    void CommitEmployPublished(
        const ObfmEmployHistoryCommit& commit,
        Status& status) noexcept;
    void CopySnapshot(ObfmLagGuidanceSnapshot& output) const noexcept;
    void Prepare(
        const DogfightGeometryFrame& frame,
        const OptionalFrameIndex& safety_frame_index,
        ObfmLagGuidancePreparation& output,
        Status& status) const noexcept;
    void BuildCandidate(
        const DogfightGeometryFrame& frame,
        const ObfmLagGuidancePreparation& preparation,
        const ObfmLagGuidanceInput& input,
        ControlIntent& output,
        ObfmLagGuidanceCommit& commit,
        Status& status) const noexcept;
    void ValidatePublished(
        const ObfmLagGuidanceCommit& commit,
        Status& status) const noexcept;
    void CommitPublished(
        const ObfmLagGuidanceCommit& commit,
        Status& status) noexcept;

private:
    ObfmLagGuidanceSnapshot snapshot_{};
    std::uint64_t lifecycle_generation_ = 0U;
    std::uint64_t commit_count_ = 0U;
};

static_assert(
    std::is_trivially_copyable<ObfmLagGuidanceInput>::value,
    "OBFM LAG inputs must stay allocation-free and trivially copyable.");
static_assert(
    std::is_trivially_copyable<ObfmLagGuidanceSnapshot>::value,
    "OBFM LAG state must stay allocation-free and trivially copyable.");
static_assert(
    std::is_trivially_copyable<ObfmLagGuidancePreparation>::value,
    "OBFM LAG preparation must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmLagGuidanceCommit>::value,
    "OBFM LAG commit must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmEmployHistoryCommit>::value,
    "OBFM EMPLOY history commit must stay allocation-free.");

} // namespace LadyLuck
