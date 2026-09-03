#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/committed/G16ProductionEvidence.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g13
{

enum class G13Truth : std::uint8_t
{
    Unresolved = 0U,
    False = 1U,
    True = 2U
};

enum class G13FlatScissorsScopeGrade : std::uint8_t
{
    NotObservable = 0U,
    Refuted = 1U,
    Admitted = 2U
};

enum class G13FlatScissorsAdmissionStatus : std::uint8_t
{
    NoAuthority = 0U,
    Hold = 1U,
    ReverseEvaluate = 2U
};

enum class G13FlatScissorsAdmissionReason : std::uint8_t
{
    SourceUnresolved = 0U,
    SourceInvalid = 1U,
    SourceIdentityUnresolved = 2U,
    ScopeNotObservable = 3U,
    ScopeRefuted = 4U,
    LosVetoUnresolved = 5U,
    FarSteadyLosVeto = 6U,
    FpoOrderUnresolved = 7U,
    FpoOrderRefuted = 8U,
    AttackerPrePassageUnresolved = 9U,
    AttackerAlreadyPassed = 10U,
    AttackerTurnCommitmentUnresolved = 11U,
    AttackerCounterTurnObserved = 12U,
    ReverseEvaluate = 13U
};

struct G13RoleExplicitEpisodeIdentity
{
    bool valid = false;
    std::int32_t defender_aircraft_id = -1;
    std::int32_t attacker_aircraft_id = -1;
    std::uint64_t episode_epoch = 0U;
    std::uint64_t previous_sample_index = 0U;
    std::uint64_t current_sample_index = 0U;
    double previous_t_sec = 0.0;
    double current_t_sec = 0.0;
};

struct G13SignedInterval
{
    bool valid = false;
    double nominal = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    std::int32_t resolved_sign = 0;
};

// Command-neutral result of the d90 G13 observer.  It carries only bounded
// geometry and lifecycle admission evidence; it is not guidance or FCS
// authority.
struct G13FlatScissorsObservation
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    G13RoleExplicitEpisodeIdentity source_identity{};
    G13FlatScissorsAdmissionStatus admission_status =
        G13FlatScissorsAdmissionStatus::NoAuthority;
    G13FlatScissorsAdmissionReason admission_reason =
        G13FlatScissorsAdmissionReason::SourceUnresolved;
    G13FlatScissorsScopeGrade scope_grade =
        G13FlatScissorsScopeGrade::NotObservable;
    G13Truth bounded_source_valid = G13Truth::Unresolved;
    G13Truth fpo_before_defender_body_39_observed = G13Truth::Unresolved;
    G13Truth attacker_pre_passage_observed = G13Truth::Unresolved;
    G13Truth attacker_original_turn_committed = G13Truth::Unresolved;
    G13Truth far_los_steady_veto = G13Truth::Unresolved;
    G13Truth horizontal_same_turn_scope = G13Truth::Unresolved;
    bool defender_turn_sign_valid = false;
    std::int32_t defender_turn_sign = 0;
    bool attacker_turn_sign_valid = false;
    std::int32_t attacker_turn_sign = 0;
    bool previous_attacker_turn_sign_valid = false;
    std::int32_t previous_attacker_turn_sign = 0;
    G13SignedInterval defender_body_39_margin_m{};
    G13SignedInterval attacker_body_39_margin_m{};
    bool fpo_geometry_evaluable = false;
    bool fpo_dual_frozen_axis_crossing_resolved = false;
    bool los_observation_valid = false;
    double range_m = 0.0;
    double los_rate_rad_s = 0.0;
    double los_steady_bound_rad_s = 0.0;
    bool los_steady = false;
    G13Truth attacker_official_far = G13Truth::Unresolved;
};

class G13FlatScissorsObserver final
{
public:
    G13FlatScissorsObserver() noexcept = default;

    // Public only so translation-unit numerical helpers can remain ordinary
    // allocation-free functions; callers must treat this as implementation
    // storage and use Update for authority-bearing receipts.
    struct FlightPathSample
    {
        bool valid = false;
        ControlFrameIdentity frame_identity{};
        Vector3 relative_position_ned_m{};
        Vector3 own_horizontal_course_ned{};
        double relative_position_error_bound_m = 0.0;
        double own_course_error_bound_rad = 0.0;
        double own_course_normalization_error_bound_l1 = 0.0;
    };

    void Reset() noexcept;
    void Update(
        const runtime::TacticalCommandBuildInput& input,
        G13FlatScissorsObservation& output,
        Status& status) noexcept;

private:
    void Anchor(
        const runtime::TacticalCommandBuildInput& input,
        const FlightPathSample& sample) noexcept;

    guidance::committed::G16ProductionEvidenceProvider
        defender_boundary_observer_{};
    guidance::committed::G16ProductionEvidenceProvider
        attacker_boundary_observer_{};
    bool boundary_seeded_ = false;
    bool previous_valid_ = false;
    runtime::TacticalCommandBuildInput previous_input_{};
    FlightPathSample previous_fpo_sample_{};
    bool previous_defender_margin_valid_ = false;
    G13SignedInterval previous_defender_margin_{};
    bool previous_attacker_turn_sign_valid_ = false;
    std::int32_t previous_attacker_turn_sign_ = 0;
    bool last_resolved_attacker_turn_sign_valid_ = false;
    std::int32_t last_resolved_attacker_turn_sign_ = 0;
    bool attacker_counter_turn_observed_ = false;
};

static_assert(
    std::is_trivially_copyable<G13FlatScissorsObservation>::value,
    "G13 observation must remain allocation-free.");

} // namespace g13
} // namespace guidance
} // namespace LadyLuck
