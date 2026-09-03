#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/FrameContext.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

// This header defines only the evidence boundary required by add/main d90's
// production-enabled G4 High-G Barrel owner.  `valid` is set only by the
// command-neutral exact Service after every typed source has been supplied; in
// particular, current Nz or governor Nz must not be substituted for the
// completed total-load receipt used by Python.
enum class HighGBarrelAttackForm : std::uint8_t
{
    Unavailable = 0U,
    Tracking = 1U,
    Snapshot = 2U,
    Indeterminate = 3U
};

enum class HighGBarrelG13AdmissionStatus : std::uint8_t
{
    Unavailable = 0U,
    Hold = 1U,
    ReverseEvaluate = 2U,
    NoAuthority = 3U
};

enum class HighGBarrelG13AdmissionReason : std::uint8_t
{
    Unavailable = 0U,
    FpoOrderRefuted = 1U
};

enum class HighGBarrelG13ScopeGrade : std::uint8_t
{
    Unavailable = 0U,
    Admitted = 1U,
    Refuted = 2U,
    NotObservable = 3U
};

enum class HighGBarrelTotalLoadSource : std::uint8_t
{
    Unavailable = 0U,
    CompletedCisV4LoadVector = 1U
};

enum class HighGBarrelVerticalExcessState : std::uint8_t
{
    WithinResolution = 0U,
    AboveCornerProven = 1U,
    BelowCornerProven = 2U
};

struct HighGBarrelVerticalExcessEvidence
{
    bool valid = false;
    double own_speed_mps = 0.0;
    bool corner_speed_valid = false;
    double corner_speed_mps = 0.0;
    bool excess_valid = false;
    double excess_mps = 0.0;
    double speed_error_bound_mps = 0.0;
    bool evidence_admitted = false;
    HighGBarrelVerticalExcessState state =
        HighGBarrelVerticalExcessState::WithinResolution;
};

// Exact current-frame fields read by d90 obfm_3d safety_admission() and
// running_safety_admission().  Source availability is explicit so a finite
// backing value can never be promoted to physical safety evidence.
struct HighGBarrelSafetyEvidence
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    double state_sample_t_sec = 0.0;
    bool envelope_valid = false;
    bool hard_deck_source_valid = false;
    double hard_deck_margin_m = 0.0;
    bool stall_source_valid = false;
    double stall_speed_1g_mps = 0.0;
    bool flight_path_gamma_limit_source_valid = false;
    double flight_path_gamma_limit_rad = 0.0;
};

struct HighGBarrelAttackFormEvidence
{
    bool valid = false;
    HighGBarrelAttackForm form = HighGBarrelAttackForm::Unavailable;
    bool continuous_aim_solution = false;
    bool tracking_retained_from_previous_sample = false;
};

// Exact bounded G13 exhaustion facts consumed by
// classify_last_ditch_core_entry().  Each tri-state Python Optional[bool] is
// represented by an adjacent validity bit; finite false backing is never
// evidence on its own.
struct HighGBarrelG13Evidence
{
    bool valid = false;
    bool source_identity_valid = false;
    HighGBarrelG13AdmissionStatus status =
        HighGBarrelG13AdmissionStatus::Unavailable;
    HighGBarrelG13AdmissionReason reason =
        HighGBarrelG13AdmissionReason::Unavailable;
    HighGBarrelG13ScopeGrade scope_grade =
        HighGBarrelG13ScopeGrade::Unavailable;
    bool bounded_source_valid_present = false;
    bool bounded_source_valid = false;
    bool fpo_before_defender_body_39_present = false;
    bool fpo_before_defender_body_39_observed = false;
    bool defender_body_39_strict_positive_present = false;
    bool defender_body_39_strict_positive_observed = false;
    bool attacker_pre_passage_present = false;
    bool attacker_pre_passage_observed = false;
    bool attacker_original_turn_committed_present = false;
    bool attacker_original_turn_committed = false;
    bool far_los_steady_veto_present = false;
    bool far_los_steady_veto = false;
    std::int32_t defender_turn_sign = 0;
    bool g13_response_engaged = false;
};

// Previous completed CIS-v4 sample.  G4 requires every total-load value as a
// single receipt and will fail closed if this seam is not bound.  The current
// TacticalAge1ControlFeedback does not yet carry these four fields.
struct HighGBarrelLoadedResponseEvidence
{
    bool valid = false;
    ControlFrameIdentity source_frame_identity{};
    double source_t_sec = 0.0;
    bool feedback_fresh = false;
    bool backend_is_cis_v4 = false;
    bool previous_command_is_root_gun_defense = false;
    DoctrineBehaviorId previous_behavior_id = DoctrineBehaviorId::Invalid;
    bool cis_nan_guard = false;
    bool cis_fallback = false;
    bool energy_rate_measurement_valid = false;
    bool auto_gcas_active = false;
    bool auto_gcas_state_valid = false;
    double total_load_cmd_raw_g = 0.0;
    double total_load_cmd_governed_g = 0.0;
    double total_load_limit_g = 0.0;
    HighGBarrelTotalLoadSource total_load_source =
        HighGBarrelTotalLoadSource::Unavailable;
    bool nz_cmd_governed_valid = false;
    double nz_cmd_governed_g = 0.0;
    bool nz_measured_valid = false;
    double nz_measured_g = 0.0;
};

struct HighGBarrelCornerIntervalEvidence
{
    bool valid = false;
    bool admitted = false;
    double upper_mps = 0.0;
};

// Complete typed seam for a future exact G4 Service.  It is intentionally not
// a member of TacticalCommandBuildInput yet: no production supplier currently
// owns the bounded G13/source-identity or completed total-load receipts.
struct HighGBarrelExactEvidence
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    HighGBarrelSafetyEvidence safety{};
    HighGBarrelAttackFormEvidence attack_form{};
    HighGBarrelG13Evidence g13{};
    HighGBarrelLoadedResponseEvidence loaded_response{};
    HighGBarrelCornerIntervalEvidence own_corner_interval{};
    HighGBarrelVerticalExcessEvidence vertical_excess{};
};

static_assert(
    std::is_trivially_copyable<HighGBarrelExactEvidence>::value,
    "G4 exact evidence must stay allocation-free.");

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
