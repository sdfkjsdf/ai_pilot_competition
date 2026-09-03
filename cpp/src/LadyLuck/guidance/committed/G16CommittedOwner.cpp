#include "LadyLuck/guidance/committed/G16CommittedOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::Vector3;

constexpr double kRoundoff =
    64.0 * std::numeric_limits<double>::epsilon();

bool Finite(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        scalar * value[0],
        scalar * value[1],
        scalar * value[2]}};
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

double Norm(const Vector3& value) noexcept
{
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

bool Unit(const Vector3& value, Vector3& output) noexcept
{
    output = Vector3{};
    const double magnitude = Norm(value);
    if (!Finite(value) || !std::isfinite(magnitude) || magnitude <= 0.0)
    {
        return false;
    }
    output = Scale(value, 1.0 / magnitude);
    return Finite(output);
}

bool HorizontalCourse(
    const Vector3& current_direction,
    const double direction_resolution_rad,
    Vector3& output) noexcept
{
    output = Vector3{};
    if (!Finite(current_direction)
        || !std::isfinite(direction_resolution_rad)
        || direction_resolution_rad < 0.0
        || direction_resolution_rad >= 0.5 * LadyLuck::constants::Pi)
    {
        return false;
    }
    const double horizontal_magnitude = std::hypot(
        current_direction[0],
        current_direction[1]);
    const double horizontal_separation_from_vertical = std::atan2(
        horizontal_magnitude,
        std::abs(current_direction[2]));
    if (!std::isfinite(horizontal_magnitude)
        || !std::isfinite(horizontal_separation_from_vertical)
        || horizontal_magnitude <= 0.0
        || horizontal_separation_from_vertical <= direction_resolution_rad)
    {
        return false;
    }
    output = Vector3{{
        current_direction[0] / horizontal_magnitude,
        current_direction[1] / horizontal_magnitude,
        0.0}};
    return Finite(output);
}

bool ActivePhase(
    const LadyLuck::guidance::committed::G16CommitPhase phase) noexcept
{
    using LadyLuck::guidance::committed::G16CommitPhase;
    return phase == G16CommitPhase::Committed
        || phase == G16CommitPhase::BlowThrough;
}

bool FiniteEvidence(
    const LadyLuck::guidance::committed::G16ProductionEvidenceReceipt&
        evidence) noexcept
{
    return evidence.valid
        && LadyLuck::IsValidControlFrameIdentity(evidence.frame_identity)
        && LadyLuck::SameControlFrameIdentity(
            evidence.frame_identity,
            evidence.frame.frame_identity)
        && evidence.boundary.valid
        && std::isfinite(evidence.boundary.signed_margin_m)
        && std::isfinite(evidence.boundary.signed_margin_error_bound_m)
        && evidence.boundary.signed_margin_error_bound_m >= 0.0
        && std::isfinite(evidence.own_speed_mps)
        && evidence.own_speed_mps > 0.0
        && std::isfinite(evidence.enemy_range_m)
        && evidence.enemy_range_m >= 0.0
        && (!evidence.enemy_range_interval.valid
            || (std::isfinite(
                    evidence.enemy_range_interval.error_bound_m)
                && evidence.enemy_range_interval.error_bound_m >= 0.0
                && std::isfinite(evidence.enemy_range_interval.lower_m)
                && std::isfinite(evidence.enemy_range_interval.upper_m)
                && evidence.enemy_range_interval.lower_m
                    <= evidence.enemy_range_interval.upper_m))
        && std::isfinite(evidence.enemy_outer_wez_range_m)
        && evidence.enemy_outer_wez_range_m > 0.0
        && (!evidence.own_velocity_direction_resolution_valid
            || (std::isfinite(
                    evidence.own_velocity_direction_resolution_rad)
                && evidence.own_velocity_direction_resolution_rad >= 0.0
                && evidence.own_velocity_direction_resolution_rad
                    < 0.5 * LadyLuck::constants::Pi))
        && Finite(evidence.frame.own.position_ned_m)
        && Finite(evidence.frame.own.velocity_ned_mps)
        && Finite(evidence.frame.opponent.velocity_ned_mps);
}

bool RobustlyOutsideLatchedWez(
    const LadyLuck::guidance::committed::G16ProductionEvidenceReceipt&
        evidence,
    const double enemy_outer_wez_range_m) noexcept
{
    return evidence.enemy_range_interval.valid
        && std::isfinite(evidence.enemy_range_interval.lower_m)
        && std::isfinite(enemy_outer_wez_range_m)
        && evidence.enemy_range_interval.lower_m > enemy_outer_wez_range_m;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

void G16CommittedOwner::Reset() noexcept
{
    phase_ = G16CommitPhase::Idle;
    handoff_stream_active_ = false;
    episode_identity_valid_ = false;
    episode_epoch_ = 0U;
    last_frame_index_ = 0U;
    last_source_time_s_ = 0.0;
    previous_committed_margin_m_ = 0.0;
    previous_committed_margin_valid_ = false;
    previous_idle_margin_m_ = 0.0;
    previous_idle_margin_valid_ = false;
    previous_enemy_range_m_ = 0.0;
    previous_enemy_range_valid_ = false;
    previous_blowthrough_robust_outside_ = false;
    previous_blowthrough_robust_outside_valid_ = false;
    enemy_outer_wez_range_m_ = 0.0;
    enemy_outer_wez_range_valid_ = false;
    g19_open_continuous_ = false;
    g19_open_continuous_valid_ = false;
    latched_side_resolved_ = false;
    latched_side_sign_ = 0;
    latched_entry_speed_mps_ = 0.0;
    latched_entry_speed_valid_ = false;
    latched_turn_direction_ned_ = Vector3{};
    latched_turn_direction_resolution_rad_ = 0.0;
    latched_turn_direction_valid_ = false;
    latched_egress_direction_ned_ = Vector3{};
    latched_egress_direction_valid_ = false;
    latched_horizontal_egress_fallback_ = false;
    cached_evidence_ = G16ProductionEvidenceReceipt{};
    cached_receipt_ = G16CommittedOwnerReceipt{};
    cached_receipt_valid_ = false;
}

void G16CommittedOwner::ResetForSafetyPreemption() noexcept
{
    Reset();
}

void G16CommittedOwner::HaltExecutionPreservingLifecycle() noexcept
{
    // This class owns no hidden execution selector.  The visible BT halts its
    // Task, while this command-neutral physical lifecycle remains unchanged.
}

void G16CommittedOwner::ResolveDirectionLatch(
    const G16ProductionEvidenceReceipt& evidence,
    Status& status) noexcept
{
    status = Status{};
    if (!latched_side_resolved_ || latched_turn_direction_valid_)
    {
        return;
    }
    if (!evidence.boundary.defender_turn_side_resolved)
    {
        // A previously latched side can outlive a same-frame turn-plane
        // observation.  Preserve the lifecycle but publish no G16 command
        // until a direction is observable again.
        return;
    }
    if (!Finite(evidence.boundary.defender_turn_acceleration_direction_ned)
        || !std::isfinite(
            evidence.boundary.defender_turn_direction_resolution_rad))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (evidence.boundary.defender_turn_direction_resolution_rad < 0.0
        || evidence.boundary.defender_turn_direction_resolution_rad
            >= 0.5 * constants::Pi)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    Vector3 turn_direction{};
    Vector3 current_direction{};
    if (!Unit(
            evidence.boundary.defender_turn_acceleration_direction_ned,
            turn_direction)
        || !Unit(evidence.frame.own.velocity_ned_mps, current_direction))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double projection = (std::max)(
        -1.0,
        (std::min)(1.0, Dot(current_direction, turn_direction)));
    Vector3 selected{};
    bool horizontal_fallback = false;
    if (projection <= kRoundoff)
    {
        selected = current_direction;
    }
    else
    {
        const Vector3 tangent = Subtract(
            current_direction,
            Scale(turn_direction, projection));
        const double tangent_magnitude = Norm(tangent);
        if (!std::isfinite(tangent_magnitude))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        if (!evidence.own_velocity_direction_resolution_valid)
        {
            return;
        }
        const double combined_direction_resolution_rad = (std::min)(
            constants::Pi,
            evidence.boundary.defender_turn_direction_resolution_rad
                + evidence.own_velocity_direction_resolution_rad);
        const double tangent_separation_rad = std::atan2(
            tangent_magnitude,
            projection);
        if (!std::isfinite(combined_direction_resolution_rad)
            || !std::isfinite(tangent_separation_rad))
        {
            status.code = StatusCode::NonFiniteInput;
            return;
        }
        if (tangent_separation_rad <= combined_direction_resolution_rad)
        {
            if (!HorizontalCourse(
                    current_direction,
                    evidence.own_velocity_direction_resolution_rad,
                    selected))
            {
                return;
            }
            horizontal_fallback = true;
        }
        else if (!Unit(tangent, selected))
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
    }
    if (!horizontal_fallback
        && Dot(selected, turn_direction) > kRoundoff)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    latched_turn_direction_ned_ = turn_direction;
    latched_turn_direction_resolution_rad_ =
        evidence.boundary.defender_turn_direction_resolution_rad;
    latched_turn_direction_valid_ = true;
    latched_egress_direction_ned_ = selected;
    latched_egress_direction_valid_ = true;
    latched_horizontal_egress_fallback_ = horizontal_fallback;
}

void G16CommittedOwner::BuildReceipt(
    const G16ProductionEvidenceReceipt& evidence,
    const G16CommitPhase before,
    const G16CommitEvent event,
    const bool sample_accepted,
    const bool crossing,
    const bool wez_crossing,
    const bool wez_outside_maintained,
    const ThreatRecoveryMarginReceipt& threat_margin,
    G16CommittedOwnerReceipt& output) noexcept
{
    output = G16CommittedOwnerReceipt{};
    output.valid = true;
    output.frame_identity = evidence.frame_identity;
    output.phase_before = before;
    output.phase_after = phase_;
    output.event = event;
    output.sample_accepted = sample_accepted;
    output.entered_this_sample =
        event == G16CommitEvent::Entered
        || event == G16CommitEvent::OvershootRecoveryEntered;
    output.completed_this_sample = event == G16CommitEvent::Completed
        || event == G16CommitEvent::CompletedAlreadyOutsideMaintained;
    output.released_this_sample = event
        == G16CommitEvent::ReleasedThreatRecoveryMarginExhausted;
    output.body_39_crossing_observed = crossing;
    output.wez_outward_crossing_observed = wez_crossing;
    output.wez_outside_maintained_observed = wez_outside_maintained;
    output.command_owner_active = ActivePhase(phase_);
    output.latched_side_resolved = latched_side_resolved_;
    output.latched_side_sign = latched_side_sign_;
    output.horizontal_egress_fallback_latched =
        latched_horizontal_egress_fallback_;
    output.threat_recovery_margin = threat_margin;
    if (event == G16CommitEvent::Completed)
    {
        output.g5b_handoff.valid = true;
        output.g5b_handoff.frame_identity = evidence.frame_identity;
        output.g5b_handoff.completed_this_sample = true;
        output.g5b_handoff.production_evidence = evidence;
    }
}

void G16CommittedOwner::Observe(
    const G16ProductionEvidenceReceipt& evidence,
    G16CommittedOwnerReceipt& output,
    Status& status) noexcept
{
    output = G16CommittedOwnerReceipt{};
    status = Status{};
    if (!evidence.valid)
    {
        return;
    }
    if (!FiniteEvidence(evidence))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (cached_receipt_valid_
        && SameControlFrameIdentity(
            evidence.frame_identity,
            cached_receipt_.frame_identity))
    {
        output = cached_receipt_;
        return;
    }

    const bool stream_needed = handoff_stream_active_
        || ActivePhase(phase_)
        || evidence.handoff_status == G16HandoffStatus::Requested;
    if (!stream_needed)
    {
        return;
    }

    if (episode_identity_valid_)
    {
        const bool rejected =
            evidence.frame_identity.episode_epoch != episode_epoch_
            || evidence.frame_identity.frame_index <= last_frame_index_
            || evidence.frame_identity.source_time_s <= last_source_time_s_;
        if (rejected)
        {
            BuildReceipt(
                evidence,
                phase_,
                G16CommitEvent::SampleRejected,
                false,
                false,
                false,
                false,
                ThreatRecoveryMarginReceipt{},
                output);
            cached_evidence_ = evidence;
            cached_receipt_ = output;
            cached_receipt_valid_ = true;
            return;
        }
    }
    else
    {
        handoff_stream_active_ = true;
    }

    const G16CommitPhase before = phase_;
    if (enemy_outer_wez_range_valid_
        && evidence.enemy_outer_wez_range_m != enemy_outer_wez_range_m_)
    {
        BuildReceipt(
            evidence,
            before,
            G16CommitEvent::SampleRejected,
            false,
            false,
            false,
            false,
            ThreatRecoveryMarginReceipt{},
            output);
        cached_evidence_ = evidence;
        cached_receipt_ = output;
        cached_receipt_valid_ = true;
        return;
    }
    if (!episode_identity_valid_)
    {
        episode_identity_valid_ = true;
        episode_epoch_ = evidence.frame_identity.episode_epoch;
    }
    last_frame_index_ = evidence.frame_identity.frame_index;
    last_source_time_s_ = evidence.frame_identity.source_time_s;

    const double margin = evidence.boundary.signed_margin_m;
    const double error = evidence.boundary.signed_margin_error_bound_m;
    const double margin_lower = margin - error;
    const double margin_upper = margin + error;
    G16CommitEvent event = G16CommitEvent::Retained;
    bool crossing = false;
    bool wez_crossing = false;
    bool wez_outside_maintained = false;
    ThreatRecoveryMarginReceipt threat_margin{};
    EvaluateThreatRecoveryMargin(
        evidence.frame,
        evidence.own_turn_capability.admitted,
        evidence.own_turn_capability.capability_g,
        threat_margin);

    if (before == G16CommitPhase::Idle)
    {
        const bool idle_crossing = previous_idle_margin_valid_
            && margin_upper <= 0.0;
        if (idle_crossing)
        {
            phase_ = G16CommitPhase::BlowThrough;
            crossing = true;
            event = G16CommitEvent::OvershootRecoveryEntered;
        }
        else if (evidence.handoff_status == G16HandoffStatus::Requested
            && margin_lower > 0.0)
        {
            phase_ = G16CommitPhase::Committed;
            event = G16CommitEvent::Entered;
        }
        else
        {
            if (margin_lower > 0.0)
            {
                previous_idle_margin_m_ = margin;
                previous_idle_margin_valid_ = true;
            }
            event = G16CommitEvent::HeldIdle;
        }

        if (phase_ != G16CommitPhase::Idle)
        {
            latched_side_resolved_ =
                evidence.boundary.defender_turn_side_resolved
                && evidence.selected_egress_side_resolved;
            latched_side_sign_ = latched_side_resolved_
                ? evidence.selected_egress_side_sign
                : 0;
            previous_committed_margin_m_ = margin;
            previous_committed_margin_valid_ = true;
            previous_idle_margin_m_ = 0.0;
            previous_idle_margin_valid_ = false;
            previous_enemy_range_m_ = evidence.enemy_range_m;
            previous_enemy_range_valid_ = true;
            enemy_outer_wez_range_m_ = evidence.enemy_outer_wez_range_m;
            enemy_outer_wez_range_valid_ = true;
            previous_blowthrough_robust_outside_valid_ =
                phase_ == G16CommitPhase::BlowThrough
                && evidence.enemy_range_interval.valid;
            previous_blowthrough_robust_outside_ =
                previous_blowthrough_robust_outside_valid_
                && RobustlyOutsideLatchedWez(
                    evidence,
                    enemy_outer_wez_range_m_);
            g19_open_continuous_ = evidence.escape_window_status
                == G16EscapeWindowStatus::Open;
            g19_open_continuous_valid_ = true;
            latched_entry_speed_mps_ = evidence.own_speed_mps;
            latched_entry_speed_valid_ = true;
        }
    }
    else if (before == G16CommitPhase::Complete)
    {
        event = G16CommitEvent::CompletionRetained;
    }
    else if (before == G16CommitPhase::Failed)
    {
        event = G16CommitEvent::FailureRetained;
    }
    else if (before == G16CommitPhase::Released)
    {
        event = G16CommitEvent::ReleaseRetained;
    }
    else
    {
        if (!latched_side_resolved_
            && evidence.boundary.defender_turn_side_resolved
            && evidence.selected_egress_side_resolved)
        {
            latched_side_resolved_ = true;
            latched_side_sign_ = evidence.selected_egress_side_sign;
        }
        if (threat_margin.evaluated && threat_margin.exhausted)
        {
            phase_ = G16CommitPhase::Released;
            event = G16CommitEvent::
                ReleasedThreatRecoveryMarginExhausted;
        }
        else
        {
            const double previous_range = previous_enemy_range_m_;
            const bool previous_range_valid = previous_enemy_range_valid_;
            previous_enemy_range_m_ = evidence.enemy_range_m;
            previous_enemy_range_valid_ = true;
            if (g19_open_continuous_valid_
                && evidence.escape_window_status
                    != G16EscapeWindowStatus::Open)
            {
                g19_open_continuous_ = false;
            }
            if (before == G16CommitPhase::BlowThrough)
            {
                const bool current_robust_outside_valid =
                    evidence.enemy_range_interval.valid;
                const bool current_robust_outside =
                    current_robust_outside_valid
                    && RobustlyOutsideLatchedWez(
                        evidence,
                        enemy_outer_wez_range_m_);
                wez_outside_maintained =
                    previous_blowthrough_robust_outside_valid_
                    && previous_blowthrough_robust_outside_
                    && current_robust_outside_valid
                    && current_robust_outside;
                previous_blowthrough_robust_outside_valid_ =
                    current_robust_outside_valid;
                previous_blowthrough_robust_outside_ =
                    current_robust_outside;
                wez_crossing = previous_range_valid
                    && previous_range <= enemy_outer_wez_range_m_
                    && evidence.enemy_range_m > enemy_outer_wez_range_m_;
                if (wez_crossing)
                {
                    phase_ = G16CommitPhase::Complete;
                    event = G16CommitEvent::Completed;
                }
                else if (wez_outside_maintained)
                {
                    phase_ = G16CommitPhase::Complete;
                    event = G16CommitEvent::
                        CompletedAlreadyOutsideMaintained;
                }
            }
            else
            {
                crossing = margin_upper <= 0.0;
                if (crossing)
                {
                    previous_committed_margin_m_ = margin;
                    previous_committed_margin_valid_ = true;
                    phase_ = G16CommitPhase::BlowThrough;
                    event = G16CommitEvent::Body39Crossed;
                    previous_blowthrough_robust_outside_valid_ =
                        evidence.enemy_range_interval.valid;
                    previous_blowthrough_robust_outside_ =
                        previous_blowthrough_robust_outside_valid_
                        && RobustlyOutsideLatchedWez(
                            evidence,
                            enemy_outer_wez_range_m_);
                }
                else if (margin_lower > 0.0)
                {
                    previous_committed_margin_m_ = margin;
                    previous_committed_margin_valid_ = true;
                }
            }
        }
    }

    if (ActivePhase(phase_) && latched_side_resolved_)
    {
        ResolveDirectionLatch(evidence, status);
        if (!status.ok())
        {
            Reset();
            output = G16CommittedOwnerReceipt{};
            return;
        }
    }
    BuildReceipt(
        evidence,
        before,
        event,
        true,
        crossing,
        wez_crossing,
        wez_outside_maintained,
        threat_margin,
        output);
    cached_evidence_ = evidence;
    cached_receipt_ = output;
    cached_receipt_valid_ = true;
}

void G16CommittedOwner::CopySelection(
    const ControlFrameIdentity& current_identity,
    G16CommittedSelection& output,
    Status& status) const noexcept
{
    output = G16CommittedSelection{};
    status = Status{};
    if (!IsValidControlFrameIdentity(current_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    output.valid = true;
    output.frame_identity = current_identity;
    output.phase = phase_;
    if (!cached_receipt_valid_
        || !cached_receipt_.sample_accepted
        || !SameControlFrameIdentity(
            current_identity,
            cached_receipt_.frame_identity)
        || !ActivePhase(phase_))
    {
        return;
    }
    if (latched_side_resolved_ && !latched_egress_direction_valid_)
    {
        // Finite direction ambiguity is an ordinary G16 nonselection.  The
        // visible Root tree retains sole authority to choose its fallback.
        return;
    }
    output.selected = true;
    output.command_ready = latched_entry_speed_valid_
        && Finite(cached_evidence_.frame.own.velocity_ned_mps)
        && Norm(cached_evidence_.frame.own.velocity_ned_mps) > 0.0;
    output.writer_id = output.command_ready
        ? ControlIntentWriterG16Committed
        : ControlIntentWriterNone;
}

void G16CommittedOwner::BuildCandidate(
    const G16ProductionEvidenceReceipt& evidence,
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    if (!FiniteEvidence(evidence)
        || !cached_receipt_valid_
        || !cached_receipt_.sample_accepted
        || !SameControlFrameIdentity(
            evidence.frame_identity,
            cached_receipt_.frame_identity)
        || !ActivePhase(phase_)
        || !latched_entry_speed_valid_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    Vector3 direction{};
    if (phase_ == G16CommitPhase::BlowThrough
        && latched_side_resolved_)
    {
        if (!latched_egress_direction_valid_
            || !Unit(latched_egress_direction_ned_, direction))
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
    }
    else if (!Unit(evidence.frame.own.velocity_ned_mps, direction))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    const double current_speed = Norm(evidence.frame.own.velocity_ned_mps);
    const double look_distance = evidence.enemy_range_m;
    if (!std::isfinite(current_speed)
        || current_speed <= 0.0
        || !std::isfinite(look_distance)
        || look_distance <= 0.0
        || !std::isfinite(enemy_outer_wez_range_m_)
        || enemy_outer_wez_range_m_ <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output.frame_identity = evidence.frame_identity;
    output.aim_point_m = Vector3{{
        evidence.frame.own.position_ned_m[0]
            + look_distance * direction[0],
        evidence.frame.own.position_ned_m[1]
            + look_distance * direction[1],
        evidence.frame.own.position_ned_m[2]
            + look_distance * direction[2]}};
    output.desired_speed_mps = (std::max)(
        current_speed,
        latched_entry_speed_mps_);
    output.desired_speed_rate_mps2 = 0.0;
    output.specific_energy_rate_bias_m2ps3 = 0.0;
    output.capture_range_des_m = enemy_outer_wez_range_m_;
    output.route_kind = ControlRouteKind::AimPoint;
    output.writer_id = ControlIntentWriterG16Committed;
    output.mode_id = DoctrineModeId::ControlZone;
    if (phase_ == G16CommitPhase::Committed)
    {
        output.behavior_id =
            DoctrineBehaviorId::G16FinalAttackPassLoadVector;
    }
    else if (!latched_side_resolved_)
    {
        output.behavior_id =
            DoctrineBehaviorId::G16BlowThroughSidePendingEnergyRetain;
    }
    else
    {
        output.behavior_id = DoctrineBehaviorId::
            G16BlowThroughMinimumChange3dLoadVector;
    }
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
