#include "LadyLuck/runtime/RootTacticalCommandWriters.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/guidance/dbfm/DbfmBreakLoadControlIntent.hpp"
#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeedControlIntent.hpp"

#include <cstddef>
#include <cmath>
#include <limits>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    // Frozen NumPy 1.26.4 length-three dot association.
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1]
            + value[2] * value[2]));
}

void Fail(
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output.Clear();
    status.code = code;
}

void FirstHorizontalUnit(
    const LadyLuck::Vector3* const candidates,
    const std::size_t candidate_count,
    LadyLuck::Vector3& output,
    LadyLuck::Status& status) noexcept
{
    output = LadyLuck::Vector3{};
    status = LadyLuck::Status{};
    for (std::size_t index = 0U; index < candidate_count; ++index)
    {
        const LadyLuck::Vector3& candidate = candidates[index];
        LadyLuck::Vector3 horizontal = candidate;
        horizontal[2] = 0.0;
        const double magnitude = VectorNorm(horizontal);
        // Python skips a derived nonfinite candidate and tries the next
        // already-validated nose/velocity direction.
        if (std::isfinite(magnitude)
            && magnitude >= LadyLuck::constants::Tiny)
        {
            output = LadyLuck::Vector3{{
                horizontal[0] / magnitude,
                horizontal[1] / magnitude,
                0.0}};
            return;
        }
    }
    status.code = LadyLuck::StatusCode::InvalidArgument;
}

bool VelocityNormalUnit(
    const LadyLuck::Vector3& velocity,
    const LadyLuck::Vector3& preferred_direction,
    const LadyLuck::Vector3& negative_down,
    const LadyLuck::Vector3& nose,
    LadyLuck::Vector3& output) noexcept
{
    output = LadyLuck::Vector3{};
    const double velocity_norm = VectorNorm(velocity);
    if (!std::isfinite(velocity_norm)
        || velocity_norm <= LadyLuck::constants::Tiny)
    {
        return false;
    }

    const LadyLuck::Vector3 velocity_hat{{
        velocity[0] / velocity_norm,
        velocity[1] / velocity_norm,
        velocity[2] / velocity_norm}};
    const LadyLuck::Vector3 candidates[] = {
        preferred_direction,
        negative_down,
        nose,
        LadyLuck::Vector3{{0.0, 0.0, -1.0}},
        LadyLuck::Vector3{{0.0, 1.0, 0.0}},
        LadyLuck::Vector3{{1.0, 0.0, 0.0}}};
    for (const LadyLuck::Vector3& candidate : candidates)
    {
        const double parallel =
            candidate[0] * velocity_hat[0]
            + candidate[1] * velocity_hat[1]
            + candidate[2] * velocity_hat[2];
        LadyLuck::Vector3 projected{{
            candidate[0] - parallel * velocity_hat[0],
            candidate[1] - parallel * velocity_hat[1],
            candidate[2] - parallel * velocity_hat[2]}};
        const double projected_norm = VectorNorm(projected);
        if (!std::isfinite(projected_norm)
            || projected_norm <= LadyLuck::constants::Tiny)
        {
            continue;
        }
        for (double& component : projected)
        {
            component /= projected_norm;
        }
        if (FiniteVector(projected))
        {
            output = projected;
            return true;
        }
    }
    return false;
}

void BuildGunBreakIntent(
    const LadyLuck::DogfightGeometryFrame& frame,
    const std::int32_t side_sign,
    const bool previous_direction_available,
    const LadyLuck::Vector3& previous_direction,
    LadyLuck::Vector3& resolved_maneuver_direction,
    bool& vertical_geometry_containment,
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status) noexcept
{
    resolved_maneuver_direction = LadyLuck::Vector3{};
    vertical_geometry_containment = false;
    output.Clear();
    status = LadyLuck::Status{};
    if (side_sign != -1 && side_sign != 1)
    {
        Fail(output, status, LadyLuck::StatusCode::InvalidArgument);
        return;
    }

    const LadyLuck::Vector3& own_position = frame.own.position_ned_m;
    const LadyLuck::Vector3& opponent_position = frame.opponent.position_ned_m;
    const LadyLuck::Vector3& own_velocity = frame.own.velocity_ned_mps;
    const LadyLuck::Vector3& own_nose = frame.own.nose_ned;
    const LadyLuck::Vector3& own_down = frame.own.down_ned;
    if (!FiniteVector(own_position)
        || !FiniteVector(opponent_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(own_nose)
        || !FiniteVector(own_down))
    {
        Fail(output, status, LadyLuck::StatusCode::NonFiniteInput);
        return;
    }

    const LadyLuck::Vector3 line_of_sight{{
        opponent_position[0] - own_position[0],
        opponent_position[1] - own_position[1],
        0.0}};
    LadyLuck::Vector3 line_of_sight_direction{};
    const LadyLuck::Vector3 break_candidates[] = {
        line_of_sight,
        own_nose,
        own_velocity};
    FirstHorizontalUnit(
        break_candidates,
        sizeof(break_candidates) / sizeof(break_candidates[0]),
        line_of_sight_direction,
        status);
    LadyLuck::Vector3 maneuver_direction{};
    if (status.ok())
    {
        // Resolved threat/course geometry keeps the existing alternating or
        // observed toward-side horizontal BREAK.
        maneuver_direction = LadyLuck::Vector3{{
            -static_cast<double>(side_sign) * line_of_sight_direction[1],
            static_cast<double>(side_sign) * line_of_sight_direction[0],
            0.0}};
    }
    else if (status.code == LadyLuck::StatusCode::InvalidArgument)
    {
        vertical_geometry_containment = true;
        // Preserve the immediately preceding accepted Gun direction when the
        // same episode supplies one.  History is optional continuity evidence,
        // so a degenerate history must not erase the current base BREAK.
        if (previous_direction_available)
        {
            const LadyLuck::Vector3 previous_candidates[] = {
                previous_direction};
            FirstHorizontalUnit(
                previous_candidates,
                sizeof(previous_candidates) / sizeof(previous_candidates[0]),
                maneuver_direction,
                status);
        }

        // A first-frame singular threat has no prior validated BREAK.  Use
        // only the current measured body-up/lift-plane direction (-body-down)
        // as a bounded evasive AimPoint reference.  The held side is retained
        // in owner state and resumes its exact Python meaning as soon as the
        // normal horizontal geometry is observable again.  This raw guidance
        // fallback does not by itself prove the sign of downstream Nz or the
        // actual aircraft response.
        if (!status.ok())
        {
            const LadyLuck::Vector3 negative_down{{
                -own_down[0],
                -own_down[1],
                -own_down[2]}};
            const LadyLuck::Vector3 lift_candidates[] = {negative_down};
            FirstHorizontalUnit(
                lift_candidates,
                sizeof(lift_candidates) / sizeof(lift_candidates[0]),
                maneuver_direction,
                status);
            if (!status.ok())
            {
                // Exact vertical/coincident geometry has no observable
                // horizontal course.  Use the existing episode side with a
                // deterministic unit course; this is numerical totalization,
                // not a new maneuver threshold or gain.
                maneuver_direction = LadyLuck::Vector3{{
                    0.0,
                    static_cast<double>(side_sign),
                    0.0}};
                status = LadyLuck::Status{};
            }
        }
    }
    else
    {
        output.Clear();
        return;
    }

    resolved_maneuver_direction = maneuver_direction;

    const double range_m = frame.enemy_offense.range_m;
    const double speed_mps = VectorNorm(own_velocity);
    const double capture_range_m = frame.enemy_offense.phase.max_range_m;
    const double finite_values[] = {range_m, speed_mps, capture_range_m};
    for (const double value : finite_values)
    {
        if (!std::isfinite(value))
        {
            Fail(output, status, LadyLuck::StatusCode::NonFiniteInput);
            return;
        }
    }

    const double aim_distance_m = range_m > LadyLuck::constants::Tiny
        ? range_m
        : (capture_range_m > LadyLuck::constants::Tiny
            ? capture_range_m
            : LadyLuck::constants::Tiny);
    LadyLuck::Vector3 aim_point{{
        own_position[0] + aim_distance_m * maneuver_direction[0],
        own_position[1] + aim_distance_m * maneuver_direction[1],
        own_position[2]}};
    if (!FiniteVector(aim_point))
    {
        // A finite but unphysical range can overflow the displacement.  Keep
        // the same direction and use only the existing numerical epsilon.
        aim_point = LadyLuck::Vector3{{
            own_position[0]
                + LadyLuck::constants::Tiny * maneuver_direction[0],
            own_position[1]
                + LadyLuck::constants::Tiny * maneuver_direction[1],
            own_position[2]}};
    }
    if (!FiniteVector(aim_point))
    {
        Fail(output, status, LadyLuck::StatusCode::NonFiniteInput);
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.aim_point_m = aim_point;
    output.desired_speed_mps = speed_mps;
    output.capture_range_des_m = capture_range_m > 0.0
        ? capture_range_m
        : 0.0;
    output.route_kind = LadyLuck::ControlRouteKind::AimPoint;
    output.behavior_id =
        LadyLuck::DoctrineBehaviorId::GunDefenseHorizontalBreak;
    // The source TacticalCommand labels the break DBFM even though Root owns
    // its higher-priority selection.
    output.mode_id = LadyLuck::DoctrineModeId::Dbfm;
    output.writer_id =
        LadyLuck::ControlIntentWriterGunDefenseHorizontalBreak;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

} // namespace

namespace LadyLuck
{
namespace runtime
{

RootTacticalCommandWriters::RootTacticalCommandWriters() noexcept
{
    Reset();
}

void RootTacticalCommandWriters::Reset() noexcept
{
    ResetGunThreatEpisode();
    horizontal_course_history_valid_ = false;
    horizontal_course_history_ned_ = Vector3{};
    horizontal_course_history_frame_ = ControlFrameIdentity{};
}

void RootTacticalCommandWriters::ResetGunThreatEpisode() noexcept
{
    gun_threat_active_ = false;
    gun_side_sign_ = 1;
    gun_entry_count_ = 0U;
    gun_break_direction_valid_ = false;
    gun_break_direction_ned_ = Vector3{};
    gun_break_direction_frame_ = ControlFrameIdentity{};
    pending_gun_break_direction_valid_ = false;
    pending_gun_break_direction_ned_ = Vector3{};
    pending_gun_break_direction_frame_ = ControlFrameIdentity{};
}

void RootTacticalCommandWriters::ObserveOfficialGunThreat(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const double observed_damage_rate = frame.enemy_offense.damage_rate;
    if (!std::isfinite(observed_damage_rate))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double damage_rate = observed_damage_rate > 0.0
        ? observed_damage_rate
        : 0.0;
    output = damage_rate > 0.0;
}

void RootTacticalCommandWriters::ObserveAdmittedGunThreat(
    const bool admitted_threat_active,
    Status& status) noexcept
{
    status = Status{};
    if (!admitted_threat_active)
    {
        // Python GunDefensePolicy.update clears the active episode on every
        // admitted no-threat observation while preserving entry_count/side.
        gun_threat_active_ = false;
        gun_break_direction_valid_ = false;
        gun_break_direction_ned_ = Vector3{};
        gun_break_direction_frame_ = ControlFrameIdentity{};
        pending_gun_break_direction_valid_ = false;
        pending_gun_break_direction_ned_ = Vector3{};
        pending_gun_break_direction_frame_ = ControlFrameIdentity{};
    }
}

void RootTacticalCommandWriters::ObserveRootGunPreTaskEvidence(
    const DogfightGeometryFrame& frame,
    RootGunPreTaskEvidence& output,
    Status& status) noexcept
{
    pre_task_evidence_provider_.Observe(frame, output, status);
    if (!status.ok())
    {
        return;
    }

    // Keep only a physically observed horizontal course from the accepted
    // current frame.  This history never spans an episode and is consumed
    // only by the immediately following frame when both current nose and
    // current velocity projections are unobservable.
    if (!FiniteVector(frame.own.nose_ned)
        || !FiniteVector(frame.own.velocity_ned_mps))
    {
        output = RootGunPreTaskEvidence{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const Vector3 course_candidates[] = {
        frame.own.nose_ned,
        frame.own.velocity_ned_mps};
    Vector3 current_course{};
    Status course_status{};
    FirstHorizontalUnit(
        course_candidates,
        sizeof(course_candidates) / sizeof(course_candidates[0]),
        current_course,
        course_status);
    if (course_status.code == StatusCode::Ok)
    {
        horizontal_course_history_valid_ = true;
        horizontal_course_history_ned_ = current_course;
        horizontal_course_history_frame_ = frame.frame_identity;
    }
    else if (course_status.code != StatusCode::InvalidArgument)
    {
        output = RootGunPreTaskEvidence{};
        status = course_status;
    }
}

void RootTacticalCommandWriters::BuildGunDefense(
    const DogfightGeometryFrame& frame,
    const RootGunPreTaskEvidence& evidence,
    const bool admitted_threat_active,
    const bool entry_side_sign_valid,
    const std::int32_t entry_side_sign,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    pending_gun_break_direction_valid_ = false;
    pending_gun_break_direction_ned_ = Vector3{};
    pending_gun_break_direction_frame_ = ControlFrameIdentity{};
    const bool expected_capability_admission =
        evidence.capability.n_channel_trusted
        && evidence.capability.n_inst_g.has_value;
    const bool optional_evidence_current = evidence.valid
        && SameControlFrameIdentity(
            evidence.frame_identity,
            frame.frame_identity);
    const bool optional_load_evidence_admitted =
        optional_evidence_current
        && evidence.capability_admitted
        && evidence.capability_admitted == expected_capability_admission
        && evidence.capability.n_inst_g.has_value
        && std::isfinite(evidence.capability.n_inst_g.value)
        && evidence.capability.n_inst_g.value > 0.0;
    if (!admitted_threat_active)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    const bool entered = !gun_threat_active_;
    std::int32_t candidate_side_sign = gun_side_sign_;
    std::uint64_t candidate_entry_count = gun_entry_count_;
    if (entered)
    {
        const bool valid_observed_side = entry_side_sign_valid
            && (entry_side_sign == -1 || entry_side_sign == 1);
        candidate_side_sign = valid_observed_side
            ? entry_side_sign
            : (candidate_entry_count % 2U == 0U ? 1 : -1);
        if (candidate_entry_count
            < (std::numeric_limits<std::uint64_t>::max)())
        {
            ++candidate_entry_count;
        }
    }

    const bool previous_direction_available = gun_threat_active_
        && gun_break_direction_valid_
        && IsValidControlFrameIdentity(gun_break_direction_frame_)
        && gun_break_direction_frame_.episode_epoch
            == frame.frame_identity.episode_epoch
        && gun_break_direction_frame_.frame_index
            < (std::numeric_limits<std::uint64_t>::max)()
        && gun_break_direction_frame_.frame_index + 1U
            == frame.frame_identity.frame_index
        && gun_break_direction_frame_.source_time_s
            < frame.frame_identity.source_time_s
        && FiniteVector(gun_break_direction_ned_);
    Vector3 candidate_maneuver_direction{};
    bool vertical_geometry_containment = false;
    ControlIntent candidate{};
    Status candidate_status{};
    BuildGunBreakIntent(
        frame,
        candidate_side_sign,
        previous_direction_available,
        gun_break_direction_ned_,
        candidate_maneuver_direction,
        vertical_geometry_containment,
        candidate,
        candidate_status);
    if (!candidate_status.ok())
    {
        Fail(output, status, candidate_status.code);
        return;
    }

    const Vector3 base_maneuver_direction = candidate_maneuver_direction;
    Vector3 direct_maneuver_direction{};
    bool direct_maneuver_direction_available = false;
    if (vertical_geometry_containment)
    {
        // Direct-NED consumes a velocity-normal force direction.  Its
        // materialization is optional: try the accepted BREAK direction, the
        // measured lift direction, nose, and fixed basis vectors in that
        // order.  Zero speed has no defined velocity-normal plane and keeps
        // the already-valid AimPoint writer instead of creating a command gap.
        const Vector3 negative_down{{
            -frame.own.down_ned[0],
            -frame.own.down_ned[1],
            -frame.own.down_ned[2]}};
        direct_maneuver_direction_available = VelocityNormalUnit(
            frame.own.velocity_ned_mps,
            base_maneuver_direction,
            negative_down,
            frame.own.nose_ned,
            direct_maneuver_direction);
    }

    // GunDefensePolicy commits the threat episode immediately after the base
    // horizontal_break_command validates.  Later pre-root materialization may
    // surface a programmer/integration fault, but it must not roll back that
    // already-admitted episode entry.
    gun_threat_active_ = true;
    gun_side_sign_ = candidate_side_sign;
    gun_entry_count_ = candidate_entry_count;

    DbfmCornerSpeedControlEvidence speed_evidence{};
    const bool finite_corner_upper =
        evidence.instantaneous_corner_interval.upper_mps.has_value
        && std::isfinite(
            evidence.instantaneous_corner_interval.upper_mps.value)
        && evidence.instantaneous_corner_interval.upper_mps.value > 0.0;
    speed_evidence.instantaneous_upper_mps.has_value =
        optional_evidence_current && finite_corner_upper;
    speed_evidence.instantaneous_upper_mps.value = finite_corner_upper
        ? evidence.instantaneous_corner_interval.upper_mps.value
        : 0.0;
    speed_evidence.instantaneous_admitted =
        optional_evidence_current
        && finite_corner_upper
        && evidence.instantaneous_corner_interval.admitted();
    ControlIntent speed_shaped = candidate;
    ControlIntent speed_trial{};
    Status speed_status{};
    ApplyDbfmDefenseSpeed(
        candidate,
        speed_evidence,
        speed_trial,
        speed_status);
    if (speed_status.ok())
    {
        Status validation{};
        speed_trial.Validate(validation);
        if (validation.ok()
            && speed_trial.writer_id
                == ControlIntentWriterGunDefenseHorizontalBreak
            && SameControlFrameIdentity(
                speed_trial.frame_identity,
                frame.frame_identity))
        {
            speed_shaped = speed_trial;
        }
    }
    status = Status{};

    if (vertical_geometry_containment)
    {
        // Route5 derives course from horizontal velocity and is singular for
        // the exact vertical flight path that triggered this containment.
        // Materialize the measured velocity-normal force directly instead.
        // An admitted instantaneous E-M load is used when available; otherwise
        // one physical g is the bounded baseline request and the downstream
        // direct-NED envelope still clips it to current feasible authority.
        double requested_load_g = 1.0;
        if (optional_load_evidence_admitted)
        {
            requested_load_g = evidence.capability.n_inst_g.value;
        }
        bool direct_selected = false;
        if (direct_maneuver_direction_available)
        {
            const double force_magnitude = requested_load_g
                * constants::StandardGravityMps2;
            ControlIntent direct = speed_shaped;
            direct.total_load_factor_limit_g.has_value = true;
            direct.total_load_factor_limit_g.value = requested_load_g;
            direct.direct_acceleration_ned_mps2.has_value = true;
            direct.direct_acceleration_ned_mps2.value = Vector3{{
                force_magnitude * direct_maneuver_direction[0],
                force_magnitude * direct_maneuver_direction[1],
                constants::StandardGravityMps2
                    + force_magnitude * direct_maneuver_direction[2]}};
            direct.direct_acceleration_tracking_enabled = false;
            direct.direct_acceleration_tracking_observation_only = false;
            direct.direct_acceleration_magnitude_tracking_enabled = false;
            direct.direct_acceleration_loaded_roll_enabled = false;
            direct.direct_acceleration_load_component_compensation_enabled =
                false;
            direct.direct_acceleration_yaw_coordination_enabled = false;
            direct.direct_acceleration_roll_priority_yaw_enabled = false;
            direct.route_kind = ControlRouteKind::DirectNedAcceleration;
            Status direct_status{};
            direct.Validate(direct_status);
            if (direct_status.ok())
            {
                output = direct;
                direct_selected = true;
            }
        }
        if (!direct_selected)
        {
            output = speed_shaped;
        }
        status = Status{};
        pending_gun_break_direction_valid_ = true;
        pending_gun_break_direction_ned_ = direct_selected
            ? direct_maneuver_direction
            : base_maneuver_direction;
        pending_gun_break_direction_frame_ = frame.frame_identity;
        return;
    }

    DbfmBreakLoadKinematics own{};
    own.position_ned_m = frame.own.position_ned_m;
    own.velocity_body_mps = frame.own.velocity_body_mps;
    own.velocity_world_ned_mps = frame.own.velocity_ned_mps;
    own.rpy_rad = frame.own.rpy_rad;
    DbfmBreakLoadEvidence break_evidence{};
    break_evidence.capability_admitted =
        optional_load_evidence_admitted;
    break_evidence.instantaneous_load_limit_g.has_value =
        optional_load_evidence_admitted;
    break_evidence.instantaneous_load_limit_g.value =
        optional_load_evidence_admitted
            ? evidence.capability.n_inst_g.value
            : 0.0;
    DbfmBreakLoadConfig break_config{};
    break_config.enabled = true;
    break_config.magnitude_tracking_enabled = true;
    break_config.loaded_roll_enabled = false;
    ControlIntent materialized = speed_shaped;
    ControlIntent load_trial{};
    Status load_status{};
    ApplyDbfmBreakLoad(
        speed_shaped,
        own,
        break_evidence,
        break_config,
        load_trial,
        load_status);
    if (load_status.ok())
    {
        Status validation{};
        load_trial.Validate(validation);
        if (validation.ok()
            && load_trial.writer_id
                == ControlIntentWriterGunDefenseHorizontalBreak
            && SameControlFrameIdentity(
                load_trial.frame_identity,
                frame.frame_identity))
        {
            materialized = load_trial;
        }
    }

    output = materialized;
    status = Status{};
    pending_gun_break_direction_valid_ = true;
    pending_gun_break_direction_ned_ = base_maneuver_direction;
    pending_gun_break_direction_frame_ = frame.frame_identity;
}

void RootTacticalCommandWriters::CommitPublishedGunDirection(
    const ControlFrameIdentity& frame_identity) noexcept
{
    const double pending_norm = VectorNorm(
        pending_gun_break_direction_ned_);
    if (pending_gun_break_direction_valid_
        && IsValidControlFrameIdentity(frame_identity)
        && SameControlFrameIdentity(
            pending_gun_break_direction_frame_,
            frame_identity)
        && FiniteVector(pending_gun_break_direction_ned_)
        && std::isfinite(pending_norm)
        && pending_norm >= constants::Tiny)
    {
        gun_break_direction_valid_ = true;
        gun_break_direction_ned_ = pending_gun_break_direction_ned_;
        gun_break_direction_frame_ = frame_identity;
    }
    else
    {
        // History is optional containment evidence.  Never convert a history
        // mismatch into a command-path fault after Root publication.
        gun_break_direction_valid_ = false;
        gun_break_direction_ned_ = Vector3{};
        gun_break_direction_frame_ = ControlFrameIdentity{};
    }
    pending_gun_break_direction_valid_ = false;
    pending_gun_break_direction_ned_ = Vector3{};
    pending_gun_break_direction_frame_ = ControlFrameIdentity{};
}

void RootTacticalCommandWriters::BuildRootAutoGcasRecovery(
    const DogfightGeometryFrame& frame,
    const safety::AutoGcasEntryReceipt& safety,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    const double speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    if (!safety.valid
        || !safety.entry_available
        || !safety.entry_recoverable
        || !SameControlFrameIdentity(safety.frame_identity, frame.frame_identity)
        || !std::isfinite(speed_mps)
        || speed_mps <= 0.0)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    ControlIntent candidate{};
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = frame.own.position_ned_m;
    candidate.desired_speed_mps = speed_mps;
    candidate.route_kind = ControlRouteKind::SafetyRecovery;
    candidate.behavior_id = DoctrineBehaviorId::AutoGcasRecovery;
    candidate.mode_id = DoctrineModeId::Safety;
    candidate.writer_id = ControlIntentWriterAutoGcasRecovery;
    candidate.Validate(status);
    if (!status.ok())
    {
        output.Clear();
        return;
    }
    output = candidate;
}

} // namespace runtime
} // namespace LadyLuck
