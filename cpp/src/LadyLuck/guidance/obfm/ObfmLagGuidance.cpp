#include "LadyLuck/guidance/obfm/ObfmLagGuidance.hpp"

#include "LadyLuck/common/Constants.hpp"

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

double Dot(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

LadyLuck::Vector3 Cross(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Norm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(Dot(value, value));
}

LadyLuck::Vector3 Add(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

LadyLuck::Vector3 Subtract(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return LadyLuck::Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

LadyLuck::Vector3 Scale(
    const LadyLuck::Vector3& value,
    const double scale) noexcept
{
    return LadyLuck::Vector3{{
        value[0] * scale,
        value[1] * scale,
        value[2] * scale}};
}

double ClipUnit(const double value) noexcept
{
    if (value < -1.0)
    {
        return -1.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}

void Fail(
    LadyLuck::ControlIntent& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output.Clear();
    status.code = code;
}

void ConcentricPathPoint(
    const LadyLuck::Vector3& own_position,
    const LadyLuck::Vector3& adversary_position,
    const LadyLuck::Vector3& adversary_velocity,
    const double current_time_s,
    const LadyLuck::ObfmLagGuidanceSnapshot& snapshot,
    LadyLuck::Vector3& output,
    LadyLuck::Status& status) noexcept
{
    output = LadyLuck::Vector3{};
    status = LadyLuck::Status{};

    if (!FiniteVector(own_position)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity)
        || !std::isfinite(current_time_s))
    {
        status.code = LadyLuck::StatusCode::NonFiniteInput;
        return;
    }

    const double adversary_speed = Norm(adversary_velocity);
    if (!std::isfinite(adversary_speed)
        || adversary_speed < LadyLuck::constants::Tiny)
    {
        output = adversary_position;
        return;
    }

    const LadyLuck::Vector3 path_direction =
        Scale(adversary_velocity, 1.0 / adversary_speed);
    const LadyLuck::Vector3 offset =
        Subtract(own_position, adversary_position);
    const double foot_parameter = Dot(offset, path_direction);
    const LadyLuck::Vector3 perpendicular =
        Subtract(offset, Scale(path_direction, foot_parameter));
    const double lag_depth = Norm(perpendicular);

    LadyLuck::Vector3 omega{};
    if (snapshot.previous_adversary_velocity_valid
        && snapshot.previous_time_valid)
    {
        const double dt = current_time_s - snapshot.previous_time_s;
        const double previous_speed =
            Norm(snapshot.previous_adversary_velocity_ned_mps);
        if (dt > 0.0
            && std::isfinite(dt)
            && previous_speed >= LadyLuck::constants::Tiny)
        {
            const LadyLuck::Vector3 previous_direction = Scale(
                snapshot.previous_adversary_velocity_ned_mps,
                1.0 / previous_speed);
            const LadyLuck::Vector3 cross =
                Cross(previous_direction, path_direction);
            const double sine = Norm(cross);
            const double cosine =
                ClipUnit(Dot(previous_direction, path_direction));
            const double angle = std::atan2(sine, cosine);
            if (sine >= LadyLuck::constants::Tiny)
            {
                omega = Scale(cross, (angle / dt) / sine);
            }
        }
    }

    const double omega_magnitude = Norm(omega);
    if (omega_magnitude * adversary_speed >= LadyLuck::constants::Tiny)
    {
        const double radius = adversary_speed / omega_magnitude;
        const LadyLuck::Vector3 normal =
            Scale(omega, 1.0 / omega_magnitude);
        LadyLuck::Vector3 centre_direction =
            Cross(omega, adversary_velocity);
        const double centre_direction_magnitude = Norm(centre_direction);
        if (!std::isfinite(radius)
            || !std::isfinite(centre_direction_magnitude)
            || centre_direction_magnitude == 0.0)
        {
            // All source vectors were finite. Loss of a representable curved
            // centre withholds the curvature refinement and preserves the
            // same-frame target-position reference.
            output = adversary_position;
            return;
        }
        centre_direction =
            Scale(centre_direction, 1.0 / centre_direction_magnitude);
        const LadyLuck::Vector3 centre = Add(
            adversary_position,
            Scale(centre_direction, radius));
        const double arc_angle = lag_depth / radius;
        const LadyLuck::Vector3 spoke =
            Subtract(adversary_position, centre);
        const double cos_a = std::cos(-arc_angle);
        const double sin_a = std::sin(-arc_angle);
        const LadyLuck::Vector3 rotated = Add(
            Add(
                Scale(spoke, cos_a),
                Scale(Cross(normal, spoke), sin_a)),
            Scale(
                normal,
                Dot(normal, spoke) * (1.0 - cos_a)));
        output = Add(centre, rotated);
    }
    else
    {
        output = Subtract(
            adversary_position,
            Scale(path_direction, lag_depth));
    }

    if (!FiniteVector(output))
    {
        // A finite extreme curvature may overflow the optional arc transport.
        // Preserve the finite target reference rather than erasing LAG.
        output = adversary_position;
    }
}

bool FiniteOptional(
    const LadyLuck::IntentOptionalValue<double>& value) noexcept
{
    return std::isfinite(value.value);
}

bool FiniteSnapshot(
    const LadyLuck::ObfmLagGuidanceSnapshot& snapshot) noexcept
{
    return FiniteVector(snapshot.previous_adversary_velocity_ned_mps)
        && std::isfinite(snapshot.previous_time_s)
        && FiniteVector(snapshot.previous_reference_point_ned_m)
        && FiniteVector(snapshot.previous_own_position_ned_m)
        && std::isfinite(snapshot.previous_speed_command_mps);
}

bool FinitePreparation(
    const LadyLuck::ObfmLagGuidancePreparation& preparation) noexcept
{
    return FiniteVector(preparation.current_reference_point_ned_m)
        && FiniteVector(preparation.previous_reference_point_ned_m)
        && FiniteVector(preparation.transported_reference_point_ned_m)
        && FiniteVector(preparation.current_own_position_ned_m)
        && FiniteVector(preparation.current_own_velocity_ned_mps)
        && FiniteVector(preparation.current_target_velocity_ned_mps)
        && std::isfinite(preparation.dt_s)
        && std::isfinite(preparation.current_time_s)
        && std::isfinite(preparation.previous_time_s)
        && std::isfinite(preparation.current_range_m)
        && std::isfinite(preparation.range_rate_mps)
        && std::isfinite(preparation.official_min_range_m)
        && std::isfinite(preparation.official_max_range_m)
        && std::isfinite(preparation.previous_speed_command_mps);
}

} // namespace

namespace LadyLuck
{

ObfmLagGuidance::ObfmLagGuidance() noexcept
{
    Reset();
}

void ObfmLagGuidance::Reset() noexcept
{
    snapshot_ = ObfmLagGuidanceSnapshot{};
    commit_count_ = 0U;
    if (lifecycle_generation_
        == (std::numeric_limits<std::uint64_t>::max)())
    {
        lifecycle_generation_ = 1U;
    }
    else
    {
        ++lifecycle_generation_;
    }
}

void ObfmLagGuidance::ClearLongitudinalStateForEmploy() noexcept
{
    snapshot_.previous_reference_point_valid = false;
    snapshot_.previous_reference_point_ned_m = Vector3{};
    snapshot_.previous_own_position_valid = false;
    snapshot_.previous_own_position_ned_m = Vector3{};
    snapshot_.previous_speed_command_valid = false;
    snapshot_.previous_speed_command_mps = 0.0;
    snapshot_.previous_frame_index = OptionalFrameIndex{};
    snapshot_.longitudinal_admitted_once = false;

    // Invalidate any uncommitted preparation without discarding the target
    // kinematic history that Python deliberately retains across EMPLOY.
    if (commit_count_ == (std::numeric_limits<std::uint64_t>::max)())
    {
        commit_count_ = 0U;
        if (lifecycle_generation_
            == (std::numeric_limits<std::uint64_t>::max)())
        {
            lifecycle_generation_ = 1U;
        }
        else
        {
            ++lifecycle_generation_;
        }
    }
    else
    {
        ++commit_count_;
    }
}

void ObfmLagGuidance::PrepareEmployHistoryCommit(
    const DogfightGeometryFrame& frame,
    ObfmEmployHistoryCommit& output,
    Status& status) const noexcept
{
    output = ObfmEmployHistoryCommit{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !FiniteVector(frame.opponent.velocity_ned_mps)
        || !std::isfinite(frame.t_sec))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    output.valid = true;
    output.frame_identity = frame.frame_identity;
    output.lifecycle_generation = lifecycle_generation_;
    output.base_commit_count = commit_count_;
    output.adversary_velocity_ned_mps =
        frame.opponent.velocity_ned_mps;
    output.time_s = frame.t_sec;
}

void ObfmLagGuidance::CommitEmployPublished(
    const ObfmEmployHistoryCommit& commit,
    Status& status) noexcept
{
    status = Status{};
    if (!commit.valid
        || !IsValidControlFrameIdentity(commit.frame_identity)
        || commit.lifecycle_generation != lifecycle_generation_
        || commit.base_commit_count != commit_count_
        || commit_count_
            == (std::numeric_limits<std::uint64_t>::max)()
        || !FiniteVector(commit.adversary_velocity_ned_mps)
        || !std::isfinite(commit.time_s))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_.previous_adversary_velocity_valid = true;
    snapshot_.previous_adversary_velocity_ned_mps =
        commit.adversary_velocity_ned_mps;
    snapshot_.previous_time_valid = true;
    snapshot_.previous_time_s = commit.time_s;
    ++commit_count_;
}

void ObfmLagGuidance::CopySnapshot(
    ObfmLagGuidanceSnapshot& output) const noexcept
{
    output = snapshot_;
}

void ObfmLagGuidance::Prepare(
    const DogfightGeometryFrame& frame,
    const OptionalFrameIndex& safety_frame_index,
    ObfmLagGuidancePreparation& output,
    Status& status) const noexcept
{
    output = ObfmLagGuidancePreparation{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !FiniteVector(frame.opponent.velocity_ned_mps)
        || !std::isfinite(frame.t_sec)
        || !std::isfinite(frame.own_offense.range_m)
        || !std::isfinite(frame.closing_speed_mps)
        || !std::isfinite(frame.own_offense.phase.min_range_m)
        || !std::isfinite(frame.own_offense.phase.max_range_m)
        || !FiniteSnapshot(snapshot_))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    Vector3 current_point{};
    ConcentricPathPoint(
        frame.own.position_ned_m,
        frame.opponent.position_ned_m,
        frame.opponent.velocity_ned_mps,
        frame.t_sec,
        snapshot_,
        current_point,
        status);
    if (status.code != StatusCode::Ok)
    {
        return;
    }

    const bool previous_index_incrementable =
        snapshot_.previous_frame_index.has_value
        && snapshot_.previous_frame_index.value
            != (std::numeric_limits<std::uint64_t>::max)();
    const bool same_episode = safety_frame_index.has_value
        && previous_index_incrementable
        && safety_frame_index.value
            == snapshot_.previous_frame_index.value + 1U
        && snapshot_.previous_time_valid
        && std::isfinite(snapshot_.previous_time_s)
        && frame.t_sec > snapshot_.previous_time_s
        && snapshot_.previous_reference_point_valid
        && snapshot_.previous_own_position_valid
        && snapshot_.previous_speed_command_valid;

    output.frame_identity = frame.frame_identity;
    output.lifecycle_generation = lifecycle_generation_;
    output.base_commit_count = commit_count_;
    output.safety_frame_index = safety_frame_index;
    output.current_reference_point_ned_m = current_point;
    output.same_reference_episode = same_episode;
    output.current_own_position_ned_m = frame.own.position_ned_m;
    output.current_own_velocity_ned_mps = frame.own.velocity_ned_mps;
    output.current_target_velocity_ned_mps =
        frame.opponent.velocity_ned_mps;
    output.current_time_s = frame.t_sec;
    output.current_range_m = frame.own_offense.range_m;
    output.range_rate_mps = -frame.closing_speed_mps;
    output.official_min_range_m = frame.own_offense.phase.min_range_m;
    output.official_max_range_m = frame.own_offense.phase.max_range_m;

    if (same_episode)
    {
        if (!snapshot_.previous_adversary_velocity_valid
            || snapshot_.previous_speed_command_mps <= 0.0)
        {
            output = ObfmLagGuidancePreparation{};
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        output.dt_s = frame.t_sec - snapshot_.previous_time_s;
        output.previous_time_s = snapshot_.previous_time_s;
        output.previous_reference_point_ned_m =
            snapshot_.previous_reference_point_ned_m;
        output.previous_speed_command_mps =
            snapshot_.previous_speed_command_mps;
        ConcentricPathPoint(
            snapshot_.previous_own_position_ned_m,
            frame.opponent.position_ned_m,
            frame.opponent.velocity_ned_mps,
            frame.t_sec,
            snapshot_,
            output.transported_reference_point_ned_m,
            status);
        if (status.code != StatusCode::Ok)
        {
            output = ObfmLagGuidancePreparation{};
            return;
        }
        output.transported_reference_point_valid = true;
    }

    output.valid = true;
}

void ObfmLagGuidance::BuildCandidate(
    const DogfightGeometryFrame& frame,
    const ObfmLagGuidancePreparation& preparation,
    const ObfmLagGuidanceInput& input,
    ControlIntent& output,
    ObfmLagGuidanceCommit& commit,
    Status& status) const noexcept
{
    output.Clear();
    commit = ObfmLagGuidanceCommit{};
    status = Status{};

    const bool phase_speed_selected = input.speed_authority
        == ObfmLagSpeedAuthority::PhaseLongitudinal;
    const bool station_speed_selected = input.speed_authority
        == ObfmLagSpeedAuthority::StationHold;

    // Visible ownership has already been decided by the parent BT. Calling
    // this leaf when a committed or higher-priority owner won is an integration
    // fault; do not synthesize a hidden fallback command.
    if (!input.ordinary_fallback_selected
        || input.behavior_id != DoctrineBehaviorId::Lag
        || input.writer_id != ControlIntentWriterObfmLag
        || (!phase_speed_selected && !station_speed_selected)
        || !preparation.valid
        || preparation.lifecycle_generation != lifecycle_generation_
        || preparation.base_commit_count != commit_count_
        || !IsValidControlFrameIdentity(frame.frame_identity)
        || !SameControlFrameIdentity(
            preparation.frame_identity,
            frame.frame_identity)
        || (phase_speed_selected
            && (!input.longitudinal.evaluated
                || !input.longitudinal.source_authoritative
                || !SameControlFrameIdentity(
                    input.longitudinal.frame_identity,
                    frame.frame_identity)
                || input.longitudinal.same_reference_episode
                    != preparation.same_reference_episode))
        || (station_speed_selected
            && (!input.station_hold.evaluated
                || !input.station_hold.desired_speed_mps.has_value
                || !SameControlFrameIdentity(
                    input.station_hold.frame_identity,
                    frame.frame_identity))))
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }
    if (!FinitePreparation(preparation)
        || (phase_speed_selected
            && (!FiniteOptional(input.longitudinal.desired_speed_mps)
                || !FiniteOptional(
                    input.longitudinal.desired_speed_rate_mps2)))
        || (station_speed_selected
            && (!FiniteOptional(input.station_hold.desired_speed_mps)
                || !FiniteOptional(
                    input.station_hold.desired_speed_rate_mps2))))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (phase_speed_selected && input.longitudinal.admitted
        && (!preparation.same_reference_episode
            || !input.longitudinal.desired_speed_mps.has_value
            || input.longitudinal.desired_speed_mps.value <= 0.0
            || !input.longitudinal.desired_speed_rate_mps2.has_value))
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }
    if (phase_speed_selected && !input.longitudinal.admitted
        && (input.longitudinal.desired_speed_mps.has_value
            || input.longitudinal.desired_speed_rate_mps2.has_value))
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }
    if (station_speed_selected
        && input.station_hold.desired_speed_mps.value <= 0.0)
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const double own_speed = Norm(frame.own.velocity_ned_mps);
    const double official_capture_range =
        frame.own_offense.phase.max_range_m;
    if (!std::isfinite(own_speed)
        || own_speed <= 0.0
        || !std::isfinite(official_capture_range)
        || official_capture_range <= 0.0)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    // The BT has already selected one speed authority.  Phase guidance may
    // retain the neutral current-speed echo on typed non-admission; station
    // guidance supplies its sole selected speed and a zero rate.  There is no
    // sequential overwrite or blending in this leaf.
    double desired_speed = own_speed;
    double desired_speed_rate = 0.0;
    if (phase_speed_selected && input.longitudinal.admitted)
    {
        desired_speed = input.longitudinal.desired_speed_mps.value;
        desired_speed_rate =
            input.longitudinal.desired_speed_rate_mps2.value;
    }
    else if (station_speed_selected)
    {
        desired_speed = input.station_hold.desired_speed_mps.value;
        desired_speed_rate =
            input.station_hold.desired_speed_rate_mps2.has_value
            ? input.station_hold.desired_speed_rate_mps2.value
            : 0.0;
    }

    ControlIntent candidate{};
    candidate.Clear();
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = preparation.current_reference_point_ned_m;
    candidate.desired_speed_mps = desired_speed;
    candidate.desired_speed_rate_mps2 = desired_speed_rate;
    candidate.capture_range_des_m = official_capture_range;
    candidate.behavior_id = input.behavior_id;
    candidate.mode_id = DoctrineModeId::Obfm;
    candidate.route_kind = ControlRouteKind::AimPoint;
    candidate.writer_id = input.writer_id;

    Status candidate_status{};
    candidate.Validate(candidate_status);
    if (!candidate_status.ok())
    {
        Fail(output, status, candidate_status.code);
        return;
    }

    output = candidate;
    commit.next_snapshot = snapshot_;
    commit.next_snapshot.previous_adversary_velocity_valid = true;
    commit.next_snapshot.previous_adversary_velocity_ned_mps =
        frame.opponent.velocity_ned_mps;
    commit.next_snapshot.previous_time_valid = true;
    commit.next_snapshot.previous_time_s = frame.t_sec;
    commit.next_snapshot.previous_reference_point_valid = true;
    commit.next_snapshot.previous_reference_point_ned_m =
        preparation.current_reference_point_ned_m;
    commit.next_snapshot.previous_own_position_valid = true;
    commit.next_snapshot.previous_own_position_ned_m =
        frame.own.position_ned_m;
    commit.next_snapshot.previous_speed_command_valid = true;
    commit.next_snapshot.previous_speed_command_mps = desired_speed;
    commit.next_snapshot.previous_frame_index =
        preparation.safety_frame_index;
    commit.next_snapshot.longitudinal_admitted_once =
        phase_speed_selected
        && preparation.same_reference_episode
        && input.longitudinal.admitted;
    commit.valid = true;
    commit.frame_identity = frame.frame_identity;
    commit.lifecycle_generation = lifecycle_generation_;
    commit.base_commit_count = commit_count_;
}

void ObfmLagGuidance::ValidatePublished(
    const ObfmLagGuidanceCommit& commit,
    Status& status) const noexcept
{
    status = Status{};
    if (!commit.valid
        || !IsValidControlFrameIdentity(commit.frame_identity)
        || commit.lifecycle_generation != lifecycle_generation_
        || commit.base_commit_count != commit_count_
        || commit_count_
            == (std::numeric_limits<std::uint64_t>::max)()
        || !FiniteSnapshot(commit.next_snapshot)
        || !commit.next_snapshot.previous_adversary_velocity_valid
        || !commit.next_snapshot.previous_time_valid
        || !commit.next_snapshot.previous_reference_point_valid
        || !commit.next_snapshot.previous_own_position_valid
        || !commit.next_snapshot.previous_speed_command_valid
        || commit.next_snapshot.previous_speed_command_mps <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
}

void ObfmLagGuidance::CommitPublished(
    const ObfmLagGuidanceCommit& commit,
    Status& status) noexcept
{
    ValidatePublished(commit, status);
    if (!status.ok())
    {
        return;
    }

    // The caller invokes this only after DoctrineBtContract::Publish succeeds
    // in the same serialized transaction. No earlier method mutates history.
    snapshot_ = commit.next_snapshot;
    ++commit_count_;
}

} // namespace LadyLuck
