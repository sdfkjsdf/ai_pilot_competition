#include "LadyLuck/guidance/committed/G16HighPrevention.hpp"

#include "LadyLuck/common/BoundedScaleProjection.hpp"
#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/control/route5/Route5Guidance.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;

constexpr double kMachineEpsilon = std::numeric_limits<double>::epsilon();
constexpr Vector3 kGravityNedMps2{{
    0.0, 0.0, LadyLuck::constants::StandardGravityMps2}};
constexpr Vector3 kNedUp{{0.0, 0.0, -1.0}};

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

Vector3 Add(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

Vector3 Subtract(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        scalar * value[0],
        scalar * value[1],
        scalar * value[2]}};
}

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
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

Vector3 VelocityNormal(
    const Vector3& value,
    const Vector3& velocity_hat) noexcept
{
    return Subtract(value, Scale(velocity_hat, Dot(value, velocity_hat)));
}

class G16HighTransverseScaleEvaluator final
{
public:
    G16HighTransverseScaleEvaluator(
        const Vector3& force_parallel,
        const Vector3& force_perpendicular,
        const Vector3& velocity_hat,
        const Vector3& desired_force,
        Vector3& evaluated_force) noexcept
        : force_parallel_(force_parallel),
          force_perpendicular_(force_perpendicular),
          velocity_hat_(velocity_hat),
          desired_force_(desired_force),
          evaluated_force_(evaluated_force)
    {
    }

    bool operator()(
        const double scale,
        double& transverse) const noexcept
    {
        evaluated_force_ = Add(
            force_parallel_,
            Scale(force_perpendicular_, scale));
        const double parallel_residual =
            Dot(evaluated_force_, velocity_hat_)
            - Dot(desired_force_, velocity_hat_);
        evaluated_force_ = Subtract(
            evaluated_force_,
            Scale(velocity_hat_, parallel_residual));
        const Vector3 recomputed_perpendicular = VelocityNormal(
            evaluated_force_,
            velocity_hat_);
        transverse = Norm(recomputed_perpendicular);
        return std::isfinite(transverse) && Finite(evaluated_force_);
    }

private:
    const Vector3& force_parallel_;
    const Vector3& force_perpendicular_;
    const Vector3& velocity_hat_;
    const Vector3& desired_force_;
    Vector3& evaluated_force_;
};

double ClipUnit(const double value) noexcept
{
    return (std::max)(-1.0, (std::min)(1.0, value));
}

double ScaledTolerance(
    const double a = 0.0,
    const double b = 0.0,
    const double c = 0.0,
    const double d = 0.0) noexcept
{
    const double scale = (std::max)(
        1.0,
        (std::max)(
            std::fabs(a),
            (std::max)(
                std::fabs(b),
                (std::max)(std::fabs(c), std::fabs(d)))));
    return 64.0 * kMachineEpsilon * scale;
}

bool HasCandidate(
    const LadyLuck::guidance::committed::G16HighCandidateMask mask,
    const LadyLuck::guidance::committed::G16HighCandidateMask candidate)
    noexcept
{
    return (static_cast<std::uint8_t>(mask)
        & static_cast<std::uint8_t>(candidate)) != 0U;
}

LadyLuck::guidance::committed::G16HighCandidateMask AddCandidate(
    const LadyLuck::guidance::committed::G16HighCandidateMask mask,
    const LadyLuck::guidance::committed::G16HighCandidateMask candidate)
    noexcept
{
    return static_cast<
        LadyLuck::guidance::committed::G16HighCandidateMask>(
            static_cast<std::uint8_t>(mask)
            | static_cast<std::uint8_t>(candidate));
}

bool ConcentricPathPoint(
    const LadyLuck::DogfightGeometryFrame& frame,
    const bool previous_velocity_valid,
    const Vector3& previous_velocity,
    const bool previous_time_valid,
    const double previous_time_s,
    Vector3& output) noexcept
{
    output = Vector3{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!Finite(own_position)
        || !Finite(adversary_position)
        || !Finite(adversary_velocity)
        || !std::isfinite(frame.t_sec))
    {
        return false;
    }
    const double adversary_speed = Norm(adversary_velocity);
    if (!std::isfinite(adversary_speed))
    {
        return false;
    }
    if (adversary_speed < LadyLuck::constants::Tiny)
    {
        output = adversary_position;
        return true;
    }
    const Vector3 path_direction =
        Scale(adversary_velocity, 1.0 / adversary_speed);
    const Vector3 offset = Subtract(own_position, adversary_position);
    const double foot_parameter = Dot(offset, path_direction);
    const Vector3 perpendicular =
        Subtract(offset, Scale(path_direction, foot_parameter));
    const double lag_depth = Norm(perpendicular);
    Vector3 omega{};
    if (previous_velocity_valid && previous_time_valid)
    {
        const double dt = frame.t_sec - previous_time_s;
        const double previous_speed = Norm(previous_velocity);
        if (dt > 0.0
            && std::isfinite(dt)
            && previous_speed >= LadyLuck::constants::Tiny)
        {
            const Vector3 previous_direction =
                Scale(previous_velocity, 1.0 / previous_speed);
            const Vector3 cross = Cross(previous_direction, path_direction);
            const double sine = Norm(cross);
            const double cosine = ClipUnit(
                Dot(previous_direction, path_direction));
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
        const Vector3 normal = Scale(omega, 1.0 / omega_magnitude);
        Vector3 centre_direction = Cross(omega, adversary_velocity);
        const double centre_direction_magnitude = Norm(centre_direction);
        if (!std::isfinite(radius)
            || !std::isfinite(centre_direction_magnitude)
            || centre_direction_magnitude == 0.0)
        {
            return false;
        }
        centre_direction =
            Scale(centre_direction, 1.0 / centre_direction_magnitude);
        const Vector3 centre = Add(
            adversary_position,
            Scale(centre_direction, radius));
        const double arc_angle = lag_depth / radius;
        const Vector3 spoke = Subtract(adversary_position, centre);
        const double cosine = std::cos(-arc_angle);
        const double sine = std::sin(-arc_angle);
        const Vector3 rotated = Add(
            Add(
                Scale(spoke, cosine),
                Scale(Cross(normal, spoke), sine)),
            Scale(normal, Dot(normal, spoke) * (1.0 - cosine)));
        output = Add(centre, rotated);
    }
    else
    {
        output = Subtract(
            adversary_position,
            Scale(path_direction, lag_depth));
    }
    return Finite(output);
}

bool ArcCaptureAcceleration(
    const Vector3& position,
    const Vector3& velocity,
    const Vector3& aim,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 velocity_hat{};
    const double speed = Norm(velocity);
    const Vector3 offset = Subtract(aim, position);
    const double distance = Norm(offset);
    Vector3 offset_hat{};
    if (!Unit(velocity, velocity_hat)
        || !Unit(offset, offset_hat)
        || !std::isfinite(speed)
        || !std::isfinite(distance)
        || speed <= 0.0
        || distance <= 0.0)
    {
        return false;
    }
    const Vector3 perpendicular = VelocityNormal(offset_hat, velocity_hat);
    const double perpendicular_norm = Norm(perpendicular);
    if (!std::isfinite(perpendicular_norm))
    {
        return false;
    }
    if (perpendicular_norm == 0.0)
    {
        return true;
    }
    const double magnitude =
        2.0 * speed * speed * perpendicular_norm / distance;
    output = Scale(perpendicular, magnitude / perpendicular_norm);
    return Finite(output);
}

bool EntrySafetyAdmitted(
    const LadyLuck::runtime::TacticalCommandBuildInput& input) noexcept
{
    const LadyLuck::control::route5::CommandEnvelope& envelope =
        input.current_envelope;
    const LadyLuck::safety::AutoGcasEntryReceipt& safety =
        input.current_safety;
    // d90 integrated/competition_adapter.py publishes this official flat
    // datum directly; it is not inferred from terrain or current GCAS state.
    constexpr double kOfficialCrashFloorM = 304.8;
    const double hard_deck_margin =
        -input.frame.own.position_ned_m[2] - kOfficialCrashFloorM;
    const double own_speed = Norm(input.frame.own.velocity_ned_mps);
    const bool resolved_stall_boundary = envelope.stall_speed_valid
        && envelope.stall_speed_source
            != LadyLuck::control::route5::StallSpeedBoundarySource::Unavailable
        && std::isfinite(envelope.stall_speed_mps)
        && envelope.stall_speed_mps > 0.0;
    // This gate decides whether the tactical High geometry may be evaluated.
    // A command-containment envelope is a downstream clamp, not evidence that
    // the aircraft can or cannot enter the maneuver.  Keep only current
    // safety/kinematic facts here; BuildEffectiveLoadAuthority supplies a
    // bounded command request independently.
    return safety.valid
        && std::isfinite(hard_deck_margin)
        && hard_deck_margin > 0.0
        && std::isfinite(own_speed)
        && own_speed > 0.0
        && (!resolved_stall_boundary
            || own_speed > envelope.stall_speed_mps);
}

bool ComputeManualSelection(
    const LadyLuck::runtime::TacticalCommandBuildInput& input,
    const LadyLuck::guidance::committed::G16PrecisionSpeedReceipt&
        precision_speed,
    const double effective_nz,
    LadyLuck::guidance::committed::G16HighCandidateMask& candidates,
    bool& resolved) noexcept
{
    using LadyLuck::guidance::committed::G16HighCandidateMask;
    candidates = G16HighCandidateMask::None;
    resolved = false;
    if (!EntrySafetyAdmitted(input)
        || !precision_speed.evaluated
        || !precision_speed.admitted
        || !LadyLuck::SameControlFrameIdentity(
            precision_speed.frame_identity,
            input.frame.frame_identity)
        || !std::isfinite(precision_speed.desired_speed_mps)
        || precision_speed.desired_speed_mps <= 0.0
        || !std::isfinite(effective_nz)
        || effective_nz <= 1.0)
    {
        return true;
    }
    const double own_speed = Norm(input.frame.own.velocity_ned_mps);
    if (!std::isfinite(own_speed) || own_speed <= 0.0)
    {
        return true;
    }
    const double precision_speed_error_mps =
        own_speed - precision_speed.desired_speed_mps;
    if (!std::isfinite(precision_speed_error_mps))
    {
        return false;
    }
    if (precision_speed_error_mps <= 0.0)
    {
        // The selected precision channel is not asking for deceleration, so
        // there is no excess kinetic energy for G16-P High to exchange for
        // altitude.  This is a resolved ordinary LAG outcome.
        resolved = true;
        return true;
    }
    // Tactical admission is the precision channel's current excess-speed
    // request.  The shared-climb builder and Route5/FCS own geometric
    // materialization and finite load clipping; target-relative closure,
    // aspect bands, and a second E-M radius proof must not veto the selected
    // v_cmd energy exchange.
    candidates = AddCandidate(candidates, G16HighCandidateMask::High);
    resolved = true;
    return true;
}

bool PreviousHighCommandApplied(
    const LadyLuck::runtime::TacticalCommandBuildInput& input) noexcept
{
    const auto& feedback = input.previous_control_feedback;
    return input.feedback_freshness
            == LadyLuck::runtime::TacticalFeedbackFreshness::Fresh
        && feedback.valid
        && feedback.writer_id
            == LadyLuck::ControlIntentWriterG16HighPrevention
        && feedback.behavior_id
            == LadyLuck::DoctrineBehaviorId::G16HighSharedClimbLoadVector;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

void G16HighPrevention::ResetExecution() noexcept
{
    phase_ = G16HighPhase::Idle;
    committed_candidates_ = G16HighCandidateMask::None;
    selection_committed_ = false;
    high_to_lag_consumed_ = false;
    shared_climb_plane_valid_ = false;
    shared_climb_plane_normal_ned_ = Vector3{};
    previous_target_velocity_valid_ = false;
    previous_target_velocity_ned_mps_ = Vector3{};
    previous_target_time_valid_ = false;
    previous_target_time_s_ = 0.0;
    cached_receipt_valid_ = false;
    cached_receipt_ = G16HighPreventionReceipt{};
}

void G16HighPrevention::Reset() noexcept
{
    observation_owner_.Reset();
    ResetExecution();
    cached_observation_valid_ = false;
    cached_observation_ = G16HighObservationReceipt{};
}

void G16HighPrevention::ResetForSafetyPreemption() noexcept
{
    // The global target-turn/apex Service has already observed this sample
    // before Root Safety selection.  Safety halts High command ownership and
    // its private execution/history without erasing that physical observer.
    HaltExecutionPreservingObservation();
}

void G16HighPrevention::HaltExecutionPreservingObservation() noexcept
{
    ResetExecution();
}

void G16HighPrevention::CaptureTransactionState(
    G16HighPreventionTransactionState& output) const noexcept
{
    output.phase = phase_;
    output.committed_candidates = committed_candidates_;
    output.selection_committed = selection_committed_;
    output.high_to_lag_consumed = high_to_lag_consumed_;
    output.shared_climb_plane_valid = shared_climb_plane_valid_;
    output.shared_climb_plane_normal_ned =
        shared_climb_plane_normal_ned_;
    output.previous_target_velocity_valid =
        previous_target_velocity_valid_;
    output.previous_target_velocity_ned_mps =
        previous_target_velocity_ned_mps_;
    output.previous_target_time_valid = previous_target_time_valid_;
    output.previous_target_time_s = previous_target_time_s_;
    output.cached_receipt_valid = cached_receipt_valid_;
    output.cached_receipt = cached_receipt_;
}

void G16HighPrevention::RestoreTransactionState(
    const G16HighPreventionTransactionState& input) noexcept
{
    phase_ = input.phase;
    committed_candidates_ = input.committed_candidates;
    selection_committed_ = input.selection_committed;
    high_to_lag_consumed_ = input.high_to_lag_consumed;
    shared_climb_plane_valid_ = input.shared_climb_plane_valid;
    shared_climb_plane_normal_ned_ =
        input.shared_climb_plane_normal_ned;
    previous_target_velocity_valid_ =
        input.previous_target_velocity_valid;
    previous_target_velocity_ned_mps_ =
        input.previous_target_velocity_ned_mps;
    previous_target_time_valid_ = input.previous_target_time_valid;
    previous_target_time_s_ = input.previous_target_time_s;
    cached_receipt_valid_ = input.cached_receipt_valid;
    cached_receipt_ = input.cached_receipt;
}

void G16HighPrevention::ObserveKinematics(
    const runtime::TacticalCommandBuildInput& input,
    G16HighObservationReceipt& output,
    Status& status) noexcept
{
    output = G16HighObservationReceipt{};
    status = Status{};
    observation_owner_.Observe(input, output, status);
    if (!status.ok())
    {
        return;
    }
    if (output.identity_restarted)
    {
        ResetExecution();
    }
    cached_observation_ = output;
    // ObservationInvalid/Seeded/FrameGap are accepted command-neutral
    // receipts.  Cache the evaluated identity even when no physical circle or
    // apex sample was admitted; only a negative API status is an internal
    // transaction failure.
    cached_observation_valid_ =
        IsValidControlFrameIdentity(output.frame_identity);
}

void G16HighPrevention::BuildEffectiveLoadAuthority(
    const runtime::TacticalCommandBuildInput& input,
    double& output,
    bool& valid) noexcept
{
    (void)input;
    output = 0.0;
    valid = false;
    // This is a requested-command bound, not a measured maneuver-capability
    // admission.  Route-5/FCS applies the current finite envelope afterwards;
    // a reduced or fallback envelope therefore shapes the request instead of
    // suppressing the High owner before it can publish.
    const double route5_limit =
        control::route5::Route5GuidanceConfig{}.n_cmd_max_g;
    if (!std::isfinite(route5_limit)
        || route5_limit <= 1.0)
    {
        return;
    }
    output = route5_limit;
    valid = std::isfinite(output) && output > 1.0;
    if (!valid)
    {
        output = 0.0;
    }
}

void G16HighPrevention::BuildGeometry(
    const runtime::TacticalCommandBuildInput& input,
    const double effective_nz_g,
    G16HighGeometryReceipt& output) const noexcept
{
    output = G16HighGeometryReceipt{};
    if (!cached_observation_valid_
        || !SameControlFrameIdentity(
            cached_observation_.frame_identity,
            input.frame.frame_identity)
        || !cached_observation_.established_turn.admitted)
    {
        return;
    }
    const G16EstablishedTurnCircleReceipt& circle =
        cached_observation_.established_turn;
    Vector3 own_course{};
    const Vector3& own_position = input.frame.own.position_ned_m;
    if (!Unit(input.frame.own.velocity_ned_mps, own_course))
    {
        return;
    }
    Vector3 projected_course = Subtract(
        own_course,
        Scale(circle.plane_normal_ned,
            Dot(own_course, circle.plane_normal_ned)));
    Vector3 projected_course_unit{};
    if (!Unit(projected_course, projected_course_unit))
    {
        return;
    }
    projected_course = projected_course_unit;
    Vector3 entry_radius{};
    if (!Unit(
            Cross(projected_course, circle.plane_normal_ned),
            entry_radius))
    {
        return;
    }
    const Vector3 entry_point = Add(
        circle.circle_centre_ned_m,
        Scale(entry_radius, circle.radius_m));
    const bool entry_ahead = Dot(
        Subtract(entry_point, own_position),
        own_course) > 0.0;
    const Vector3 window =
        Subtract(circle.circle_centre_ned_m, entry_point);
    Vector3 window_hat{};
    if (!Unit(window, window_hat))
    {
        return;
    }
    const double plane_offset = Dot(
        Subtract(own_position, entry_point),
        circle.plane_normal_ned);
    const Vector3 projected_own = Subtract(
        own_position,
        Scale(circle.plane_normal_ned, plane_offset));
    const Vector3 relative = Subtract(projected_own, entry_point);
    const double signed_tangent = Dot(relative, projected_course);
    const double radial_fraction = Dot(relative, window_hat) / Norm(window);
    const double intersection_distance = -signed_tangent;
    const Vector3 intersection = Add(
        projected_own,
        Scale(projected_course, intersection_distance));
    const bool intersection_ahead = intersection_distance > 0.0;
    const bool on_segment = radial_fraction >= 0.0
        && radial_fraction <= 1.0;
    const bool room = entry_ahead && intersection_ahead;

    output.evaluated = true;
    output.turn_path_room_available = room;
    output.entry_point_ahead = entry_ahead;
    output.course_intersection_ahead = intersection_ahead;
    output.course_intersection_on_window_segment = on_segment;
    output.plane_offset_m = plane_offset;
    output.entry_point_ned_m = entry_point;
    output.course_intersection_ned_m = intersection;
    if (!room || !on_segment)
    {
        output.direct_lag_reentry_resolved = true;
        output.direct_lag_reentry_admissible = false;
        return;
    }
    const double own_speed = Norm(input.frame.own.velocity_ned_mps);
    const G16TurnChordReceipt& recent = circle.recent_chord;
    const double target_speed_lower =
        circle.target_speed_mps - circle.target_speed_error_bound_mps;
    const double target_speed_upper =
        circle.target_speed_mps + circle.target_speed_error_bound_mps;
    if (!std::isfinite(effective_nz_g)
        || effective_nz_g <= 1.0
        || target_speed_lower <= 0.0
        || !recent.valid
        || recent.turn_rate_lower_radps <= 0.0
        || recent.turn_rate_upper_radps <= 0.0)
    {
        return;
    }
    const double own_radius_upper = own_speed * own_speed
        / (constants::StandardGravityMps2
            * std::sqrt(effective_nz_g * effective_nz_g - 1.0));
    const double target_radius_lower = target_speed_lower
        / recent.turn_rate_upper_radps;
    const double target_radius_upper = target_speed_upper
        / recent.turn_rate_lower_radps;
    if (!std::isfinite(own_radius_upper)
        || !std::isfinite(target_radius_lower)
        || !std::isfinite(target_radius_upper))
    {
        return;
    }
    output.radius_advantage_resolved = true;
    output.radius_advantage_lower_m =
        target_radius_lower - own_radius_upper;
    output.direct_lag_reentry_resolved = true;
    output.direct_lag_reentry_admissible =
        output.radius_advantage_lower_m > 0.0;
}

void G16HighPrevention::BuildSharedClimb(
    const runtime::TacticalCommandBuildInput& input,
    const G16PrecisionSpeedReceipt& precision_speed,
    const double effective_nz_g,
    G16HighReferenceReceipt& output) noexcept
{
    output = G16HighReferenceReceipt{};
    if (!precision_speed.evaluated
        || !precision_speed.admitted
        || !SameControlFrameIdentity(
            precision_speed.frame_identity,
            input.frame.frame_identity)
        || !std::isfinite(precision_speed.desired_speed_mps)
        || precision_speed.desired_speed_mps <= 0.0
        || !std::isfinite(effective_nz_g))
    {
        return;
    }
    output.evaluated = true;
    if (effective_nz_g <= 1.0)
    {
        output.physically_infeasible = true;
        return;
    }
    Vector3 velocity_hat{};
    if (!Unit(input.frame.own.velocity_ned_mps, velocity_hat))
    {
        return;
    }
    // Precision-speed High is an energy exchange, not a target-turn
    // classification.  NED-up projected into the own velocity-normal plane is
    // the unique climb axis for a non-vertical flight path.
    shared_climb_plane_valid_ = true;
    shared_climb_plane_normal_ned_ = kNedUp;
    Vector3 out_direction = VelocityNormal(kNedUp, velocity_hat);
    const double out_norm = Norm(out_direction);
    const double out_tolerance = ScaledTolerance(out_norm);
    if (!std::isfinite(out_norm) || out_norm <= out_tolerance)
    {
        return;
    }
    out_direction = Scale(out_direction, 1.0 / out_norm);
    const double upward_alignment = Dot(out_direction, kNedUp);
    if (std::fabs(upward_alignment) <= out_tolerance)
    {
        return;
    }
    if (upward_alignment < 0.0)
    {
        out_direction = Scale(out_direction, -1.0);
    }
    Vector3 raw_lag{};
    Vector3 lag_point{};
    if (ConcentricPathPoint(
            input.frame,
            previous_target_velocity_valid_,
            previous_target_velocity_ned_mps_,
            previous_target_time_valid_,
            previous_target_time_s_,
            lag_point))
    {
        Vector3 evaluated_lag{};
        if (ArcCaptureAcceleration(
                input.frame.own.position_ned_m,
                input.frame.own.velocity_ned_mps,
                lag_point,
                evaluated_lag))
        {
            raw_lag = VelocityNormal(evaluated_lag, velocity_hat);
        }
    }
    const Vector3 same_plane = Subtract(
        raw_lag,
        Scale(out_direction, Dot(raw_lag, out_direction)));
    const double raw_required = Norm(same_plane);
    output.climb_axis_raw_required_accel_mps2 = raw_required;
    if (!std::isfinite(raw_required))
    {
        return;
    }
    const double own_speed = Norm(input.frame.own.velocity_ned_mps);
    const double capture_range = input.frame.own_offense.phase.max_range_m;
    if (!std::isfinite(own_speed)
        || own_speed <= 0.0
        || !std::isfinite(capture_range)
        || capture_range <= 0.0)
    {
        return;
    }
    const double cap =
        effective_nz_g * constants::StandardGravityMps2;
    Vector3 force = Add(same_plane, Scale(out_direction, cap));
    const double raw_force = Norm(force);
    if (!std::isfinite(cap)
        || cap <= 0.0
        || !std::isfinite(raw_force)
        || raw_force <= 0.0)
    {
        output.physically_infeasible = true;
        return;
    }
    if (raw_force > cap)
    {
        force = Scale(force, cap / raw_force);
    }
    const double load = Norm(force) / constants::StandardGravityMps2;
    if (!std::isfinite(load)
        || load <= 0.0
        || load > effective_nz_g
            + ScaledTolerance(load, effective_nz_g))
    {
        output.physically_infeasible = true;
        return;
    }
    const double speed_energy_height_m = (std::max)(
        0.0,
        (own_speed * own_speed
            - precision_speed.desired_speed_mps
                * precision_speed.desired_speed_mps)
            / (2.0 * constants::StandardGravityMps2));
    if (!std::isfinite(speed_energy_height_m)
        || speed_energy_height_m <= 0.0)
    {
        return;
    }
    const Vector3 climb_aim = Add(
        Add(
            input.frame.own.position_ned_m,
            Scale(velocity_hat, capture_range)),
        Scale(kNedUp, speed_energy_height_m));
    output.available = true;
    output.aim_point_ned_m = climb_aim;
    output.transverse_specific_force_ned_mps2 = force;
    output.acceleration_ned_mps2 = Add(kGravityNedMps2, force);
    output.requested_load_g = load;
    output.effective_load_limit_g = effective_nz_g;
    output.climb_axis_required_accel_mps2 = Dot(force, out_direction);
}

void G16HighPrevention::BuildRollIn(
    const runtime::TacticalCommandBuildInput& input,
    const double effective_nz_g,
    G16HighReferenceReceipt& output) const noexcept
{
    output = G16HighReferenceReceipt{};
    output.evaluated = true;
    if (!std::isfinite(effective_nz_g) || effective_nz_g <= 1.0)
    {
        return;
    }
    Vector3 aim{};
    if (!ConcentricPathPoint(
            input.frame,
            previous_target_velocity_valid_,
            previous_target_velocity_ned_mps_,
            previous_target_time_valid_,
            previous_target_time_s_,
            aim))
    {
        return;
    }
    Vector3 raw{};
    if (!ArcCaptureAcceleration(
            input.frame.own.position_ned_m,
            input.frame.own.velocity_ned_mps,
            aim,
            raw))
    {
        return;
    }
    Vector3 velocity_hat{};
    if (!Unit(input.frame.own.velocity_ned_mps, velocity_hat))
    {
        return;
    }
    const double cap =
        effective_nz_g * constants::StandardGravityMps2;
    const Vector3 desired_force = Subtract(raw, kGravityNedMps2);
    const Vector3 force_parallel =
        Scale(velocity_hat, Dot(desired_force, velocity_hat));
    const Vector3 force_perpendicular =
        Subtract(desired_force, force_parallel);
    const double raw_transverse = Norm(force_perpendicular);
    Vector3 admitted_acceleration = raw;
    if (!std::isfinite(raw_transverse))
    {
        return;
    }
    if (raw_transverse >= cap)
    {
        const double initial_scale = std::nextafter(cap, 0.0)
            / raw_transverse;
        Vector3 evaluated_force{};
        const G16HighTransverseScaleEvaluator evaluate_scale{
            force_parallel,
            force_perpendicular,
            velocity_hat,
            desired_force,
            evaluated_force};
        double admitted_scale = 0.0;
        double admitted_transverse = 0.0;
        if (!LadyLuck::common::LargestRepresentableScaleBelowBound(
                initial_scale,
                cap,
                evaluate_scale,
                admitted_scale,
                admitted_transverse)
            || !evaluate_scale(admitted_scale, admitted_transverse)
            || !(admitted_transverse < cap))
        {
            output.physically_infeasible = true;
            return;
        }
        admitted_acceleration = Add(kGravityNedMps2, evaluated_force);
    }
    const Vector3 admitted_specific_force =
        Subtract(admitted_acceleration, kGravityNedMps2);
    const Vector3 transverse_force =
        VelocityNormal(admitted_specific_force, velocity_hat);
    const double force_magnitude = Norm(transverse_force);
    if (!std::isfinite(force_magnitude) || force_magnitude <= 0.0)
    {
        output.physically_infeasible = true;
        return;
    }
    const Vector3 total_acceleration =
        Add(kGravityNedMps2, transverse_force);
    Vector3 target_direction{};
    Vector3 aim_direction{};
    if (!Unit(
            VelocityNormal(
                Subtract(
                    input.frame.opponent.position_ned_m,
                    input.frame.own.position_ned_m),
                velocity_hat),
            target_direction)
        || !Unit(
            VelocityNormal(
                Subtract(aim, input.frame.own.position_ned_m),
                velocity_hat),
            aim_direction)
        || Dot(transverse_force, target_direction) <= 0.0
        || Dot(
            VelocityNormal(total_acceleration, velocity_hat),
            aim_direction) <= 0.0)
    {
        output.physically_infeasible = true;
        return;
    }
    const double load =
        force_magnitude / constants::StandardGravityMps2;
    if (load > effective_nz_g + ScaledTolerance(load, effective_nz_g))
    {
        output.physically_infeasible = true;
        return;
    }
    output.available = true;
    output.aim_point_ned_m = aim;
    output.acceleration_ned_mps2 = total_acceleration;
    output.transverse_specific_force_ned_mps2 = transverse_force;
    output.requested_load_g = load;
    output.effective_load_limit_g = effective_nz_g;
}

void G16HighPrevention::BuildIntent(
    const runtime::TacticalCommandBuildInput& input,
    const G16PrecisionSpeedReceipt& precision_speed,
    const G16HighReferenceReceipt& reference,
    const bool inversion_allowed,
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    const double own_speed = Norm(input.frame.own.velocity_ned_mps);
    const double capture_range = input.frame.own_offense.phase.max_range_m;
    double gamma_limit =
        control::route5::Route5GuidanceConfig{}.gamma_cmd_limit_rad;
    const bool gamma_metadata_present =
        input.current_longitudinal_evidence.valid
        && input.current_longitudinal_evidence
            .flight_path_gamma_limit_valid;
    if (gamma_metadata_present)
    {
        gamma_limit = input.current_longitudinal_evidence
            .flight_path_gamma_limit_rad;
    }
    const Vector3 path = Subtract(
        reference.aim_point_ned_m,
        input.frame.own.position_ned_m);
    const double path_norm = Norm(path);
    if (!reference.available
        || !precision_speed.evaluated
        || !precision_speed.admitted
        || !SameControlFrameIdentity(
            precision_speed.frame_identity,
            input.frame.frame_identity)
        || !std::isfinite(precision_speed.desired_speed_mps)
        || precision_speed.desired_speed_mps <= 0.0
        || !Finite(reference.acceleration_ned_mps2)
        || !std::isfinite(own_speed)
        || own_speed <= 0.0
        || !std::isfinite(capture_range)
        || capture_range <= 0.0
        || !std::isfinite(gamma_limit)
        || gamma_limit <= 0.0
        || gamma_limit >= 0.5 * constants::Pi
        || !std::isfinite(path_norm)
        || path_norm <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    Vector3 velocity_hat{};
    if (!Unit(input.frame.own.velocity_ned_mps, velocity_hat))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    Vector3 specific_force =
        Subtract(reference.acceleration_ned_mps2, kGravityNedMps2);
    const double parallel = Dot(specific_force, velocity_hat);
    if (!std::isfinite(parallel))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    // Preserve the finite load request and remove only the component parallel
    // to velocity. Binary64 projection residue is not a reason to discard the
    // whole High command.
    specific_force = Subtract(
        specific_force,
        Scale(velocity_hat, parallel));
    const Vector3 projected_acceleration =
        Add(kGravityNedMps2, specific_force);
    if (!Finite(specific_force) || !Finite(projected_acceleration))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double raw_gamma = std::atan2(
        -path[2],
        std::hypot(path[0], path[1]));
    const double admitted_gamma = (std::max)(
        -gamma_limit,
        (std::min)(gamma_limit, raw_gamma));
    ControlIntent candidate{};
    candidate.Clear();
    candidate.frame_identity = input.frame.frame_identity;
    candidate.aim_point_m = reference.aim_point_ned_m;
    candidate.desired_speed_mps = precision_speed.desired_speed_mps;
    candidate.desired_speed_rate_mps2 =
        -constants::StandardGravityMps2 * std::sin(admitted_gamma);
    candidate.path_inversion_allowed.has_value = true;
    candidate.path_inversion_allowed.value = inversion_allowed;
    candidate.capture_range_des_m = capture_range;
    candidate.total_load_factor_limit_g.has_value = true;
    candidate.total_load_factor_limit_g.value =
        reference.effective_load_limit_g;
    candidate.direct_load_vector_acceleration_ned_mps2.has_value = true;
    candidate.direct_load_vector_acceleration_ned_mps2.value =
        projected_acceleration;
    candidate.behavior_id =
        DoctrineBehaviorId::G16HighSharedClimbLoadVector;
    candidate.mode_id = DoctrineModeId::Obfm;
    candidate.route_kind = ControlRouteKind::DirectLoadVectorAcceleration;
    candidate.writer_id = ControlIntentWriterG16HighPrevention;
    candidate.Validate(status);
    if (!status.ok())
    {
        return;
    }
    output = candidate;
}

void G16HighPrevention::AdvanceLifecycle(
    const G16ProductionEvidenceReceipt& evidence,
    const G16HighCandidateMask current_candidates,
    const bool selection_resolved,
    const bool current_speed_excess_resolved,
    const bool current_speed_excess,
    const G16HighGeometryReceipt& geometry,
    const G16HighReferenceReceipt& shared_climb,
    const G16HighReferenceReceipt& roll_in,
    const bool apex_resolved,
    const bool apex_crossed,
    const bool roll_complete_resolved,
    const bool roll_complete,
    G16HighReason& reason) noexcept
{
    if (evidence.handoff_status == G16HandoffStatus::Requested)
    {
        phase_ = G16HighPhase::G16EHandoff;
        reason = G16HighReason::G16EHandoff;
        return;
    }
    if (phase_ == G16HighPhase::G16EHandoff
        || phase_ == G16HighPhase::HighObfmLagHandoff
        || phase_ == G16HighPhase::UnsupportedDisplacement)
    {
        return;
    }
    if (phase_ != G16HighPhase::Idle
        && current_speed_excess_resolved
        && !current_speed_excess)
    {
        // High owns altitude exchange only while the current precision speed
        // channel requests deceleration. Return immediately to writer 5 once
        // Vown-v_cmd is no longer positive.
        phase_ = G16HighPhase::HighObfmLagHandoff;
        reason = G16HighReason::HighToLagHandoff;
        return;
    }
    switch (phase_)
    {
    case G16HighPhase::Idle:
        if (selection_resolved
            && HasCandidate(
                current_candidates,
                G16HighCandidateMask::High))
        {
            committed_candidates_ = current_candidates;
            selection_committed_ = true;
            if (shared_climb.available)
            {
                phase_ = G16HighPhase::SharedClimb;
                reason = G16HighReason::SharedClimbCommand;
            }
            else
            {
                reason = G16HighReason::SharedClimbReferenceUnavailable;
            }
        }
        else
        {
            reason = G16HighReason::ManualSelectionNotHigh;
        }
        return;
    case G16HighPhase::SharedClimb:
    case G16HighPhase::SharedClimbReferenceWait:
    {
        const bool high_event = geometry.direct_lag_reentry_resolved
            && geometry.direct_lag_reentry_admissible;
        if (!high_event && !apex_crossed)
        {
            if (phase_ == G16HighPhase::SharedClimbReferenceWait)
            {
                if (shared_climb.available)
                {
                    phase_ = G16HighPhase::SharedClimb;
                    reason = G16HighReason::SharedClimbCommand;
                }
            }
            else if (!shared_climb.available)
            {
                phase_ = G16HighPhase::SharedClimbReferenceWait;
                reason = G16HighReason::SharedClimbReferenceUnavailable;
            }
            return;
        }
        if (high_event
            && HasCandidate(
                committed_candidates_,
                G16HighCandidateMask::High))
        {
            if (roll_in.available)
            {
                phase_ = apex_crossed
                    ? G16HighPhase::HighPostApexRollIn
                    : G16HighPhase::HighRollIn;
                reason = G16HighReason::HighRollInCommand;
            }
            else
            {
                phase_ = apex_crossed
                    ? G16HighPhase::HighPostApexRollInCapabilityWait
                    : G16HighPhase::HighRollInCapabilityWait;
                reason = G16HighReason::HighRollInReferenceUnavailable;
            }
            return;
        }
        if (!geometry.direct_lag_reentry_resolved)
        {
            return;
        }
        if (apex_crossed
            && !geometry.direct_lag_reentry_admissible
            && HasCandidate(
                committed_candidates_,
                G16HighCandidateMask::Displacement))
        {
            phase_ = G16HighPhase::UnsupportedDisplacement;
            reason = G16HighReason::DisplacementOwnerNotInThisModule;
            return;
        }
        if (apex_crossed
            && committed_candidates_ == G16HighCandidateMask::High)
        {
            phase_ = roll_in.available
                ? G16HighPhase::HighPostApexRollIn
                : G16HighPhase::HighPostApexRollInCapabilityWait;
            reason = roll_in.available
                ? G16HighReason::HighRollInCommand
                : G16HighReason::HighRollInReferenceUnavailable;
        }
        return;
    }
    case G16HighPhase::HighRollInCapabilityWait:
    case G16HighPhase::HighPostApexRollInCapabilityWait:
    {
        const bool post_apex =
            phase_ == G16HighPhase::HighPostApexRollInCapabilityWait
            || apex_crossed;
        if (roll_in.available)
        {
            phase_ = post_apex
                ? G16HighPhase::HighPostApexRollIn
                : G16HighPhase::HighRollIn;
            reason = G16HighReason::HighRollInCommand;
        }
        return;
    }
    case G16HighPhase::HighRollIn:
        if (!roll_in.available)
        {
            phase_ = apex_crossed
                ? G16HighPhase::HighPostApexRollInCapabilityWait
                : G16HighPhase::HighRollInCapabilityWait;
            reason = G16HighReason::HighRollInReferenceUnavailable;
            return;
        }
        if (apex_crossed
            && geometry.direct_lag_reentry_resolved
            && !geometry.direct_lag_reentry_admissible
            && HasCandidate(
                committed_candidates_,
                G16HighCandidateMask::Displacement))
        {
            phase_ = G16HighPhase::UnsupportedDisplacement;
            reason = G16HighReason::DisplacementOwnerNotInThisModule;
            return;
        }
        if (!apex_resolved || !apex_crossed)
        {
            return;
        }
        if (roll_complete_resolved && roll_complete)
        {
            phase_ = G16HighPhase::HighObfmLagHandoff;
            reason = G16HighReason::HighToLagHandoff;
        }
        else
        {
            phase_ = G16HighPhase::HighPostApexRollIn;
        }
        return;
    case G16HighPhase::HighPostApexRollIn:
        if (!roll_in.available)
        {
            phase_ = G16HighPhase::HighPostApexRollInCapabilityWait;
            reason = G16HighReason::HighRollInReferenceUnavailable;
        }
        else if (roll_complete_resolved && roll_complete)
        {
            phase_ = G16HighPhase::HighObfmLagHandoff;
            reason = G16HighReason::HighToLagHandoff;
        }
        return;
    case G16HighPhase::HighObfmLagHandoff:
    case G16HighPhase::G16EHandoff:
    case G16HighPhase::UnsupportedDisplacement:
        return;
    }
}

void G16HighPrevention::Evaluate(
    const runtime::TacticalCommandBuildInput& input,
    const G16ProductionEvidenceReceipt& evidence,
    const G16PrecisionSpeedReceipt& precision_speed,
    G16HighPreventionReceipt& output,
    Status& status) noexcept
{
    output = G16HighPreventionReceipt{};
    status = Status{};
    if (!input.valid || !IsValidControlFrameIdentity(input.frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }
    if (cached_receipt_valid_
        && SameControlFrameIdentity(
            cached_receipt_.frame_identity,
            input.frame.frame_identity))
    {
        output = cached_receipt_;
        return;
    }
    G16HighPreventionTransactionState transaction_at_entry{};
    CaptureTransactionState(transaction_at_entry);
    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    output.precision_speed = precision_speed;
    output.phase_before = phase_;
    output.phase_after = phase_;
    output.reason = G16HighReason::TransactionUnavailable;
    if (!evidence.valid)
    {
        cached_receipt_ = output;
        cached_receipt_valid_ = true;
        return;
    }
    if (!SameControlFrameIdentity(
            evidence.frame_identity,
            input.frame.frame_identity)
        || !SameControlFrameIdentity(
            evidence.frame.frame_identity,
            input.frame.frame_identity)
        || !cached_observation_valid_
        || !SameControlFrameIdentity(
            cached_observation_.frame_identity,
            input.frame.frame_identity))
    {
        RestoreTransactionState(transaction_at_entry);
        output = G16HighPreventionReceipt{};
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!precision_speed.evaluated
        || !SameControlFrameIdentity(
            precision_speed.frame_identity,
            input.frame.frame_identity)
        || (precision_speed.admitted
            && (!std::isfinite(precision_speed.desired_speed_mps)
                || precision_speed.desired_speed_mps <= 0.0
                || !std::isfinite(
                    precision_speed.desired_speed_rate_mps2))))
    {
        RestoreTransactionState(transaction_at_entry);
        output = G16HighPreventionReceipt{};
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.source_simultaneous = evidence.source_simultaneous
        && cached_observation_.source_simultaneous;
    output.observation = cached_observation_;
    if (!output.source_simultaneous)
    {
        previous_target_velocity_valid_ = false;
        previous_target_velocity_ned_mps_ = Vector3{};
        previous_target_time_valid_ = false;
        previous_target_time_s_ = 0.0;
        cached_receipt_ = output;
        cached_receipt_valid_ = true;
        return;
    }
    double effective_nz = 0.0;
    bool effective_nz_valid = false;
    BuildEffectiveLoadAuthority(input, effective_nz, effective_nz_valid);
    output.effective_nz_valid = effective_nz_valid;
    output.effective_nz_limit_g = effective_nz;
    if (effective_nz_valid)
    {
        BuildGeometry(input, effective_nz, output.geometry);
    }
    G16HighCandidateMask current_candidates =
        G16HighCandidateMask::None;
    bool selection_resolved = false;
    if (selection_committed_)
    {
        current_candidates = committed_candidates_;
        selection_resolved = true;
    }
    else if (effective_nz_valid
        && !ComputeManualSelection(
                 input,
                 precision_speed,
                 effective_nz,
                 current_candidates,
                 selection_resolved))
    {
        RestoreTransactionState(transaction_at_entry);
        output = G16HighPreventionReceipt{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const double current_own_speed = Norm(input.frame.own.velocity_ned_mps);
    if (!std::isfinite(current_own_speed) || current_own_speed <= 0.0)
    {
        RestoreTransactionState(transaction_at_entry);
        output = G16HighPreventionReceipt{};
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    const bool current_speed_excess_resolved = precision_speed.evaluated;
    const bool current_speed_excess = precision_speed.admitted
        && current_own_speed > precision_speed.desired_speed_mps;
    output.precision_speed = precision_speed;
    const bool shared_service_active = phase_ == G16HighPhase::SharedClimb
        || phase_ == G16HighPhase::SharedClimbReferenceWait
        || (phase_ == G16HighPhase::Idle
            && selection_resolved
            && current_candidates != G16HighCandidateMask::None);
    if (shared_service_active)
    {
        BuildSharedClimb(
            input,
            precision_speed,
            effective_nz,
            output.shared_climb);
    }
    BuildRollIn(input, effective_nz, output.roll_in);
    output.previous_high_command_applied =
        PreviousHighCommandApplied(input);
    const bool apex_resolved = cached_observation_.apex.evaluated;
    const bool apex_crossed = cached_observation_.apex.apex_crossed;
    const bool roll_complete_resolved =
        output.previous_high_command_applied
        && cached_observation_.roll_in.high_roll_in_complete_resolved;
    const bool roll_complete = roll_complete_resolved
        && cached_observation_.roll_in.high_roll_in_complete;
    G16HighReason reason = !effective_nz_valid
        ? G16HighReason::EffectiveLoadAuthorityUnavailable
        : selection_resolved
        ? G16HighReason::ManualSelectionNotHigh
        : G16HighReason::EntryGeometryUnavailable;
    AdvanceLifecycle(
        evidence,
        current_candidates,
        selection_resolved,
        current_speed_excess_resolved,
        current_speed_excess,
        output.geometry,
        output.shared_climb,
        output.roll_in,
        apex_resolved,
        apex_crossed,
        roll_complete_resolved,
        roll_complete,
        reason);
    output.phase_after = phase_;
    output.committed_candidates = committed_candidates_;
    output.selection_committed = selection_committed_;
    output.reason = reason;
    if (phase_ == G16HighPhase::SharedClimb
        && output.shared_climb.available)
    {
        BuildIntent(
            input,
            precision_speed,
            output.shared_climb,
            false,
            output.candidate,
            status);
        output.reference_role = G16HighReferenceRole::SharedClimb;
    }
    else if ((phase_ == G16HighPhase::HighRollIn
            || phase_ == G16HighPhase::HighPostApexRollIn)
        && output.roll_in.available)
    {
        BuildIntent(
            input,
            precision_speed,
            output.roll_in,
            true,
            output.candidate,
            status);
        output.reference_role = G16HighReferenceRole::HighRollIn;
    }
    else if (phase_ == G16HighPhase::HighObfmLagHandoff)
    {
        output.reference_role = G16HighReferenceRole::ObfmLagHandoff;
        output.high_to_lag.valid = true;
        output.high_to_lag.frame_identity = input.frame.frame_identity;
        output.high_to_lag.selected_this_sample = true;
        output.high_to_lag.consumed = high_to_lag_consumed_;
        output.high_to_lag.production_evidence = evidence;
    }
    else if (phase_ == G16HighPhase::G16EHandoff)
    {
        output.reference_role = G16HighReferenceRole::G16EHandoff;
    }
    else if (phase_ == G16HighPhase::UnsupportedDisplacement)
    {
        output.reference_role = G16HighReferenceRole::Unsupported;
    }
    if (!status.ok())
    {
        RestoreTransactionState(transaction_at_entry);
        output = G16HighPreventionReceipt{};
        return;
    }
    output.command_ready = output.candidate.writer_id
        == ControlIntentWriterG16HighPrevention;
    output.selected = output.command_ready
        || output.reference_role == G16HighReferenceRole::ObfmLagHandoff
        || output.reference_role == G16HighReferenceRole::G16EHandoff;
    previous_target_velocity_valid_ = true;
    previous_target_velocity_ned_mps_ =
        input.frame.opponent.velocity_ned_mps;
    previous_target_time_valid_ = true;
    previous_target_time_s_ = input.frame.t_sec;
    cached_receipt_ = output;
    cached_receipt_valid_ = true;
}

void G16HighPrevention::CopySelection(
    const ControlFrameIdentity& current_identity,
    G16HighSelection& output,
    Status& status) const noexcept
{
    output = G16HighSelection{};
    status = Status{};
    if (!cached_receipt_valid_
        || !IsValidControlFrameIdentity(current_identity)
        || !SameControlFrameIdentity(
            cached_receipt_.frame_identity,
            current_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output.valid = true;
    output.frame_identity = current_identity;
    output.command_selected = cached_receipt_.command_ready;
    output.high_to_lag_selected =
        cached_receipt_.reference_role
            == G16HighReferenceRole::ObfmLagHandoff
        && !cached_receipt_.high_to_lag.consumed;
    output.g16_e_handoff = cached_receipt_.reference_role
        == G16HighReferenceRole::G16EHandoff;
    output.writer_id = output.command_selected
        ? ControlIntentWriterG16HighPrevention
        : ControlIntentWriterNone;
    output.phase = cached_receipt_.phase_after;
}

void G16HighPrevention::BuildCandidate(
    const ControlFrameIdentity& current_identity,
    ControlIntent& output,
    Status& status) const noexcept
{
    output.Clear();
    status = Status{};
    if (!cached_receipt_valid_
        || !IsValidControlFrameIdentity(current_identity)
        || !SameControlFrameIdentity(
            cached_receipt_.frame_identity,
            current_identity)
        || !cached_receipt_.command_ready)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output = cached_receipt_.candidate;
    output.Validate(status);
}

void G16HighPrevention::ConsumeHighToLagHandoff(
    const ControlFrameIdentity& current_identity,
    G16HighToLagHandoff& output,
    Status& status) noexcept
{
    output = G16HighToLagHandoff{};
    status = Status{};
    if (!cached_receipt_valid_
        || !IsValidControlFrameIdentity(current_identity)
        || !SameControlFrameIdentity(
            cached_receipt_.frame_identity,
            current_identity)
        || cached_receipt_.reference_role
            != G16HighReferenceRole::ObfmLagHandoff
        || high_to_lag_consumed_)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    output = cached_receipt_.high_to_lag;
    output.consumed = true;
    high_to_lag_consumed_ = true;
    cached_receipt_.high_to_lag.consumed = true;
}

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
