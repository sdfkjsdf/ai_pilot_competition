#include "LadyLuck/guidance/g4/HighGBarrelOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::ControlIntent;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g4::HighGBarrelDecision;
using LadyLuck::guidance::g4::HighGBarrelExactEvidence;
using LadyLuck::guidance::g4::HighGBarrelOwnerSnapshot;
using LadyLuck::guidance::g4::HighGBarrelReason;
using LadyLuck::guidance::g4::HighGBarrelSelectionReceipt;
using LadyLuck::guidance::g4::HighGBarrelTaskReceipt;
using LadyLuck::guidance::g4::HighGBarrelVariant;
using LadyLuck::runtime::TacticalCommandBuildInput;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

double ScaledTolerance(
    const double first,
    const double second = 0.0,
    const double third = 0.0,
    const double fourth = 0.0) noexcept
{
    const double scale = (std::max)(
        1.0,
        (std::max)(
            std::fabs(first),
            (std::max)(
                std::fabs(second),
                (std::max)(std::fabs(third), std::fabs(fourth)))));
    return 64.0 * (std::numeric_limits<double>::epsilon)() * scale;
}

bool Unit3(const Vector3& input, Vector3& output) noexcept
{
    output = Vector3{};
    if (!FiniteVector(input))
    {
        return false;
    }
    const double magnitude = Norm3(input);
    if (!std::isfinite(magnitude)
        || magnitude <= (std::numeric_limits<double>::epsilon)())
    {
        return false;
    }
    output = Vector3{{
        input[0] / magnitude,
        input[1] / magnitude,
        input[2] / magnitude}};
    return FiniteVector(output);
}

bool VelocityNormalDirection(
    const Vector3& direction,
    const Vector3& velocity_hat,
    Vector3& output) noexcept
{
    output = Vector3{};
    Vector3 velocity{};
    if (!Unit3(velocity_hat, velocity) || !FiniteVector(direction))
    {
        return false;
    }
    const double parallel = Dot3(direction, velocity);
    const Vector3 projected{{
        direction[0] - parallel * velocity[0],
        direction[1] - parallel * velocity[1],
        direction[2] - parallel * velocity[2]}};
    const double magnitude = Norm3(projected);
    if (!std::isfinite(magnitude)
        || magnitude <= ScaledTolerance(magnitude))
    {
        return false;
    }
    output = Vector3{{
        projected[0] / magnitude,
        projected[1] / magnitude,
        projected[2] / magnitude}};
    return FiniteVector(output);
}

bool CurrentSafetyAndCommandLoad(
    const TacticalCommandBuildInput& input,
    double& load_g) noexcept
{
    load_g = 0.0;
    const double own_speed = Norm3(input.frame.own.velocity_ned_mps);
    // Root selection has already established that Auto-GCAS did not own this
    // tick. Rechecking GCAS observation bits here can only interrupt an
    // admitted gun escape without transferring command authority to GCAS.
    if (!std::isfinite(own_speed)
        || own_speed <= LadyLuck::constants::Tiny)
    {
        return false;
    }
    // The downstream Direct-NED shaper is the single physical-envelope
    // authority.  A transiently unavailable optional envelope must not switch
    // an active gun escape back to writer 2.  Reuse the root Gun BREAK's
    // established one-g bounded baseline until a positive current load is
    // available; the shaper still clips every realized body/Nz reference.
    load_g = LadyLuck::runtime::CurrentCommandEnvelopeAvailable(input)
            && std::isfinite(input.current_envelope.nz_feasible_g)
            && input.current_envelope.nz_feasible_g > 0.0
        ? input.current_envelope.nz_feasible_g
        : 1.0;
    return true;
}

bool BuildWezShortestExitDirection(
    const TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    Vector3& lift_direction,
    Vector3& enemy_to_own_hat,
    double& outward_alignment) noexcept
{
    lift_direction = Vector3{};
    enemy_to_own_hat = Vector3{};
    outward_alignment = 0.0;

    Vector3 velocity_hat{};
    Vector3 gun_axis_hat{};
    if (!Unit3(input.frame.own.velocity_ned_mps, velocity_hat)
        || !Unit3(
            input.frame.enemy_offense.los_hat_ned,
            enemy_to_own_hat)
        || !Unit3(
            input.frame.enemy_offense.attacker_nose_ned,
            gun_axis_hat))
    {
        return false;
    }

    // For the official circular gun cone, removing the component parallel to
    // the attacker's nose gives the unique nearest cone-side direction except
    // on the exact centerline.  Its dot product with enemy_to_own_hat is
    // non-negative, so the same vector also has an outward-range component;
    // no tactical blend gain is introduced.
    const double axial = Dot3(enemy_to_own_hat, gun_axis_hat);
    const Vector3 transverse{{
        enemy_to_own_hat[0] - axial * gun_axis_hat[0],
        enemy_to_own_hat[1] - axial * gun_axis_hat[1],
        enemy_to_own_hat[2] - axial * gun_axis_hat[2]}};
    Vector3 transverse_hat{};
    const bool transverse_resolved = Unit3(transverse, transverse_hat);
    const Vector3 body_up{{
        -input.frame.own.down_ned[0],
        -input.frame.own.down_ned[1],
        -input.frame.own.down_ned[2]}};
    Vector3 preferred_direction{};
    if (transverse_resolved)
    {
        preferred_direction = transverse_hat;
    }
    else
    {
        preferred_direction = body_up;
    }

    if (!VelocityNormalDirection(
            preferred_direction,
            velocity_hat,
            lift_direction))
    {
        const Vector3 root_aim_direction{{
            root_intent.aim_point_m[0]
                - input.frame.own.position_ned_m[0],
            root_intent.aim_point_m[1]
                - input.frame.own.position_ned_m[1],
            root_intent.aim_point_m[2]
                - input.frame.own.position_ned_m[2]}};
        if (!VelocityNormalDirection(
                body_up,
                velocity_hat,
                lift_direction)
            && !VelocityNormalDirection(
                root_aim_direction,
                velocity_hat,
                lift_direction)
            && !VelocityNormalDirection(
                input.frame.own.right_ned,
                velocity_hat,
                lift_direction))
        {
            return false;
        }
    }

    outward_alignment = Dot3(lift_direction, enemy_to_own_hat);
    if (!std::isfinite(outward_alignment))
    {
        return false;
    }

    // Projection into the aircraft's velocity-normal force plane can reverse
    // the radial component in unusual crossing geometry.  In that case use
    // the directly projected outward direction, provided it still points to
    // the same side of the cone.  This is a geometric fallback, not a gain.
    if (outward_alignment < 0.0)
    {
        Vector3 outward_lift{};
        if (!VelocityNormalDirection(
                enemy_to_own_hat,
                velocity_hat,
                outward_lift))
        {
            // When range direction is parallel to velocity, lift cannot have a
            // radial component. Keep the cone-exit lift and let the inherited
            // maximum-power longitudinal reference provide range growth.
            if (outward_alignment < -ScaledTolerance(outward_alignment))
            {
                lift_direction = Vector3{{
                    -lift_direction[0],
                    -lift_direction[1],
                    -lift_direction[2]}};
                outward_alignment = Dot3(
                    lift_direction,
                    enemy_to_own_hat);
            }
            return std::isfinite(outward_alignment)
                && outward_alignment
                    >= -ScaledTolerance(outward_alignment);
        }
        lift_direction = outward_lift;
        outward_alignment = Dot3(lift_direction, enemy_to_own_hat);
    }
    return std::isfinite(outward_alignment)
        && outward_alignment >= 0.0;
}

HighGBarrelReason ClassifyEntry(
    const TacticalCommandBuildInput& input,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence&,
    double& requested_load_g) noexcept
{
    requested_load_g = 0.0;
    if (!root_gun_selected)
    {
        return HighGBarrelReason::OfficialRootGunOwnerInactive;
    }
    double current_load_g = 0.0;
    if (!CurrentSafetyAndCommandLoad(input, current_load_g))
    {
        return HighGBarrelReason::CurrentPhysicalLoadUnavailable;
    }
    requested_load_g = current_load_g;
    return HighGBarrelReason::Admitted;
}

HighGBarrelReason ClassifyContinuation(
    const TacticalCommandBuildInput& input,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence&,
    const HighGBarrelOwnerSnapshot&,
    double& requested_load_g) noexcept
{
    requested_load_g = 0.0;
    if (!root_gun_selected)
    {
        return HighGBarrelReason::OfficialRootGunOwnerInactive;
    }
    if (!CurrentSafetyAndCommandLoad(input, requested_load_g))
    {
        return HighGBarrelReason::RunningSafetyRejected;
    }
    return HighGBarrelReason::Admitted;
}

void ClearLateralOwners(ControlIntent& output) noexcept
{
    output.direct_p_cmd_radps = LadyLuck::IntentOptionalValue<double>{};
    output.direct_nz_cmd_g = LadyLuck::IntentOptionalValue<double>{};
    output.direct_beta_cmd_rad = LadyLuck::IntentOptionalValue<double>{};
    output.direct_acceleration_ned_mps2 =
        LadyLuck::IntentOptionalValue<Vector3>{};
    output.direct_acceleration_roll_rate_reference_radps =
        LadyLuck::IntentOptionalValue<double>{};
    output.direct_acceleration_tracking_enabled = false;
    output.direct_acceleration_tracking_observation_only = false;
    output.direct_acceleration_magnitude_tracking_enabled = false;
    output.direct_acceleration_loaded_roll_enabled = false;
    output.direct_acceleration_load_component_compensation_enabled = false;
    output.direct_acceleration_yaw_coordination_enabled = false;
    output.direct_acceleration_roll_priority_yaw_enabled = false;
    output.direct_bank_cmd_rad = LadyLuck::IntentOptionalValue<double>{};
    output.direct_turn_rate_cmd_radps = LadyLuck::IntentOptionalValue<double>{};
    output.direct_load_vector_acceleration_ned_mps2 =
        LadyLuck::IntentOptionalValue<Vector3>{};
}

bool SnapshotFinite(const HighGBarrelOwnerSnapshot& value) noexcept
{
    return std::isfinite(value.requested_load_magnitude_g);
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g4
{

void HighGBarrelOwner::Reset() noexcept
{
    snapshot_ = HighGBarrelOwnerSnapshot{};
}

void HighGBarrelOwner::Observe(
    const runtime::TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    const bool root_gun_selected,
    const HighGBarrelExactEvidence& evidence,
    HighGBarrelSelectionReceipt& output,
    Status& status) const noexcept
{
    output = HighGBarrelSelectionReceipt{};
    status = Status{};
    Status root_status{};
    root_intent.Validate(root_status);
    if (!input.valid
        || !root_status.ok()
        || !IsValidControlFrameIdentity(input.frame.frame_identity)
        || (evidence.valid
            && !SameControlFrameIdentity(
                input.frame.frame_identity,
                evidence.frame_identity))
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            root_intent.frame_identity)
        || !SnapshotFinite(snapshot_))
    {
        status.code = root_status.ok()
            ? StatusCode::InvalidConfiguration
            : root_status.code;
        return;
    }

    output.valid = true;
    output.frame_identity = input.frame.frame_identity;
    output.engaged_before = snapshot_.engaged;
    output.vertical_excess = evidence.vertical_excess;
    if (snapshot_.engaged)
    {
        double requested_load_g = 0.0;
        const HighGBarrelReason reason = ClassifyContinuation(
            input,
            root_gun_selected,
            evidence,
            snapshot_,
            requested_load_g);
        output.reason = reason;
        if (reason != HighGBarrelReason::Admitted)
        {
            output.decision = HighGBarrelDecision::ReleasePassthrough;
            output.released_if_published = true;
            return;
        }
        output.selected_variant = snapshot_.variant;
        output.decision = snapshot_.variant == HighGBarrelVariant::Underneath
            ? HighGBarrelDecision::Underneath
            : HighGBarrelDecision::OverTheTop;
        output.requested_load_magnitude_g = requested_load_g;
        return;
    }

    double requested_load_g = 0.0;
    const HighGBarrelReason reason = ClassifyEntry(
        input,
        root_gun_selected,
        evidence,
        requested_load_g);
    output.reason = reason;
    if (reason != HighGBarrelReason::Admitted)
    {
        output.decision = HighGBarrelDecision::RootPassthrough;
        return;
    }

    output.entry_admitted = true;
    output.entered_if_published = true;
    output.requested_load_magnitude_g = requested_load_g;
    // Writer 14 is now one WEZ-exit owner.  Keep the deployed OverTheTop enum
    // only as an internal ABI branch key; vertical-energy state no longer
    // changes the evasive law or hands ownership to writer 30.
    output.selected_variant = HighGBarrelVariant::OverTheTop;
    output.decision = HighGBarrelDecision::OverTheTop;
}

void HighGBarrelOwner::BuildCandidate(
    const HighGBarrelVariant selected_variant,
    const runtime::TacticalCommandBuildInput& input,
    const ControlIntent& root_intent,
    const HighGBarrelExactEvidence& evidence,
    const HighGBarrelSelectionReceipt& selection,
    ControlIntent& output,
    HighGBarrelOwnerSnapshot& commit,
    HighGBarrelTaskReceipt& receipt,
    Status& status) const noexcept
{
    output.Clear();
    commit = snapshot_;
    receipt = HighGBarrelTaskReceipt{};
    status = Status{};
    Status root_status{};
    root_intent.Validate(root_status);
    const bool variant_valid = selected_variant == HighGBarrelVariant::Underneath
        || selected_variant == HighGBarrelVariant::OverTheTop;
    const HighGBarrelDecision expected_decision =
        selected_variant == HighGBarrelVariant::Underneath
        ? HighGBarrelDecision::Underneath
        : HighGBarrelDecision::OverTheTop;
    if (!root_status.ok()
        || !input.valid
        || !selection.valid
        || !variant_valid
        || selection.decision != expected_decision
        || selection.selected_variant != selected_variant
        || (evidence.valid
            && !SameControlFrameIdentity(
                input.frame.frame_identity,
                evidence.frame_identity))
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            selection.frame_identity)
        || !SameControlFrameIdentity(
            input.frame.frame_identity,
            root_intent.frame_identity)
        || !std::isfinite(input.accepted_estimator.sample_dt_s)
        || input.accepted_estimator.sample_dt_s <= 0.0)
    {
        status.code = root_status.ok()
            ? StatusCode::InvalidConfiguration
            : root_status.code;
        return;
    }

    bool entered = false;
    if (!commit.engaged)
    {
        if (!selection.entry_admitted
            || !std::isfinite(selection.requested_load_magnitude_g)
            || selection.requested_load_magnitude_g <= 0.0)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        commit.engaged = true;
        commit.variant = selected_variant;
        commit.phase = HighGBarrelPhase::WezShortestExit;
        commit.requested_load_magnitude_g =
            selection.requested_load_magnitude_g;
        commit.last_release_reason = HighGBarrelReason::Unavailable;
        entered = true;
    }
    if (commit.variant != selected_variant
        || commit.phase != HighGBarrelPhase::WezShortestExit
        || !SnapshotFinite(commit)
        || commit.requested_load_magnitude_g <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!entered)
    {
        if (!std::isfinite(selection.requested_load_magnitude_g)
            || selection.requested_load_magnitude_g <= 0.0)
        {
            status.code = StatusCode::InvalidConfiguration;
            return;
        }
        // Recompute against the current physical envelope each tick instead
        // of holding a previous-frame load receipt as maneuver authority.
        commit.requested_load_magnitude_g =
            selection.requested_load_magnitude_g;
    }

    Vector3 velocity_hat{};
    Vector3 desired_bank{};
    Vector3 enemy_to_own_hat{};
    double outward_alignment = 0.0;
    const bool reference_available =
        Unit3(input.frame.own.velocity_ned_mps, velocity_hat)
        && BuildWezShortestExitDirection(
            input,
            root_intent,
            desired_bank,
            enemy_to_own_hat,
            outward_alignment);
    if (!reference_available)
    {
        commit = HighGBarrelOwnerSnapshot{};
        commit.last_release_reason =
            HighGBarrelReason::LoadedRollCommandUnavailable;
        output = root_intent;
        receipt.valid = true;
        receipt.frame_identity = input.frame.frame_identity;
        receipt.root_passthrough_required = true;
        receipt.reason = HighGBarrelReason::LoadedRollCommandUnavailable;
        status = Status{};
        return;
    }

    const double force_magnitude = commit.requested_load_magnitude_g
        * constants::StandardGravityMps2;
    const Vector3 acceleration{{
        force_magnitude * desired_bank[0],
        force_magnitude * desired_bank[1],
        constants::StandardGravityMps2
            + force_magnitude * desired_bank[2]}};
    // VelocityNormalDirection already constructs this load in the admissible
    // force plane. Do not repeat a near-bitwise orthogonality comparison here;
    // the Direct-NED shaper owns the physical projection and clipping.
    if (!FiniteVector(acceleration))
    {
        commit = HighGBarrelOwnerSnapshot{};
        commit.last_release_reason =
            HighGBarrelReason::LoadedRollCommandUnavailable;
        output = root_intent;
        receipt.valid = true;
        receipt.frame_identity = input.frame.frame_identity;
        receipt.root_passthrough_required = true;
        receipt.reason = HighGBarrelReason::LoadedRollCommandUnavailable;
        return;
    }

    output = root_intent;
    ClearLateralOwners(output);
    output.total_load_factor_limit_g.has_value = true;
    output.total_load_factor_limit_g.value =
        commit.requested_load_magnitude_g;
    output.path_inversion_allowed.has_value = true;
    output.path_inversion_allowed.value = true;
    output.direct_acceleration_ned_mps2.has_value = true;
    output.direct_acceleration_ned_mps2.value = acceleration;
    output.direct_acceleration_loaded_roll_enabled = true;
    output.route_kind = ControlRouteKind::DirectNedAcceleration;
    output.behavior_id = DoctrineBehaviorG4WezShortestExit;
    output.writer_id = ControlIntentWriterG4HighGBarrel;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
        commit = snapshot_;
        return;
    }
    receipt.valid = true;
    receipt.frame_identity = input.frame.frame_identity;
    receipt.candidate_available = true;
    receipt.entered_this_tick = entered;
    receipt.variant = selected_variant;
    receipt.phase = commit.phase;
    receipt.reason = HighGBarrelReason::Admitted;
    receipt.desired_bank_direction_ned = desired_bank;
    receipt.acceleration_ned_mps2 = acceleration;
    receipt.load_magnitude_g = commit.requested_load_magnitude_g;
    receipt.los_projection_valid = true;
    receipt.los_direction_velocity_normal_ned = enemy_to_own_hat;
    receipt.los_pull_alignment = outward_alignment;
}

void HighGBarrelOwner::BuildReleaseCommit(
    const HighGBarrelSelectionReceipt& selection,
    HighGBarrelOwnerSnapshot& commit,
    Status& status) const noexcept
{
    commit = snapshot_;
    status = Status{};
    if (!selection.valid
        || selection.decision != HighGBarrelDecision::ReleasePassthrough
        || !snapshot_.engaged)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    commit = HighGBarrelOwnerSnapshot{};
    commit.last_release_reason = selection.reason;
}

void HighGBarrelOwner::CommitPublished(
    const HighGBarrelOwnerSnapshot& commit,
    Status& status) noexcept
{
    status = Status{};
    if (!SnapshotFinite(commit)
        || (commit.engaged
            && (commit.variant == HighGBarrelVariant::None
                || commit.phase != HighGBarrelPhase::WezShortestExit
                || commit.requested_load_magnitude_g <= 0.0))
        || (!commit.engaged
            && (commit.variant != HighGBarrelVariant::None
                || commit.phase != HighGBarrelPhase::None
                || commit.requested_load_magnitude_g != 0.0)))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    snapshot_ = commit;
}

void HighGBarrelOwner::CopySnapshot(
    HighGBarrelOwnerSnapshot& output) const noexcept
{
    output = snapshot_;
}

} // namespace g4
} // namespace guidance
} // namespace LadyLuck
