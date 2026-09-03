#include "LadyLuck/control/direct_ned/DirectNedLoadVector.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/common/Numerics.hpp"

#include <algorithm>
#include <cmath>

namespace LadyLuck
{
namespace control
{
namespace direct_ned
{
namespace
{
constexpr double EpsLowG = 0.2;
constexpr double EpsHighG = 0.5;
constexpr double ServoGainPerSecond = 4.0;
constexpr double TurnSideReleaseMarginRad = 0.2;
constexpr double BetaCommandRad = 0.0;
constexpr double BetaGainPerSecond = 2.0;
constexpr double YawLoadGain = 0.25;

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteMatrix(const Matrix3RowMajor& value) noexcept
{
    for (const double element : value)
    {
        if (!std::isfinite(element))
        {
            return false;
        }
    }
    return true;
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    const double result = (left[0] * right[0] + left[1] * right[1])
        + left[2] * right[2];
    // NumPy's three-element dot accumulator returns +0 for an exact signed-
    // zero cancellation.  Preserve that public bit contract without changing
    // any nonzero result.
    return result == 0.0 ? 0.0 : result;
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
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

Vector3 Scale(const Vector3& value, const double scale) noexcept
{
    return Vector3{{value[0] * scale, value[1] * scale, value[2] * scale}};
}

Vector3 Divide(const Vector3& value, const double divisor) noexcept
{
    // Python normalizes with vector / norm, not vector * (1 / norm).  The two
    // forms are physically equivalent but can differ by one binary64 ULP.
    return Vector3{{
        value[0] / divisor,
        value[1] / divisor,
        value[2] / divisor}};
}

Vector3 Cross(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

Vector3 MatrixVector(
    const Matrix3RowMajor& matrix,
    const Vector3& vector) noexcept
{
    // NumPy 2.2.6's frozen 3x3 @ 3-vector kernel reduces lanes as
    // (p0 + p2) + p1.  This is intentionally distinct from np.dot's local
    // three-vector order above.
    return Vector3{{
        (matrix[0] * vector[0] + matrix[2] * vector[2])
            + matrix[1] * vector[1],
        (matrix[3] * vector[0] + matrix[5] * vector[2])
            + matrix[4] * vector[1],
        (matrix[6] * vector[0] + matrix[8] * vector[2])
            + matrix[7] * vector[1]}};
}

Vector3 MatrixTransposeVector(
    const Matrix3RowMajor& matrix,
    const Vector3& vector) noexcept
{
    return Vector3{{
        (matrix[0] * vector[0] + matrix[6] * vector[2])
            + matrix[3] * vector[1],
        (matrix[1] * vector[0] + matrix[7] * vector[2])
            + matrix[4] * vector[1],
        (matrix[2] * vector[0] + matrix[8] * vector[2])
            + matrix[5] * vector[1]}};
}

double Clamp(const double value, const double lower, const double upper) noexcept
{
    // np.clip preserves an in-range signed zero.  Branch instead of using
    // std::min/std::max so the frozen IEEE-754 contract does the same.
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

double WrapPi(const double value) noexcept
{
    return std::atan2(std::sin(value), std::cos(value));
}

double Smoothstep01(const double value) noexcept
{
    const double x = Clamp(value, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

Vector3 Normalize(const Vector3& value) noexcept
{
    return Divide(value, Norm3(value));
}

Vector3 DeterministicPerpendicular(const Vector3& axis) noexcept
{
    const Vector3 candidates[] = {
        Vector3{{0.0, 0.0, -1.0}},
        Vector3{{0.0, 1.0, 0.0}},
        Vector3{{1.0, 0.0, 0.0}}};
    for (const Vector3& candidate : candidates)
    {
        const Vector3 projected =
            Subtract(candidate, Scale(axis, Dot3(candidate, axis)));
        const double norm = Norm3(projected);
        if (norm > constants::Tiny)
        {
            return Divide(projected, norm);
        }
    }
    return Vector3{{0.0, 0.0, 0.0}};
}

Vector3 ReprojectUnit(
    const Vector3& direction,
    const Vector3& velocity_hat,
    const Vector3* const fallback) noexcept
{
    Vector3 projected = Subtract(
        direction,
        Scale(velocity_hat, Dot3(direction, velocity_hat)));
    double norm = Norm3(projected);
    if (norm > constants::Tiny)
    {
        return Divide(projected, norm);
    }
    if (fallback != nullptr)
    {
        projected = Subtract(
            *fallback,
            Scale(velocity_hat, Dot3(*fallback, velocity_hat)));
        norm = Norm3(projected);
        if (norm > constants::Tiny)
        {
            return Divide(projected, norm);
        }
    }
    return DeterministicPerpendicular(velocity_hat);
}

double SignedAngle(
    const Vector3& from,
    const Vector3& to,
    const Vector3& axis) noexcept
{
    const Vector3 cross = Cross(from, to);
    return std::atan2(
        Dot3(cross, axis),
        Clamp(Dot3(from, to), -1.0, 1.0));
}

Vector3 RotateAboutAxis(
    const Vector3& direction,
    const Vector3& axis,
    const double angle,
    const Vector3* const fallback) noexcept
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const Vector3 axial_projection =
        Scale(axis, Dot3(axis, direction));
    Vector3 rotated = Add(
        Add(Scale(direction, cosine), Scale(Cross(axis, direction), sine)),
        Scale(axial_projection, 1.0 - cosine));
    return ReprojectUnit(rotated, axis, fallback);
}

struct RotateTowardReceipt
{
    Vector3 direction{};
    bool antipodal = false;
};

RotateTowardReceipt RotateToward(
    const Vector3& from,
    const Vector3& to,
    const double weight,
    const Vector3& axis) noexcept
{
    RotateTowardReceipt receipt{};
    const double clamped_weight = Clamp(weight, 0.0, 1.0);
    const double dot = Clamp(Dot3(from, to), -1.0, 1.0);
    const double cross_norm = Norm3(Cross(from, to));
    receipt.antipodal = dot < 0.0 && cross_norm <= constants::Tiny;
    if (clamped_weight <= 0.0)
    {
        receipt.direction = from;
        return receipt;
    }
    if (clamped_weight >= 1.0)
    {
        receipt.direction = to;
        return receipt;
    }
    double angle = SignedAngle(from, to, axis);
    if (receipt.antipodal)
    {
        angle = constants::Pi;
    }
    if (std::fabs(angle) <= constants::Tiny)
    {
        receipt.direction = from;
        return receipt;
    }
    // Python rotate_toward() uses the requested target as the degenerate
    // reprojection fallback.  The ordinary proper-unit domain never needs the
    // fallback, but preserving it keeps the failure boundary exact.
    receipt.direction = RotateAboutAxis(
        from,
        axis,
        clamped_weight * angle,
        &to);
    return receipt;
}

Status Failure(const StatusCode code) noexcept
{
    Status status{};
    status.code = code;
    return status;
}

Status ValidateState(const DirectNedLoadVectorState& state) noexcept
{
    if (!IsValidControlFrameIdentity(state.frame_identity))
    {
        return Failure(StatusCode::InvalidArgument);
    }
    if (!FiniteVector(state.velocity_ned_mps)
        || !FiniteMatrix(state.c_body_from_ned)
        || !std::isfinite(state.speed_mps)
        || !std::isfinite(state.alpha_rad)
        || !std::isfinite(state.beta_rad)
        || !std::isfinite(state.pitch_rad)
        || !std::isfinite(state.roll_rad)
        || !std::isfinite(state.nz_feasible_g)
        || !std::isfinite(state.ground_speed_horizontal_mps)
        || !std::isfinite(state.max_p_radps))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    if (state.speed_mps < 0.0
        || state.nz_feasible_g < 0.0
        || state.ground_speed_horizontal_mps < 0.0
        || state.max_p_radps <= 0.0)
    {
        return Failure(StatusCode::InvalidArgument);
    }
    return Status{};
}

Status ValidateCommand(const DirectNedLoadVectorCommand& command) noexcept
{
    if (!command.valid
        || !IsValidControlFrameIdentity(command.frame_identity))
    {
        return Failure(StatusCode::InvalidArgument);
    }
    if (!FiniteVector(command.acceleration_ned_mps2)
        || (command.roll_rate_reference_valid
            && !std::isfinite(command.roll_rate_reference_radps)))
    {
        return Failure(StatusCode::NonFiniteInput);
    }
    return Status{};
}

double GVelocitySchedule(const double horizontal_speed_mps) noexcept
{
    const double x = horizontal_speed_mps * constants::MetersToFeet;
    if (x <= 80.0)
    {
        return 0.0;
    }
    if (x >= 150.0)
    {
        return 100.0;
    }
    if (x <= 100.0)
    {
        const double t = (x - 80.0) / 20.0;
        return 0.0 + t * (15.0 - 0.0);
    }
    const double t = (x - 100.0) / 50.0;
    return 15.0 + t * (100.0 - 15.0);
}

double CoordinatedYawReference(
    const double p_cmd_radps,
    const DirectNedLoadVectorState& state) noexcept
{
    const double speed = std::max(
        state.speed_mps,
        numerics::CisPairLongitudinalSpeedRegularizationMps);
    const double ca = std::cos(state.alpha_rad);
    const double sa = std::sin(state.alpha_rad);
    const double cth = std::cos(state.pitch_rad);
    const double sph = std::sin(state.roll_rad);
    const double regulator =
        BetaGainPerSecond * (state.beta_rad - BetaCommandRad);
    const double inverse_cosine = numerics::RegularizedSignedInverse(
        ca,
        numerics::CisPairCosineRegularization);
    double r_cmd = (
        p_cmd_radps * sa
        + (constants::StandardGravityMps2 / speed) * cth * sph
        + regulator) * inverse_cosine;
    const double ny_cmd =
        (speed / constants::StandardGravityMps2) * regulator;
    const double horizontal_speed =
        std::max(state.ground_speed_horizontal_mps, 0.0);
    const double schedule = GVelocitySchedule(horizontal_speed);
    if (schedule > 1.0e-9)
    {
        r_cmd += YawLoadGain * ny_cmd / schedule;
    }
    return r_cmd;
}
}

void DirectNedLoadVector::Reset() noexcept
{
    has_direction_hold_ = false;
    direction_hold_ned_ = Vector3{};
    has_c6_gate_ = false;
    c6_gate_value_ = 0.0;
    has_last_nz_ = false;
    last_nz_cmd_g_ = 0.0;
    servo_filtered_error_rad_ = 0.0;
    servo_filtered_rate_radps_ = 0.0;
    servo_committed_turn_side_ = 0;
    servo_last_p_cmd_radps_ = 0.0;
    servo_last_feedback_error_rad_ = 0.0;
    servo_output_slew_active_ = false;
    has_last_output_ = false;
}

void DirectNedLoadVector::CopySnapshot(
    DirectNedLoadVectorSnapshot& output) const noexcept
{
    output = DirectNedLoadVectorSnapshot{};
    output.has_direction_hold = has_direction_hold_;
    output.direction_hold_ned = direction_hold_ned_;
    output.has_c6_gate = has_c6_gate_;
    output.c6_gate_value = c6_gate_value_;
    output.has_last_nz = has_last_nz_;
    output.last_nz_cmd_g = last_nz_cmd_g_;
    output.servo_filtered_error_rad = servo_filtered_error_rad_;
    output.servo_filtered_rate_radps = servo_filtered_rate_radps_;
    output.servo_committed_turn_side = servo_committed_turn_side_;
    output.servo_last_p_cmd_radps = servo_last_p_cmd_radps_;
    output.servo_last_feedback_error_rad = servo_last_feedback_error_rad_;
    output.servo_output_slew_active = servo_output_slew_active_;
    output.has_last_output = has_last_output_;
}

void DirectNedLoadVector::Step(
    const DirectNedLoadVectorCommand& command,
    const DirectNedLoadVectorState& state,
    const double dt_s,
    DirectNedLoadVectorOutput& output,
    Status& status) noexcept
{
    output = DirectNedLoadVectorOutput{};
    status = ValidateCommand(command);
    if (!status.sample_valid())
    {
        return;
    }
    status = ValidateState(state);
    if (!status.sample_valid())
    {
        return;
    }
    if (!SameControlFrameIdentity(
            command.frame_identity,
            state.frame_identity))
    {
        status = Failure(StatusCode::InvalidArgument);
        return;
    }
    if (!std::isfinite(dt_s))
    {
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }
    if (dt_s < 0.0)
    {
        status = Failure(StatusCode::InvalidDt);
        return;
    }

    bool next_has_direction_hold = has_direction_hold_;
    Vector3 next_direction_hold = direction_hold_ned_;
    bool next_has_c6_gate = has_c6_gate_;
    double next_c6_gate_value = c6_gate_value_;
    bool next_has_last_nz = has_last_nz_;
    double next_last_nz = last_nz_cmd_g_;
    double next_servo_filtered_error = servo_filtered_error_rad_;
    double next_servo_filtered_rate = servo_filtered_rate_radps_;
    std::int32_t next_servo_committed_turn_side =
        servo_committed_turn_side_;
    double next_servo_last_p = servo_last_p_cmd_radps_;
    double next_servo_last_feedback_error = servo_last_feedback_error_rad_;
    bool next_servo_output_slew_active = servo_output_slew_active_;

    const Vector3& acceleration = command.acceleration_ned_mps2;
    std::uint32_t flags = 0U;

    Vector3 velocity_hat{};
    const double velocity_norm = Norm3(state.velocity_ned_mps);
    if (velocity_norm > constants::Tiny)
    {
        velocity_hat = Divide(state.velocity_ned_mps, velocity_norm);
    }
    else
    {
        const Vector3 body_forward{{1.0, 0.0, 0.0}};
        const Vector3 forward_ned =
            MatrixTransposeVector(state.c_body_from_ned, body_forward);
        velocity_hat = Normalize(forward_ned);
        flags |= GuardSpeedDegenerate;
    }

    const Vector3 gravity{{0.0, 0.0, constants::StandardGravityMps2}};
    const Vector3 desired_specific_force = Subtract(acceleration, gravity);
    const Vector3 force_parallel = Scale(
        velocity_hat,
        Dot3(desired_specific_force, velocity_hat));
    const Vector3 force_perp =
        Subtract(desired_specific_force, force_parallel);
    const double force_perp_norm = Norm3(force_perp);
    const double force_perp_norm_g =
        force_perp_norm / constants::StandardGravityMps2;
    const double clip_scale = std::min(
        1.0,
        state.nz_feasible_g * constants::StandardGravityMps2
            / std::max(force_perp_norm, constants::Tiny));
    const double force_limited_magnitude = force_perp_norm * clip_scale;
    const double guard_weight = Smoothstep01(
        (force_perp_norm_g - EpsLowG) / (EpsHighG - EpsLowG));
    if (force_perp_norm_g <= EpsLowG)
    {
        flags |= GuardLowG;
    }

    const Vector3 body_down{{0.0, 0.0, 1.0}};
    const Vector3 body_down_ned =
        MatrixTransposeVector(state.c_body_from_ned, body_down);
    const Vector3 lift_ned = Scale(body_down_ned, -1.0);
    const Vector3 projected_lift = Subtract(
        lift_ned,
        Scale(velocity_hat, Dot3(lift_ned, velocity_hat)));
    const double projected_lift_norm = Norm3(projected_lift);
    Vector3 current_lift{};
    if (projected_lift_norm > constants::Tiny)
    {
        current_lift = Divide(projected_lift, projected_lift_norm);
    }
    else
    {
        const Vector3 fallback = next_has_direction_hold
            ? next_direction_hold
            : DeterministicPerpendicular(velocity_hat);
        current_lift = ReprojectUnit(fallback, velocity_hat, nullptr);
        flags |= GuardLiftAxisDegenerate;
    }

    if (!next_has_direction_hold)
    {
        // Seed the base force direction before considering the optional roll
        // lead. Optional lead nonadmission must not revoke this current-frame
        // acceleration authority.
        direction_hold_ned_ = current_lift;
        has_direction_hold_ = true;
        next_direction_hold = current_lift;
        next_has_direction_hold = true;
    }
    const Vector3 direction_previous = ReprojectUnit(
        next_direction_hold,
        velocity_hat,
        &current_lift);
    const Vector3 requested_direction_raw = force_perp_norm > constants::Tiny
        ? Divide(force_perp, force_perp_norm)
        : direction_previous;

    Vector3 direction_raw = requested_direction_raw;
    bool roll_reference_valid = false;
    double roll_reference = 0.0;
    bool equivalent_lead_valid = false;
    double equivalent_lead = 0.0;
    bool requested_direction_error_valid = false;
    double requested_direction_error = 0.0;
    if (command.roll_rate_reference_valid)
    {
        const double requested = Clamp(
            command.roll_rate_reference_radps,
            -state.max_p_radps,
            state.max_p_radps);
        const double lead = requested / ServoGainPerSecond;
        if (std::fabs(requested) <= constants::Tiny)
        {
            roll_reference_valid = true;
            roll_reference = 0.0;
            equivalent_lead_valid = true;
            equivalent_lead = 0.0;
        }
        else if (std::fabs(lead) < 0.5 * constants::Pi
            && force_perp_norm > constants::Tiny)
        {
            const double winding_error = SignedAngle(
                current_lift,
                requested_direction_raw,
                velocity_hat);
            if (winding_error * requested > 0.0)
            {
                direction_raw = RotateAboutAxis(
                    current_lift,
                    velocity_hat,
                    lead,
                    &current_lift);
                roll_reference_valid = true;
                roll_reference = requested;
                equivalent_lead_valid = true;
                equivalent_lead = lead;
                requested_direction_error_valid = true;
                requested_direction_error = winding_error;
            }
        }
        // A finite but geometrically inconsistent optional roll lead is
        // nonadmitted. The requested acceleration direction remains the base
        // owner instead of turning an optional overlay into command loss.
    }

    const Vector3 force_limited =
        Scale(direction_raw, force_limited_magnitude);
    const Vector3 force_body =
        MatrixVector(state.c_body_from_ned, force_limited);
    const double nz_cmd_raw =
        -force_body[2] / constants::StandardGravityMps2;
    const double previous_dot_raw =
        Clamp(Dot3(direction_previous, direction_raw), -1.0, 1.0);
    const double raw_separation = std::acos(previous_dot_raw);
    if (Dot3(direction_previous, direction_raw) < 0.0)
    {
        flags |= GuardDirectionReversal;
    }
    const RotateTowardReceipt target = RotateToward(
        direction_previous,
        direction_raw,
        guard_weight,
        velocity_hat);
    if (target.antipodal)
    {
        flags |= GuardAntipodal;
    }
    const double direction_step = std::acos(Clamp(
        Dot3(direction_previous, target.direction),
        -1.0,
        1.0));
    next_direction_hold = target.direction;

    const double direction_error = SignedAngle(
        current_lift,
        target.direction,
        velocity_hat);
    double servo_error = WrapPi(direction_error);
    const double release_threshold =
        constants::Pi - TurnSideReleaseMarginRad;
    if (next_servo_committed_turn_side != 0)
    {
        if (std::fabs(servo_error) < release_threshold)
        {
            next_servo_committed_turn_side = 0;
        }
    }
    else if (std::fabs(servo_error) >= release_threshold)
    {
        double side_source = servo_error;
        if (std::fabs(next_servo_last_p) > constants::Tiny)
        {
            side_source = next_servo_last_p;
        }
        else if (std::fabs(next_servo_filtered_rate) > constants::Tiny)
        {
            side_source = next_servo_filtered_rate;
        }
        else if (std::fabs(side_source) <= constants::Tiny)
        {
            side_source = 1.0;
        }
        next_servo_committed_turn_side = side_source > 0.0 ? 1 : -1;
    }
    servo_error = WrapPi(servo_error);
    if (next_servo_committed_turn_side != 0
        && std::fabs(servo_error) >= release_threshold)
    {
        servo_error = std::copysign(
            std::fabs(servo_error),
            static_cast<double>(next_servo_committed_turn_side));
    }
    double p_cmd = ServoGainPerSecond * servo_error;
    p_cmd = Clamp(p_cmd, -std::fabs(state.max_p_radps),
        std::fabs(state.max_p_radps));
    next_servo_last_feedback_error = servo_error;
    next_servo_last_p = p_cmd;

    const double gravity_projection =
        std::cos(state.pitch_rad) * std::cos(state.roll_rad);
    const double c6_value = Clamp(std::cos(std::fabs(direction_error)), 0.0, 1.0);
    next_c6_gate_value = c6_value;
    next_has_c6_gate = true;
    const double nz_cmd = c6_value * nz_cmd_raw
        + (1.0 - c6_value) * gravity_projection;
    next_last_nz = nz_cmd;
    next_has_last_nz = true;

    const double longitudinal_speed_mps =
        state.speed_mps * std::cos(state.alpha_rad);
    const double q_cmd = (nz_cmd - gravity_projection)
        * constants::StandardGravityMps2
        * numerics::RegularizedSignedInverse(
            longitudinal_speed_mps,
            numerics::CisPairLongitudinalSpeedRegularizationMps);
    const double r_cmd = CoordinatedYawReference(p_cmd, state);

    output.frame_identity = state.frame_identity;
    output.valid = true;
    output.p_cmd_radps = p_cmd;
    output.q_cmd_radps = q_cmd;
    output.r_cmd_radps = r_cmd;
    output.nz_cmd_g = nz_cmd;
    output.nz_cmd_raw_g = nz_cmd_raw;
    output.force_perp_norm_g = force_perp_norm_g;
    output.guard_weight = guard_weight;
    output.target_direction_ned = target.direction;
    output.raw_direction_ned = direction_raw;
    output.direction_error_rad = direction_error;
    output.direction_step_angle_rad = direction_step;
    output.raw_direction_separation_rad = raw_separation;
    output.c6_gate_value = c6_value;
    output.guard_flags = flags;
    output.clip_scale = clip_scale;
    output.roll_rate_reference_valid = roll_reference_valid;
    output.roll_rate_reference_radps = roll_reference;
    output.roll_rate_equivalent_bank_lead_valid = equivalent_lead_valid;
    output.roll_rate_equivalent_bank_lead_rad = equivalent_lead;
    output.requested_direction_error_valid = requested_direction_error_valid;
    output.requested_direction_error_rad = requested_direction_error;
    const double output_values[] = {
        output.p_cmd_radps,
        output.q_cmd_radps,
        output.r_cmd_radps,
        output.nz_cmd_g,
        output.nz_cmd_raw_g,
        output.force_perp_norm_g,
        output.guard_weight,
        output.direction_error_rad,
        output.direction_step_angle_rad,
        output.raw_direction_separation_rad,
        output.c6_gate_value,
        output.clip_scale,
        output.roll_rate_reference_radps,
        output.roll_rate_equivalent_bank_lead_rad,
        output.requested_direction_error_rad};
    bool output_finite = FiniteVector(output.target_direction_ned)
        && FiniteVector(output.raw_direction_ned);
    for (const double value : output_values)
    {
        output_finite = output_finite && std::isfinite(value);
    }
    if (!output_finite)
    {
        output = DirectNedLoadVectorOutput{};
        status = Failure(StatusCode::NonFiniteInput);
        return;
    }

    has_direction_hold_ = next_has_direction_hold;
    direction_hold_ned_ = next_direction_hold;
    has_c6_gate_ = next_has_c6_gate;
    c6_gate_value_ = next_c6_gate_value;
    has_last_nz_ = next_has_last_nz;
    last_nz_cmd_g_ = next_last_nz;
    servo_filtered_error_rad_ = next_servo_filtered_error;
    servo_filtered_rate_radps_ = next_servo_filtered_rate;
    servo_committed_turn_side_ = next_servo_committed_turn_side;
    servo_last_p_cmd_radps_ = next_servo_last_p;
    servo_last_feedback_error_rad_ = next_servo_last_feedback_error;
    servo_output_slew_active_ = next_servo_output_slew_active;
    has_last_output_ = true;
    status = Status{};
}
}
}
}
