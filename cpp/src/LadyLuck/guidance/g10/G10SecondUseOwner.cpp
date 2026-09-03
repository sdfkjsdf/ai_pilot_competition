#include "LadyLuck/guidance/g10/G10SecondUseOwner.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace
{

using LadyLuck::ControlFrameIdentity;
using LadyLuck::ControlIntent;
using LadyLuck::ControlRouteKind;
using LadyLuck::DoctrineBehaviorId;
using LadyLuck::DoctrineModeId;
using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::g10::G10BarrelLoadSelectionReceipt;
using LadyLuck::guidance::g10::G10BarrelLoadSelectionStatus;
using LadyLuck::guidance::g10::G10FrozenPairRuntimeAdmission;
using LadyLuck::guidance::g10::G10FrozenPairRuntimeReason;
using LadyLuck::guidance::g10::G10OptionalBool;
using LadyLuck::guidance::g10::G10OptionalDouble;
using LadyLuck::guidance::g10::G10SecondUseCommand;
using LadyLuck::guidance::g10::G10SecondUseCommandLabel;
using LadyLuck::guidance::g10::G10SecondUseOperation;
using LadyLuck::guidance::g10::G10SecondUseOperationTrace;
using LadyLuck::guidance::g10::G10SecondUseOwnerInput;
using LadyLuck::guidance::g10::G10SecondUseOwner;
using LadyLuck::guidance::g10::G10SecondUseOwnerPhase;
using LadyLuck::guidance::g10::G10SecondUseOwnerReason;
using LadyLuck::guidance::g10::G10SecondUseOwnerReceipt;
using LadyLuck::guidance::g10::G10SecondUseOwnerSnapshot;
using LadyLuck::guidance::g10::G10SecondUseSelectionBinding;
using LadyLuck::guidance::g10::G10SecondUseProviderWindowReceipt;
using LadyLuck::guidance::g10::G10SecondUseReferenceRole;
using LadyLuck::guidance::g10::G10SecondUseTargetPathHistory;
using LadyLuck::guidance::g10::G10VelocityBankRollProgress;

constexpr double kNominalRollProgressRad =
    1.5 * LadyLuck::constants::Pi;
constexpr double kStageAimDistanceM = 10000.0;
constexpr double kDefaultCaptureRangeM = 650.0;
constexpr double kMachineEpsilon =
    std::numeric_limits<double>::epsilon();
constexpr double kAttitudeObservationMinimum =
    1.4901161193847656e-8; // sqrt(binary64 epsilon), exactly as Python.

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3(const Vector3& left, const Vector3& right) noexcept
{
    // NumPy's three-element association retained by the existing C++ ports.
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

Vector3 Cross3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]}};
}

double Norm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3(value, value));
}

Vector3 Add3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2]}};
}

Vector3 Subtract3(const Vector3& left, const Vector3& right) noexcept
{
    return Vector3{{
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2]}};
}

Vector3 Scale3(const Vector3& value, const double scalar) noexcept
{
    return Vector3{{
        value[0] * scalar,
        value[1] * scalar,
        value[2] * scalar}};
}

double ScaledTolerance(
    const std::initializer_list<double> values) noexcept
{
    double scale = 1.0;
    for (const double value : values)
    {
        scale = (std::max)(scale, std::fabs(value));
    }
    return 64.0 * kMachineEpsilon * scale;
}

bool Unit3(const Vector3& value, Vector3& output) noexcept
{
    if (!FiniteVector(value))
    {
        return false;
    }
    const double magnitude = Norm3(value);
    if (!std::isfinite(magnitude) || magnitude <= kMachineEpsilon)
    {
        return false;
    }
    output = Scale3(value, 1.0 / magnitude);
    return FiniteVector(output);
}

void SetOptional(G10OptionalDouble& output, const double value) noexcept
{
    output.has_value = true;
    output.value = value;
}

void AppendOperation(
    G10SecondUseOperationTrace& trace,
    const G10SecondUseOperation operation) noexcept
{
    if (trace.count < trace.values.size())
    {
        trace.values[trace.count] = operation;
        ++trace.count;
    }
}

void ReleaseG10SecondUseOwner(
    G10SecondUseOwner& owner,
    const bool was_engaged,
    const G10SecondUseCommand& root_command,
    G10SecondUseOwnerReceipt& output,
    const G10SecondUseOwnerReason reason,
    const bool evaluated,
    const G10SecondUseSelectionBinding& binding,
    const G10FrozenPairRuntimeAdmission& pair) noexcept
{
    G10SecondUseOperationTrace trace = output.operation_trace;
    AppendOperation(trace, G10SecondUseOperation::OwnerReleased);
    owner.Reset();
    output = G10SecondUseOwnerReceipt{};
    output.valid = true;
    output.evaluated = evaluated;
    output.released_this_tick = was_engaged;
    output.reason = reason;
    output.binding = binding;
    output.runtime_pair = pair;
    output.command = root_command;
    output.operation_trace = trace;
}

class G10SecondUseReleaseAction final
{
public:
    G10SecondUseReleaseAction(
        G10SecondUseOwner& owner,
        const G10SecondUseCommand& root_command,
        G10SecondUseOwnerReceipt& output) noexcept
        : owner_(owner), root_command_(root_command), output_(output)
    {
    }

    void operator()(
        const G10SecondUseOwnerReason reason,
        const bool evaluated,
        const G10SecondUseSelectionBinding& binding,
        const G10FrozenPairRuntimeAdmission& pair) noexcept
    {
        G10SecondUseOwnerSnapshot snapshot{};
        owner_.CopySnapshot(snapshot);
        ReleaseG10SecondUseOwner(
            owner_,
            snapshot.engaged,
            root_command_,
            output_,
            reason,
            evaluated,
            binding,
            pair);
    }

private:
    G10SecondUseOwner& owner_;
    const G10SecondUseCommand& root_command_;
    G10SecondUseOwnerReceipt& output_;
};

bool WithinLimit(const double value, const double limit) noexcept
{
    const double tolerance = ScaledTolerance({value, limit});
    return value <= limit + tolerance;
}

G10SecondUseReferenceRole RoleForPhase(
    const G10SecondUseOwnerPhase phase) noexcept
{
    switch (phase)
    {
    case G10SecondUseOwnerPhase::BarrelPitchUp:
    case G10SecondUseOwnerPhase::BarrelLoadedRoll:
        return G10SecondUseReferenceRole::BarrelRollAttack;
    case G10SecondUseOwnerPhase::DescendingLagReacquire:
        return G10SecondUseReferenceRole::DescendingLag;
    case G10SecondUseOwnerPhase::G16EHandOff:
        return G10SecondUseReferenceRole::G16EHandOff;
    case G10SecondUseOwnerPhase::Idle:
    case G10SecondUseOwnerPhase::Complete:
    default:
        return G10SecondUseReferenceRole::None;
    }
}

bool IsBarrelStagePhase(const G10SecondUseOwnerPhase phase) noexcept
{
    return phase == G10SecondUseOwnerPhase::BarrelLoadedRoll;
}

void MarkContractFailure(
    G10SecondUseOwnerReceipt& output,
    Status& status,
    const StatusCode code) noexcept
{
    output = G10SecondUseOwnerReceipt{};
    output.reason = G10SecondUseOwnerReason::ContractRejected;
    status.code = code;
}

bool ValidateRoot(
    const DogfightGeometryFrame& frame,
    const G10SecondUseOwnerInput& input,
    Status& status) noexcept
{
    if (input.root_command.label != G10SecondUseCommandLabel::Upstream)
    {
        status.code = StatusCode::InvalidConfiguration;
        return false;
    }
    Status root_status{};
    input.root_command.intent.Validate(root_status);
    if (!root_status.ok())
    {
        status = root_status;
        return false;
    }
    if (!LadyLuck::IsValidControlFrameIdentity(frame.frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            frame.frame_identity,
            input.root_command.intent.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return false;
    }
    return true;
}

bool ReadmitPair(
    const G10BarrelLoadSelectionReceipt& selection,
    const G10OptionalDouble& runtime_nz_feasible_g,
    const bool runtime_nz_source_nonempty,
    const double runtime_roll_limit_radps,
    G10FrozenPairRuntimeAdmission& output,
    Status& status) noexcept
{
    output = G10FrozenPairRuntimeAdmission{};
    status = Status{};
    output.valid = true;

    if (!selection.valid
        || selection.status != G10BarrelLoadSelectionStatus::Selected)
    {
        output.reason = G10FrozenPairRuntimeReason::PairNotSelected;
        return true;
    }
    const double load = selection.selected_load_magnitude_g;
    const double roll = selection.selected_roll_rate_magnitude_radps;
    const double frozen_load_limit = selection.effective_load_limit_g;
    const double frozen_roll_limit = selection.effective_roll_rate_limit_radps;
    if (!std::isfinite(load)
        || !std::isfinite(roll)
        || !std::isfinite(frozen_load_limit)
        || !std::isfinite(frozen_roll_limit)
        || load <= 0.0
        || roll <= 0.0
        || frozen_load_limit <= 0.0
        || frozen_roll_limit <= 0.0
        || !WithinLimit(load, frozen_load_limit)
        || !WithinLimit(roll, frozen_roll_limit))
    {
        output.valid = false;
        output.reason = G10FrozenPairRuntimeReason::ContractRejected;
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    SetOptional(output.frozen_load_limit_g, frozen_load_limit);
    SetOptional(output.frozen_roll_rate_limit_radps, frozen_roll_limit);

    double admitted_load_limit = frozen_load_limit;
    if (runtime_nz_feasible_g.has_value
        && runtime_nz_source_nonempty)
    {
        const double runtime_load = runtime_nz_feasible_g.value;
        if (!std::isfinite(runtime_load) || runtime_load <= 0.0)
        {
            output.valid = false;
            output.reason = G10FrozenPairRuntimeReason::ContractRejected;
            status.code = StatusCode::InvalidArgument;
            return false;
        }
        admitted_load_limit = runtime_load;
    }
    SetOptional(output.runtime_load_limit_g, admitted_load_limit);

    if (!std::isfinite(runtime_roll_limit_radps)
        || runtime_roll_limit_radps <= 0.0)
    {
        output.valid = false;
        output.reason = G10FrozenPairRuntimeReason::ContractRejected;
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    SetOptional(
        output.runtime_roll_rate_limit_radps,
        runtime_roll_limit_radps);

    const double admitted_load = (std::min)(load, admitted_load_limit);
    const double admitted_roll = (std::min)(roll, runtime_roll_limit_radps);
    SetOptional(output.load_magnitude_g, admitted_load);
    SetOptional(output.roll_rate_magnitude_radps, admitted_roll);
    output.admitted = true;
    output.reason = G10FrozenPairRuntimeReason::Admitted;
    output.clamped = admitted_load < load || admitted_roll < roll;
    return true;
}

bool VelocityNormalDirection(
    const Vector3& direction,
    const Vector3& velocity_hat,
    Vector3& output) noexcept
{
    if (!FiniteVector(direction) || !FiniteVector(velocity_hat))
    {
        return false;
    }
    output = Subtract3(direction, Scale3(
        velocity_hat,
        Dot3(direction, velocity_hat)));
    const double magnitude = Norm3(output);
    if (!std::isfinite(magnitude)
        || magnitude <= ScaledTolerance({magnitude}))
    {
        return false;
    }
    output = Scale3(output, 1.0 / magnitude);
    return FiniteVector(output);
}

enum class TransportResult : std::uint8_t
{
    Available = 0U,
    Ambiguous = 1U,
    ContractFault = 2U
};

TransportResult ParallelTransportNormalDirection(
    const Vector3& previous_velocity_source,
    const Vector3& current_velocity_source,
    const Vector3& previous_direction_source,
    Vector3& output) noexcept
{
    Vector3 previous_velocity{};
    Vector3 current_velocity{};
    if (!Unit3(previous_velocity_source, previous_velocity)
        || !Unit3(current_velocity_source, current_velocity))
    {
        return TransportResult::ContractFault;
    }
    Vector3 previous_direction{};
    if (!VelocityNormalDirection(
            previous_direction_source,
            previous_velocity,
            previous_direction))
    {
        return TransportResult::ContractFault;
    }

    Vector3 rotation_axis = Cross3(previous_velocity, current_velocity);
    const double sine_angle = Norm3(rotation_axis);
    const double cosine_angle = (std::max)(
        -1.0,
        (std::min)(1.0, Dot3(previous_velocity, current_velocity)));
    Vector3 transported{};
    if (sine_angle <= ScaledTolerance({sine_angle}))
    {
        if (cosine_angle < 0.0)
        {
            return TransportResult::Ambiguous;
        }
        transported = previous_direction;
    }
    else
    {
        rotation_axis = Scale3(rotation_axis, 1.0 / sine_angle);
        transported = Add3(
            Add3(
                Scale3(previous_direction, cosine_angle),
                Scale3(
                    Cross3(rotation_axis, previous_direction),
                    sine_angle)),
            Scale3(
                rotation_axis,
                (1.0 - cosine_angle)
                    * Dot3(rotation_axis, previous_direction)));
    }
    transported = Subtract3(
        transported,
        Scale3(current_velocity, Dot3(transported, current_velocity)));
    const double transported_norm = Norm3(transported);
    if (!std::isfinite(transported_norm))
    {
        return TransportResult::ContractFault;
    }
    if (transported_norm <= ScaledTolerance({transported_norm}))
    {
        return TransportResult::Ambiguous;
    }
    output = Scale3(transported, 1.0 / transported_norm);
    return TransportResult::Available;
}

bool RotateAboutUnitAxis(
    const Vector3& vector,
    const Vector3& axis_source,
    const double angle_rad,
    Vector3& output) noexcept
{
    Vector3 axis{};
    if (!FiniteVector(vector)
        || !Unit3(axis_source, axis)
        || !std::isfinite(angle_rad))
    {
        return false;
    }
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    output = Add3(
        Add3(
            Scale3(vector, cosine),
            Scale3(Cross3(axis, vector), sine)),
        Scale3(axis, (1.0 - cosine) * Dot3(axis, vector)));
    const double magnitude = Norm3(output);
    if (!std::isfinite(magnitude)
        || magnitude <= ScaledTolerance({magnitude}))
    {
        return false;
    }
    output = Scale3(output, 1.0 / magnitude);
    return true;
}

enum class OptionalGeometryResult : std::uint8_t
{
    Available = 0U,
    Unavailable = 1U,
    ContractFault = 2U
};

OptionalGeometryResult VelocityNormalUpReference(
    const Vector3& velocity,
    Vector3& output) noexcept
{
    Vector3 velocity_hat{};
    if (!Unit3(velocity, velocity_hat))
    {
        return OptionalGeometryResult::ContractFault;
    }
    const Vector3 ned_up{{0.0, 0.0, -1.0}};
    Vector3 projected = Subtract3(
        ned_up,
        Scale3(velocity_hat, Dot3(ned_up, velocity_hat)));
    const double magnitude = Norm3(projected);
    if (!std::isfinite(magnitude))
    {
        return OptionalGeometryResult::ContractFault;
    }
    const double tolerance = ScaledTolerance({magnitude});
    if (magnitude <= tolerance)
    {
        return OptionalGeometryResult::Unavailable;
    }
    projected = Scale3(projected, 1.0 / magnitude);
    const double upward_alignment = Dot3(projected, ned_up);
    if (std::fabs(upward_alignment) <= tolerance)
    {
        return OptionalGeometryResult::Unavailable;
    }
    output = upward_alignment > 0.0
        ? projected
        : Scale3(projected, -1.0);
    return OptionalGeometryResult::Available;
}

OptionalGeometryResult AttitudeVelocityBankDirection(
    const DogfightGeometryFrame& frame,
    Vector3& output) noexcept
{
    const Vector3& velocity = frame.own.velocity_ned_mps;
    const Vector3& angles = frame.own.rpy_rad;
    if (!FiniteVector(velocity) || !FiniteVector(angles))
    {
        return OptionalGeometryResult::ContractFault;
    }
    const double speed = Norm3(velocity);
    if (!std::isfinite(speed))
    {
        return OptionalGeometryResult::ContractFault;
    }
    if (speed <= kAttitudeObservationMinimum)
    {
        return OptionalGeometryResult::Unavailable;
    }
    const Vector3 velocity_hat = Scale3(velocity, 1.0 / speed);
    const double roll = angles[0];
    const double pitch = angles[1];
    const double yaw = angles[2];
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    // Negative third column of Rz(yaw)*Ry(pitch)*Rx(roll).
    const Vector3 body_up_ned{{
        -(cy * sp * cr + sy * sr),
        -(sy * sp * cr - cy * sr),
        -(cp * cr)}};
    Vector3 velocity_normal_up = Subtract3(
        body_up_ned,
        Scale3(velocity_hat, Dot3(body_up_ned, velocity_hat)));
    const double magnitude = Norm3(velocity_normal_up);
    if (!std::isfinite(magnitude))
    {
        return OptionalGeometryResult::ContractFault;
    }
    if (magnitude <= kAttitudeObservationMinimum)
    {
        return OptionalGeometryResult::Unavailable;
    }
    output = Scale3(velocity_normal_up, 1.0 / magnitude);
    return OptionalGeometryResult::Available;
}

bool AdvanceRollProgress(
    const DogfightGeometryFrame& frame,
    const std::int32_t direction_sign,
    const Vector3& observed_bank_direction,
    const double maximum_observable_roll_rate_radps,
    G10VelocityBankRollProgress& progress,
    Status& status) noexcept
{
    status = Status{};
    if (direction_sign != -1 && direction_sign != 1)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    if (!std::isfinite(frame.t_sec)
        || !std::isfinite(maximum_observable_roll_rate_radps)
        || maximum_observable_roll_rate_radps <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    Vector3 velocity_hat{};
    if (!Unit3(frame.own.velocity_ned_mps, velocity_hat))
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    Vector3 current_bank{};
    if (!VelocityNormalDirection(
            observed_bank_direction,
            velocity_hat,
            current_bank))
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    if (!progress.present)
    {
        progress = G10VelocityBankRollProgress{};
        progress.present = true;
        progress.valid = true;
        progress.direction_sign = direction_sign;
        progress.previous_velocity_hat_ned = velocity_hat;
        progress.previous_bank_direction_ned = current_bank;
        progress.previous_observation_time_s = frame.t_sec;
        return true;
    }
    if (!progress.valid)
    {
        return true;
    }
    const double dt = frame.t_sec - progress.previous_observation_time_s;
    if (dt <= 0.0)
    {
        progress.valid = false;
        return true;
    }
    const double maximum_delta = maximum_observable_roll_rate_radps * dt;
    if (maximum_delta >= LadyLuck::constants::Pi)
    {
        progress.valid = false;
        return true;
    }
    Vector3 transported{};
    const TransportResult transport = ParallelTransportNormalDirection(
        progress.previous_velocity_hat_ned,
        velocity_hat,
        progress.previous_bank_direction_ned,
        transported);
    if (transport == TransportResult::ContractFault)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    if (transport == TransportResult::Ambiguous)
    {
        progress.valid = false;
        return true;
    }
    const double delta = std::atan2(
        Dot3(velocity_hat, Cross3(transported, current_bank)),
        Dot3(transported, current_bank));
    const double tolerance = ScaledTolerance({delta, maximum_delta});
    if (std::fabs(delta) > maximum_delta + tolerance)
    {
        progress.valid = false;
        return true;
    }
    const double signed_increment =
        static_cast<double>(progress.direction_sign) * delta;
    progress.progress_rad = (std::max)(
        0.0,
        progress.progress_rad + signed_increment);
    progress.previous_velocity_hat_ned = velocity_hat;
    progress.previous_bank_direction_ned = current_bank;
    progress.previous_observation_time_s = frame.t_sec;
    progress.nominal_270_observed =
        progress.progress_rad >= kNominalRollProgressRad;
    return true;
}

bool ObserveRollAndProviderWindows(
    const DogfightGeometryFrame& frame,
    G10SecondUseOwnerSnapshot& snapshot,
    G10SecondUseProviderWindowReceipt& output,
    Status& status) noexcept
{
    output = G10SecondUseProviderWindowReceipt{};
    status = Status{};

    Vector3 observed_bank{};
    const OptionalGeometryResult observed_result =
        AttitudeVelocityBankDirection(frame, observed_bank);
    if (observed_result == OptionalGeometryResult::ContractFault)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    output.actual_bank_observation_available =
        observed_result == OptionalGeometryResult::Available;

    const G10SecondUseOwnerPhase previous_phase = snapshot.phase;
    const bool pitch_up_command_already_applied =
        snapshot.phase != G10SecondUseOwnerPhase::BarrelPitchUp
        || snapshot.last_observation_time_valid;
    if (output.actual_bank_observation_available
        && pitch_up_command_already_applied)
    {
        if (!AdvanceRollProgress(
                frame,
                snapshot.committed_roll_sign,
                observed_bank,
                snapshot.committed_roll_rate_limit_radps,
                snapshot.roll_progress,
                status))
        {
            return false;
        }
        if (!snapshot.roll_complete
            && snapshot.roll_progress.present
            && (snapshot.roll_progress.nominal_270_observed
                || snapshot.roll_progress.progress_rad
                    >= kNominalRollProgressRad))
        {
            snapshot.roll_complete = true;
        }
    }

    output.barrel_loaded_roll_active =
        snapshot.roll_progress.present && snapshot.roll_progress.valid;
    output.rear_preview_required =
        previous_phase == G10SecondUseOwnerPhase::BarrelLoadedRoll
        || previous_phase
            == G10SecondUseOwnerPhase::DescendingLagReacquire;
    output.valid = true;
    return true;
}

bool BuildEnergyNeutralAimCommand(
    const DogfightGeometryFrame& frame,
    const Vector3& aim_point,
    const double desired_speed_mps,
    const double flight_path_gamma_limit_rad,
    const G10SecondUseCommandLabel label,
    G10SecondUseCommand& output,
    Status& status) noexcept
{
    status = Status{};
    if (!FiniteVector(aim_point)
        || !FiniteVector(frame.own.position_ned_m)
        || !std::isfinite(desired_speed_mps)
        || desired_speed_mps <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const Vector3 path = Subtract3(aim_point, frame.own.position_ned_m);
    if (!(Norm3(path) > 0.0))
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const double gamma_limit = std::fabs(flight_path_gamma_limit_rad);
    if (!std::isfinite(gamma_limit)
        || gamma_limit <= 0.0
        || gamma_limit >= 0.5 * LadyLuck::constants::Pi)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const double raw_gamma = std::atan2(
        -path[2],
        std::hypot(path[0], path[1]));
    const double admitted_gamma = (std::min)(
        gamma_limit,
        (std::max)(-gamma_limit, raw_gamma));
    const double speed_rate =
        -LadyLuck::constants::StandardGravityMps2
        * std::sin(admitted_gamma);
    const double capture_range = frame.own_offense.phase.max_range_m;
    if (!std::isfinite(capture_range) || capture_range <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    ControlIntent candidate{};
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = aim_point;
    candidate.desired_speed_mps = desired_speed_mps;
    candidate.desired_speed_rate_mps2 = speed_rate;
    candidate.path_inversion_allowed.has_value = true;
    candidate.path_inversion_allowed.value = false;
    candidate.capture_range_des_m = capture_range;
    candidate.behavior_id = DoctrineBehaviorId::Invalid;
    candidate.mode_id = DoctrineModeId::Obfm;
    candidate.route_kind = ControlRouteKind::AimPoint;
    candidate.writer_id = LadyLuck::ControlIntentWriterNone;
    output.intent = candidate;
    output.label = label;
    return true;
}

enum class PitchUpBuildResult : std::uint8_t
{
    Available = 0U,
    Unavailable = 1U,
    ContractFault = 2U
};

PitchUpBuildResult BuildPitchUpCommand(
    const DogfightGeometryFrame& frame,
    const Vector3& station_velocity,
    const double gamma_limit,
    G10SecondUseCommand& output,
    Status& status) noexcept
{
    status = Status{};
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity) || !FiniteVector(station_velocity))
    {
        status.code = StatusCode::NonFiniteInput;
        return PitchUpBuildResult::ContractFault;
    }
    const Vector3 own_horizontal{{
        own_velocity[0], own_velocity[1], 0.0}};
    const Vector3 station_horizontal{{
        station_velocity[0], station_velocity[1], 0.0}};
    const double own_horizontal_speed = Norm3(own_horizontal);
    const double station_horizontal_speed = Norm3(station_horizontal);
    if (!std::isfinite(own_horizontal_speed)
        || !std::isfinite(station_horizontal_speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return PitchUpBuildResult::ContractFault;
    }
    if (own_horizontal_speed <= 0.0
        || station_horizontal_speed <= 0.0)
    {
        return PitchUpBuildResult::Unavailable;
    }
    const Vector3 aligned_station = Scale3(
        own_horizontal,
        station_horizontal_speed / own_horizontal_speed);
    const double own_speed = Norm3(own_velocity);
    if (!std::isfinite(own_speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return PitchUpBuildResult::ContractFault;
    }
    if (station_horizontal_speed >= own_speed)
    {
        return PitchUpBuildResult::Unavailable;
    }
    const double up_speed = std::sqrt((std::max)(
        own_speed * own_speed
            - station_horizontal_speed * station_horizontal_speed,
        0.0));
    Vector3 desired_velocity = aligned_station;
    desired_velocity[2] = -up_speed;
    const double desired_norm = Norm3(desired_velocity);
    const double capture_range = frame.own_offense.phase.max_range_m;
    if (!std::isfinite(desired_norm)
        || !std::isfinite(capture_range)
        || !FiniteVector(frame.own.position_ned_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return PitchUpBuildResult::ContractFault;
    }
    if (desired_norm <= 0.0 || capture_range <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return PitchUpBuildResult::ContractFault;
    }
    const Vector3 aim = Add3(
        frame.own.position_ned_m,
        Scale3(desired_velocity, capture_range / desired_norm));
    if (!BuildEnergyNeutralAimCommand(
            frame,
            aim,
            own_speed,
            gamma_limit,
            G10SecondUseCommandLabel::PitchUp,
            output,
            status))
    {
        return PitchUpBuildResult::ContractFault;
    }
    return PitchUpBuildResult::Available;
}

bool BuildWindingBaseCommand(
    const DogfightGeometryFrame& frame,
    const double gamma_limit_source,
    G10SecondUseCommand& output,
    Status& status) noexcept
{
    status = Status{};
    const Vector3& velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(velocity) || !FiniteVector(frame.own.position_ned_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    const double speed = Norm3(velocity);
    const double horizontal_speed = std::hypot(velocity[0], velocity[1]);
    const double gamma_limit = std::fabs(gamma_limit_source);
    if (!std::isfinite(speed)
        || !std::isfinite(horizontal_speed)
        || speed <= 0.0
        || horizontal_speed <= 0.0
        || !std::isfinite(gamma_limit)
        || gamma_limit <= 0.0
        || gamma_limit >= 0.5 * LadyLuck::constants::Pi)
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    const double raw_gamma = std::atan2(-velocity[2], horizontal_speed);
    const double admitted_gamma = (std::min)(
        gamma_limit,
        (std::max)(-gamma_limit, raw_gamma));
    ControlIntent candidate{};
    candidate.frame_identity = frame.frame_identity;
    candidate.aim_point_m = Add3(
        frame.own.position_ned_m,
        Scale3(velocity, kStageAimDistanceM / speed));
    candidate.desired_speed_mps = speed;
    candidate.desired_speed_rate_mps2 =
        -LadyLuck::constants::StandardGravityMps2
        * std::sin(admitted_gamma);
    candidate.capture_range_des_m = kDefaultCaptureRangeM;
    candidate.behavior_id = DoctrineBehaviorId::Invalid;
    candidate.mode_id = DoctrineModeId::ControlZone;
    candidate.route_kind = ControlRouteKind::AimPoint;
    candidate.writer_id = LadyLuck::ControlIntentWriterNone;
    output.intent = candidate;
    output.label = G10SecondUseCommandLabel::PositiveLoadedWinding;
    return true;
}

enum class LoadedRollBuildResult : std::uint8_t
{
    Available = 0U,
    Unavailable = 1U,
    ContractFault = 2U
};

LoadedRollBuildResult BuildLoadedRollCommand(
    const DogfightGeometryFrame& frame,
    const G10FrozenPairRuntimeAdmission& pair,
    const std::int32_t direction_sign,
    const Vector3& bank_seed,
    const bool previous_bank_valid,
    const Vector3& previous_bank,
    const double sample_dt_s,
    const double gamma_limit,
    G10SecondUseCommand& output,
    Vector3& desired_bank_direction,
    Status& status) noexcept
{
    status = Status{};
    if (!pair.valid || !pair.admitted)
    {
        return LoadedRollBuildResult::Unavailable;
    }
    if (direction_sign != -1 && direction_sign != 1)
    {
        status.code = StatusCode::InvalidArgument;
        return LoadedRollBuildResult::ContractFault;
    }
    if (!pair.roll_rate_magnitude_radps.has_value
        || !pair.load_magnitude_g.has_value
        || !pair.runtime_roll_rate_limit_radps.has_value
        || !pair.runtime_load_limit_g.has_value)
    {
        status.code = StatusCode::InvalidConfiguration;
        return LoadedRollBuildResult::ContractFault;
    }
    const double requested_roll = pair.roll_rate_magnitude_radps.value;
    const double requested_load = pair.load_magnitude_g.value;
    const double roll_limit = pair.runtime_roll_rate_limit_radps.value;
    const double load_limit = pair.runtime_load_limit_g.value;
    if (!std::isfinite(sample_dt_s)
        || !std::isfinite(requested_roll)
        || !std::isfinite(requested_load)
        || sample_dt_s <= 0.0
        || requested_roll <= 0.0
        || requested_load <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return LoadedRollBuildResult::ContractFault;
    }
    const double roll_increment = requested_roll * sample_dt_s;
    if (roll_increment <= 0.0
        || roll_increment >= LadyLuck::constants::Pi)
    {
        return LoadedRollBuildResult::Unavailable;
    }
    Vector3 velocity_hat{};
    if (!Unit3(frame.own.velocity_ned_mps, velocity_hat))
    {
        status.code = StatusCode::InvalidArgument;
        return LoadedRollBuildResult::ContractFault;
    }
    const Vector3 commanded_bank = previous_bank_valid
        ? previous_bank
        : bank_seed;
    Vector3 transported{};
    const TransportResult transport = ParallelTransportNormalDirection(
        frame.own.velocity_ned_mps,
        velocity_hat,
        commanded_bank,
        transported);
    if (transport == TransportResult::ContractFault)
    {
        status.code = StatusCode::InvalidArgument;
        return LoadedRollBuildResult::ContractFault;
    }
    if (transport == TransportResult::Ambiguous)
    {
        return LoadedRollBuildResult::Unavailable;
    }
    if (!RotateAboutUnitAxis(
            transported,
            velocity_hat,
            static_cast<double>(direction_sign) * roll_increment,
            desired_bank_direction))
    {
        status.code = StatusCode::InvalidArgument;
        return LoadedRollBuildResult::ContractFault;
    }
    const Vector3 gravity{{
        0.0, 0.0, LadyLuck::constants::StandardGravityMps2}};
    const Vector3 acceleration = Add3(
        gravity,
        Scale3(
            desired_bank_direction,
            requested_load * LadyLuck::constants::StandardGravityMps2));
    if (!WithinLimit(requested_roll, roll_limit)
        || !WithinLimit(requested_load, load_limit))
    {
        return LoadedRollBuildResult::Unavailable;
    }
    const Vector3 specific_force = Subtract3(acceleration, gravity);
    const double parallel = Dot3(specific_force, velocity_hat);
    const double parallel_tolerance = ScaledTolerance(
        {parallel, Norm3(specific_force)});
    if (std::fabs(parallel) > parallel_tolerance)
    {
        return LoadedRollBuildResult::Unavailable;
    }

    G10SecondUseCommand base{};
    if (!BuildWindingBaseCommand(frame, gamma_limit, base, status))
    {
        return LoadedRollBuildResult::ContractFault;
    }
    base.intent.total_load_factor_limit_g.has_value = true;
    base.intent.total_load_factor_limit_g.value = load_limit;
    base.intent.path_inversion_allowed.has_value = true;
    base.intent.path_inversion_allowed.value = true;
    base.intent.direct_load_vector_acceleration_ned_mps2.has_value = true;
    base.intent.direct_load_vector_acceleration_ned_mps2.value = acceleration;
    base.intent.route_kind = ControlRouteKind::DirectLoadVectorAcceleration;
    output = base;
    return LoadedRollBuildResult::Available;
}

bool ComputeConcentricPathPoint(
    const DogfightGeometryFrame& frame,
    const bool previous_velocity_valid,
    const Vector3& previous_velocity,
    const bool previous_time_valid,
    const double previous_time_s,
    Vector3& output,
    Status& status) noexcept
{
    status = Status{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& target_position = frame.opponent.position_ned_m;
    const Vector3& target_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(target_position)
        || !FiniteVector(target_velocity)
        || !std::isfinite(frame.t_sec))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    const double target_speed = Norm3(target_velocity);
    if (!std::isfinite(target_speed))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    if (target_speed < LadyLuck::constants::Tiny)
    {
        output = target_position;
        return true;
    }
    const Vector3 path_direction = Scale3(
        target_velocity,
        1.0 / target_speed);
    const Vector3 offset = Subtract3(own_position, target_position);
    const double foot_parameter = Dot3(offset, path_direction);
    const Vector3 perpendicular = Subtract3(
        offset,
        Scale3(path_direction, foot_parameter));
    const double lag_depth = Norm3(perpendicular);
    Vector3 omega{};
    if (previous_velocity_valid && previous_time_valid)
    {
        if (!FiniteVector(previous_velocity)
            || !std::isfinite(previous_time_s))
        {
            status.code = StatusCode::InvalidConfiguration;
            return false;
        }
        const double dt = frame.t_sec - previous_time_s;
        const double previous_speed = Norm3(previous_velocity);
        if (dt > 0.0
            && std::isfinite(dt)
            && previous_speed >= LadyLuck::constants::Tiny)
        {
            const Vector3 previous_direction = Scale3(
                previous_velocity,
                1.0 / previous_speed);
            const Vector3 cross = Cross3(
                previous_direction,
                path_direction);
            const double sine = Norm3(cross);
            const double cosine = (std::max)(
                -1.0,
                (std::min)(
                    1.0,
                    Dot3(previous_direction, path_direction)));
            const double angle = std::atan2(sine, cosine);
            if (sine >= LadyLuck::constants::Tiny)
            {
                omega = Scale3(cross, angle / (dt * sine));
            }
        }
    }
    const double omega_magnitude = Norm3(omega);
    if (omega_magnitude * target_speed >= LadyLuck::constants::Tiny)
    {
        const double radius = target_speed / omega_magnitude;
        const Vector3 normal = Scale3(omega, 1.0 / omega_magnitude);
        Vector3 centre_direction = Cross3(omega, target_velocity);
        const double centre_direction_norm = Norm3(centre_direction);
        if (!std::isfinite(centre_direction_norm)
            || centre_direction_norm <= 0.0)
        {
            status.code = StatusCode::InvalidArgument;
            return false;
        }
        centre_direction = Scale3(
            centre_direction,
            1.0 / centre_direction_norm);
        const Vector3 centre = Add3(
            target_position,
            Scale3(centre_direction, radius));
        const double arc_angle = lag_depth / radius;
        const Vector3 spoke = Subtract3(target_position, centre);
        const double cosine = std::cos(-arc_angle);
        const double sine = std::sin(-arc_angle);
        const Vector3 rotated = Add3(
            Add3(
                Scale3(spoke, cosine),
                Scale3(Cross3(normal, spoke), sine)),
            Scale3(
                normal,
                Dot3(normal, spoke) * (1.0 - cosine)));
        output = Add3(centre, rotated);
    }
    else
    {
        output = Subtract3(
            target_position,
            Scale3(path_direction, lag_depth));
    }
    if (!FiniteVector(output))
    {
        status.code = StatusCode::NonFiniteInput;
        return false;
    }
    return true;
}

void AdvanceLifecycle(
    G10SecondUseOwnerSnapshot& snapshot,
    const bool machine_crossed,
    const bool external_descending_lag_applied,
    const bool barrel_loaded_roll_active,
    const G10OptionalBool& nominal_270) noexcept
{
    if (snapshot.phase == G10SecondUseOwnerPhase::Complete
        || snapshot.phase == G10SecondUseOwnerPhase::G16EHandOff)
    {
        return;
    }
    if (machine_crossed)
    {
        snapshot.phase = G10SecondUseOwnerPhase::G16EHandOff;
        return;
    }

    switch (snapshot.phase)
    {
    case G10SecondUseOwnerPhase::BarrelPitchUp:
        if (barrel_loaded_roll_active)
        {
            snapshot.phase = G10SecondUseOwnerPhase::BarrelLoadedRoll;
        }
        return;
    case G10SecondUseOwnerPhase::BarrelLoadedRoll:
        if (nominal_270.has_value && nominal_270.value)
        {
            snapshot.phase =
                G10SecondUseOwnerPhase::DescendingLagReacquire;
        }
        return;
    case G10SecondUseOwnerPhase::DescendingLagReacquire:
        if (snapshot.descending_lag_applied
            || external_descending_lag_applied)
        {
            snapshot.phase = G10SecondUseOwnerPhase::Complete;
        }
        return;
    case G10SecondUseOwnerPhase::Idle:
    case G10SecondUseOwnerPhase::Complete:
    case G10SecondUseOwnerPhase::G16EHandOff:
    default:
        return;
    }
}

bool CommitTargetPath(
    const DogfightGeometryFrame& frame,
    const bool pending_rear_valid,
    const Vector3& pending_rear,
    const std::uint64_t pending_sample_index,
    const G10SecondUseCommand* const published_command,
    G10SecondUseTargetPathHistory& history,
    Status& status) noexcept
{
    status = Status{};
    if (!FiniteVector(frame.opponent.velocity_ned_mps)
        || !std::isfinite(frame.t_sec))
    {
        history.previous_target_velocity_valid = false;
        history.previous_time_valid = false;
        return true;
    }
    history.previous_target_velocity_valid = true;
    history.previous_target_velocity_ned_mps =
        frame.opponent.velocity_ned_mps;
    history.previous_time_valid = true;
    history.previous_time_s = frame.t_sec;
    if (published_command != nullptr && pending_rear_valid)
    {
        if (!FiniteVector(pending_rear)
            || !FiniteVector(frame.own.position_ned_m)
            || !std::isfinite(
                published_command->intent.desired_speed_mps))
        {
            status.code = StatusCode::InvalidArgument;
            return false;
        }
        history.previous_rear_attack_point_valid = true;
        history.previous_rear_attack_point_ned_m = pending_rear;
        history.previous_rear_attack_sample_index_valid = true;
        history.previous_rear_attack_sample_index = pending_sample_index;
        history.previous_own_position_valid = true;
        history.previous_own_position_ned_m = frame.own.position_ned_m;
        history.previous_speed_command_valid = true;
        history.previous_speed_command_mps =
            published_command->intent.desired_speed_mps;
    }
    else
    {
        history.previous_rear_attack_point_valid = false;
        history.previous_rear_attack_sample_index_valid = false;
        history.previous_own_position_valid = false;
        history.previous_speed_command_valid = false;
    }
    return true;
}

enum class DescendingBuildResult : std::uint8_t
{
    Available = 0U,
    Unavailable = 1U,
    ContractFault = 2U
};

DescendingBuildResult BuildDescendingLagCommand(
    const DogfightGeometryFrame& frame,
    const G10SecondUseOwnerInput& input,
    const Vector3& rear_point,
    G10SecondUseCommand& output,
    Status& status) noexcept
{
    status = Status{};
    if (!FiniteVector(rear_point)
        || !FiniteVector(input.supply.station_velocity_mps))
    {
        status.code = StatusCode::InvalidConfiguration;
        return DescendingBuildResult::ContractFault;
    }
    if (!std::isfinite(input.root_command.intent.desired_speed_mps)
        || !std::isfinite(
            input.root_command.intent.desired_speed_rate_mps2)
        || input.root_command.intent.desired_speed_mps <= 0.0)
    {
        status.code = StatusCode::InvalidConfiguration;
        return DescendingBuildResult::ContractFault;
    }
    G10SecondUseCommand candidate{};
    if (!BuildEnergyNeutralAimCommand(
            frame,
            rear_point,
            Norm3(frame.own.velocity_ned_mps),
            input.supply.flight_path_gamma_limit_rad,
            G10SecondUseCommandLabel::DescendingLag,
            candidate,
            status))
    {
        return DescendingBuildResult::ContractFault;
    }
    candidate.intent.desired_speed_mps =
        input.root_command.intent.desired_speed_mps;
    candidate.intent.desired_speed_rate_mps2 =
        input.root_command.intent.desired_speed_rate_mps2;
    output = candidate;
    return DescendingBuildResult::Available;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace g10
{

void G10SecondUseOwner::Reset() noexcept
{
    snapshot_ = G10SecondUseOwnerSnapshot{};
}

void G10SecondUseOwner::CopySnapshot(
    G10SecondUseOwnerSnapshot& output) const noexcept
{
    output = snapshot_;
}

void G10SecondUseOwner::Update(
    const DogfightGeometryFrame& frame,
    const G10SecondUseOwnerInput& input,
    G10SecondUseOwnerReceipt& output,
    Status& status) noexcept
{
    output = G10SecondUseOwnerReceipt{};
    status = Status{};
    output.command = input.root_command;

    G10SecondUseReleaseAction release{
        *this,
        input.root_command,
        output};

    if (!input.gate_enabled)
    {
        release(
            G10SecondUseOwnerReason::GateDisabled,
            false,
            G10SecondUseSelectionBinding{},
            G10FrozenPairRuntimeAdmission{});
        return;
    }
    if (!input.root_command_available)
    {
        release(
            G10SecondUseOwnerReason::UpstreamCommandUnavailable,
            true,
            G10SecondUseSelectionBinding{},
            G10FrozenPairRuntimeAdmission{});
        return;
    }
    if (!ValidateRoot(frame, input, status))
    {
        MarkContractFailure(output, status, status.code);
        return;
    }
    if (!input.supply.valid)
    {
        release(
            G10SecondUseOwnerReason::SupplyFault,
            true,
            snapshot_.committed_binding,
            snapshot_.committed_runtime_pair);
        return;
    }
    bool entered = false;
    if (!snapshot_.engaged)
    {
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::BindSelection);
        G10SecondUseSelectionBinding binding{};
        Status binding_status{};
        BindG10SecondUseSelection(
            input.bridge,
            frame,
            input.supply.speed_dump_decision,
            binding,
            binding_status);
        if (!binding_status.ok())
        {
            MarkContractFailure(output, status, binding_status.code);
            return;
        }
        if (!input.bridge.admitted)
        {
            release(
                G10SecondUseOwnerReason::BridgeNotAdmitted,
                true,
                binding,
                G10FrozenPairRuntimeAdmission{});
            return;
        }
        if (!binding.bound
            || !binding.selection_available
            || !binding.barrel_roll_attack_family)
        {
            release(
                G10SecondUseOwnerReason::SelectionNotBound,
                true,
                binding,
                G10FrozenPairRuntimeAdmission{});
            return;
        }

        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::EntryPairReadmission);
        G10FrozenPairRuntimeAdmission entry_pair{};
        Status pair_status{};
        if (!ReadmitPair(
                input.supply.load_selection,
                input.runtime.nz_feasible_g,
                input.runtime.nz_feasible_source_nonempty,
                input.supply.roll_rate_limit_radps,
                entry_pair,
                pair_status))
        {
            MarkContractFailure(output, status, pair_status.code);
            return;
        }
        if (input.bridge.adversary_post_reversal_turn_sign != -1
            && input.bridge.adversary_post_reversal_turn_sign != 1)
        {
            release(
                G10SecondUseOwnerReason::RollDirectionUnresolved,
                true,
                binding,
                entry_pair);
            return;
        }
        snapshot_ = G10SecondUseOwnerSnapshot{};
        snapshot_.engaged = true;
        snapshot_.expected_frame_index =
            frame.frame_identity.frame_index;
        snapshot_.phase = G10SecondUseOwnerPhase::BarrelPitchUp;
        snapshot_.committed_roll_sign =
            -input.bridge.adversary_post_reversal_turn_sign;
        snapshot_.committed_admission = input.bridge;
        snapshot_.committed_binding = binding;
        snapshot_.committed_load_selection =
            input.supply.load_selection;
        snapshot_.committed_roll_rate_limit_radps =
            input.supply.roll_rate_limit_radps;
        snapshot_.committed_nz_source_nonempty =
            input.runtime.nz_feasible_source_nonempty;
        snapshot_.committed_runtime_pair = entry_pair;
        snapshot_.pair_readmission_count = 1U;
        entered = true;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::EntryStateCommitted);
    }

    const G10SecondUseSelectionBinding binding =
        snapshot_.committed_binding;
    if (frame.frame_identity.frame_index
        != snapshot_.expected_frame_index)
    {
        G10FrozenPairRuntimeAdmission previous_pair =
            snapshot_.committed_runtime_pair;
        release(
            G10SecondUseOwnerReason::MachineSampleSequenceBroken,
            true,
            binding,
            previous_pair);
        return;
    }
    AppendOperation(
        output.operation_trace,
        G10SecondUseOperation::SequenceValidated);

    // The selected pair is admitted once on entry.  Route5/FCS performs the
    // authoritative current-envelope shaping; repeating the same tactical
    // pair admission every tick created a second, failure-producing limiter.
    const G10FrozenPairRuntimeAdmission runtime_pair =
        snapshot_.committed_runtime_pair;

    const bool crossed_now = input.supply.moving_body_3_9_crossed;
    if (crossed_now)
    {
        snapshot_.crossing_seen = true;
    }
    else if (snapshot_.crossing_seen && !snapshot_.crossing_rearmed)
    {
        snapshot_.crossing_rearmed = true;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::CrossingStateMutated);
        release(
            G10SecondUseOwnerReason::NoseToTailConversionAchieved,
            true,
            binding,
            runtime_pair);
        return;
    }
    AppendOperation(
        output.operation_trace,
        G10SecondUseOperation::CrossingStateMutated);

    if (!std::isfinite(frame.t_sec)
        || !std::isfinite(input.sample_dt_s)
        || input.sample_dt_s <= 0.0
        || (snapshot_.last_observation_time_valid
            && frame.t_sec <= snapshot_.last_observation_time_s))
    {
        MarkContractFailure(
            output,
            status,
            StatusCode::InvalidDt);
        return;
    }

    bool machine_crossed = false;
    if (!crossed_now)
    {
        snapshot_.crossing_rearmed = true;
    }
    else if (snapshot_.crossing_rearmed)
    {
        machine_crossed = true;
    }

    Vector3 bank_seed{};
    const OptionalGeometryResult seed_result =
        VelocityNormalUpReference(
            frame.own.velocity_ned_mps,
            bank_seed);
    if (seed_result == OptionalGeometryResult::ContractFault)
    {
        MarkContractFailure(
            output,
            status,
            StatusCode::InvalidArgument);
        return;
    }
    if (seed_result == OptionalGeometryResult::Unavailable)
    {
        release(
            G10SecondUseOwnerReason::CommittedMachineReleased,
            true,
            binding,
            runtime_pair);
        return;
    }

    G10SecondUseProviderWindowReceipt provider_windows{};
    Status provider_window_status{};
    if (!ObserveRollAndProviderWindows(
            frame,
            snapshot_,
            provider_windows,
            provider_window_status))
    {
        MarkContractFailure(output, status, provider_window_status.code);
        return;
    }
    AppendOperation(
        output.operation_trace,
        G10SecondUseOperation::RollProgressObserved);

    const bool barrel_loaded_roll_active =
        provider_windows.barrel_loaded_roll_active;
    G10OptionalBool nominal_270{};
    if (barrel_loaded_roll_active)
    {
        nominal_270.has_value = true;
        nominal_270.value =
            snapshot_.roll_progress.nominal_270_observed;
    }
    const G10SecondUseOwnerPhase previous_phase = snapshot_.phase;
    const bool path_required = provider_windows.rear_preview_required;
    bool pending_rear_valid = false;
    Vector3 pending_rear{};
    if (path_required)
    {
        Status rear_status{};
        if (!ComputeConcentricPathPoint(
                frame,
                snapshot_.target_path.previous_target_velocity_valid,
                snapshot_.target_path.previous_target_velocity_ned_mps,
                snapshot_.target_path.previous_time_valid,
                snapshot_.target_path.previous_time_s,
                pending_rear,
                rear_status))
        {
            Reset();
            MarkContractFailure(output, status, rear_status.code);
            return;
        }
        pending_rear_valid = true;
    }

    AdvanceLifecycle(
        snapshot_,
        machine_crossed,
        input.supply.descending_lag_command_applied_before_state,
        barrel_loaded_roll_active,
        nominal_270);
    if (previous_phase
            != G10SecondUseOwnerPhase::DescendingLagReacquire
        && snapshot_.phase
            == G10SecondUseOwnerPhase::DescendingLagReacquire)
    {
        // The current state cannot prove response to a command not yet issued.
        snapshot_.descending_lag_applied = false;
    }
    if (frame.frame_identity.frame_index
        == (std::numeric_limits<std::uint64_t>::max)())
    {
        MarkContractFailure(
            output,
            status,
            StatusCode::InvalidArgument);
        return;
    }
    snapshot_.expected_frame_index =
        frame.frame_identity.frame_index + 1U;
    snapshot_.last_observation_time_valid = true;
    snapshot_.last_observation_time_s = frame.t_sec;
    AppendOperation(
        output.operation_trace,
        G10SecondUseOperation::LifecycleAdvanced);

    G10SecondUseCommand candidate{};
    bool command_available = false;
    bool hold_winding = false;
    const G10SecondUseReferenceRole role = RoleForPhase(snapshot_.phase);
    if (role == G10SecondUseReferenceRole::DescendingLag)
    {
        const DescendingBuildResult result = BuildDescendingLagCommand(
            frame,
            input,
            pending_rear,
            candidate,
            status);
        if (result == DescendingBuildResult::ContractFault)
        {
            Reset();
            MarkContractFailure(output, status, status.code);
            return;
        }
        command_available = result == DescendingBuildResult::Available;
        snapshot_.descending_lag_applied = command_available;
        if (command_available)
        {
            AppendOperation(
                output.operation_trace,
                G10SecondUseOperation::DescendingLagMaterialized);
        }
        Status commit_status{};
        if (!CommitTargetPath(
                frame,
                pending_rear_valid,
                pending_rear,
                frame.frame_identity.frame_index,
                command_available ? &candidate : nullptr,
                snapshot_.target_path,
                commit_status))
        {
            MarkContractFailure(output, status, commit_status.code);
            return;
        }
        ++snapshot_.target_path_commit_count;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::TargetPathCommitted);
    }
    else if (role == G10SecondUseReferenceRole::BarrelRollAttack
        && snapshot_.phase == G10SecondUseOwnerPhase::BarrelPitchUp)
    {
        const PitchUpBuildResult pitch_up_result = BuildPitchUpCommand(
                frame,
                input.supply.station_velocity_mps,
                input.supply.flight_path_gamma_limit_rad,
                candidate,
                status);
        if (pitch_up_result == PitchUpBuildResult::ContractFault)
        {
            MarkContractFailure(output, status, status.code);
            return;
        }
        if (pitch_up_result == PitchUpBuildResult::Unavailable)
        {
            release(
                G10SecondUseOwnerReason::CommittedMachineReleased,
                true,
                binding,
                runtime_pair);
            return;
        }
        command_available = true;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::PitchUpMaterialized);
        Status commit_status{};
        if (!CommitTargetPath(
                frame,
                pending_rear_valid,
                pending_rear,
                frame.frame_identity.frame_index,
                &candidate,
                snapshot_.target_path,
                commit_status))
        {
            MarkContractFailure(output, status, commit_status.code);
            return;
        }
        ++snapshot_.target_path_commit_count;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::TargetPathCommitted);
    }
    else if (role == G10SecondUseReferenceRole::BarrelRollAttack
        && IsBarrelStagePhase(snapshot_.phase))
    {
        if (!snapshot_.roll_complete)
        {
            Vector3 desired_bank{};
            const LoadedRollBuildResult result = BuildLoadedRollCommand(
                frame,
                runtime_pair,
                snapshot_.committed_roll_sign,
                bank_seed,
                snapshot_.previous_commanded_bank_direction_valid,
                snapshot_.previous_commanded_bank_direction_ned,
                input.sample_dt_s,
                input.supply.flight_path_gamma_limit_rad,
                candidate,
                desired_bank,
                status);
            if (result == LoadedRollBuildResult::ContractFault)
            {
                MarkContractFailure(output, status, status.code);
                return;
            }
            command_available = result == LoadedRollBuildResult::Available;
            hold_winding = !command_available;
            if (command_available)
            {
                snapshot_.previous_commanded_bank_direction_valid = true;
                snapshot_.previous_commanded_bank_direction_ned = desired_bank;
                AppendOperation(
                    output.operation_trace,
                    G10SecondUseOperation::WindingMaterialized);
            }
            Status commit_status{};
            if (!CommitTargetPath(
                    frame,
                    pending_rear_valid,
                    pending_rear,
                    frame.frame_identity.frame_index,
                    command_available ? &candidate : nullptr,
                    snapshot_.target_path,
                    commit_status))
            {
                MarkContractFailure(output, status, commit_status.code);
                return;
            }
            ++snapshot_.target_path_commit_count;
            AppendOperation(
                output.operation_trace,
                G10SecondUseOperation::TargetPathCommitted);
        }
    }

    if (!command_available && hold_winding)
    {
        output.valid = true;
        output.evaluated = true;
        output.engaged = false;
        output.commitment_retained = true;
        output.reason = G10SecondUseOwnerReason::WindingCommandWithheld;
        output.binding = binding;
        output.runtime_pair = runtime_pair;
        output.phase = snapshot_.phase;
        // Python _hold() deliberately publishes no selected reference role;
        // only the retained internal snapshot still owns the maneuver.
        output.reference_role = G10SecondUseReferenceRole::None;
        output.command = input.root_command;
        AppendOperation(
            output.operation_trace,
            G10SecondUseOperation::CommandWithheld);
        return;
    }
    if (!command_available)
    {
        release(
            G10SecondUseOwnerReason::CommittedMachineReleased,
            true,
            binding,
            runtime_pair);
        return;
    }

    output.valid = true;
    output.evaluated = true;
    output.engaged = true;
    output.commitment_retained = true;
    output.entered_this_tick = entered;
    output.reason = G10SecondUseOwnerReason::SecondUseCommandPublished;
    output.binding = binding;
    output.runtime_pair = runtime_pair;
    output.phase = snapshot_.phase;
    output.reference_role = role;
    output.command = candidate;
    output.reference_changed = true;
    AppendOperation(
        output.operation_trace,
        G10SecondUseOperation::CommandPublished);
}

} // namespace g10
} // namespace guidance
} // namespace LadyLuck
