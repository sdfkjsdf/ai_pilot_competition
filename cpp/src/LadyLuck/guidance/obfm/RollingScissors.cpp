#include "LadyLuck/guidance/obfm/RollingScissors.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::guidance::obfm::RScissorsHoldSelectionReason;
using LadyLuck::guidance::obfm::RScissorsHoldSelectionReceipt;
using LadyLuck::guidance::obfm::RollDefenseObservation;
using LadyLuck::guidance::obfm::RollDefenseObservationReason;
using LadyLuck::guidance::obfm::RollingScissorsReleaseAction;
using LadyLuck::guidance::obfm::RollingScissorsReleaseReason;
using LadyLuck::guidance::obfm::RollingScissorsReleaseReceipt;

constexpr double kStandardGravityMps2 = 9.80665;

double PositiveInfinity() noexcept
{
    return (std::numeric_limits<double>::infinity)();
}

bool FiniteVector(const Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double Dot3NumpyAssociation(
    const Vector3& left,
    const Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double NumpyNorm3(const Vector3& value) noexcept
{
    return std::sqrt(Dot3NumpyAssociation(value, value));
}

struct DoubleLength
{
    double hi = 0.0;
    double lo = 0.0;
};

DoubleLength DoubleLengthFastSum(
    const double left,
    const double right) noexcept
{
    const double sum = left + right;
    return DoubleLength{sum, (left - sum) + right};
}

DoubleLength DoubleLengthMultiply(
    const double left,
    const double right) noexcept
{
    const double product = left * right;
    return DoubleLength{
        product,
        std::fma(left, right, -product)};
}

template <std::size_t CoordinateCount>
double PythonMathHypot(
    std::array<double, CoordinateCount> coordinates) noexcept
{
    double maximum = 0.0;
    for (std::size_t index = 0U; index < CoordinateCount; ++index)
    {
        coordinates[index] = std::fabs(coordinates[index]);
        maximum = (std::max)(maximum, coordinates[index]);
    }
    if (std::isinf(maximum))
    {
        return maximum;
    }
    if (!std::isfinite(maximum))
    {
        return (std::numeric_limits<double>::quiet_NaN)();
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    const double minimum_normal =
        (std::numeric_limits<double>::min)();
    if (maximum_exponent < -1023)
    {
        for (std::size_t index = 0U; index < CoordinateCount; ++index)
        {
            coordinates[index] /= minimum_normal;
        }
        return minimum_normal * PythonMathHypot(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < CoordinateCount; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const DoubleLength product = DoubleLengthMultiply(scaled, scaled);
        const DoubleLength sum = DoubleLengthFastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const DoubleLength negative_square = DoubleLengthMultiply(
        -result,
        result);
    const DoubleLength corrected_sum = DoubleLengthFastSum(
        compensated_sum,
        negative_square.hi);
    compensated_sum = corrected_sum.hi;
    fraction_one += negative_square.lo;
    fraction_two += corrected_sum.lo;
    const double correction =
        compensated_sum - 1.0 + (fraction_one + fraction_two);
    result += correction / (2.0 * result);
    return result / scale;
}

double MathHypot2(const double x, const double y) noexcept
{
    return PythonMathHypot<2U>(std::array<double, 2U>{{x, y}});
}

double MathHypot3(const Vector3& value) noexcept
{
    return PythonMathHypot<3U>(
        std::array<double, 3U>{{value[0], value[1], value[2]}});
}

bool Float32WireVectorErrorBound(
    const Vector3& value,
    double& output) noexcept
{
    output = 0.0;
    if (!FiniteVector(value))
    {
        return false;
    }
    const float float_infinity =
        (std::numeric_limits<float>::infinity)();
    Vector3 residual{};
    Vector3 full_cell{};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const float represented_f32 = static_cast<float>(value[index]);
        const double represented = static_cast<double>(represented_f32);
        const double upper = static_cast<double>(
            std::nextafter(represented_f32, float_infinity));
        const double lower = static_cast<double>(
            std::nextafter(represented_f32, -float_infinity));
        if (!std::isfinite(represented)
            || !std::isfinite(upper)
            || !std::isfinite(lower))
        {
            return false;
        }
        residual[index] = value[index] - represented;
        full_cell[index] = (std::max)(
            std::fabs(upper - represented),
            std::fabs(represented - lower));
    }
    const double residual_norm = std::nextafter(
        MathHypot3(residual),
        PositiveInfinity());
    const double cell_norm = std::nextafter(
        MathHypot3(full_cell),
        PositiveInfinity());
    output = std::nextafter(
        residual_norm + cell_norm,
        PositiveInfinity());
    return std::isfinite(output);
}

bool SignatureCore(const RollDefenseObservation* const observation) noexcept
{
    return observation != nullptr
        && observation->valid
        && observation->s2_per_phase
        && (observation->admitted
            || observation->reason
                == RollDefenseObservationReason::EnergyStandingNotResolved
            || observation->reason
                == RollDefenseObservationReason::ForwardSpeedDropNotResolved);
}

bool WireBoundOrReject(
    const Vector3& value,
    double& output,
    Status& status) noexcept
{
    if (!Float32WireVectorErrorBound(value, output))
    {
        status.code = StatusCode::InvalidArgument;
        return false;
    }
    return true;
}

void RejectHold(
    RScissorsHoldSelectionReceipt& output,
    const RScissorsHoldSelectionReason reason) noexcept
{
    output = RScissorsHoldSelectionReceipt{};
    output.reason = reason;
}

void RejectReleaseContract(
    RollingScissorsReleaseReceipt& output) noexcept
{
    output = RollingScissorsReleaseReceipt{};
    output.reason = RollingScissorsReleaseReason::ContractRejected;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void EvaluateRollingScissorsSignatureCore(
    const RollDefenseObservation* const observation,
    bool& output,
    Status& status) noexcept
{
    output = SignatureCore(observation);
    status = Status{};
}

void EvaluateRollingScissorsPlaneSeparated(
    const RollingScissorsPlaneSeparationReceipt* const receipt,
    bool& output,
    Status& status) noexcept
{
    output = receipt != nullptr
        && receipt->valid
        && receipt->separation_valid
        && receipt->relation
            == RollingScissorsPlaneRelation::ResolvablySeparated;
    status = Status{};
}

void EvaluateRollingScissorsOwnVerticalActivity(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        return;
    }
    double bound = 0.0;
    if (!WireBoundOrReject(own_velocity, bound, status))
    {
        return;
    }
    output = std::fabs(own_velocity[2]) - bound > 0.0;
}

void EvaluateRollingScissorsOwnClimbing(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        return;
    }
    double bound = 0.0;
    if (!WireBoundOrReject(own_velocity, bound, status))
    {
        return;
    }
    output = own_velocity[2] + bound < 0.0;
}

void EvaluateRollingScissorsMutualClimb(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    if (observation == nullptr
        || !observation->valid
        || observation->phase_sign != -1)
    {
        return;
    }
    EvaluateRollingScissorsOwnClimbing(frame, output, status);
}

void EvaluateRollingScissorsMutualReach(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    if (observation == nullptr || !observation->valid)
    {
        return;
    }
    if (!observation->envelope_mature)
    {
        output = true;
        return;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    if (!FiniteVector(own_position) || !FiniteVector(adversary_position))
    {
        return;
    }
    double own_bound = 0.0;
    double adversary_bound = 0.0;
    if (!WireBoundOrReject(own_position, own_bound, status)
        || !WireBoundOrReject(adversary_position, adversary_bound, status))
    {
        return;
    }
    const double envelope = (std::max)(
        0.0,
        observation->roll_envelope_m);
    const double low = observation->sweep_low_m - envelope;
    const double high = observation->sweep_high_m + envelope;
    const double own_altitude = -own_position[2];
    const double adversary_altitude = -adversary_position[2];
    const bool own_outside = own_altitude - own_bound > high
        || own_altitude + own_bound < low;
    const bool adversary_outside =
        adversary_altitude - adversary_bound > high
        || adversary_altitude + adversary_bound < low;
    output = !(own_outside || adversary_outside);
}

void EvaluateRollingScissorsStandingReversed(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity))
    {
        return;
    }
    double own_position_bound = 0.0;
    double own_velocity_bound = 0.0;
    double adversary_position_bound = 0.0;
    double adversary_velocity_bound = 0.0;
    if (!WireBoundOrReject(own_position, own_position_bound, status)
        || !WireBoundOrReject(own_velocity, own_velocity_bound, status)
        || !WireBoundOrReject(
            adversary_position,
            adversary_position_bound,
            status)
        || !WireBoundOrReject(
            adversary_velocity,
            adversary_velocity_bound,
            status))
    {
        return;
    }
    const double own_speed = NumpyNorm3(own_velocity);
    const double adversary_speed = NumpyNorm3(adversary_velocity);
    const double own_speed_upper = own_speed + own_velocity_bound;
    const double adversary_speed_lower = (std::max)(
        0.0,
        adversary_speed - adversary_velocity_bound);
    const double own_upper =
        (-own_position[2] + own_position_bound)
        + own_speed_upper * own_speed_upper
            / (2.0 * kStandardGravityMps2);
    const double adversary_lower =
        (-adversary_position[2] - adversary_position_bound)
        + adversary_speed_lower * adversary_speed_lower
            / (2.0 * kStandardGravityMps2);
    output = adversary_lower > own_upper;
}

void EvaluateRollingScissorsOwnPushedAhead(
    const DogfightGeometryFrame& frame,
    bool& output,
    Status& status) noexcept
{
    output = false;
    status = Status{};
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity))
    {
        return;
    }
    double own_position_bound = 0.0;
    double adversary_position_bound = 0.0;
    double adversary_velocity_bound = 0.0;
    if (!WireBoundOrReject(own_position, own_position_bound, status)
        || !WireBoundOrReject(
            adversary_position,
            adversary_position_bound,
            status)
        || !WireBoundOrReject(
            adversary_velocity,
            adversary_velocity_bound,
            status))
    {
        return;
    }
    const double adversary_horizontal = MathHypot2(
        adversary_velocity[0],
        adversary_velocity[1]);
    if (!(adversary_horizontal - adversary_velocity_bound > 0.0))
    {
        return;
    }
    const double course_north =
        adversary_velocity[0] / adversary_horizontal;
    const double course_east =
        adversary_velocity[1] / adversary_horizontal;
    const double along =
        (own_position[0] - adversary_position[0]) * course_north
        + (own_position[1] - adversary_position[1]) * course_east;
    output = along - (own_position_bound + adversary_position_bound) > 0.0;
}

const char* RScissorsHoldSelectionReasonLabel(
    const RScissorsHoldSelectionReason reason) noexcept
{
    switch (reason)
    {
    case RScissorsHoldSelectionReason::StateNotFinite:
        return "state_not_finite";
    case RScissorsHoldSelectionReason::AdversaryHorizontalSpeedNotResolved:
        return "adversary_horizontal_speed_not_resolved";
    case RScissorsHoldSelectionReason::HoldMaterialized:
        return "r_scissors_hold_materialized";
    case RScissorsHoldSelectionReason::ContractRejected:
        return "r_scissors_hold_contract_rejected";
    case RScissorsHoldSelectionReason::SignatureCoreNotResolved:
    default:
        return "signature_core_not_resolved";
    }
}

const char* RScissorsHoldBehaviorLabel() noexcept
{
    return "R_SCISSORS_HOLD";
}

void MaterializeRScissorsHold(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    RScissorsHoldSelectionReceipt& output,
    Status& status) noexcept
{
    output = RScissorsHoldSelectionReceipt{};
    status = Status{};
    if (!SignatureCore(observation))
    {
        RejectHold(
            output,
            RScissorsHoldSelectionReason::SignatureCoreNotResolved);
        return;
    }
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity))
    {
        RejectHold(output, RScissorsHoldSelectionReason::StateNotFinite);
        return;
    }
    double adversary_velocity_bound = 0.0;
    if (!WireBoundOrReject(
            adversary_velocity,
            adversary_velocity_bound,
            status))
    {
        RejectHold(output, RScissorsHoldSelectionReason::ContractRejected);
        return;
    }
    const double adversary_horizontal = MathHypot2(
        adversary_velocity[0],
        adversary_velocity[1]);
    if (!(adversary_horizontal - adversary_velocity_bound > 0.0))
    {
        RejectHold(
            output,
            RScissorsHoldSelectionReason::
                AdversaryHorizontalSpeedNotResolved);
        return;
    }
    const double envelope = (std::max)(
        0.0,
        observation->roll_envelope_m);
    const double adversary_altitude = -adversary_position[2];
    output.selected = true;
    output.reason = RScissorsHoldSelectionReason::HoldMaterialized;
    output.overlay.valid = true;
    output.overlay.aim_point_m = Vector3{{
        adversary_position[0],
        adversary_position[1],
        -(adversary_altitude + envelope)}};
    output.overlay.desired_speed_mps = adversary_horizontal;
    output.overlay.desired_speed_rate_mps2 = 0.0;
}

const char* RollingScissorsReleaseReasonLabel(
    const RollingScissorsReleaseReason reason) noexcept
{
    switch (reason)
    {
    case RollingScissorsReleaseReason::Maintained:
        return "maintained";
    case RollingScissorsReleaseReason::ScissorsLost:
        return "scissors_lost";
    case RollingScissorsReleaseReason::StandingReversed:
        return "standing_reversed";
    case RollingScissorsReleaseReason::BandExit:
        return "band_exit";
    case RollingScissorsReleaseReason::SignatureDropped:
        return "signature_dropped";
    case RollingScissorsReleaseReason::MutualReachExit:
        return "mutual_reach_exit";
    case RollingScissorsReleaseReason::ContractRejected:
        return "rolling_scissors_release_contract_rejected";
    case RollingScissorsReleaseReason::StateInvalid:
    default:
        return "state_invalid";
    }
}

void EvaluateRollingScissorsRelease(
    const DogfightGeometryFrame& frame,
    const RollDefenseObservation* const observation,
    const bool in_engagement_band,
    RollingScissorsReleaseReceipt& output,
    Status& status) noexcept
{
    output = RollingScissorsReleaseReceipt{};
    status = Status{};
    output.evaluated = true;
    if (observation == nullptr || !observation->valid)
    {
        output.action = RollingScissorsReleaseAction::Release;
        output.reason = RollingScissorsReleaseReason::StateInvalid;
        return;
    }

    EvaluateRollingScissorsOwnPushedAhead(
        frame,
        output.own_pushed_ahead,
        status);
    if (!status.ok())
    {
        RejectReleaseContract(output);
        return;
    }
    if (output.own_pushed_ahead)
    {
        output.action = RollingScissorsReleaseAction::Release;
        output.reason = RollingScissorsReleaseReason::ScissorsLost;
        return;
    }

    EvaluateRollingScissorsStandingReversed(
        frame,
        output.standing_reversed,
        status);
    if (!status.ok())
    {
        RejectReleaseContract(output);
        return;
    }
    if (output.standing_reversed)
    {
        output.action = RollingScissorsReleaseAction::Release;
        output.reason = RollingScissorsReleaseReason::StandingReversed;
        return;
    }

    output.engagement_band_resolved = in_engagement_band;
    if (!in_engagement_band)
    {
        output.action = RollingScissorsReleaseAction::Release;
        output.reason = RollingScissorsReleaseReason::BandExit;
        return;
    }

    output.signature_core = SignatureCore(observation);
    if (!output.signature_core)
    {
        output.action = RollingScissorsReleaseAction::Suspend;
        output.reason = RollingScissorsReleaseReason::SignatureDropped;
        return;
    }

    EvaluateRollingScissorsMutualReach(
        frame,
        observation,
        output.mutual_reach,
        status);
    if (!status.ok())
    {
        RejectReleaseContract(output);
        return;
    }
    if (!output.mutual_reach)
    {
        output.action = RollingScissorsReleaseAction::Suspend;
        output.reason = RollingScissorsReleaseReason::MutualReachExit;
        return;
    }

    output.action = RollingScissorsReleaseAction::Maintain;
    output.reason = RollingScissorsReleaseReason::Maintained;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
