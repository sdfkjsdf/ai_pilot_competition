#include "LadyLuck/guidance/obfm/RollDefenseObserver.hpp"

#include "LadyLuck/common/CompensatedDouble.hpp"
#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using LadyLuck::DogfightGeometryFrame;
using LadyLuck::Status;
using LadyLuck::StatusCode;
using LadyLuck::Vector3;
using LadyLuck::common::CompensatedDouble;
using LadyLuck::common::ExactProduct;
using LadyLuck::common::FastSum;
using LadyLuck::guidance::obfm::RollDefenseObservation;
using LadyLuck::guidance::obfm::RollDefenseObservationReason;

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

// Allocation-free n=3 specialization of CPython vector_norm(), used by
// Python's three-argument math.hypot in the d90 resolution authority.
double MathHypot3(const Vector3& value) noexcept
{
    Vector3 coordinates{{
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])}};
    const double maximum = (std::max)(
        coordinates[0],
        (std::max)(coordinates[1], coordinates[2]));
    if (std::isinf(maximum))
    {
        return maximum;
    }
    if (!FiniteVector(coordinates))
    {
        return (std::numeric_limits<double>::quiet_NaN)();
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    const double minimum_normal = (std::numeric_limits<double>::min)();
    if (maximum_exponent < -1023)
    {
        coordinates[0] /= minimum_normal;
        coordinates[1] /= minimum_normal;
        coordinates[2] /= minimum_normal;
        return minimum_normal * MathHypot3(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double compensated_sum = 1.0;
    double fraction_one = 0.0;
    double fraction_two = 0.0;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        const double scaled = coordinates[index] * scale;
        const CompensatedDouble product = ExactProduct(scaled, scaled);
        const CompensatedDouble sum = FastSum(
            compensated_sum,
            product.hi);
        compensated_sum = sum.hi;
        fraction_one += product.lo;
        fraction_two += sum.lo;
    }
    double result = std::sqrt(
        compensated_sum - 1.0 + (fraction_one + fraction_two));
    const CompensatedDouble negative_square = ExactProduct(-result, result);
    const CompensatedDouble corrected_sum = FastSum(
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

bool DirectionConeHalfAngleRad(
    const Vector3& vector,
    const double wire_bound_m,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    const double norm = NumpyNorm3(vector);
    if (!std::isfinite(norm)
        || !std::isfinite(wire_bound_m)
        || wire_bound_m < 0.0)
    {
        return false;
    }
    if (norm <= wire_bound_m)
    {
        return true;
    }
    const double ratio = wire_bound_m / (norm - wire_bound_m);
    output = ratio >= 1.0
        ? std::nextafter(0.5 * LadyLuck::constants::Pi, 0.0)
        : std::nextafter(std::asin(ratio), PositiveInfinity());
    observable = std::isfinite(output);
    return observable;
}

bool HorizontalCourseErrorBoundRad(
    const Vector3& direction,
    const double direction_error_bound_rad,
    bool& observable,
    double& output) noexcept
{
    observable = false;
    output = 0.0;
    if (!FiniteVector(direction)
        || !std::isfinite(direction_error_bound_rad)
        || direction_error_bound_rad < 0.0
        || direction_error_bound_rad > LadyLuck::constants::Pi)
    {
        return false;
    }
    const double magnitude_upper = std::nextafter(
        MathHypot3(direction),
        PositiveInfinity());
    const double horizontal_lower = std::nextafter(
        std::hypot(direction[0], direction[1]),
        0.0);
    if (!std::isfinite(magnitude_upper) || magnitude_upper <= 0.0)
    {
        return false;
    }
    const double horizontal_fraction_lower = std::nextafter(
        horizontal_lower / magnitude_upper,
        0.0);
    if (direction_error_bound_rad >= 0.5 * LadyLuck::constants::Pi
        || horizontal_fraction_lower <= direction_error_bound_rad)
    {
        return true;
    }
    const double ratio_upper = (std::min)(
        1.0,
        std::nextafter(
            direction_error_bound_rad / horizontal_fraction_lower,
            PositiveInfinity()));
    output = std::nextafter(std::asin(ratio_upper), PositiveInfinity());
    observable = std::isfinite(output);
    return observable;
}

double WrappedDeltaRad(
    const double current,
    const double previous) noexcept
{
    double delta = current - previous;
    while (delta > LadyLuck::constants::Pi)
    {
        delta -= 2.0 * LadyLuck::constants::Pi;
    }
    while (delta <= -LadyLuck::constants::Pi)
    {
        delta += 2.0 * LadyLuck::constants::Pi;
    }
    return delta;
}

RollDefenseObservation MakeObservation(
    const RollDefenseObservationReason reason,
    const bool admitted,
    const std::int32_t sign,
    const std::size_t phase_count,
    const bool standing,
    const double envelope_m,
    const bool envelope_mature,
    const double sweep_low_m,
    const double sweep_high_m,
    const bool s3_relative,
    const bool s2_per_phase) noexcept
{
    RollDefenseObservation output{};
    output.valid = true;
    output.reason = reason;
    output.admitted = admitted;
    output.phase_sign = sign;
    output.phase_count = static_cast<std::uint32_t>(phase_count);
    output.standing_resolved = standing;
    output.roll_envelope_m = envelope_m;
    output.envelope_mature = envelope_mature;
    output.sweep_low_m = sweep_low_m;
    output.sweep_high_m = sweep_high_m;
    output.s3_relative = s3_relative;
    output.s2_per_phase = s2_per_phase;
    return output;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* RollDefenseObservationReasonLabel(
    const RollDefenseObservationReason reason) noexcept
{
    switch (reason)
    {
    case RollDefenseObservationReason::AlternationNotEstablished:
        return "alternation_not_established";
    case RollDefenseObservationReason::ReversalNotResolved:
        return "reversal_not_resolved";
    case RollDefenseObservationReason::ForwardSpeedDropNotResolved:
        return "forward_speed_drop_not_resolved";
    case RollDefenseObservationReason::EnergyStandingNotResolved:
        return "energy_standing_not_resolved";
    case RollDefenseObservationReason::RollingDefenseSignatureResolved:
        return "rolling_defense_signature_resolved";
    case RollDefenseObservationReason::ObserverContractRejected:
        return "roll_defense_observer_contract_rejected";
    case RollDefenseObservationReason::StateNotObservable:
    default:
        return "state_not_observable";
    }
}

RollDefenseObserver::RollDefenseObserver() noexcept
{
    Reset();
}

void RollDefenseObserver::Reset() noexcept
{
    phase_signs_ = std::array<std::int32_t, AlternationMinimum>{};
    phase_start_alt_m_ = std::array<double, AlternationMinimum>{};
    phase_start_forward_mps_ =
        std::array<double, AlternationMinimum>{};
    phase_start_forward_bound_mps_ =
        std::array<double, AlternationMinimum>{};
    phase_turn_bits_ = std::array<std::uint8_t, AlternationMinimum>{};
    phase_count_ = 0U;
    completed_excursions_m_ =
        std::array<double, AlternationMinimum>{};
    completed_excursion_count_ = 0U;
    previous_course_present_ = false;
    previous_course_rad_ = 0.0;
    previous_course_bound_rad_ = 0.0;
    phase_start_tick_ = tick_;
    longest_excursion_m_ = 0.0;
    longest_phase_ticks_ = 0U;
    quarantine_sign_ = 0;
}

void RollDefenseObserver::ResetEpisode(const std::int32_t sign) noexcept
{
    phase_signs_ = std::array<std::int32_t, AlternationMinimum>{};
    phase_start_alt_m_ = std::array<double, AlternationMinimum>{};
    phase_start_forward_mps_ =
        std::array<double, AlternationMinimum>{};
    phase_start_forward_bound_mps_ =
        std::array<double, AlternationMinimum>{};
    phase_turn_bits_ = std::array<std::uint8_t, AlternationMinimum>{};
    phase_count_ = 0U;
    completed_excursions_m_ =
        std::array<double, AlternationMinimum>{};
    completed_excursion_count_ = 0U;
    longest_excursion_m_ = 0.0;
    longest_phase_ticks_ = 0U;
    phase_start_tick_ = tick_;
    quarantine_sign_ = sign;
}

void RollDefenseObserver::Update(
    const DogfightGeometryFrame& frame,
    RollDefenseObservation& output,
    Status& status) noexcept
{
    output = RollDefenseObservation{};
    status = Status{};
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(adversary_position)
        || !FiniteVector(adversary_velocity)
        || !FiniteVector(own_position)
        || !FiniteVector(own_velocity))
    {
        Reset();
        return;
    }

    double adversary_velocity_bound = 0.0;
    double own_velocity_bound = 0.0;
    double adversary_position_bound = 0.0;
    double own_position_bound = 0.0;
    if (!Float32WireVectorErrorBound(
            adversary_velocity,
            adversary_velocity_bound)
        || !Float32WireVectorErrorBound(
            own_velocity,
            own_velocity_bound)
        || !Float32WireVectorErrorBound(
            adversary_position,
            adversary_position_bound)
        || !Float32WireVectorErrorBound(
            own_position,
            own_position_bound))
    {
        Reset();
        output.reason = RollDefenseObservationReason::ObserverContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const double down_velocity = adversary_velocity[2];
    std::int32_t sign = 0;
    if (down_velocity - adversary_velocity_bound > 0.0)
    {
        sign = 1;
    }
    else if (down_velocity + adversary_velocity_bound < 0.0)
    {
        sign = -1;
    }
    const double adversary_altitude = -adversary_position[2];
    const double forward_speed = std::hypot(
        adversary_velocity[0],
        adversary_velocity[1]);

    std::int32_t turn_sign = 0;
    bool direction_observable = false;
    double direction_cone_rad = 0.0;
    if (!DirectionConeHalfAngleRad(
            adversary_velocity,
            adversary_velocity_bound,
            direction_observable,
            direction_cone_rad))
    {
        Reset();
        output.reason = RollDefenseObservationReason::ObserverContractRejected;
        status.code = StatusCode::InvalidArgument;
        return;
    }
    bool course_present = false;
    double course_rad = 0.0;
    double course_bound_rad = 0.0;
    if (direction_observable)
    {
        if (!HorizontalCourseErrorBoundRad(
                adversary_velocity,
                direction_cone_rad,
                course_present,
                course_bound_rad))
        {
            Reset();
            output.reason =
                RollDefenseObservationReason::ObserverContractRejected;
            status.code = StatusCode::InvalidArgument;
            return;
        }
        if (course_present)
        {
            course_rad = std::atan2(
                adversary_velocity[1],
                adversary_velocity[0]);
            if (previous_course_present_)
            {
                const double delta = WrappedDeltaRad(
                    course_rad,
                    previous_course_rad_);
                if (std::fabs(delta)
                    > previous_course_bound_rad_ + course_bound_rad)
                {
                    turn_sign = delta > 0.0 ? 1 : -1;
                }
            }
        }
    }
    previous_course_present_ = course_present;
    previous_course_rad_ = course_present ? course_rad : 0.0;
    previous_course_bound_rad_ = course_present ? course_bound_rad : 0.0;

    ++tick_;
    if (phase_count_ > 0U
        && completed_excursion_count_ >= AlternationMinimum)
    {
        const std::uint64_t live_ticks = tick_ - phase_start_tick_;
        const double live_excursion = std::fabs(
            adversary_altitude - phase_start_alt_m_[phase_count_ - 1U]);
        if (live_ticks > 2U * longest_phase_ticks_
            || (sign == phase_signs_[phase_count_ - 1U]
                && live_excursion > 2.0 * longest_excursion_m_))
        {
            ResetEpisode(sign);
        }
    }

    std::int32_t sign_for_phases = sign;
    if (quarantine_sign_ != 0)
    {
        if (sign == -quarantine_sign_)
        {
            quarantine_sign_ = 0;
        }
        else
        {
            sign_for_phases = 0;
        }
    }
    if (quarantine_sign_ == 0)
    {
        sign_for_phases = sign;
    }

    if (sign_for_phases != 0
        && (phase_count_ == 0U
            || phase_signs_[phase_count_ - 1U] != sign_for_phases))
    {
        if (phase_count_ > 0U)
        {
            const std::uint64_t raw_duration = tick_ - phase_start_tick_;
            const std::uint64_t duration = (std::max)(
                static_cast<std::uint64_t>(1U),
                raw_duration);
            longest_phase_ticks_ = (std::max)(
                longest_phase_ticks_,
                duration);
            const double excursion = std::fabs(
                adversary_altitude - phase_start_alt_m_[phase_count_ - 1U]);
            longest_excursion_m_ = (std::max)(
                longest_excursion_m_,
                excursion);
            phase_start_tick_ = tick_;
            if (completed_excursion_count_ < AlternationMinimum)
            {
                completed_excursions_m_[completed_excursion_count_++] =
                    excursion;
            }
            else
            {
                completed_excursions_m_[0] = completed_excursions_m_[1];
                completed_excursions_m_[1] = completed_excursions_m_[2];
                completed_excursions_m_[2] = excursion;
            }
        }
        else
        {
            phase_start_tick_ = tick_;
        }

        if (phase_count_ < AlternationMinimum)
        {
            phase_signs_[phase_count_] = sign_for_phases;
            phase_start_alt_m_[phase_count_] = adversary_altitude;
            phase_start_forward_mps_[phase_count_] = forward_speed;
            phase_start_forward_bound_mps_[phase_count_] =
                adversary_velocity_bound;
            phase_turn_bits_[phase_count_] = 0U;
            ++phase_count_;
        }
        else
        {
            for (std::size_t index = 1U;
                 index < AlternationMinimum;
                 ++index)
            {
                phase_signs_[index - 1U] = phase_signs_[index];
                phase_start_alt_m_[index - 1U] = phase_start_alt_m_[index];
                phase_start_forward_mps_[index - 1U] =
                    phase_start_forward_mps_[index];
                phase_start_forward_bound_mps_[index - 1U] =
                    phase_start_forward_bound_mps_[index];
                phase_turn_bits_[index - 1U] = phase_turn_bits_[index];
            }
            const std::size_t last = AlternationMinimum - 1U;
            phase_signs_[last] = sign_for_phases;
            phase_start_alt_m_[last] = adversary_altitude;
            phase_start_forward_mps_[last] = forward_speed;
            phase_start_forward_bound_mps_[last] =
                adversary_velocity_bound;
            phase_turn_bits_[last] = 0U;
        }
    }
    if (turn_sign != 0 && phase_count_ > 0U)
    {
        phase_turn_bits_[phase_count_ - 1U] |= turn_sign > 0 ? 1U : 2U;
    }

    const double own_speed = NumpyNorm3(own_velocity);
    const double adversary_speed = NumpyNorm3(adversary_velocity);
    const double own_altitude = -own_position[2];
    const double own_energy_lower =
        own_altitude - own_position_bound
        + std::pow((std::max)(0.0, own_speed - own_velocity_bound), 2.0)
            / (2.0 * constants::StandardGravityMps2);
    const double adversary_energy_upper =
        adversary_altitude + adversary_position_bound
        + std::pow(adversary_speed + adversary_velocity_bound, 2.0)
            / (2.0 * constants::StandardGravityMps2);
    const bool standing = own_energy_lower > adversary_energy_upper;

    double envelope_m = 0.0;
    for (std::size_t index = 0U;
         index < completed_excursion_count_;
         ++index)
    {
        envelope_m = (std::max)(envelope_m, completed_excursions_m_[index]);
    }
    const bool envelope_mature =
        completed_excursion_count_ >= AlternationMinimum;
    double sweep_low_m = adversary_altitude;
    double sweep_high_m = adversary_altitude;
    if (phase_count_ > 0U)
    {
        sweep_low_m = phase_start_alt_m_[0];
        sweep_high_m = phase_start_alt_m_[0];
        for (std::size_t index = 1U; index < phase_count_; ++index)
        {
            sweep_low_m = (std::min)(sweep_low_m, phase_start_alt_m_[index]);
            sweep_high_m = (std::max)(sweep_high_m, phase_start_alt_m_[index]);
        }
    }
    const double own_forward_speed = std::hypot(
        own_velocity[0],
        own_velocity[1]);
    const bool s3_relative =
        forward_speed + adversary_velocity_bound
        < own_forward_speed - own_velocity_bound;
    std::uint8_t window_turn_bits = 0U;
    for (std::size_t index = 0U; index < phase_count_; ++index)
    {
        window_turn_bits |= phase_turn_bits_[index];
    }
    bool completed_phases_have_turn = phase_count_ > 1U;
    for (std::size_t index = 0U;
         index + 1U < phase_count_;
         ++index)
    {
        completed_phases_have_turn = completed_phases_have_turn
            && phase_turn_bits_[index] != 0U;
    }
    const bool s2_per_phase = completed_phases_have_turn
        && (window_turn_bits & 1U) != 0U
        && (window_turn_bits & 2U) != 0U;

    if (phase_count_ < AlternationMinimum)
    {
        output = MakeObservation(
            RollDefenseObservationReason::AlternationNotEstablished,
            false,
            sign,
            phase_count_,
            standing,
            envelope_m,
            envelope_mature,
            sweep_low_m,
            sweep_high_m,
            s3_relative,
            s2_per_phase);
        return;
    }
    if ((window_turn_bits & 1U) == 0U
        || (window_turn_bits & 2U) == 0U)
    {
        output = MakeObservation(
            RollDefenseObservationReason::ReversalNotResolved,
            false,
            sign,
            phase_count_,
            standing,
            envelope_m,
            envelope_mature,
            sweep_low_m,
            sweep_high_m,
            s3_relative,
            s2_per_phase);
        return;
    }
    if (!(forward_speed + adversary_velocity_bound
        < phase_start_forward_mps_[0]
            - phase_start_forward_bound_mps_[0]))
    {
        output = MakeObservation(
            RollDefenseObservationReason::ForwardSpeedDropNotResolved,
            false,
            sign,
            phase_count_,
            standing,
            envelope_m,
            envelope_mature,
            sweep_low_m,
            sweep_high_m,
            s3_relative,
            s2_per_phase);
        return;
    }
    if (!standing)
    {
        output = MakeObservation(
            RollDefenseObservationReason::EnergyStandingNotResolved,
            false,
            sign,
            phase_count_,
            false,
            envelope_m,
            envelope_mature,
            sweep_low_m,
            sweep_high_m,
            s3_relative,
            s2_per_phase);
        return;
    }
    output = MakeObservation(
        RollDefenseObservationReason::RollingDefenseSignatureResolved,
        true,
        sign,
        phase_count_,
        true,
        envelope_m,
        envelope_mature,
        sweep_low_m,
        sweep_high_m,
        s3_relative,
        s2_per_phase);
}

bool BarrelRollCounterWithholdsPull(
    const RollDefenseObservation* const observation) noexcept
{
    return observation != nullptr && observation->admitted;
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
