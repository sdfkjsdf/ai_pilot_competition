#include "LadyLuck/guidance/obfm/TerminalAiming.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double BankAngleGain = 2.0;
constexpr double MaximumBankCommandRad = 1.4;
constexpr double RollRateGain = 1.0;
constexpr double PitchRateGain = 0.8;
constexpr double MaximumRollRateCommandRadps = 2.0;
constexpr double MinimumLoadFactorCommandG = -1.0;
constexpr double MaximumLoadFactorCommandG = 6.0;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteMatrix(const LadyLuck::Matrix3RowMajor& value) noexcept
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

double PythonDot3(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + (left[1] * right[1] + left[2] * right[2]);
}

double PythonNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(PythonDot3(value, value));
}

double PythonMathHypot3(const LadyLuck::Vector3& value) noexcept
{
    // Exact finite three-coordinate specialization of CPython 3.11.15
    // mathmodule.c vector_norm(). d90 calls math.hypot(*vector) only when the
    // established NumPy norm underflows to zero; nesting two-argument hypot
    // differs from that n-ary implementation by one ULP for some inputs.
    constexpr double Splitter = 134217729.0;
    const double coordinates[3] = {
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])};
    const double maximum = std::max(
        coordinates[0],
        std::max(coordinates[1], coordinates[2]));
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    std::frexp(maximum, &maximum_exponent);
    double correction_sum = 1.0;
    double fraction_one = 0.0;
    if (maximum_exponent >= -1023)
    {
        const double scale = std::ldexp(1.0, -maximum_exponent);
        double fraction_two = 0.0;
        double fraction_three = 0.0;
        for (const double coordinate : coordinates)
        {
            double x = coordinate * scale;
            const double split = x * Splitter;
            const double high = split - (split - x);
            const double low = x - high;

            x = high * high;
            double old_sum = correction_sum;
            correction_sum += x;
            fraction_one += (old_sum - correction_sum) + x;

            x = 2.0 * high * low;
            old_sum = correction_sum;
            correction_sum += x;
            fraction_two += (old_sum - correction_sum) + x;

            fraction_three += low * low;
        }
        double root = std::sqrt(
            correction_sum - 1.0
            + (fraction_one + fraction_two + fraction_three));

        double x = root;
        const double split = x * Splitter;
        const double high = split - (split - x);
        const double low = x - high;

        x = -high * high;
        double old_sum = correction_sum;
        correction_sum += x;
        fraction_one += (old_sum - correction_sum) + x;

        x = -2.0 * high * low;
        old_sum = correction_sum;
        correction_sum += x;
        fraction_two += (old_sum - correction_sum) + x;

        x = -low * low;
        old_sum = correction_sum;
        correction_sum += x;
        fraction_three += (old_sum - correction_sum) + x;

        x = correction_sum - 1.0
            + (fraction_one + fraction_two + fraction_three);
        return (root + x / (2.0 * root)) / scale;
    }

    for (const double coordinate : coordinates)
    {
        double x = coordinate / maximum;
        x *= x;
        const double old_sum = correction_sum;
        correction_sum += x;
        fraction_one += (old_sum - correction_sum) + x;
    }
    return maximum * std::sqrt(correction_sum - 1.0 + fraction_one);
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

LadyLuck::Vector3 Negate(const LadyLuck::Vector3& value) noexcept
{
    return LadyLuck::Vector3{{-value[0], -value[1], -value[2]}};
}

LadyLuck::Vector3 Divide(
    const LadyLuck::Vector3& value,
    const double divisor) noexcept
{
    return LadyLuck::Vector3{{
        value[0] / divisor,
        value[1] / divisor,
        value[2] / divisor}};
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

LadyLuck::Vector3 TransposeMultiply(
    const LadyLuck::Matrix3RowMajor& matrix,
    const LadyLuck::Vector3& vector) noexcept
{
    return LadyLuck::Vector3{{
        matrix[0] * vector[0]
            + (matrix[3] * vector[1] + matrix[6] * vector[2]),
        matrix[1] * vector[0]
            + (matrix[4] * vector[1] + matrix[7] * vector[2]),
        matrix[2] * vector[0]
            + (matrix[5] * vector[1] + matrix[8] * vector[2])}};
}

double Clip(
    const double value,
    const double lower,
    const double upper) noexcept
{
    return std::max(lower, std::min(value, upper));
}

double WrapRadians(const double value) noexcept
{
    return std::atan2(std::sin(value), std::cos(value));
}

void Fail(
    LadyLuck::guidance::obfm::TerminalTrackingReceipt& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::guidance::obfm::TerminalTrackingReceipt{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void EvaluateTerminalTracking(
    const DogfightGeometryFrame& frame,
    const bool enabled,
    const bool eligible_obfm_pursuit_behavior,
    TerminalTrackingReceipt& output,
    Status& status) noexcept
{
    output = TerminalTrackingReceipt{};
    status = Status{};

    // d90 returns the unmodified command before touching geometry when the
    // feature or the LAG/FOLLOW/EMPLOY behavior admission is absent.
    if (!enabled || !eligible_obfm_pursuit_behavior)
    {
        output.evaluated = true;
        return;
    }

    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }

    const double official_range_m = frame.own_offense.range_m;
    const double official_phase_max_m =
        frame.own_offense.phase.max_range_m;
    if (!std::isfinite(official_range_m)
        || !std::isfinite(official_phase_max_m))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    if (official_range_m < 0.0 || official_phase_max_m <= 0.0)
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    output.frame_identity = frame.frame_identity;
    output.evaluated = true;
    if (official_range_m == 0.0
        || official_range_m >= official_phase_max_m)
    {
        return;
    }

    const Vector3& target_position = frame.opponent.position_ned_m;
    const Vector3& target_velocity = frame.opponent.velocity_ned_mps;
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& own_down = frame.own.down_ned;
    const Matrix3RowMajor& body_to_ned = frame.own.dcm_body_to_ned;
    const Vector3& rpy = frame.own.rpy_rad;
    if (!FiniteVector(target_position)
        || !FiniteVector(target_velocity)
        || !FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(own_down)
        || !FiniteMatrix(body_to_ned)
        || !FiniteVector(rpy))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const Vector3 line_of_sight =
        Subtract(target_position, own_position);
    const Vector3 lift_hat = Negate(own_down);
    double range_m = PythonNorm3(line_of_sight);
    double speed_mps = PythonNorm3(own_velocity);
    if (range_m == 0.0
        && (line_of_sight[0] != 0.0
            || line_of_sight[1] != 0.0
            || line_of_sight[2] != 0.0))
    {
        range_m = PythonMathHypot3(line_of_sight);
    }
    if (speed_mps == 0.0
        && (own_velocity[0] != 0.0
            || own_velocity[1] != 0.0
            || own_velocity[2] != 0.0))
    {
        speed_mps = PythonMathHypot3(own_velocity);
    }
    if (!std::isfinite(range_m) || !std::isfinite(speed_mps))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const Vector3 ned_up{{0.0, 0.0, -1.0}};
    const double level_compensation_g =
        PythonDot3(lift_hat, ned_up);
    if (range_m == 0.0 || speed_mps == 0.0)
    {
        return;
    }

    const Vector3 relative_velocity =
        Subtract(target_velocity, own_velocity);
    if (!FiniteVector(relative_velocity))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const double range_squared = range_m * range_m;
    Vector3 line_of_sight_rate{};
    if (std::isfinite(range_squared) && range_squared > 0.0)
    {
        line_of_sight_rate = Divide(
            Cross(line_of_sight, relative_velocity),
            range_squared);
    }
    else
    {
        line_of_sight_rate = Divide(
            Cross(Divide(line_of_sight, range_m), relative_velocity),
            range_m);
    }
    if (!FiniteVector(line_of_sight_rate))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const Vector3 line_of_sight_rate_body =
        TransposeMultiply(body_to_ned, line_of_sight_rate);
    if (!FiniteVector(line_of_sight_rate_body))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    const double bearing_command_rad = std::atan2(
        line_of_sight[1],
        line_of_sight[0]);
    const double bearing_error_rad =
        WrapRadians(bearing_command_rad - rpy[2]);
    const double bank_feedforward_rad = std::atan2(
        speed_mps * line_of_sight_rate[2],
        constants::StandardGravityMps2);
    double roll_command_rad = Clip(
        BankAngleGain * bearing_error_rad,
        -MaximumBankCommandRad,
        MaximumBankCommandRad) + bank_feedforward_rad;
    roll_command_rad = Clip(
        roll_command_rad,
        -MaximumBankCommandRad,
        MaximumBankCommandRad);
    const double pitch_command_rad = std::asin(Clip(
        -line_of_sight[2] / range_m,
        -1.0,
        1.0));

    double p_command_radps = RollRateGain
        * WrapRadians(roll_command_rad - rpy[0])
        + line_of_sight_rate_body[0];
    p_command_radps = Clip(
        p_command_radps,
        -MaximumRollRateCommandRadps,
        MaximumRollRateCommandRadps);
    const double q_desired_radps = PitchRateGain
        * WrapRadians(pitch_command_rad - rpy[1])
        + line_of_sight_rate_body[1];
    double nz_command_g = level_compensation_g
        + speed_mps * q_desired_radps
            / constants::StandardGravityMps2;
    nz_command_g = Clip(
        nz_command_g,
        MinimumLoadFactorCommandG,
        MaximumLoadFactorCommandG);
    if (!std::isfinite(p_command_radps)
        || !std::isfinite(nz_command_g))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    output.admitted = true;
    output.p_cmd_radps = p_command_radps;
    output.nz_cmd_g = nz_command_g;
}

void ApplyTerminalTracking(
    const ControlIntent& upstream,
    const TerminalTrackingReceipt& receipt,
    ControlIntent& output,
    Status& status) noexcept
{
    output.Clear();
    status = Status{};
    upstream.Validate(status);
    if (!status.ok())
    {
        return;
    }
    if (!receipt.evaluated)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!receipt.admitted)
    {
        output = upstream;
        return;
    }
    if (!IsValidControlFrameIdentity(receipt.frame_identity)
        || !SameControlFrameIdentity(
            receipt.frame_identity,
            upstream.frame_identity)
        || !std::isfinite(receipt.p_cmd_radps)
        || !std::isfinite(receipt.nz_cmd_g))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    output = upstream;
    output.direct_p_cmd_radps.has_value = true;
    output.direct_p_cmd_radps.value = receipt.p_cmd_radps;
    output.direct_nz_cmd_g.has_value = true;
    output.direct_nz_cmd_g.value = receipt.nz_cmd_g;
    output.route_kind = ControlRouteKind::DirectBodyReferences;
    output.Validate(status);
    if (!status.ok())
    {
        output.Clear();
    }
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
