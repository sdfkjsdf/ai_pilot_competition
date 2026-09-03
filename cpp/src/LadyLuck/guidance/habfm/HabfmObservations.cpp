#include "LadyLuck/guidance/habfm/HabfmObservations.hpp"

#include <cmath>
#include <limits>

namespace
{
constexpr double kBodyVelocityQuantumMps = 0.001 * 0.3048;
constexpr double kPlaneInfoFloat32RelativeQuantum = 0x1.0p-23;
constexpr double kStandardGravityMps2 = 9.80665;

template <typename T>
LadyLuck::Result<T> Failure(const LadyLuck::StatusCode code) noexcept
{
    LadyLuck::Result<T> result{};
    result.status.code = code;
    return result;
}

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const LadyLuck::Vector3& value) noexcept
{
    // numpy.linalg.norm on the active float64 3-vector path is sqrt(x.dot(x)).
    const double square_sum = value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2];
    return std::sqrt(square_sum);
}

double Dot3(
    const LadyLuck::Vector3& left,
    const LadyLuck::Vector3& right) noexcept
{
    return left[0] * right[0]
        + left[1] * right[1]
        + left[2] * right[2];
}

struct DoubleLength
{
    double hi = 0.0;
    double lo = 0.0;
};

DoubleLength FastSum(const double a, const double b) noexcept
{
    DoubleLength result{};
    result.hi = a + b;
    result.lo = (a - result.hi) + b;
    return result;
}

DoubleLength ExactProduct(const double left, const double right) noexcept
{
    DoubleLength result{};
    result.hi = left * right;
    result.lo = std::fma(left, right, -result.hi);
    return result;
}

double PythonHypot3(const LadyLuck::Vector3& value) noexcept
{
    // CPython 3.12 vector_norm specialized to the three finite coordinates
    // used by math.hypot(*velocity).  This preserves the active cue's second,
    // accurately rounded speed calculation instead of silently replacing it
    // with the earlier NumPy dot-product norm.
    LadyLuck::Vector3 coordinates{{
        std::fabs(value[0]),
        std::fabs(value[1]),
        std::fabs(value[2])}};
    double maximum = 0.0;
    for (const double coordinate : coordinates)
    {
        if (coordinate > maximum)
        {
            maximum = coordinate;
        }
    }
    if (maximum == 0.0)
    {
        return maximum;
    }

    int maximum_exponent = 0;
    (void)std::frexp(maximum, &maximum_exponent);
    if (maximum_exponent < -1023)
    {
        const double minimum_normal = (std::numeric_limits<double>::min)();
        for (double& coordinate : coordinates)
        {
            coordinate /= minimum_normal;
        }
        return minimum_normal * PythonHypot3(coordinates);
    }

    const double scale = std::ldexp(1.0, -maximum_exponent);
    double corrected_sum = 1.0;
    double fraction_products = 0.0;
    double fraction_sums = 0.0;
    for (double coordinate : coordinates)
    {
        coordinate *= scale;
        const DoubleLength product = ExactProduct(coordinate, coordinate);
        const DoubleLength sum = FastSum(corrected_sum, product.hi);
        corrected_sum = sum.hi;
        fraction_products += product.lo;
        fraction_sums += sum.lo;
    }

    double magnitude = std::sqrt(
        corrected_sum - 1.0 + (fraction_products + fraction_sums));
    const DoubleLength negative_square = ExactProduct(-magnitude, magnitude);
    const DoubleLength residual_sum = FastSum(
        corrected_sum,
        negative_square.hi);
    corrected_sum = residual_sum.hi;
    fraction_products += negative_square.lo;
    fraction_sums += residual_sum.lo;
    const double residual = corrected_sum - 1.0
        + (fraction_products + fraction_sums);
    magnitude += residual / (2.0 * magnitude);
    return magnitude / scale;
}

double VectorQuantumBoundMps() noexcept
{
    return std::sqrt(3.0) * kBodyVelocityQuantumMps;
}

double SpeedEnergyErrorM(const double observed_speed_mps) noexcept
{
    const double quantum_norm = VectorQuantumBoundMps();
    return quantum_norm
        * (2.0 * observed_speed_mps + quantum_norm)
        / (2.0 * kStandardGravityMps2);
}

double AltitudeErrorM(const double observed_altitude_m) noexcept
{
    return std::fabs(observed_altitude_m)
        * kPlaneInfoFloat32RelativeQuantum;
}
}

namespace LadyLuck
{
Result<double> SpecificEnergyM(
    const double altitude_m,
    const double speed_mps) noexcept
{
    if (!std::isfinite(altitude_m) || !std::isfinite(speed_mps))
    {
        return Failure<double>(StatusCode::NonFiniteInput);
    }
    if (speed_mps < 0.0)
    {
        return Failure<double>(StatusCode::InvalidArgument);
    }

    Result<double> result{};
    result.value = altitude_m
        + speed_mps * speed_mps / (2.0 * kStandardGravityMps2);
    return result;
}

Result<double> EnergyEvidenceBandM(
    const double own_altitude_m,
    const double own_speed_mps,
    const double adversary_altitude_m,
    const double adversary_speed_mps) noexcept
{
    if (!std::isfinite(own_altitude_m)
        || !std::isfinite(own_speed_mps)
        || !std::isfinite(adversary_altitude_m)
        || !std::isfinite(adversary_speed_mps))
    {
        return Failure<double>(StatusCode::NonFiniteInput);
    }

    Result<double> result{};
    result.value = SpeedEnergyErrorM(own_speed_mps)
        + SpeedEnergyErrorM(adversary_speed_mps)
        + AltitudeErrorM(own_altitude_m)
        + AltitudeErrorM(adversary_altitude_m);
    return result;
}

Result<HabfmMergeIntentEvidence> EvaluateMergeIntent(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& corner_speed_lower_mps,
    const HabfmOptionalScalar& corner_speed_upper_mps,
    const bool corner_interval_admitted) noexcept
{
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(adversary_velocity))
    {
        return Failure<HabfmMergeIntentEvidence>(
            StatusCode::NonFiniteInput);
    }

    const double speed_mps = NumpyNorm3(adversary_velocity);
    const double speed_error_bound_mps = VectorQuantumBoundMps();
    if (!std::isfinite(speed_mps) || speed_mps < 0.0
        || !std::isfinite(speed_error_bound_mps)
        || speed_error_bound_mps < 0.0)
    {
        return Failure<HabfmMergeIntentEvidence>(
            StatusCode::NonFiniteInput);
    }

    // Keep Python's left-to-right `and` admission contract.  Invalid stored
    // values are fail-closed evidence, not API failures, and are not consumed
    // at all while the caller's admission flag is false.
    const bool admitted = corner_interval_admitted
        && corner_speed_lower_mps.has_value
        && corner_speed_upper_mps.has_value
        && std::isfinite(corner_speed_lower_mps.value)
        && std::isfinite(corner_speed_upper_mps.value)
        && 0.0 < corner_speed_lower_mps.value
        && corner_speed_lower_mps.value <= corner_speed_upper_mps.value;

    Result<HabfmMergeIntentEvidence> result{};
    HabfmMergeIntentEvidence& evidence = result.value;
    evidence.adversary_speed_mps = speed_mps;
    evidence.speed_error_bound_mps = speed_error_bound_mps;
    evidence.evidence_admitted = admitted;
    if (!admitted)
    {
        evidence.intent = HabfmMergeIntentState::NotProven;
        evidence.reason = HabfmMergeIntentReason::
            CornerSpeedEvidenceAbsentDefaultTurnFightPosture;
        return result;
    }

    evidence.corner_speed_lower_mps.has_value = true;
    evidence.corner_speed_lower_mps.value = corner_speed_lower_mps.value;
    evidence.corner_speed_upper_mps.has_value = true;
    evidence.corner_speed_upper_mps.value = corner_speed_upper_mps.value;
    if (speed_mps - speed_error_bound_mps > corner_speed_upper_mps.value)
    {
        evidence.intent = HabfmMergeIntentState::EnergyFightProven;
        evidence.reason = HabfmMergeIntentReason::
            ObservedSpeedExceedsCornerIntervalAtEveryMass;
    }
    else
    {
        evidence.intent = HabfmMergeIntentState::NotProven;
        evidence.reason = HabfmMergeIntentReason::
            ObservedSpeedWithinOrBelowCornerInterval;
    }
    return result;
}

Result<HabfmPursuitCourseEvidence> EvaluatePursuitCourse(
    const DogfightGeometryFrame& frame) noexcept
{
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(adversary_position)
        || !FiniteVector(own_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_velocity))
    {
        return Failure<HabfmPursuitCourseEvidence>(
            StatusCode::NonFiniteInput);
    }

    const Vector3 line_of_sight{{
        adversary_position[0] - own_position[0],
        adversary_position[1] - own_position[1],
        adversary_position[2] - own_position[2]}};
    const double range_m = NumpyNorm3(line_of_sight);
    const double own_speed_mps = NumpyNorm3(own_velocity);
    if (!std::isfinite(range_m) || !std::isfinite(own_speed_mps))
    {
        return Failure<HabfmPursuitCourseEvidence>(
            StatusCode::NonFiniteInput);
    }

    Result<HabfmPursuitCourseEvidence> result{};
    HabfmPursuitCourseEvidence& evidence = result.value;
    const double vector_quantum_mps = VectorQuantumBoundMps();
    // Finite coincident contact or a velocity whose direction is below the
    // PlaneInfo resolution is ordinary unavailable pursuit evidence.  Return
    // before every normalization/division; the visible HABFM geometry
    // Condition will then fall through to the same-frame Root hold.
    if (range_m <= 0.0 || own_speed_mps <= vector_quantum_mps)
    {
        return result;
    }

    const Vector3 line_of_sight_unit{{
        line_of_sight[0] / range_m,
        line_of_sight[1] / range_m,
        line_of_sight[2] / range_m}};
    const double radial_speed = Dot3(
        adversary_velocity,
        line_of_sight_unit);
    const Vector3 transverse_velocity{{
        adversary_velocity[0]
            - radial_speed * line_of_sight_unit[0],
        adversary_velocity[1]
            - radial_speed * line_of_sight_unit[1],
        adversary_velocity[2]
            - radial_speed * line_of_sight_unit[2]}};
    const double transverse_speed_mps = NumpyNorm3(transverse_velocity);
    if (!std::isfinite(transverse_speed_mps))
    {
        return Failure<HabfmPursuitCourseEvidence>(
            StatusCode::NonFiniteInput);
    }
    evidence.transverse_defined = transverse_speed_mps > vector_quantum_mps;
    if (!evidence.transverse_defined)
    {
        return result;
    }

    const Vector3 transverse_unit{{
        transverse_velocity[0] / transverse_speed_mps,
        transverse_velocity[1] / transverse_speed_mps,
        transverse_velocity[2] / transverse_speed_mps}};
    const Vector3 own_course_unit{{
        own_velocity[0] / own_speed_mps,
        own_velocity[1] / own_speed_mps,
        own_velocity[2] / own_speed_mps}};
    evidence.lead_metric = Dot3(own_course_unit, transverse_unit);
    evidence.resolution_bound =
        2.0 * vector_quantum_mps / own_speed_mps
        + 2.0 * vector_quantum_mps / transverse_speed_mps;
    if (!std::isfinite(evidence.lead_metric)
        || !std::isfinite(evidence.resolution_bound)
        || evidence.resolution_bound < 0.0)
    {
        return Failure<HabfmPursuitCourseEvidence>(
            StatusCode::NonFiniteInput);
    }

    if (evidence.lead_metric > evidence.resolution_bound)
    {
        evidence.state = HabfmPursuitCourseState::LeadProven;
    }
    else if (evidence.lead_metric < -evidence.resolution_bound)
    {
        evidence.state = HabfmPursuitCourseState::LagProven;
    }
    return result;
}

Result<HabfmVerticalExcessEvidence> EvaluateVerticalExcess(
    const DogfightGeometryFrame& frame,
    const HabfmOptionalScalar& corner_speed_mps,
    const bool corner_admitted) noexcept
{
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_velocity))
    {
        return Failure<HabfmVerticalExcessEvidence>(
            StatusCode::NonFiniteInput);
    }

    const double own_speed_mps = NumpyNorm3(own_velocity);
    const double speed_error_bound_mps = VectorQuantumBoundMps();
    if (!std::isfinite(own_speed_mps) || own_speed_mps < 0.0)
    {
        return Failure<HabfmVerticalExcessEvidence>(
            StatusCode::NonFiniteInput);
    }
    const bool admitted = corner_admitted
        && corner_speed_mps.has_value
        && std::isfinite(corner_speed_mps.value)
        && corner_speed_mps.value > 0.0;

    Result<HabfmVerticalExcessEvidence> result{};
    HabfmVerticalExcessEvidence& evidence = result.value;
    evidence.own_speed_mps = own_speed_mps;
    evidence.speed_error_bound_mps = speed_error_bound_mps;
    evidence.evidence_admitted = admitted;
    if (!admitted)
    {
        return result;
    }

    evidence.corner_speed_mps.has_value = true;
    evidence.corner_speed_mps.value = corner_speed_mps.value;
    evidence.excess_mps.has_value = true;
    evidence.excess_mps.value = own_speed_mps - corner_speed_mps.value;
    if (!std::isfinite(evidence.excess_mps.value))
    {
        return Failure<HabfmVerticalExcessEvidence>(
            StatusCode::NonFiniteInput);
    }
    if (evidence.excess_mps.value > speed_error_bound_mps)
    {
        evidence.state = HabfmVerticalExcessState::AboveCornerProven;
    }
    else if (evidence.excess_mps.value < -speed_error_bound_mps)
    {
        evidence.state = HabfmVerticalExcessState::BelowCornerProven;
    }
    return result;
}

Result<HabfmPreTaskObservations> EvaluateHabfmPreTaskObservations(
    const DogfightGeometryFrame& frame,
    const HabfmObservationInputs& inputs) noexcept
{
    const Result<HabfmMergeIntentEvidence> merge = EvaluateMergeIntent(
        frame,
        inputs.adversary_corner_speed_lower_mps,
        inputs.adversary_corner_speed_upper_mps,
        inputs.adversary_corner_interval_admitted);
    if (!merge.ok())
    {
        return Failure<HabfmPreTaskObservations>(merge.status.code);
    }

    const Result<HabfmPursuitCourseEvidence> pursuit =
        EvaluatePursuitCourse(frame);
    if (!pursuit.ok())
    {
        return Failure<HabfmPreTaskObservations>(pursuit.status.code);
    }

    const Result<HabfmVerticalExcessEvidence> vertical =
        EvaluateVerticalExcess(
            frame,
            inputs.own_corner_speed_upper_mps,
            inputs.own_corner_interval_admitted);
    if (!vertical.ok())
    {
        return Failure<HabfmPreTaskObservations>(vertical.status.code);
    }

    Result<HabfmPreTaskObservations> result{};
    result.value.merge_intent = merge.value;
    result.value.pursuit_course = pursuit.value;
    result.value.vertical_excess = vertical.value;
    return result;
}

Result<HabfmCheckpointCueEvidence> EvaluateCheckpointCue(
    const DogfightGeometryFrame& frame) noexcept
{
    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& own_nose = frame.own.nose_ned;
    const Vector3& adversary_nose = frame.opponent.nose_ned;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!FiniteVector(own_position)
        || !FiniteVector(adversary_position)
        || !FiniteVector(own_nose)
        || !FiniteVector(adversary_nose)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_velocity))
    {
        return Failure<HabfmCheckpointCueEvidence>(
            StatusCode::NonFiniteInput);
    }

    // Preserve mode_decision_input_from_frame's eager transition closure,
    // including its otherwise-unused finite specific-energy check.
    const Vector3 own_from_adversary{{
        own_position[0] - adversary_position[0],
        own_position[1] - adversary_position[1],
        own_position[2] - adversary_position[2]}};
    const Vector3 adversary_from_own{{
        -own_from_adversary[0],
        -own_from_adversary[1],
        -own_from_adversary[2]}};
    const bool own_behind_adversary =
        Dot3(own_from_adversary, adversary_nose) < 0.0;
    const bool adversary_behind_own =
        Dot3(adversary_from_own, own_nose) < 0.0;

    const double mode_own_speed_mps = NumpyNorm3(own_velocity);
    const double mode_adversary_speed_mps = NumpyNorm3(adversary_velocity);
    const double mode_own_speed_squared =
        mode_own_speed_mps * mode_own_speed_mps;
    const double mode_adversary_speed_squared =
        mode_adversary_speed_mps * mode_adversary_speed_mps;
    if (!std::isfinite(mode_own_speed_squared)
        || !std::isfinite(mode_adversary_speed_squared))
    {
        return Failure<HabfmCheckpointCueEvidence>(
            StatusCode::NonFiniteInput);
    }
    const double mode_own_energy_m = -own_position[2]
        + mode_own_speed_squared / (2.0 * kStandardGravityMps2);
    const double mode_adversary_energy_m = -adversary_position[2]
        + mode_adversary_speed_squared / (2.0 * kStandardGravityMps2);
    const double mode_delta_energy_m =
        mode_own_energy_m - mode_adversary_energy_m;
    if (!std::isfinite(mode_delta_energy_m))
    {
        return Failure<HabfmCheckpointCueEvidence>(
            StatusCode::NonFiniteInput);
    }

    const bool angle_favourable =
        own_behind_adversary && !adversary_behind_own;
    const bool angle_unfavourable =
        adversary_behind_own && !own_behind_adversary;
    const double own_altitude_m = -own_position[2];
    const double adversary_altitude_m = -adversary_position[2];
    const double own_speed_mps = PythonHypot3(own_velocity);
    const double adversary_speed_mps = PythonHypot3(adversary_velocity);
    const Result<double> own_specific_energy = SpecificEnergyM(
        own_altitude_m,
        own_speed_mps);
    if (!own_specific_energy.ok())
    {
        return Failure<HabfmCheckpointCueEvidence>(
            own_specific_energy.status.code);
    }
    const Result<double> adversary_specific_energy = SpecificEnergyM(
        adversary_altitude_m,
        adversary_speed_mps);
    if (!adversary_specific_energy.ok())
    {
        return Failure<HabfmCheckpointCueEvidence>(
            adversary_specific_energy.status.code);
    }
    const double delta_specific_energy_m =
        own_specific_energy.value - adversary_specific_energy.value;
    const Result<double> evidence_band = EnergyEvidenceBandM(
        own_altitude_m,
        own_speed_mps,
        adversary_altitude_m,
        adversary_speed_mps);
    if (!evidence_band.ok())
    {
        return Failure<HabfmCheckpointCueEvidence>(
            evidence_band.status.code);
    }

    const bool energy_deficit_proven =
        delta_specific_energy_m < -evidence_band.value;
    HabfmCheckpointCueState cue = HabfmCheckpointCueState::Neutral;
    if (angle_favourable && !energy_deficit_proven)
    {
        cue = HabfmCheckpointCueState::Winning;
    }
    else if (angle_unfavourable && energy_deficit_proven)
    {
        cue = HabfmCheckpointCueState::Losing;
    }
    if (!std::isfinite(delta_specific_energy_m)
        || !std::isfinite(evidence_band.value)
        || evidence_band.value < 0.0)
    {
        return Failure<HabfmCheckpointCueEvidence>(
            StatusCode::NonFiniteInput);
    }

    Result<HabfmCheckpointCueEvidence> result{};
    result.value.cue = cue;
    result.value.angle_favourable = angle_favourable;
    result.value.angle_unfavourable = angle_unfavourable;
    result.value.delta_specific_energy_m = delta_specific_energy_m;
    result.value.evidence_band_m = evidence_band.value;
    result.value.energy_deficit_proven = energy_deficit_proven;
    return result;
}
}
