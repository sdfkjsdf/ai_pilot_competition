#include "LadyLuck/guidance/dbfm/DbfmAltitudeSeparated.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
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

// Preserve the frozen NumPy 1.26.4/MKL three-element reduction.
double PythonNorm3(const LadyLuck::Vector3& value) noexcept
{
    const double term0 = value[0] * value[0];
    const double term1 = value[1] * value[1];
    const double term2 = value[2] * value[2];
    return std::sqrt((term0 + term2) + term1);
}

bool ActivationValid(
    const LadyLuck::guidance::dbfm::DbfmAltitudeSeparatedActivation&
        activation) noexcept
{
    return activation.enabled
        == activation.exact_vertical_threat_provenance;
}
}

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{
void EvaluateDbfmAltitudeSeparated(
    const DogfightGeometryFrame& frame,
    const bool dbfm_owner_selected,
    const DbfmAltitudeSeparatedActivation& activation,
    const em::MergeCornerInterval& sustained_corner,
    DbfmAltitudeSeparatedReceipt& output,
    Status& status) noexcept
{
    output = DbfmAltitudeSeparatedReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};

    // Python call order: the DBFM root mode condition precedes the selector,
    // and the altitude-separated Service is not evaluated for another owner.
    if (!dbfm_owner_selected)
    {
        output.reason = DbfmAltitudeSeparatedReason::NonOwner;
        return;
    }
    if (!ActivationValid(activation))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!activation.enabled)
    {
        output.reason = DbfmAltitudeSeparatedReason::ProductionDisabled;
        return;
    }

    output.vertical_threat.evaluated = true;
    if (sustained_corner.status != em::CornerIntervalStatus::Admitted)
    {
        output.reason =
            DbfmAltitudeSeparatedReason::SustainedCornerUnavailable;
        return;
    }
    // An Admitted status declares both interval endpoints ready.  Missing or
    // malformed endpoints are a structural producer contradiction, not an
    // ordinary tactical non-admission.
    if (!sustained_corner.lower_mps.has_value
        || !sustained_corner.upper_mps.has_value)
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!std::isfinite(sustained_corner.lower_mps.value)
        || !std::isfinite(sustained_corner.upper_mps.value))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (sustained_corner.lower_mps.value <= 0.0
        || sustained_corner.upper_mps.value <= 0.0
        || sustained_corner.lower_mps.value
            > sustained_corner.upper_mps.value)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& adversary_position = frame.opponent.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    const Vector3& adversary_velocity = frame.opponent.velocity_ned_mps;
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !FiniteVector(own_position)
        || !FiniteVector(adversary_position)
        || !FiniteVector(own_velocity)
        || !FiniteVector(adversary_velocity))
    {
        output.reason = DbfmAltitudeSeparatedReason::FrameStateUnavailable;
        return;
    }

    const double own_altitude_m = -own_position[2];
    const double adversary_altitude_m = -adversary_position[2];
    const double altitude_gap_m = adversary_altitude_m - own_altitude_m;
    if (!std::isfinite(own_altitude_m)
        || !std::isfinite(adversary_altitude_m)
        || !std::isfinite(altitude_gap_m))
    {
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    const double float32_epsilon =
        static_cast<double>(std::numeric_limits<float>::epsilon());
    const double altitude_scale_m = std::max(
        std::max(std::fabs(own_altitude_m), std::fabs(adversary_altitude_m)),
        1.0);
    const double altitude_band_m = altitude_scale_m * float32_epsilon;
    if (!std::isfinite(altitude_band_m))
    {
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    const bool enemy_above_proven = altitude_gap_m > altitude_band_m;

    const double own_speed_mps = PythonNorm3(own_velocity);
    const double adversary_speed_mps = PythonNorm3(adversary_velocity);
    if (!std::isfinite(own_speed_mps)
        || !std::isfinite(adversary_speed_mps))
    {
        output.reason = DbfmAltitudeSeparatedReason::FrameStateUnavailable;
        return;
    }
    const double own_energy_m = own_altitude_m
        + own_speed_mps * own_speed_mps
            / (2.0 * constants::StandardGravityMps2);
    const double adversary_energy_m = adversary_altitude_m
        + adversary_speed_mps * adversary_speed_mps
            / (2.0 * constants::StandardGravityMps2);
    if (!std::isfinite(own_energy_m)
        || !std::isfinite(adversary_energy_m))
    {
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    const double energy_scale_m = std::max(
        std::max(std::fabs(own_energy_m), std::fabs(adversary_energy_m)),
        1.0);
    const double energy_band_m = energy_scale_m * float32_epsilon;
    if (!std::isfinite(energy_band_m))
    {
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    const bool energy_standing_holds = adversary_energy_m
        >= own_energy_m - energy_band_m;
    const double sustained_speed_mps = sustained_corner.upper_mps.value;
    const double climb_budget_m = std::max(
        0.0,
        (own_speed_mps * own_speed_mps
            - sustained_speed_mps * sustained_speed_mps)
            / (2.0 * constants::StandardGravityMps2));
    if (!std::isfinite(climb_budget_m))
    {
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    const bool unfaceable = enemy_above_proven
        && altitude_gap_m > climb_budget_m + altitude_band_m;

    output.vertical_threat.admitted = true;
    output.vertical_threat.enemy_above_proven = enemy_above_proven;
    output.vertical_threat.energy_standing_holds = energy_standing_holds;
    output.vertical_threat.unfaceable = unfaceable;
    output.vertical_threat.altitude_gap_available = true;
    output.vertical_threat.altitude_gap_m = altitude_gap_m;
    output.vertical_threat.climb_budget_available = true;
    output.vertical_threat.climb_budget_m = climb_budget_m;

    if (!unfaceable || !energy_standing_holds)
    {
        output.reason = DbfmAltitudeSeparatedReason::NotSection4Situation;
        return;
    }

    Vector3 away{{
        own_position[0] - adversary_position[0],
        own_position[1] - adversary_position[1],
        0.0}};
    const double horizontal_away_norm = PythonNorm3(away);
    if (!std::isfinite(horizontal_away_norm))
    {
        output.reason = DbfmAltitudeSeparatedReason::FrameStateUnavailable;
        return;
    }
    // Python Service rejects only the exact overhead degeneracy before the
    // Task.  A nonzero sub-TINY vector would mean the Service declared the Task
    // ready but _horizontal_unit cannot materialize it, so retain that as a
    // contract fault rather than silently inventing an own-nose direction.
    if (horizontal_away_norm < constants::Tiny)
    {
        output.reason =
            DbfmAltitudeSeparatedReason::HorizontalAwayUndefined;
        return;
    }
    const double range_m = frame.enemy_offense.range_m;
    const double capture_range_m = frame.enemy_offense.phase.max_range_m;
    if (!std::isfinite(range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (range_m <= 0.0)
    {
        output.reason = DbfmAltitudeSeparatedReason::ReferenceRangeUnavailable;
        return;
    }
    if (!std::isfinite(capture_range_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    if (capture_range_m <= 0.0)
    {
        output.reason = DbfmAltitudeSeparatedReason::CaptureRangeUnavailable;
        return;
    }
    if (own_speed_mps <= 0.0)
    {
        output.reason = DbfmAltitudeSeparatedReason::OwnSpeedUnavailable;
        return;
    }

    away[0] /= horizontal_away_norm;
    away[1] /= horizontal_away_norm;
    output.aim_point_ned_m = Vector3{{
        own_position[0] + range_m * away[0],
        own_position[1] + range_m * away[1],
        own_position[2]}};
    if (!FiniteVector(output.aim_point_ned_m))
    {
        output.aim_point_ned_m = Vector3{};
        output.reason = DbfmAltitudeSeparatedReason::ArithmeticUnavailable;
        return;
    }
    output.desired_speed_mps = own_speed_mps;
    output.capture_range_des_m = capture_range_m;
    output.candidate_available = true;
    output.reason = DbfmAltitudeSeparatedReason::Selected;
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
