#include "LadyLuck/guidance/obfm/ObfmNearStationCarrot.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool CheckedAdd(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right > 0.0 && left >= maximum - right)
        || (right < 0.0 && left <= -maximum - right))
    {
        return false;
    }
    output = left + right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedSubtract(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double maximum = (std::numeric_limits<double>::max)();
    if ((right < 0.0 && left >= maximum + right)
        || (right > 0.0 && left <= -maximum + right))
    {
        return false;
    }
    output = left - right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

bool CheckedMultiply(
    const double left,
    const double right,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    const double left_absolute = std::fabs(left);
    const double right_absolute = std::fabs(right);
    const double maximum = (std::numeric_limits<double>::max)();
    if ((left_absolute > 1.0
            && right_absolute >= maximum / left_absolute)
        || (right_absolute > 1.0
            && left_absolute >= maximum / right_absolute))
    {
        return false;
    }
    output = left * right;
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

// Admit std::hypot before invoking it. The final libm call is retained so
// every ordinary finite result stays bit-identical to the frozen authority.
bool CheckedHypot2(
    const double first,
    const double second,
    double& output) noexcept
{
    output = 0.0;
    if (!std::isfinite(first) || !std::isfinite(second))
    {
        return false;
    }
    const double scale = (std::max)(std::fabs(first), std::fabs(second));
    if (scale == 0.0)
    {
        return true;
    }
    const double normalized_first = first / scale;
    const double normalized_second = second / scale;
    double first_square = 0.0;
    double second_square = 0.0;
    double sum = 0.0;
    if (!CheckedMultiply(
            normalized_first,
            normalized_first,
            first_square)
        || !CheckedMultiply(
            normalized_second,
            normalized_second,
            second_square)
        || !CheckedAdd(first_square, second_square, sum))
    {
        return false;
    }
    const double root = std::sqrt(sum);
    double admitted_result = 0.0;
    if (!std::isfinite(root)
        || !CheckedMultiply(scale, root, admitted_result))
    {
        return false;
    }
    output = std::hypot(first, second);
    if (!std::isfinite(output))
    {
        output = 0.0;
        return false;
    }
    return true;
}

void CopyIntentBytes(
    const LadyLuck::ControlIntent& source,
    LadyLuck::ControlIntent& destination) noexcept
{
    static_assert(
        std::is_trivially_copyable<LadyLuck::ControlIntent>::value,
        "Byte-preserving containment requires a trivially copyable intent.");
    // In-place use is permitted at the future leaf seam; memmove keeps the
    // byte-preserving contract defined even when source and destination alias.
    std::memmove(&destination, &source, sizeof(source));
}

void PreserveBase(
    LadyLuck::guidance::obfm::ObfmNearStationCarrotReceipt& receipt,
    const LadyLuck::guidance::obfm::ObfmNearStationCarrotReason reason,
    LadyLuck::Status& status) noexcept
{
    receipt.applied = false;
    receipt.base_preserved = true;
    receipt.reason = reason;
    status = LadyLuck::Status{};
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

void ShapeObfmNearStationCarrotAimPoint(
    const ControlIntent& base_intent,
    const ObfmNearStationCarrotInput& input,
    ObfmNearStationCarrotReceipt& receipt,
    ControlIntent& output_intent,
    Status& status) noexcept
{
    CopyIntentBytes(base_intent, output_intent);
    receipt = ObfmNearStationCarrotReceipt{};
    status = Status{};

    // Non-owners do not evaluate evidence and cannot veto another producer.
    if (!input.owner_selected)
    {
        return;
    }

    receipt.evaluated = true;
    base_intent.Validate(status);
    if (!status.ok())
    {
        receipt.reason = ObfmNearStationCarrotReason::BaseIntentInvalid;
        return;
    }
    receipt.base_validated = true;

    if (base_intent.route_kind != ControlRouteKind::AimPoint)
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::RouteNotAimPoint,
            status);
        return;
    }

    if (!std::isfinite(input.own_position_ned_m[0])
        || !std::isfinite(input.own_position_ned_m[1]))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::OwnHorizontalPositionUnavailable,
            status);
        return;
    }

    if (!input.target_track_velocity_available)
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::TargetTrackUnavailable,
            status);
        return;
    }
    if (!FiniteVector(input.target_track_velocity_ned_mps))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::TargetTrackNonFinite,
            status);
        return;
    }

    double station_north_m = 0.0;
    double station_east_m = 0.0;
    if (!CheckedSubtract(
            base_intent.aim_point_m[0],
            input.own_position_ned_m[0],
            station_north_m)
        || !CheckedSubtract(
            base_intent.aim_point_m[1],
            input.own_position_ned_m[1],
            station_east_m))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::StationGeometryUnavailable,
            status);
        return;
    }

    double station_distance_m = 0.0;
    if (!CheckedHypot2(
            station_north_m,
            station_east_m,
            station_distance_m))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::StationGeometryUnavailable,
            status);
        return;
    }
    receipt.horizontal_station_distance_m = station_distance_m;

    double distance_from_far_m = 0.0;
    if (!CheckedSubtract(
            ObfmNearStationCarrotBlendFarM,
            station_distance_m,
            distance_from_far_m))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::ArithmeticUnavailable,
            status);
        return;
    }
    // The exact authority constants are finite and Far is strictly above Near.
    double normalized = distance_from_far_m
        / (ObfmNearStationCarrotBlendFarM
            - ObfmNearStationCarrotBlendNearM);
    if (normalized < 0.0)
    {
        normalized = 0.0;
    }
    else if (normalized > 1.0)
    {
        normalized = 1.0;
    }
    receipt.normalized_near_station_blend = normalized;

    const double station_weight =
        normalized * normalized * (3.0 - 2.0 * normalized);
    if (!std::isfinite(station_weight))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::ArithmeticUnavailable,
            status);
        return;
    }
    receipt.station_weight = station_weight;

    if (station_weight == 0.0)
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::InactiveAtOrBeyondFarBand,
            status);
        return;
    }

    double horizontal_track_norm_mps = 0.0;
    if (!CheckedHypot2(
            input.target_track_velocity_ned_mps[0],
            input.target_track_velocity_ned_mps[1],
            horizontal_track_norm_mps))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::ArithmeticUnavailable,
            status);
        return;
    }
    receipt.horizontal_track_norm_mps = horizontal_track_norm_mps;
    if (horizontal_track_norm_mps
        <= ObfmNearStationCarrotTrackNormMinimumMps)
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::TargetHorizontalTrackUndefined,
            status);
        return;
    }

    const double unit_track_north =
        input.target_track_velocity_ned_mps[0] / horizontal_track_norm_mps;
    const double unit_track_east =
        input.target_track_velocity_ned_mps[1] / horizontal_track_norm_mps;
    const double lookahead_m =
        station_weight * ObfmNearStationCarrotLookaheadM;
    const double carrot_north_m = lookahead_m * unit_track_north;
    const double carrot_east_m = lookahead_m * unit_track_east;
    double shaped_north_m = 0.0;
    double shaped_east_m = 0.0;
    if (!std::isfinite(unit_track_north)
        || !std::isfinite(unit_track_east)
        || !std::isfinite(lookahead_m)
        || !std::isfinite(carrot_north_m)
        || !std::isfinite(carrot_east_m)
        || !CheckedAdd(
            base_intent.aim_point_m[0],
            carrot_north_m,
            shaped_north_m)
        || !CheckedAdd(
            base_intent.aim_point_m[1],
            carrot_east_m,
            shaped_east_m))
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::ArithmeticUnavailable,
            status);
        return;
    }

    ControlIntent candidate{};
    CopyIntentBytes(base_intent, candidate);
    candidate.aim_point_m[0] = shaped_north_m;
    candidate.aim_point_m[1] = shaped_east_m;

    Status candidate_status{};
    candidate.Validate(candidate_status);
    if (!candidate_status.ok())
    {
        PreserveBase(
            receipt,
            ObfmNearStationCarrotReason::ArithmeticUnavailable,
            status);
        return;
    }

    CopyIntentBytes(candidate, output_intent);
    receipt.applied = true;
    receipt.base_preserved = false;
    receipt.reason = ObfmNearStationCarrotReason::Applied;
    status = Status{};
}

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
