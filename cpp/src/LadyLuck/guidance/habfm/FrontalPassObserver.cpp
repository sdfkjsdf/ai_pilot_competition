#include "LadyLuck/guidance/habfm/FrontalPassObserver.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/geometry/WezGeometry.hpp"
#include "LadyLuck/geometry/WezRule.hpp"
#include "LadyLuck/math/Attitude321.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace
{
struct MeasuredTrackingNode
{
    double commanded_abeam_m = 0.0;
    double worst_command_shortfall_m = 0.0;
};

// add/main d90e929b4096ce88691cd5198394abc5d89b2d4a,
// guidance/behavior_tree/frontal_pass_observer.py:159-168. Preserve order:
// the first qualifying node is the Python step-up inversion result.
constexpr std::array<MeasuredTrackingNode, 8U> FrontalTrackingMeasuredNodes{{
    {15.960911370362162, 8.376793633760492},
    {37.253476893796474, 20.052292920835473},
    {39.917435076246655, 21.59170313123645},
    {63.89556450188385, 35.61873784932733},
    {93.16906853037888, 53.96260320533958},
    {159.79958715892585, 96.65219565829649},
    {165.0, 100.06641877172132},
    {170.0, 103.29575246601644}}};

// Exact binary value of 2**-23 used by habfm_selection.py.
constexpr double PlaneInfoFloat32RelativeQuantum =
    1.1920928955078125e-7;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

void SetFault(
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    status.code = code;
}
}

namespace LadyLuck
{
void OfficialConeAbeamM(
    const double t_sec,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept
{
    output = FrontalPassOptionalScalar{};
    status = Status{};
    if (!std::isfinite(t_sec))
    {
        return;
    }

    bool widest_valid = false;
    double widest = 0.0;
    for (std::size_t index = 0U; index < OfficialWezPhaseCount; ++index)
    {
        const Result<WezPhase> phase = OfficialWezPhaseAt(index);
        if (!phase.ok())
        {
            SetFault(status, StatusCode::InvalidConfiguration);
            return;
        }
        if (t_sec < phase.value.start_sec)
        {
            continue;
        }
        const double corridor = phase.value.max_range_m
            * std::tan(phase.value.angle_rad);
        if (!widest_valid || corridor > widest)
        {
            widest = corridor;
            widest_valid = true;
        }
    }
    if (!widest_valid || !std::isfinite(widest) || widest <= 0.0)
    {
        return;
    }
    output.has_value = true;
    output.value = widest;
}

void FrameObservationErrorM(
    const Vector3& own_position_m,
    const Vector3& adversary_position_m,
    double& output_m,
    Status& status) noexcept
{
    output_m = 0.0;
    status = Status{};
    if (!FiniteVector(own_position_m) || !FiniteVector(adversary_position_m))
    {
        SetFault(status, StatusCode::NonFiniteInput);
        return;
    }

    const double error = (
        VectorNorm(own_position_m) + VectorNorm(adversary_position_m))
        * PlaneInfoFloat32RelativeQuantum;
    if (!std::isfinite(error) || error < 0.0)
    {
        SetFault(status, StatusCode::NonFiniteInput);
        return;
    }
    output_m = error;
}

void EffectiveCorridorHalfWidthM(
    const double cone_half_width_m,
    const double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept
{
    output = FrontalPassOptionalScalar{};
    status = Status{};

    // Preserve Python's evaluation order: float(cone) + float(observation).
    const double required_m = cone_half_width_m + observation_error_m;
    if (!std::isfinite(required_m))
    {
        return;
    }
    for (const MeasuredTrackingNode& node : FrontalTrackingMeasuredNodes)
    {
        if (node.commanded_abeam_m - node.worst_command_shortfall_m
            >= required_m)
        {
            output.has_value = true;
            output.value = node.commanded_abeam_m;
            return;
        }
    }
}

void BandClearingAbeamPairM(
    const double t_sec,
    const double range_m,
    const double lateral_m,
    const double observation_error_m,
    FrontalPassBandClearingPair& output,
    Status& status) noexcept
{
    output = FrontalPassBandClearingPair{};
    status = Status{};

    // _band_clearing_pair catches scalar conversion/finite faults and returns
    // None. Therefore these are normal invalid observations, not API faults.
    if (!std::isfinite(t_sec)
        || !std::isfinite(range_m)
        || !std::isfinite(lateral_m)
        || !std::isfinite(observation_error_m)
        || observation_error_m < 0.0)
    {
        return;
    }
    const double lateral = std::fabs(lateral_m);

    bool widest_safe_valid = false;
    bool widest_compressed_valid = false;
    double widest_safe = 0.0;
    double widest_compressed = 0.0;
    for (std::size_t index = 0U; index < OfficialWezPhaseCount; ++index)
    {
        const Result<WezPhase> phase = OfficialWezPhaseAt(index);
        if (!phase.ok())
        {
            SetFault(status, StatusCode::InvalidConfiguration);
            return;
        }
        if (t_sec < phase.value.start_sec)
        {
            continue;
        }

        const double max_range_m = phase.value.max_range_m;
        const double cone_half_width_m = max_range_m
            * std::tan(phase.value.angle_rad);
        FrontalPassOptionalScalar effective{};
        Status effective_status{};
        EffectiveCorridorHalfWidthM(
            cone_half_width_m,
            observation_error_m,
            effective,
            effective_status);
        if (!effective_status.ok())
        {
            status = effective_status;
            return;
        }
        if (!effective.has_value)
        {
            return;
        }

        double safe = 0.0;
        double compressed = 0.0;
        if (range_m > max_range_m)
        {
            const double run_in = range_m / (range_m - max_range_m);
            const double deficit = (std::max)(
                0.0,
                effective.value - lateral);
            safe = deficit * run_in + lateral;
            compressed = (std::max)(
                0.0,
                effective.value - lateral * max_range_m / range_m)
                * run_in;
        }
        else
        {
            safe = (std::max)(lateral, effective.value);
            compressed = effective.value;
        }

        // Preserve Python min(max(compressed, effective), safe) exactly.
        compressed = (std::min)(
            (std::max)(compressed, effective.value),
            safe);
        if (!widest_safe_valid || safe > widest_safe)
        {
            widest_safe = safe;
            widest_safe_valid = true;
        }
        if (!widest_compressed_valid || compressed > widest_compressed)
        {
            widest_compressed = compressed;
            widest_compressed_valid = true;
        }
    }

    if (!widest_safe_valid || !widest_compressed_valid)
    {
        return;
    }
    if (!std::isfinite(widest_safe)
        || widest_safe <= 0.0
        || !std::isfinite(widest_compressed)
        || widest_compressed <= 0.0)
    {
        return;
    }
    output.has_value = true;
    output.safe_abeam_m = widest_safe;
    output.compressed_abeam_m = widest_compressed;
}

void BandClearingAbeamM(
    const double t_sec,
    const double range_m,
    const double lateral_m,
    const double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept
{
    output = FrontalPassOptionalScalar{};
    FrontalPassBandClearingPair pair{};
    BandClearingAbeamPairM(
        t_sec,
        range_m,
        lateral_m,
        observation_error_m,
        pair,
        status);
    if (!status.ok() || !pair.has_value)
    {
        return;
    }
    output.has_value = true;
    output.value = pair.safe_abeam_m;
}

void CompressedBandClearingAbeamM(
    const double t_sec,
    const double range_m,
    const double lateral_m,
    const double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept
{
    output = FrontalPassOptionalScalar{};
    FrontalPassBandClearingPair pair{};
    BandClearingAbeamPairM(
        t_sec,
        range_m,
        lateral_m,
        observation_error_m,
        pair,
        status);
    if (!status.ok() || !pair.has_value)
    {
        return;
    }
    output.has_value = true;
    output.value = pair.compressed_abeam_m;
}

void EvaluateFrontalPass(
    const DogfightGeometryFrame& frame,
    const std::int32_t fallback_side_sign,
    FrontalPassEvidence& output,
    Status& status) noexcept
{
    output = FrontalPassEvidence{};
    status = Status{};
    if (fallback_side_sign != -1 && fallback_side_sign != 1)
    {
        SetFault(status, StatusCode::InvalidArgument);
        return;
    }

    // Preserve Python field-read/validation order.
    if (!std::isfinite(frame.t_sec)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.position_ned_m)
        || !std::isfinite(frame.enemy_offense.range_m)
        || !std::isfinite(frame.enemy_offense.ata_rad))
    {
        SetFault(status, StatusCode::NonFiniteInput);
        return;
    }

    const Result<WezPhaseMatch> threatened = MatchWezPhase(
        frame.enemy_offense.range_m,
        frame.enemy_offense.ata_rad,
        frame.t_sec);
    if (!threatened.ok())
    {
        status = threatened.status;
        return;
    }
    output.threatened_now = threatened.value.matched;

    if (!std::isfinite(frame.closing_speed_mps))
    {
        output = FrontalPassEvidence{};
        SetFault(status, StatusCode::NonFiniteInput);
        return;
    }
    if (frame.closing_speed_mps <= 0.0)
    {
        output.reason =
            FrontalPassReason::OpeningGeometryNoPendingFrontalPass;
        return;
    }

    Vector3 line_of_sight{{
        frame.opponent.position_ned_m[0] - frame.own.position_ned_m[0],
        frame.opponent.position_ned_m[1] - frame.own.position_ned_m[1],
        frame.opponent.position_ned_m[2] - frame.own.position_ned_m[2]}};
    line_of_sight[2] = 0.0;
    const double separation_m = VectorNorm(line_of_sight);
    if (!std::isfinite(separation_m) || separation_m < constants::Tiny)
    {
        output.reason = FrontalPassReason::DegenerateHorizontalLineOfSight;
        return;
    }
    line_of_sight[0] /= separation_m;
    line_of_sight[1] /= separation_m;
    line_of_sight[2] /= separation_m;

    // Intentionally preserve the Python 3-D offense geometry here. Do not
    // substitute horizontal separation/ATA: range*sin(ATA) is authoritative.
    const double lateral_m = frame.enemy_offense.range_m
        * std::sin(frame.enemy_offense.ata_rad);
    double observation_error_m = 0.0;
    Status observation_status{};
    FrameObservationErrorM(
        frame.own.position_ned_m,
        frame.opponent.position_ned_m,
        observation_error_m,
        observation_status);
    if (!observation_status.ok())
    {
        output = FrontalPassEvidence{};
        status = observation_status;
        return;
    }

    FrontalPassBandClearingPair pair{};
    Status pair_status{};
    BandClearingAbeamPairM(
        frame.t_sec,
        frame.enemy_offense.range_m,
        lateral_m,
        observation_error_m,
        pair,
        pair_status);
    if (!pair_status.ok())
    {
        output = FrontalPassEvidence{};
        status = pair_status;
        return;
    }
    if (!pair.has_value)
    {
        output.reason = FrontalPassReason::NoUnlockedOfficialPhase;
        return;
    }

    const Vector3 lateral_axis{{
        -line_of_sight[1],
        line_of_sight[0],
        0.0}};
    const double lateral_component =
        frame.own.velocity_ned_mps[0] * lateral_axis[0]
        + frame.own.velocity_ned_mps[1] * lateral_axis[1]
        + frame.own.velocity_ned_mps[2] * lateral_axis[2];

    std::int32_t side_sign = fallback_side_sign;
    FrontalPassReason reason =
        FrontalPassReason::ExactHeadOnUsesCallerEntrySide;
    if (lateral_component > 0.0)
    {
        side_sign = 1;
        reason = FrontalPassReason::OffsetTowardTheAlreadyOpenSide;
    }
    else if (lateral_component < 0.0)
    {
        side_sign = -1;
        reason = FrontalPassReason::OffsetTowardTheAlreadyOpenSide;
    }

    output.admitted = true;
    output.reason = reason;
    output.safe_abeam_m = pair.safe_abeam_m;
    output.compressed_abeam_m = pair.compressed_abeam_m;
    output.side_sign = side_sign;
}

void FrontalPassTracker::Reset() noexcept
{
    latched_side_valid_ = false;
    latched_side_sign_ = 0;
}

void FrontalPassTracker::GetLatchedSideSign(
    std::int32_t& side_sign,
    bool& has_value) const noexcept
{
    side_sign = latched_side_valid_ ? latched_side_sign_ : 0;
    has_value = latched_side_valid_;
}

void FrontalPassTracker::Update(
    const DogfightGeometryFrame& frame,
    const std::int32_t fallback_side_sign,
    FrontalPassEvidence& output,
    Status& status) noexcept
{
    EvaluateFrontalPass(frame, fallback_side_sign, output, status);
    if (!status.ok())
    {
        Reset();
        return;
    }
    if (!output.admitted)
    {
        Reset();
        return;
    }
    if (!latched_side_valid_)
    {
        latched_side_sign_ = output.side_sign;
        latched_side_valid_ = true;
        return;
    }
    if (output.side_sign == latched_side_sign_)
    {
        return;
    }
    output.side_sign = latched_side_sign_;
    output.reason = FrontalPassReason::SideLatchedFromFirstAdmission;
}
}
