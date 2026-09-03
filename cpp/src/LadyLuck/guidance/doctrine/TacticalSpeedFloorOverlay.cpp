#include "LadyLuck/guidance/doctrine/TacticalSpeedFloorOverlay.hpp"

#include "LadyLuck/common/Constants.hpp"
#include "LadyLuck/plant/dynamics/MassModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

constexpr double CompetitionCharacterizationMassKg =
    LadyLuck::plant::dynamics::InitialMassKg;
const char CompetitionCharacterizationMassSource[] =
    "plant MASS0_KG initial mass constant (characterization; fuel burn over "
    "an episode is below the EM grid resolution)";
const char CapabilityNzSource[] =
    "not_required_for_capability_ceiling";
const char CompetitionCleanConfigurationSource[] =
    "clean configuration (competition runtime; gear and speedbrake commanded "
    "retracted)";

constexpr double LegacyAutoGcasEffectiveRollRateRadps = 3.14;
constexpr double LegacyAutoGcasPipelineDtS = 1.0 / 60.0;
constexpr double DoctrineShallowDiveCapRad =
    10.0 * LadyLuck::constants::Pi / 180.0;
constexpr std::uint32_t FloorBisectionSteps = 24U;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const LadyLuck::Vector3& value) noexcept
{
    // NumPy 1.26 length-three reduction association used by the d90 port.
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1] + value[2] * value[2]));
}

LadyLuck::safety::AutoGcasConfig LegacyRecoveryPredictorConfig() noexcept
{
    LadyLuck::safety::AutoGcasConfig config{};
    // dbfm_escape_energy.py constructs AutoGCAS() directly. Its historical
    // default differs from the production filter's achieved-bank entry model.
    config.entry_effective_roll_rate_radps =
        LegacyAutoGcasEffectiveRollRateRadps;
    config.maximum_roll_rate_radps = 3.14;
    return config;
}

void Fail(
    LadyLuck::guidance::doctrine::TacticalSpeedFloorOverlayReceipt& output,
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    output = LadyLuck::guidance::doctrine::
        TacticalSpeedFloorOverlayReceipt{};
    status.code = code;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

void TacticalSpeedFloorEvidenceProvider::Reset() noexcept
{
}

void TacticalSpeedFloorEvidenceProvider::Observe(
    const DogfightGeometryFrame& frame,
    TacticalSpeedFloorEvidence& output,
    Status& status) noexcept
{
    output = TacticalSpeedFloorEvidence{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity)
        || !FiniteVector(frame.own.position_ned_m)
        || !FiniteVector(frame.own.velocity_ned_mps))
    {
        status.code = !FiniteVector(frame.own.position_ned_m)
                || !FiniteVector(frame.own.velocity_ned_mps)
            ? StatusCode::NonFiniteInput
            : StatusCode::InvalidArgument;
        return;
    }

    const double speed_mps = NumpyNorm3(frame.own.velocity_ned_mps);
    const double altitude_m = -frame.own.position_ned_m[2];
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    em::StrictEmInput strict_input{};
    strict_input.speed_mps = speed_mps;
    strict_input.altitude_m = altitude_m;
    strict_input.mass_kg = CompetitionCharacterizationMassKg;
    strict_input.mass_valid = true;
    strict_input.mass_source = CompetitionCharacterizationMassSource;
    strict_input.nz_g = 0.0;
    strict_input.nz_valid = false;
    strict_input.nz_source = CapabilityNzSource;
    strict_input.gear_pos_norm = 0.0;
    strict_input.speedbrake_pos_norm = 0.0;
    strict_input.speedbrake_valid = true;
    strict_input.configuration_source = CompetitionCleanConfigurationSource;

    output.frame_identity = frame.frame_identity;
    output.capability = strict_envelope_.Observe(strict_input);
    output.capability_admitted = output.capability.n_channel_trusted
        && output.capability.n_inst_g.has_value;
    output.valid = true;
}

TacticalSpeedFloorOverlay::TacticalSpeedFloorOverlay() noexcept
    : recovery_predictor_(LegacyRecoveryPredictorConfig())
{
}

bool TacticalSpeedFloorOverlay::RecoveryAllowsDivePosture(
    const ControlFrameIdentity& frame_identity,
    const double dive_rad,
    const double speed_mps,
    const double altitude_m,
    const double pull_capability_n_g) const noexcept
{
    const double values[] = {
        dive_rad,
        speed_mps,
        altitude_m,
        pull_capability_n_g};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    if (dive_rad < 0.0
        || dive_rad >= 0.5 * constants::Pi
        || speed_mps <= 0.0
        || pull_capability_n_g <= 1.0)
    {
        return false;
    }

    safety::AutoGcasEntryInput entry{};
    entry.estimator_frame_identity = frame_identity;
    entry.envelope_frame_identity = frame_identity;
    entry.t_sec = frame_identity.source_time_s;
    entry.dt_s = LegacyAutoGcasPipelineDtS;
    entry.ownship.frame_index = frame_identity.frame_index;
    entry.ownship.position_ned_m = Vector3{{0.0, 0.0, -altitude_m}};
    entry.ownship.rpy_rad = Vector3{{0.0, -dive_rad, 0.0}};
    entry.ownship.velocity_body_mps = Vector3{{speed_mps, 0.0, 0.0}};
    entry.ownship.speed_mps = speed_mps;
    entry.roll_rate_endpoint_radps = 0.0;
    entry.roll_rate_endpoint_valid = true;
    entry.measured_nz_g = 0.0;
    entry.measured_nz_valid = false;
    entry.available_nz_g = pull_capability_n_g;
    entry.available_nz_valid = true;

    safety::AutoGcasEntryReceipt receipt{};
    Status status{};
    recovery_predictor_.EvaluateEntry(entry, receipt, status);
    return status.code == StatusCode::Ok
        && receipt.valid
        && !receipt.entry_should_activate;
}

double TacticalSpeedFloorOverlay::FloorLimitedDiveRad(
    const ControlFrameIdentity& frame_identity,
    const double speed_mps,
    const double altitude_m,
    const bool capability_admitted,
    const double pull_capability_n_g) const noexcept
{
    if (!capability_admitted
        || !std::isfinite(pull_capability_n_g)
        || pull_capability_n_g <= 1.0)
    {
        return 0.0;
    }

    if (RecoveryAllowsDivePosture(
            frame_identity,
            DoctrineShallowDiveCapRad,
            speed_mps,
            altitude_m,
            pull_capability_n_g))
    {
        return DoctrineShallowDiveCapRad;
    }
    if (!RecoveryAllowsDivePosture(
            frame_identity,
            0.0,
            speed_mps,
            altitude_m,
            pull_capability_n_g))
    {
        return 0.0;
    }

    double low = 0.0;
    double high = DoctrineShallowDiveCapRad;
    for (std::uint32_t index = 0U; index < FloorBisectionSteps; ++index)
    {
        const double mid = 0.5 * (low + high);
        if (RecoveryAllowsDivePosture(
                frame_identity,
                mid,
                speed_mps,
                altitude_m,
                pull_capability_n_g))
        {
            low = mid;
        }
        else
        {
            high = mid;
        }
    }
    return low;
}

void TacticalSpeedFloorOverlay::Evaluate(
    const TacticalSpeedFloorOverlayInput& input,
    TacticalSpeedFloorOverlayReceipt& output,
    Status& status) const noexcept
{
    output = TacticalSpeedFloorOverlayReceipt{};
    status = Status{};
    if (!input.evidence.valid
        || input.modifier_writer_id == ControlIntentWriterNone
        || !IsValidControlFrameIdentity(input.frame.frame_identity)
        || !SameControlFrameIdentity(
            input.upstream_intent.frame_identity,
            input.frame.frame_identity)
        || !SameControlFrameIdentity(
            input.evidence.frame_identity,
            input.frame.frame_identity))
    {
        Fail(output, status, StatusCode::InvalidArgument);
        return;
    }

    Status intent_status{};
    input.upstream_intent.Validate(intent_status);
    if (intent_status.code != StatusCode::Ok)
    {
        Fail(output, status, intent_status.code);
        return;
    }

    output.frame_identity = input.frame.frame_identity;
    output.valid = true;
    output.upstream_writer_id = input.upstream_intent.writer_id;
    output.candidate = input.upstream_intent;

    bool entry = false;
    input.upstream_intent.ClassifyG17EntryOrSpacing(entry, intent_status);
    if (intent_status.code != StatusCode::Ok)
    {
        Fail(output, status, intent_status.code);
        return;
    }
    // The former generic-hold handoff domain has no production owner.
    const bool neutral = false;
    bool approach_lag = false;
    if (neutral)
    {
        output.domain = TacticalSpeedFloorDomain::NeutralHandoff;
    }
    else if (entry)
    {
        output.domain = TacticalSpeedFloorDomain::EntryFamily;
    }
    else if (input.upstream_intent.behavior_id == DoctrineBehaviorId::Lag)
    {
        const double range_m = input.frame.enemy_offense.range_m;
        const double own_reach_m =
            input.frame.own_offense.phase.max_range_m;
        const double enemy_reach_m =
            input.frame.enemy_offense.phase.max_range_m;
        if (!std::isfinite(range_m)
            || !std::isfinite(own_reach_m)
            || !std::isfinite(enemy_reach_m))
        {
            Fail(output, status, StatusCode::NonFiniteInput);
            return;
        }
        if (range_m < 0.0 || own_reach_m <= 0.0 || enemy_reach_m <= 0.0)
        {
            Fail(output, status, StatusCode::InvalidArgument);
            return;
        }
        approach_lag = range_m > std::max(own_reach_m, enemy_reach_m);
        output.domain = approach_lag
            ? TacticalSpeedFloorDomain::ApproachLag
            : TacticalSpeedFloorDomain::InFightLag;
        if (!approach_lag)
        {
            output.application =
                TacticalSpeedFloorApplication::UnchangedInFightLag;
            return;
        }
    }
    else
    {
        output.application =
            TacticalSpeedFloorApplication::UnchangedOutOfDomain;
        return;
    }

    output.applicable = true;
    if (!FiniteVector(input.frame.own.position_ned_m)
        || !FiniteVector(input.frame.own.velocity_ned_mps))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }
    output.own_altitude_m = -input.frame.own.position_ned_m[2];
    output.own_speed_mps = NumpyNorm3(input.frame.own.velocity_ned_mps);
    if (!std::isfinite(output.own_altitude_m)
        || !std::isfinite(output.own_speed_mps))
    {
        Fail(output, status, StatusCode::NonFiniteInput);
        return;
    }

    TacticalSpeedFloorSample floor_sample{};
    SampleTacticalSpeedFloor(output.own_altitude_m, floor_sample);
    if (!floor_sample.admitted())
    {
        output.application =
            TacticalSpeedFloorApplication::UnchangedFloorUnadmitted;
        return;
    }
    const double floor_mps = floor_sample.floor_mps.value;
    if (!std::isfinite(floor_mps) || floor_mps <= 0.0)
    {
        Fail(output, status, StatusCode::InvalidConfiguration);
        return;
    }
    output.floor_mps = floor_mps;
    output.float32_band_mps = std::max(
        std::max(std::fabs(output.own_speed_mps), std::fabs(floor_mps)),
        1.0) * static_cast<double>(std::numeric_limits<float>::epsilon());
    if (output.own_speed_mps
        >= floor_mps - output.float32_band_mps)
    {
        output.application =
            TacticalSpeedFloorApplication::UnchangedWithinFloat32Band;
        return;
    }

    if (entry || approach_lag)
    {
        if (input.upstream_intent.desired_speed_mps >= floor_mps)
        {
            output.application = TacticalSpeedFloorApplication::
                UnchangedDesiredSpeedAlreadyAtFloor;
            return;
        }
        output.candidate.desired_speed_mps = floor_mps;
        output.candidate.desired_speed_rate_mps2 = 0.0;
        output.application =
            TacticalSpeedFloorApplication::AppliedSpeedOnly;
    }
    else
    {
        const bool capability_admitted =
            input.evidence.capability_admitted
            && input.evidence.capability.n_inst_g.has_value;
        const double capability_n_g =
            input.evidence.capability.n_inst_g.value;
        output.dive_depression_rad = FloorLimitedDiveRad(
            input.frame.frame_identity,
            output.own_speed_mps,
            output.own_altitude_m,
            capability_admitted,
            capability_n_g);
        const double north_m = input.upstream_intent.aim_point_m[0]
            - input.frame.own.position_ned_m[0];
        const double east_m = input.upstream_intent.aim_point_m[1]
            - input.frame.own.position_ned_m[1];
        const double horizontal_m = std::sqrt(
            north_m * north_m + east_m * east_m);
        if (!std::isfinite(horizontal_m))
        {
            Fail(output, status, StatusCode::NonFiniteInput);
            return;
        }
        output.candidate.aim_point_m[2] =
            input.frame.own.position_ned_m[2]
            + horizontal_m * std::tan(output.dive_depression_rad);
        output.candidate.desired_speed_mps = floor_mps;
        output.candidate.desired_speed_rate_mps2 = 0.0;
        output.candidate.behavior_id =
            DoctrineBehaviorId::TacticalSpeedFloorNeutralHandoff;
        output.application = output.dive_depression_rad > 0.0
            ? TacticalSpeedFloorApplication::AppliedNeutralDive
            : TacticalSpeedFloorApplication::AppliedNeutralLevel;
    }

    output.candidate.writer_id = input.modifier_writer_id;
    Status candidate_status{};
    output.candidate.Validate(candidate_status);
    if (candidate_status.code != StatusCode::Ok)
    {
        Fail(output, status, candidate_status.code);
        return;
    }
    output.modified = true;
    output.published_writer_id = input.modifier_writer_id;
}

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
