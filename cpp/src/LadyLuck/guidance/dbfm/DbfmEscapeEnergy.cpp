#include "LadyLuck/guidance/dbfm/DbfmEscapeEnergy.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double CapabilityTableMachEdge = 2.0;
constexpr double DoctrineShallowDiveCapRad =
    0.17453292519943295; // radians(10 deg), manual 5-10 deg band edge
constexpr std::uint32_t FloorBisectionSteps = 24U;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double PythonNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1]
            + value[2] * value[2]));
}

bool ActivationValid(
    const LadyLuck::guidance::dbfm::DbfmEscapeEnergyActivation&
        activation) noexcept
{
    return activation.enabled == activation.exact_escape_energy_provenance;
}

bool BehaviorInDomain(
    const LadyLuck::guidance::dbfm::DbfmEscapeEnergyBehavior behavior) noexcept
{
    using LadyLuck::guidance::dbfm::DbfmEscapeEnergyBehavior;
    return behavior == DbfmEscapeEnergyBehavior::Escape
        || behavior == DbfmEscapeEnergyBehavior::AltitudeSeparated;
}

double IsaSpeedOfSoundMps(const double altitude_m) noexcept
{
    const double temperature_k = altitude_m <= 11000.0
        ? 288.15 - 0.0065 * altitude_m
        : 216.65;
    return std::sqrt(1.4 * 287.05287 * temperature_k);
}

bool ValidateSameFrameAutoGcasReceipt(
    const LadyLuck::DogfightGeometryFrame& frame,
    const LadyLuck::safety::AutoGcasEntryReceipt& receipt,
    bool& state_available,
    double& bank_rad,
    LadyLuck::Status& status) noexcept
{
    state_available = false;
    bank_rad = 0.0;
    status = LadyLuck::Status{};
    // This receipt controls only the optional shallow-dive refinement.  The
    // caller already owns a finite horizontal-away command, so missing,
    // stale, or internally incomplete safety evidence must level the dive;
    // it must never erase the base escape command or create a control gap.
    if (!receipt.valid
        || !LadyLuck::IsValidControlFrameIdentity(frame.frame_identity)
        || !LadyLuck::IsValidControlFrameIdentity(receipt.frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            frame.frame_identity,
            receipt.frame_identity))
    {
        return true;
    }

    const LadyLuck::safety::AutoGcasEntryInput& evaluated =
        receipt.evaluated_input;
    if (!LadyLuck::SameControlFrameIdentity(
            receipt.frame_identity,
            evaluated.estimator_frame_identity)
        || !LadyLuck::SameControlFrameIdentity(
            receipt.frame_identity,
            evaluated.envelope_frame_identity)
        || evaluated.ownship.frame_index
            != receipt.frame_identity.frame_index)
    {
        return true;
    }
    if (!std::isfinite(frame.t_sec)
        || !std::isfinite(evaluated.t_sec))
    {
        return true;
    }
    if (!receipt.entry_available && receipt.entry_recoverable)
    {
        return true;
    }
    if (!receipt.entry_available || !receipt.entry_recoverable)
    {
        // Exact Python ordinary non-admission: the speed/unload half remains
        // usable, while the optional dive levels.
        return true;
    }

    bank_rad = evaluated.ownship.rpy_rad[0];
    if (!std::isfinite(bank_rad))
    {
        bank_rad = 0.0;
        return true;
    }
    state_available = true;
    return true;
}

// Encode the Python evaluator's explicit climb-rate scalar into the existing
// C++ AutoGcasEntryInput, whose public surface reconstructs climb rate from
// body velocity and attitude.  Pitch is zero and the better-conditioned of
// the bank sine/cosine terms carries the scalar; neither body-velocity norm nor
// attitude is used by the recovery predictor after ClimbRate() is evaluated.
void EncodeClimbRate(
    const double target_climb_rate_mps,
    LadyLuck::PlaneState& state) noexcept
{
    const double bank_rad = state.rpy_rad[0];
    const double sine_bank = std::sin(bank_rad);
    const double cosine_bank = std::cos(bank_rad);
    state.rpy_rad[1] = 0.0;
    state.velocity_body_mps = LadyLuck::Vector3{{0.0, 0.0, 0.0}};
    if (std::fabs(cosine_bank) >= std::fabs(sine_bank))
    {
        state.velocity_body_mps[2] =
            -target_climb_rate_mps / cosine_bank;
    }
    else
    {
        state.velocity_body_mps[1] =
            -target_climb_rate_mps / sine_bank;
    }
}

bool RecoveryAllowsDivePosture(
    const double dive_rad,
    const double speed_mps,
    const double altitude_m,
    const double pull_capability_n_g,
    const double roll_rad,
    const double measured_climb_rate_mps,
    const LadyLuck::safety::AutoGcasEntryReceipt& current_entry,
    const LadyLuck::safety::AutoGcas& predictor) noexcept
{
    const double values[] = {
        dive_rad,
        speed_mps,
        altitude_m,
        pull_capability_n_g,
        roll_rad,
        measured_climb_rate_mps};
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    if (dive_rad < 0.0
        || dive_rad >= 0.5 * LadyLuck::constants::Pi
        || speed_mps <= 0.0
        || pull_capability_n_g <= 1.0)
    {
        return false;
    }

    const double candidate_climb_rate_mps =
        -speed_mps * std::sin(dive_rad);
    const double target_climb_rate_mps = std::min(
        candidate_climb_rate_mps,
        measured_climb_rate_mps);
    if (target_climb_rate_mps < candidate_climb_rate_mps)
    {
        // Preserve Python's measured-sink realization check before asking the
        // same-frame entry evaluator.  The idealized candidate puts all speed
        // on body x at pitch=-dive; body z is the sole free term used to carry
        // an already-more-negative measured climb rate.
        const double pitch_rad = -dive_rad;
        const double denominator =
            std::cos(roll_rad) * std::cos(pitch_rad);
        if (denominator == 0.0)
        {
            return false;
        }
        const double body_w_mps =
            (candidate_climb_rate_mps - target_climb_rate_mps)
            / denominator;
        const double realized_climb_rate_mps =
            speed_mps * std::sin(pitch_rad)
            - body_w_mps * std::cos(roll_rad) * std::cos(pitch_rad);
        const double representation_band_mps = std::max(
            std::max(
                std::fabs(realized_climb_rate_mps),
                std::fabs(target_climb_rate_mps)),
            1.0)
            * std::numeric_limits<double>::epsilon();
        if (!std::isfinite(realized_climb_rate_mps)
            || realized_climb_rate_mps
                > target_climb_rate_mps + representation_band_mps)
        {
            return false;
        }
    }

    LadyLuck::safety::AutoGcasEntryInput candidate =
        current_entry.evaluated_input;
    candidate.ownship.position_ned_m[2] = -altitude_m;
    candidate.ownship.rpy_rad[0] = roll_rad;
    candidate.ownship.speed_mps = speed_mps;
    candidate.available_nz_g = pull_capability_n_g;
    candidate.available_nz_valid = true;
    EncodeClimbRate(target_climb_rate_mps, candidate.ownship);

    LadyLuck::safety::AutoGcasEntryReceipt candidate_receipt{};
    LadyLuck::Status candidate_status{};
    predictor.EvaluateEntry(candidate, candidate_receipt, candidate_status);
    if (!candidate_status.ok()
        || !candidate_receipt.valid
        || !candidate_receipt.entry_available)
    {
        return false;
    }
    return !candidate_receipt.entry_should_activate;
}

double FloorLimitedDiveRad(
    const double speed_mps,
    const double altitude_m,
    const double pull_capability_n_g,
    const double roll_rad,
    const double measured_climb_rate_mps,
    const LadyLuck::safety::AutoGcasEntryReceipt& current_entry) noexcept
{
    if (!std::isfinite(pull_capability_n_g)
        || pull_capability_n_g <= 1.0
        || !std::isfinite(roll_rad))
    {
        return 0.0;
    }
    const LadyLuck::safety::AutoGcas predictor{};
    if (RecoveryAllowsDivePosture(
            DoctrineShallowDiveCapRad,
            speed_mps,
            altitude_m,
            pull_capability_n_g,
            roll_rad,
            measured_climb_rate_mps,
            current_entry,
            predictor))
    {
        return DoctrineShallowDiveCapRad;
    }
    if (!RecoveryAllowsDivePosture(
            0.0,
            speed_mps,
            altitude_m,
            pull_capability_n_g,
            roll_rad,
            measured_climb_rate_mps,
            current_entry,
            predictor))
    {
        return 0.0;
    }
    double low = 0.0;
    double high = DoctrineShallowDiveCapRad;
    for (std::uint32_t index = 0U;
         index < FloorBisectionSteps;
         ++index)
    {
        const double middle = 0.5 * (low + high);
        if (RecoveryAllowsDivePosture(
                middle,
                speed_mps,
                altitude_m,
                pull_capability_n_g,
                roll_rad,
                measured_climb_rate_mps,
                current_entry,
                predictor))
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    return low;
}
}

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{
em::EmValue DbfmMaximumPowerSpeedReferenceMps(
    const double altitude_m) noexcept
{
    em::EmValue output{};
    if (!std::isfinite(altitude_m))
    {
        return output;
    }
    const double sound_speed_mps = IsaSpeedOfSoundMps(altitude_m);
    const double reference_mps = CapabilityTableMachEdge * sound_speed_mps;
    if (!std::isfinite(reference_mps) || reference_mps <= 0.0)
    {
        return output;
    }
    output.has_value = true;
    output.value = reference_mps;
    return output;
}

void ApplyDbfmEscapeEnergy(
    const DogfightGeometryFrame& frame,
    const DbfmEscapeEnergyBehavior behavior,
    const DbfmEscapeEnergyActivation& activation,
    const DbfmEscapeEnergyBaseReference& base_reference,
    const em::EnergyManeuverCapability& capability,
    const safety::AutoGcasEntryReceipt& current_auto_gcas_entry,
    DbfmEscapeEnergyReceipt& output,
    Status& status) noexcept
{
    output = DbfmEscapeEnergyReceipt{};
    output.frame_identity = frame.frame_identity;
    status = Status{};

    if (!ActivationValid(activation))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!activation.enabled)
    {
        output.reason = DbfmEscapeEnergyReason::ProductionDisabled;
        return;
    }
    if (!base_reference.valid)
    {
        output.reason = DbfmEscapeEnergyReason::BaseReferenceUnavailable;
        return;
    }
    if (!BehaviorInDomain(behavior))
    {
        // Python returns the existing command unchanged for every behavior
        // outside ESCAPE/ALTITUDE_SEPARATED.  This is ordinary non-ownership,
        // including a future behavior ID that this frozen overlay does not
        // recognize; it is not contradictory ready evidence.
        output.reason = DbfmEscapeEnergyReason::NonOwner;
        return;
    }
    if (!IsValidControlFrameIdentity(base_reference.frame_identity)
        || !SameControlFrameIdentity(
            frame.frame_identity,
            base_reference.frame_identity))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }

    bool gcas_state_available = false;
    double roll_rad = 0.0;
    if (!ValidateSameFrameAutoGcasReceipt(
            frame,
            current_auto_gcas_entry,
            gcas_state_available,
            roll_rad,
            status))
    {
        return;
    }

    const bool capability_available = capability.n_channel_trusted
        && capability.n_inst_g.has_value;
    const double pull_capability_n_g = capability_available
        ? capability.n_inst_g.value
        : 0.0;

    const Vector3& own_position = frame.own.position_ned_m;
    const Vector3& own_velocity = frame.own.velocity_ned_mps;
    if (!FiniteVector(own_position) || !FiniteVector(own_velocity))
    {
        output.reason = DbfmEscapeEnergyReason::StateUnavailable;
        return;
    }
    const double own_speed_mps = PythonNorm3(own_velocity);
    const double own_altitude_m = -own_position[2];
    const double measured_climb_rate_mps = -own_velocity[2];
    if (!std::isfinite(own_speed_mps)
        || !std::isfinite(own_altitude_m)
        || own_speed_mps <= 0.0)
    {
        output.reason = DbfmEscapeEnergyReason::StateUnavailable;
        return;
    }

    const em::EmValue maximum_power_speed =
        DbfmMaximumPowerSpeedReferenceMps(own_altitude_m);
    if (!maximum_power_speed.has_value)
    {
        output.reason = DbfmEscapeEnergyReason::StateUnavailable;
        return;
    }

    double dive_rad = 0.0;
    if (capability_available && gcas_state_available)
    {
        dive_rad = FloorLimitedDiveRad(
            own_speed_mps,
            own_altitude_m,
            pull_capability_n_g,
            roll_rad,
            measured_climb_rate_mps,
            current_auto_gcas_entry);
    }

    if (!FiniteVector(base_reference.aim_point_ned_m))
    {
        output.reason = DbfmEscapeEnergyReason::HorizontalAimUndefined;
        return;
    }
    const double horizontal_north_m =
        base_reference.aim_point_ned_m[0] - own_position[0];
    const double horizontal_east_m =
        base_reference.aim_point_ned_m[1] - own_position[1];
    const double horizontal_range_m = std::sqrt(
        horizontal_north_m * horizontal_north_m
        + horizontal_east_m * horizontal_east_m);
    if (!std::isfinite(horizontal_range_m) || horizontal_range_m <= 0.0)
    {
        output.reason = DbfmEscapeEnergyReason::HorizontalAimUndefined;
        return;
    }

    output.aim_point_ned_m = base_reference.aim_point_ned_m;
    output.aim_point_ned_m[2] = own_position[2]
        + horizontal_range_m * std::tan(dive_rad);
    output.desired_speed_mps = maximum_power_speed.value;
    output.desired_speed_rate_mps2 = 0.0;
    output.dive_depression_rad = dive_rad;
    output.dive_admitted = dive_rad > 0.0;
    output.candidate_available = true;
    if (!gcas_state_available)
    {
        output.reason =
            DbfmEscapeEnergyReason::AppliedLevelGcasUnavailable;
    }
    else if (!capability_available)
    {
        output.reason =
            DbfmEscapeEnergyReason::AppliedLevelCapabilityUnavailable;
    }
    else if (dive_rad == 0.0)
    {
        output.reason =
            DbfmEscapeEnergyReason::AppliedLevelRecoveryLimited;
    }
    else
    {
        output.reason = DbfmEscapeEnergyReason::AppliedDive;
    }
}

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
