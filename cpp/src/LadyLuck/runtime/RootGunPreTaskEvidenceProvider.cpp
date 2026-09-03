#include "LadyLuck/runtime/RootGunPreTaskEvidenceProvider.hpp"

#include <cmath>

namespace
{

constexpr double CompetitionCharacterizationMassKg = 11159.27948674;
const char CompetitionCharacterizationMassSource[] =
    "plant MASS0_KG initial mass constant (characterization; fuel burn over "
    "an episode is below the EM grid resolution)";
const char CapabilityNzSource[] =
    "not_required_for_capability_ceiling";
const char CompetitionCleanConfigurationSource[] =
    "clean configuration (competition runtime; gear and speedbrake commanded "
    "retracted)";

double VectorNorm(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + (value[1] * value[1]
            + value[2] * value[2]));
}

} // namespace

namespace LadyLuck
{
namespace runtime
{

void RootGunPreTaskEvidenceProvider::Observe(
    const DogfightGeometryFrame& frame,
    RootGunPreTaskEvidence& output,
    Status& status) const noexcept
{
    output = RootGunPreTaskEvidence{};
    status = Status{};
    if (!IsValidControlFrameIdentity(frame.frame_identity))
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    const double observed_damage_rate = frame.enemy_offense.damage_rate;
    if (!std::isfinite(observed_damage_rate))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    // Damage is a one-sided physical rate. A finite negative transport or
    // estimator artifact cannot prove an active gun threat and must not erase
    // the current-frame tactical command. Preserve the physical meaning by
    // projecting it onto the admissible half-line.
    const double damage_rate = observed_damage_rate > 0.0
        ? observed_damage_rate
        : 0.0;

    const double speed_mps = VectorNorm(frame.own.velocity_ned_mps);
    const double altitude_m = -frame.own.position_ned_m[2];
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }

    guidance::em::StrictEmInput strict_input{};
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
    strict_input.configuration_source =
        CompetitionCleanConfigurationSource;
    const guidance::em::EnergyManeuverCapability capability =
        strict_em_.Observe(strict_input);

    output.frame_identity = frame.frame_identity;
    output.valid = true;
    output.official_gun_threat = damage_rate > 0.0;
    output.strict_em_input = strict_input;
    output.capability = capability;
    output.capability_admitted = capability.n_channel_trusted
        && capability.n_inst_g.has_value;
    output.instantaneous_corner_interval =
        corner_provider_.InstantaneousInterval(altitude_m);
}

} // namespace runtime
} // namespace LadyLuck
