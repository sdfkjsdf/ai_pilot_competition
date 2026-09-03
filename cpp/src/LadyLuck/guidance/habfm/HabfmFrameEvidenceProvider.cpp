#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"

#include "LadyLuck/guidance/doctrine/TacticalSpeedFloorObserver.hpp"
#include "LadyLuck/plant/dynamics/MassModel.hpp"

#include <cmath>

namespace
{

constexpr double kStandardGravityMps2 = 9.80665;

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

double NumpyNorm3(const LadyLuck::Vector3& value) noexcept
{
    return std::sqrt(
        value[0] * value[0]
        + value[1] * value[1]
        + value[2] * value[2]);
}

} // namespace

namespace LadyLuck
{

const char* TacticalSpeedFloorStatusText(
    const TacticalSpeedFloorStatus status) noexcept
{
    switch (status)
    {
    case TacticalSpeedFloorStatus::DerivedByHalfLoopBisection:
        return "derived_by_half_loop_bisection";
    case TacticalSpeedFloorStatus::EmPublicationNotAdmitted:
        return "em_publication_not_admitted";
    case TacticalSpeedFloorStatus::AltitudeNotFinite:
        return "altitude_not_finite";
    case TacticalSpeedFloorStatus::AltitudeOutsidePublishedAxis:
        return "altitude_outside_published_axis";
    case TacticalSpeedFloorStatus::DerivedTrajectoryOutsidePublishedAxis:
        return "derived_trajectory_outside_published_axis";
    default:
        return "unknown_tactical_speed_floor_status";
    }
}

const char* TacticalSpeedFloorProvenance() noexcept
{
    return "G17 tactical speed floor: minimum entry speed whose half loop at "
        "the published sustained load (Ps = 0 by the anchor definition -> "
        "exact gravity-exchange energy identity) completes with the apex "
        "above the published 1-g minimum-speed boundary; dt 0.1 s "
        "reachable-tube convention, 24-step bisection B2 convention; "
        "unsourced '(corner+50kt)' rejected; no fitted constant";
}

const char* SustainedTurnPointStatusText(
    const SustainedTurnPointStatus status) noexcept
{
    switch (status)
    {
    case SustainedTurnPointStatus::Admitted:
        return "admitted";
    case SustainedTurnPointStatus::AltitudeOutsidePublishedAxis:
        return "altitude_outside_published_axis";
    case SustainedTurnPointStatus::PublishedSustainedPointNotTrusted:
        return "published_sustained_point_not_trusted";
    case SustainedTurnPointStatus::PublishedSustainedLoadNotAboveOneG:
        return "published_sustained_load_not_above_one_g";
    case SustainedTurnPointStatus::PublishedLookupInvalid:
        return "published_lookup_invalid";
    default:
        return "unknown_sustained_turn_point_status";
    }
}

const char* SustainedTurnPointProvenance() noexcept
{
    return "REQ-HABFM-09A sustained-turn operating point: published E-M "
        "V_corner_sus paired atomically with the safety-margined sustained "
        "total load N_sus at the same own altitude and admitted mass; omega "
        "and radius are derived under the horizontal coordinated-turn "
        "assumption; no extrapolation outside the published axes";
}

HabfmFrameEvidenceProvider::HabfmFrameEvidenceProvider() noexcept
{
    Reset();
}

void HabfmFrameEvidenceProvider::Reset() noexcept
{
    // Immutable rows are the Python lifetime cache; no episode state exists.
}

void HabfmFrameEvidenceProvider::SampleTacticalSpeedFloor(
    const double altitude_m,
    TacticalSpeedFloorSample& output) const noexcept
{
    guidance::doctrine::SampleTacticalSpeedFloor(altitude_m, output);
}

void HabfmFrameEvidenceProvider::Build(
    const DogfightGeometryFrame& frame,
    HabfmFrameEvidence& output,
    HabfmFrameEvidenceStatus& status) const noexcept
{
    output = HabfmFrameEvidence{};
    status = HabfmFrameEvidenceStatus::FrameStateNotFinite;

    if (!std::isfinite(frame.own.position_ned_m[2])
        || !std::isfinite(frame.opponent.position_ned_m[2])
        || !FiniteVector(frame.own.velocity_ned_mps)
        || !FiniteVector(frame.opponent.velocity_ned_mps))
    {
        return;
    }

    const double own_altitude_m = -frame.own.position_ned_m[2];
    const double adversary_altitude_m = -frame.opponent.position_ned_m[2];
    const double own_speed_mps = NumpyNorm3(frame.own.velocity_ned_mps);
    const double adversary_speed_mps =
        NumpyNorm3(frame.opponent.velocity_ned_mps);
    if (!std::isfinite(own_speed_mps)
        || !std::isfinite(adversary_speed_mps))
    {
        status = HabfmFrameEvidenceStatus::SpeedNormNotFinite;
        return;
    }

    output.own_altitude_m = guidance::em::EmValue{true, own_altitude_m};
    output.adversary_altitude_m =
        guidance::em::EmValue{true, adversary_altitude_m};
    output.own_speed_mps = guidance::em::EmValue{true, own_speed_mps};
    output.adversary_speed_mps =
        guidance::em::EmValue{true, adversary_speed_mps};

    output.adversary_corner_interval =
        corner_provider_.InstantaneousInterval(adversary_altitude_m);
    output.own_corner_interval =
        corner_provider_.InstantaneousInterval(own_altitude_m);
    output.own_sustained_corner_interval =
        corner_provider_.SustainedInterval(own_altitude_m);

    if (!output.own_sustained_corner_interval.admitted())
    {
        output.own_sustained_turn_point.status =
            output.own_sustained_corner_interval.status
                    == guidance::em::CornerIntervalStatus::
                        AltitudeOutsidePublishedAxis
                ? SustainedTurnPointStatus::AltitudeOutsidePublishedAxis
                : SustainedTurnPointStatus::PublishedLookupInvalid;
    }
    else
    {
        const double sustained_speed_mps =
            output.own_sustained_corner_interval.upper_mps.value;
        guidance::em::EmCellTrustReceipt cell_trust{};
        strict_envelope_.ObserveCellTrust(
            sustained_speed_mps,
            own_altitude_m,
            cell_trust);
        if (!cell_trust.lookup_valid())
        {
            output.own_sustained_turn_point.status =
                SustainedTurnPointStatus::PublishedLookupInvalid;
        }
        else if (!cell_trust.cell.trusted)
        {
            output.own_sustained_turn_point.status =
                SustainedTurnPointStatus::PublishedSustainedPointNotTrusted;
        }
        else
        {
            guidance::em::PublishedSustainedNLookup sustained_lookup{};
            strict_envelope_.ObservePublishedSustainedN(
                sustained_speed_mps,
                own_altitude_m,
                plant::dynamics::InitialMassKg,
                sustained_lookup);
            if (!sustained_lookup.lookup_valid())
            {
                output.own_sustained_turn_point.status =
                    SustainedTurnPointStatus::PublishedLookupInvalid;
            }
            else if (!sustained_lookup.trusted)
            {
                output.own_sustained_turn_point.status =
                    SustainedTurnPointStatus::
                        PublishedSustainedPointNotTrusted;
            }
            else
            {
                const double load_factor_g =
                    sustained_lookup.load_factor_g.value;
                if (load_factor_g <= 1.0)
                {
                    output.own_sustained_turn_point.status =
                        SustainedTurnPointStatus::
                            PublishedSustainedLoadNotAboveOneG;
                }
                else
                {
                    const double turn_rate_radps = kStandardGravityMps2
                        * std::sqrt(load_factor_g * load_factor_g - 1.0)
                        / sustained_speed_mps;
                    const double turn_radius_m =
                        sustained_speed_mps / turn_rate_radps;
                    if (!std::isfinite(turn_rate_radps)
                        || turn_rate_radps <= 0.0
                        || !std::isfinite(turn_radius_m)
                        || turn_radius_m <= 0.0)
                    {
                        output.own_sustained_turn_point.status =
                            SustainedTurnPointStatus::PublishedLookupInvalid;
                    }
                    else
                    {
                        SustainedTurnOperatingPoint& point =
                            output.own_sustained_turn_point;
                        point.status = SustainedTurnPointStatus::Admitted;
                        point.speed_mps = guidance::em::EmValue{
                            true,
                            sustained_speed_mps};
                        point.load_factor_g = guidance::em::EmValue{
                            true,
                            load_factor_g};
                        point.turn_rate_radps = guidance::em::EmValue{
                            true,
                            turn_rate_radps};
                        point.turn_radius_m = guidance::em::EmValue{
                            true,
                            turn_radius_m};
                        point.mass_kg = guidance::em::EmValue{
                            true,
                            plant::dynamics::InitialMassKg};
                    }
                }
            }
        }
    }

    const guidance::em::EnergyResolution own_resolution =
        corner_provider_.EnergyResolutionAt(own_altitude_m, own_speed_mps);
    const guidance::em::EnergyResolution adversary_resolution =
        corner_provider_.EnergyResolutionAt(
            adversary_altitude_m,
            adversary_speed_mps);
    if (!own_resolution.admitted())
    {
        output.merge_energy_resolution = own_resolution;
    }
    else if (!adversary_resolution.admitted())
    {
        output.merge_energy_resolution = adversary_resolution;
    }
    else if (own_resolution.resolution_m.value
        >= adversary_resolution.resolution_m.value)
    {
        output.merge_energy_resolution = own_resolution;
        output.merge_energy_resolution_owner = EnergyResolutionOwner::Ownship;
    }
    else
    {
        output.merge_energy_resolution = adversary_resolution;
        output.merge_energy_resolution_owner =
            EnergyResolutionOwner::Adversary;
    }

    SampleTacticalSpeedFloor(own_altitude_m, output.g17_speed_floor);
    status = HabfmFrameEvidenceStatus::Built;
}

} // namespace LadyLuck
