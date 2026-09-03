#include "LadyLuck/guidance/doctrine/TacticalSpeedFloorObserver.hpp"

#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace
{

constexpr std::size_t FloorRowCount = 25U;
constexpr double PublishedAltitudeMinimumM = 0.0;
constexpr double PublishedAltitudeMaximumM = 12000.0;
constexpr char ExpectedEmTableSha256[] =
    "C808635A214D71CE29B6177C7E22E7C8C81BE7F65D49F20D05F66D4371DF60F8";

// Exact d90 TacticalSpeedFloorObserver() memo rows at reference mass. Every
// row used dt=0.1 s half-loop integration and the 24-step B2 bisection.
const double FloorAltitudeAxisM[FloorRowCount] =
{
    0.0, 500.0, 1000.0, 1500.0, 2000.0,
    2500.0, 3000.0, 3500.0, 4000.0, 4500.0,
    5000.0, 5500.0, 6000.0, 6500.0, 7000.0,
    7500.0, 8000.0, 8500.0, 9000.0, 9500.0,
    10000.0, 10500.0, 11000.0, 11500.0, 12000.0
};

const double FloorSpeedMps[FloorRowCount] =
{
    222.03665018081665, 222.67244935035706, 224.10747647285461,
    226.51937246322632, 229.56234991550446, 232.81911253929138,
    236.27682447433472, 240.57267427444458, 245.93653619289398,
    251.21739864349365, 256.3754868507385, 261.76043033599854,
    268.0745905637741, 276.44224643707275, 288.99475395679474,
    308.8294440507889, 339.9457573890686, 359.0593546628952,
    370.4120343923569, 378.5536676645279, 384.9106287956238,
    389.8245745897293, 393.24701607227325, 395.1867711544037,
    395.81242084503174
};

const double FloorApexAltitudeM[FloorRowCount] =
{
    1783.4536680761369, 2297.890600187193, 2830.641476927408,
    3386.160216004554, 3957.0447601237483, 4533.955807604633,
    5116.795006571452, 5708.815008643405, 6312.869497181771,
    6923.523383901308, 7532.033605213218, 8146.996398435266,
    8787.996979958705, 9470.979673507461, 10257.130082357571,
    11248.11889937539, 12674.468984302925, 13856.489879303283,
    14779.18014340245, 15590.369503646816, 16338.018611353991,
    17032.25221595193, 17668.966981781505, 18246.973732910657,
    18772.212262441793
};

std::size_t ConservativeFloorCell(const double altitude_m) noexcept
{
    // Exact np.searchsorted(axis, altitude), clipped to [0, last].
    for (std::size_t index = 0U; index < FloorRowCount; ++index)
    {
        if (altitude_m <= FloorAltitudeAxisM[index])
        {
            return index;
        }
    }
    return FloorRowCount - 1U;
}

bool FixedFloorAuthorityAdmitted() noexcept
{
    const LadyLuck::guidance::em::EmTableAuthority authority =
        LadyLuck::guidance::em::StrictEnergyManeuverEnvelope::Authority();
    return authority.table_identity_valid
        && authority.schema_valid
        && authority.provenance_valid
        && authority.table_sha256 != nullptr
        && std::strcmp(authority.table_sha256, ExpectedEmTableSha256) == 0;
}

} // namespace

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

void SampleTacticalSpeedFloor(
    const double altitude_m,
    TacticalSpeedFloorSample& output) noexcept
{
    output = TacticalSpeedFloorSample{};
    if (!std::isfinite(altitude_m))
    {
        output.status = TacticalSpeedFloorStatus::AltitudeNotFinite;
        return;
    }
    if (!FixedFloorAuthorityAdmitted())
    {
        output.status =
            TacticalSpeedFloorStatus::EmPublicationNotAdmitted;
        return;
    }
    if (altitude_m < PublishedAltitudeMinimumM
        || altitude_m > PublishedAltitudeMaximumM)
    {
        output.status =
            TacticalSpeedFloorStatus::AltitudeOutsidePublishedAxis;
        return;
    }

    const std::size_t cell = ConservativeFloorCell(altitude_m);
    const double floor_mps = FloorSpeedMps[cell];
    const double apex_altitude_m = FloorApexAltitudeM[cell];
    const double cell_altitude_m = FloorAltitudeAxisM[cell];
    // The frozen Python cache endpoint-clamps both the query and the
    // per-step E-M publication.  Rows whose derived half loop leaves the
    // published altitude axis therefore carry a finite number but no
    // published maneuver authority.  Treat that as ordinary unavailable
    // evidence so the caller retains its same-frame base command.
    if (!std::isfinite(floor_mps)
        || !std::isfinite(apex_altitude_m)
        || !std::isfinite(cell_altitude_m)
        || floor_mps <= 0.0
        || cell_altitude_m < PublishedAltitudeMinimumM
        || cell_altitude_m > PublishedAltitudeMaximumM
        || apex_altitude_m < cell_altitude_m
        || apex_altitude_m > PublishedAltitudeMaximumM)
    {
        output.status = TacticalSpeedFloorStatus::
            DerivedTrajectoryOutsidePublishedAxis;
        return;
    }
    output.status =
        TacticalSpeedFloorStatus::DerivedByHalfLoopBisection;
    output.floor_mps = em::EmValue{true, floor_mps};
    output.apex_altitude_m =
        em::EmValue{true, apex_altitude_m};
    output.cell_altitude_m =
        em::EmValue{true, cell_altitude_m};
}

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
