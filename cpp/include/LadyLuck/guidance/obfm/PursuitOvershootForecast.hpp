#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

const char* PursuitOvershootShadowProvenance() noexcept;
const char* PursuitEscapeWindowScope() noexcept;

struct PursuitOptionalDouble
{
    bool has_value = false;
    double value = 0.0;
};

enum class PursuitEscapeWindowStatus : std::uint8_t
{
    Open = 0U,
    Closed = 1U,
    Unresolved = 2U
};

enum class PursuitEscapeWindowReason : std::uint8_t
{
    OpponentTurnCapabilityNotAdmitted = 0U,
    WeaponRangeExitBeforeOpponentFace = 1U,
    OpponentFaceBeforeOrAtWeaponRangeExit = 2U
};

const char* PursuitEscapeWindowReasonLabel(
    PursuitEscapeWindowReason reason) noexcept;

struct PursuitEscapeWindowObservation
{
    bool available = false;
    bool admitted = false;
    PursuitEscapeWindowStatus status =
        PursuitEscapeWindowStatus::Unresolved;
    PursuitOptionalDouble opponent_capability_g{};
    double weapon_outer_range_m = 0.0;
    double initial_range_m = 0.0;
    PursuitOptionalDouble range_at_bite_m{};
    PursuitOptionalDouble initial_opponent_face_angle_rad{};
    PursuitOptionalDouble opponent_maximum_turn_rate_rad_s{};
    PursuitOptionalDouble time_to_bite_s{};
    PursuitOptionalDouble time_to_weapon_range_exit_s{};
    PursuitOptionalDouble time_margin_s{};
    PursuitEscapeWindowReason reason =
        PursuitEscapeWindowReason::OpponentTurnCapabilityNotAdmitted;
    bool tactical_command_authority = false;
    bool production_authority = false;
};

enum class PursuitOvershootForecastStatus : std::uint8_t
{
    Forced = 0U,
    NotForced = 1U,
    Unresolved = 2U
};

enum class PursuitOvershootForecastReason : std::uint8_t
{
    PublicationNotAdmitted = 0U,
    FrameStateNotFinite = 1U,
    AdversaryCourseNotResolved = 2U,
    OwnAlreadyAhead = 3U,
    AlongTrackSignNotResolved = 4U,
    ClosureNotResolvedPositive = 5U,
    PublicationDomainNotTrusted = 6U,
    MaintainedPursuitOvershootForced = 7U,
    ArrestNotRefuted = 8U
};

const char* PursuitOvershootForecastReasonLabel(
    PursuitOvershootForecastReason reason) noexcept;

struct PursuitOvershootForecast
{
    bool valid = false;
    PursuitOvershootForecastStatus status =
        PursuitOvershootForecastStatus::Unresolved;
    PursuitOvershootForecastReason reason =
        PursuitOvershootForecastReason::PublicationNotAdmitted;
    bool maintained_passage_projected = false;
    PursuitOptionalDouble along_track_m{};
    PursuitOptionalDouble closure_mps{};
    PursuitOptionalDouble braking_distance_m{};
    PursuitOptionalDouble optimistic_arrest_mps2{};
    bool branch_a_admitted = false;
    PursuitOptionalDouble energy_standing_gap_m{};
    PursuitOptionalDouble zoom_budget_m{};
    PursuitOptionalDouble crossing_angle_rad{};
    PursuitOptionalDouble speed_advantage_mps{};
    PursuitEscapeWindowObservation escape_window{};
};

struct PursuitOvershootForecasterConfig
{
    // false is the exact Python mass_kg=None/reference-mass path.  The backing
    // scalar is ignored in that case.
    bool mass_kg_available = false;
    double mass_kg = 0.0;
};

// Pure per-frame d90 producer.  It issues no guidance or flight-control
// command, owns no latch, and mutates no history.
class PursuitOvershootForecaster final
{
public:
    PursuitOvershootForecaster() noexcept;
    explicit PursuitOvershootForecaster(
        const PursuitOvershootForecasterConfig& config) noexcept;

    void Update(
        const DogfightGeometryFrame& frame,
        PursuitOvershootForecast& output,
        Status& status) const noexcept;

private:
    PursuitOvershootForecasterConfig config_{};
    em::StrictEnergyManeuverEnvelope envelope_{};
};

static_assert(
    std::is_trivially_copyable<PursuitOvershootForecast>::value,
    "Pursuit forecast receipts must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
