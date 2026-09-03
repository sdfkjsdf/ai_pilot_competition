#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"
#include "LadyLuck/runtime/TacticalCommandBuildInput.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace committed
{

enum class G16EscapeWindowStatus : std::uint8_t
{
    Open = 0U,
    Closed = 1U,
    Unresolved = 2U
};

enum class G16PreventionStatus : std::uint8_t
{
    Unresolved = 0U,
    NotRefuted = 1U,
    LocalInfeasible = 2U
};

enum class G16HandoffStatus : std::uint8_t
{
    Requested = 0U,
    NotRequested = 1U,
    Unresolved = 2U
};

enum class G16EvidenceReason : std::uint8_t
{
    Admitted = 0U,
    ThreeCausalSamplesNotInitialized = 1U,
    SampleTimeDiscontinuous = 2U,
    SameIndexSourceUnavailable = 3U,
    CurrentCommandEnvelopeUnavailable = 4U,
    FreshCompletedEnergyAuthorityUnavailable = 5U,
    OwnTurnCapabilityUnavailable = 6U,
    OpponentTurnCapabilityUnavailable = 7U,
    DefenderFlightPathUnobservable = 8U,
    BoundaryUncertaintyUnavailable = 9U,
    StrictlyBehindBoundaryUnresolved = 10U,
    BoundaryClosureNotResolved = 11U,
    OptimisticPreventionNotRefuted = 12U,
    EscapeWindowClosed = 13U,
    TurnSideUnresolved = 14U,
    EgressCandidate = 15U,
    InvalidSourceContract = 16U,
    ArithmeticInvalid = 17U
};

struct G16TurnCapabilityReceipt
{
    guidance::em::CharacterizedRawNLookup raw_lookup{};
    double capability_g = 0.0;
    bool admitted = false;
    bool physical_authority = false;
    bool fixed_command_bound = false;
};

struct G16BoundaryReceipt
{
    bool valid = false;
    double signed_margin_m = 0.0;
    double margin_rate_mps = 0.0;
    double noncontrol_margin_accel_mps2 = 0.0;
    double signed_margin_error_bound_m = 0.0;
    double margin_rate_error_bound_mps = 0.0;
    double noncontrol_margin_accel_error_bound_mps2 = 0.0;
    Vector3 positive_margin_direction_ned{};
    Vector3 own_acceleration_ned_mps2{};
    bool defender_turn_side_resolved = false;
    Vector3 defender_turn_acceleration_direction_ned{};
    double defender_turn_direction_resolution_rad = 0.0;
    double defender_turn_support_rotation_rad = 0.0;
    double defender_turn_support_rotation_resolution_rad = 0.0;
    double derivative_support_duration_s = 0.0;
};

struct G16EnemyRangeIntervalReceipt
{
    bool valid = false;
    double error_bound_m = 0.0;
    double lower_m = 0.0;
    double upper_m = 0.0;
};

// Command-neutral, identity-bound evidence for the production G16-E owner.
// It contains current aircraft geometry and bounded capability observations;
// it is not a guidance command, a p/q/r/Nz reference, or aircraft response.
struct G16ProductionEvidenceReceipt
{
    bool valid = false;
    ControlFrameIdentity frame_identity{};
    DogfightGeometryFrame frame{};
    bool source_simultaneous = false;
    G16BoundaryReceipt boundary{};
    G16TurnCapabilityReceipt own_turn_capability{};
    G16TurnCapabilityReceipt opponent_turn_capability{};
    control::tecs_cis::TecsCisCompletedEnergyAuthorityReceipt
        previous_completed_energy_authority{};
    G16EscapeWindowStatus escape_window_status =
        G16EscapeWindowStatus::Unresolved;
    bool escape_window_admitted = false;
    G16PreventionStatus prevention_status = G16PreventionStatus::Unresolved;
    bool prevention_failure_candidate = false;
    double positive_margin_control_accel_required_mps2 = 0.0;
    bool positive_margin_control_accel_required_valid = false;
    double positive_margin_control_accel_max_mps2 = 0.0;
    bool positive_margin_control_accel_max_valid = false;
    G16HandoffStatus handoff_status = G16HandoffStatus::Unresolved;
    bool selected_egress_side_resolved = false;
    std::int32_t selected_egress_side_sign = 0;
    double own_speed_mps = 0.0;
    double enemy_range_m = 0.0;
    G16EnemyRangeIntervalReceipt enemy_range_interval{};
    double enemy_outer_wez_range_m = 0.0;
    bool own_velocity_direction_resolution_valid = false;
    double own_velocity_direction_resolution_rad = 0.0;
    bool official_employ_active = false;
    G16EvidenceReason reason = G16EvidenceReason::InvalidSourceContract;
};

class G16ProductionEvidenceProvider final
{
public:
    G16ProductionEvidenceProvider() noexcept = default;

    void Reset() noexcept;
    void Observe(
        const runtime::TacticalCommandBuildInput& input,
        G16ProductionEvidenceReceipt& output,
        Status& status) noexcept;

private:
    struct Sample
    {
        double time_s = 0.0;
        Vector3 own_velocity_ned_mps{};
        Vector3 defender_velocity_ned_mps{};
        Vector3 defender_nose_ned{};
        double own_velocity_error_bound_mps = 0.0;
        double defender_velocity_error_bound_mps = 0.0;
        double defender_nose_error_bound = 0.0;
        double defender_direction_resolution_rad = 0.0;
    };

    void SeedSample(const Sample& sample) noexcept;
    void BuildBoundary(
        const runtime::TacticalCommandBuildInput& input,
        G16BoundaryReceipt& output,
        Status& status) noexcept;
    void BuildCapabilitiesAndDecision(
        const runtime::TacticalCommandBuildInput& input,
        G16ProductionEvidenceReceipt& output,
        Status& status) noexcept;

    std::array<Sample, 3U> samples_{};
    std::size_t sample_count_ = 0U;
    guidance::em::StrictEnergyManeuverEnvelope em_envelope_{};
};

static_assert(
    std::is_trivially_copyable<G16ProductionEvidenceReceipt>::value,
    "G16 evidence must remain allocation-free.");

} // namespace committed
} // namespace guidance
} // namespace LadyLuck
