#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/doctrine/ModeDecision.hpp"
#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

// Frozen production capability transform from add/main d90
// mode_turn_circle_capability.py.  It characterizes a maximum coordinated
// turn circle; it is neither a load command nor aircraft-response evidence.
constexpr double TurnCirclePositiveCapabilityScale =
    1.1953828780506852;
constexpr double TurnCircleMaximumCommandLoadFactorG = 9.0;

enum class TurnCircleEventOrder : std::uint8_t
{
    ReachFirst = 0U,
    FaceFirstOrEqual = 1U,
    NotEvaluated = 2U
};

enum class DirectionalTurnCircleReason : std::uint8_t
{
    ReachFirst = 0U,
    FaceFirstOrEqual = 1U,
    TurnCircleKinematicsUnavailable = 2U,
    CapabilityNotAdmitted = 3U
};

enum class BilateralTurnCircleDominanceStatus : std::uint8_t
{
    OwnAdvantageProven = 0U,
    AdversaryAdvantageProven = 1U,
    NeutralProven = 2U,
    HabfmFallback = 3U
};

struct OptionalTurnCircleScalar
{
    bool has_value = false;
    double value = 0.0;
};

struct ManualTurnCircleCapabilityReceipt
{
    em::CharacterizedRawNLookup raw_lookup{};
    OptionalTurnCircleScalar capability_g{};
    // `admitted` means a finite bound is available for the positional
    // turn-circle classifier. Physical authority is tracked separately so a
    // fixed 9-g command bound is never reported as measured E-M capability.
    bool admitted = false;
    bool physical_authority = false;
    bool fixed_command_bound = false;
};

struct DoctrineTurnCircleEvents
{
    bool valid = false;
    OptionalTurnCircleScalar reach_time_s{};
    OptionalTurnCircleScalar face_time_s{};
    OptionalTurnCircleScalar reach_not_observed_through_s{};
    TurnCircleEventOrder event_order = TurnCircleEventOrder::NotEvaluated;
    double turn_radius_m = 0.0;
    double maximum_turn_rate_rad_s = 0.0;
    double initial_face_angle_rad = 0.0;
};

struct DirectionalDoctrineTurnCircleReceipt
{
    bool attacker_rear_halfspace = false;
    bool attacker_nose_toward = false;
    bool events_available = false;
    DoctrineTurnCircleEvents events{};
    ManualTurnCircleCapabilityReceipt capability{};
    DirectionalTurnCircleReason reason =
        DirectionalTurnCircleReason::TurnCircleKinematicsUnavailable;
};

struct BilateralDoctrineTurnCircleReceipt
{
    bool valid = false;
    bool evaluated = false;
    DirectionalDoctrineTurnCircleReceipt own_attack{};
    DirectionalDoctrineTurnCircleReceipt opponent_attack{};
    TacticalMode candidate_mode = TacticalMode::Habfm;
    BilateralTurnCircleDominanceStatus dominance_status =
        BilateralTurnCircleDominanceStatus::HabfmFallback;
    bool tactical_mode_authority = false;
    bool production_authority = false;
};

class ManualTurnCircleCapabilityProvider final
{
public:
    void Observe(
        double speed_mps,
        double altitude_m,
        ManualTurnCircleCapabilityReceipt& output,
        Status& status) const noexcept;

private:
    em::StrictEnergyManeuverEnvelope envelope_{};
};

// Allocation-free mathematical core corresponding to
// bilateral_manual_turn_circle_from_kinematics.  The supplied capability
// receipts make the canonical manual 3000-ft/2000-ft examples independently
// testable without copying or replacing the production E-M table.
void EvaluateBilateralManualTurnCircleFromCapabilities(
    const DogfightGeometryFrame& frame,
    const ManualTurnCircleCapabilityReceipt& own_capability,
    const ManualTurnCircleCapabilityReceipt& opponent_capability,
    bool production_authority,
    BilateralDoctrineTurnCircleReceipt& output,
    Status& status) noexcept;

class BilateralTurnCircleRootAuthority final
{
public:
    // This provider only publishes a positional-mode receipt.  Safety and gun
    // remain senior ModeDecision bypasses, and no raw guidance/FCS command is
    // emitted here.
    void Observe(
        const DogfightGeometryFrame& frame,
        BilateralDoctrineTurnCircleReceipt& output,
        Status& status) const noexcept;

private:
    ManualTurnCircleCapabilityProvider capability_provider_{};
};

static_assert(
    std::is_trivially_copyable<ManualTurnCircleCapabilityReceipt>::value,
    "Turn-circle capability receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<BilateralDoctrineTurnCircleReceipt>::value,
    "Bilateral turn-circle receipt must remain allocation-free.");

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
