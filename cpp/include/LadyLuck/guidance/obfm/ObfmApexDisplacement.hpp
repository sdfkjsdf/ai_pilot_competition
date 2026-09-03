#pragma once

#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

enum class ObfmApexVerticalPhase : std::uint8_t
{
    Climbing = 0U,
    Descending = 1U,
    Level = 2U,
    NotObserved = 3U
};

// Only a measured climb produced by the applied G16 High command may arm
// Apex Displacement. Spacing and unrelated climbs cannot create this event.
enum class ObfmApexClimbOwner : std::uint8_t
{
    None = 0U,
    G16High = 1U,
    SpacingArrest = 2U
};

enum class ObfmApexDisplacementReason : std::uint8_t
{
    SelectorServiceNotReached = 0U,
    FrameEvidenceUnavailable = 1U,
    FeatureDisabled = 2U,
    G16HighClimbArmed = 3U,
    G16HighApexLatched = 4U,
    ApexNotLatched = 5U,
    LagReferenceUnavailable = 6U,
    TurnAuthorityUnavailable = 7U,
    DirectLagEntryAvailable = 8U,
    LateralDisplacementRequired = 9U,
    DecoratorNotReached = 10U,
    DecoratorNotAdmitted = 11U,
    DecoratorSelected = 12U,
    SafetySampleUnavailable = 13U,
    CommandReady = 14U,
    DeclaredReadyContradiction = 15U,
    UnownedClimbIgnored = 16U,
    SpacingClimbIgnored = 17U
};

const char* ObfmApexDisplacementReasonLabel(
    ObfmApexDisplacementReason reason) noexcept;

struct ObfmApexObservationReceipt
{
    bool evaluated = false;
    ObfmApexVerticalPhase vertical_phase =
        ObfmApexVerticalPhase::NotObserved;
    bool climb_armed = false;
    bool apex_latched = false;
    ObfmApexClimbOwner climb_owner = ObfmApexClimbOwner::None;
    ObfmApexDisplacementReason reason =
        ObfmApexDisplacementReason::FrameEvidenceUnavailable;
};

// The sole admission question is whether the available lateral acceleration
// can bend the current velocity onto the existing LAG aim vector before the
// aircraft reaches that point.
struct ObfmApexDisplacementReference
{
    bool evaluated = false;
    bool admitted = false;
    bool direct_lag_entry_available = false;
    bool apex_latched = false;
    std::int32_t away_side_sign = 0;
    Vector3 lag_point_ned_m{};
    double turn_radius_m = 0.0;
    double required_lateral_accel_mps2 = 0.0;
    double available_lateral_accel_mps2 = 0.0;
    ObfmApexDisplacementReason reason =
        ObfmApexDisplacementReason::FrameEvidenceUnavailable;
};

struct ObfmApexDisplacementServiceInput
{
    bool selector_service_reached = false;
    bool frame_evidence_declared_ready = false;
    bool feature_enabled = false;
    ObfmApexClimbOwner climb_owner = ObfmApexClimbOwner::None;
    bool turn_capability_available = false;
    double turn_capability_n_g = 0.0;
    bool lag_point_available = false;
    Vector3 lag_point_ned_m{};
    bool preferred_side_available = false;
    std::int32_t preferred_side_sign = 0;
};

struct ObfmApexDisplacementServiceReceipt
{
    ControlFrameIdentity frame_identity{};
    bool service_evaluated = false;
    bool selected_result = false;
    std::uint32_t selected_count = 0U;
    ObfmApexObservationReceipt apex{};
    ObfmApexDisplacementReference reference{};
    ObfmApexDisplacementReason reason =
        ObfmApexDisplacementReason::SelectorServiceNotReached;
};

struct ObfmApexDisplacementSelection
{
    ControlFrameIdentity frame_identity{};
    bool branch_reached = false;
    bool selected = false;
    std::uint32_t selection_count = 0U;
    ObfmApexDisplacementReason reason =
        ObfmApexDisplacementReason::DecoratorNotReached;
};

// Apex owns no longitudinal law. It consumes the same smoothed precision
// speed reference that the lower LAG leaf would publish.
struct ObfmApexDisplacementTaskInput
{
    bool safety_sample_available = false;
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    double total_load_factor_limit_g = 0.0;
    double capture_range_des_m = 0.0;
};

struct ObfmApexDisplacementCommand
{
    Vector3 aim_point_ned_m{};
    double desired_speed_mps = 0.0;
    double desired_speed_rate_mps2 = 0.0;
    bool path_inversion_allowed = false;
    double capture_range_des_m = 0.0;
    double total_load_factor_limit_g = 0.0;
};

struct ObfmApexDisplacementTaskReceipt
{
    ControlFrameIdentity frame_identity{};
    bool task_active = false;
    bool candidate_valid = false;
    std::uint32_t candidate_count = 0U;
    ObfmApexDisplacementCommand candidate{};
    ObfmApexDisplacementReason reason =
        ObfmApexDisplacementReason::SafetySampleUnavailable;
};

struct ObfmApexDisplacementHaltReceipt
{
    bool valid = false;
    bool was_active = false;
    bool clear_command_only_if_still_owner = true;
};

class ObfmApexDisplacement final
{
public:
    ObfmApexDisplacement() noexcept;
    void ResetEpisode() noexcept;
    void ObserveService(
        const DogfightGeometryFrame& frame,
        const ObfmApexDisplacementServiceInput& input,
        ObfmApexDisplacementServiceReceipt& output,
        Status& status) noexcept;
    void EvaluateDecorator(
        bool branch_reached,
        const ObfmApexDisplacementServiceReceipt& service,
        ObfmApexDisplacementSelection& output,
        Status& status) const noexcept;
    void EnterTask(
        const ObfmApexDisplacementServiceReceipt& service,
        const ObfmApexDisplacementSelection& selection,
        Status& status) noexcept;
    void TickTask(
        const DogfightGeometryFrame& frame,
        const ObfmApexDisplacementServiceReceipt& service,
        const ObfmApexDisplacementTaskInput& input,
        ObfmApexDisplacementTaskReceipt& output,
        Status& status) const noexcept;
    void HaltTask(ObfmApexDisplacementHaltReceipt& output) noexcept;

private:
    void ClearTaskLifecycle() noexcept;
    bool climb_armed_ = false;
    bool apex_latched_ = false;
    ObfmApexClimbOwner climb_owner_ = ObfmApexClimbOwner::None;
    bool task_active_ = false;
    std::int32_t away_side_sign_ = 0;
};

static_assert(
    std::is_trivially_copyable<ObfmApexDisplacementServiceInput>::value,
    "OBFM Apex Service input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmApexDisplacementServiceReceipt>::value,
    "OBFM Apex Service receipt must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<ObfmApexDisplacementTaskReceipt>::value,
    "OBFM Apex Task receipt must stay allocation-free.");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
