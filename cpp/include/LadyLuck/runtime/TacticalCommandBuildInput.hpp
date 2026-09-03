#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/EstimatorOutput.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/control/route5/ReferenceGovernor.hpp"
#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/obfm/ObfmLongitudinalAuthority.hpp"
#include "LadyLuck/runtime/TacticalCompletedTotalLoadReceipt.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

enum class TacticalFeedbackFreshness : std::int32_t
{
    Missing = 0,
    Fresh = 1,
    Stale = 2
};

enum class TacticalControlBackendId : std::int32_t
{
    Unavailable = 0,
    TecsCisV4 = 1,
    AutoGcasRecovery = 2
};

enum class TacticalFeedbackSourceKind : std::int32_t
{
    Unavailable = 0,
    AcceptedEstimatorFrame = 1,
    RejectedCommandFrame = 2
};

// Completed previous-sample control evidence.  Every backing scalar remains
// finite; each physical or diagnostic authority has an explicit validity bit.
// `valid` means a final command completed and was authorized on that sample,
// not that every optional authority below was available.
struct TacticalAge1ControlFeedback
{
    bool valid = false;
    TacticalFeedbackSourceKind source_kind =
        TacticalFeedbackSourceKind::Unavailable;
    bool source_frame_index_valid = false;
    std::uint64_t source_frame_index = 0U;
    ControlFrameIdentity source_frame_identity{};
    double source_decision_time_s = 0.0;
    TacticalControlBackendId command_backend_id =
        TacticalControlBackendId::Unavailable;
    std::uint32_t writer_id = ControlIntentWriterNone;
    DoctrineBehaviorId behavior_id = DoctrineBehaviorId::Invalid;
    DoctrineModeId mode_id = DoctrineModeId::Invalid;
    control::tecs_cis::NormalizedControlCommand transmitted_command{};
    bool cis_integrity_valid = false;
    bool cis_clipped = false;
    bool cis_fallback = false;
    bool energy_lower_saturated = false;
    bool energy_upper_saturated = false;
    bool energy_rate_measurement_valid = false;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    bool energy_rate_authority_valid = false;
    double energy_authority_mass_kg = 0.0;
    control::tecs_cis::TecsCisCompletedEnergyAuthorityReceipt
        completed_energy_authority{};
    TacticalCompletedTotalLoadReceipt completed_total_load{};
    bool nz_cmd_governed_valid = false;
    double nz_cmd_governed_g = 0.0;
    bool nz_measured_valid = false;
    double nz_measured_g = 0.0;
    bool nz_feasible_valid = false;
    double nz_feasible_g = 0.0;
    bool auto_gcas_active = false;
    safety::AutoGcasPhase auto_gcas_phase =
        safety::AutoGcasPhase::Inactive;
    bool auto_gcas_state_valid = false;
    bool auto_gcas_fault = false;
};

// Same-frame sources that the Python adapter reads before the doctrine tick:
// current CIS-v4 desired-speed rate bounds, N-mu gamma limit, and the exact
// corrected gear-up NZFEAS governor source.  No tactical value is synthesized
// when any source is unavailable.
struct TacticalCurrentLongitudinalAuthorityEvidence
{
    bool valid = false;
    control::tecs_cis::TecsCisLongitudinalAuthorityConfiguration
        tecs_configuration{};
    bool flight_path_gamma_limit_valid = false;
    double flight_path_gamma_limit_rad = 0.0;
    control::route5::ReferenceGovernorNzfeasAuthorityReceipt nzfeas{};
};

// Exact pre-tactical current-frame evidence seam.  The raw guidance request
// is built from this receipt; it is not a body-rate, load, surface, thrust, or
// aircraft-response result.
struct TacticalCommandBuildInput
{
    bool valid = false;
    DogfightGeometryFrame frame{};
    EstimatorOutputV6 accepted_estimator{};
    control::route5::CommandEnvelope current_envelope{};
    // True for either characterized physical limits or the finite fixed
    // command-containment bounds.  This is sufficient for guidance/FCS
    // clipping, but must not be cited as measured aircraft capability.
    bool current_command_envelope_available = false;
    bool current_physical_envelope_available = false;
    safety::AutoGcasEntryReceipt current_safety{};
    TacticalCurrentLongitudinalAuthorityEvidence
        current_longitudinal_evidence{};
    TacticalAge1ControlFeedback previous_control_feedback{};
    TacticalFeedbackFreshness feedback_freshness =
        TacticalFeedbackFreshness::Missing;
    std::uint64_t feedback_frame_age = 0U;
    ObfmLongitudinalAuthorityReceipt obfm_longitudinal_authority{};
};

inline bool CurrentCommandEnvelopeAvailable(
    const TacticalCommandBuildInput& input) noexcept
{
    // `current_physical_envelope_available` implies bounded command
    // authority and is retained here for direct unit fixtures built before
    // the explicit containment field was introduced.
    return input.current_command_envelope_available
        || input.current_physical_envelope_available;
}

class TacticalCommandBuildInputBuilder final
{
public:
    TacticalCommandBuildInputBuilder() noexcept = default;

    void Build(
        const DogfightGeometryFrame& frame,
        const EstimatorOutputV6& accepted_estimator,
        const control::route5::CommandEnvelope& current_envelope,
        const safety::AutoGcasEntryReceipt& current_safety,
        const TacticalCurrentLongitudinalAuthorityEvidence&
            current_longitudinal_evidence,
        const TacticalAge1ControlFeedback& previous_control_feedback,
        TacticalCommandBuildInput& output,
        Status& status) const noexcept;

    void PrepareFeedback(
        const ControlFrameIdentity& source_frame_identity,
        const ControlIntent& tactical,
        const EstimatorOutputV6& estimate,
        const control::route5::CommandEnvelope& envelope,
        const control::tecs_cis::BodyRateLoadEnergyCommand& reference,
        const control::tecs_cis::TecsCisOutput& control,
        const TacticalCompletedTotalLoadReceipt& completed_total_load,
        TacticalAge1ControlFeedback& output,
        Status& status) const noexcept;

    void CompleteFeedback(
        const safety::AutoGcasReceipt& auto_gcas,
        double source_decision_time_s,
        const control::tecs_cis::NormalizedControlCommand&
            transmitted_wire_command,
        TacticalAge1ControlFeedback& output,
        Status& status) const noexcept;

};

static_assert(
    std::is_trivially_copyable<TacticalAge1ControlFeedback>::value,
    "Tactical age-1 feedback must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<TacticalCommandBuildInput>::value,
    "Tactical build input must stay allocation-free.");
static_assert(
    std::is_trivially_copyable<
        TacticalCurrentLongitudinalAuthorityEvidence>::value,
    "Tactical current authority evidence must stay allocation-free.");

} // namespace runtime
} // namespace LadyLuck
