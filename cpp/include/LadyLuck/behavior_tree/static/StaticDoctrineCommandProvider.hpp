#pragma once

#include "LadyLuck/behavior_tree/static/StaticImmediateGunResponseStagedOwner.hpp"
#include "LadyLuck/behavior_tree/static/StaticBtAtomicEvaluator.hpp"
#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bAdapter.hpp"
#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bInputBuilder.hpp"
#include "LadyLuck/behavior_tree/static/StaticDoctrineObfmG16G5bOwner.hpp"
#include "LadyLuck/behavior_tree/static/StaticHabfmG10StagedOwner.hpp"
#include "LadyLuck/behavior_tree/static/StaticSafetyGunStagedOwner.hpp"
#include "LadyLuck/guidance/GunDefenseControlIntent.hpp"
#include "LadyLuck/guidance/dbfm/DbfmBreakLoadControlIntent.hpp"
#include "LadyLuck/guidance/dbfm/DbfmBreakMargin.hpp"
#include "LadyLuck/guidance/dbfm/DbfmAltitudeSeparated.hpp"
#include "LadyLuck/guidance/dbfm/DbfmEscapeEnergy.hpp"
#include "LadyLuck/guidance/dbfm/DbfmHardTurnControlIntent.hpp"
#include "LadyLuck/guidance/habfm/HabfmFrameEvidenceProvider.hpp"
#include "LadyLuck/runtime/TacticalControlCore.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class StaticDoctrineRouteImplementation : std::uint8_t
{
    NotEvaluated = 0U,
    Implemented = 1U,
    NotImplemented = 2U
};

enum class StaticDoctrineCandidateDisposition : std::uint8_t
{
    NotEvaluated = 0U,
    Selected = 1U,
    NotApplicable = 2U,
    Fault = 3U
};

enum class StaticDoctrinePreparedOwner : std::uint8_t
{
    None = 0U,
    Habfm = 2U,
    Dbfm = 3U,
    SafetyGun = 4U,
    ObfmG16G5b = 5U
};

enum class StaticDoctrineCommandProviderReason : std::uint8_t
{
    NotEvaluated = 0U,
    DbfmHardTurnSelected = 1U,
    HabfmSelected = 2U,
    HabfmNotImplemented = 3U,
    InputNotAdmitted = 4U,
    RootObservationFault = 5U,
    RootReceiptContractFault = 6U,
    TacticalModeOutOfRange = 7U,
    DbfmFrameEvidenceUnavailable = 8U,
    DbfmGeometryObservationFault = 9U,
    DbfmGeometryNotApplicable = 10U,
    DbfmHardTurnBuildFault = 11U,
    DbfmHardTurnNotApplicable = 12U,
    DbfmWriterContractFault = 13U,
    DbfmDefenseSpeedFault = 14U,
    ObfmLagSelected = 15U,
    ObfmOwnSpeedUnavailable = 16U,
    ObfmStationObservationFault = 17U,
    ObfmLagPreparationFault = 18U,
    ObfmLagBuildFault = 19U,
    ObfmWriterContractFault = 20U,
    ObfmLagCommitFault = 21U,
    HabfmObservationFault = 24U,
    HabfmNotApplicable = 25U,
    HabfmPreparationFault = 26U,
    HabfmWriterContractFault = 27U,
    HabfmCommitFault = 28U,
    AutoGcasSelected = 29U,
    ImmediateGunDefenseSelected = 30U,
    SafetyGunPreparationFault = 31U,
    ImmediateGunSnapshotSelected = 32U,
    GunSnapshotPreparationFault = 33U,
    ObfmInputBuildFault = 34U,
    ObfmLifecycleFault = 35U,
    ObfmPreparedStateFault = 36U,
    ObfmG16CommittedSelected = 37U,
    ObfmG16HighSelected = 38U,
    ObfmG5bSelected = 39U,
    ImmediateGunG4HighGSelected = 41U,
    ImmediateGunPhaseHardTurnSelected = 43U,
    ImmediateGunResponsePreparationFault = 44U,
    ObfmLagSelectedAfterTacticalFault = 45U,
    DbfmBreakSelected = 46U,
    ObfmEmploySelected = 47U,
    ObfmEntrySelected = 48U,
    CurrentEffectEmploySelected = 49U,
    HabfmG10SecondUseSelected = 50U,
    HabfmAvoidPassSelected = 51U,
    ObfmSpacingSelected = 52U,
    DbfmAltitudeSeparatedSelected = 53U,
    ObfmG3CounterBarrelSelected = 54U,
    ObfmG3CounterRollingScissorsSelected = 55U,
    ObfmG3ScissorsSelected = 56U,
    ObfmApexSelected = 57U
};

// Fixed diagnostic state for one provider call.  This contains raw guidance
// evidence and candidates only; it is not an FCS command or aircraft-response
// receipt. The fixed OBFM spine selects writer 10, 6, 8, 5, 7, 17, 18, 21, 28, or 29 and
// retains ordinary writer 5 as its total lower route. HABFM selects avoid-pass writer
// 11, G10 writer 19, or its writer-4 base (optionally promoted to H09
// Direct-Load); DBFM selects BREAK writer 22, altitude-separated writer 23,
// or terminal HARD_TURN writer 3.
struct StaticDoctrineCommandProviderSnapshot
{
    bool build_attempted = false;
    bool input_admitted = false;
    bool root_classification_available = false;
    bool projection_port_supplied = false;
    bool projection_port_used = false;
    bool prepared_transaction_ready = false;
    bool prepared_transaction_committed = false;
    ControlFrameIdentity prepared_frame_identity{};
    std::uint32_t prepared_writer_id = ControlIntentWriterNone;

    StaticSafetyGunPreparedReceipt safety_gun{};
    bool safety_current_required = false;
    bool safety_continuation_required = false;
    bool safety_selected = false;
    runtime::RootGunPreTaskEvidence gun_evidence{};
    StaticImmediateGunDefenseAdmissionReceipt gun_admission{};
    bool gun_selected = false;
    StaticImmediateGunResponsePreparedReceipt gun_response{};
    // Compatibility projection of gun_response.snapshot for existing
    // diagnostics. It is not a second owner or transaction.
    StaticGunSnapshotPreparedReceipt gun_snapshot{};

    ControlFrameIdentity frame_identity{};
    guidance::doctrine::BilateralDoctrineTurnCircleReceipt root_receipt{};
    StatusCode root_status_code = StatusCode::Ok;
    guidance::doctrine::TacticalMode classified_mode =
        guidance::doctrine::TacticalMode::Habfm;
    StaticDoctrineObfmG16G5bInputBuilderReceipt obfm_input_builder{};
    StaticDoctrineObfmG16G5bSnapshot obfm_owner{};
    StaticDoctrineObfmG16G5bResult obfm_result{};
    BtTickResult obfm_adapter_commit{};

    StaticDoctrineRouteImplementation obfm_implementation =
        StaticDoctrineRouteImplementation::Implemented;
    StaticDoctrineRouteImplementation habfm_implementation =
        StaticDoctrineRouteImplementation::Implemented;
    StaticDoctrineRouteImplementation dbfm_implementation =
        StaticDoctrineRouteImplementation::Implemented;
    StaticDoctrineRouteImplementation classified_route_implementation =
        StaticDoctrineRouteImplementation::NotEvaluated;
    StaticDoctrineCandidateDisposition candidate_disposition =
        StaticDoctrineCandidateDisposition::NotEvaluated;
    StaticDoctrineCommandProviderReason reason =
        StaticDoctrineCommandProviderReason::NotEvaluated;

    HabfmFrameEvidence dbfm_frame_evidence{};
    HabfmFrameEvidenceStatus dbfm_frame_evidence_status =
        HabfmFrameEvidenceStatus::FrameStateNotFinite;
    DbfmHardTurnGeometryReceipt dbfm_geometry{};
    StatusCode dbfm_geometry_status_code = StatusCode::Ok;
    guidance::dbfm::DbfmBreakMarginTurnCapabilityReceipt
        dbfm_break_capability{};
    guidance::dbfm::DbfmBreakMarginReceipt dbfm_break_margin{};
    StatusCode dbfm_break_margin_status_code = StatusCode::Ok;
    HorizontalBreakReferenceReceipt dbfm_break_reference{};
    StatusCode dbfm_break_reference_status_code = StatusCode::Ok;
    StatusCode dbfm_break_load_status_code = StatusCode::Ok;
    bool dbfm_break_load_legacy_fallback = false;
    guidance::dbfm::DbfmAltitudeSeparatedReceipt
        dbfm_altitude_separated{};
    StatusCode dbfm_altitude_separated_status_code = StatusCode::Ok;
    guidance::dbfm::DbfmEscapeEnergyReceipt dbfm_escape_energy{};
    StatusCode dbfm_escape_energy_status_code = StatusCode::Ok;
    BtTickResult dbfm_selector_result{};
    bool dbfm_break_candidate_available = false;
    bool dbfm_altitude_separated_candidate_available = false;
    bool dbfm_hard_turn_candidate_available = false;
    ControlIntent raw_candidate{};
    StatusCode raw_candidate_status_code = StatusCode::Ok;
    ControlIntent selected_candidate{};
    StatusCode selected_candidate_status_code = StatusCode::Ok;
    StatusCode provider_status_code = StatusCode::InvalidConfiguration;

    bool obfm_current_speed_echo_ready = false;
    double obfm_current_speed_echo_mps = 0.0;
    ObfmLagSpeedAuthority obfm_speed_authority =
        ObfmLagSpeedAuthority::Unavailable;
    ObfmStationHoldServiceReceipt obfm_station_hold{};
    StatusCode obfm_station_status_code = StatusCode::Ok;
    ObfmLagGuidancePreparation obfm_lag_preparation{};
    StatusCode obfm_lag_preparation_status_code = StatusCode::Ok;
    ObfmLongitudinalProviderReceipt obfm_longitudinal{};
    StatusCode obfm_longitudinal_status_code = StatusCode::Ok;
    ObfmBumplessSpeedReceipt obfm_bumpless{};
    StatusCode obfm_bumpless_status_code = StatusCode::Ok;
    ObfmLagGuidanceCommit obfm_lag_commit{};
    bool obfm_lag_commit_applied = false;

    HabfmTerminalControlIntentObservation habfm_observation{};
    StatusCode habfm_observation_status_code = StatusCode::Ok;
    HabfmTerminalPreparedControlIntent habfm_prepared{};
    StaticHabfmG10Disposition habfm_g10_disposition =
        StaticHabfmG10Disposition::NotEvaluated;
    StatusCode habfm_g10_optional_status_code = StatusCode::Ok;
    HabfmAvoidPassOverlayReceipt habfm_avoid_overlay{};
    StatusCode habfm_avoid_optional_status_code = StatusCode::Ok;
    guidance::habfm::HabfmH09AltitudeStorageAdmission habfm_h09_storage{};
    runtime::CurrentCisV4EnergyProjectionReceipt habfm_h09_projection{};
    guidance::habfm::HabfmH09ResidualClimbAllocation habfm_h09_allocation{};
    StatusCode habfm_h09_optional_status_code = StatusCode::Ok;
    bool habfm_h09_projection_attempted = false;
    bool habfm_h09_active = false;
    BtTickResult habfm_selector_result{};
    StatusCode habfm_preparation_status_code = StatusCode::Ok;
    bool habfm_commit_applied = false;

    std::uint8_t candidate_count = 0U;
    std::uint8_t selected_candidate_count = 0U;
};

struct StaticDoctrinePreparedTransaction
{
    bool valid = false;
    StaticDoctrinePreparedOwner owner = StaticDoctrinePreparedOwner::None;
    ControlFrameIdentity frame_identity{};
    std::uint32_t writer_id = ControlIntentWriterNone;
};

// Allocation-free production provider used directly by the DLL and the
// standalone static runtime composition roots.
class StaticDoctrineCommandProvider final
    : public runtime::ITacticalCommandProvider
{
public:
    StaticDoctrineCommandProvider() noexcept;

    StaticDoctrineCommandProvider(
        const StaticDoctrineCommandProvider&) = delete;
    StaticDoctrineCommandProvider& operator=(
        const StaticDoctrineCommandProvider&) = delete;

    void Reset() noexcept override;
    void Build(
        const runtime::TacticalCommandBuildInput& input,
        ControlIntent& output,
        Status& status) noexcept override;
    void BuildWithProjection(
        const runtime::TacticalCommandBuildInput& input,
        runtime::ICurrentCisV4EnergyProjectionPort& projection_port,
        ControlIntent& output,
        Status& status) noexcept override;
    void CommitPrepared(
        const ControlFrameIdentity& frame_identity,
        std::uint32_t writer_id,
        Status& status) noexcept override;
    void AbortPrepared() noexcept override;

    void CopySnapshot(
        StaticDoctrineCommandProviderSnapshot& output) const noexcept;
    void CopySnapshot(
        StaticDoctrineCommandProviderSnapshot& output,
        Status& status) const noexcept;

private:
    void BuildInternal(
        const runtime::TacticalCommandBuildInput& input,
        runtime::ICurrentCisV4EnergyProjectionPort* projection_port,
        ControlIntent& output,
        Status& status) noexcept;
    void BuildObfm(
        const runtime::TacticalCommandBuildInput& input,
        const StaticDoctrineObfmG16G5bInput& obfm_input,
        bool include_current_effect_employ,
        ControlIntent& output,
        Status& status) noexcept;
    void BuildHabfm(
        const runtime::TacticalCommandBuildInput& input,
        runtime::ICurrentCisV4EnergyProjectionPort* projection_port,
        ControlIntent& output,
        Status& status) noexcept;
    void ClearPreparedTransaction() noexcept;
    void StagePreparedTransaction(
        StaticDoctrinePreparedOwner owner,
        const ControlFrameIdentity& frame_identity,
        std::uint32_t writer_id) noexcept;

    StaticSafetyGunStagedOwner safety_gun_owner_{};
    StaticImmediateGunResponseStagedOwner immediate_gun_response_owner_{};
    HabfmFrameEvidenceProvider frame_evidence_provider_{};
    StaticHabfmG10StagedOwner habfm_g10_owner_{};
    StaticDoctrineObfmG16G5bInputBuilder obfm_input_builder_{};
    StaticDoctrineObfmG16G5bOwner obfm_owner_{};
    StaticDoctrineObfmG16G5bAdapter<StaticDoctrineObfmG16G5bOwner>
        obfm_adapter_{};
    StaticHabfmG10Prepared habfm_g10_prepared_{};
    StaticDoctrinePreparedTransaction prepared_transaction_{};
    StaticDoctrineCommandProviderSnapshot snapshot_{};
};

static_assert(
    std::is_trivially_copyable<
        StaticDoctrineCommandProviderSnapshot>::value,
    "Static doctrine provider snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
        StaticDoctrinePreparedTransaction>::value,
    "Static doctrine prepared transaction must remain trivially copyable.");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
