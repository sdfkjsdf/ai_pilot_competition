#include "LadyLuck/contracts/ControlIntent.hpp"

#include "LadyLuck/common/Constants.hpp"

#include <cmath>

namespace
{

bool FiniteVector(const LadyLuck::Vector3& value) noexcept
{
    return std::isfinite(value[0])
        && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool FiniteOptionalScalar(
    const LadyLuck::IntentOptionalValue<double>& value) noexcept
{
    // Absence is semantic only. Its storage is also required to be finite so
    // it cannot become a hidden NaN availability channel.
    return std::isfinite(value.value);
}

bool FiniteOptionalVector(
    const LadyLuck::IntentOptionalValue<LadyLuck::Vector3>& value) noexcept
{
    return FiniteVector(value.value);
}

bool ValidRouteKind(const LadyLuck::ControlRouteKind value) noexcept
{
    using LadyLuck::ControlRouteKind;
    switch (value)
    {
    case ControlRouteKind::AimPoint:
    case ControlRouteKind::DirectBodyReferences:
    case ControlRouteKind::DirectNedAcceleration:
    case ControlRouteKind::DirectBankTurn:
    case ControlRouteKind::DirectLoadVectorAcceleration:
    case ControlRouteKind::SafetyRecovery:
        return true;
    case ControlRouteKind::Invalid:
    default:
        return false;
    }
}

bool ValidBehaviorId(const LadyLuck::DoctrineBehaviorId value) noexcept
{
    using LadyLuck::DoctrineBehaviorId;
    switch (value)
    {
    case DoctrineBehaviorId::Tracking:
    case DoctrineBehaviorId::ControlZoneAcquireSafeVpp:
    case DoctrineBehaviorId::Stage06ShooterHoldRecapture:
    case DoctrineBehaviorId::AutoGcasRecovery:
    case DoctrineBehaviorId::TacticalSpeedFloorNeutralHandoff:
    case DoctrineBehaviorId::Lag:
    case DoctrineBehaviorId::EntrySetupLegacy:
    case DoctrineBehaviorId::SpacingArrestLegacy:
    case DoctrineBehaviorId::ObfmEntrySetupCurrentTurnEntryWindow:
    case DoctrineBehaviorId::ObfmSpacingArrestPathEnergyExchange:
    case DoctrineBehaviorId::ObfmSpacingArrestPostHitRminArrest:
    case DoctrineBehaviorId::ObfmSpacingArrestLevelRecovery:
    case DoctrineBehaviorId::ObfmSpacingArrestWezReacquire:
    case DoctrineBehaviorId::GunDefenseHorizontalBreak:
    case DoctrineBehaviorId::DbfmHardTurn:
    case DoctrineBehaviorId::HabfmMergeApproach:
    case DoctrineBehaviorId::HabfmEnergyFight:
    case DoctrineBehaviorId::HabfmOneCircle:
    case DoctrineBehaviorId::HabfmTwoCircle:
    case DoctrineBehaviorId::G16FinalAttackPassLoadVector:
    case DoctrineBehaviorId::G16BlowThroughSidePendingEnergyRetain:
    case DoctrineBehaviorId::G16BlowThroughMinimumChange3dLoadVector:
    case DoctrineBehaviorId::G5bDelayedClimbExtendMaxPower:
    case DoctrineBehaviorId::G5bDelayedClimbClimbUnload:
    case DoctrineBehaviorId::G16HighSharedClimbLoadVector:
    case DoctrineBehaviorId::Employ:
    case DoctrineBehaviorId::HabfmAvoidPassCross:
    case DoctrineBehaviorId::HabfmAvoidPassExtend:
    case DoctrineBehaviorId::ScissorsFirstReverse:
    case DoctrineBehaviorId::PrefireSnapshotPlaneChange:
    case DoctrineBehaviorId::G4HighGOverTheTop:
    case DoctrineBehaviorId::G4HighGUnderneath:
    case DoctrineBehaviorId::G4DivingSpiralDiveEntry:
    case DoctrineBehaviorId::G4DivingSpiralLoadedRoll:
    case DoctrineBehaviorId::G4WezShortestExit:
    case DoctrineBehaviorId::Obfm3dBarrelRollAttack:
    case DoctrineBehaviorId::ObfmScissorsGapBuild:
    case DoctrineBehaviorId::ObfmScissorsEgressAccel:
    case DoctrineBehaviorId::BrCounterHold:
    case DoctrineBehaviorId::RScissorsHold:
    case DoctrineBehaviorId::G10SecondUsePitchUp:
    case DoctrineBehaviorId::G10SecondUsePositiveLoadedWinding:
    case DoctrineBehaviorId::G10SecondUseDescendingLag:
    case DoctrineBehaviorId::ObfmApexDisplacement:
    case DoctrineBehaviorId::DbfmBreak:
    case DoctrineBehaviorId::DbfmAltitudeSeparated:
    case DoctrineBehaviorId::DbfmEscape:
    case DoctrineBehaviorId::OfficialGunTrackingJink:
    case DoctrineBehaviorId::OfficialGunSnapshotPlaneChange:
        return true;
    case DoctrineBehaviorId::Invalid:
    default:
        return false;
    }
}

bool ValidModeId(const LadyLuck::DoctrineModeId value) noexcept
{
    using LadyLuck::DoctrineModeId;
    switch (value)
    {
    case DoctrineModeId::ControlZone:
    case DoctrineModeId::Shooter:
    case DoctrineModeId::DefensiveDeny:
    case DoctrineModeId::EnergyRecovery:
    case DoctrineModeId::OvershootControl:
    case DoctrineModeId::Safety:
    case DoctrineModeId::GunThreat:
    case DoctrineModeId::Committed:
    case DoctrineModeId::Obfm:
    case DoctrineModeId::Habfm:
    case DoctrineModeId::Dbfm:
        return true;
    case DoctrineModeId::Invalid:
    default:
        return false;
    }
}

bool CloseToPythonReference(
    const double left,
    const double right) noexcept
{
    if (left == right)
    {
        return true;
    }
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    return std::fabs(left - right) <= 1.0e-12 * std::fabs(right);
}

bool RoutePresenceConsistent(const LadyLuck::ControlIntent& intent) noexcept
{
    const bool direct_body = intent.direct_p_cmd_radps.has_value
        || intent.direct_nz_cmd_g.has_value
        || intent.direct_beta_cmd_rad.has_value;
    const bool direct_ned = intent.direct_acceleration_ned_mps2.has_value;
    const bool bank_turn = intent.direct_bank_cmd_rad.has_value
        || intent.direct_turn_rate_cmd_radps.has_value;
    const bool direct_load =
        intent.direct_load_vector_acceleration_ned_mps2.has_value;

    using LadyLuck::ControlRouteKind;
    switch (intent.route_kind)
    {
    case ControlRouteKind::AimPoint:
        return !direct_body && !direct_ned && !bank_turn && !direct_load;
    case ControlRouteKind::DirectBodyReferences:
        // A coexisting H09 bank pair has source priority over legacy direct
        // body fields, so it must be classified as DirectBankTurn below.
        return direct_body && !direct_ned && !bank_turn && !direct_load;
    case ControlRouteKind::DirectNedAcceleration:
        // d90 structurally permits beta and longitudinal direct_accel beside
        // direct-NED. Maneuver-specific adapters may impose a stricter scope.
        return direct_ned
            && !intent.direct_p_cmd_radps.has_value
            && !intent.direct_nz_cmd_g.has_value
            && !bank_turn
            && !direct_load;
    case ControlRouteKind::DirectBankTurn:
        return intent.direct_bank_cmd_rad.has_value
            && intent.direct_turn_rate_cmd_radps.has_value
            && !direct_ned
            && !direct_load;
    case ControlRouteKind::DirectLoadVectorAcceleration:
        return direct_load && !direct_body && !direct_ned && !bank_turn;
    case ControlRouteKind::SafetyRecovery:
        return !direct_body && !direct_ned && !bank_turn && !direct_load;
    case ControlRouteKind::Invalid:
    default:
        return false;
    }
}

void Failure(
    LadyLuck::Status& status,
    const LadyLuck::StatusCode code) noexcept
{
    status.code = code;
}

} // namespace

namespace LadyLuck
{

void ControlIntent::Clear() noexcept
{
    *this = ControlIntent{};
}

void ControlIntent::Validate(Status& status) const noexcept
{
    status = Status{};

    // The no-NaN contract covers required values and optional backing storage.
    if (!IsValidControlFrameIdentity(frame_identity))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
    if (!FiniteVector(aim_point_m)
        || !std::isfinite(desired_speed_mps)
        || !FiniteOptionalVector(aim_point_velocity_mps)
        || !std::isfinite(desired_speed_rate_mps2)
        || !std::isfinite(specific_energy_rate_bias_m2ps3)
        || !std::isfinite(capture_range_des_m)
        || !std::isfinite(aim_blend)
        || !std::isfinite(lead_time_tau_sec)
        || !std::isfinite(lateral_offset_m)
        || !std::isfinite(vertical_offset_m)
        || !std::isfinite(vertical_yoyo_scale)
        || !FiniteOptionalScalar(k_roll)
        || !FiniteOptionalScalar(k_pitch)
        || !FiniteOptionalScalar(throttle_bias)
        || !FiniteOptionalScalar(total_load_factor_limit_g)
        || !FiniteOptionalScalar(direct_p_cmd_radps)
        || !FiniteOptionalScalar(direct_nz_cmd_g)
        || !FiniteOptionalScalar(direct_beta_cmd_rad)
        || !FiniteOptionalScalar(direct_accel_cmd_mps2)
        || !FiniteOptionalVector(direct_acceleration_ned_mps2)
        || !FiniteOptionalScalar(
            direct_acceleration_roll_rate_reference_radps)
        || !FiniteOptionalScalar(direct_bank_cmd_rad)
        || !FiniteOptionalScalar(direct_turn_rate_cmd_radps)
        || !FiniteOptionalVector(
            direct_load_vector_acceleration_ned_mps2))
    {
        Failure(status, StatusCode::NonFiniteInput);
        return;
    }

    if (!ValidRouteKind(route_kind)
        || !ValidBehaviorId(behavior_id)
        || !ValidModeId(mode_id)
        || writer_id == ControlIntentWriterNone)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    // Exact d90 structural ownership for direct-NED acceleration.
    if (direct_acceleration_ned_mps2.has_value
        && (direct_p_cmd_radps.has_value
            || direct_nz_cmd_g.has_value
            || direct_bank_cmd_rad.has_value
            || direct_load_vector_acceleration_ned_mps2.has_value))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    if (direct_acceleration_roll_rate_reference_radps.has_value
        && !direct_acceleration_ned_mps2.has_value)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    if (direct_acceleration_tracking_enabled
        && !direct_acceleration_ned_mps2.has_value)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
    if (direct_acceleration_tracking_observation_only
        && !(direct_acceleration_tracking_enabled
            && direct_acceleration_ned_mps2.has_value))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
    if (direct_acceleration_magnitude_tracking_enabled
        && !(direct_acceleration_tracking_enabled
            && direct_acceleration_ned_mps2.has_value))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
    if ((direct_acceleration_loaded_roll_enabled
            || direct_acceleration_load_component_compensation_enabled
            || direct_acceleration_yaw_coordination_enabled
            || direct_acceleration_roll_priority_yaw_enabled)
        && !direct_acceleration_ned_mps2.has_value)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
    if (direct_acceleration_yaw_coordination_enabled
        && direct_acceleration_roll_priority_yaw_enabled)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    if (total_load_factor_limit_g.has_value
        && total_load_factor_limit_g.value <= 0.0)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    constexpr double HalfPi = 0.5 * constants::Pi;
    if (direct_bank_cmd_rad.has_value)
    {
        const double bank = direct_bank_cmd_rad.value;
        if (std::fabs(bank) >= HalfPi
            || !direct_turn_rate_cmd_radps.has_value
            || direct_turn_rate_cmd_radps.value <= 0.0
            || !total_load_factor_limit_g.has_value
            || desired_speed_mps <= 0.0)
        {
            Failure(status, StatusCode::InvalidArgument);
            return;
        }

        const double lateral_from_bank = std::tan(std::fabs(bank));
        const double lateral_from_rate = desired_speed_mps
            * direct_turn_rate_cmd_radps.value
            / constants::StandardGravityMps2;
        const double load_from_bank = 1.0 / std::cos(std::fabs(bank));
        if (!CloseToPythonReference(lateral_from_bank, lateral_from_rate)
            || !CloseToPythonReference(
                load_from_bank,
                total_load_factor_limit_g.value))
        {
            Failure(status, StatusCode::InvalidConfiguration);
            return;
        }
    }
    else if (direct_turn_rate_cmd_radps.has_value)
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    if (direct_load_vector_acceleration_ned_mps2.has_value
        && (direct_p_cmd_radps.has_value
            || direct_nz_cmd_g.has_value
            || direct_beta_cmd_rad.has_value
            || direct_bank_cmd_rad.has_value
            || direct_acceleration_ned_mps2.has_value
            || direct_acceleration_roll_rate_reference_radps.has_value))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    if (!RoutePresenceConsistent(*this))
    {
        Failure(status, StatusCode::InvalidConfiguration);
    }
}

void ControlIntent::ClassifyG17EntryOrSpacing(
    bool& applicable,
    Status& status) const noexcept
{
    applicable = false;
    status = Status{};
    if (!ValidBehaviorId(behavior_id))
    {
        Failure(status, StatusCode::InvalidArgument);
        return;
    }

    switch (behavior_id)
    {
    case DoctrineBehaviorId::EntrySetupLegacy:
    case DoctrineBehaviorId::SpacingArrestLegacy:
    case DoctrineBehaviorId::ObfmEntrySetupCurrentTurnEntryWindow:
        applicable = true;
        return;
    case DoctrineBehaviorId::Tracking:
    case DoctrineBehaviorId::ControlZoneAcquireSafeVpp:
    case DoctrineBehaviorId::Stage06ShooterHoldRecapture:
    case DoctrineBehaviorId::AutoGcasRecovery:
    case DoctrineBehaviorId::TacticalSpeedFloorNeutralHandoff:
    case DoctrineBehaviorId::Lag:
    case DoctrineBehaviorId::GunDefenseHorizontalBreak:
    case DoctrineBehaviorId::DbfmHardTurn:
    case DoctrineBehaviorId::HabfmMergeApproach:
    case DoctrineBehaviorId::HabfmEnergyFight:
    case DoctrineBehaviorId::HabfmOneCircle:
    case DoctrineBehaviorId::HabfmTwoCircle:
    case DoctrineBehaviorId::G16FinalAttackPassLoadVector:
    case DoctrineBehaviorId::G16BlowThroughSidePendingEnergyRetain:
    case DoctrineBehaviorId::G16BlowThroughMinimumChange3dLoadVector:
    case DoctrineBehaviorId::G5bDelayedClimbExtendMaxPower:
    case DoctrineBehaviorId::G5bDelayedClimbClimbUnload:
    case DoctrineBehaviorId::G16HighSharedClimbLoadVector:
    case DoctrineBehaviorId::Employ:
    case DoctrineBehaviorId::HabfmAvoidPassCross:
    case DoctrineBehaviorId::HabfmAvoidPassExtend:
    case DoctrineBehaviorId::ScissorsFirstReverse:
    case DoctrineBehaviorId::PrefireSnapshotPlaneChange:
    case DoctrineBehaviorId::G4HighGOverTheTop:
    case DoctrineBehaviorId::G4HighGUnderneath:
    case DoctrineBehaviorId::G4DivingSpiralDiveEntry:
    case DoctrineBehaviorId::G4DivingSpiralLoadedRoll:
    case DoctrineBehaviorId::G4WezShortestExit:
    case DoctrineBehaviorId::Obfm3dBarrelRollAttack:
    case DoctrineBehaviorId::ObfmScissorsGapBuild:
    case DoctrineBehaviorId::ObfmScissorsEgressAccel:
    case DoctrineBehaviorId::BrCounterHold:
    case DoctrineBehaviorId::RScissorsHold:
    case DoctrineBehaviorId::G10SecondUsePitchUp:
    case DoctrineBehaviorId::G10SecondUsePositiveLoadedWinding:
    case DoctrineBehaviorId::G10SecondUseDescendingLag:
    case DoctrineBehaviorId::ObfmApexDisplacement:
    case DoctrineBehaviorId::DbfmBreak:
    case DoctrineBehaviorId::DbfmAltitudeSeparated:
    case DoctrineBehaviorId::DbfmEscape:
    case DoctrineBehaviorId::OfficialGunTrackingJink:
    case DoctrineBehaviorId::OfficialGunSnapshotPlaneChange:
    case DoctrineBehaviorId::ObfmSpacingArrestPathEnergyExchange:
    case DoctrineBehaviorId::ObfmSpacingArrestPostHitRminArrest:
    case DoctrineBehaviorId::ObfmSpacingArrestLevelRecovery:
    case DoctrineBehaviorId::ObfmSpacingArrestWezReacquire:
        return;
    case DoctrineBehaviorId::Invalid:
    default:
        Failure(status, StatusCode::InvalidArgument);
        return;
    }
}

void CopyControlIntentFieldOrder(
    ControlIntentFieldOrderReceipt& output,
    Status& status) noexcept
{
    output = ControlIntentFieldOrderReceipt{};
    output.schema_version = ControlIntentSchemaVersion;
    output.field_count = static_cast<std::uint32_t>(ControlIntentFieldCount);
    output.field_order = {{
        ControlIntentFieldId::AimPointM,
        ControlIntentFieldId::DesiredSpeedMps,
        ControlIntentFieldId::AimPointVelocityMps,
        ControlIntentFieldId::DesiredSpeedRateMps2,
        ControlIntentFieldId::SpecificEnergyRateBiasM2ps3,
        ControlIntentFieldId::PathInversionAllowed,
        ControlIntentFieldId::CaptureRangeDesM,
        ControlIntentFieldId::AimBlend,
        ControlIntentFieldId::LeadTimeTauSec,
        ControlIntentFieldId::LateralOffsetM,
        ControlIntentFieldId::VerticalOffsetM,
        ControlIntentFieldId::VerticalYoyoScale,
        ControlIntentFieldId::KRoll,
        ControlIntentFieldId::KPitch,
        ControlIntentFieldId::ThrottleBias,
        ControlIntentFieldId::TotalLoadFactorLimitG,
        ControlIntentFieldId::DirectPCmdRadps,
        ControlIntentFieldId::DirectNzCmdG,
        ControlIntentFieldId::DirectBetaCmdRad,
        ControlIntentFieldId::DirectAccelCmdMps2,
        ControlIntentFieldId::DirectAccelerationNedMps2,
        ControlIntentFieldId::DirectAccelerationRollRateReferenceRadps,
        ControlIntentFieldId::DirectAccelerationTrackingEnabled,
        ControlIntentFieldId::DirectAccelerationTrackingObservationOnly,
        ControlIntentFieldId::DirectAccelerationMagnitudeTrackingEnabled,
        ControlIntentFieldId::DirectAccelerationLoadedRollEnabled,
        ControlIntentFieldId::DirectAccelerationLoadComponentCompensationEnabled,
        ControlIntentFieldId::DirectAccelerationYawCoordinationEnabled,
        ControlIntentFieldId::DirectAccelerationRollPriorityYawEnabled,
        ControlIntentFieldId::DirectBankCmdRad,
        ControlIntentFieldId::DirectTurnRateCmdRadps,
        ControlIntentFieldId::BehaviorId,
        ControlIntentFieldId::ModeId,
        ControlIntentFieldId::DirectLoadVectorAccelerationNedMps2}};
    output.valid = true;
    status = Status{};
}

} // namespace LadyLuck
