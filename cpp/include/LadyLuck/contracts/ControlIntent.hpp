#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Kinematics.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{

template <typename T>
struct IntentOptionalValue
{
    bool has_value = false;
    T value{};
};

// Allocation-free route ownership. The route is explicit and must agree with
// the presence fields; downstream control must never infer a fallback route.
enum class ControlRouteKind : std::uint8_t
{
    Invalid = 0,
    AimPoint = 1,
    DirectBodyReferences = 2,
    DirectNedAcceleration = 3,
    DirectBankTurn = 4,
    DirectLoadVectorAcceleration = 5,
    SafetyRecovery = 6
};

// Stable numeric replacements for behavior_label. Unknown source labels are
// not silently collapsed into Tracking; their offline oracle adapter must add
// an explicit value here before they can enter the 60 Hz control path.
enum class DoctrineBehaviorId : std::uint16_t
{
    Invalid = 0,
    Tracking = 1,
    ControlZoneAcquireSafeVpp = 2,
    Stage06ShooterHoldRecapture = 3,
    AutoGcasRecovery = 4,
    TacticalSpeedFloorNeutralHandoff = 5,
    Lag = 6,
    EntrySetupLegacy = 7,
    SpacingArrestLegacy = 8,
    ObfmEntrySetupCurrentTurnEntryWindow = 9,
    ObfmSpacingArrestPathEnergyExchange = 10,
    ObfmSpacingArrestPostHitRminArrest = 11,
    ObfmSpacingArrestLevelRecovery = 12,
    ObfmSpacingArrestWezReacquire = 13,
    GunDefenseHorizontalBreak = 14,
    DbfmHardTurn = 15,
    HabfmMergeApproach = 16,
    HabfmEnergyFight = 17,
    HabfmOneCircle = 18,
    HabfmTwoCircle = 19,
    G16FinalAttackPassLoadVector = 20,
    G16BlowThroughSidePendingEnergyRetain = 21,
    G16BlowThroughMinimumChange3dLoadVector = 22,
    G5bDelayedClimbExtendMaxPower = 23,
    G5bDelayedClimbClimbUnload = 24,
    G16HighSharedClimbLoadVector = 25,
    Employ = 27,
    HabfmAvoidPassCross = 28,
    HabfmAvoidPassExtend = 29,
    ScissorsFirstReverse = 30,
    PrefireSnapshotPlaneChange = 31,
    G4HighGOverTheTop = 32,
    G4HighGUnderneath = 33,
    Obfm3dBarrelRollAttack = 34,
    ObfmScissorsGapBuild = 35,
    ObfmScissorsEgressAccel = 36,
    BrCounterHold = 37,
    RScissorsHold = 38,
    G10SecondUsePitchUp = 39,
    G10SecondUsePositiveLoadedWinding = 40,
    G10SecondUseDescendingLag = 42,
    ObfmApexDisplacement = 44,
    DbfmBreak = 45,
    DbfmAltitudeSeparated = 46,
    DbfmEscape = 47,
    ReservedBehavior48 = 48,
    OfficialGunTrackingJink = 49,
    OfficialGunSnapshotPlaneChange = 50,
    G4DivingSpiralDiveEntry = 51,
    G4DivingSpiralLoadedRoll = 52,
    G4WezShortestExit = 53
};

// Stable numeric replacements for mode_label. OvershootControl remains
// explicit because the frozen Route-5 boundary rejects that route by identity.
enum class DoctrineModeId : std::uint8_t
{
    Invalid = 0,
    ControlZone = 1,
    Shooter = 2,
    DefensiveDeny = 3,
    EnergyRecovery = 4,
    OvershootControl = 5,
    Safety = 6,
    GunThreat = 7,
    Committed = 8,
    Obfm = 9,
    Habfm = 10,
    Dbfm = 11
};

// One-based source-field ordinals from add/main d90e929b. Fields 32 and 33
// preserve their positions while replacing dynamic strings with fixed enums.
enum class ControlIntentFieldId : std::uint8_t
{
    AimPointM = 1,
    DesiredSpeedMps = 2,
    AimPointVelocityMps = 3,
    DesiredSpeedRateMps2 = 4,
    SpecificEnergyRateBiasM2ps3 = 5,
    PathInversionAllowed = 6,
    CaptureRangeDesM = 7,
    AimBlend = 8,
    LeadTimeTauSec = 9,
    LateralOffsetM = 10,
    VerticalOffsetM = 11,
    VerticalYoyoScale = 12,
    KRoll = 13,
    KPitch = 14,
    ThrottleBias = 15,
    TotalLoadFactorLimitG = 16,
    DirectPCmdRadps = 17,
    DirectNzCmdG = 18,
    DirectBetaCmdRad = 19,
    DirectAccelCmdMps2 = 20,
    DirectAccelerationNedMps2 = 21,
    DirectAccelerationRollRateReferenceRadps = 22,
    DirectAccelerationTrackingEnabled = 23,
    DirectAccelerationTrackingObservationOnly = 24,
    DirectAccelerationMagnitudeTrackingEnabled = 25,
    DirectAccelerationLoadedRollEnabled = 26,
    DirectAccelerationLoadComponentCompensationEnabled = 27,
    DirectAccelerationYawCoordinationEnabled = 28,
    DirectAccelerationRollPriorityYawEnabled = 29,
    DirectBankCmdRad = 30,
    DirectTurnRateCmdRadps = 31,
    BehaviorId = 32,
    ModeId = 33,
    DirectLoadVectorAccelerationNedMps2 = 34
};

constexpr std::size_t ControlIntentFieldCount = 34U;
constexpr std::uint32_t ControlIntentSchemaVersion = 1U;
constexpr std::uint32_t ControlIntentWriterNone = 0U;
// Auto-GCAS is the first visible Root command owner.  It preserves writer 1's
// deployed numeric identity without retaining the former generic hold path.
constexpr std::uint32_t ControlIntentWriterAutoGcasRecovery = 1U;
constexpr std::uint32_t ControlIntentWriterGunDefenseHorizontalBreak = 2U;
constexpr std::uint32_t ControlIntentWriterDbfmHardTurn = 3U;
constexpr std::uint32_t ControlIntentWriterHabfm = 4U;
// Writer 5 is the standalone production OBFM LAG leaf.
constexpr std::uint32_t ControlIntentWriterG16Committed = 6U;
// Writer 7 is G5b. Writer 8 is the production G16-P High owner.
constexpr std::uint32_t ControlIntentWriterG16HighPrevention = 8U;
constexpr std::uint32_t ControlIntentWriterTacticalSpeedFloor = 9U;
constexpr std::uint32_t ControlIntentWriterHabfmAvoidPass = 11U;
constexpr std::uint32_t ControlIntentWriterPrefireSnapshotPlaneChange = 12U;
constexpr std::uint32_t ControlIntentWriterG13FirstReverse = 13U;
constexpr std::uint32_t ControlIntentWriterG4HighGBarrel = 14U;
constexpr std::uint32_t ControlIntentWriterG3ChaseDownAimFloor = 15U;
constexpr std::uint32_t ControlIntentWriterG3Scissors = 16U;
constexpr std::uint32_t ControlIntentWriterG3CounterBarrel = 17U;
constexpr std::uint32_t ControlIntentWriterG3CounterRollingScissors = 18U;
constexpr std::uint32_t ControlIntentWriterG10SecondUse = 19U;
// Writer 20 retired with the incomplete FOLLOW/control-zone owner.  Keep the
// numeric hole so every remaining writer preserves its deployed identity.
constexpr std::uint32_t ControlIntentWriterObfmApexDisplacement = 21U;
constexpr std::uint32_t ControlIntentWriterDbfmBreak = 22U;
constexpr std::uint32_t ControlIntentWriterDbfmAltitudeSeparated = 23U;
constexpr std::uint32_t ControlIntentWriterDbfmEscape = 24U;
// Writer 25 retired with the redundant DBFM EXTEND owner. Preserve the
// numeric hole so all later deployed writer identities remain stable.
constexpr std::uint32_t ControlIntentWriterReserved25 = 25U;
constexpr std::uint32_t ControlIntentWriterOfficialGunTrackingJink = 26U;
constexpr std::uint32_t ControlIntentWriterOfficialGunSnapshotPlaneChange =
    27U;
constexpr std::uint32_t ControlIntentWriterObfmSpacing = 28U;
// Current-turn Entry Setup is a distinct typed Root producer.  It never
// aliases the legacy Entry placeholder or the higher-priority Spacing owner.
constexpr std::uint32_t ControlIntentWriterObfmEntrySetup = 29U;
// Dedicated G4 Diving Spiral owner.  It is mutually exclusive with writer 14
// through the visible PostRoot selector and never aliases High-G Barrel state.
constexpr std::uint32_t ControlIntentWriterG4DivingSpiral = 30U;

struct ControlIntentFieldOrderReceipt
{
    std::uint32_t schema_version = 0U;
    std::uint32_t field_count = 0U;
    std::array<ControlIntentFieldId, ControlIntentFieldCount> field_order{};
    bool valid = false;
};

// Raw tactical guidance intent. This requests aircraft motion and energy; it
// is not a p/q/r/Nz, control-surface, thrust, estimator, or aircraft-response
// receipt. Every stored numeric value, including absent optional storage, must
// remain finite so NaN can never be used as an availability sentinel.
struct ControlIntent
{
    // Transaction metadata, not an additional Python TacticalCommand field.
    ControlFrameIdentity frame_identity{};
    Vector3 aim_point_m{};                                                //  1
    double desired_speed_mps = 0.0;                                      //  2
    IntentOptionalValue<Vector3> aim_point_velocity_mps{};                //  3
    double desired_speed_rate_mps2 = 0.0;                                //  4
    double specific_energy_rate_bias_m2ps3 = 0.0;                        //  5
    IntentOptionalValue<bool> path_inversion_allowed{};                   //  6
    double capture_range_des_m = 650.0;                                  //  7
    double aim_blend = 0.0;                                              //  8
    double lead_time_tau_sec = 0.0;                                      //  9
    double lateral_offset_m = 0.0;                                       // 10
    double vertical_offset_m = 0.0;                                      // 11
    double vertical_yoyo_scale = 0.0;                                    // 12
    IntentOptionalValue<double> k_roll{};                                // 13
    IntentOptionalValue<double> k_pitch{};                               // 14
    IntentOptionalValue<double> throttle_bias{};                         // 15
    IntentOptionalValue<double> total_load_factor_limit_g{};             // 16
    IntentOptionalValue<double> direct_p_cmd_radps{};                    // 17
    IntentOptionalValue<double> direct_nz_cmd_g{};                       // 18
    IntentOptionalValue<double> direct_beta_cmd_rad{};                   // 19
    IntentOptionalValue<double> direct_accel_cmd_mps2{};                 // 20
    IntentOptionalValue<Vector3> direct_acceleration_ned_mps2{};         // 21
    IntentOptionalValue<double>
        direct_acceleration_roll_rate_reference_radps{};                 // 22
    bool direct_acceleration_tracking_enabled = false;                   // 23
    bool direct_acceleration_tracking_observation_only = false;          // 24
    bool direct_acceleration_magnitude_tracking_enabled = false;         // 25
    bool direct_acceleration_loaded_roll_enabled = false;                // 26
    bool direct_acceleration_load_component_compensation_enabled = false; // 27
    bool direct_acceleration_yaw_coordination_enabled = false;           // 28
    bool direct_acceleration_roll_priority_yaw_enabled = false;          // 29
    IntentOptionalValue<double> direct_bank_cmd_rad{};                   // 30
    IntentOptionalValue<double> direct_turn_rate_cmd_radps{};            // 31
    DoctrineBehaviorId behavior_id = DoctrineBehaviorId::Tracking;       // 32
    DoctrineModeId mode_id = DoctrineModeId::ControlZone;                // 33
    IntentOptionalValue<Vector3>
        direct_load_vector_acceleration_ned_mps2{};                      // 34

    ControlRouteKind route_kind = ControlRouteKind::AimPoint;
    std::uint32_t writer_id = ControlIntentWriterNone;

    void Clear() noexcept;
    void Validate(Status& status) const noexcept;
    void ClassifyG17EntryOrSpacing(
        bool& applicable,
        Status& status) const noexcept;
};

void CopyControlIntentFieldOrder(
    ControlIntentFieldOrderReceipt& output,
    Status& status) noexcept;

// Frozen x64 C++14 layout manifests. These constants describe storage only;
// they do not change route admission, writer ownership, or Validate().
using IntentOptionalBool = IntentOptionalValue<bool>;
using IntentOptionalDouble = IntentOptionalValue<double>;
using IntentOptionalVector3 = IntentOptionalValue<Vector3>;

struct IntentOptionalValueX64LayoutManifest
{
    static constexpr std::size_t BoolSize = 2U;
    static constexpr std::size_t BoolAlignment = 1U;
    static constexpr std::size_t BoolHasValueOffset = 0U;
    static constexpr std::size_t BoolValueOffset = 1U;
    static constexpr std::size_t DoubleSize = 16U;
    static constexpr std::size_t DoubleAlignment = 8U;
    static constexpr std::size_t DoubleHasValueOffset = 0U;
    static constexpr std::size_t DoubleValueOffset = 8U;
    static constexpr std::size_t Vector3Size = 32U;
    static constexpr std::size_t Vector3Alignment = 8U;
    static constexpr std::size_t Vector3HasValueOffset = 0U;
    static constexpr std::size_t Vector3ValueOffset = 8U;
};

struct ControlIntentFieldOrderReceiptX64LayoutManifest
{
    static constexpr std::size_t Size = 44U;
    static constexpr std::size_t Alignment = 4U;
    static constexpr std::size_t SchemaVersionOffset = 0U;
    static constexpr std::size_t FieldCountOffset = 4U;
    static constexpr std::size_t FieldOrderOffset = 8U;
    static constexpr std::size_t ValidOffset = 42U;
};

struct ControlIntentX64LayoutManifest
{
    static constexpr std::size_t Size = 432U;
    static constexpr std::size_t Alignment = 8U;
    static constexpr std::size_t FrameIdentityOffset = 0U;
    static constexpr std::size_t RouteKindOffset = 424U;
    static constexpr std::size_t WriterIdOffset = 428U;
};

// Indices are ControlIntentFieldId ordinal minus one. This preserves all 34
// Python-command field positions independently from transaction metadata.
constexpr std::array<std::size_t, ControlIntentFieldCount>
    ControlIntentFieldOffsetManifest{{
        32U,  // AimPointM
        56U,  // DesiredSpeedMps
        64U,  // AimPointVelocityMps
        96U,  // DesiredSpeedRateMps2
        104U, // SpecificEnergyRateBiasM2ps3
        112U, // PathInversionAllowed
        120U, // CaptureRangeDesM
        128U, // AimBlend
        136U, // LeadTimeTauSec
        144U, // LateralOffsetM
        152U, // VerticalOffsetM
        160U, // VerticalYoyoScale
        168U, // KRoll
        184U, // KPitch
        200U, // ThrottleBias
        216U, // TotalLoadFactorLimitG
        232U, // DirectPCmdRadps
        248U, // DirectNzCmdG
        264U, // DirectBetaCmdRad
        280U, // DirectAccelCmdMps2
        296U, // DirectAccelerationNedMps2
        328U, // DirectAccelerationRollRateReferenceRadps
        344U, // DirectAccelerationTrackingEnabled
        345U, // DirectAccelerationTrackingObservationOnly
        346U, // DirectAccelerationMagnitudeTrackingEnabled
        347U, // DirectAccelerationLoadedRollEnabled
        348U, // DirectAccelerationLoadComponentCompensationEnabled
        349U, // DirectAccelerationYawCoordinationEnabled
        350U, // DirectAccelerationRollPriorityYawEnabled
        352U, // DirectBankCmdRad
        368U, // DirectTurnRateCmdRadps
        384U, // BehaviorId
        386U, // ModeId
        392U  // DirectLoadVectorAccelerationNedMps2
    }};

static_assert(sizeof(void*) == 8U,
              "ControlIntent ABI manifest is frozen for x64 production");
static_assert(std::is_standard_layout<IntentOptionalBool>::value,
              "optional bool must remain standard-layout");
static_assert(std::is_trivially_copyable<IntentOptionalBool>::value,
              "optional bool must remain trivially copyable");
static_assert(sizeof(IntentOptionalBool) ==
                  IntentOptionalValueX64LayoutManifest::BoolSize,
              "optional bool size changed");
static_assert(alignof(IntentOptionalBool) ==
                  IntentOptionalValueX64LayoutManifest::BoolAlignment,
              "optional bool alignment changed");
static_assert(offsetof(IntentOptionalBool, has_value) ==
                  IntentOptionalValueX64LayoutManifest::BoolHasValueOffset,
              "optional bool has_value offset changed");
static_assert(offsetof(IntentOptionalBool, value) ==
                  IntentOptionalValueX64LayoutManifest::BoolValueOffset,
              "optional bool value offset changed");

static_assert(std::is_standard_layout<IntentOptionalDouble>::value,
              "optional double must remain standard-layout");
static_assert(std::is_trivially_copyable<IntentOptionalDouble>::value,
              "optional double must remain trivially copyable");
static_assert(sizeof(IntentOptionalDouble) ==
                  IntentOptionalValueX64LayoutManifest::DoubleSize,
              "optional double size changed");
static_assert(alignof(IntentOptionalDouble) ==
                  IntentOptionalValueX64LayoutManifest::DoubleAlignment,
              "optional double alignment changed");
static_assert(offsetof(IntentOptionalDouble, has_value) ==
                  IntentOptionalValueX64LayoutManifest::DoubleHasValueOffset,
              "optional double has_value offset changed");
static_assert(offsetof(IntentOptionalDouble, value) ==
                  IntentOptionalValueX64LayoutManifest::DoubleValueOffset,
              "optional double value offset changed");

static_assert(std::is_standard_layout<IntentOptionalVector3>::value,
              "optional Vector3 must remain standard-layout");
static_assert(std::is_trivially_copyable<IntentOptionalVector3>::value,
              "optional Vector3 must remain trivially copyable");
static_assert(sizeof(IntentOptionalVector3) ==
                  IntentOptionalValueX64LayoutManifest::Vector3Size,
              "optional Vector3 size changed");
static_assert(alignof(IntentOptionalVector3) ==
                  IntentOptionalValueX64LayoutManifest::Vector3Alignment,
              "optional Vector3 alignment changed");
static_assert(offsetof(IntentOptionalVector3, has_value) ==
                  IntentOptionalValueX64LayoutManifest::Vector3HasValueOffset,
              "optional Vector3 has_value offset changed");
static_assert(offsetof(IntentOptionalVector3, value) ==
                  IntentOptionalValueX64LayoutManifest::Vector3ValueOffset,
              "optional Vector3 value offset changed");

static_assert(
    std::is_standard_layout<ControlIntentFieldOrderReceipt>::value,
    "field-order receipt must remain standard-layout");
static_assert(
    std::is_trivially_copyable<ControlIntentFieldOrderReceipt>::value,
    "field-order receipt must remain trivially copyable");
static_assert(sizeof(ControlIntentFieldOrderReceipt) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::Size,
              "field-order receipt size changed");
static_assert(alignof(ControlIntentFieldOrderReceipt) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::Alignment,
              "field-order receipt alignment changed");
static_assert(offsetof(ControlIntentFieldOrderReceipt, schema_version) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::
                      SchemaVersionOffset,
              "field-order receipt schema_version offset changed");
static_assert(offsetof(ControlIntentFieldOrderReceipt, field_count) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::
                      FieldCountOffset,
              "field-order receipt field_count offset changed");
static_assert(offsetof(ControlIntentFieldOrderReceipt, field_order) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::
                      FieldOrderOffset,
              "field-order receipt field_order offset changed");
static_assert(offsetof(ControlIntentFieldOrderReceipt, valid) ==
                  ControlIntentFieldOrderReceiptX64LayoutManifest::ValidOffset,
              "field-order receipt valid offset changed");

static_assert(std::is_standard_layout<ControlIntent>::value,
              "ControlIntent must remain standard-layout");
static_assert(
    std::is_trivially_copyable<ControlIntent>::value,
    "ControlIntent must stay allocation-free and trivially copyable.");
static_assert(sizeof(ControlIntent) == ControlIntentX64LayoutManifest::Size,
              "ControlIntent x64 size changed");
static_assert(alignof(ControlIntent) ==
                  ControlIntentX64LayoutManifest::Alignment,
              "ControlIntent x64 alignment changed");
static_assert(offsetof(ControlIntent, frame_identity) ==
                  ControlIntentX64LayoutManifest::FrameIdentityOffset,
              "ControlIntent frame_identity offset changed");
static_assert(offsetof(ControlIntent, aim_point_m) ==
                  ControlIntentFieldOffsetManifest[0U],
              "ControlIntent field 1 offset changed");
static_assert(offsetof(ControlIntent, desired_speed_mps) ==
                  ControlIntentFieldOffsetManifest[1U],
              "ControlIntent field 2 offset changed");
static_assert(offsetof(ControlIntent, aim_point_velocity_mps) ==
                  ControlIntentFieldOffsetManifest[2U],
              "ControlIntent field 3 offset changed");
static_assert(offsetof(ControlIntent, desired_speed_rate_mps2) ==
                  ControlIntentFieldOffsetManifest[3U],
              "ControlIntent field 4 offset changed");
static_assert(offsetof(ControlIntent, specific_energy_rate_bias_m2ps3) ==
                  ControlIntentFieldOffsetManifest[4U],
              "ControlIntent field 5 offset changed");
static_assert(offsetof(ControlIntent, path_inversion_allowed) ==
                  ControlIntentFieldOffsetManifest[5U],
              "ControlIntent field 6 offset changed");
static_assert(offsetof(ControlIntent, capture_range_des_m) ==
                  ControlIntentFieldOffsetManifest[6U],
              "ControlIntent field 7 offset changed");
static_assert(offsetof(ControlIntent, aim_blend) ==
                  ControlIntentFieldOffsetManifest[7U],
              "ControlIntent field 8 offset changed");
static_assert(offsetof(ControlIntent, lead_time_tau_sec) ==
                  ControlIntentFieldOffsetManifest[8U],
              "ControlIntent field 9 offset changed");
static_assert(offsetof(ControlIntent, lateral_offset_m) ==
                  ControlIntentFieldOffsetManifest[9U],
              "ControlIntent field 10 offset changed");
static_assert(offsetof(ControlIntent, vertical_offset_m) ==
                  ControlIntentFieldOffsetManifest[10U],
              "ControlIntent field 11 offset changed");
static_assert(offsetof(ControlIntent, vertical_yoyo_scale) ==
                  ControlIntentFieldOffsetManifest[11U],
              "ControlIntent field 12 offset changed");
static_assert(offsetof(ControlIntent, k_roll) ==
                  ControlIntentFieldOffsetManifest[12U],
              "ControlIntent field 13 offset changed");
static_assert(offsetof(ControlIntent, k_pitch) ==
                  ControlIntentFieldOffsetManifest[13U],
              "ControlIntent field 14 offset changed");
static_assert(offsetof(ControlIntent, throttle_bias) ==
                  ControlIntentFieldOffsetManifest[14U],
              "ControlIntent field 15 offset changed");
static_assert(offsetof(ControlIntent, total_load_factor_limit_g) ==
                  ControlIntentFieldOffsetManifest[15U],
              "ControlIntent field 16 offset changed");
static_assert(offsetof(ControlIntent, direct_p_cmd_radps) ==
                  ControlIntentFieldOffsetManifest[16U],
              "ControlIntent field 17 offset changed");
static_assert(offsetof(ControlIntent, direct_nz_cmd_g) ==
                  ControlIntentFieldOffsetManifest[17U],
              "ControlIntent field 18 offset changed");
static_assert(offsetof(ControlIntent, direct_beta_cmd_rad) ==
                  ControlIntentFieldOffsetManifest[18U],
              "ControlIntent field 19 offset changed");
static_assert(offsetof(ControlIntent, direct_accel_cmd_mps2) ==
                  ControlIntentFieldOffsetManifest[19U],
              "ControlIntent field 20 offset changed");
static_assert(offsetof(ControlIntent, direct_acceleration_ned_mps2) ==
                  ControlIntentFieldOffsetManifest[20U],
              "ControlIntent field 21 offset changed");
static_assert(
    offsetof(ControlIntent, direct_acceleration_roll_rate_reference_radps) ==
        ControlIntentFieldOffsetManifest[21U],
    "ControlIntent field 22 offset changed");
static_assert(offsetof(ControlIntent, direct_acceleration_tracking_enabled) ==
                  ControlIntentFieldOffsetManifest[22U],
              "ControlIntent field 23 offset changed");
static_assert(
    offsetof(ControlIntent, direct_acceleration_tracking_observation_only) ==
        ControlIntentFieldOffsetManifest[23U],
    "ControlIntent field 24 offset changed");
static_assert(
    offsetof(ControlIntent, direct_acceleration_magnitude_tracking_enabled) ==
        ControlIntentFieldOffsetManifest[24U],
    "ControlIntent field 25 offset changed");
static_assert(offsetof(ControlIntent, direct_acceleration_loaded_roll_enabled) ==
                  ControlIntentFieldOffsetManifest[25U],
              "ControlIntent field 26 offset changed");
static_assert(
    offsetof(
        ControlIntent,
        direct_acceleration_load_component_compensation_enabled) ==
        ControlIntentFieldOffsetManifest[26U],
    "ControlIntent field 27 offset changed");
static_assert(
    offsetof(ControlIntent, direct_acceleration_yaw_coordination_enabled) ==
        ControlIntentFieldOffsetManifest[27U],
    "ControlIntent field 28 offset changed");
static_assert(
    offsetof(ControlIntent, direct_acceleration_roll_priority_yaw_enabled) ==
        ControlIntentFieldOffsetManifest[28U],
    "ControlIntent field 29 offset changed");
static_assert(offsetof(ControlIntent, direct_bank_cmd_rad) ==
                  ControlIntentFieldOffsetManifest[29U],
              "ControlIntent field 30 offset changed");
static_assert(offsetof(ControlIntent, direct_turn_rate_cmd_radps) ==
                  ControlIntentFieldOffsetManifest[30U],
              "ControlIntent field 31 offset changed");
static_assert(offsetof(ControlIntent, behavior_id) ==
                  ControlIntentFieldOffsetManifest[31U],
              "ControlIntent field 32 offset changed");
static_assert(offsetof(ControlIntent, mode_id) ==
                  ControlIntentFieldOffsetManifest[32U],
              "ControlIntent field 33 offset changed");
static_assert(
    offsetof(ControlIntent, direct_load_vector_acceleration_ned_mps2) ==
        ControlIntentFieldOffsetManifest[33U],
    "ControlIntent field 34 offset changed");
static_assert(offsetof(ControlIntent, route_kind) ==
                  ControlIntentX64LayoutManifest::RouteKindOffset,
              "ControlIntent route_kind offset changed");
static_assert(offsetof(ControlIntent, writer_id) ==
                  ControlIntentX64LayoutManifest::WriterIdOffset,
              "ControlIntent writer_id offset changed");
static_assert(
    std::is_nothrow_copy_assignable<ControlIntent>::value,
    "ControlIntent copies must stay noexcept in the 60 Hz path.");

} // namespace LadyLuck
