#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "LadyLuck/contracts/ScalarTypes.hpp"

constexpr std::uint32_t AIPILOT_ABI_VERSION_V1 = 1U;
constexpr LadyLuck::Float64 AIPILOT_NOMINAL_DT_S_V1 = 1.0 / 60.0;

#pragma pack(push, 1)

typedef struct _ControlValue
{
    LadyLuck::Float32 RollCMD;
    LadyLuck::Float32 PitchCMD;
    LadyLuck::Float32 RudderCMD;
    LadyLuck::Float32 Throttle;
} ControlValue, *pControlValue;

// Complete kinematic observation available from the competition PlaneInfo.
// This is not a full aircraft dynamics state: p/q/r, Nz, actuator feedback,
// mass, and aerodynamic truth are not supplied. Position is N/E/Up in metres,
// attitude is roll/pitch/yaw in degrees, and velocity is body u/v/w in m/s.
typedef struct PlaneKinematicObservationV1
{
    std::uint32_t abi_version;
    std::uint32_t struct_size;
    std::uint64_t frame_index;
    std::int32_t plane_id;
    std::int32_t force_side;
    LadyLuck::Float32 position_n_m;
    LadyLuck::Float32 position_e_m;
    LadyLuck::Float32 position_up_m;
    LadyLuck::Float32 roll_deg;
    LadyLuck::Float32 pitch_deg;
    LadyLuck::Float32 yaw_deg;
    LadyLuck::Float32 body_u_mps;
    LadyLuck::Float32 body_v_mps;
    LadyLuck::Float32 body_w_mps;
} PlaneKinematicObservationV1;

typedef struct KinematicObservationInputV1
{
    std::uint32_t abi_version;
    std::uint32_t struct_size;
    std::uint64_t command_frame_index;
    LadyLuck::Float64 command_time_s;
    LadyLuck::Float64 nominal_dt_s;
    std::int32_t context_own_plane_id;
    std::int32_t context_target_plane_id;
    PlaneKinematicObservationV1 ownship;
    PlaneKinematicObservationV1 target;
} KinematicObservationInputV1;

typedef struct DerivedPlaneStateV1
{
    std::uint64_t frame_index;
    std::int32_t plane_id;
    std::int32_t force_side;
    LadyLuck::Float64 position_ned_m[3];
    LadyLuck::Float64 rpy_rad[3];
    LadyLuck::Float64 velocity_body_mps[3];
    LadyLuck::Float64 speed_mps;
    LadyLuck::Float64 alpha_rad;
    LadyLuck::Float64 beta_rad;
} DerivedPlaneStateV1;

typedef struct FrameContractDiagnosticsV1
{
    std::uint32_t abi_version;
    std::uint32_t struct_size;
    std::uint64_t command_frame_index;
    LadyLuck::Float64 command_time_s;
    LadyLuck::Float64 nominal_dt_s;
    std::int32_t context_own_plane_id;
    std::int32_t context_target_plane_id;
    DerivedPlaneStateV1 ownship;
    DerivedPlaneStateV1 target;
    std::uint32_t own_measurement_valid;
    std::uint32_t target_observation_valid;
    std::uint32_t same_index_input_candidate;
    std::uint32_t estimator_transaction_committed;
    std::uint32_t same_index_geometry_valid;
    std::uint64_t gap_count;
    std::int32_t gap_policy;
    std::uint64_t episode_epoch;
    LadyLuck::Float64 measurement_time_s;
    LadyLuck::Float64 sample_dt_s;
    std::uint32_t writer_count;
    std::int32_t writer_id;
} FrameContractDiagnosticsV1;

enum AIPilotControlCommandOutcomeV1 : std::int32_t
{
    AIP_CONTROL_OUTCOME_CURRENT_BASE = 0,
    AIP_CONTROL_OUTCOME_TACTICAL = 1,
    AIP_CONTROL_OUTCOME_SAFETY = 2,
    AIP_CONTROL_OUTCOME_INPUT_REJECTED = 3
};

enum AIPilotCurrentBaseOwnershipEventV1 : std::int32_t
{
    AIP_CURRENT_BASE_EVENT_NONE = 0,
    AIP_CURRENT_BASE_EVENT_STARTED = 1,
    AIP_CURRENT_BASE_EVENT_ENDED = 2
};

// Additive diagnostics record. It does not alter the frozen observation or
// control layout and makes doctrine-base ownership visible without inventing
// a command or overloading the runtime status.
typedef struct ControlCommandOutcomeDiagnosticsV1
{
    std::uint32_t abi_version;
    std::uint32_t struct_size;
    std::uint64_t command_frame_index;
    std::int32_t outcome;
    std::int32_t current_base_event;
    std::int32_t runtime_status;
    std::uint32_t control_authorized;
    std::uint64_t current_base_started_count;
    std::uint64_t current_base_ended_count;
    std::uint32_t current_base_active;
    std::uint32_t reserved;
} ControlCommandOutcomeDiagnosticsV1;

#pragma pack(pop)

// Transport-level pre-start samples have no usable relative-flight geometry.
// Keep one exact predicate shared by the ABI gate and ControlCore preflight so
// an own-only zero, target-only zero, or coincident pair cannot be interpreted
// differently on the two sides of the estimator/BT boundary.
inline bool AIPilotExactZeroBodySpeedV1(
    const PlaneKinematicObservationV1& plane) noexcept
{
    return plane.body_u_mps == 0.0F
        && plane.body_v_mps == 0.0F
        && plane.body_w_mps == 0.0F;
}

inline bool AIPilotPreStartCommandNeutralV1(
    const KinematicObservationInputV1& input) noexcept
{
    const bool coincident_position =
        input.ownship.position_n_m == input.target.position_n_m
        && input.ownship.position_e_m == input.target.position_e_m
        && input.ownship.position_up_m == input.target.position_up_m;
    return AIPilotExactZeroBodySpeedV1(input.ownship)
        || AIPilotExactZeroBodySpeedV1(input.target)
        || coincident_position;
}

enum AIPilotRuntimeStatusV1 : std::int32_t
{
    AIP_RUNTIME_NOT_RUN = 0,
    AIP_RUNTIME_INPUT_ACCEPTED_PORT_INCOMPLETE = 1,
    AIP_RUNTIME_CONTROL_PRIMARY = 3,
    AIP_RUNTIME_CONTROL_CONTAINMENT = 4,
    AIP_RUNTIME_CONTROL_FAIL_CLOSED = 5,
    AIP_RUNTIME_PRESTART_COMMAND_NEUTRAL = 6,
    AIP_RUNTIME_NULL_INPUT = -1,
    AIP_RUNTIME_INVALID_ABI_VERSION = -2,
    AIP_RUNTIME_INVALID_STRUCT_SIZE = -3,
    AIP_RUNTIME_NONFINITE_INPUT = -4,
    AIP_RUNTIME_INVALID_TIME = -5,
    AIP_RUNTIME_INVALID_OUTPUT_BUFFER = -6,
    AIP_RUNTIME_BT_NOT_FOUND = -7,
    AIP_RUNTIME_BT_CONTRACT_FAILED = -8,
    AIP_RUNTIME_INTERNAL_CONTRACT_FAULT = -9
};

enum AIPilotTreeStatusV1 : std::int32_t
{
    AIP_TREE_CREATED = 1,
    AIP_TREE_ALREADY_EXISTS = 2,
    AIP_TREE_INITIALIZATION_FAILED = -1
};

enum AIPilotObservationGapPolicyV1 : std::int32_t
{
    AIP_GAP_NOT_EVALUATED = 0,
    AIP_GAP_FIRST_SEED = 1,
    AIP_GAP_NORMAL = 2,
    AIP_GAP_OBSERVER_REPRIME = 3,
    AIP_GAP_RESYNC = 4
};

// Frozen exported CopyFrameContractDiagnostics buffer layout. The C ABI is
// packed to one-byte alignment; the manifest is named explicitly so native
// and external frozen parsers can validate the same field map.
struct FrameContractDiagnosticsV1X64LayoutManifest
{
    static constexpr std::size_t Size = 328U;
    static constexpr std::size_t Alignment = 1U;
    static constexpr std::size_t AbiVersionOffset = 0U;
    static constexpr std::size_t StructSizeOffset = 4U;
    static constexpr std::size_t CommandFrameIndexOffset = 8U;
    static constexpr std::size_t CommandTimeOffset = 16U;
    static constexpr std::size_t NominalDtOffset = 24U;
    static constexpr std::size_t ContextOwnPlaneIdOffset = 32U;
    static constexpr std::size_t ContextTargetPlaneIdOffset = 36U;
    static constexpr std::size_t OwnshipOffset = 40U;
    static constexpr std::size_t TargetOffset = 152U;
    static constexpr std::size_t OwnMeasurementValidOffset = 264U;
    static constexpr std::size_t TargetObservationValidOffset = 268U;
    static constexpr std::size_t SameIndexInputCandidateOffset = 272U;
    static constexpr std::size_t EstimatorTransactionCommittedOffset = 276U;
    static constexpr std::size_t SameIndexGeometryValidOffset = 280U;
    static constexpr std::size_t GapCountOffset = 284U;
    static constexpr std::size_t GapPolicyOffset = 292U;
    static constexpr std::size_t EpisodeEpochOffset = 296U;
    static constexpr std::size_t MeasurementTimeOffset = 304U;
    static constexpr std::size_t SampleDtOffset = 312U;
    static constexpr std::size_t WriterCountOffset = 320U;
    static constexpr std::size_t WriterIdOffset = 324U;
};

struct ControlCommandOutcomeDiagnosticsV1X64LayoutManifest
{
    static constexpr std::size_t Size = 56U;
    static constexpr std::size_t Alignment = 1U;
    static constexpr std::size_t AbiVersionOffset = 0U;
    static constexpr std::size_t StructSizeOffset = 4U;
    static constexpr std::size_t CommandFrameIndexOffset = 8U;
    static constexpr std::size_t OutcomeOffset = 16U;
    static constexpr std::size_t CurrentBaseEventOffset = 20U;
    static constexpr std::size_t RuntimeStatusOffset = 24U;
    static constexpr std::size_t ControlAuthorizedOffset = 28U;
    static constexpr std::size_t CurrentBaseStartedCountOffset = 32U;
    static constexpr std::size_t CurrentBaseEndedCountOffset = 40U;
    static constexpr std::size_t CurrentBaseActiveOffset = 48U;
    static constexpr std::size_t ReservedOffset = 52U;
};

static_assert(std::is_standard_layout<ControlValue>::value, "ControlValue must be standard-layout.");
static_assert(std::is_trivially_copyable<ControlValue>::value, "ControlValue must be trivially copyable.");
static_assert(alignof(ControlValue) == 1U, "ControlValue ABI alignment changed.");
static_assert(sizeof(ControlValue) == 16U, "ControlValue ABI changed.");
static_assert(offsetof(ControlValue, RollCMD) == 0U, "ControlValue offsets changed.");
static_assert(offsetof(ControlValue, PitchCMD) == 4U, "ControlValue offsets changed.");
static_assert(offsetof(ControlValue, RudderCMD) == 8U, "ControlValue offsets changed.");
static_assert(offsetof(ControlValue, Throttle) == 12U, "ControlValue offsets changed.");

static_assert(std::is_standard_layout<PlaneKinematicObservationV1>::value, "Plane observation must be standard-layout.");
static_assert(std::is_trivially_copyable<PlaneKinematicObservationV1>::value, "Plane observation must be trivially copyable.");
static_assert(alignof(PlaneKinematicObservationV1) == 1U, "Plane observation ABI alignment changed.");
static_assert(sizeof(PlaneKinematicObservationV1) == 60U, "Plane kinematic observation ABI changed.");
static_assert(offsetof(PlaneKinematicObservationV1, abi_version) == 0U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, struct_size) == 4U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, frame_index) == 8U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, plane_id) == 16U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, force_side) == 20U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, position_n_m) == 24U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, position_e_m) == 28U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, position_up_m) == 32U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, roll_deg) == 36U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, pitch_deg) == 40U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, yaw_deg) == 44U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, body_u_mps) == 48U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, body_v_mps) == 52U, "Plane observation offsets changed.");
static_assert(offsetof(PlaneKinematicObservationV1, body_w_mps) == 56U, "Plane observation offsets changed.");

static_assert(std::is_standard_layout<KinematicObservationInputV1>::value, "Observation input must be standard-layout.");
static_assert(std::is_trivially_copyable<KinematicObservationInputV1>::value, "Observation input must be trivially copyable.");
static_assert(alignof(KinematicObservationInputV1) == 1U, "Observation input ABI alignment changed.");
static_assert(sizeof(KinematicObservationInputV1) == 160U, "Kinematic observation input ABI changed.");
static_assert(offsetof(KinematicObservationInputV1, abi_version) == 0U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, struct_size) == 4U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, command_frame_index) == 8U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, command_time_s) == 16U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, nominal_dt_s) == 24U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, context_own_plane_id) == 32U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, context_target_plane_id) == 36U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, ownship) == 40U, "Observation input offsets changed.");
static_assert(offsetof(KinematicObservationInputV1, target) == 100U, "Observation input offsets changed.");

static_assert(std::is_standard_layout<DerivedPlaneStateV1>::value, "Derived plane state must be standard-layout.");
static_assert(std::is_trivially_copyable<DerivedPlaneStateV1>::value, "Derived plane state must be trivially copyable.");
static_assert(alignof(DerivedPlaneStateV1) == 1U, "Derived plane state ABI alignment changed.");
static_assert(sizeof(DerivedPlaneStateV1) == 112U, "DerivedPlaneStateV1 ABI changed.");
static_assert(offsetof(DerivedPlaneStateV1, frame_index) == 0U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, plane_id) == 8U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, force_side) == 12U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, position_ned_m) == 16U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, rpy_rad) == 40U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, velocity_body_mps) == 64U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, speed_mps) == 88U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, alpha_rad) == 96U, "Derived plane state offsets changed.");
static_assert(offsetof(DerivedPlaneStateV1, beta_rad) == 104U, "Derived plane state offsets changed.");

static_assert(std::is_standard_layout<FrameContractDiagnosticsV1>::value, "Frame diagnostics must be standard-layout.");
static_assert(std::is_trivially_copyable<FrameContractDiagnosticsV1>::value, "Frame diagnostics must be trivially copyable.");
static_assert(alignof(FrameContractDiagnosticsV1) == FrameContractDiagnosticsV1X64LayoutManifest::Alignment, "Frame diagnostics ABI alignment changed.");
static_assert(sizeof(FrameContractDiagnosticsV1) == FrameContractDiagnosticsV1X64LayoutManifest::Size, "FrameContractDiagnosticsV1 ABI changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, abi_version) == FrameContractDiagnosticsV1X64LayoutManifest::AbiVersionOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, struct_size) == FrameContractDiagnosticsV1X64LayoutManifest::StructSizeOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, command_frame_index) == FrameContractDiagnosticsV1X64LayoutManifest::CommandFrameIndexOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, command_time_s) == FrameContractDiagnosticsV1X64LayoutManifest::CommandTimeOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, nominal_dt_s) == FrameContractDiagnosticsV1X64LayoutManifest::NominalDtOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, context_own_plane_id) == FrameContractDiagnosticsV1X64LayoutManifest::ContextOwnPlaneIdOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, context_target_plane_id) == FrameContractDiagnosticsV1X64LayoutManifest::ContextTargetPlaneIdOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, ownship) == FrameContractDiagnosticsV1X64LayoutManifest::OwnshipOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, target) == FrameContractDiagnosticsV1X64LayoutManifest::TargetOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, own_measurement_valid) == FrameContractDiagnosticsV1X64LayoutManifest::OwnMeasurementValidOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, target_observation_valid) == FrameContractDiagnosticsV1X64LayoutManifest::TargetObservationValidOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, same_index_input_candidate) == FrameContractDiagnosticsV1X64LayoutManifest::SameIndexInputCandidateOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, estimator_transaction_committed) == FrameContractDiagnosticsV1X64LayoutManifest::EstimatorTransactionCommittedOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, same_index_geometry_valid) == FrameContractDiagnosticsV1X64LayoutManifest::SameIndexGeometryValidOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, gap_count) == FrameContractDiagnosticsV1X64LayoutManifest::GapCountOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, gap_policy) == FrameContractDiagnosticsV1X64LayoutManifest::GapPolicyOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, episode_epoch) == FrameContractDiagnosticsV1X64LayoutManifest::EpisodeEpochOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, measurement_time_s) == FrameContractDiagnosticsV1X64LayoutManifest::MeasurementTimeOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, sample_dt_s) == FrameContractDiagnosticsV1X64LayoutManifest::SampleDtOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, writer_count) == FrameContractDiagnosticsV1X64LayoutManifest::WriterCountOffset, "Frame diagnostics offsets changed.");
static_assert(offsetof(FrameContractDiagnosticsV1, writer_id) == FrameContractDiagnosticsV1X64LayoutManifest::WriterIdOffset, "Frame diagnostics offsets changed.");
static_assert(std::is_standard_layout<ControlCommandOutcomeDiagnosticsV1>::value, "Command outcome diagnostics must be standard-layout.");
static_assert(std::is_trivially_copyable<ControlCommandOutcomeDiagnosticsV1>::value, "Command outcome diagnostics must be trivially copyable.");
static_assert(alignof(ControlCommandOutcomeDiagnosticsV1) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::Alignment, "Command outcome diagnostics ABI alignment changed.");
static_assert(sizeof(ControlCommandOutcomeDiagnosticsV1) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::Size, "Command outcome diagnostics ABI size changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, abi_version) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::AbiVersionOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, struct_size) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::StructSizeOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, command_frame_index) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::CommandFrameIndexOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, outcome) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::OutcomeOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, current_base_event) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::CurrentBaseEventOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, runtime_status) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::RuntimeStatusOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, control_authorized) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::ControlAuthorizedOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, current_base_started_count) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::CurrentBaseStartedCountOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, current_base_ended_count) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::CurrentBaseEndedCountOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, current_base_active) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::CurrentBaseActiveOffset, "Command outcome diagnostics offsets changed.");
static_assert(offsetof(ControlCommandOutcomeDiagnosticsV1, reserved) == ControlCommandOutcomeDiagnosticsV1X64LayoutManifest::ReservedOffset, "Command outcome diagnostics offsets changed.");
