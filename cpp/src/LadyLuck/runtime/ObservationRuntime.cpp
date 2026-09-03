#include "LadyLuck/runtime/ObservationRuntime.h"

#include "LadyLuck/behavior_tree/static/StaticDoctrineCommandProvider.hpp"

#include <atomic>
#include <cmath>
#include <cstring>

namespace
{
std::atomic_flag RuntimeLock = ATOMIC_FLAG_INIT;
std::atomic_flag ControlLock = ATOMIC_FLAG_INIT;
KinematicObservationInputV1 LastInput{};
bool HasLastInput = false;
std::int32_t LastStatus = AIP_RUNTIME_NOT_RUN;
std::uint64_t AcceptedCount = 0U;
bool LastControlAuthorized = false;
ControlCommandOutcomeDiagnosticsV1 LastCommandOutcomeDiagnostics{};
bool HasLastCommandOutcomeDiagnostics = false;
LadyLuck::behavior_tree::static_bt::StaticDoctrineCommandProvider
    CommandProvider;
LadyLuck::runtime::TacticalControlCore ControlCore(CommandProvider);

static_assert(
    static_cast<std::int32_t>(
        LadyLuck::runtime::ControlCommandOutcome::CurrentBase) ==
        AIP_CONTROL_OUTCOME_CURRENT_BASE &&
    static_cast<std::int32_t>(
        LadyLuck::runtime::ControlCommandOutcome::Tactical) ==
        AIP_CONTROL_OUTCOME_TACTICAL &&
    static_cast<std::int32_t>(
        LadyLuck::runtime::ControlCommandOutcome::Safety) ==
        AIP_CONTROL_OUTCOME_SAFETY &&
    static_cast<std::int32_t>(
        LadyLuck::runtime::ControlCommandOutcome::InputRejected) ==
        AIP_CONTROL_OUTCOME_INPUT_REJECTED,
    "internal and exported command outcome values diverged");
static_assert(
    static_cast<std::int32_t>(
        LadyLuck::runtime::CurrentBaseOwnershipEvent::None) ==
        AIP_CURRENT_BASE_EVENT_NONE &&
    static_cast<std::int32_t>(
        LadyLuck::runtime::CurrentBaseOwnershipEvent::Started) ==
        AIP_CURRENT_BASE_EVENT_STARTED &&
    static_cast<std::int32_t>(
        LadyLuck::runtime::CurrentBaseOwnershipEvent::Ended) ==
        AIP_CURRENT_BASE_EVENT_ENDED,
    "internal and exported CurrentBase event values diverged");

class RuntimeLockGuard
{
public:
    RuntimeLockGuard() noexcept
    {
        while (RuntimeLock.test_and_set(std::memory_order_acquire))
        {
        }
    }

    ~RuntimeLockGuard() noexcept
    {
        RuntimeLock.clear(std::memory_order_release);
    }

    RuntimeLockGuard(const RuntimeLockGuard&) = delete;
    RuntimeLockGuard& operator=(const RuntimeLockGuard&) = delete;
};

class ControlLockGuard
{
public:
    ControlLockGuard() noexcept
    {
        while (ControlLock.test_and_set(std::memory_order_acquire))
        {
        }
    }

    ~ControlLockGuard() noexcept
    {
        ControlLock.clear(std::memory_order_release);
    }

    ControlLockGuard(const ControlLockGuard&) = delete;
    ControlLockGuard& operator=(const ControlLockGuard&) = delete;
};

bool IsFinite(const float value) noexcept
{
    return std::isfinite(static_cast<double>(value));
}

bool IsFinite(const double value) noexcept
{
    return std::isfinite(value);
}

bool PlaneIsFinite(const PlaneKinematicObservationV1& plane) noexcept
{
    return IsFinite(plane.position_n_m)
        && IsFinite(plane.position_e_m)
        && IsFinite(plane.position_up_m)
        && IsFinite(plane.roll_deg)
        && IsFinite(plane.pitch_deg)
        && IsFinite(plane.yaw_deg)
        && IsFinite(plane.body_u_mps)
        && IsFinite(plane.body_v_mps)
        && IsFinite(plane.body_w_mps);
}

std::int32_t Validate(const KinematicObservationInputV1* input) noexcept
{
    if (input == nullptr)
    {
        return AIP_RUNTIME_NULL_INPUT;
    }

    // Validate the top-level prefix before reading fields at V1-only offsets.
    // This makes an obsolete/undersized caller fail without reading past the
    // size it declared.
    if (input->abi_version != AIPILOT_ABI_VERSION_V1)
    {
        return AIP_RUNTIME_INVALID_ABI_VERSION;
    }

    if (input->struct_size != sizeof(KinematicObservationInputV1))
    {
        return AIP_RUNTIME_INVALID_STRUCT_SIZE;
    }

    if (input->ownship.abi_version != AIPILOT_ABI_VERSION_V1)
    {
        return AIP_RUNTIME_INVALID_ABI_VERSION;
    }

    if (input->ownship.struct_size != sizeof(PlaneKinematicObservationV1))
    {
        return AIP_RUNTIME_INVALID_STRUCT_SIZE;
    }

    if (input->target.abi_version != AIPILOT_ABI_VERSION_V1)
    {
        return AIP_RUNTIME_INVALID_ABI_VERSION;
    }

    if (input->target.struct_size != sizeof(PlaneKinematicObservationV1))
    {
        return AIP_RUNTIME_INVALID_STRUCT_SIZE;
    }

    if (!IsFinite(input->command_time_s)
        || !IsFinite(input->nominal_dt_s)
        || input->command_time_s < 0.0
        || input->nominal_dt_s <= 0.0
        || input->nominal_dt_s != AIPILOT_NOMINAL_DT_S_V1
        || input->command_time_s
            != static_cast<double>(input->command_frame_index)
                * AIPILOT_NOMINAL_DT_S_V1)
    {
        return AIP_RUNTIME_INVALID_TIME;
    }

    if (input->ownship.plane_id < 0
        || input->ownship.force_side == 0
        || input->target.plane_id < 0
        || input->target.force_side == 0
        || input->context_own_plane_id != input->ownship.plane_id
        || input->context_target_plane_id != input->target.plane_id
        || input->target.plane_id == input->ownship.plane_id
        || input->target.force_side == input->ownship.force_side)
    {
        return AIP_RUNTIME_BT_CONTRACT_FAILED;
    }

    if (!PlaneIsFinite(input->ownship)
        || !PlaneIsFinite(input->target))
    {
        return AIP_RUNTIME_NONFINITE_INPUT;
    }

    // A zero-speed member or coincident pair is transport readiness state,
    // not a flight-control sample. Keep the shared pre-start contract outside
    // LastInput and ControlCore::Step so it cannot seed estimator/BT state.
    if (AIPilotPreStartCommandNeutralV1(*input))
    {
        return AIP_RUNTIME_PRESTART_COMMAND_NEUTRAL;
    }

    return AIP_RUNTIME_INPUT_ACCEPTED_PORT_INCOMPLETE;
}
}

namespace AIP_Runtime
{
std::int32_t RecordKinematicObservationV1(
    const KinematicObservationInputV1* input) noexcept
{
    const std::int32_t status = Validate(input);

    RuntimeLockGuard lock;
    LastStatus = status;
    // A newly presented observation has no command authority until the
    // current-frame control transaction explicitly succeeds.
    LastControlAuthorized = false;
    if (status == AIP_RUNTIME_INPUT_ACCEPTED_PORT_INCOMPLETE)
    {
        std::memcpy(&LastInput, input, sizeof(LastInput));
        HasLastInput = true;
    }
    else if (status == AIP_RUNTIME_PRESTART_COMMAND_NEUTRAL)
    {
        std::memset(&LastInput, 0, sizeof(LastInput));
        HasLastInput = false;
        LastCommandOutcomeDiagnostics.abi_version = AIPILOT_ABI_VERSION_V1;
        LastCommandOutcomeDiagnostics.struct_size =
            static_cast<std::uint32_t>(
                sizeof(ControlCommandOutcomeDiagnosticsV1));
        LastCommandOutcomeDiagnostics.command_frame_index =
            input->command_frame_index;
        LastCommandOutcomeDiagnostics.outcome =
            AIP_CONTROL_OUTCOME_INPUT_REJECTED;
        LastCommandOutcomeDiagnostics.current_base_event =
            AIP_CURRENT_BASE_EVENT_NONE;
        LastCommandOutcomeDiagnostics.runtime_status = status;
        LastCommandOutcomeDiagnostics.control_authorized = 0U;
        HasLastCommandOutcomeDiagnostics = true;
    }
    else
    {
        LastCommandOutcomeDiagnostics.abi_version = AIPILOT_ABI_VERSION_V1;
        LastCommandOutcomeDiagnostics.struct_size =
            static_cast<std::uint32_t>(
                sizeof(ControlCommandOutcomeDiagnosticsV1));
        LastCommandOutcomeDiagnostics.command_frame_index = 0U;
        LastCommandOutcomeDiagnostics.outcome =
            AIP_CONTROL_OUTCOME_INPUT_REJECTED;
        LastCommandOutcomeDiagnostics.current_base_event =
            AIP_CURRENT_BASE_EVENT_NONE;
        LastCommandOutcomeDiagnostics.runtime_status = status;
        LastCommandOutcomeDiagnostics.control_authorized = 0U;
        HasLastCommandOutcomeDiagnostics = true;
    }

    return status;
}

void CommitAcceptedKinematicObservationV1() noexcept
{
    RuntimeLockGuard lock;
    ++AcceptedCount;
}

ControlValue NeutralControlV1() noexcept
{
    // ABI return value for a pre-start placeholder or rejected external input.
    // This value is never a tactical fallback and never carries command
    // authority; callers must use LastControlAuthorized/status to distinguish
    // it from a current-frame FCS command.
    return ControlValue{ 0.0F, 0.0F, 0.0F, 0.0F };
}

LadyLuck::Status PrepareControlOwnerV1(
    const std::int32_t owner_plane_id,
    const std::int32_t owner_force_side) noexcept
{
    ControlLockGuard lock;
    LadyLuck::Status status{};
    ControlCore.PrepareOwner(
        owner_plane_id,
        owner_force_side,
        status);
    return status;
}

void CopyStaticDoctrineProviderSnapshotV1(
    LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderSnapshot& output,
    LadyLuck::Status& status) noexcept
{
    ControlLockGuard lock;
    CommandProvider.CopySnapshot(output, status);
}

void PreflightProductionControlInputV1(
    const KinematicObservationInputV1& input,
    LadyLuck::runtime::ControlCorePreflightReceipt& output) noexcept
{
    ControlLockGuard lock;
    ControlCore.Preflight(input, output);
}

ControlValue StepProductionControlV1(
    const KinematicObservationInputV1& input) noexcept
{
    LadyLuck::runtime::ControlCoreReceipt receipt{};
    LadyLuck::runtime::TacticalControlCoreSnapshot core_snapshot{};
    {
        ControlLockGuard lock;
        ControlCore.Step(input, receipt);
        ControlCore.CopySnapshot(core_snapshot);
    }

    RuntimeLockGuard lock;
    LastControlAuthorized = receipt.control_authorized;
    LastCommandOutcomeDiagnostics.abi_version = AIPILOT_ABI_VERSION_V1;
    LastCommandOutcomeDiagnostics.struct_size =
        static_cast<std::uint32_t>(
            sizeof(ControlCommandOutcomeDiagnosticsV1));
    LastCommandOutcomeDiagnostics.command_frame_index =
        input.command_frame_index;
    LastCommandOutcomeDiagnostics.outcome =
        static_cast<std::int32_t>(receipt.outcome);
    LastCommandOutcomeDiagnostics.current_base_event =
        static_cast<std::int32_t>(receipt.current_base_event);
    LastCommandOutcomeDiagnostics.control_authorized =
        receipt.control_authorized ? 1U : 0U;
    LastCommandOutcomeDiagnostics.current_base_started_count =
        core_snapshot.current_base_started_count;
    LastCommandOutcomeDiagnostics.current_base_ended_count =
        core_snapshot.current_base_ended_count;
    LastCommandOutcomeDiagnostics.current_base_active =
        core_snapshot.current_base_active ? 1U : 0U;
    LastCommandOutcomeDiagnostics.reserved = 0U;
    HasLastCommandOutcomeDiagnostics = true;
    if (receipt.prestart_command_neutral)
    {
        LastStatus = AIP_RUNTIME_PRESTART_COMMAND_NEUTRAL;
        LastCommandOutcomeDiagnostics.runtime_status = LastStatus;
        LastControlAuthorized = false;
        return NeutralControlV1();
    }
    if (receipt.estimator_transaction_committed)
    {
        ++AcceptedCount;
    }
    if (!receipt.control_authorized)
    {
        LastStatus = receipt.fault_class
                == LadyLuck::runtime::ControlCoreFaultClass::InputRejected
            ? AIP_RUNTIME_BT_CONTRACT_FAILED
            : AIP_RUNTIME_INTERNAL_CONTRACT_FAULT;
        LastCommandOutcomeDiagnostics.runtime_status = LastStatus;
        return NeutralControlV1();
    }

    LastStatus = AIP_RUNTIME_CONTROL_PRIMARY;
    LastCommandOutcomeDiagnostics.runtime_status = LastStatus;
    return receipt.control;
}

std::uint32_t GetLastControlAuthorizationV1() noexcept
{
    RuntimeLockGuard lock;
    return LastControlAuthorized ? 1U : 0U;
}

std::int32_t CopyLastControlCommandOutcomeDiagnosticsV1(
    ControlCommandOutcomeDiagnosticsV1* output,
    const std::uint32_t output_size) noexcept
{
    if (output == nullptr ||
        output_size < sizeof(ControlCommandOutcomeDiagnosticsV1))
    {
        return AIP_RUNTIME_INVALID_OUTPUT_BUFFER;
    }

    std::memset(output, 0, sizeof(ControlCommandOutcomeDiagnosticsV1));
    RuntimeLockGuard lock;
    if (!HasLastCommandOutcomeDiagnostics)
    {
        return AIP_RUNTIME_NOT_RUN;
    }
    std::memcpy(
        output,
        &LastCommandOutcomeDiagnostics,
        sizeof(LastCommandOutcomeDiagnostics));
    return LastCommandOutcomeDiagnostics.runtime_status;
}

void SetLastObservationStatusV1(const std::int32_t status) noexcept
{
    RuntimeLockGuard lock;
    LastStatus = status;
    LastControlAuthorized = false;
    LastCommandOutcomeDiagnostics.abi_version = AIPILOT_ABI_VERSION_V1;
    LastCommandOutcomeDiagnostics.struct_size =
        static_cast<std::uint32_t>(
            sizeof(ControlCommandOutcomeDiagnosticsV1));
    LastCommandOutcomeDiagnostics.command_frame_index =
        HasLastInput ? LastInput.command_frame_index : 0U;
    LastCommandOutcomeDiagnostics.outcome =
        AIP_CONTROL_OUTCOME_INPUT_REJECTED;
    LastCommandOutcomeDiagnostics.current_base_event =
        AIP_CURRENT_BASE_EVENT_NONE;
    LastCommandOutcomeDiagnostics.runtime_status = status;
    LastCommandOutcomeDiagnostics.control_authorized = 0U;
    HasLastCommandOutcomeDiagnostics = true;
}

std::int32_t CopyLastKinematicObservationV1(
    KinematicObservationInputV1* output,
    const std::uint32_t output_size) noexcept
{
    if (output == nullptr || output_size < sizeof(KinematicObservationInputV1))
    {
        return AIP_RUNTIME_INVALID_OUTPUT_BUFFER;
    }

    std::memset(output, 0, sizeof(KinematicObservationInputV1));
    RuntimeLockGuard lock;
    if (!HasLastInput || LastStatus < 0)
    {
        return LastStatus;
    }

    std::memcpy(output, &LastInput, sizeof(LastInput));
    return LastStatus;
}

std::int32_t GetLastObservationStatusV1() noexcept
{
    RuntimeLockGuard lock;
    return LastStatus;
}

std::uint64_t GetAcceptedObservationCountV1() noexcept
{
    RuntimeLockGuard lock;
    return AcceptedCount;
}

void ResetObservationRuntimeV1() noexcept
{
    {
        ControlLockGuard control_lock;
        LadyLuck::Status reset_status{};
        ControlCore.Reset(reset_status);
        static_cast<void>(reset_status);
    }
    RuntimeLockGuard lock;
    std::memset(&LastInput, 0, sizeof(LastInput));
    HasLastInput = false;
    LastStatus = AIP_RUNTIME_NOT_RUN;
    AcceptedCount = 0U;
    LastControlAuthorized = false;
    std::memset(
        &LastCommandOutcomeDiagnostics,
        0,
        sizeof(LastCommandOutcomeDiagnostics));
    HasLastCommandOutcomeDiagnostics = false;
}
}
