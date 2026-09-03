#pragma once

#include "AIPilotABI.h"
#include "LadyLuck/behavior_tree/static/StaticDoctrineCommandProvider.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/runtime/TacticalControlCore.hpp"

namespace AIP_Runtime
{
std::int32_t RecordKinematicObservationV1(
    const KinematicObservationInputV1* input) noexcept;
void CommitAcceptedKinematicObservationV1() noexcept;
ControlValue NeutralControlV1() noexcept;
LadyLuck::Status PrepareControlOwnerV1(
    std::int32_t owner_plane_id,
    std::int32_t owner_force_side) noexcept;
void CopyStaticDoctrineProviderSnapshotV1(
    LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderSnapshot& output,
    LadyLuck::Status& status) noexcept;
void PreflightProductionControlInputV1(
    const KinematicObservationInputV1& input,
    LadyLuck::runtime::ControlCorePreflightReceipt& output) noexcept;
ControlValue StepProductionControlV1(
    const KinematicObservationInputV1& input) noexcept;
std::uint32_t GetLastControlAuthorizationV1() noexcept;
std::int32_t CopyLastControlCommandOutcomeDiagnosticsV1(
    ControlCommandOutcomeDiagnosticsV1* output,
    std::uint32_t output_size) noexcept;
void SetLastObservationStatusV1(std::int32_t status) noexcept;
std::int32_t CopyLastKinematicObservationV1(
    KinematicObservationInputV1* output,
    std::uint32_t output_size) noexcept;
std::int32_t GetLastObservationStatusV1() noexcept;
std::uint64_t GetAcceptedObservationCountV1() noexcept;
void ResetObservationRuntimeV1() noexcept;
}
