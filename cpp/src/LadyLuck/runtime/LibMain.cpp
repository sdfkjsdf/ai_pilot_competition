#include <cstring>
#include "LadyLuck/runtime/AIPilotABI.h"
#include "LadyLuck/runtime/ObservationRuntime.h"
#include "LadyLuck/guidance/contracts/FrameContract.h"
#include "LadyLuck/runtime/ProductionOwnerRegistryV1.hpp"

extern "C"
{
    // Versioned lossless competition-observation interface. It contains only
    // the kinematics supplied by PlaneInfo plus connector-owned timing/roles.
    __declspec(dllexport) std::uint32_t GetAIPilotABIVersion() noexcept;
    __declspec(dllexport) ControlValue StepKinematicObservationV1(
        const KinematicObservationInputV1* Input) noexcept;
    __declspec(dllexport) std::int32_t CopyLastKinematicObservationV1(
        KinematicObservationInputV1* Output,
        std::uint32_t OutputSize) noexcept;
    __declspec(dllexport) std::int32_t GetLastKinematicObservationStatusV1() noexcept;
    __declspec(dllexport) std::uint32_t GetLastControlAuthorizationV1() noexcept;
    __declspec(dllexport) std::int32_t CopyLastControlCommandOutcomeDiagnosticsV1(
        ControlCommandOutcomeDiagnosticsV1* Output,
        std::uint32_t OutputSize) noexcept;
    __declspec(dllexport) std::uint64_t GetAcceptedKinematicObservationCountV1() noexcept;
    __declspec(dllexport) std::int32_t CreateBehaviorTreeV1(int OwnerID, int ForceID) noexcept;
    __declspec(dllexport) std::uint32_t GetBehaviorTreeCountV1() noexcept;
    __declspec(dllexport) std::int32_t CopyFrameContractDiagnosticsV1(
        int OwnerID,
        FrameContractDiagnosticsV1* Output,
        std::uint32_t OutputSize) noexcept;

    __declspec(dllexport) void Reset();
    __declspec(dllexport) void RemoveBT(int OwnerID);

}

LadyLuck::runtime::ProductionOwnerRegistryV1 ProductionOwnersV1;

std::uint32_t GetAIPilotABIVersion() noexcept
{
    return AIPILOT_ABI_VERSION_V1;
}

ControlValue StepKinematicObservationV1(
    const KinematicObservationInputV1* Input) noexcept
{
    ControlValue output = AIP_Runtime::NeutralControlV1();
    const std::int32_t input_status =
        AIP_Runtime::RecordKinematicObservationV1(Input);
    if (input_status == AIP_RUNTIME_INPUT_ACCEPTED_PORT_INCOMPLETE)
    {
        const auto owner = ProductionOwnersV1.Find(
            Input->ownship.plane_id);
        if (owner.code
            != LadyLuck::runtime::ProductionOwnerRegistryV1Code::Found)
        {
            AIP_Runtime::SetLastObservationStatusV1(
                AIP_RUNTIME_BT_NOT_FOUND);
        }
        else
        {
            // ControlCore first admits the estimator/geometry frame, then its
            // injected doctrine provider ticks the only command-owning tree.
            output = AIP_Runtime::StepProductionControlV1(*Input);
        }
    }
    return output;
}

std::int32_t CopyLastKinematicObservationV1(
    KinematicObservationInputV1* Output,
    const std::uint32_t OutputSize) noexcept
{
    return AIP_Runtime::CopyLastKinematicObservationV1(Output, OutputSize);
}

std::int32_t GetLastKinematicObservationStatusV1() noexcept
{
    return AIP_Runtime::GetLastObservationStatusV1();
}

std::uint32_t GetLastControlAuthorizationV1() noexcept
{
    return AIP_Runtime::GetLastControlAuthorizationV1();
}

std::int32_t CopyLastControlCommandOutcomeDiagnosticsV1(
    ControlCommandOutcomeDiagnosticsV1* Output,
    const std::uint32_t OutputSize) noexcept
{
    return AIP_Runtime::CopyLastControlCommandOutcomeDiagnosticsV1(
        Output, OutputSize);
}

std::uint64_t GetAcceptedKinematicObservationCountV1() noexcept
{
    return AIP_Runtime::GetAcceptedObservationCountV1();
}

std::uint32_t GetBehaviorTreeCountV1() noexcept
{
    return ProductionOwnersV1.Count();
}

std::int32_t CopyFrameContractDiagnosticsV1(
    const int OwnerID,
    FrameContractDiagnosticsV1* Output,
    const std::uint32_t OutputSize) noexcept
{
    if (Output == nullptr || OutputSize < sizeof(FrameContractDiagnosticsV1))
    {
        return AIP_RUNTIME_INVALID_OUTPUT_BUFFER;
    }

    std::memset(Output, 0, sizeof(FrameContractDiagnosticsV1));

    const std::int32_t runtime_status = AIP_Runtime::GetLastObservationStatusV1();
    if (runtime_status < 0)
    {
        return runtime_status;
    }

    if (ProductionOwnersV1.Find(OwnerID).code
        != LadyLuck::runtime::ProductionOwnerRegistryV1Code::Found)
    {
        return AIP_RUNTIME_BT_NOT_FOUND;
    }

    KinematicObservationInputV1 input{};
    const std::int32_t copy_status =
        AIP_Runtime::CopyLastKinematicObservationV1(
            &input,
            static_cast<std::uint32_t>(sizeof(input)));
    if (copy_status < 0
        || input.ownship.plane_id != OwnerID
        || !AIP_Guidance::BuildFrameContractV1(input, *Output))
    {
        return AIP_RUNTIME_BT_CONTRACT_FAILED;
    }

    LadyLuck::behavior_tree::static_bt::
        StaticDoctrineCommandProviderSnapshot receipt{};
    LadyLuck::Status receipt_status{};
    AIP_Runtime::CopyStaticDoctrineProviderSnapshotV1(
        receipt,
        receipt_status);
    if (receipt_status.code != LadyLuck::StatusCode::Ok
        || receipt.candidate_disposition
            != LadyLuck::behavior_tree::static_bt::
                StaticDoctrineCandidateDisposition::Selected
        || receipt.selected_candidate_count != 1U
        || receipt.selected_candidate.writer_id
            == LadyLuck::ControlIntentWriterNone
        || receipt.selected_candidate.frame_identity.frame_index
            != input.ownship.frame_index)
    {
        return AIP_RUNTIME_BT_CONTRACT_FAILED;
    }
    Output->writer_count = receipt.selected_candidate_count;
    Output->writer_id = static_cast<std::int32_t>(
        receipt.selected_candidate.writer_id);
    return runtime_status;
}

std::int32_t CreateBehaviorTreeV1(const int OwnerID, const int ForceID) noexcept
{
    using LadyLuck::runtime::ProductionOwnerRegistryV1Code;

    const auto inspection = ProductionOwnersV1.InspectCreate(
        OwnerID,
        ForceID);
    if (inspection.code
        == ProductionOwnerRegistryV1Code::AlreadyExistsSameForce)
    {
        return AIP_TREE_ALREADY_EXISTS;
    }
    if (inspection.code != ProductionOwnerRegistryV1Code::NotFound)
    {
        return AIP_TREE_INITIALIZATION_FAILED;
    }

    const LadyLuck::Status prepare_status =
        AIP_Runtime::PrepareControlOwnerV1(OwnerID, ForceID);
    if (prepare_status.code != LadyLuck::StatusCode::Ok)
    {
        return AIP_TREE_INITIALIZATION_FAILED;
    }

    const auto committed = ProductionOwnersV1.Create(OwnerID, ForceID);
    if (committed.code != ProductionOwnerRegistryV1Code::Created)
    {
        return AIP_TREE_INITIALIZATION_FAILED;
    }
    return AIP_TREE_CREATED;
}

void Reset()
{
    (void)ProductionOwnersV1.Reset();
    AIP_Runtime::ResetObservationRuntimeV1();
}

void RemoveBT(int OwnerID)
{
    const auto removed = ProductionOwnersV1.Remove(OwnerID);
    if (removed.code
        == LadyLuck::runtime::ProductionOwnerRegistryV1Code::Removed)
    {
        AIP_Runtime::ResetObservationRuntimeV1();
    }
}
