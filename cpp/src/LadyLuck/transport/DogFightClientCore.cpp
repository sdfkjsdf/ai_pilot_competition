#include "LadyLuck/transport/DogFightClientCore.hpp"

namespace LadyLuck
{
namespace transport
{
namespace
{

Status RequireExactSuccess(const Status status) noexcept
{
    if (status.code == StatusCode::Ok
        || static_cast<std::int32_t>(status.code) < 0)
    {
        return status;
    }

    // ICommandPolicy and IPacketSink are transaction APIs, not observer
    // receipt APIs. A non-negative, non-Ok receipt violates this seam and must
    // not admit a default value or an invented zero command.
    Status failure{};
    failure.code = StatusCode::InvalidConfiguration;
    return failure;
}

} // namespace

DogFightClientCoreConfig::DogFightClientCoreConfig() noexcept
{
    const Result<std::size_t> assignment = team_name.Assign("ASDF");
    (void)assignment;
}

DogFightClientCore::DogFightClientCore(
    ICommandPolicy& command_policy,
    IPacketSink& packet_sink,
    const DogFightClientCoreConfig& config)
    : command_policy_(command_policy),
      packet_sink_(packet_sink)
{
    heartbeat_state_.state = config.simulation_state;
    heartbeat_client_.team_name = config.team_name;
    heartbeat_client_.ai_type = config.ai_type;
}

Status DogFightClientCore::ResetPolicy(
    const DogFightPairingReceipt& receipt) noexcept
{
    if (!receipt.policy_reset_required)
    {
        return Status{};
    }
    return RequireExactSuccess(command_policy_.Reset(pairing_state_.Snapshot()));
}

Status DogFightClientCore::ProcessReadyPair(
    const DogFightPairingReceipt& receipt) noexcept
{
    if (!receipt.has_ready_pair)
    {
        return Status{};
    }

    // ActionRepeat is deliberately fixed at one: each complete raw pair owns
    // exactly one policy call and, on success, one command datagram attempt.
    const Result<CMD> command = command_policy_.Compute(receipt.ready_pair);
    const Status policy_status = RequireExactSuccess(command.status);
    if (policy_status.code != StatusCode::Ok)
    {
        return policy_status;
    }

    const Result<CMDWire> packet = PackCMD(command.value);
    if (packet.status.code != StatusCode::Ok)
    {
        return packet.status;
    }

    return RequireExactSuccess(
        packet_sink_.Send(packet.value.data(), packet.value.size()));
}

Status DogFightClientCore::ProcessDatagram(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    const Result<MessageType> message_type = UnpackMessageType(data, size);
    if (message_type.status.code != StatusCode::Ok)
    {
        return message_type.status;
    }

    switch (message_type.value)
    {
    case MessageType::MT_SetPlaneID:
    {
        const Result<SetPlaneID> packet = UnpackSetPlaneID(data, size);
        if (packet.status.code != StatusCode::Ok)
        {
            return packet.status;
        }
        return ResetPolicy(pairing_state_.OnSetPlaneID(packet.value));
    }
    case MessageType::MT_Init:
    {
        const Result<Init> packet = UnpackInit(data, size);
        if (packet.status.code != StatusCode::Ok)
        {
            return packet.status;
        }
        return ResetPolicy(pairing_state_.OnInit(packet.value));
    }
    case MessageType::MT_GameControl:
    {
        const Result<GameControl> packet = UnpackGameControl(data, size);
        if (packet.status.code != StatusCode::Ok)
        {
            return packet.status;
        }
        pairing_state_.OnGameControl(packet.value);
        return Status{};
    }
    case MessageType::MT_PlaneInfo:
    {
        const Result<PlaneInfo> packet = UnpackPlaneInfo(data, size);
        if (packet.status.code != StatusCode::Ok)
        {
            return packet.status;
        }
        return ProcessReadyPair(pairing_state_.OnPlaneInfo(packet.value));
    }
    case MessageType::MT_Damage:
    case MessageType::MT_SimState:
    case MessageType::MT_VP:
    case MessageType::MT_CMD:
    case MessageType::MT_StatgeInfo:
    case MessageType::MT_ClientInfo:
        // Known but client-unhandled messages match client.py's no-op branch.
        return Status{};
    default:
    {
        Status status{};
        status.code = StatusCode::InvalidArgument;
        return status;
    }
    }
}

Status DogFightClientCore::EmitHeartbeat() noexcept
{
    const DogFightPairingSnapshot snapshot = pairing_state_.Snapshot();
    heartbeat_client_.plane_id = snapshot.assigned_plane_id;

    // Pack both fixed messages before performing either external side effect.
    const Result<SimulationStateWire> simulation_packet =
        PackSimulationState(heartbeat_state_);
    if (simulation_packet.status.code != StatusCode::Ok)
    {
        return simulation_packet.status;
    }
    const Result<ClientJoinInfoWire> client_packet =
        PackClientJoinInfo(heartbeat_client_);
    if (client_packet.status.code != StatusCode::Ok)
    {
        return client_packet.status;
    }

    // These are two independent UDP side effects. A failure of the second
    // send is returned after the first may have committed; the application
    // must stop rather than retrying an ambiguous heartbeat transaction.
    Status status = RequireExactSuccess(packet_sink_.Send(
        simulation_packet.value.data(),
        simulation_packet.value.size()));
    if (status.code != StatusCode::Ok)
    {
        return status;
    }
    status = RequireExactSuccess(packet_sink_.Send(
        client_packet.value.data(),
        client_packet.value.size()));
    return status;
}

DogFightPairingSnapshot DogFightClientCore::Snapshot() const noexcept
{
    return pairing_state_.Snapshot();
}

} // namespace transport
} // namespace LadyLuck
