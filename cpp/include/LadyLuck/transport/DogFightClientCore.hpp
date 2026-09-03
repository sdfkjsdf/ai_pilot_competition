#pragma once

#include "LadyLuck/transport/DogFightPairingState.hpp"

#include <cstddef>
#include <cstdint>

namespace LadyLuck
{
namespace transport
{

// Allocation-free runtime seam. Implementations must return StatusCode::Ok
// only when reset completed and the policy is ready for the next pair.
class ICommandPolicy
{
public:
    virtual ~ICommandPolicy() noexcept = default;

    virtual Status Reset(
        const DogFightPairingSnapshot& context) noexcept = 0;
    virtual Result<CMD> Compute(
        const DogFightReadyPair& pair) noexcept = 0;
};

// The sink owns transport I/O. ClientCore neither opens sockets nor retries a
// failed send, and one call always represents one complete datagram.
class IPacketSink
{
public:
    virtual ~IPacketSink() noexcept = default;

    virtual Status Send(
        const std::uint8_t* data,
        std::size_t size) noexcept = 0;
};

struct DogFightClientCoreConfig
{
    DogFightClientCoreConfig() noexcept;

    std::int32_t simulation_state = 1;
    TeamNameBytes team_name{};
    AIType ai_type = AIType::ReinforcementLearning;
};

// Socket-free chronology owner for the provided Python client's active packet
// path. Raw position, attitude, and body-velocity axes pass through untouched;
// coordinate conversion, guidance, FCS shaping, and command admission remain
// downstream policy responsibilities.
class DogFightClientCore final
{
public:
    static constexpr std::uint32_t ActionRepeat = 1U;

    DogFightClientCore(
        ICommandPolicy& command_policy,
        IPacketSink& packet_sink,
        const DogFightClientCoreConfig& config);

    DogFightClientCore(const DogFightClientCore&) = delete;
    DogFightClientCore& operator=(const DogFightClientCore&) = delete;

    Status ProcessDatagram(
        const std::uint8_t* data,
        std::size_t size) noexcept;

    // Emits exactly SimulationState then ClientJoinInfo. The join packet uses
    // the assigned plane ID captured at entry. UDP has no two-datagram atomic
    // commit: if the second send fails, SimulationState may already have been
    // transmitted. Any failure is terminal for the caller; ClientCore never
    // retries either packet.
    Status EmitHeartbeat() noexcept;

    DogFightPairingSnapshot Snapshot() const noexcept;

private:
    Status ResetPolicy(const DogFightPairingReceipt& receipt) noexcept;
    Status ProcessReadyPair(
        const DogFightPairingReceipt& receipt) noexcept;

    ICommandPolicy& command_policy_;
    IPacketSink& packet_sink_;
    DogFightPairingState pairing_state_{};
    SimulationState heartbeat_state_{};
    ClientJoinInfo heartbeat_client_{};
};

} // namespace transport
} // namespace LadyLuck
