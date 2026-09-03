#pragma once

#include "LadyLuck/contracts/Status.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace transport
{
enum class MessageType : std::int32_t
{
    MT_GameControl = 0,
    MT_Init = 1,
    MT_PlaneInfo = 2,
    MT_Damage = 3,
    MT_SimState = 4,
    MT_VP = 5,
    MT_CMD = 6,
    MT_StatgeInfo = 7,
    MT_ClientInfo = 8,
    MT_SetPlaneID = 9
};

enum class AIType : std::int32_t
{
    RuleBased = 0,
    ReinforcementLearning = 1,
    SupervisedLearning = 2,
    Fusion = 3,
    etc = 4
};

struct Vector3D
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Rotation3D
{
    float roll = 0.0F;
    float pitch = 0.0F;
    float yaw = 0.0F;
};

struct GameControl
{
    std::int32_t command = 0;
};

struct Init
{
    Vector3D plane1_location{};
    Rotation3D plane1_rotation{};
    float plane1_speed = 0.0F;
    Vector3D plane2_location{};
    Rotation3D plane2_rotation{};
    float plane2_speed = 0.0F;
};

struct PlaneInfo
{
    std::uint64_t index = 0U;
    std::int32_t plane_id = 0;
    Vector3D position{};
    Rotation3D rotation{};
    Vector3D velocity{};
};

struct SimulationState
{
    std::int32_t state = 0;
};

struct SetPlaneID
{
    std::int32_t plane_id = 0;
};

constexpr std::size_t TeamNameWireStorageSize = 30U;
constexpr std::size_t TeamNameContentCapacity = 29U;

// Fixed representation of the protocol's 30-byte team-name field. At most 29
// caller bytes are retained; byte 29 is always NUL/padding. An oversized input
// is deliberately accepted and truncated to the first 29 bytes, preserving the
// provided Python protocol behavior. Assign() returns the retained byte count
// so truncation is observable without exceptions or allocation.
struct TeamNameBytes
{
    std::array<std::uint8_t, TeamNameWireStorageSize> bytes{};

    Result<std::size_t> Assign(
        const char* utf8_bytes,
        std::size_t byte_count) noexcept;

    template <std::size_t Size>
    Result<std::size_t> Assign(const char (&utf8_bytes)[Size]) noexcept
    {
        const std::size_t byte_count = Size > 0U
                && utf8_bytes[Size - 1U] == '\0'
            ? Size - 1U
            : Size;
        return Assign(utf8_bytes, byte_count);
    }

    // Compatibility for existing literal and contiguous-text configuration
    // ingress. Production code that needs the retained length calls Assign()
    // directly and checks its typed Result.
    template <std::size_t Size>
    TeamNameBytes& operator=(const char (&utf8_bytes)[Size]) noexcept
    {
        (void)Assign(utf8_bytes);
        return *this;
    }

    template <typename ContiguousText>
    TeamNameBytes& operator=(const ContiguousText& utf8_bytes) noexcept
    {
        (void)Assign(
            utf8_bytes.data(),
            static_cast<std::size_t>(utf8_bytes.size()));
        return *this;
    }
};

static_assert(
    sizeof(TeamNameBytes) == TeamNameWireStorageSize,
    "TeamNameBytes must contain exactly the 30-byte wire storage");
static_assert(
    std::is_standard_layout<TeamNameBytes>::value,
    "TeamNameBytes must use a standard layout");
static_assert(
    std::is_trivially_copyable<TeamNameBytes>::value,
    "TeamNameBytes must remain trivially copyable");

struct ClientJoinInfo
{
    TeamNameBytes team_name{};
    AIType ai_type = AIType::RuleBased;
    std::int32_t plane_id = 0;
};

struct CMD
{
    std::int32_t plane_id = 0;
    std::uint64_t index = 0U;
    float roll_cmd = 0.0F;
    float pitch_cmd = 0.0F;
    float yaw_cmd = 0.0F;
    float throttle_cmd = 0.0F;
};

constexpr std::size_t MessageTypeWireSize = 4U;
constexpr std::size_t GameControlWireSize = 5U;
constexpr std::size_t InitWireSize = 60U;
constexpr std::size_t PlaneInfoWireSize = 49U;
constexpr std::size_t SimulationStateWireSize = 5U;
constexpr std::size_t SetPlaneIDWireSize = 5U;
constexpr std::size_t ClientJoinInfoWireSize = 39U;
constexpr std::size_t CMDWireSize = 29U;

using SimulationStateWire = std::array<std::uint8_t, SimulationStateWireSize>;
using ClientJoinInfoWire = std::array<std::uint8_t, ClientJoinInfoWireSize>;
using CMDWire = std::array<std::uint8_t, CMDWireSize>;

// These codecs only transport raw simulator observations and commands. They do
// not shape guidance references or generate body-rate, load, surface, or thrust
// commands. Position and velocity float bits are preserved without a Z-axis
// flip; the single NEU-Up to NED-Down position conversion is downstream-owned,
// while body-axis u/v/w velocity must never be flipped. IEEE NaN and infinity
// payloads are likewise transported unchanged; downstream observation policy
// owns finite-value admission. Buffers may contain a trailing datagram payload,
// matching the Python decoder's prefix behavior.
Result<SimulationStateWire> PackSimulationState(
    const SimulationState& state) noexcept;
Result<ClientJoinInfoWire> PackClientJoinInfo(
    const ClientJoinInfo& info) noexcept;
Result<CMDWire> PackCMD(const CMD& command) noexcept;

Result<MessageType> UnpackMessageType(
    const std::uint8_t* data,
    std::size_t size) noexcept;
Result<GameControl> UnpackGameControl(
    const std::uint8_t* data,
    std::size_t size) noexcept;
Result<SetPlaneID> UnpackSetPlaneID(
    const std::uint8_t* data,
    std::size_t size) noexcept;
Result<Init> UnpackInit(
    const std::uint8_t* data,
    std::size_t size) noexcept;
Result<PlaneInfo> UnpackPlaneInfo(
    const std::uint8_t* data,
    std::size_t size) noexcept;
}
}
