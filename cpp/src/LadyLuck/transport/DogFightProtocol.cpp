#include "LadyLuck/transport/DogFightProtocol.hpp"

#include <cstring>
#include <limits>

static_assert(sizeof(float) == sizeof(std::uint32_t),
    "DogFight protocol requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559,
    "DogFight protocol requires IEC 60559 binary32 float");

namespace
{
bool IsSignedByte(std::int32_t value) noexcept
{
    return value >= -128 && value <= 127;
}

bool IsValidMessageType(std::int32_t value) noexcept
{
    return value >= static_cast<std::int32_t>(
        LadyLuck::transport::MessageType::MT_GameControl)
        && value <= static_cast<std::int32_t>(
            LadyLuck::transport::MessageType::MT_SetPlaneID);
}

bool IsValidAIType(LadyLuck::transport::AIType value) noexcept
{
    const std::int32_t raw = static_cast<std::int32_t>(value);
    return raw >= static_cast<std::int32_t>(
        LadyLuck::transport::AIType::RuleBased)
        && raw <= static_cast<std::int32_t>(
            LadyLuck::transport::AIType::etc);
}

void WriteU32LE(
    std::uint8_t* output,
    std::size_t offset,
    std::uint32_t value) noexcept
{
    output[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    output[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    output[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    output[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void WriteI32LE(
    std::uint8_t* output,
    std::size_t offset,
    std::int32_t value) noexcept
{
    WriteU32LE(output, offset, static_cast<std::uint32_t>(value));
}

void WriteU64LE(
    std::uint8_t* output,
    std::size_t offset,
    std::uint64_t value) noexcept
{
    for (std::size_t byte = 0U; byte < 8U; ++byte)
    {
        output[offset + byte] = static_cast<std::uint8_t>(
            (value >> (byte * 8U)) & 0xFFU);
    }
}

void WriteFloatLE(
    std::uint8_t* output,
    std::size_t offset,
    float value) noexcept
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    WriteU32LE(output, offset, bits);
}

std::uint32_t ReadU32LE(
    const std::uint8_t* input,
    std::size_t offset) noexcept
{
    return static_cast<std::uint32_t>(input[offset])
        | (static_cast<std::uint32_t>(input[offset + 1U]) << 8U)
        | (static_cast<std::uint32_t>(input[offset + 2U]) << 16U)
        | (static_cast<std::uint32_t>(input[offset + 3U]) << 24U);
}

std::int32_t ReadI32LE(
    const std::uint8_t* input,
    std::size_t offset) noexcept
{
    const std::uint32_t value = ReadU32LE(input, offset);
    if (value <= static_cast<std::uint32_t>(
            (std::numeric_limits<std::int32_t>::max)()))
    {
        return static_cast<std::int32_t>(value);
    }
    return static_cast<std::int32_t>(
        static_cast<std::int64_t>(value) - 0x100000000LL);
}

std::uint64_t ReadU64LE(
    const std::uint8_t* input,
    std::size_t offset) noexcept
{
    std::uint64_t value = 0U;
    for (std::size_t byte = 0U; byte < 8U; ++byte)
    {
        value |= static_cast<std::uint64_t>(input[offset + byte])
            << (byte * 8U);
    }
    return value;
}

std::int32_t ReadI8(const std::uint8_t value) noexcept
{
    if (value <= 127U)
    {
        return static_cast<std::int32_t>(value);
    }
    return static_cast<std::int32_t>(value) - 256;
}

float ReadFloatLE(
    const std::uint8_t* input,
    std::size_t offset) noexcept
{
    const std::uint32_t bits = ReadU32LE(input, offset);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

LadyLuck::Status ValidatePacket(
    const std::uint8_t* data,
    std::size_t size,
    std::size_t required_size,
    LadyLuck::transport::MessageType expected_type) noexcept
{
    LadyLuck::Status status{};
    if (data == nullptr || size < required_size)
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
        return status;
    }

    const std::int32_t raw_type = ReadI32LE(data, 0U);
    if (!IsValidMessageType(raw_type)
        || raw_type != static_cast<std::int32_t>(expected_type))
    {
        status.code = LadyLuck::StatusCode::InvalidArgument;
    }
    return status;
}

}

namespace LadyLuck
{
namespace transport
{
Result<std::size_t> TeamNameBytes::Assign(
    const char* const utf8_bytes,
    const std::size_t byte_count) noexcept
{
    Result<std::size_t> result{};
    if (utf8_bytes == nullptr && byte_count != 0U)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    TeamNameBytes candidate{};
    const std::size_t retained_bytes = byte_count < TeamNameContentCapacity
        ? byte_count
        : TeamNameContentCapacity;
    for (std::size_t index = 0U; index < retained_bytes; ++index)
    {
        candidate.bytes[index] = static_cast<std::uint8_t>(
            static_cast<unsigned char>(utf8_bytes[index]));
    }
    candidate.bytes[TeamNameContentCapacity] = 0U;

    *this = candidate;
    result.value = retained_bytes;
    return result;
}

Result<SimulationStateWire> PackSimulationState(
    const SimulationState& state) noexcept
{
    Result<SimulationStateWire> result{};
    if (!IsSignedByte(state.state))
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    WriteI32LE(
        result.value.data(),
        0U,
        static_cast<std::int32_t>(MessageType::MT_SimState));
    result.value[4U] = static_cast<std::uint8_t>(state.state);
    return result;
}

Result<ClientJoinInfoWire> PackClientJoinInfo(
    const ClientJoinInfo& info) noexcept
{
    Result<ClientJoinInfoWire> result{};
    if (!IsValidAIType(info.ai_type) || !IsSignedByte(info.plane_id))
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    WriteI32LE(
        result.value.data(),
        0U,
        static_cast<std::int32_t>(MessageType::MT_ClientInfo));
    for (std::size_t index = 0U; index < TeamNameContentCapacity; ++index)
    {
        result.value[4U + index] = info.team_name.bytes[index];
    }
    result.value[4U + TeamNameContentCapacity] = 0U;
    WriteI32LE(
        result.value.data(),
        34U,
        static_cast<std::int32_t>(info.ai_type));
    result.value[38U] = static_cast<std::uint8_t>(info.plane_id);
    return result;
}

Result<CMDWire> PackCMD(const CMD& command) noexcept
{
    Result<CMDWire> result{};
    if (!IsSignedByte(command.plane_id))
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    WriteI32LE(
        result.value.data(),
        0U,
        static_cast<std::int32_t>(MessageType::MT_CMD));
    result.value[4U] = static_cast<std::uint8_t>(command.plane_id);
    WriteU64LE(result.value.data(), 5U, command.index);
    WriteFloatLE(result.value.data(), 13U, command.roll_cmd);
    WriteFloatLE(result.value.data(), 17U, command.pitch_cmd);
    WriteFloatLE(result.value.data(), 21U, command.yaw_cmd);
    WriteFloatLE(result.value.data(), 25U, command.throttle_cmd);
    return result;
}

Result<MessageType> UnpackMessageType(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    Result<MessageType> result{};
    if (data == nullptr || size < MessageTypeWireSize)
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }

    const std::int32_t raw_type = ReadI32LE(data, 0U);
    if (!IsValidMessageType(raw_type))
    {
        result.status.code = StatusCode::InvalidArgument;
        return result;
    }
    result.value = static_cast<MessageType>(raw_type);
    return result;
}

Result<GameControl> UnpackGameControl(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    Result<GameControl> result{};
    result.status = ValidatePacket(
        data,
        size,
        GameControlWireSize,
        MessageType::MT_GameControl);
    if (!result.status.ok())
    {
        return result;
    }

    result.value.command = ReadI8(data[4U]);
    return result;
}

Result<SetPlaneID> UnpackSetPlaneID(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    Result<SetPlaneID> result{};
    result.status = ValidatePacket(
        data,
        size,
        SetPlaneIDWireSize,
        MessageType::MT_SetPlaneID);
    if (!result.status.ok())
    {
        return result;
    }

    result.value.plane_id = ReadI8(data[4U]);
    return result;
}

Result<Init> UnpackInit(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    Result<Init> result{};
    result.status = ValidatePacket(
        data,
        size,
        InitWireSize,
        MessageType::MT_Init);
    if (!result.status.ok())
    {
        return result;
    }

    Init candidate{};
    candidate.plane1_location = Vector3D{
        ReadFloatLE(data, 4U),
        ReadFloatLE(data, 8U),
        ReadFloatLE(data, 12U)};
    candidate.plane1_rotation = Rotation3D{
        ReadFloatLE(data, 16U),
        ReadFloatLE(data, 20U),
        ReadFloatLE(data, 24U)};
    candidate.plane1_speed = ReadFloatLE(data, 28U);
    candidate.plane2_location = Vector3D{
        ReadFloatLE(data, 32U),
        ReadFloatLE(data, 36U),
        ReadFloatLE(data, 40U)};
    candidate.plane2_rotation = Rotation3D{
        ReadFloatLE(data, 44U),
        ReadFloatLE(data, 48U),
        ReadFloatLE(data, 52U)};
    candidate.plane2_speed = ReadFloatLE(data, 56U);

    result.value = candidate;
    return result;
}

Result<PlaneInfo> UnpackPlaneInfo(
    const std::uint8_t* data,
    std::size_t size) noexcept
{
    Result<PlaneInfo> result{};
    result.status = ValidatePacket(
        data,
        size,
        PlaneInfoWireSize,
        MessageType::MT_PlaneInfo);
    if (!result.status.ok())
    {
        return result;
    }

    PlaneInfo candidate{};
    candidate.index = ReadU64LE(data, 4U);
    candidate.plane_id = ReadI8(data[12U]);
    candidate.position = Vector3D{
        ReadFloatLE(data, 13U),
        ReadFloatLE(data, 17U),
        ReadFloatLE(data, 21U)};
    candidate.rotation = Rotation3D{
        ReadFloatLE(data, 25U),
        ReadFloatLE(data, 29U),
        ReadFloatLE(data, 33U)};
    candidate.velocity = Vector3D{
        ReadFloatLE(data, 37U),
        ReadFloatLE(data, 41U),
        ReadFloatLE(data, 45U)};

    result.value = candidate;
    return result;
}
}
}
