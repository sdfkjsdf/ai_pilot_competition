#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace LadyLuck
{
namespace transport
{

constexpr std::size_t WinsockUdpReceiveCapacity = 1024U;

enum class WinsockUdpMode : std::int32_t
{
    Closed = 0,
    LocalUnconnected = 1,
    Connected = 2
};

enum class WinsockUdpCode : std::int32_t
{
    Opened = 0,
    AlreadyOpen = 1,
    Sent = 2,
    Received = 3,
    TimedOut = 4,
    Closed = 5,
    AlreadyClosed = 6,
    DatagramTruncated = 7,
    InvalidArgument = -1,
    WinsockStartupFailed = -2,
    AddressResolutionFailed = -3,
    SocketCreationFailed = -4,
    BindFailed = -5,
    ConnectFailed = -6,
    TimeoutConfigurationFailed = -7,
    LocalEndpointQueryFailed = -8,
    NotOpen = -9,
    PayloadTooLarge = -10,
    SendFailed = -11,
    PartialDatagramSend = -12,
    ReceiveFailed = -13,
    CloseFailed = -14
};

struct WinsockUdpEndpoint
{
    bool is_valid = false;
    // Network-order IPv4 octets are retained independently of text format.
    std::array<std::uint8_t, 4> ipv4_address{};
    std::uint16_t port = 0U;
    std::array<char, 16> address_text{};
};

struct WinsockUdpTransportConfig
{
    // Consumed synchronously by Open(); ownership remains with the caller.
    const char* server_host = nullptr;
    std::uint16_t server_port = 9999U;
    std::uint32_t receive_timeout_ms = 200U;
};

struct WinsockUdpReceipt
{
    WinsockUdpCode code = WinsockUdpCode::InvalidArgument;
    int native_error = 0;
    std::size_t byte_count = 0U;

    bool ok() const noexcept
    {
        return static_cast<std::int32_t>(code) >= 0;
    }
};

struct WinsockUdpReceiveResult
{
    WinsockUdpReceipt receipt{};
    std::array<std::uint8_t, WinsockUdpReceiveCapacity> bytes{};
    WinsockUdpEndpoint remote_endpoint{};

    bool has_datagram() const noexcept
    {
        // A truncated UDP message is not safe input to the protocol decoder.
        // The prefix remains available for diagnostics, but only a complete
        // message may cross the ClientCore boundary.
        return receipt.code == WinsockUdpCode::Received;
    }

    bool has_truncated_prefix() const noexcept
    {
        return receipt.code == WinsockUdpCode::DatagramTruncated;
    }
};

struct WinsockUdpTransportSnapshot
{
    bool winsock_started = false;
    bool is_open = false;
    WinsockUdpMode mode = WinsockUdpMode::Closed;
    std::uint32_t receive_timeout_ms = 0U;
    WinsockUdpEndpoint local_endpoint{};
    WinsockUdpEndpoint server_endpoint{};
};

// Owns one Winsock UDP socket. Open/Close are lifecycle calls and must be
// externally serialized. Once open, one receiving thread and sending threads
// may use ReceivePacket/SendPacket concurrently; those calls do not mutate the
// transport configuration. No method retries or generates a command packet.
class WinsockUdpTransport final
{
public:
    WinsockUdpTransport() noexcept = default;
    ~WinsockUdpTransport() noexcept;

    WinsockUdpTransport(const WinsockUdpTransport&) = delete;
    WinsockUdpTransport& operator=(const WinsockUdpTransport&) = delete;
    WinsockUdpTransport(WinsockUdpTransport&&) = delete;
    WinsockUdpTransport& operator=(WinsockUdpTransport&&) = delete;

    WinsockUdpReceipt Open(
        const WinsockUdpTransportConfig& config) noexcept;
    WinsockUdpReceipt SendPacket(
        const std::uint8_t* data,
        std::size_t size) const noexcept;
    WinsockUdpReceiveResult ReceivePacket() const noexcept;
    WinsockUdpReceipt Close() noexcept;

    WinsockUdpTransportSnapshot Snapshot() const noexcept;

    static bool IsLoopbackHost(const char* host) noexcept;

private:
    static constexpr std::uintptr_t InvalidSocketHandle =
        static_cast<std::uintptr_t>(-1);

    bool winsock_started_ = false;
    std::uintptr_t socket_handle_ = InvalidSocketHandle;
    WinsockUdpMode mode_ = WinsockUdpMode::Closed;
    std::uint32_t receive_timeout_ms_ = 0U;
    WinsockUdpEndpoint local_endpoint_{};
    WinsockUdpEndpoint server_endpoint_{};
    std::array<std::uint8_t, 16> server_socket_address_{};
    std::int32_t server_socket_address_size_ = 0;
};

} // namespace transport
} // namespace LadyLuck
