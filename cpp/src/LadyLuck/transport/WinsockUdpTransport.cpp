#include "LadyLuck/transport/WinsockUdpTransport.hpp"

#if !defined(_WIN32)
#error WinsockUdpTransport requires Windows
#endif

#include <winsock2.h>
#include <ws2tcpip.h>

#include <climits>
#include <cstring>

namespace LadyLuck
{
namespace transport
{
namespace
{

static_assert(sizeof(SOCKET) <= sizeof(std::uintptr_t),
    "SOCKET handle does not fit transport storage");
static_assert(sizeof(sockaddr_in) == 16U,
    "unexpected IPv4 sockaddr layout");

SOCKET NativeSocket(const std::uintptr_t handle) noexcept
{
    return static_cast<SOCKET>(handle);
}

WinsockUdpReceipt Receipt(
    const WinsockUdpCode code,
    const int native_error = 0,
    const std::size_t byte_count = 0U) noexcept
{
    WinsockUdpReceipt receipt{};
    receipt.code = code;
    receipt.native_error = native_error;
    receipt.byte_count = byte_count;
    return receipt;
}

bool IsAsciiSpace(const char value) noexcept
{
    return value == ' ' || value == '\t' || value == '\r'
        || value == '\n' || value == '\f' || value == '\v';
}

char AsciiLower(const char value) noexcept
{
    if (value >= 'A' && value <= 'Z')
    {
        return static_cast<char>(value - 'A' + 'a');
    }
    return value;
}

bool EqualsTrimmedAsciiInsensitive(
    const char* const value,
    const char* const expected) noexcept
{
    if (value == nullptr || expected == nullptr)
    {
        return false;
    }

    const char* begin = value;
    while (*begin != '\0' && IsAsciiSpace(*begin))
    {
        ++begin;
    }
    const char* end = begin + std::strlen(begin);
    while (end != begin && IsAsciiSpace(*(end - 1)))
    {
        --end;
    }

    const char* right = expected;
    const char* left = begin;
    while (left != end && *right != '\0')
    {
        if (AsciiLower(*left) != AsciiLower(*right))
        {
            return false;
        }
        ++left;
        ++right;
    }
    return left == end && *right == '\0';
}

bool BuildEndpoint(
    const sockaddr_in& address,
    WinsockUdpEndpoint& endpoint) noexcept
{
    endpoint = WinsockUdpEndpoint{};
    const std::uint8_t* const address_bytes =
        reinterpret_cast<const std::uint8_t*>(&address.sin_addr);
    for (std::size_t index = 0U; index < endpoint.ipv4_address.size(); ++index)
    {
        endpoint.ipv4_address[index] = address_bytes[index];
    }
    endpoint.port = ntohs(address.sin_port);

    const char* const formatted = InetNtopA(
        AF_INET,
        const_cast<IN_ADDR*>(&address.sin_addr),
        endpoint.address_text.data(),
        static_cast<DWORD>(endpoint.address_text.size()));
    if (formatted == nullptr)
    {
        endpoint = WinsockUdpEndpoint{};
        return false;
    }
    endpoint.is_valid = true;
    return true;
}

bool ResolveServer(
    const char* const host,
    const std::uint16_t port,
    const bool loopback,
    sockaddr_in& address,
    int& native_error) noexcept
{
    address = sockaddr_in{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    if (loopback)
    {
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        native_error = 0;
        return true;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* addresses = nullptr;
    const int result = getaddrinfo(host, nullptr, &hints, &addresses);
    if (result != 0 || addresses == nullptr)
    {
        native_error = result;
        if (addresses != nullptr)
        {
            freeaddrinfo(addresses);
        }
        return false;
    }

    bool copied = false;
    for (const addrinfo* candidate = addresses;
         candidate != nullptr;
         candidate = candidate->ai_next)
    {
        if (candidate->ai_addr != nullptr
            && candidate->ai_addrlen >= static_cast<int>(sizeof(sockaddr_in)))
        {
            const sockaddr_in* const ipv4 =
                reinterpret_cast<const sockaddr_in*>(candidate->ai_addr);
            address.sin_addr = ipv4->sin_addr;
            copied = true;
            break;
        }
    }
    freeaddrinfo(addresses);
    native_error = copied ? 0 : WSAHOST_NOT_FOUND;
    return copied;
}

bool ConfigureReceiveTimeout(
    const SOCKET socket_handle,
    const std::uint32_t timeout_ms,
    int& native_error) noexcept
{
    if (timeout_ms == 0U)
    {
        u_long nonblocking = 1UL;
        if (ioctlsocket(socket_handle, FIONBIO, &nonblocking) == SOCKET_ERROR)
        {
            native_error = WSAGetLastError();
            return false;
        }
        native_error = 0;
        return true;
    }

    const DWORD timeout = static_cast<DWORD>(timeout_ms);
    if (setsockopt(
            socket_handle,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeout),
            static_cast<int>(sizeof(timeout))) == SOCKET_ERROR)
    {
        native_error = WSAGetLastError();
        return false;
    }
    native_error = 0;
    return true;
}

} // namespace

WinsockUdpTransport::~WinsockUdpTransport() noexcept
{
    static_cast<void>(Close());
    if (winsock_started_)
    {
        WSACleanup();
        winsock_started_ = false;
    }
}

bool WinsockUdpTransport::IsLoopbackHost(const char* const host) noexcept
{
    return EqualsTrimmedAsciiInsensitive(host, "localhost")
        || EqualsTrimmedAsciiInsensitive(host, "127.0.0.1")
        || EqualsTrimmedAsciiInsensitive(host, "::1");
}

WinsockUdpReceipt WinsockUdpTransport::Open(
    const WinsockUdpTransportConfig& config) noexcept
{
    if (socket_handle_ != InvalidSocketHandle)
    {
        return Receipt(WinsockUdpCode::AlreadyOpen);
    }
    if (config.server_host == nullptr || config.server_host[0] == '\0')
    {
        return Receipt(WinsockUdpCode::InvalidArgument, WSAEINVAL);
    }

    if (!winsock_started_)
    {
        WSADATA data{};
        const int startup = WSAStartup(MAKEWORD(2, 2), &data);
        if (startup != 0)
        {
            return Receipt(WinsockUdpCode::WinsockStartupFailed, startup);
        }
        if (LOBYTE(data.wVersion) != 2 || HIBYTE(data.wVersion) != 2)
        {
            WSACleanup();
            return Receipt(
                WinsockUdpCode::WinsockStartupFailed,
                WSAVERNOTSUPPORTED);
        }
        winsock_started_ = true;
    }

    const bool loopback = IsLoopbackHost(config.server_host);
    sockaddr_in server_address{};
    int native_error = 0;
    if (!ResolveServer(
            config.server_host,
            config.server_port,
            loopback,
            server_address,
            native_error))
    {
        return Receipt(
            WinsockUdpCode::AddressResolutionFailed,
            native_error);
    }

    const SOCKET opened = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (opened == INVALID_SOCKET)
    {
        return Receipt(
            WinsockUdpCode::SocketCreationFailed,
            WSAGetLastError());
    }

    WinsockUdpCode failure_code = WinsockUdpCode::Opened;
    if (loopback)
    {
        sockaddr_in local_address{};
        local_address.sin_family = AF_INET;
        local_address.sin_port = htons(0U);
        local_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(
                opened,
                reinterpret_cast<const sockaddr*>(&local_address),
                static_cast<int>(sizeof(local_address))) == SOCKET_ERROR)
        {
            native_error = WSAGetLastError();
            failure_code = WinsockUdpCode::BindFailed;
        }
    }
    else if (connect(
                 opened,
                 reinterpret_cast<const sockaddr*>(&server_address),
                 static_cast<int>(sizeof(server_address))) == SOCKET_ERROR)
    {
        native_error = WSAGetLastError();
        failure_code = WinsockUdpCode::ConnectFailed;
    }

    sockaddr_in local_address{};
    int local_address_size = static_cast<int>(sizeof(local_address));
    WinsockUdpEndpoint local_endpoint{};
    WinsockUdpEndpoint server_endpoint{};
    if (failure_code == WinsockUdpCode::Opened
        && (getsockname(
                opened,
                reinterpret_cast<sockaddr*>(&local_address),
                &local_address_size) == SOCKET_ERROR
            || !BuildEndpoint(local_address, local_endpoint)))
    {
        native_error = WSAGetLastError();
        failure_code = WinsockUdpCode::LocalEndpointQueryFailed;
    }
    if (failure_code == WinsockUdpCode::Opened
        && !BuildEndpoint(server_address, server_endpoint))
    {
        native_error = WSAGetLastError();
        failure_code = WinsockUdpCode::AddressResolutionFailed;
    }
    if (failure_code == WinsockUdpCode::Opened
        && !ConfigureReceiveTimeout(
            opened,
            config.receive_timeout_ms,
            native_error))
    {
        failure_code = WinsockUdpCode::TimeoutConfigurationFailed;
    }

    if (failure_code != WinsockUdpCode::Opened)
    {
        closesocket(opened);
        return Receipt(failure_code, native_error);
    }

    socket_handle_ = static_cast<std::uintptr_t>(opened);
    mode_ = loopback
        ? WinsockUdpMode::LocalUnconnected
        : WinsockUdpMode::Connected;
    receive_timeout_ms_ = config.receive_timeout_ms;
    local_endpoint_ = local_endpoint;
    server_endpoint_ = server_endpoint;
    std::memcpy(
        server_socket_address_.data(),
        &server_address,
        sizeof(server_address));
    server_socket_address_size_ = static_cast<std::int32_t>(
        sizeof(server_address));
    return Receipt(WinsockUdpCode::Opened);
}

WinsockUdpReceipt WinsockUdpTransport::SendPacket(
    const std::uint8_t* const data,
    const std::size_t size) const noexcept
{
    if (socket_handle_ == InvalidSocketHandle)
    {
        return Receipt(WinsockUdpCode::NotOpen, WSAENOTSOCK);
    }
    if ((data == nullptr && size != 0U)
        || size > static_cast<std::size_t>(INT_MAX))
    {
        return Receipt(
            size > static_cast<std::size_t>(INT_MAX)
                ? WinsockUdpCode::PayloadTooLarge
                : WinsockUdpCode::InvalidArgument,
            WSAEINVAL);
    }

    const char empty_payload = '\0';
    const char* const payload = size == 0U
        ? &empty_payload
        : reinterpret_cast<const char*>(data);
    int sent = SOCKET_ERROR;
    if (mode_ == WinsockUdpMode::LocalUnconnected)
    {
        sockaddr_in server_address{};
        std::memcpy(
            &server_address,
            server_socket_address_.data(),
            sizeof(server_address));
        sent = sendto(
            NativeSocket(socket_handle_),
            payload,
            static_cast<int>(size),
            0,
            reinterpret_cast<const sockaddr*>(&server_address),
            server_socket_address_size_);
    }
    else
    {
        sent = send(
            NativeSocket(socket_handle_),
            payload,
            static_cast<int>(size),
            0);
    }

    if (sent == SOCKET_ERROR)
    {
        return Receipt(WinsockUdpCode::SendFailed, WSAGetLastError());
    }
    if (static_cast<std::size_t>(sent) != size)
    {
        return Receipt(
            WinsockUdpCode::PartialDatagramSend,
            0,
            static_cast<std::size_t>(sent));
    }
    return Receipt(WinsockUdpCode::Sent, 0, size);
}

WinsockUdpReceiveResult WinsockUdpTransport::ReceivePacket() const noexcept
{
    WinsockUdpReceiveResult result{};
    if (socket_handle_ == InvalidSocketHandle)
    {
        result.receipt = Receipt(WinsockUdpCode::NotOpen, WSAENOTSOCK);
        return result;
    }

    sockaddr_in remote_address{};
    int remote_address_size = static_cast<int>(sizeof(remote_address));
    int received = SOCKET_ERROR;
    if (mode_ == WinsockUdpMode::LocalUnconnected)
    {
        received = recvfrom(
            NativeSocket(socket_handle_),
            reinterpret_cast<char*>(result.bytes.data()),
            static_cast<int>(result.bytes.size()),
            0,
            reinterpret_cast<sockaddr*>(&remote_address),
            &remote_address_size);
    }
    else
    {
        received = recv(
            NativeSocket(socket_handle_),
            reinterpret_cast<char*>(result.bytes.data()),
            static_cast<int>(result.bytes.size()),
            0);
    }

    if (received == SOCKET_ERROR)
    {
        const int native_error = WSAGetLastError();
        if (native_error == WSAETIMEDOUT || native_error == WSAEWOULDBLOCK)
        {
            result.receipt = Receipt(
                WinsockUdpCode::TimedOut,
                native_error);
            return result;
        }
        if (native_error == WSAEMSGSIZE)
        {
            result.receipt = Receipt(
                WinsockUdpCode::DatagramTruncated,
                native_error,
                result.bytes.size());
            if (mode_ == WinsockUdpMode::LocalUnconnected)
            {
                static_cast<void>(BuildEndpoint(
                    remote_address,
                    result.remote_endpoint));
            }
            else
            {
                result.remote_endpoint = server_endpoint_;
            }
            return result;
        }
        result.receipt = Receipt(
            WinsockUdpCode::ReceiveFailed,
            native_error);
        return result;
    }

    if (mode_ == WinsockUdpMode::LocalUnconnected)
    {
        if (!BuildEndpoint(remote_address, result.remote_endpoint))
        {
            result.receipt = Receipt(
                WinsockUdpCode::ReceiveFailed,
                WSAGetLastError());
            return result;
        }
    }
    else
    {
        result.remote_endpoint = server_endpoint_;
    }
    result.receipt = Receipt(
        WinsockUdpCode::Received,
        0,
        static_cast<std::size_t>(received));
    return result;
}

WinsockUdpReceipt WinsockUdpTransport::Close() noexcept
{
    if (socket_handle_ == InvalidSocketHandle)
    {
        return Receipt(WinsockUdpCode::AlreadyClosed);
    }

    const SOCKET closing = NativeSocket(socket_handle_);
    socket_handle_ = InvalidSocketHandle;
    mode_ = WinsockUdpMode::Closed;
    receive_timeout_ms_ = 0U;
    local_endpoint_ = WinsockUdpEndpoint{};
    server_endpoint_ = WinsockUdpEndpoint{};
    server_socket_address_ = std::array<std::uint8_t, 16>{};
    server_socket_address_size_ = 0;

    if (closesocket(closing) == SOCKET_ERROR)
    {
        return Receipt(WinsockUdpCode::CloseFailed, WSAGetLastError());
    }
    return Receipt(WinsockUdpCode::Closed);
}

WinsockUdpTransportSnapshot WinsockUdpTransport::Snapshot() const noexcept
{
    WinsockUdpTransportSnapshot snapshot{};
    snapshot.winsock_started = winsock_started_;
    snapshot.is_open = socket_handle_ != InvalidSocketHandle;
    snapshot.mode = mode_;
    snapshot.receive_timeout_ms = receive_timeout_ms_;
    snapshot.local_endpoint = local_endpoint_;
    snapshot.server_endpoint = server_endpoint_;
    return snapshot;
}

} // namespace transport
} // namespace LadyLuck
