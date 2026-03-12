// Windows headers must come before libmodbus to avoid Winsock conflicts
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <modbus.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// Ensure Winsock is initialised exactly once per process.
struct WsaInit
{
    WsaInit()
    {
#ifdef _WIN32
        WSADATA data{};
        WSAStartup(MAKEWORD(2, 2), &data);
#endif
    }
    ~WsaInit()
    {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};
static WsaInit g_wsaInit;

std::string ToHexBytes(const std::string &value)
{
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (std::size_t i = 0; i < value.size(); ++i)
    {
        if (i > 0)
        {
            oss << '-';
        }
        oss.width(2);
        oss.fill('0');
        oss << static_cast<int>(static_cast<unsigned char>(value[i]));
    }
    return oss.str();
}

#ifdef _WIN32
struct TcpProbeResult
{
    bool success = false;
    int rc = -1;
    int wsa = 0;
    int select_rc = -1;
    int so_error = 0;
    int so_error_capture = 0;
    std::uint64_t socket_handle = 0;
    int fd_setsize = FD_SETSIZE;
    long long elapsed_ms = 0;
};

TcpProbeResult ProbeTcpConnect(const std::string &ip, std::uint16_t port, int timeout_sec)
{
    TcpProbeResult result{};

    const auto begin = std::chrono::steady_clock::now();
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET)
    {
        result.wsa = WSAGetLastError();
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
        return result;
    }
    result.socket_handle = static_cast<std::uint64_t>(s);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    const int pton_rc = InetPtonA(AF_INET, ip.c_str(), &addr.sin_addr);
    if (pton_rc != 1)
    {
        result.rc = -1;
        result.wsa = WSAGetLastError();
        closesocket(s);
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
        return result;
    }

    u_long nonblocking = 1;
    if (ioctlsocket(s, FIONBIO, &nonblocking) != 0)
    {
        result.wsa = WSAGetLastError();
        closesocket(s);
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
        return result;
    }

    result.rc = connect(s, reinterpret_cast<sockaddr *>(&addr), static_cast<int>(sizeof(addr)));
    if (result.rc == 0)
    {
        result.success = true;
        closesocket(s);
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
        return result;
    }

    result.wsa = WSAGetLastError();
    if (result.wsa == WSAEWOULDBLOCK || result.wsa == WSAEINPROGRESS || result.wsa == WSAEALREADY)
    {
        fd_set wset;
        FD_ZERO(&wset);
        FD_SET(s, &wset);

        timeval tv{};
        tv.tv_sec = timeout_sec;
        tv.tv_usec = 0;

        result.select_rc = select(0, nullptr, &wset, nullptr, &tv);
        if (result.select_rc == 1)
        {
            int optlen = static_cast<int>(sizeof(result.so_error));
            if (getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&result.so_error), &optlen) ==
                SOCKET_ERROR)
            {
                result.so_error_capture = WSAGetLastError();
            }
            if (result.so_error == 0)
            {
                result.success = true;
            }
        }
        else if (result.select_rc == 0)
        {
            result.wsa = WSAETIMEDOUT;
        }
        else
        {
            result.wsa = WSAGetLastError();
        }
    }

    closesocket(s);
    result.elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
    return result;
}
#endif

class ModbusClient
{
  public:
    ModbusClient() : ip_("127.0.0.1"), port_(502), ctx_(nullptr), connected_(false), last_errno_(0)
    {
    }

    ~ModbusClient()
    {
        Disconnect();
    }

    // Non-copyable, non-movable
    ModbusClient(const ModbusClient &) = delete;
    ModbusClient &operator=(const ModbusClient &) = delete;

    bool SetIp(const std::string &ip)
    {
        if (ip.empty())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (connected_)
        {
            return false; // must disconnect first
        }

        ip_ = ip;
        return true;
    }

    std::string GetIp() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return ip_;
    }

    bool SetPort(std::uint16_t port)
    {
        if (port == 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (connected_)
        {
            return false; // must disconnect first
        }

        port_ = port;
        return true;
    }

    std::uint16_t GetPort() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return port_;
    }

    bool Connect()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (connected_)
        {
            return true;
        }

        const std::string ip_hex = ToHexBytes(ip_);
#ifdef _WIN32
        errno = 0;
        WSASetLastError(0);
        in_addr parsed_addr{};
        const int ip_parse_rc = InetPtonA(AF_INET, ip_.c_str(), &parsed_addr);
        const int ip_parse_errno = errno;
        const int ip_parse_wsa = WSAGetLastError();
        if (ip_parse_rc != 1)
        {
            const int parse_err =
                (ip_parse_wsa != 0) ? ip_parse_wsa : ((ip_parse_errno != 0) ? ip_parse_errno : EINVAL);
            last_errno_ = 15000 + parse_err;
            last_error_str_ = "ip_precheck failed"
                              " | target=" +
                              ip_ + ":" + std::to_string(static_cast<int>(port_)) +
                              " | ip_len=" + std::to_string(ip_.size()) + " | ip_hex=" + ip_hex +
                              " | inet_pton_rc=" + std::to_string(ip_parse_rc) +
                              " | errno=" + std::to_string(ip_parse_errno) + " | WSA=" + std::to_string(ip_parse_wsa) +
                              " | reason=" + std::string(modbus_strerror(parse_err));
            return false;
        }
#endif

        ctx_ = modbus_new_tcp(ip_.c_str(), static_cast<int>(port_));
        if (!ctx_)
        {
            last_errno_ = 10000 + errno;
            last_error_str_ = "modbus_new_tcp failed"
                              " | target=" +
                              ip_ + ":" + std::to_string(static_cast<int>(port_)) +
                              " | ip_len=" + std::to_string(ip_.size()) + " | ip_hex=" + ip_hex +
                              " | errno=" + std::to_string(errno) + " | reason=" + std::string(modbus_strerror(errno));
            return false;
        }

        // Enable debug mode to see internal errors
        modbus_set_debug(ctx_, TRUE);

        // 3 second connection timeout
        struct timeval tv{};
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        modbus_set_response_timeout(ctx_, tv.tv_sec, static_cast<std::uint32_t>(tv.tv_usec));

        errno = 0;
#ifdef _WIN32
        const TcpProbeResult probe = ProbeTcpConnect(ip_, port_, static_cast<int>(tv.tv_sec));
        WSASetLastError(0);
        const int wsa_before_connect = WSAGetLastError();
#endif
        const int errno_before_connect = errno;
        const auto connect_begin = std::chrono::steady_clock::now();
        const int connect_rc = modbus_connect(ctx_);
        const auto connect_end = std::chrono::steady_clock::now();
        const auto connect_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(connect_end - connect_begin).count();
        const int errno_after_connect = errno;

        if (connect_rc == -1)
        {
#ifdef _WIN32
            const int socket_before_free = modbus_get_socket(ctx_);
            const int wsa_after_connect = WSAGetLastError();
            const int socket_fd_after_connect = modbus_get_socket(ctx_);

            int so_error = 0;
            int so_error_capture = 0;
            if (socket_fd_after_connect >= 0)
            {
                int optlen = static_cast<int>(sizeof(so_error));
                const int gs_rc = getsockopt(socket_fd_after_connect, SOL_SOCKET, SO_ERROR,
                                             reinterpret_cast<char *>(&so_error), &optlen);
                if (gs_rc == SOCKET_ERROR)
                {
                    so_error_capture = WSAGetLastError();
                }
            }

            int err = errno_after_connect;
            if (err == 0)
            {
                err = wsa_after_connect;
            }
            if (err == 0)
            {
                err = so_error;
            }

            last_errno_ = 20000 + err;
            last_error_str_ =
                "modbus_connect failed"
                " | rc=" +
                std::to_string(connect_rc) + " | target=" + ip_ + ":" + std::to_string(static_cast<int>(port_)) +
                " | ip_len=" + std::to_string(ip_.size()) + " | ip_hex=" + ip_hex +
                " | elapsed_ms=" + std::to_string(connect_elapsed_ms) +
                " | errno(before=" + std::to_string(errno_before_connect) +
                ", after=" + std::to_string(errno_after_connect) + ")" +
                " | WSA(before=" + std::to_string(wsa_before_connect) + ", after=" + std::to_string(wsa_after_connect) +
                ")" + " | socket_before_free=" + std::to_string(socket_before_free) +
                " | socket_after=" + std::to_string(socket_fd_after_connect) +
                " | so_error=" + std::to_string(so_error) + " | so_error_capture=" + std::to_string(so_error_capture) +
                " | probe(success=" + std::to_string(probe.success ? 1 : 0) + ", rc=" + std::to_string(probe.rc) +
                ", wsa=" + std::to_string(probe.wsa) + ", select=" + std::to_string(probe.select_rc) +
                ", so_error=" + std::to_string(probe.so_error) +
                ", so_error_capture=" + std::to_string(probe.so_error_capture) +
                ", socket=" + std::to_string(probe.socket_handle) + ", FD_SETSIZE=" + std::to_string(probe.fd_setsize) +
                ", elapsed_ms=" + std::to_string(probe.elapsed_ms) + ")" +
                " | reason=" + std::string(modbus_strerror(err));
#else
            const int err = (errno_after_connect != 0) ? errno_after_connect : errno_before_connect;
            last_errno_ = 20000 + err;
            last_error_str_ = "modbus_connect failed"
                              " | rc=" +
                              std::to_string(connect_rc) + " | target=" + ip_ + ":" +
                              std::to_string(static_cast<int>(port_)) +
                              " | elapsed_ms=" + std::to_string(connect_elapsed_ms) +
                              " | errno(before=" + std::to_string(errno_before_connect) +
                              ", after=" + std::to_string(errno_after_connect) + ")" +
                              " | reason=" + std::string(modbus_strerror(err));
#endif
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }

        last_errno_ = 0;
        last_error_str_.clear();
        connected_ = true;
        return true;
    }

    bool Disconnect()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return true;
        }

        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        connected_ = false;
        return true;
    }

    bool IsConnected() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

    int GetLastErrno() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_errno_;
    }

    std::string GetLastErrorString() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_str_;
    }

    // Read coils (FC01): address 00001-09999, PDU address = address - 1
    // out_bits: array of at least count uint8_t (0 or 1 per bit)
    bool ReadCoils(int address, int count, std::uint8_t *out_bits)
    {
        if (address < 1 || address > 9999 || count <= 0 || !out_bits)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_read_bits(ctx_, address - 1, count, out_bits) != -1;
    }

    // Read discrete inputs (FC02): address 10001-19999, PDU address = address - 10001
    bool ReadDiscreteInputs(int address, int count, std::uint8_t *out_bits)
    {
        if (address < 10001 || address > 19999 || count <= 0 || !out_bits)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_read_input_bits(ctx_, address - 10001, count, out_bits) != -1;
    }

    // Read input registers (FC04): address 30001-39999, PDU address = address - 30001
    bool ReadInputRegisters(int address, int count, std::uint16_t *out_regs)
    {
        if (address < 30001 || address > 39999 || count <= 0 || !out_regs)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_read_input_registers(ctx_, address - 30001, count, out_regs) != -1;
    }

    // Read holding registers (FC03): address 40001-49999, PDU address = address - 40001
    bool ReadHoldingRegisters(int address, int count, std::uint16_t *out_regs)
    {
        if (address < 40001 || address > 49999 || count <= 0 || !out_regs)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_read_registers(ctx_, address - 40001, count, out_regs) != -1;
    }

    // Write single coil (FC05): address 00001-09999, value 0 or 1
    bool WriteCoil(int address, int value)
    {
        if (address < 1 || address > 9999 || (value != 0 && value != 1))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_write_bit(ctx_, address - 1, value) != -1;
    }

    // Write multiple coils (FC0F): address 00001-09999
    bool WriteCoils(int address, int count, const std::uint8_t *bits)
    {
        if (address < 1 || address > 9999 || count <= 0 || !bits)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_write_bits(ctx_, address - 1, count, bits) != -1;
    }

    // Write single holding register (FC06): address 40001-49999
    bool WriteHoldingRegister(int address, int value)
    {
        if (address < 40001 || address > 49999 || value < 0 || value > 65535)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_write_register(ctx_, address - 40001, static_cast<std::uint16_t>(value)) != -1;
    }

    // Write multiple holding registers (FC10): address 40001-49999
    bool WriteHoldingRegisters(int address, int count, const std::uint16_t *regs)
    {
        if (address < 40001 || address > 49999 || count <= 0 || !regs)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_ || !ctx_)
        {
            return false;
        }

        return modbus_write_registers(ctx_, address - 40001, count, regs) != -1;
    }

  private:
    mutable std::mutex mutex_;
    std::string ip_;
    std::uint16_t port_;
    modbus_t *ctx_;
    bool connected_;
    int last_errno_;
    std::string last_error_str_;
};
} // namespace

extern "C"
{
    __declspec(dllexport) void *ModbusClient_Create()
    {
        return new (std::nothrow) ModbusClient();
    }

    __declspec(dllexport) void ModbusClient_Destroy(void *handle)
    {
        delete static_cast<ModbusClient *>(handle);
    }

    __declspec(dllexport) int ModbusClient_SetIp(void *handle, const char *ip)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || ip == nullptr)
        {
            return 0;
        }

        return client->SetIp(ip) ? 1 : 0;
    }

    __declspec(dllexport) int ModbusClient_GetIp(void *handle, char *buffer, int bufferSize)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || buffer == nullptr || bufferSize <= 0)
        {
            return 0;
        }

        const std::string ip = client->GetIp();
        const std::size_t copyLength =
            ip.size() < static_cast<std::size_t>(bufferSize - 1) ? ip.size() : static_cast<std::size_t>(bufferSize - 1);

        std::memcpy(buffer, ip.c_str(), copyLength);
        buffer[copyLength] = '\0';
        return 1;
    }

    __declspec(dllexport) int ModbusClient_SetPort(void *handle, std::uint16_t port)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->SetPort(port) ? 1 : 0;
    }

    __declspec(dllexport) std::uint16_t ModbusClient_GetPort(void *handle)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->GetPort();
    }

    __declspec(dllexport) int ModbusClient_Connect(void *handle)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->Connect() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusClient_Disconnect(void *handle)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->Disconnect() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusClient_IsConnected(void *handle)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->IsConnected() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusClient_GetLastErrno(void *handle)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return -1;
        }

        return client->GetLastErrno();
    }

    __declspec(dllexport) int ModbusClient_GetLastErrorString(void *handle, char *buffer, int bufferSize)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || buffer == nullptr || bufferSize <= 0)
        {
            return 0;
        }

        const std::string errStr = client->GetLastErrorString();
        const std::size_t copyLength = errStr.size() < static_cast<std::size_t>(bufferSize - 1)
                                           ? errStr.size()
                                           : static_cast<std::size_t>(bufferSize - 1);

        std::memcpy(buffer, errStr.c_str(), copyLength);
        buffer[copyLength] = '\0';
        return 1;
    }

    // FC01 – Read Coils
    // out_bits: caller-allocated array of `count` bytes (each 0 or 1)
    __declspec(dllexport) int ModbusClient_ReadCoils(void *handle, int address, int count, std::uint8_t *out_bits)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || out_bits == nullptr)
        {
            return 0;
        }

        return client->ReadCoils(address, count, out_bits) ? 1 : 0;
    }

    // FC02 – Read Discrete Inputs
    __declspec(dllexport) int ModbusClient_ReadDiscreteInputs(void *handle, int address, int count,
                                                              std::uint8_t *out_bits)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || out_bits == nullptr)
        {
            return 0;
        }

        return client->ReadDiscreteInputs(address, count, out_bits) ? 1 : 0;
    }

    // FC04 – Read Input Registers
    // out_regs: caller-allocated array of `count` uint16_t
    __declspec(dllexport) int ModbusClient_ReadInputRegisters(void *handle, int address, int count,
                                                              std::uint16_t *out_regs)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || out_regs == nullptr)
        {
            return 0;
        }

        return client->ReadInputRegisters(address, count, out_regs) ? 1 : 0;
    }

    // FC03 – Read Holding Registers
    __declspec(dllexport) int ModbusClient_ReadHoldingRegisters(void *handle, int address, int count,
                                                                std::uint16_t *out_regs)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || out_regs == nullptr)
        {
            return 0;
        }

        return client->ReadHoldingRegisters(address, count, out_regs) ? 1 : 0;
    }

    // FC05 – Write Single Coil
    __declspec(dllexport) int ModbusClient_WriteCoil(void *handle, int address, int value)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->WriteCoil(address, value) ? 1 : 0;
    }

    // FC0F – Write Multiple Coils
    __declspec(dllexport) int ModbusClient_WriteCoils(void *handle, int address, int count, const std::uint8_t *bits)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || bits == nullptr)
        {
            return 0;
        }

        return client->WriteCoils(address, count, bits) ? 1 : 0;
    }

    // FC06 – Write Single Holding Register
    __declspec(dllexport) int ModbusClient_WriteHoldingRegister(void *handle, int address, int value)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr)
        {
            return 0;
        }

        return client->WriteHoldingRegister(address, value) ? 1 : 0;
    }

    // FC10 – Write Multiple Holding Registers
    __declspec(dllexport) int ModbusClient_WriteHoldingRegisters(void *handle, int address, int count,
                                                                 const std::uint16_t *regs)
    {
        auto *client = static_cast<ModbusClient *>(handle);
        if (client == nullptr || regs == nullptr)
        {
            return 0;
        }

        return client->WriteHoldingRegisters(address, count, regs) ? 1 : 0;
    }
}
