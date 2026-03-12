// Windows headers must come before libmodbus to avoid Winsock conflicts
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#include <modbus.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

namespace
{
enum class AddressType : int
{
    Coil = 0,
    DiscreteInput = 1,
    InputRegister = 2,
    HoldingRegister = 3
};

class ModbusServer
{
  public:
    ModbusServer() : ip_("0.0.0.0"), port_(502), running_(false), should_stop_(false)
    {
    }

    ~ModbusServer()
    {
        should_stop_ = true;
        if (server_thread_.joinable())
        {
            server_thread_.join();
        }
    }

    // Non-copyable, non-movable
    ModbusServer(const ModbusServer &) = delete;
    ModbusServer &operator=(const ModbusServer &) = delete;

    bool Start()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_)
            {
                return true;
            }
        }

        // Ensure any previous thread has finished
        if (server_thread_.joinable())
        {
            server_thread_.join();
        }

        should_stop_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = true;
        }
        server_thread_ = std::thread(&ModbusServer::ServerLoop, this);
        return true;
    }

    bool Stop()
    {
        should_stop_ = true;
        if (server_thread_.joinable())
        {
            server_thread_.join();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        return true;
    }

    bool IsRunning() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return running_;
    }

    bool SetIp(const std::string &ip)
    {
        if (ip.empty())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
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
        port_ = port;
        return true;
    }

    std::uint16_t GetPort() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return port_;
    }

    bool AddAddress(AddressType type, int address)
    {
        if (address < 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        addresses_[TypeIndex(type)].push_back(address);
        values_[TypeIndex(type)].push_back(0);
        return true;
    }

    bool RemoveAddress(AddressType type, int index)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto &addrs = addresses_[TypeIndex(type)];
        auto &vals = values_[TypeIndex(type)];
        if (index < 0 || static_cast<std::size_t>(index) >= addrs.size())
        {
            return false;
        }

        addrs.erase(addrs.begin() + index);
        vals.erase(vals.begin() + index);
        return true;
    }

    int GetAddressCount(AddressType type) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(addresses_[TypeIndex(type)].size());
    }

    bool GetAddressAt(AddressType type, int index, int &outAddress) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto &addrs = addresses_[TypeIndex(type)];
        if (index < 0 || static_cast<std::size_t>(index) >= addrs.size())
        {
            return false;
        }

        outAddress = addrs[static_cast<std::size_t>(index)];
        return true;
    }

    bool SetAddressAt(AddressType type, int index, int address)
    {
        if (address < 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto &addrs = addresses_[TypeIndex(type)];
        if (index < 0 || static_cast<std::size_t>(index) >= addrs.size())
        {
            return false;
        }

        addrs[static_cast<std::size_t>(index)] = address;
        return true;
    }

    bool GetValueAt(AddressType type, int index, int &outValue) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto &vals = values_[TypeIndex(type)];
        if (index < 0 || static_cast<std::size_t>(index) >= vals.size())
        {
            return false;
        }

        outValue = vals[static_cast<std::size_t>(index)];
        return true;
    }

    bool SetValueAt(AddressType type, int index, int value)
    {
        if (!IsValueInRange(type, value))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto &vals = values_[TypeIndex(type)];
        if (index < 0 || static_cast<std::size_t>(index) >= vals.size())
        {
            return false;
        }

        vals[static_cast<std::size_t>(index)] = value;
        return true;
    }

  private:
    // Numbers of entries allocated in the libmodbus mapping.
    // Covers full Modbus address ranges:
    //   Coils         00001-09999  → tab_bits[0..9998]
    //   Discrete      10001-19999  → tab_input_bits[0..9998]
    //   Input regs    30001-39999  → tab_input_registers[0..9998]
    //   Holding regs  40001-49999  → tab_registers[0..9998]
    static constexpr int kRangeSize = 9999;

    static std::size_t TypeIndex(AddressType type)
    {
        return static_cast<std::size_t>(type);
    }

    static bool IsValueInRange(AddressType type, int value)
    {
        switch (type)
        {
        case AddressType::Coil:
        case AddressType::DiscreteInput:
            return value == 0 || value == 1;
        case AddressType::InputRegister:
        case AddressType::HoldingRegister:
            return value >= 0 && value <= 65535;
        default:
            return false;
        }
    }

    // Must be called with mutex_ held.
    // Writes all configured address/value pairs into the libmodbus mapping tables.
    void SyncValuesToMapping(modbus_mapping_t *mb) const
    {
        // Coils: address 00001-09999 → PDU index = address - 1
        const auto &coil_addrs = addresses_[0];
        const auto &coil_vals = values_[0];
        for (std::size_t i = 0; i < coil_addrs.size() && i < coil_vals.size(); ++i)
        {
            int idx = coil_addrs[i] - 1;
            if (idx >= 0 && idx < kRangeSize)
            {
                mb->tab_bits[idx] = static_cast<std::uint8_t>(coil_vals[i]);
            }
        }

        // Discrete inputs: address 10001-19999 → PDU index = address - 10001
        const auto &di_addrs = addresses_[1];
        const auto &di_vals = values_[1];
        for (std::size_t i = 0; i < di_addrs.size() && i < di_vals.size(); ++i)
        {
            int idx = di_addrs[i] - 10001;
            if (idx >= 0 && idx < kRangeSize)
            {
                mb->tab_input_bits[idx] = static_cast<std::uint8_t>(di_vals[i]);
            }
        }

        // Input registers: address 30001-39999 → PDU index = address - 30001
        const auto &ir_addrs = addresses_[2];
        const auto &ir_vals = values_[2];
        for (std::size_t i = 0; i < ir_addrs.size() && i < ir_vals.size(); ++i)
        {
            int idx = ir_addrs[i] - 30001;
            if (idx >= 0 && idx < kRangeSize)
            {
                mb->tab_input_registers[idx] = static_cast<std::uint16_t>(ir_vals[i]);
            }
        }

        // Holding registers: address 40001-49999 → PDU index = address - 40001
        const auto &hr_addrs = addresses_[3];
        const auto &hr_vals = values_[3];
        for (std::size_t i = 0; i < hr_addrs.size() && i < hr_vals.size(); ++i)
        {
            int idx = hr_addrs[i] - 40001;
            if (idx >= 0 && idx < kRangeSize)
            {
                mb->tab_registers[idx] = static_cast<std::uint16_t>(hr_vals[i]);
            }
        }
    }

    // Must be called with mutex_ held.
    // Pulls values from the libmodbus mapping into configured value arrays.
    // This preserves client write effects (FC05/FC06/FC0F/FC10) across requests.
    void SyncMappingToValues(const modbus_mapping_t *mb)
    {
        if (mb == nullptr)
        {
            return;
        }

        // Coils: address 00001-09999 → PDU index = address - 1
        auto &coil_addrs = addresses_[0];
        auto &coil_vals = values_[0];
        for (std::size_t i = 0; i < coil_addrs.size() && i < coil_vals.size(); ++i)
        {
            int idx = coil_addrs[i] - 1;
            if (idx >= 0 && idx < kRangeSize)
            {
                coil_vals[i] = static_cast<int>(mb->tab_bits[idx]);
            }
        }

        // Discrete inputs: address 10001-19999 → PDU index = address - 10001
        auto &di_addrs = addresses_[1];
        auto &di_vals = values_[1];
        for (std::size_t i = 0; i < di_addrs.size() && i < di_vals.size(); ++i)
        {
            int idx = di_addrs[i] - 10001;
            if (idx >= 0 && idx < kRangeSize)
            {
                di_vals[i] = static_cast<int>(mb->tab_input_bits[idx]);
            }
        }

        // Input registers: address 30001-39999 → PDU index = address - 30001
        auto &ir_addrs = addresses_[2];
        auto &ir_vals = values_[2];
        for (std::size_t i = 0; i < ir_addrs.size() && i < ir_vals.size(); ++i)
        {
            int idx = ir_addrs[i] - 30001;
            if (idx >= 0 && idx < kRangeSize)
            {
                ir_vals[i] = static_cast<int>(mb->tab_input_registers[idx]);
            }
        }

        // Holding registers: address 40001-49999 → PDU index = address - 40001
        auto &hr_addrs = addresses_[3];
        auto &hr_vals = values_[3];
        for (std::size_t i = 0; i < hr_addrs.size() && i < hr_vals.size(); ++i)
        {
            int idx = hr_addrs[i] - 40001;
            if (idx >= 0 && idx < kRangeSize)
            {
                hr_vals[i] = static_cast<int>(mb->tab_registers[idx]);
            }
        }
    }

    // Background server thread:
    //   - Opens a TCP socket on ip_:port_
    //   - Allocates a full modbus_mapping_t covering all address ranges
    //   - Accepts one client connection at a time
    //   - Handles Modbus requests with 200 ms select() timeouts (for stoppability)
    //   - On each request, syncs the latest values_ into the mapping before replying
    void ServerLoop()
    {
        std::string ip;
        int port;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            ip = ip_;
            port = static_cast<int>(port_);
        }

        modbus_t *ctx = modbus_new_tcp(ip.c_str(), port);
        if (!ctx)
        {
            std::lock_guard<std::mutex> lk(mutex_);
            running_ = false;
            return;
        }

        // Allocate mapping for full address ranges
        // modbus_mapping_new(nb_bits, nb_input_bits, nb_registers, nb_input_registers)
        modbus_mapping_t *mb_mapping = modbus_mapping_new(kRangeSize, kRangeSize, kRangeSize, kRangeSize);
        if (!mb_mapping)
        {
            modbus_free(ctx);
            std::lock_guard<std::mutex> lk(mutex_);
            running_ = false;
            return;
        }

        // Initial sync of configured values into mapping
        {
            std::lock_guard<std::mutex> lk(mutex_);
            SyncValuesToMapping(mb_mapping);
        }

        int listen_fd = modbus_tcp_listen(ctx, 5);
        if (listen_fd == -1)
        {
            modbus_mapping_free(mb_mapping);
            modbus_free(ctx);
            std::lock_guard<std::mutex> lk(mutex_);
            running_ = false;
            return;
        }

        while (!should_stop_)
        {
            // Wait for an incoming connection with a timeout so we can check should_stop_
            fd_set rdset;
            FD_ZERO(&rdset);
#ifdef _WIN32
            FD_SET(static_cast<SOCKET>(listen_fd), &rdset);
#else
            FD_SET(listen_fd, &rdset);
#endif
            struct timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 200000; // 200 ms

            // On Windows the first argument to select() is ignored
            int rc = select(listen_fd + 1, &rdset, nullptr, nullptr, &tv);
            if (rc <= 0)
            {
                continue; // timeout or benign error – check should_stop_ and retry
            }

            // Accept the client.
            // modbus_tcp_accept() overwrites *tmp with the new client fd and sets ctx->s.
            // We keep listen_fd unchanged by passing a copy.
            int tmp = listen_fd;
            int client_fd = modbus_tcp_accept(ctx, &tmp);
            // Now tmp == client_fd, ctx->s == client_fd, listen_fd is unchanged.
            if (client_fd == -1)
            {
                continue;
            }

            // Serve the connected client
            while (!should_stop_)
            {
                fd_set crdset;
                FD_ZERO(&crdset);
#ifdef _WIN32
                FD_SET(static_cast<SOCKET>(client_fd), &crdset);
#else
                FD_SET(client_fd, &crdset);
#endif
                struct timeval ctv{};
                ctv.tv_sec = 0;
                ctv.tv_usec = 200000;

                int crc = select(client_fd + 1, &crdset, nullptr, nullptr, &ctv);
                if (crc < 0)
                {
                    break; // socket error – disconnect
                }
                if (crc == 0)
                {
                    continue; // timeout – check should_stop_
                }

                std::uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
                int n = modbus_receive(ctx, query); // reads from ctx->s == client_fd
                if (n <= 0)
                {
                    break; // client disconnected or protocol error
                }

                // Sync the latest values into the mapping, then reply
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    SyncValuesToMapping(mb_mapping);
                }
                modbus_reply(ctx, query, n, mb_mapping);

                // Persist write-side changes from mapping into values_ so they are not
                // overwritten by the next SyncValuesToMapping() call.
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    SyncMappingToValues(mb_mapping);
                }
            }

            // Close the client socket (sets ctx->s = -1)
            modbus_close(ctx);
        }

        // Close the listening socket (not managed by modbus_free)
#ifdef _WIN32
        closesocket(static_cast<SOCKET>(listen_fd));
#else
        close(listen_fd);
#endif

        modbus_mapping_free(mb_mapping);
        modbus_free(ctx); // ctx->s is already -1 after modbus_close, so no double-close

        std::lock_guard<std::mutex> lk(mutex_);
        running_ = false;
    }

    mutable std::mutex mutex_;
    std::string ip_;
    std::uint16_t port_;
    bool running_;
    std::atomic<bool> should_stop_;
    std::thread server_thread_;

    std::array<std::vector<int>, 4> addresses_;
    std::array<std::vector<int>, 4> values_;
};

bool TryAddressType(int rawType, AddressType &outType)
{
    if (rawType < static_cast<int>(AddressType::Coil) || rawType > static_cast<int>(AddressType::HoldingRegister))
    {
        return false;
    }

    outType = static_cast<AddressType>(rawType);
    return true;
}
} // namespace

extern "C"
{
    __declspec(dllexport) void *ModbusServer_Create()
    {
        return new (std::nothrow) ModbusServer();
    }

    __declspec(dllexport) void ModbusServer_Destroy(void *handle)
    {
        delete static_cast<ModbusServer *>(handle);
    }

    __declspec(dllexport) int ModbusServer_Start(void *handle)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        return server->Start() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_Stop(void *handle)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        return server->Stop() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_IsRunning(void *handle)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        return server->IsRunning() ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_SetIp(void *handle, const char *ip)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr || ip == nullptr)
        {
            return 0;
        }

        return server->SetIp(ip) ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_GetIp(void *handle, char *buffer, int bufferSize)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr || buffer == nullptr || bufferSize <= 0)
        {
            return 0;
        }

        const std::string ip = server->GetIp();
        const std::size_t copyLength =
            ip.size() < static_cast<std::size_t>(bufferSize - 1) ? ip.size() : static_cast<std::size_t>(bufferSize - 1);

        std::memcpy(buffer, ip.c_str(), copyLength);
        buffer[copyLength] = '\0';
        return 1;
    }

    __declspec(dllexport) int ModbusServer_SetPort(void *handle, std::uint16_t port)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        return server->SetPort(port) ? 1 : 0;
    }

    __declspec(dllexport) std::uint16_t ModbusServer_GetPort(void *handle)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        return server->GetPort();
    }

    __declspec(dllexport) int ModbusServer_AddAddress(void *handle, int type, int address)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        return server->AddAddress(addressType, address) ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_RemoveAddress(void *handle, int type, int index)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        return server->RemoveAddress(addressType, index) ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_GetAddressCount(void *handle, int type)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        return server->GetAddressCount(addressType);
    }

    __declspec(dllexport) int ModbusServer_GetAddressAt(void *handle, int type, int index, int *outAddress)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr || outAddress == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        int value = 0;
        if (!server->GetAddressAt(addressType, index, value))
        {
            return 0;
        }

        *outAddress = value;
        return 1;
    }

    __declspec(dllexport) int ModbusServer_SetAddressAt(void *handle, int type, int index, int address)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        return server->SetAddressAt(addressType, index, address) ? 1 : 0;
    }

    __declspec(dllexport) int ModbusServer_GetValueAt(void *handle, int type, int index, int *outValue)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr || outValue == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        int value = 0;
        if (!server->GetValueAt(addressType, index, value))
        {
            return 0;
        }

        *outValue = value;
        return 1;
    }

    __declspec(dllexport) int ModbusServer_SetValueAt(void *handle, int type, int index, int value)
    {
        auto *server = static_cast<ModbusServer *>(handle);
        if (server == nullptr)
        {
            return 0;
        }

        AddressType addressType;
        if (!TryAddressType(type, addressType))
        {
            return 0;
        }

        return server->SetValueAt(addressType, index, value) ? 1 : 0;
    }
}
