// Windows headers must come before libmodbus to avoid Winsock conflicts
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#include <modbus.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <vector>

namespace
{
class ModbusClient
{
  public:
    ModbusClient() : ip_("127.0.0.1"), port_(502), ctx_(nullptr), connected_(false)
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

        ctx_ = modbus_new_tcp(ip_.c_str(), static_cast<int>(port_));
        if (!ctx_)
        {
            return false;
        }

        // 3 second connection timeout
        struct timeval tv{};
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        modbus_set_response_timeout(ctx_, tv.tv_sec, static_cast<std::uint32_t>(tv.tv_usec));

        if (modbus_connect(ctx_) == -1)
        {
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }

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
