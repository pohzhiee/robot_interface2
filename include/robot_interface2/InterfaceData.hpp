#ifndef ROBOTINTERFACE_INTERFACEDATA_HPP
#define ROBOTINTERFACE_INTERFACEDATA_HPP

#include "robot_interface2/span.hpp"
using tcb::span;

inline uint32_t Crc32Fast(uint32_t Crc, uint32_t Data)
{
    static const uint32_t CrcTable[16] = {// Nibble lookup table for 0x04C11DB7 polynomial
            0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
            0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
            0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD};

    Crc = Crc ^ Data; // Apply all 32-bits

    // Process 32-bits, 4 at a time, or 8 rounds

    Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; // Assumes 32-bit reg, masking index to 4-bits
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; //  0x04C11DB7 Polynomial used in STM32
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];

    return (Crc);
}

inline uint32_t GetCRC32(span<const uint32_t> data)
{
    uint32_t crc_val = 0xFFFFFFFF;
    for (uint32_t i = 0; i < data.size(); i++)
    {
        crc_val = Crc32Fast(crc_val, data[i]);
    }
    return crc_val;
}

enum class MotorCommandType : uint16_t
{
    Ignore = 0,
    Position = 0b1,
    Velocity = 0b11,
    Torque = 0b111,
    Read = 0b1111,
    SetZero = 0b11111
};

struct MotorCommandSingle
{
    // if we change Param to a double the padding would be different leading to bigger than expected size
    MotorCommandType CommandType;
    uint16_t reserved;
    std::array<uint8_t, 8> Param;
    uint32_t reserved2;
};
static_assert(sizeof(MotorCommandSingle) == 16);

enum class MessageType : uint16_t
{
    Invalid = 0,
    CommandMessage = 0xAA | (0x11 << 8),
    ConfigMessage = 0xAB | (0x33 << 8)
};

struct CommandMessage
{
    MessageType messageType;
    uint16_t reserved;
    uint32_t MessageId;
    std::array<MotorCommandSingle, 6> MotorCommands;
    mutable uint32_t CRC32;

    void GenerateCRC() const
    {
        uint32_t CRCCalc = GetCRC32(
            tcb::span<const uint32_t>(reinterpret_cast<uint32_t const *>(this), sizeof(CommandMessage) / 4 - 1));
        this->CRC32 = CRCCalc;
    }
};
static_assert(sizeof(CommandMessage) == 108);

enum class MotorConfigType : uint16_t
{ // The values of this enum is made to not overlap with the MotorCommandType enum so that we can identify when it is
  // wrong
  // Only MotorCommandType::Ignore and MotorConfigType::Nothing share the same value of 0 as default when 0 initialised
    Nothing = 0b0,
    GearRatio = 0b110, // Param for this config can be negative value to indicate flip in direction
    IgnoreLimits = 0b1100,
    LowerJointLimit = 0b11100,
    UpperJointLimit = 0b111100,
    PositionModeSpeed = 0b1111100,
    SetTorqueMultiplier = 0b11111100,
    JointAngleEstimate = 0b111111100
};

enum class ConsiderLimits : uint64_t
{
    Ignore = 0b1010101,
    Consider = 0b1111111 // technically we don't need this, we just need to check for ignore
};

struct MotorConfigSingle
{
    MotorConfigType ConfigType;
    uint16_t Reserved;
    std::array<uint8_t, 8> Data;
    std::array<uint8_t, 4> Reserved2;
};
static_assert(sizeof(MotorConfigSingle) == 16);

struct ConfigMessage
{
    MessageType messageType;
    uint16_t Reserved;
    uint32_t MessageId;
    std::array<MotorConfigSingle, 6> MotorConfigs;
    mutable uint32_t CRC32;

    void GenerateCRC() const
    {
        uint32_t CRCCalc = GetCRC32(
            tcb::span<const uint32_t>(reinterpret_cast<uint32_t const *>(this), sizeof(ConfigMessage) / 4 - 1));
        this->CRC32 = CRCCalc;
    }
};
static_assert(sizeof(ConfigMessage) == 108);

struct MotorFeedbackSingle
{
    float Torque{};         // motor raw torque value, from -33 to 33
    float Angle{};          // in output rad
    float Velocity{};       // in output rad/s
    uint16_t Temperature{}; // in deg C
    bool Ready{false};
    uint8_t Reserved{};
};
static_assert(sizeof(MotorFeedbackSingle) == 16);

struct MotorFeedbackStatus
{
    bool Motor1Ready : 1;
    bool Motor2Ready : 1;
    bool Motor3Ready : 1;
    bool Motor4Ready : 1;
    bool Motor5Ready : 1;
    bool Motor6Ready : 1;
    bool IsError : 1;
    bool IsIdle : 1;
    uint8_t ErrorCode : 8;
};
static_assert(sizeof(MotorFeedbackStatus) == 2);

struct MotorFeedbackFull
{
    uint8_t Header1 = 0xBB;
    uint8_t Header2 = 0x11;
    MotorFeedbackStatus status{};
    uint32_t MessageId{0};
    std::array<MotorFeedbackSingle, 6> Feedbacks{};
    mutable uint32_t CRC32{};

    [[nodiscard]] bool CheckCRC() const
    {
        uint32_t CRCCalc = GetCRC32(
            span<const uint32_t>(reinterpret_cast<uint32_t const *>(this), sizeof(MotorFeedbackFull) / 4 - 1));
        return CRCCalc == this->CRC32;
    }

    void GenerateCRC() const
    {
        uint32_t CRCCalc = GetCRC32(
            span<const uint32_t>(reinterpret_cast<uint32_t const *>(this), sizeof(MotorFeedbackFull) / 4 - 1));
        this->CRC32 = CRCCalc;
    }
};
static_assert(sizeof(MotorFeedbackFull) == 108);

struct MotorFeedbackRaw
{
    uint8_t Command : 8;
    uint8_t Temperature : 8;
    int16_t TorqueCurrentRaw : 16;
    int16_t Speed : 16; // in deg/s
    uint16_t EncoderPos : 16;
};
static_assert(sizeof(MotorFeedbackRaw) == 8);
#endif // ROBOTINTERFACE_INTERFACEDATA_HPP
