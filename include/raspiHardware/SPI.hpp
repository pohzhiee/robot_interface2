#ifndef ROBOT_INTERFACE2_SPI_HPP
#define ROBOT_INTERFACE2_SPI_HPP
#include "Common.hpp"
#include <array>
#include <cstdint>
#include <vector>

constexpr uint32_t SPI_Offset = 0x204000;
constexpr uint32_t SPI_Periph_Width = 0x200;
constexpr uint32_t SPI_Base = Peripheral_Base + SPI_Offset;

struct SPIRegisters
{
    volatile uint32_t CS;
    volatile uint32_t FIFO;
    volatile uint32_t CLK;
    volatile uint32_t DLEN;
    volatile uint32_t LTOH;
    volatile uint32_t DC;
};
static_assert(sizeof(SPIRegisters) == 24);

template <uint64_t N> constexpr SPIRegisters *GetSPI(uintptr_t address)
{
    static_assert(N >= 0);
    static_assert(N <= 6);
    return reinterpret_cast<SPIRegisters *>(address + N * SPI_Periph_Width);
}

inline SPIRegisters *GetSPI(uintptr_t address, uint8_t spiNum)
{
    return reinterpret_cast<SPIRegisters *>(address + spiNum * SPI_Periph_Width);
}

struct SPISettings
{
    uint16_t SPI0_CLK{50};                // SPI0 clock prescaler
    uint32_t SPI0_CSDefault{0b11u << 4u}; // SPI0 default CS register value
    uint16_t SPI3_CLK{50};                // SPI3 clock prescaler
    uint32_t SPI3_CSDefault{0b11u << 4u}; // SPI3 default CS register value
    uint16_t SPI4_CLK{50};                // SPI4 clock prescaler
    uint32_t SPI4_CSDefault{0b11u << 4u}; // SPI4 default CS register value
    uint16_t SPI5_CLK{50};                // SPI5 clock prescaler
    uint32_t SPI5_CSDefault{0b11u << 4u}; // SPI5 default CS register value
    uint16_t SPI6_CLK{50};                // SPI6 clock prescaler
    uint32_t SPI6_CSDefault{0b11u << 4u}; // SPI6 default CS register value
};

class RpiSPIDriver
{
  public:
    virtual void AddTransceiveData(uint8_t spiNum, span<const uint8_t> txBuf, span<uint8_t> rxBuf) = 0;
    virtual void Transceive() = 0;
};
#endif // ROBOT_INTERFACE2_SPI_HPP
