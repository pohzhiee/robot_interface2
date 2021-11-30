#ifndef ROBOT_INTERFACE2_UART_HPP
#define ROBOT_INTERFACE2_UART_HPP
#include "Common.hpp"

constexpr uint32_t UART_Offset = 0x201000;
constexpr uint32_t UART_Periph_Width = 0x200;
constexpr uint32_t UART_Base = Peripheral_Base + UART_Offset;

struct UARTRegisters
{
    volatile uint32_t DR; // data register
    volatile uint32_t RSRECR;
    uint32_t : 32;
    uint32_t : 32;
    uint32_t : 32;
    uint32_t : 32;
    volatile uint32_t FR; // flag register
    uint32_t : 32;
    uint32_t : 32;
    volatile uint32_t IBRD;  // integer baud rate divisor
    volatile uint32_t FBRD;  // fractional baud rate divisor
    volatile uint32_t LCRH;  // Line control register
    volatile uint32_t CR;    // control register
    volatile uint32_t IFLS;  // Interrupt FIFO Level Select Register
    volatile uint32_t IMSC;  // Interrupt Mask Set Clear Register
    volatile uint32_t RIS;   // Raw Interrupt Status Register
    volatile uint32_t MIS;   // Masked Interrupt Status Register
    volatile uint32_t ICR;   // Interrupt Clear Register
    volatile uint32_t DMACR; // DMA Control Register, 0x48 offset
};
static_assert(sizeof(UARTRegisters) == 0x4C);

struct UART_FR_Reg
{
    bool ClearToSend : 1;
    uint8_t : 2;
    bool Busy : 1;
    bool RxFifoEmpty : 1;
    bool TxFifoFull : 1;
    bool RxFifoFull : 1;
    bool TxFifoEmpty : 1;
    uint32_t : 24;
};
static_assert(sizeof(UART_FR_Reg) == sizeof(uint32_t));

template <unsigned N> constexpr UARTRegisters *Get_UART(uintptr_t addr)
{
    static_assert(N >= 0);
    static_assert(N < 6);
    return reinterpret_cast<UARTRegisters *>(addr + N * UART_Periph_Width);
}

#endif // ROBOT_INTERFACE2_UART_HPP
