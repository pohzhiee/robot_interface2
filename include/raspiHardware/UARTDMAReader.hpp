#ifndef ROBOT_INTERFACE2_UARTDMAREADER_HPP
#define ROBOT_INTERFACE2_UARTDMAREADER_HPP

#include "DMA.hpp"
#include "GPIO.hpp"
#include "UART.hpp"
#include "UncachedMem.hpp"
#include <atomic>
#include <functional>
#include <ostream>

namespace robot_interface2
{
class UARTDMAReader
{
  public:
    UARTDMAReader(UARTRegisters *uart_ptr, DMARegisters *dma_ptr, uint8_t uart_num, uintptr_t gpioMmapPtr,
                  uint16_t dma_len = 256, uint32_t baud = 2e6);
    ~UARTDMAReader();
    using CallbackFn = std::function<void(span<uint8_t>)>;
    void Run();
    void Stop();
    void AddCallback(const CallbackFn &callback);
    static bool printDebugInfo;

  private:
    std::atomic<bool> stop_requested_{false};
    void SetupControlBlocks(uint8_t uart_num, uint16_t dma_len);
    static std::array<uint8_t, 256> ReadBufferData(span<volatile uint32_t> buf);
    static void InitUART(UARTRegisters *uart_ptr, uint32_t baud);
    static void InitUARTGPIO(uintptr_t gpioMmapPtr, uint8_t uartNum);
    UncachedMemBlock control_block_mem_block_;
    UncachedMemBlock data_mem_block_;
    span<DMAControlBlock, Page_Size / sizeof(DMAControlBlock)> con_blocks_{nullptr, 128};
    span<volatile uint32_t, Page_Size / 4> rx_buffer_{nullptr, 1024};
    uint32_t uart_phys_addr_;
    UARTRegisters *uart_ptr_;
    DMARegisters *dma_ptr_;
    uint32_t baud_;
    uint16_t dma_len_;
    std::vector<CallbackFn> callbacks_{};
};
} // namespace robot_interface2

#endif // ROBOT_INTERFACE2_UARTDMAREADER_HPP
