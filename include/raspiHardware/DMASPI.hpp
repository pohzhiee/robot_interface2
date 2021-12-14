#ifndef ROBOT_INTERFACE2_DMASPI_HPP
#define ROBOT_INTERFACE2_DMASPI_HPP
#include "UncachedMem.hpp"
#include "raspiHardware/DMA.hpp"
#include "raspiHardware/SPI.hpp"

namespace robot_interface2
{

struct DMASPISetting
{
    uint16_t SPI_CLK{40}; // SPI clock prescaler
    uint8_t rxDmaNum;
    uint8_t txDmaNum;
};

class DMASPI : public RpiSPIDriver
{
  public:
    struct ControlBlocks
    {
        DMAControlBlock txDMAConBlock;
        DMAControlBlock rxDMAConBlock;
    };
    /**
     * This constructor assumes that the pointers are obtained by mmap, and it is assumed that mmap is already
     * successful No safety guarantee if the pointer points to an invalid addresses.
     * Use default settings for all SPI settings, only CS0 enabled
     * @param spi_settings struct which contains information about which SPI to be enabled and clock prescaler value to
     * be used
     * @param gpio_ptr pointer to mmap'ed GPIO
     * @param spi_base_ptr pointer to mmap'ed SPI0, with size according to how many SPI to be enabled
     */
    DMASPI(std::array<DMASPISetting, 2> spiSettings, uintptr_t gpioMmapPtr, uintptr_t dmaMmapPtr, uintptr_t spiMmapPtr);
    void AddTransceiveData(uint8_t spiNum, span<const uint8_t> txBuf, span<uint8_t> rxBuf) override;
    void Transceive() override;

  private:
    static ControlBlocks SetupControlBlocks(uint8_t spiNum, uint16_t dma_len, uintptr_t txPhysAddr,
                                            uintptr_t rxPhysAddr);
    static void InitialiseSPI(uint8_t spiNum, uintptr_t gpioMmapPtr, uintptr_t spiMmapPtr, uint32_t CLK);

    std::array<DMASPISetting, 2> spiSettings_{};
    uintptr_t gpioMmapPtr_{};
    uintptr_t dmaMmapPtr_{};
    uintptr_t spiMmapPtr_{};
    std::array<span<uint8_t>, 7> rxBufs_{};
    UncachedMemBlock conBlockMemBlock_{};
    span<DMAControlBlock, Page_Size / sizeof(DMAControlBlock)> conBlocks_{nullptr, Page_Size / sizeof(DMAControlBlock)};
    span<volatile uint32_t, Page_Size / 4> txBuffer_{nullptr, Page_Size / 4}, rxBuffer_{nullptr, Page_Size / 4};
};
} // namespace robot_interface2
#endif // ROBOT_INTERFACE2_DMASPI_HPP
