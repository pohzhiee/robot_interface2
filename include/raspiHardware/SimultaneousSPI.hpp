#ifndef ROBOTINTERFACE_SIMULTANEOUSSPI_HPP
#define ROBOTINTERFACE_SIMULTANEOUSSPI_HPP
#include "raspiHardware/SPI.hpp"
class SimultaneousSPI : public RpiSPIDriver
{
  public:
    /**
     * This constructor assumes that the pointers are obtained by mmap, and it is assumed that mmap is already
     * successful No safety guarantee if the pointer points to an invalid addresses.
     * Use default settings for all SPI settings, only CS0 enabled
     * @param spi_settings struct which contains information about which SPI to be enabled and clock prescaler value to
     * be used
     * @param gpio_ptr pointer to mmap'ed GPIO
     * @param spi_base_ptr pointer to mmap'ed SPI0, with size according to how many SPI to be enabled
     */
    SimultaneousSPI(SPISettings spiSettings, uintptr_t gpioPtr, uintptr_t spiBasePtr);
    void AddTransceiveData(uint8_t spiNum, span<const uint8_t> txBuf, span<uint8_t> rxBuf) override;
    void Transceive() override;

  private:
    void CheckAndInitialiseSPI(uint8_t spiNum);

    std::array<SPIRegisters *, 7> spiPtrs_{};
    SPISettings settings_{};
    std::array<uint32_t, 7> CSDefaults_{};
    uintptr_t gpioBasePtr_{};
    uintptr_t spiBasePtr_{};
    std::vector<uint8_t> transceiveIndex{};
    std::array<span<const uint8_t>, 7> txBufs_;
    std::array<span<uint8_t>, 7> rxBufs_;
};
#endif // ROBOTINTERFACE_SIMULTANEOUSSPI_HPP
