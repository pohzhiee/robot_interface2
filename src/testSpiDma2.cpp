#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include <iostream>
#include <memory>

using namespace robot_interface2;

int main()
{

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    uintptr_t dmaMmapPtr = mmapPeriph(DMA_Base);
    uintptr_t spiMmapPtr = mmapPeriph(SPI_Base);
    if (gpioMmapPtr == 0)
    {
        std::cerr << "Failed to mmap gpio" << std::endl;
        return -1;
    }

    SetAF<8>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<9>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<10>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<11>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);

    SetAF<18>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<19>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<20>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<21>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);

    std::array<uint8_t, 108> txBuf, rxBuf{};
    std::array<uint8_t, 108> txBuf2, rxBuf2{};
    for (int i = 0; i < 108; i++)
    {
        txBuf.at(i) = 10 + i;
        txBuf2.at(i) = 128 + i;
    }
    std::array<DMASPISetting, 2> spiSetting = {DMASPISetting{.SPI_CLK = 40, .rxDmaNum = 0, .txDmaNum = 1},
                                               DMASPISetting{.SPI_CLK = 40, .rxDmaNum = 8, .txDmaNum = 9}};
    auto dmaSpi = std::make_shared<DMASPI>(spiSetting, gpioMmapPtr, dmaMmapPtr, spiMmapPtr);
    dmaSpi->AddTransceiveData(0, {txBuf.data(), txBuf.size()}, {rxBuf.data(), rxBuf.size()});
    dmaSpi->AddTransceiveData(6, {txBuf2.data(), txBuf2.size()}, {rxBuf2.data(), rxBuf2.size()});
    std::cerr << "Starting transceive" << std::endl;
    dmaSpi->Transceive();
//    for (int i = 0; i < 108; i++)
//    {
//        std::cout << std::dec << static_cast<uint32_t>(rxBuf.at(i)) << ' ';
//    }
    std::cout << std::endl;
}