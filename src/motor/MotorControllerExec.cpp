#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include "raspiHardware/SPI.hpp"
#include "raspiHardware/SimultaneousSPI.hpp"
#include "robot_interface2/motor/MotorController.hpp"
#include <ecal/ecal.h>
#include <iostream>

using namespace robot_interface2;
int main()
{
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Motor Controller");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Motor Controller");

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    uintptr_t dmaMmapPtr = mmapPeriph(DMA_Base);
    uintptr_t spiMmapPtr = mmapPeriph(SPI_Base);
    if (gpioMmapPtr == 0)
    {
        std::cerr << "Failed to mmap gpio" << std::endl;
        return -1;
    }
    auto simulSpi = std::make_shared<SimultaneousSPI>(SPISettings{}, gpioMmapPtr, spiMmapPtr);
    std::array<DMASPISetting, 2> spiSetting = {DMASPISetting{.SPI_CLK = 20, .rxDmaNum = 7, .txDmaNum = 8},
                                               DMASPISetting{.SPI_CLK = 20, .rxDmaNum = 9, .txDmaNum = 10}};
    auto dmaSpi = std::make_shared<DMASPI>(spiSetting, gpioMmapPtr, dmaMmapPtr, spiMmapPtr);
    auto a = robot_interface2::MotorController(std::move(dmaSpi));
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}