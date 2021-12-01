#include "raspiHardware/DMA.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include "raspiHardware/UART.hpp"
#include "robot_interface2/xsens/MTiReceiver.hpp"
#include <ecal/ecal.h>
#include <iostream>

int main()
{
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 MTi AHRS data publisher");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 MTi AHRS data publisher");

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    uintptr_t uartMmapPtr = mmapPeriph(UART_Base);
    uintptr_t dmaMmapPtr = mmapPeriph(DMA_Base);
    auto a = robot_interface2::MTiReceiver(gpioMmapPtr, uartMmapPtr, dmaMmapPtr);
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}