#include <iostream>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "robot_interface2/motor/MotorController.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/SPI.hpp"
#include "raspiHardware/Mmap.hpp"

using namespace eCAL::protobuf;

int main()
{
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Motor Controller");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Motor Controller");

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    uintptr_t spiMmapPtr = mmapPeriph(SPI_Base);
    auto a = robot_interface2::MotorController(gpioMmapPtr, spiMmapPtr);
    while (eCAL::Ok()) {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}