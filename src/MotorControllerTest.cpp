#include "robot_interface2/motor/MotorController.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include <chrono>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <iostream>
#include <thread>
using namespace std::chrono;
using namespace std::chrono_literals;

using namespace eCAL::protobuf;

int main()
{
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Motor Controller Test node");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Motor Controller Test node");

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    if (gpioMmapPtr == 0)
        return 1;
    auto pin6 = GPIO_Output<6>(gpioMmapPtr);
    auto pin16 = GPIO_Output<16>(gpioMmapPtr);
    auto pin26 = GPIO_Output<26>(gpioMmapPtr);
    auto pub = std::make_unique<CPublisher<robot_interface::MotorCmdMsg>>("motor_cmd");
    auto sub = std::make_unique<CSubscriber<robot_interface::MotorFeedbackMsg>>("motor_feedback");
    auto subFunc = [&](auto *topicName, const auto &msg, auto time, auto clock, auto id) { pin6.Toggle(); };
    auto next = std::chrono::steady_clock::now() + 5ms;
    robot_interface::MotorCmdMsg msg;
    for (int i = 0; i < 3; i++)
    {
        auto cmdPtr = msg.mutable_commands()->Add();
        cmdPtr->set_motor_id(i);
        cmdPtr->set_command(robot_interface::MotorCmd_CommandType_TORQUE);
        cmdPtr->set_parameter(0.0);
    }
    sub->AddReceiveCallback(subFunc);
    eCAL::Process::SleepMS(2000);

    for (int i = 0; i < 8000; i++)
    {
        if (steady_clock::now() > next)
        {
            std::cout << "Timing delayed on tx " << i << std::endl;
            next = steady_clock::now();
        }
        std::this_thread::sleep_until(next);
        next = next + duration<int64_t, std::ratio<1, 800>>{1};
        msg.set_message_id(i + 1000);
        pin16.Toggle();
        pub->Send(msg);
        pin26.Toggle();
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}