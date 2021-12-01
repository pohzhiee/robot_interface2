#ifndef ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#define ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#include "raspiHardware/SPI.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/motor_feedback_msg.pb.h>
namespace robot_interface2
{
using eCAL::protobuf::CPublisher;
using eCAL::protobuf::CSubscriber;
class MotorController
{
  public:
    MotorController(uintptr_t gpioMmapPtr, uintptr_t spiMmapPtr);
    std::unique_ptr<CSubscriber<robot_interface::MotorCmdMsg>> motorCommandSub_;
    std::unique_ptr<CPublisher<robot_interface::MotorFeedbackMsg>> motorFeedbackPub_;
    void OnCmdMsg(const char *topic_name_, const robot_interface::MotorCmdMsg &msg, long long time_, long long clock_);
    std::unique_ptr<RpiSPIDriver> spi_;
};
} // namespace robot_interface2
#endif // ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
