#ifndef ROBOT_INTERFACE2_FLYSKYREMOTERECEIVER_HPP
#define ROBOT_INTERFACE2_FLYSKYREMOTERECEIVER_HPP
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_interface_protobuf/flysky_message.pb.h>
#include <raspiHardware/GPIO.hpp>
#include <thread>

namespace robot_interface2{
namespace Flysky{
class iBusParser;
}
class UARTDMAReader;
using eCAL::protobuf::CSubscriber;
using eCAL::protobuf::CPublisher;
class FlyskyRemoteReceiver{
public:
    FlyskyRemoteReceiver(uintptr_t gpioMmapPtr, uintptr_t uartMmapPtr, uintptr_t dmaMmapPtr);
    ~FlyskyRemoteReceiver();
private:
    void Run();
    std::unique_ptr<CPublisher<robot_interface::FlyskyMessage>> flyskyMessagePub_;
    std::unique_ptr<UARTDMAReader> uart_;
    std::unique_ptr<Flysky::iBusParser> iBusParser_;
    GPIO_Output<6> pin6;
    GPIO_Output<16> pin16;
    GPIO_Output<26> pin26;
    std::thread runThread_;
    uint32_t msgCount_{0};
};
}
#endif //ROBOT_INTERFACE2_FLYSKYREMOTERECEIVER_HPP
