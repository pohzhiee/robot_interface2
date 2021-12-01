#ifndef ROBOT_INTERFACE2_MTIRECEIVER_HPP
#define ROBOT_INTERFACE2_MTIRECEIVER_HPP
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_interface_protobuf/imu_message.pb.h>
#include "raspiHardware/SPI.hpp"
#include <thread>
#include <xstypes/xsmessage.h>

namespace robot_interface2{
using eCAL::protobuf::CPublisher;
class UARTDMAReader;
class MTiParser;
class MTiReceiver{
public:
    MTiReceiver(uintptr_t gpioMmapPtr, uintptr_t uartMmapPtr, uintptr_t dmaMmapPtr);
    ~MTiReceiver();
private:
    void Run();
    void ValidMessageCb(const XsMessage &msg);
    std::unique_ptr<CPublisher<robot_interface::ImuMessage>> imuMessagePub_;
    std::unique_ptr<UARTDMAReader> uart_;
    std::unique_ptr<MTiParser> mtiParser_;
    std::thread runThread_;
    uint32_t msgCount_{0};
};
}
#endif //ROBOT_INTERFACE2_MTIRECEIVER_HPP
