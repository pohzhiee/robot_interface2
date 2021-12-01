#include "robot_interface2/xsens/MTiReceiver.hpp"
#include "raspiHardware/UARTDMAReader.hpp"
#include "robot_interface2/xsens/MTiParser.hpp"
#include "xsdatapacket.h"
#include "xstypes/xsmessage.h"
#include <chrono>

using namespace std::chrono;
namespace robot_interface2
{

MTiReceiver::MTiReceiver(uintptr_t gpioMmapPtr, uintptr_t uartMmapPtr, uintptr_t dmaMmapPtr)
    : imuMessagePub_(std::make_unique<CPublisher<robot_interface::ImuMessage>>("imu")),
      uart_(std::make_unique<UARTDMAReader>(Get_UART<2>(uartMmapPtr), Get_DMA<3>(dmaMmapPtr), 2, gpioMmapPtr, 151,
                                            460800)),
      mtiParser_(std::make_unique<MTiParser>())
{
    uart_->AddCallback([this](auto data) {
        for (auto &c : data)
        {
            mtiParser_->AddByte(c);
        }
    });
    mtiParser_->AddCallback([&](const XsMessage &msg, auto info) { ValidMessageCb(msg); });
    runThread_ = std::thread([&]() { uart_->Run(); });
}
MTiReceiver::~MTiReceiver()
{
    uart_->Stop();
    runThread_.join();
}
void MTiReceiver::ValidMessageCb(const XsMessage &msg)
{
    auto packet = XsDataPacket(&msg);
    assert(packet.containsPacketCounter());
    assert(packet.containsSampleTimeFine());
    assert(packet.containsOrientation());
    assert(packet.containsFreeAcceleration());
    assert(packet.containsCalibratedAcceleration());
    assert(packet.containsCalibratedGyroscopeData());
    auto imuMsg = robot_interface::ImuMessage();
    imuMsg.set_message_id(packet.packetCounter());

    auto orientationPtr = imuMsg.mutable_base_orientation();
    orientationPtr->set_w(packet.orientationQuaternion().w());
    orientationPtr->set_x(packet.orientationQuaternion().x());
    orientationPtr->set_y(packet.orientationQuaternion().y());
    orientationPtr->set_z(packet.orientationQuaternion().z());

    auto angVelPtr = imuMsg.mutable_base_angular_velocity();
    angVelPtr->set_x(packet.calibratedGyroscopeData().at(0));
    angVelPtr->set_y(packet.calibratedGyroscopeData().at(1));
    angVelPtr->set_z(packet.calibratedGyroscopeData().at(2));

    auto linAccPtr = imuMsg.mutable_base_linear_acceleration();
    linAccPtr->set_x(packet.freeAcceleration().at(0));
    linAccPtr->set_y(packet.freeAcceleration().at(1));
    linAccPtr->set_z(packet.freeAcceleration().at(2));

    imuMsg.set_sample_time_ns(static_cast<uint64_t>(packet.sampleTimeFine()) * 100'000);
    imuMsg.set_real_time_ns(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
    imuMessagePub_->Send(imuMsg);
};

} // namespace robot_interface2