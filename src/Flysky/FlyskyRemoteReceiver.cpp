#include "robot_interface2/Flysky/FlyskyRemoteReceiver.hpp"
#include "robot_interface2/Flysky/iBusParser.hpp"
#include "raspiHardware/UARTDMAReader.hpp"
namespace {
robot_interface::FlyskyMessage_SwitchState ToProtoSwitchState(const robot_interface2::Flysky::SwitchState& state)
{
    using robot_interface::FlyskyMessage_SwitchState;
    using robot_interface2::Flysky::SwitchState;
    switch (state) {
    case SwitchState::Up:
        return FlyskyMessage_SwitchState::FlyskyMessage_SwitchState_UP;
    case SwitchState::Middle:
        return FlyskyMessage_SwitchState::FlyskyMessage_SwitchState_MIDDLE;
    case SwitchState::Down:
        return FlyskyMessage_SwitchState::FlyskyMessage_SwitchState_DOWN;
    case SwitchState::Unknown:
        return FlyskyMessage_SwitchState::FlyskyMessage_SwitchState_UNKNOWN;
    }
}

robot_interface::FlyskyMessage FlyskyStateToProto(const robot_interface2::Flysky::RemoteState& state)
{
    auto msg = robot_interface::FlyskyMessage();
    msg.set_channel1(state.Channel1.raw_data);
    msg.set_channel2(state.Channel2.raw_data);
    msg.set_channel3(state.Channel3.raw_data);
    msg.set_channel4(state.Channel4.raw_data);
    msg.set_channel5(state.Channel5.raw_data);
    msg.set_channel6(state.Channel6.raw_data);
    msg.set_switch_a(ToProtoSwitchState(state.SwitchA));
    msg.set_switch_b(ToProtoSwitchState(state.SwitchB));
    msg.set_switch_c(ToProtoSwitchState(state.SwitchC));
    msg.set_switch_d(ToProtoSwitchState(state.SwitchD));
    msg.set_switch_e(ToProtoSwitchState(state.SwitchE));

    return msg;
}
}

namespace robot_interface2 {
FlyskyRemoteReceiver::FlyskyRemoteReceiver(uintptr_t gpioMmapPtr, uintptr_t uartMmapPtr, uintptr_t dmaMmapPtr)
        :
        flyskyMessagePub_(std::make_unique<CPublisher<robot_interface::FlyskyMessage>>("flysky")),
        uart_(std::make_unique<UARTDMAReader>(Get_UART<0>(uartMmapPtr), Get_DMA<6>(dmaMmapPtr), 0, gpioMmapPtr, 32, 115200)),
        iBusParser_(std::make_unique<Flysky::iBusParser>()),
        pin6(gpioMmapPtr), pin16(gpioMmapPtr), pin26(gpioMmapPtr)
{
    UARTDMAReader::printDebugInfo = true;
    uart_->AddCallback([this](auto data) {
        for (auto& c: data) {
            iBusParser_->AddByte(c);
        }
        pin6.Toggle();
    });
    iBusParser_->AddCallback([&](auto state, auto info) {
        auto protoMsg = FlyskyStateToProto(state);
        protoMsg.set_message_id(msgCount_);
        msgCount_++;
        flyskyMessagePub_->Send(protoMsg);
        pin16.Toggle();
    });
    runThread_ = std::thread([&](){pin26.Toggle(); uart_->Run();});
}
FlyskyRemoteReceiver::~FlyskyRemoteReceiver(){
    uart_->Stop();
    runThread_.join();
    pin26.Toggle();
};


}
