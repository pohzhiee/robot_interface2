#include "robot_interface2/motor/MotorController.hpp"
#include "raspiHardware/SimultaneousSPI.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include <cstring>
namespace
{
MotorCommandType ProtoMotorCmdToInterface(robot_interface::MotorCmd_CommandType cmdType)
{
    using robot_interface::MotorCmd_CommandType;
    switch (cmdType)
    {
    case MotorCmd_CommandType::MotorCmd_CommandType_IGNORE:
        return MotorCommandType::Ignore;
    case MotorCmd_CommandType::MotorCmd_CommandType_POSITION:
        return MotorCommandType::Position;
    case MotorCmd_CommandType::MotorCmd_CommandType_VELOCITY:
        return MotorCommandType::Velocity;
    case MotorCmd_CommandType::MotorCmd_CommandType_TORQUE:
        return MotorCommandType::Torque;
    case MotorCmd_CommandType::MotorCmd_CommandType_READ:
        return MotorCommandType::Read;
    default:
        throw std::runtime_error("Invalid protobuf motor command received");
    }
}

template <typename T> inline std::array<uint8_t, sizeof(T)> BytesFrom(T input)
{
    std::array<uint8_t, sizeof(T)> temp;
    std::memcpy(temp.data(), &input, sizeof(T));
    return temp;
}

} // namespace

namespace robot_interface2
{

MotorController::MotorController(uintptr_t gpioMmapPtr, uintptr_t spiMmapPtr)
    : motorCommandSub_(std::make_unique<CSubscriber<robot_interface::MotorCmdMsg>>("motor_cmd")),
      motorFeedbackPub_(std::make_unique<CPublisher<robot_interface::MotorFeedbackMsg>>("motor_feedback")),
      spi_(std::make_unique<SimultaneousSPI>(SPISettings{}, gpioMmapPtr, spiMmapPtr))
{
    auto a = [this](auto *topicName, const auto &msg, auto time, auto clock, auto id) {
        OnCmdMsg(topicName, msg, time, clock);
    };
    motorCommandSub_->AddReceiveCallback(a);
}
void MotorController::OnCmdMsg(const char *topic_name_, const robot_interface::MotorCmdMsg &msg, long long int time_,
                               long long int clock_)
{
    // Create message
    std::array<MotorCommandSingle, 12> motorCommandSingles{};
    for (const auto &cmd : msg.commands())
    {
        auto index = cmd.motor_id();
        if (index >= 12)
        {
            std::cerr << "Invalid motor index received in motor command, index: " << index << std::endl;
            continue;
        }
        assert(index < 12);
        motorCommandSingles.at(index) = MotorCommandSingle{.CommandType = ProtoMotorCmdToInterface(cmd.command()),
                                                           .Param = BytesFrom<double>(cmd.parameter())};
    }

    auto hasHind = std::any_of(msg.commands().cbegin(), msg.commands().cend(),
                               [](const auto &cmd) { return cmd.motor_id() > 5 && cmd.motor_id() <= 11; });
    auto hasFront = std::any_of(msg.commands().cbegin(), msg.commands().cend(),
                                [](const auto &cmd) { return cmd.motor_id() <= 5; });
    // Move all relevant data into the message, if not dirty we use the ignore command
    MotorFeedbackFull frontFeedback, hindFeedback;
    if (hasFront)
    {
        CommandMessage message{
            .messageType = MessageType::CommandMessage,
            .MessageId = msg.message_id(),
        };
        std::memcpy(message.MotorCommands.data(), motorCommandSingles.data(), sizeof(MotorCommandSingle) * 6);
        message.GenerateCRC();
        spi_->AddTransceiveData(0, span<uint8_t>(reinterpret_cast<uint8_t *>(&message), sizeof(message)),
                                span<uint8_t>(reinterpret_cast<uint8_t *>(&frontFeedback), sizeof(frontFeedback)));
    }
    if (hasHind)
    {
        CommandMessage message{
            .messageType = MessageType::CommandMessage,
            .MessageId = msg.message_id(),
        };
        std::memcpy(message.MotorCommands.data(), motorCommandSingles.data() + 6, sizeof(MotorCommandSingle) * 6);
        message.GenerateCRC();
        spi_->AddTransceiveData(6, span<uint8_t>(reinterpret_cast<uint8_t *>(&message), sizeof(message)),
                                span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
    }
    spi_->Transceive();
    std::array<MotorFeedbackSingle, 12> feedbacks;
    robot_interface::MotorFeedbackMsg feedbackMsg;
    if (hasFront)
    {
        std::memcpy(feedbacks.data(), frontFeedback.Feedbacks.data(), sizeof(MotorFeedbackSingle) * 6);
        for (int i = 0; i < 6; i++)
        {
            auto feedbackPtr = feedbackMsg.mutable_feedbacks()->Add();
            ;
            feedbackPtr->set_motor_id(i);
            feedbackPtr->set_angle(feedbacks.at(i).Angle);
            feedbackPtr->set_velocity(feedbacks.at(i).Velocity);
            feedbackPtr->set_torque(feedbacks.at(i).Torque);
            feedbackPtr->set_temperature(feedbacks.at(i).Temperature);
            feedbackPtr->set_ready(feedbacks.at(i).Ready);
        }
    }
    if (hasHind)
    {
        std::memcpy(feedbacks.data() + 6, hindFeedback.Feedbacks.data(), sizeof(MotorFeedbackSingle) * 6);
        for (int i = 6; i < 12; i++)
        {
            auto feedbackPtr = feedbackMsg.mutable_feedbacks()->Add();
            ;
            feedbackPtr->set_motor_id(i);
            feedbackPtr->set_angle(feedbacks.at(i).Angle);
            feedbackPtr->set_velocity(feedbacks.at(i).Velocity);
            feedbackPtr->set_torque(feedbacks.at(i).Torque);
            feedbackPtr->set_temperature(feedbacks.at(i).Temperature);
            feedbackPtr->set_ready(feedbacks.at(i).Ready);
        }
    }
    motorFeedbackPub_->Send(feedbackMsg);
}

} // namespace robot_interface2