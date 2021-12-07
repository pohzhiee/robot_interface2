#include "robot_interface2/utils/MessageConversion.hpp"
#include "robot_interface2/utils/Common.hpp"
#include <cstring>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>

namespace robot_interface2
{

MotorCommandSingle ProtobufCmdToControllerCmd(const robot_interface::MotorCmd &cmd)
{
    return MotorCommandSingle{.CommandType = ProtoCmdTypeToInterfaceCmdType(cmd.command()),
                              .reserved = {},
                              .Param = BytesFrom<double>(cmd.parameter()),
                              .reserved2 = 0};
}

std::optional<std::array<CommandMessage, 2>> ProtobufCmdMsgToControllerCmdMsg(
    const robot_interface::MotorCmdMsg &protoCmd)
{
    std::array<MotorCommandSingle, 12> motorCommandSingles{};
    for (const auto &cmd : protoCmd.commands())
    {
        auto index = cmd.motor_id();
        if (index >= 12)
        {
            std::cerr << "Invalid motor index received in motor command, index: " << index << std::endl;
            return std::nullopt;
        }
        motorCommandSingles.at(index) = ProtobufCmdToControllerCmd(cmd);
    }
    std::array<CommandMessage, 2> cmdMsgs{
        CommandMessage{.messageType = MessageType::CommandMessage, .reserved = {}, .MessageId = protoCmd.message_id()},
        CommandMessage{.messageType = MessageType::CommandMessage, .reserved = {}, .MessageId = protoCmd.message_id()}};

    std::memcpy(cmdMsgs.at(0).MotorCommands.data(), motorCommandSingles.data(), sizeof(MotorCommandSingle) * 6);
    std::memcpy(cmdMsgs.at(1).MotorCommands.data(), motorCommandSingles.data() + 6, sizeof(MotorCommandSingle) * 6);
    cmdMsgs.at(0).GenerateCRC();
    cmdMsgs.at(1).GenerateCRC();
    return cmdMsgs;
}
MotorCommandType ProtoCmdTypeToInterfaceCmdType(robot_interface::MotorCmd_CommandType cmdType)
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

robot_interface::MotorFeedbackMsg FeedbackToProto(const MotorFeedbackFull &frontFeedback,
                                                  const MotorFeedbackFull &hindFeedback)
{
    std::array<MotorFeedbackSingle, 12> feedbacks;
    robot_interface::MotorFeedbackMsg feedbackMsg;
    std::memcpy(feedbacks.data(), frontFeedback.Feedbacks.data(), sizeof(MotorFeedbackSingle) * 6);
    std::memcpy(feedbacks.data() + 6, hindFeedback.Feedbacks.data(), sizeof(MotorFeedbackSingle) * 6);
    for (int i = 0; i < 12; i++)
    {
        auto feedbackPtr = feedbackMsg.mutable_feedbacks()->Add();
        feedbackPtr->set_motor_id(i);
        feedbackPtr->set_angle(feedbacks.at(i).Angle);
        feedbackPtr->set_velocity(feedbacks.at(i).Velocity);
        feedbackPtr->set_torque(feedbacks.at(i).Torque);
        feedbackPtr->set_temperature(feedbacks.at(i).Temperature);
        feedbackPtr->set_ready(feedbacks.at(i).Ready);
    }
    return feedbackMsg;
}
std::pair<ConfigMessage, ConfigMessage> GenerateConfigMessage(MotorConfigType type, span<double, 12> param)
{
    ConfigMessage frontConfigmsg{.messageType = MessageType::ConfigMessage, .MessageId = 10101},
        hindConfigMsg{.messageType = MessageType::ConfigMessage, .MessageId = 10101};
    for (size_t i = 0; i < 6; i++)
    {
        frontConfigmsg.MotorConfigs.at(i) = MotorConfigSingle{.ConfigType = type, .Data = BytesFrom(param[i])};
        hindConfigMsg.MotorConfigs.at(i) = MotorConfigSingle{.ConfigType = type, .Data = BytesFrom(param[i + 6])};
    }
    frontConfigmsg.GenerateCRC();
    hindConfigMsg.GenerateCRC();
    return {frontConfigmsg, hindConfigMsg};
}
} // namespace robot_interface2