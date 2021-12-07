#ifndef ROBOT_INTERFACE2_UTILS_MESSAGECONVERSION_HPP
#define ROBOT_INTERFACE2_UTILS_MESSAGECONVERSION_HPP
#include <cstdint>
#include <optional>
#include <robot_interface2/InterfaceData.hpp>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/motor_feedback_msg.pb.h>

namespace robot_interface
{
class MotorCmdMsg;
class MotorCmd;
} // namespace robot_interface
namespace robot_interface2
{
[[nodiscard]] MotorCommandType ProtoCmdTypeToInterfaceCmdType(robot_interface::MotorCmd_CommandType cmdType);

[[nodiscard]] std::optional<std::array<CommandMessage, 2>> ProtobufCmdMsgToControllerCmdMsg(
    const robot_interface::MotorCmdMsg &protoCmd);

[[nodiscard]] robot_interface::MotorFeedbackMsg FeedbackToProto(
    const MotorFeedbackFull &frontFeedback, const MotorFeedbackFull &hindFeedback);

std::pair<ConfigMessage, ConfigMessage> GenerateConfigMessage(MotorConfigType type, span<double, 12> param);

} // namespace robot_interface2

#endif // ROBOT_INTERFACE2_UTILS_MESSAGECONVERSION_HPP
