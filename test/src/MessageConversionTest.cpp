#include "robot_interface2/utils/MessageConversion.hpp"
#include "robot_interface2/utils/Common.hpp"
#include "gtest/gtest.h"
#include <random>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <string>

using namespace robot_interface2;
struct RandomMotorCmdMsg
{
    robot_interface::MotorCmdMsg protoMsg;
    std::array<CommandMessage, 2> cmdMsg;
    std::array<double, 12> cmdValues;
    std::array<robot_interface::MotorCmd_CommandType, 12> cmdTypes{};
};

namespace
{
robot_interface::MotorCmdMsg GenerateMotorCmdProto(uint32_t msgId, std::array<int, 12> indexes,
                                                   std::array<double, 12> cmdVal,
                                                   std::array<robot_interface::MotorCmd_CommandType, 12> cmdType)
{
    robot_interface::MotorCmdMsg protoMsg;
    protoMsg.set_message_id(msgId);
    auto cmdPtr = protoMsg.mutable_commands();
    //    std::array<int, 12> indexes{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    //    std::array<double, 12> cmd{1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
    for (size_t i = 0; i < 12; i++)
    {
        auto cmd = cmdPtr->Add();
        auto id = indexes.at(i);
        cmd->set_motor_id(id);
        cmd->set_command(cmdType.at(i));
        cmd->set_parameter(cmdVal.at(i));
    }
    return protoMsg;
}

RandomMotorCmdMsg GenerateRandomMotorCmdProto()
{
    RandomMotorCmdMsg msgInfo;
    using robot_interface::MotorCmd_CommandType;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> doubleDist;
    std::uniform_int_distribution<int> intDist;
    std::uniform_int_distribution<uint32_t> uintDist;
    std::array<int, 12> indexes{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    std::array<robot_interface::MotorCmd_CommandType, 12> cmdTypes{};
    std::random_shuffle(indexes.begin(), indexes.end());
    std::random_shuffle(cmdTypes.begin(), cmdTypes.end());
    std::array<double, 12> cmdValues{};
    for (int i = 0; i < 12; i++)
    {
        cmdValues.at(i) = std::fmod(doubleDist(gen), 10.0);
        auto cmdTypeVal = intDist(gen) % 5;
        cmdTypes.at(i) = static_cast<robot_interface::MotorCmd_CommandType>(cmdTypeVal);
        assert(robot_interface::MotorCmd_CommandType_IsValid(cmdTypeVal));
    }
    auto protoMsgId = uintDist(gen);
    msgInfo.protoMsg = GenerateMotorCmdProto(protoMsgId, indexes, cmdValues, cmdTypes);

    std::array<MotorCommandSingle, 12> interfaceCmdSingle{};
    for (int i = 0; i < 12; i++)
    {
        auto index = indexes.at(i);
        MotorCmd_CommandType asd = cmdTypes.at(index);
        auto cmdType = (asd);
        interfaceCmdSingle.at(index) = MotorCommandSingle{.CommandType = ProtoCmdTypeToInterfaceCmdType(cmdTypes.at(i)),
                                                          .reserved = {},
                                                          .Param = BytesFrom<double>(cmdValues.at(i)),
                                                          .reserved2 = {}};
        msgInfo.cmdValues.at(index) = cmdValues.at(i);
        msgInfo.cmdTypes.at(index) = cmdTypes.at(i);
    }
    CommandMessage msg1{.messageType = MessageType::CommandMessage,
                        .reserved = {},
                        .MessageId = protoMsgId,
                        .MotorCommands = {},
                        .CRC32 = {}},
        msg2{.messageType = MessageType::CommandMessage,
             .reserved = {},
             .MessageId = protoMsgId,
             .MotorCommands = {},
             .CRC32 = {}};
    for (int i = 0; i < 6; i++)
    {
        msg1.MotorCommands.at(i) = interfaceCmdSingle.at(i);
        msg2.MotorCommands.at(i) = interfaceCmdSingle.at(i + 6);
    }
    msg1.GenerateCRC();
    msg2.GenerateCRC();
    msgInfo.cmdMsg = {msg1, msg2};
    return msgInfo;
}

TEST(MessageConversionTest, ProtoCmdMsgToCmdmsg)
{
    // Here we test out whether the conversion done using 2 different methods are the same
    // If they are the same that should mean they are both correct, very hard to get both equally wrong
    // One method is coded in this test, the other method is the one that we are going to test
    for (int i = 0; i < 100; i++)
    {
        auto randomMsg = GenerateRandomMotorCmdProto();
        auto cmdMsgs = ProtobufCmdMsgToControllerCmdMsg(randomMsg.protoMsg);
        ASSERT_EQ(cmdMsgs.has_value(), true);
        auto cmdMsgsDesired = randomMsg.cmdMsg;
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                double calc = FromBytes<double>(cmdMsgs->at(k).MotorCommands.at(j).Param);
                double ori2 = FromBytes<double>(cmdMsgsDesired.at(k).MotorCommands.at(j).Param);
                double ori = randomMsg.cmdValues.at(j + k * 6);
                ASSERT_EQ(calc, ori);
                ASSERT_EQ(ori2, ori);
            }
            for (int k = 0; k < 2; k++)
            {
                auto calc = (cmdMsgs->at(k).MotorCommands.at(j).CommandType);
                auto ori2 = (cmdMsgsDesired.at(k).MotorCommands.at(j).CommandType);
                auto ori = ProtoCmdTypeToInterfaceCmdType(randomMsg.cmdTypes.at(j + k * 6));
                ASSERT_EQ(calc, ori);
                ASSERT_EQ(ori2, ori);
            }
        }
        ASSERT_EQ(std::memcmp(cmdMsgsDesired.data(), cmdMsgs->data(), sizeof(cmdMsgsDesired)), 0);
    }
}

} // namespace
