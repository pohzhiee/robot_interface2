#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_interface_protobuf/flysky_message.pb.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>

using eCAL::protobuf::CPublisher;
using eCAL::protobuf::CSubscriber;
using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

enum class RunMode
{
    ZeroTorque,
    Normal
};

class SomeNode : public rclcpp::Node
{
  public:
    SomeNode()
        : rclcpp::Node("joint_state_to_cmd"),
          stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
          flyskySub_(std::make_unique<CSubscriber<robot_interface::FlyskyMessage>>("flysky"))
    {
        nameIndexMap_["torso_to_abduct_fl_j"] = 0;
        nameIndexMap_["abduct_fl_to_thigh_fl_j"] = 1;
        nameIndexMap_["thigh_fl_to_knee_fl_j"] = 2;
        nameIndexMap_["torso_to_abduct_fr_j"] = 3;
        nameIndexMap_["abduct_fr_to_thigh_fr_j"] = 4;
        nameIndexMap_["thigh_fr_to_knee_fr_j"] = 5;
        nameIndexMap_["torso_to_abduct_hl_j"] = 6;
        nameIndexMap_["abduct_hl_to_thigh_hl_j"] = 7;
        nameIndexMap_["thigh_hl_to_knee_hl_j"] = 8;
        nameIndexMap_["torso_to_abduct_hr_j"] = 9;
        nameIndexMap_["abduct_hr_to_thigh_hr_j"] = 10;
        nameIndexMap_["thigh_hr_to_knee_hr_j"] = 11;

        stateEstimatorSub_->AddReceiveCallback(
            [&](auto, const robot_interface::StateEstimatorMessage &msg, auto, auto, auto) {
                std::lock_guard lock(stateEstimatorMutex_);
                latestStateEstimatorMessage_ = msg;
                lastStateEstimatorMessageTime_ = steady_clock::now();
            });
        flyskySub_->AddReceiveCallback([&](auto, const robot_interface::FlyskyMessage &msg, auto, auto, auto) {
            std::lock_guard lock(flyskyMutex_);
            latestFlyskyMessage_ = msg;
            lastFlyskyMessageTime_ = steady_clock::now();
            if (msg.switch_a() == robot_interface::FlyskyMessage_SwitchState_DOWN)
            {
                runMode_ = RunMode::ZeroTorque;
                return;
            }
            runMode_ = RunMode::Normal;
        });

        motorCmdPub_ = std::make_unique<CPublisher<robot_interface::MotorCmdMsg>>("motor_cmd");
        jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 100, std::bind(&SomeNode::SomeHandler, this, _1));
    }

  private:
    std::optional<robot_interface::StateEstimatorMessage> GetLatestStateEstimatorMsg();
    std::optional<robot_interface::FlyskyMessage> GetLatestFlyskyMsg();
    void SomeHandler(const sensor_msgs::msg::JointState::SharedPtr msg);
    robot_interface::MotorCmdMsg RunZeroTorque();
    std::optional<robot_interface::MotorCmdMsg> RunNormal(const sensor_msgs::msg::JointState::SharedPtr &msg);
    std::unordered_map<std::string, int> nameIndexMap_{};
    std::unique_ptr<CPublisher<robot_interface::MotorCmdMsg>> motorCmdPub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    std::unique_ptr<CSubscriber<robot_interface::StateEstimatorMessage>> stateEstimatorSub_;
    std::unique_ptr<CSubscriber<robot_interface::FlyskyMessage>> flyskySub_;
    size_t count_{0};
    uint64_t messageCount_{0};

    // Pub subs
    // Subscriber data
    std::optional<robot_interface::FlyskyMessage> latestFlyskyMessage_{std::nullopt};
    std::optional<robot_interface::StateEstimatorMessage> latestStateEstimatorMessage_{std::nullopt};
    std::chrono::time_point<std::chrono::steady_clock> lastStateEstimatorMessageTime_{};
    std::chrono::time_point<std::chrono::steady_clock> lastFlyskyMessageTime_{};
    std::mutex flyskyMutex_;
    std::mutex stateEstimatorMutex_;
    std::atomic<RunMode> runMode_{RunMode::ZeroTorque};
};

void SomeNode::SomeHandler(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    auto latestStateEstimatorMessage = GetLatestStateEstimatorMsg();
    if (!latestStateEstimatorMessage.has_value())
        return;
    auto latestFlyskyMessage = GetLatestFlyskyMsg();
    if (!latestFlyskyMessage.has_value())
        return;
    std::optional<robot_interface::MotorCmdMsg> cmdMsg;
    switch (runMode_)
    {
    case RunMode::ZeroTorque:
        cmdMsg = RunZeroTorque();
        break;
    case RunMode::Normal:
        cmdMsg = RunNormal(msg);
        break;
    default:
        cmdMsg = std::nullopt;
    }
    if (!cmdMsg.has_value())
        return;
    motorCmdPub_->Send(cmdMsg.value());
}

robot_interface::MotorCmdMsg SomeNode::RunZeroTorque()
{
    robot_interface::MotorCmdMsg cmdMsg;
    auto cmdArrPtr = cmdMsg.mutable_commands();
    for (int i = 0; i < 12; i++)
    {
        auto motorCmdPtr = cmdArrPtr->Add();
        motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_TORQUE);
        motorCmdPtr->set_motor_id(i);
        motorCmdPtr->set_parameter(0.0);
    }
    cmdMsg.set_message_id(messageCount_);
    messageCount_++;
    return cmdMsg;
}

std::optional<robot_interface::MotorCmdMsg> SomeNode::RunNormal(const sensor_msgs::msg::JointState::SharedPtr &msg)
{
    if (msg->name.size() != 12)
    {
        RCLCPP_INFO(this->get_logger(), "Number of joint names not 12, is %d", msg->name.size());
        return std::nullopt;
    }
    robot_interface::MotorCmdMsg protoMsg;
    protoMsg.set_message_id(count_);
    count_++;
    RCLCPP_INFO(this->get_logger(), "N_joints: %d", msg->name.size());
    auto cmdPtr = protoMsg.mutable_commands();
    for (size_t i = 0; i < 12; i++)
    {
        if (nameIndexMap_.find(msg->name.at(i)) == nameIndexMap_.end())
        {
            continue;
        }
        auto cmd = cmdPtr->Add();
        auto id = nameIndexMap_[msg->name.at(i)];
        cmd->set_motor_id(id);
        cmd->set_command(robot_interface::MotorCmd_CommandType_POSITION);
        cmd->set_parameter(msg->position.at(i));
    }
    return protoMsg;
}

std::optional<robot_interface::StateEstimatorMessage> SomeNode::GetLatestStateEstimatorMsg()
{
    std::lock_guard lock(stateEstimatorMutex_);
    if (!latestStateEstimatorMessage_.has_value())
    {
        RCLCPP_INFO(this->get_logger(), "No state estimator message");
        return std::nullopt;
    }
    if (steady_clock::now() - lastStateEstimatorMessageTime_ > 500ms)
    {
        RCLCPP_INFO(this->get_logger(), "Last state estimator message more than 500ms old");
        latestStateEstimatorMessage_ = std::nullopt;
        return std::nullopt;
    }
    if (latestStateEstimatorMessage_->joint_positions().size() != 12)
    {
        RCLCPP_INFO(this->get_logger(), "Last state estimator message does not have 12 joint positions, has: %d",
                    latestStateEstimatorMessage_->joint_positions().size());
        return std::nullopt;
    }
    return latestStateEstimatorMessage_.value();
}
std::optional<robot_interface::FlyskyMessage> SomeNode::GetLatestFlyskyMsg()
{
    std::lock_guard lock(flyskyMutex_);
    if (!latestFlyskyMessage_.has_value())
    {
        RCLCPP_INFO(this->get_logger(), "No flysky message");
        return std::nullopt;
    }
    if (steady_clock::now() - lastFlyskyMessageTime_ > 500ms)
    {
        RCLCPP_INFO(this->get_logger(), "Last flysky message more than 500ms old");
        latestFlyskyMessage_ = std::nullopt;
        return std::nullopt;
    }
    return latestFlyskyMessage_.value();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    eCAL::Initialize({}, "Ros2 JointState to Cmd");

    //     set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Ros2 JointState to Cmd");
    auto node = std::make_shared<SomeNode>();
    while (rclcpp::ok())
        if (!eCAL::Ok())
            break;
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    eCAL::Finalize();
    return 0;
}