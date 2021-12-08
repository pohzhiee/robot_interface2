#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>

using eCAL::protobuf::CPublisher;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SomeNode : public rclcpp::Node
{
  public:
    SomeNode()
        : rclcpp::Node("joint_state_to_cmd")/*, motorCmdPub_(std::move(pub))*/
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

        motorCmdPub_ = std::make_unique<CPublisher<robot_interface::MotorCmdMsg>>("motor_cmd");
        jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 100, std::bind(&SomeNode::SomeHandler, this, _1));
    }

  private:
    void SomeHandler(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() != 12)
        {
            RCLCPP_INFO(this->get_logger(), "Number of joint names not 12, is %d", msg->name.size());
        }
        robot_interface::MotorCmdMsg protoMsg;
        protoMsg.set_message_id(count_);
        count_++;
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
        motorCmdPub_->Send(protoMsg);
    }
    std::unordered_map<std::string, int> nameIndexMap_{};
    std::unique_ptr<CPublisher<robot_interface::MotorCmdMsg>> motorCmdPub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    size_t count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    eCAL::Initialize({}, "Ros2 JointState to Cmd");

//     set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Ros2 JointState to Cmd");
    auto node = std::make_shared<SomeNode>();
    while(rclcpp::ok())
//        if(!eCAL::Ok())
//            break;
        rclcpp::spin_some(node);
    rclcpp::shutdown();
    eCAL::Finalize();
    return 0;
}