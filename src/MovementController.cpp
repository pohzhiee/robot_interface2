#include <hyq_cheetah/MPC.hpp>
#include <hyq_cheetah/MainController.hpp>
#include <hyq_cheetah/RecoveryStandController.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <fstream>
#include <hyq_cheetah/helpers/config.hpp>
#include <optional>
#include <robot_interface_protobuf/flysky_message.pb.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>
#include <spdlog/spdlog.h>
#include <thread>

using namespace hyq_cheetah;
using namespace eCAL::protobuf;
using namespace std::chrono;
using namespace std::chrono_literals;
namespace
{
hyq_cheetah::StateEstimatorData StateEstimatorDataFromProtobuf(const robot_interface::StateEstimatorMessage &msg)
{
    assert(msg.joint_positions().size() == 12);
    assert(msg.joint_velocities().size() == 12);
    hyq_cheetah::StateEstimatorData data;
    data.baseRotation = Eigen::Quaterniond(msg.base_orientation().w(), msg.base_orientation().x(),
                                           msg.base_orientation().y(), msg.base_orientation().z());
    data.worldAngularVelocity = Eigen::Vector3d(msg.world_angular_velocity().x(), msg.world_angular_velocity().y(),
                                                msg.world_angular_velocity().z());
    data.worldLinearVelocity = Eigen::Vector3d(msg.world_linear_velocity().x(), msg.world_linear_velocity().y(),
                                               msg.world_linear_velocity().z());
    std::copy_n(msg.joint_positions().cbegin(), 12, data.jointPositions.begin());
    std::copy_n(msg.joint_velocities().cbegin(), 12, data.jointVelocities.begin());
    return data;
}

hyq_cheetah::UserInput UserInputFromFlyskyProtobuf(const robot_interface::FlyskyMessage &msg)
{
    assert(msg.switch_a() != robot_interface::FlyskyMessage_SwitchState_UNKNOWN);
    hyq_cheetah::UserInput userInput;
    // Ch1 is y, -ve
    // Ch2 is x, +ve
    // Ch4 is yaw rate, -ve
    // We map to min/max velocity of -2/2 m/s, angular velocity of -2/2 rad/s
    uint32_t ch1Val = msg.channel1();
    auto convert = [](uint32_t val, double sign) -> double { return (-6 + static_cast<double>(val) * 0.004) * sign; };
    userInput.x_vel_cmd = convert(msg.channel2(), -1.0);
    userInput.y_vel_cmd = convert(msg.channel1(), 1.0);
    userInput.yaw_turn_rate = convert(msg.channel4(), -1.0);
    userInput.height = 0.305;

    return userInput;
}
} // namespace

enum class RunMode
{
    ZeroTorque,
    ZeroPosition,
    Recovery,
    BalanceWalk
};

class DummyBalanceController : public BalanceController
{
    [[nodiscard]] Eigen::Matrix<double, 3, numLegs> Run(const StateEstimatorData &estimatedState,
                                                        const UserInput &userInput, const Eigen::Vector3d &desRpy,
                                                        const Eigen::Matrix<double, 3, numLegs> &footDist,
                                                        const Gait &gait) const final
    {
        return Eigen::Matrix<double, 3, numLegs>::Zero();
    }
    void Reset() final
    {
        std::cout << "reset" << std::endl;
    };
};

class SomeClass
{
  public:
    explicit SomeClass(const std::string &robotName);
    ~SomeClass();

    void RunLoop();
    std::optional<robot_interface::MotorCmdMsg> RunBalanceController(
        const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
        const robot_interface::FlyskyMessage &flyskyMsg);
    robot_interface::MotorCmdMsg RunRecoveryStandController(
        const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
        const robot_interface::FlyskyMessage &flyskyMsg);
    robot_interface::MotorCmdMsg RunZeroTorque();
    robot_interface::MotorCmdMsg RunZeroPosition();
    robot_interface::MotorCmdMsg RunRead();

  private:
    std::optional<robot_interface::StateEstimatorMessage> GetLatestStateEstimatorMsg();
    std::optional<robot_interface::FlyskyMessage> GetLatestFlyskyMsg();
    std::unique_ptr<RecoveryStandController> recoveryStandController_;
    std::unique_ptr<MainController> mainController_;
    // Pub subs
    std::unique_ptr<CSubscriber<robot_interface::StateEstimatorMessage>> stateEstimatorSub_;
    std::unique_ptr<CSubscriber<robot_interface::FlyskyMessage>> flyskySub_;
    std::unique_ptr<CPublisher<robot_interface::MotorCmdMsg>> motorCmdPub_;
    // Subscriber data
    std::optional<robot_interface::FlyskyMessage> latestFlyskyMessage_{std::nullopt};
    std::optional<robot_interface::StateEstimatorMessage> latestStateEstimatorMessage_{std::nullopt};
    std::chrono::time_point<std::chrono::steady_clock> lastStateEstimatorMessageTime_{};
    std::chrono::time_point<std::chrono::steady_clock> lastFlyskyMessageTime_{};
    // Thread management
    std::mutex flyskyMutex_;
    std::mutex stateEstimatorMutex_;
    std::thread runThread_;
    std::atomic<bool> stopRequested_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> mainControllerStartTime_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> recoveryControllerStartTime_{};

    // State management
    std::atomic<RunMode> runMode_{RunMode::ZeroTorque};
    RunMode previousRunMode_{RunMode::ZeroTorque};
    std::uint64_t messageCount_{0};
};

SomeClass::SomeClass(const std::string &robotName)
    : stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
      flyskySub_(std::make_unique<CSubscriber<robot_interface::FlyskyMessage>>("flysky")),
      motorCmdPub_(std::make_unique<CPublisher<robot_interface::MotorCmdMsg>>("motor_cmd"))
{
    std::string tomlPath = fmt::format("{}{}/controllerConfig.toml", config::install_robots_path, robotName);
    std::string urdfPath = fmt::format("{}{}/{}.urdf", config::install_robots_path, robotName, robotName);
    auto mainControllerConfig = ConfigFromToml(tomlPath);
    auto gait = std::make_shared<Gait>(mainControllerConfig.gaitConfig);
    //    auto balanceController = std::make_shared<MPC>(mainControllerConfig.mpcConfig);
    auto balanceController2 = std::make_shared<DummyBalanceController>();
    mainController_ = std::make_unique<MainController>(mainControllerConfig, balanceController2, gait, urdfPath);
    recoveryStandController_ = std::make_unique<RecoveryStandController>(mainControllerConfig.recoveryConfig);
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
        switch (msg.switch_b())
        {
        case robot_interface::FlyskyMessage_SwitchState_UP:
            runMode_ = RunMode::ZeroPosition;
            return;
        case robot_interface::FlyskyMessage_SwitchState_MIDDLE:
            runMode_ = RunMode::Recovery;
            return;
        case robot_interface::FlyskyMessage_SwitchState_DOWN:
            runMode_ = RunMode::BalanceWalk;
            return;
        default:
            break;
        }
        runMode_ = RunMode::ZeroTorque;
    });
    runThread_ = std::thread([&]() { this->RunLoop(); });
}

SomeClass::~SomeClass()
{
    stopRequested_ = true;
    runThread_.join();
}

void SomeClass::RunLoop()
{
    std::this_thread::sleep_for(10ms);
    auto next = steady_clock::now() + 5ms;
    mainControllerStartTime_ = high_resolution_clock::now();
    while (!stopRequested_)
    {
        auto GetCmd = [&](){
            auto latestStateEstimatorMessage = GetLatestStateEstimatorMsg();
            if (!latestStateEstimatorMessage.has_value())
            {
                return RunRead();
            }
            auto latestFlyskyMessage = GetLatestFlyskyMsg();
            if (!latestFlyskyMessage.has_value())
                return RunRead();
            std::optional<robot_interface::MotorCmdMsg> cmdMsg;
            switch (runMode_)
            {
            case RunMode::ZeroTorque:
                cmdMsg = RunZeroTorque();
                previousRunMode_ = RunMode::ZeroTorque;
                break;
            case RunMode::ZeroPosition:
                cmdMsg = RunZeroPosition();
                previousRunMode_ = RunMode::ZeroPosition;
                break;
            case RunMode::BalanceWalk:
                if (previousRunMode_ != RunMode::BalanceWalk)
                {
                    mainControllerStartTime_ = high_resolution_clock::now();
                    mainController_->Reset();
                }
                cmdMsg = RunBalanceController(latestStateEstimatorMessage.value(), latestFlyskyMessage.value());
                previousRunMode_ = RunMode::BalanceWalk;
                break;
            case RunMode::Recovery:
                if (previousRunMode_ != RunMode::Recovery)
                {
                    recoveryControllerStartTime_ = high_resolution_clock::now();
                    recoveryStandController_->Reset();
                }
                cmdMsg = RunRecoveryStandController(latestStateEstimatorMessage.value(), latestFlyskyMessage.value());
                previousRunMode_ = RunMode::Recovery;
                break;
            default:
                cmdMsg = std::nullopt;
            }
            if (!cmdMsg.has_value())
                return RunRead();
            return cmdMsg.value();
        };

        auto cmd = GetCmd();
        auto loopStartTime = steady_clock::now();
        if (loopStartTime > next)
        {
            next = loopStartTime;
            std::cerr << "Timing missed" << std::endl;
        }
        std::this_thread::sleep_until(next);
        next = next + duration<int64_t, std::ratio<1, 800>>{1};

        motorCmdPub_->Send(cmd);
    }
}

robot_interface::MotorCmdMsg SomeClass::RunRead()
{
    robot_interface::MotorCmdMsg cmdMsg;
    auto cmdArrPtr = cmdMsg.mutable_commands();
    for (int i = 0; i < 12; i++)
    {
        auto motorCmdPtr = cmdArrPtr->Add();
        motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_READ);
        motorCmdPtr->set_motor_id(i);
        motorCmdPtr->set_parameter(0.0);
    }
    cmdMsg.set_message_id(messageCount_);
    messageCount_++;
    return cmdMsg;
}

robot_interface::MotorCmdMsg SomeClass::RunZeroTorque()
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
robot_interface::MotorCmdMsg SomeClass::RunZeroPosition()
{
    robot_interface::MotorCmdMsg cmdMsg;
    auto cmdArrPtr = cmdMsg.mutable_commands();
    for (int i = 0; i < 12; i++)
    {
        auto motorCmdPtr = cmdArrPtr->Add();
        motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_POSITION);
        motorCmdPtr->set_motor_id(i);
        motorCmdPtr->set_parameter(0.0);
    }
    cmdMsg.set_message_id(messageCount_);
    messageCount_++;
    return cmdMsg;
}
std::optional<robot_interface::MotorCmdMsg> SomeClass::RunBalanceController(
    const robot_interface::StateEstimatorMessage &stateEstimatorMsg, const robot_interface::FlyskyMessage &flyskyMsg)
{
    ControllerInputData data;
    data.estimatedState = StateEstimatorDataFromProtobuf(stateEstimatorMsg);
    data.userInput = UserInputFromFlyskyProtobuf(flyskyMsg);
    data.timeSinceStart = duration_cast<nanoseconds>(high_resolution_clock::now() - mainControllerStartTime_).count();
    auto output = mainController_->Run(data);
    if (!output.has_value())
    {
        spdlog::debug("Controller generates no output");
        return std::nullopt;
    }
    robot_interface::MotorCmdMsg cmdMsg;
    auto cmdArrPtr = cmdMsg.mutable_commands();

    for (int i = 0; i < 4; i++)
    {
        auto swingJointVel = output->swingJointVel.at(i);
        auto stanceJointVel = output->stanceJointVel.at(i);
        for (int j = 0; j < 3; j++)
        {
            auto motorCmdPtr = cmdArrPtr->Add();
            if (swingJointVel.has_value())
            {
                // This only has value during swing, so during stance we are still using torque controller
                motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_VELOCITY);
                motorCmdPtr->set_motor_id(i * 3 + j);
                motorCmdPtr->set_parameter(swingJointVel.value()(j));
            }
            else if(stanceJointVel.has_value())
            {
                motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_VELOCITY);
                motorCmdPtr->set_motor_id(i * 3 + j);
                motorCmdPtr->set_parameter(stanceJointVel.value()(j));
            }
            else{
                motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_TORQUE);
                motorCmdPtr->set_motor_id(i * 3 + j);
                motorCmdPtr->set_parameter(0.0);
                spdlog::warn("No stance vel and no swing vel");
            }
        }
    }
    cmdMsg.set_message_id(messageCount_);
    messageCount_++;
    return cmdMsg;
}
robot_interface::MotorCmdMsg SomeClass::RunRecoveryStandController(
    const robot_interface::StateEstimatorMessage &stateEstimatorMsg, const robot_interface::FlyskyMessage &flyskyMsg)
{
    using Eigen::Map;
    using Eigen::Matrix;
    const auto stateEstimatorData = StateEstimatorDataFromProtobuf(stateEstimatorMsg);
    auto cmd = recoveryStandController_->Run(
        stateEstimatorData,
        duration_cast<nanoseconds>(high_resolution_clock::now() - recoveryControllerStartTime_).count());
    robot_interface::MotorCmdMsg cmdMsg;
    auto cmdArrPtr = cmdMsg.mutable_commands();
    auto intermediatePos = recoveryStandController_->GetIntermediatePos();
    Map<const Matrix<double, 12, 1>> intermediatePosVec(intermediatePos.data(), intermediatePos.size());
    Map<const Matrix<double, 12, 1>> currentPosVec(stateEstimatorData.jointPositions.data(),
                                                   stateEstimatorData.jointPositions.size());
    Matrix<double, 12, 1> posError = intermediatePosVec - currentPosVec;
    const double kp = 4.0;
    Matrix<double, 12, 1> velCmd = kp * posError;

    for (int i = 0; i < 12; i++)
    {
        auto motorCmdPtr = cmdArrPtr->Add();
        motorCmdPtr->set_motor_id(i);
        motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_VELOCITY);
        motorCmdPtr->set_parameter(velCmd(i));
    }
    cmdMsg.set_message_id(messageCount_);
    messageCount_++;
    return cmdMsg;
}

std::optional<robot_interface::StateEstimatorMessage> SomeClass::GetLatestStateEstimatorMsg()
{
    std::lock_guard lock(stateEstimatorMutex_);
    if (!latestStateEstimatorMessage_.has_value())
    {
        spdlog::debug("No state estimator message");
        return std::nullopt;
    }
    if (steady_clock::now() - lastStateEstimatorMessageTime_ > 500ms)
    {
        spdlog::info("Last state estimator message more than 500ms old");
        latestStateEstimatorMessage_ = std::nullopt;
        return std::nullopt;
    }
    if (latestStateEstimatorMessage_->joint_positions().size() != 12)
    {
        spdlog::info(fmt::format("Last state estimator message does not have 12 joint positions, has: {}",
                                 latestStateEstimatorMessage_->joint_positions().size()));
        return std::nullopt;
    }
    return latestStateEstimatorMessage_.value();
}

std::optional<robot_interface::FlyskyMessage> SomeClass::GetLatestFlyskyMsg()
{
    std::lock_guard lock(flyskyMutex_);
    if (!latestFlyskyMessage_.has_value())
    {
        spdlog::debug("No flysky message");
        return std::nullopt;
    }
    if (steady_clock::now() - lastFlyskyMessageTime_ > 500ms)
    {
        spdlog::info("Last flysky message more than 500ms old");
        latestFlyskyMessage_ = std::nullopt;
        return std::nullopt;
    }
    return latestFlyskyMessage_.value();
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Please input robot name, e.g.: ./movement_controller robot1" << std::endl;
        return 1;
    }
    std::string robotName(argv[1]);
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Movement Controller");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Movement Controller");

    SomeClass a(robotName);
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }
    eCAL::Finalize();
}