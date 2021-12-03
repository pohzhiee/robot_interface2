#include <hyq_cheetah/MPC.hpp>
#include <hyq_cheetah/MainController.hpp>
#include <hyq_cheetah/RecoveryStandController.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <hyq_cheetah/helpers/config.hpp>
#include <robot_interface_protobuf/flysky_message.pb.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>
#include <spdlog/spdlog.h>
#include <thread>
#include <optional>

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
    userInput.x_vel_cmd = convert(msg.channel2(), 1.0);
    userInput.y_vel_cmd = convert(msg.channel1(), -1.0);
    userInput.yaw_turn_rate = convert(msg.channel4(), -1.0);
    userInput.height = 0.29;

    return userInput;
}
} // namespace

class SomeClass
{
  public:
    explicit SomeClass(const std::string &robotName);
    ~SomeClass();

//    std::unique_ptr<RecoveryStandController> recoveryStandController_;
    std::unique_ptr<MainController> mainController_;

    void RunLoop();

  private:
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
};


SomeClass::SomeClass(const std::string &robotName)
    : stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
      flyskySub_(std::make_unique<CSubscriber<robot_interface::FlyskyMessage>>("flysky"))
{
    std::string tomlPath = fmt::format("{}{}/controllerConfig.toml", config::install_robots_path, robotName);
    std::string urdfPath = fmt::format("{}{}/{}.urdf", config::install_robots_path, robotName, robotName);
    auto mainControllerConfig = ConfigFromToml(tomlPath);
    auto gait = std::make_shared<Gait>(mainControllerConfig.gaitConfig);
    auto balanceController = std::make_shared<MPC>(mainControllerConfig.mpcConfig);
    mainController_ = std::make_unique<MainController>(mainControllerConfig, balanceController, gait, urdfPath);
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
    });
    runThread_ = std::thread([&]() { this->RunLoop(); });
}

void SomeClass::RunLoop()
{
    auto next = steady_clock::now() + 5ms;
    mainControllerStartTime_ = high_resolution_clock::now();
    while (!stopRequested_)
    {
        auto loopStartTime = steady_clock::now();
        if (loopStartTime > next)
        {
            next = loopStartTime;
        }
        std::this_thread::sleep_until(next);
        next = next + duration<int64_t, std::ratio<1, 200>>{1};

        robot_interface::StateEstimatorMessage latestStateEstimatorMessage;
        robot_interface::FlyskyMessage latestFlyskyMessage;
        {
            std::lock_guard lock(flyskyMutex_);
            if (!latestFlyskyMessage_.has_value())
            {
                spdlog::debug("No flysky message");
                continue;
            }
            if (steady_clock::now() - lastFlyskyMessageTime_ > 500ms)
            {
                spdlog::debug("Last flysky message more than 500ms old");
                latestFlyskyMessage_ = std::nullopt;
                continue;
            }
            latestFlyskyMessage = latestFlyskyMessage_.value();
        }
        {
            std::lock_guard lock(stateEstimatorMutex_);
            if (!latestStateEstimatorMessage_.has_value())
            {
                spdlog::debug("No state estimator message");
                continue;
            }
            if (steady_clock::now() - lastStateEstimatorMessageTime_ > 500ms)
            {
                spdlog::debug("Last state estimator message more than 500ms old");
                latestStateEstimatorMessage_ = std::nullopt;
                continue;
            }
            if (latestStateEstimatorMessage_->joint_positions().size() != 12)
            {
                spdlog::debug(fmt::format("Last state estimator message does not have 12 joint positions, has: {}",
                                          latestStateEstimatorMessage_->joint_positions().size()));
                continue;
            }
            latestStateEstimatorMessage = latestStateEstimatorMessage_.value();
        }

        ControllerInputData data;
        data.estimatedState = StateEstimatorDataFromProtobuf(latestStateEstimatorMessage);
        data.userInput = UserInputFromFlyskyProtobuf(latestFlyskyMessage);
        data.timeSinceStart =
            duration_cast<nanoseconds>(high_resolution_clock::now() - mainControllerStartTime_).count();
        auto output = mainController_->Run(data);
        if (!output.has_value())
        {
            spdlog::debug("Controller generates no output");
            continue;
        }
        robot_interface::MotorCmdMsg cmdMsg;
        auto cmdArrPtr = cmdMsg.mutable_commands();
        for (int i = 0; i < 12; i++)
        {
            auto motorCmdPtr = cmdArrPtr->Add();
            motorCmdPtr->set_command(robot_interface::MotorCmd_CommandType_TORQUE);
            motorCmdPtr->set_motor_id(i);
            motorCmdPtr->set_parameter(output->commands.at(i));
        }
        motorCmdPub_->Send(cmdMsg);
    }
}
SomeClass::~SomeClass()
{
    stopRequested_ = true;
    runThread_.join();
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Please input robot name, e.g.: ./main_exec spot" << std::endl;
        return 1;
    }
    std::string robotName(argv[1]);
    // initialize eCAL API
    eCAL::Initialize({}, "HyQ Cheetah Controller");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "HyQ Cheetah Controller");

    SomeClass a(robotName);
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}