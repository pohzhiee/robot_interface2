#include <hyq_cheetah/MPC.hpp>

#include "robot_interface2/utils/NlohmannJsonEigenConversion.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
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
    explicit SomeClass();
    ~SomeClass();

    void RunLoop();
    void RunBalanceController(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                              const robot_interface::FlyskyMessage &flyskyMsg);
    void RunRecoveryStandController(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                                    const robot_interface::FlyskyMessage &flyskyMsg);
    void RunZeroTorque();
    void RunZeroPosition();
    void RunRead();

  private:
    std::optional<robot_interface::StateEstimatorMessage> GetLatestStateEstimatorMsg();
    std::optional<robot_interface::FlyskyMessage> GetLatestFlyskyMsg();
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
    std::vector<nlohmann::json> dataItems{};
};

SomeClass::SomeClass()
    : stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
      flyskySub_(std::make_unique<CSubscriber<robot_interface::FlyskyMessage>>("flysky"))
{
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
    auto next = steady_clock::now() + 5ms;
    mainControllerStartTime_ = high_resolution_clock::now();
    while (!stopRequested_)
    {
        auto loopStartTime = steady_clock::now();
        if (loopStartTime > next)
        {
            next = loopStartTime;
            std::cerr << "Timing missed" << std::endl;
        }
        std::this_thread::sleep_until(next);
        next = next + duration<int64_t, std::ratio<1, 800>>{1};

        auto latestStateEstimatorMessage = GetLatestStateEstimatorMsg();
        if (!latestStateEstimatorMessage.has_value())
        {
            continue;
        }
        auto latestFlyskyMessage = GetLatestFlyskyMsg();
        if (!latestFlyskyMessage.has_value())
            continue;
        if (previousRunMode_ == RunMode::BalanceWalk && runMode_ != RunMode::BalanceWalk)
        {
            std::ofstream o("pretty.json");
            nlohmann::json j;
            j = dataItems;
            o << std::setw(4) << j << std::endl;
            dataItems.clear();
        }
        switch (runMode_)
        {
        case RunMode::ZeroTorque:
            RunZeroTorque();
            previousRunMode_ = RunMode::ZeroTorque;
            break;
        case RunMode::ZeroPosition:
            RunZeroPosition();
            previousRunMode_ = RunMode::ZeroPosition;
            break;
        case RunMode::BalanceWalk:
            if (previousRunMode_ != RunMode::BalanceWalk)
            {
                mainControllerStartTime_ = high_resolution_clock::now();
            }
            RunBalanceController(latestStateEstimatorMessage.value(), latestFlyskyMessage.value());
            previousRunMode_ = RunMode::BalanceWalk;
            break;
        case RunMode::Recovery:
            if (previousRunMode_ != RunMode::Recovery)
            {
                recoveryControllerStartTime_ = high_resolution_clock::now();
            }
            RunRecoveryStandController(latestStateEstimatorMessage.value(), latestFlyskyMessage.value());
            previousRunMode_ = RunMode::Recovery;
            break;
        default:
            break;
        }
    }
}

void SomeClass::RunRead()
{
}

void SomeClass::RunZeroTorque()
{
}
void SomeClass::RunZeroPosition()
{
}
void SomeClass::RunBalanceController(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                                     const robot_interface::FlyskyMessage &flyskyMsg)
{
    using namespace nlohmann;
    const auto &estimatedState = StateEstimatorDataFromProtobuf(stateEstimatorMsg);
    const auto &userInput = UserInputFromFlyskyProtobuf(flyskyMsg);
    const auto timeSinceStart = duration_cast<nanoseconds>(high_resolution_clock::now() - mainControllerStartTime_).count();
    {
        // Record data
        json dataItem;
        json baseRotJson, worldAngVelJson, worldLinVelJson;
        to_json(baseRotJson, estimatedState.baseRotation);
        to_json(worldAngVelJson, estimatedState.worldAngularVelocity);
        to_json(worldLinVelJson, estimatedState.worldLinearVelocity);
        dataItem["baseRotation"] = baseRotJson;
        dataItem["worldAngularVel"] = worldAngVelJson;
        dataItem["worldLinearVel"] = worldLinVelJson;
        dataItem["jointPositions"] = estimatedState.jointPositions;
        dataItem["jointVelocities"] = estimatedState.jointVelocities;
        dataItem["xVelCmd"] = userInput.x_vel_cmd;
        dataItem["yVelCmd"] = userInput.y_vel_cmd;
        dataItem["yawTurnRate"] = userInput.yaw_turn_rate;
        dataItem["height"] = userInput.height;
        dataItem["timeSinceStart"] = timeSinceStart;
        dataItems.emplace_back(std::move(dataItem));
    }
}
void SomeClass::RunRecoveryStandController(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                                           const robot_interface::FlyskyMessage &flyskyMsg)
{
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
    std::string robotName(argv[1]);
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Data Recorder");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Data Recorder");

    SomeClass a;
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }
    eCAL::Finalize();
}