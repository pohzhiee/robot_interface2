#include "robot_interface2/utils/NlohmannJsonEigenConversion.hpp"
#include <Eigen/Geometry>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <fmt/chrono.h>
#include <fstream>
#include <iomanip>
#include <optional>
#include <robot_interface_protobuf/flysky_message.pb.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/motor_feedback_msg.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>
#include <spdlog/spdlog.h>
#include <sqlite3.h>
#include <signal.h>
#include <thread>

using namespace eCAL::protobuf;
using namespace std::chrono;
using namespace std::chrono_literals;
namespace
{
struct UserInput
{
    double yaw_turn_rate;
    double x_vel_cmd;
    double y_vel_cmd;
    double height;
};

struct StateEstimatorData
{
    Eigen::Quaterniond baseRotation;        // quaternion, in world frame
    Eigen::Vector3d worldLinearVelocity;    // in world frame
    Eigen::Vector3d worldAngularVelocity;   // in world frame
    std::array<double, 12> jointPositions;  // in radians
    std::array<double, 12> jointVelocities; // in rad/s
};
StateEstimatorData StateEstimatorDataFromProtobuf(const robot_interface::StateEstimatorMessage &msg)
{
    assert(msg.joint_positions().size() == 12);
    assert(msg.joint_velocities().size() == 12);
    StateEstimatorData data;
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

UserInput UserInputFromFlyskyProtobuf(const robot_interface::FlyskyMessage &msg)
{
    assert(msg.switch_a() != robot_interface::FlyskyMessage_SwitchState_UNKNOWN);
    UserInput userInput;
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

class SomeClass
{
  public:
    explicit SomeClass();
    ~SomeClass();

    void RunLoop();
    void RecordBalancingData(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                             const robot_interface::FlyskyMessage &flyskyMsg);

  private:
    std::optional<robot_interface::StateEstimatorMessage> GetLatestStateEstimatorMsg();
    std::optional<robot_interface::FlyskyMessage> GetLatestFlyskyMsg();
    // Pub subs
    std::unique_ptr<CSubscriber<robot_interface::StateEstimatorMessage>> stateEstimatorSub_;
    std::unique_ptr<CSubscriber<robot_interface::FlyskyMessage>> flyskySub_;
    std::unique_ptr<CSubscriber<robot_interface::MotorCmdMsg>> cmdSub_;
    std::unique_ptr<CSubscriber<robot_interface::MotorFeedbackMsg>> feedbackSub_;
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

    // State management
    std::atomic<RunMode> runMode_{RunMode::ZeroTorque};
    RunMode previousRunMode_{RunMode::ZeroTorque};
    std::uint64_t messageCount_{0};
    std::vector<nlohmann::json> dataItems{};
    sqlite3 *sqliteDb_;

    // Callbacks
    void CmdMsgCb(const char *, const robot_interface::MotorCmdMsg &msg_, long long, long long, long long);
    void FeedbackMsgCb(const char *, const robot_interface::MotorFeedbackMsg &msg_, long long, long long, long long);
};

SomeClass::SomeClass()
    : stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
      flyskySub_(std::make_unique<CSubscriber<robot_interface::FlyskyMessage>>("flysky")),
      cmdSub_(std::make_unique<CSubscriber<robot_interface::MotorCmdMsg>>("motor_cmd")),
      feedbackSub_(std::make_unique<CSubscriber<robot_interface::MotorFeedbackMsg>>("motor_feedback"))
{
    std::time_t t = std::time(nullptr);
    std::string dbName = fmt::format("{:%Y_%m_%d_%H%M%S}.db", *std::localtime(&t));
    int rc = sqlite3_open(dbName.c_str(), &sqliteDb_);
    if (rc != SQLITE_OK)
    {
        throw std::runtime_error(fmt::format("Failed to open sqlite database at {}", dbName));
    }
    constexpr std::string_view createTableTemplate = "create table {}\n"
                                                     "(\n"
                                                     "    time_ns integer not null,\n"
                                                     "    type    text    not null,\n"
                                                     "    value   real    not null\n"
                                                     ");";
    for (int i = 0; i < 12; i++)
    {
        std::string tableName = fmt::format("motor{}", i);
        std::string createTableString = fmt::format(createTableTemplate, tableName);
        sqlite3_exec(sqliteDb_, "BEGIN TRANSACTION;", NULL, NULL, NULL);
        rc = sqlite3_exec(sqliteDb_, createTableString.c_str(), nullptr, nullptr, nullptr);
        sqlite3_exec(sqliteDb_, "END TRANSACTION;", NULL, NULL, NULL);
        if (rc != SQLITE_OK)
        {
            throw std::runtime_error(fmt::format("Failed to create table {}", tableName));
        }
    }
    cmdSub_->AddReceiveCallback(
        [this](auto a, const auto &b, auto c, auto d, auto e) { this->CmdMsgCb(a, b, c, d, e); });
    feedbackSub_->AddReceiveCallback(
        [this](auto a, const auto &b, auto c, auto d, auto e) { this->FeedbackMsgCb(a, b, c, d, e); });
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
    sqlite3_close(sqliteDb_);
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
            previousRunMode_ = RunMode::ZeroTorque;
            break;
        case RunMode::ZeroPosition:
            previousRunMode_ = RunMode::ZeroPosition;
            break;
        case RunMode::BalanceWalk:
            if (previousRunMode_ != RunMode::BalanceWalk)
            {
                mainControllerStartTime_ = high_resolution_clock::now();
            }
            RecordBalancingData(latestStateEstimatorMessage.value(), latestFlyskyMessage.value());
            previousRunMode_ = RunMode::BalanceWalk;
            break;
        case RunMode::Recovery:
            previousRunMode_ = RunMode::Recovery;
            break;
        default:
            break;
        }
    }
}

void SomeClass::RecordBalancingData(const robot_interface::StateEstimatorMessage &stateEstimatorMsg,
                                    const robot_interface::FlyskyMessage &flyskyMsg)
{
    using namespace nlohmann;
    const auto &estimatedState = StateEstimatorDataFromProtobuf(stateEstimatorMsg);
    const auto &userInput = UserInputFromFlyskyProtobuf(flyskyMsg);
    const auto timeSinceStart =
        duration_cast<nanoseconds>(high_resolution_clock::now() - mainControllerStartTime_).count();
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

void SomeClass::CmdMsgCb(const char * /*topic_name_*/, const robot_interface::MotorCmdMsg &msg, long long time_,
                         long long /*clock_*/, long long /*id_*/)
{
    constexpr std::string_view insertTemplate = "insert into {} (time_ns, type, value)\n"
                                                "values ({}, \'{}\', {});";
    uint64_t timeNs = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
    for (auto &cmd : msg.commands())
    {
        std::string typeStr;
        switch (cmd.command())
        {
        case robot_interface::MotorCmd_CommandType_POSITION:
            typeStr = "cmd_pos";
            break;
        case robot_interface::MotorCmd_CommandType_VELOCITY:
            typeStr = "cmd_vel";
            break;
        case robot_interface::MotorCmd_CommandType_TORQUE:
            typeStr = "cmd_torque";
            break;
        default:
            continue;
        }
        auto id = cmd.motor_id();
        std::string tableName = fmt::format("motor{}", id);
        std::string sqlStatement = fmt::format(insertTemplate, tableName, timeNs, typeStr, cmd.parameter());
        sqlite3_exec(sqliteDb_, "BEGIN TRANSACTION;", NULL, NULL, NULL);
        // Any (modifying) SQL commands executed here are not committed until at the you call:
        int rc = sqlite3_exec(sqliteDb_, sqlStatement.c_str(), nullptr, nullptr, nullptr);
        sqlite3_exec(sqliteDb_, "END TRANSACTION;", NULL, NULL, NULL);
        if (rc != SQLITE_OK)
        {
            std::cerr << "[cmd] insert error on table " << tableName << " at time " << timeNs << std::endl;
        }
    }
}
void SomeClass::FeedbackMsgCb(const char * /*topic_name_*/, const robot_interface::MotorFeedbackMsg &msg,
                              long long time, long long clock, long long id)
{

    constexpr std::string_view insertTemplate = "insert into {0} (time_ns, type, value)\n"
                                                "values ({1}, \'pos\', {2});"
                                                "insert into {0} (time_ns, type, value)\n"
                                                "values ({1}, \'vel\', {3});"
                                                "insert into {0} (time_ns, type, value)\n"
                                                "values ({1}, \'torque\', {4});"
                                                "insert into {0} (time_ns, type, value)\n"
                                                "values ({1}, \'temperature\', {5});";
    static bool first = true;
    uint64_t timeNs = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
    if(first){
        first = false;
        std::cerr << "Timens: " << timeNs << std::endl;
        std::cerr << "Time: " << time << std::endl;
        std::cerr << "clock: " << clock << std::endl;
        std::cerr << "id: " << id << std::endl;
    }
    for (auto &feedback : msg.feedbacks())
    {
        if (!feedback.ready())
            continue;
        std::string typeStr;
        auto motorId = feedback.motor_id();
        std::string tableName = fmt::format("motor{}", motorId);
        std::string sqlStatement = fmt::format(insertTemplate, tableName, timeNs, feedback.angle(), feedback.velocity(),
                                               feedback.torque(), feedback.temperature());
        sqlite3_exec(sqliteDb_, "BEGIN TRANSACTION;", NULL, NULL, NULL);
        // Any (modifying) SQL commands executed here are not committed until at the you call:
        int rc = sqlite3_exec(sqliteDb_, sqlStatement.c_str(), nullptr, nullptr, nullptr);
        sqlite3_exec(sqliteDb_, "END TRANSACTION;", NULL, NULL, NULL);
        if (rc != SQLITE_OK)
        {
            std::cerr << "[fdbk] insert error on table " << tableName << " at time " << timeNs << std::endl;
        }
    }
}

bool isInt = false;
void my_handler(int s){
    printf("Caught signal %d\n",s);
    isInt = true;
}


int main(int argc, char *argv[])
{
    signal(SIGINT,my_handler);
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 Data Recorder");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 Data Recorder");

    SomeClass a;
    while (eCAL::Ok() && !isInt)
    {
        eCAL::Process::SleepMS(100);
    }
    std::cout << "Ending" << std::endl;
    eCAL::Finalize();
}