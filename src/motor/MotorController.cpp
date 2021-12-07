#include "robot_interface2/motor/MotorController.hpp"
#include "raspiHardware/SPI.hpp"
#include "raspiHardware/SimultaneousSPI.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include <Eigen/Core>
#include <chrono>
#include <cstring>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_interface_protobuf/motor_cmd_msg.pb.h>
#include <robot_interface_protobuf/motor_feedback_msg.pb.h>
#include <thread>
using namespace std::chrono;
using namespace std::chrono_literals;
using eCAL::protobuf::CPublisher;
using eCAL::protobuf::CSubscriber;
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
class MotorController::Impl
{
  public:
    Impl(std::shared_ptr<RpiSPIDriver> &&spi);
    ~Impl();

  private:
    void RunLoop();
    void SetConfig();
    std::unique_ptr<CSubscriber<robot_interface::MotorCmdMsg>> motorCommandSub_;
    std::unique_ptr<CPublisher<robot_interface::MotorFeedbackMsg>> motorFeedbackPub_;
    void OnCmdMsg(const char *topic_name_, const robot_interface::MotorCmdMsg &msg, long long time_, long long clock_);
    time_point<steady_clock> lastCmdMsgTime_{};
    std::shared_ptr<RpiSPIDriver> spi_;
    std::thread runThread_;
    std::mutex timeMutex_, spiMutex_;
    std::atomic<bool> stopRequested_{false};
};

MotorController::Impl::Impl(std::shared_ptr<RpiSPIDriver> &&spi)
    : motorCommandSub_(std::make_unique<CSubscriber<robot_interface::MotorCmdMsg>>("motor_cmd")),
      motorFeedbackPub_(std::make_unique<CPublisher<robot_interface::MotorFeedbackMsg>>("motor_feedback")),
      spi_(spi)
{
    lastCmdMsgTime_ = steady_clock::now();
    auto a = [this](auto *topicName, const auto &msg, auto time, auto clock, auto id) {
        {
            std::lock_guard lock(timeMutex_);
            auto now = steady_clock::now();
            auto timeDiffMs = duration_cast<milliseconds>(now - lastCmdMsgTime_).count();
            lastCmdMsgTime_ = steady_clock::now();
            if(timeDiffMs >= 1000) // This means this is the first message in a while, so we ignore the first one
                return;

        }
        OnCmdMsg(topicName, msg, time, clock);
    };
    SetConfig();
    motorCommandSub_->AddReceiveCallback(a);
    runThread_ = std::thread([&]() { RunLoop(); });
}

MotorController::Impl::~Impl()
{
    stopRequested_ = true;
    runThread_.join();
}

void MotorController::Impl::OnCmdMsg(const char *topic_name_, const robot_interface::MotorCmdMsg &msg,
                                     long long int time_, long long int clock_)
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
    {
        std::lock_guard lock(spiMutex_);
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
            std::memcpy(message.MotorCommands.data(), &motorCommandSingles.at(6), sizeof(MotorCommandSingle) * 6);
            message.GenerateCRC();
            spi_->AddTransceiveData(6, span<uint8_t>(reinterpret_cast<uint8_t *>(&message), sizeof(message)),
                                    span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
        }
        spi_->Transceive();
    }
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

void MotorController::Impl::RunLoop()
{

    auto next = steady_clock::now() + 50ms;
    while (!stopRequested_)
    {
        std::this_thread::sleep_until(next);
        if (stopRequested_)
            return;
        next = next + duration<int64_t, std::ratio<1, 50>>{1};
        uint64_t timeDiffMs;
        {
            std::lock_guard lock(timeMutex_);
            timeDiffMs = duration_cast<milliseconds>(steady_clock::now() - lastCmdMsgTime_).count();
        }
        if (timeDiffMs < 1000)
        {
            continue;
        }
        MotorFeedbackFull frontFeedback, hindFeedback;
        {
            std::lock_guard lock(spiMutex_);
            constexpr MotorCommandSingle readCmd =
                MotorCommandSingle{.CommandType = MotorCommandType::Read, .reserved = 0, .Param = {}, .reserved2 = 0};
            CommandMessage message{.messageType = MessageType::CommandMessage,
                                   .MessageId = 1,
                                   .MotorCommands = {readCmd, readCmd, readCmd, readCmd, readCmd, readCmd}};
            message.GenerateCRC();
            spi_->AddTransceiveData(0, span<uint8_t>(reinterpret_cast<uint8_t *>(&message), sizeof(message)),
                                    span<uint8_t>(reinterpret_cast<uint8_t *>(&frontFeedback), sizeof(frontFeedback)));

            CommandMessage message2 = message;
            spi_->AddTransceiveData(6, span<uint8_t>(reinterpret_cast<uint8_t *>(&message2), sizeof(message2)),
                                    span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
            spi_->Transceive();
        }

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
        motorFeedbackPub_->Send(feedbackMsg);
    }
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

void MotorController::Impl::SetConfig()
{
    std::array<double, 12> torqueMultipliers{1.3, 0.85, 0.7, 1.2, 0.9, 0.7, 1.43, 0.89, 0.68, 1.34, 0.92, 0.65};
    std::array<double, 12> gearRatios{-1.0, 1.0, 1.08, -1.0, -1.0, -1.08, 1.0, 1.0, 1.08, 1.0, -1.0, 1.08};
    Eigen::Matrix<double, 12, 1> lowerLimitsDeg =
        (Eigen::Matrix<double, 12, 1>() << -45, -90, -135, -45, -90, -135, -45, -90, -135, -45, -90, -135).finished();
    Eigen::Matrix<double, 12, 1> lowerLimitsRad = lowerLimitsDeg * M_PI / 180.0;
    Eigen::Matrix<double, 12, 1> upperLimitsRad = lowerLimitsRad * -1;

    auto [frontGearRatioConfigMsg, hindGearRatioConfigMsg] =
        GenerateConfigMessage(MotorConfigType::GearRatio, {gearRatios.data(), gearRatios.size()});
    auto [frontTorqueMultConfigMsg, hindTorqueMultConfigMsg] = GenerateConfigMessage(
        MotorConfigType::SetTorqueMultiplier, {torqueMultipliers.data(), torqueMultipliers.size()});
    auto [frontLowerLimConfigMsg, hindLowerLimConfigMsg] = GenerateConfigMessage(
        MotorConfigType::LowerJointLimit, {lowerLimitsRad.data(), static_cast<uint32_t>(lowerLimitsRad.size())});
    auto [frontUpperLimConfigMsg, hindUpperLimConfigMsg] = GenerateConfigMessage(
        MotorConfigType::UpperJointLimit, {upperLimitsRad.data(), static_cast<uint32_t>(upperLimitsRad.size())});
    MotorFeedbackFull frontFeedback, hindFeedback;

    auto transceive = [&](const ConfigMessage &frontConfig, const ConfigMessage &hindConfig) {
        spi_->AddTransceiveData(
            0, span<const uint8_t>(reinterpret_cast<const uint8_t *>(&frontConfig), sizeof(frontConfig)),
            span<uint8_t>(reinterpret_cast<uint8_t *>(&frontFeedback), sizeof(frontFeedback)));
        spi_->AddTransceiveData(6,
                                span<const uint8_t>(reinterpret_cast<const uint8_t *>(&hindConfig), sizeof(hindConfig)),
                                span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
        spi_->Transceive();
        std::this_thread::sleep_for(5ms);
    };

    transceive(frontGearRatioConfigMsg, hindGearRatioConfigMsg);
    transceive(frontTorqueMultConfigMsg, frontTorqueMultConfigMsg);
    transceive(frontUpperLimConfigMsg, hindUpperLimConfigMsg);
    transceive(frontLowerLimConfigMsg, hindLowerLimConfigMsg);
}

MotorController::MotorController(std::shared_ptr<RpiSPIDriver> spi)
{
    impl_ = std::make_unique<Impl>(std::move(spi));
}

MotorController::~MotorController() = default;

} // namespace robot_interface2