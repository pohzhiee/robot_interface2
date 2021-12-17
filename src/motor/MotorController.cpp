#include "robot_interface2/motor/MotorController.hpp"
#include "raspiHardware/SPI.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include "robot_interface2/utils/Common.hpp"
#include "robot_interface2/utils/MessageConversion.hpp"
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
    uint32_t idleMessageId{0};
};

MotorController::Impl::Impl(std::shared_ptr<RpiSPIDriver> &&spi)
    : motorCommandSub_(std::make_unique<CSubscriber<robot_interface::MotorCmdMsg>>("motor_cmd")),
      motorFeedbackPub_(std::make_unique<CPublisher<robot_interface::MotorFeedbackMsg>>("motor_feedback")), spi_(spi)
{
    lastCmdMsgTime_ = steady_clock::now();
    auto a = [this](auto *topicName, const auto &msg, auto time, auto clock, auto id) {
        {
            std::lock_guard lock(timeMutex_);
            auto now = steady_clock::now();
            auto timeDiffMs = duration_cast<milliseconds>(now - lastCmdMsgTime_).count();
            lastCmdMsgTime_ = steady_clock::now();
            if (timeDiffMs >= 999) // This means this is the first message in a while, so we ignore the first one
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
    auto cmdMsg = ProtobufCmdMsgToControllerCmdMsg(msg);
    if (!cmdMsg.has_value())
        return;

    std::array<uint8_t, sizeof(CommandMessage)> frontMsg = BytesFrom(cmdMsg->at(0)), hindMsg = BytesFrom(cmdMsg->at(1));
    MotorFeedbackFull frontFeedback, hindFeedback;
    spi_->AddTransceiveData(0, span<const uint8_t>(frontMsg.data(), frontMsg.size()),
                            span<uint8_t>(reinterpret_cast<uint8_t *>(&frontFeedback), sizeof(frontFeedback)));
    spi_->AddTransceiveData(6, span<const uint8_t>(hindMsg.data(), hindMsg.size()),
                            span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
    spi_->Transceive();

    auto feedbackProto = FeedbackToProto(frontFeedback, hindFeedback);
    motorFeedbackPub_->Send(feedbackProto);
}

void MotorController::Impl::RunLoop()
{

    auto next = steady_clock::now() + 50ms;
    while (!stopRequested_)
    {
        std::this_thread::sleep_until(next);
        if (stopRequested_)
            return;
        next = next + duration<int64_t, std::ratio<1, 500>>{1};
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
                                   .MessageId = idleMessageId,
                                   .MotorCommands = {readCmd, readCmd, readCmd, readCmd, readCmd, readCmd}};
            idleMessageId++;
            message.GenerateCRC();
            spi_->AddTransceiveData(0, span<uint8_t>(reinterpret_cast<uint8_t *>(&message), sizeof(message)),
                                    span<uint8_t>(reinterpret_cast<uint8_t *>(&frontFeedback), sizeof(frontFeedback)));

            CommandMessage message2 = message;
            spi_->AddTransceiveData(6, span<uint8_t>(reinterpret_cast<uint8_t *>(&message2), sizeof(message2)),
                                    span<uint8_t>(reinterpret_cast<uint8_t *>(&hindFeedback), sizeof(hindFeedback)));
            spi_->Transceive();
        }
        auto feedbackProto = FeedbackToProto(frontFeedback, hindFeedback);
        motorFeedbackPub_->Send(feedbackProto);
    }
}


void MotorController::Impl::SetConfig()
{
    std::array<double, 12> torqueMultipliers{1.3, 0.85, 0.7, 1.2, 0.9, 0.7, 1.43, 0.89, 0.68, 1.34, 0.92, 0.65};
    std::array<double, 12> gearRatios{-1.0, 1.0, 1.08, -1.0, -1.0, -1.08, 1.0, 1.0, 1.08, 1.0, -1.0, -1.08};
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