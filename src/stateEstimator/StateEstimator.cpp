#include "robot_interface2/stateEstimator/StateEstimator.hpp"
#include "TrapzIntegrator.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <iir/Butterworth.h>
#include <robot_interface_protobuf/imu_message.pb.h>
#include <robot_interface_protobuf/motor_feedback_msg.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>
#include <thread>
using namespace std::chrono_literals;
using namespace std::chrono;
using eCAL::protobuf::CPublisher;
using eCAL::protobuf::CSubscriber;

namespace
{
robot_interface::StateEstimatorMessage FillMessage(uint32_t id, const robot_interface::Quaternion &orientation,
                                                   const Eigen::Vector3d &worldLinVel,
                                                   const Eigen::Vector3d &worldAngularVel,
                                                   const Eigen::Vector3d &worldLinAcc)
{
    robot_interface::StateEstimatorMessage msg;
    msg.set_real_time_ns(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
    return msg;
}
} // namespace

namespace robot_interface2
{

class StateEstimator::Impl
{
  public:
    Impl()
        : imuMessageSub_(std::make_unique<CSubscriber<robot_interface::ImuMessage>>("imu")),
          motorFeedbackSub_(std::make_unique<CSubscriber<robot_interface::MotorFeedbackMsg>>("motor_feedback")),
          stateEstimatorPub_(std::make_unique<CPublisher<robot_interface::StateEstimatorMessage>>("state_estimator")),
          runThread_([&]() { RunFunction(); })
    {
        imuMessageSub_->AddReceiveCallback(
            [this](auto * /*topicName*/, const auto &msg, auto /*time*/, auto /*clock*/, auto /*id*/) {
                std::lock_guard lock(imuDataMutex_);
                lastImuMessage_ = latestImuMessage_;
                latestImuMessage_ = msg;
                lastImuMessageTime_ = steady_clock::now();
            });
        motorFeedbackSub_->AddReceiveCallback(
            [this](auto * /*topicName*/, const auto &msg, auto /*time*/, auto /*clock*/, auto /*id*/) {
                std::lock_guard lock(motorDataMutex_);
                latestMotorFeedback_ = msg;
                lastMotorFeedbackTime_ = steady_clock::now();
            });
        const double samplingRate = 100;     // Hz
        const double cutoffFrequency = 0.04; // Hz
        for (auto &filter : butterworthFilters_)
        {
            filter.setup(samplingRate, cutoffFrequency);
        }
    }
    ~Impl()
    {
        stopRequested_ = true;
        runThread_.join();
    }

  private:
    void RunFunction();
    bool ProcessImuData(robot_interface::StateEstimatorMessage &msg);
    bool ProcessMotorData(robot_interface::StateEstimatorMessage &msg);
    // eCAL pub and subs
    std::unique_ptr<CSubscriber<robot_interface::MotorFeedbackMsg>> motorFeedbackSub_{nullptr};
    std::unique_ptr<CSubscriber<robot_interface::ImuMessage>> imuMessageSub_{nullptr};
    std::unique_ptr<CPublisher<robot_interface::StateEstimatorMessage>> stateEstimatorPub_{nullptr};
    // Subscriber data
    std::optional<robot_interface::MotorFeedbackMsg> latestMotorFeedback_{std::nullopt};
    std::optional<robot_interface::ImuMessage> latestImuMessage_{std::nullopt};
    std::optional<robot_interface::ImuMessage> lastImuMessage_{std::nullopt};
    std::chrono::time_point<std::chrono::steady_clock> lastImuMessageTime_{};
    std::chrono::time_point<std::chrono::steady_clock> lastMotorFeedbackTime_{};
    // Thread management
    std::mutex imuDataMutex_;
    std::mutex motorDataMutex_;
    std::thread runThread_;
    std::atomic<bool> stopRequested_{false};
    // Filtering data and objects
    std::array<Iir::Butterworth::HighPass<4>, 3> butterworthFilters_{};
    TrapzIntegrator<3> integrator_{};

    uint32_t msgCount_{0};
};
void StateEstimator::Impl::RunFunction()
{
    auto next = steady_clock::now() + 5ms;
    while (!stopRequested_)
    {
        auto now = steady_clock::now();
        if (now > next)
        {
            next = now;
        }
        std::this_thread::sleep_until(next);
        next = next + duration<int64_t, std::ratio<1, 200>>{1};
        auto msg = robot_interface::StateEstimatorMessage();
        bool imuResult = ProcessImuData(msg);
        bool motorResult = ProcessMotorData(msg);
        if (!imuResult && !motorResult)
        { // When both imu and motor data are not ready
            continue;
        }
        stateEstimatorPub_->Send(msg);
    }
}
bool StateEstimator::Impl::ProcessImuData(robot_interface::StateEstimatorMessage &msg)
{
    robot_interface::ImuMessage latestImuMessage;
    uint64_t imuSampleTimeDiff;
    {
        std::lock_guard lock(imuDataMutex_);
        if (!lastImuMessage_.has_value())
            return false;
        if (steady_clock::now() - lastImuMessageTime_ > 500ms)
        {
            std::cerr << "More than 500ms since last imu data" << std::endl;
            lastImuMessage_ = std::nullopt;
            latestImuMessage_ = std::nullopt;
            return false;
        }
        assert(latestImuMessage_.has_value());

        lastImuMessage_.value();
        latestImuMessage = latestImuMessage_.value();
        imuSampleTimeDiff = latestImuMessage.sample_time_ns() - lastImuMessage_->sample_time_ns();
    }
    Eigen::Vector3d baseLinAcc(latestImuMessage.base_linear_acceleration().x(),
                               latestImuMessage.base_linear_acceleration().y(),
                               latestImuMessage.base_linear_acceleration().z());
    Eigen::Vector3d baseAngularVel(latestImuMessage.base_angular_velocity().x(),
                                   latestImuMessage.base_angular_velocity().y(),
                                   latestImuMessage.base_angular_velocity().z());
    Eigen::Quaterniond quat(latestImuMessage.base_orientation().w(), latestImuMessage.base_orientation().x(),
                            latestImuMessage.base_orientation().y(), latestImuMessage.base_orientation().z());
    auto baseToWorldRotMat = quat.normalized().toRotationMatrix();
    Eigen::Vector3d worldLinAcc = baseToWorldRotMat * baseLinAcc;
    Eigen::Vector3d worldLinAccFiltered = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++)
    {
        auto &butterworthFilter = butterworthFilters_.at(i);
        worldLinAccFiltered(i) = butterworthFilter.filter(worldLinAcc(i));
    }
    auto worldLinVel = integrator_.AddData(worldLinAccFiltered, static_cast<double>(imuSampleTimeDiff / 1'000'000) * 0.001);
    Eigen::Vector3d worldAngularVel = baseToWorldRotMat * baseAngularVel;

    msg.set_message_id(msgCount_);
    msgCount_++;
    (*(msg.mutable_base_orientation())) = latestImuMessage.base_orientation();
    auto *linVelPtr = msg.mutable_world_linear_velocity();
    linVelPtr->set_x(worldLinVel.x());
    linVelPtr->set_y(worldLinVel.y());
    linVelPtr->set_z(worldLinVel.z());
    auto *angVelPtr = msg.mutable_world_angular_velocity();
    angVelPtr->set_x(worldAngularVel.x());
    angVelPtr->set_y(worldAngularVel.y());
    angVelPtr->set_z(worldAngularVel.z());
    auto *linAccPtr = msg.mutable_world_linear_acceleration();
    linAccPtr->set_x(worldLinAcc.x());
    linAccPtr->set_y(worldLinAcc.y());
    linAccPtr->set_z(worldLinAcc.z());
    return true;
}
bool StateEstimator::Impl::ProcessMotorData(robot_interface::StateEstimatorMessage &msg)
{
    robot_interface::MotorFeedbackMsg latestMotorFeedback;
    {
        std::lock_guard lock(motorDataMutex_);
        if (!latestMotorFeedback_.has_value())
            return false;
        if (steady_clock::now() - lastMotorFeedbackTime_ > 500ms)
        {
            std::cerr << "More than 500ms since last motor feedback" << std::endl;
            latestMotorFeedback_ = std::nullopt;
            return false;
        }
        latestMotorFeedback = latestMotorFeedback_.value();
    }
    // we use sorted map to guarantee our joint positions will always be in ascending order of motor id
    std::map<uint32_t, double> jointPosMap{}, jointVelMap{};
    for (const auto &feedback : latestMotorFeedback.feedbacks())
    {
        if (!feedback.ready())
            continue;
        jointPosMap.at(feedback.motor_id()) = feedback.angle();
        jointVelMap.at(feedback.motor_id()) = feedback.velocity();
    }
    if (jointPosMap.size() != 12)
    {
        std::cerr << "Latest motor feedback has less than 12 joint data, some motors not ready, num ready: "
                  << jointPosMap.size() << std::endl;
        return false;
    }
    auto jointPosArrPtr = msg.mutable_joint_positions();
    auto jointVelArrPtr = msg.mutable_joint_velocities();
    for (const auto &pair : jointPosMap)
    {
        jointPosArrPtr->Add(pair.second);
    }
    for (const auto &pair : jointVelMap)
    {
        jointVelArrPtr->Add(pair.second);
    }
    return true;
}

StateEstimator::StateEstimator() : impl_(std::make_unique<StateEstimator::Impl>())
{
}
StateEstimator::~StateEstimator() = default;

} // namespace robot_interface2