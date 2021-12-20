#include "robot_interface2/stateEstimator/StateEstimator.hpp"
#include "TrapzIntegrator.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <iir/Butterworth.h>
#include <optional>
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

//(*(msg.mutable_base_orientation())) = latestImuMessage.base_orientation();
// auto *linVelPtr = msg.mutable_world_linear_velocity();
// linVelPtr->set_x(worldLinVel.x());
// linVelPtr->set_y(worldLinVel.y());
// linVelPtr->set_z(worldLinVel.z());
// auto *angVelPtr = msg.mutable_world_angular_velocity();
// angVelPtr->set_x(worldAngularVel.x());
// angVelPtr->set_y(worldAngularVel.y());
// angVelPtr->set_z(worldAngularVel.z());
// auto *linAccPtr = msg.mutable_world_linear_acceleration();
// linAccPtr->set_x(worldLinAcc.x());
// linAccPtr->set_y(worldLinAcc.y());
// linAccPtr->set_z(worldLinAcc.z());
struct ImuEstimatedState
{
    robot_interface::Quaternion baseOrientation;
    robot_interface::Vector3 worldLinVel;
    robot_interface::Vector3 worldAngVel;
    robot_interface::Vector3 worldLinAcc;
};

} // namespace

namespace robot_interface2
{

class StateEstimator::Impl
{
  public:
    Impl()
        : imuMessageSub_(std::make_unique<CSubscriber<robot_interface::ImuMessage>>("imu")),
          motorFeedbackSub_(std::make_unique<CSubscriber<robot_interface::MotorFeedbackMsg>>("motor_feedback")),
          stateEstimatorPub_(std::make_unique<CPublisher<robot_interface::StateEstimatorMessage>>("state_estimator"))
    {
        const double samplingRate = 100;     // Hz
        const double cutoffFrequency = 0.04; // Hz
        for (auto &filter : butterworthFilters_)
        {
            filter.setup(samplingRate, cutoffFrequency);
        }
        imuMessageSub_->AddReceiveCallback(
            [this](auto * /*topicName*/, const auto &msg, auto /*time*/, auto /*clock*/, auto /*id*/) {
                lastImuMessageTime_ = steady_clock::now();
                if (lastImuMessage_ == std::nullopt)
                {
                    lastImuMessage_ = msg;
                    return;
                }
                auto imuEstimatedState = ProcessImuData(msg);
                lastImuMessage_ = msg;
                std::lock_guard lock(imuDataMutex_);
                latestImuEstimatedState_ = imuEstimatedState;
            });
        motorFeedbackSub_->AddReceiveCallback(
            [this](auto * /*topicName*/, const auto &msg, auto /*time*/, auto /*clock*/, auto /*id*/) {
                std::lock_guard lock(motorDataMutex_);
                auto stateEstimatorMsg = robot_interface::StateEstimatorMessage();
                stateEstimatorMsg.set_message_id(msg.message_id());
                ProcessMotorData(stateEstimatorMsg, msg);
                if (steady_clock::now() - lastImuMessageTime_ > 50ms && latestImuEstimatedState_.has_value())
                {
                    std::cerr << "Stale imu message" << std::endl;
                    latestImuEstimatedState_ = std::nullopt;
                }
                if (latestImuEstimatedState_.has_value())
                {
                    *stateEstimatorMsg.mutable_base_orientation() = latestImuEstimatedState_->baseOrientation;
                    *stateEstimatorMsg.mutable_world_linear_acceleration() = latestImuEstimatedState_->worldLinAcc;
                    *stateEstimatorMsg.mutable_world_linear_velocity() = latestImuEstimatedState_->worldLinVel;
                    *stateEstimatorMsg.mutable_world_angular_velocity() = latestImuEstimatedState_->worldAngVel;
                }
                stateEstimatorPub_->Send(stateEstimatorMsg);
            });
    }
    ~Impl()
    {
    }

  private:
    ImuEstimatedState ProcessImuData(const robot_interface::ImuMessage &msg);
    bool ProcessMotorData(robot_interface::StateEstimatorMessage &msg,
                          const robot_interface::MotorFeedbackMsg &feedbackMsg);
    // eCAL pub and subs
    std::unique_ptr<CSubscriber<robot_interface::MotorFeedbackMsg>> motorFeedbackSub_{nullptr};
    std::unique_ptr<CSubscriber<robot_interface::ImuMessage>> imuMessageSub_{nullptr};
    std::unique_ptr<CPublisher<robot_interface::StateEstimatorMessage>> stateEstimatorPub_{nullptr};
    // Subscriber data
    std::optional<robot_interface::ImuMessage> latestImuMessage_{std::nullopt};
    std::optional<robot_interface::ImuMessage> lastImuMessage_{std::nullopt};
    std::optional<ImuEstimatedState> latestImuEstimatedState_{std::nullopt};
    std::chrono::time_point<std::chrono::steady_clock> lastImuMessageTime_{};
    // Thread management
    std::mutex imuDataMutex_;
    std::mutex motorDataMutex_;
    // Filtering data and objects
    std::array<Iir::Butterworth::HighPass<4>, 3> butterworthFilters_{};
    TrapzIntegrator<3> integrator_{};

    uint32_t msgCount_{0};
};

ImuEstimatedState StateEstimator::Impl::ProcessImuData(const robot_interface::ImuMessage &msg)
{
    // sampleTimeFine on imu is on 10kHz clock rate, and wraps around at 2^32, so will take 4.97 days
    // We assume that the imu will not be on for that long, so we always assume it will not wrap around
    auto imuSampleTimeDiff = msg.sample_time_ns() - lastImuMessage_->sample_time_ns();
    Eigen::Quaterniond quat(msg.base_orientation().w(), msg.base_orientation().x(), msg.base_orientation().y(),
                            msg.base_orientation().z());
    auto baseToWorldRotMat = quat.normalized().toRotationMatrix();

    Eigen::Vector3d baseAngularVel(msg.base_angular_velocity().x(), msg.base_angular_velocity().y(),
                                   msg.base_angular_velocity().z());
    Eigen::Vector3d worldAngularVel = baseToWorldRotMat * baseAngularVel;

    Eigen::Vector3d baseLinAcc(msg.base_linear_acceleration().x(), msg.base_linear_acceleration().y(),
                               msg.base_linear_acceleration().z());
    Eigen::Vector3d worldLinAcc = baseToWorldRotMat * baseLinAcc;
    Eigen::Vector3d worldLinAccFiltered = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++)
    {
        auto &butterworthFilter = butterworthFilters_.at(i);
        worldLinAccFiltered(i) = butterworthFilter.filter(worldLinAcc(i));
    }

    auto worldLinVel =
        integrator_.AddData(worldLinAccFiltered, static_cast<double>(imuSampleTimeDiff / 1'000'000) * 0.001);

    ImuEstimatedState temp{};
    temp.baseOrientation = msg.base_orientation();
    temp.worldLinVel.set_x(worldLinVel.x());
    temp.worldLinVel.set_y(worldLinVel.y());
    temp.worldLinVel.set_z(worldLinVel.z());
    temp.worldAngVel.set_x(worldAngularVel.x());
    temp.worldAngVel.set_y(worldAngularVel.y());
    temp.worldAngVel.set_z(worldAngularVel.z());
    temp.worldLinAcc.set_x(worldLinAccFiltered.x());
    temp.worldLinAcc.set_y(worldLinAccFiltered.y());
    temp.worldLinAcc.set_z(worldLinAccFiltered.z());
    return temp;
}

bool StateEstimator::Impl::ProcessMotorData(robot_interface::StateEstimatorMessage &msg,
                                            const robot_interface::MotorFeedbackMsg &feedbackMsg)
{
    // we use sorted map to guarantee our joint positions will always be in ascending order of motor id
    std::map<uint32_t, double> jointPosMap{}, jointVelMap{};
    for (const auto &feedback : feedbackMsg.feedbacks())
    {
        if (!feedback.ready())
            continue;
        jointPosMap[feedback.motor_id()] = feedback.angle();
        jointVelMap[feedback.motor_id()] = feedback.velocity();
    }
    if (jointPosMap.size() != 12)
    {
        std::cerr << "Latest motor feedback of id: " << feedbackMsg.message_id()
                  << " has less than 12 joint data, num ready: " << jointPosMap.size() << std::endl;
        std::cerr << "Ready: ";
        for (auto &pair : jointPosMap)
        {
            std::cerr << pair.first << ' ';
        }
        std::cerr << std::endl;
    }
    auto jointPosArrPtr = msg.mutable_joint_positions();
    auto jointVelArrPtr = msg.mutable_joint_velocities();
    auto jointIndexArrPtr = msg.mutable_joint_indexes();
    for (const auto &pair : jointPosMap)
    {
        jointIndexArrPtr->Add(pair.first);
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