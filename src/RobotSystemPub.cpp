#include <Eigen/Core>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <optional>
#include <robot_interface_protobuf/robot_system_message.pb.h>
#include <robot_interface_protobuf/state_estimator_message.pb.h>

namespace
{
const double hip = 0.06;
const double thigh = 0.201;
const double knee = 0.22;
const double xBodyFrame = 0.2235;
const double yBodyFrame = 0.0659;
inline Eigen::Matrix3d ComputeLegJacobian(const Eigen::Vector3d &q, int index)
{
    Eigen::Matrix3d Jacobian = Eigen::Matrix3d::Zero();
    double leftRightSign = (index == 0 || index == 2) ? 1.0 : -1.0;

    double s1 = std::sin(q(0));
    double s2 = std::sin(q(1));
    double s3 = std::sin(q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(q(1));
    double c3 = std::cos(q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    Jacobian(0, 0) = 0.0;
    Jacobian(0, 1) = knee * c23 + thigh * c2;
    Jacobian(0, 2) = knee * c23;
    Jacobian(1, 0) = knee * c1 * c23 + thigh * c1 * c2 - hip * leftRightSign * s1;
    Jacobian(1, 1) = -knee * s1 * s23 - thigh * s1 * s2;
    Jacobian(1, 2) = -knee * s1 * s23;
    Jacobian(2, 0) = knee * s1 * c23 + thigh * c2 * s1 + hip * leftRightSign * c1;
    Jacobian(2, 1) = knee * c1 * s23 + thigh * c1 * s2;
    Jacobian(2, 2) = knee * c1 * s23;

    return Jacobian;
}

inline Eigen::Vector3d ComputeLegPositionFromCom(const Eigen::Vector3d &q, int index)
{
    Eigen::Vector3d Position = Eigen::Vector3d::Zero();
    double leftRightSign = (index == 0 || index == 2) ? 1.0 : -1.0;
    double frontBackSign = (index == 0 || index == 1) ? 1.0 : -1.0; // front = 1, back = -1

    double s1 = std::sin(q(0));
    double s2 = std::sin(q(1));
    double s3 = std::sin(q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(q(1));
    double c3 = std::cos(q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    Position(0) = knee * s23 + thigh * s2 + frontBackSign * xBodyFrame;
    Position(1) = hip * leftRightSign * c1 + knee * (s1 * c23) + thigh * c2 * s1 + leftRightSign * yBodyFrame;
    Position(2) = hip * leftRightSign * s1 - knee * (c1 * c23) - thigh * c1 * c2;
    return Position;
}

template <typename Type, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
robot_interface::Matrix eigenMatToProtobuf(const Eigen::Matrix<Type, Rows, Cols> &eigenMat)
{
    robot_interface::Matrix pbMat;
    *pbMat.mutable_data() = {eigenMat.data(), eigenMat.data() + eigenMat.size()};
    pbMat.set_nrows(eigenMat.rows());
    pbMat.set_ncols(eigenMat.cols());
    return pbMat;
}

} // namespace

using namespace eCAL::protobuf;
using namespace std::chrono;
using namespace std::chrono_literals;
class SomeClass
{
  public:
    SomeClass();

  private:
    // Pub subs
    std::unique_ptr<CSubscriber<robot_interface::StateEstimatorMessage>> stateEstimatorSub_;
    std::unique_ptr<CPublisher<robot_interface::RobotSystemMessage>> robotSystemPub_;
    // Subscriber data
    std::optional<robot_interface::StateEstimatorMessage> latestStateEstimatorMessage_{std::nullopt};
    std::chrono::time_point<std::chrono::steady_clock> lastStateEstimatorMessageTime_{};
    void OnStateEstimatorMsg(const char *topic_name_, const robot_interface::StateEstimatorMessage &msg,
                             long long time_, long long clock_);
};
SomeClass::SomeClass()
    : stateEstimatorSub_(std::make_unique<CSubscriber<robot_interface::StateEstimatorMessage>>("state_estimator")),
      robotSystemPub_(std::make_unique<CPublisher<robot_interface::RobotSystemMessage>>("robot_system"))
{
}

void SomeClass::OnStateEstimatorMsg(const char * /*topic_name_*/, const robot_interface::StateEstimatorMessage &msg,
                                    long long /*time_*/, long long /*clock_*/)
{
    if (msg.joint_positions().size() != 12)
    {
        std::cerr << "Num joint pos not 12" << std::endl;
        return;
    }
    std::array<Eigen::Vector3d, 4> legJointPos{
        Eigen::Vector3d(msg.joint_positions(0), msg.joint_positions(1), msg.joint_positions(2)),
        Eigen::Vector3d(msg.joint_positions(3), msg.joint_positions(4), msg.joint_positions(5)),
        Eigen::Vector3d(msg.joint_positions(6), msg.joint_positions(7), msg.joint_positions(8)),
        Eigen::Vector3d(msg.joint_positions(9), msg.joint_positions(10), msg.joint_positions(11)),
    };
    robot_interface::RobotSystemMessage robotSystemMsg;
    auto footPosProtoArr = robotSystemMsg.mutable_foot_positions();
    auto footJacProtoArr = robotSystemMsg.mutable_foot_jacobians();
    for (size_t i = 0; i < 4; i++)
    {
        auto pos = ComputeLegPositionFromCom(legJointPos.at(i), i);
        auto footPosProtoVec = footPosProtoArr->Add();
        footPosProtoVec->set_x(pos.x());
        footPosProtoVec->set_y(pos.y());
        footPosProtoVec->set_z(pos.z());
        auto jac = ComputeLegJacobian(legJointPos.at(i), i);
        auto footJacProtoMat = footJacProtoArr->Add();
        *footJacProtoMat = eigenMatToProtobuf(jac);
    }
    robotSystemPub_->Send(robotSystemMsg);
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
    eCAL::Initialize({}, "Robot system publisher");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot system publisher");

    SomeClass a;
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}