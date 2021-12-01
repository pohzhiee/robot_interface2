#ifndef ROBOT_INTERFACE2_TRAPZINTEGRATOR_HPP
#define ROBOT_INTERFACE2_TRAPZINTEGRATOR_HPP
#include <Eigen/Core>

template <int N> class TrapzIntegrator
{
  public:
    TrapzIntegrator() = default;
    inline Eigen::Matrix<double, N, 1> AddData(Eigen::Matrix<double, N, 1> data, double dt)
    {
        if (isFirst_)
        {
            isFirst_ = false;
            prevData_ = data;
            return Eigen::Matrix<double, N, 1>::Zero();
        }
        prevAccumulated_ += (prevData_ + data) * dt * 0.5;
        prevData_ = data;
        return prevAccumulated_;
    }

  private:
    Eigen::Matrix<double, N, 1> prevAccumulated_{};
    Eigen::Matrix<double, N, 1> prevData_{};
    bool isFirst_{true};
};
#endif // ROBOT_INTERFACE2_TRAPZINTEGRATOR_HPP
