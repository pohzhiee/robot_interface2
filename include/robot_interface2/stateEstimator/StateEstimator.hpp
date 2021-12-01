#ifndef ROBOT_INTERFACE2_STATEESTIMATOR_HPP
#define ROBOT_INTERFACE2_STATEESTIMATOR_HPP
#include <memory>
namespace robot_interface2
{
class StateEstimator
{
    class Impl;

  public:
    StateEstimator();
    ~StateEstimator();

  private:
    std::unique_ptr<Impl> impl_;
};
} // namespace robot_interface2
#endif // ROBOT_INTERFACE2_STATEESTIMATOR_HPP
