#ifndef ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#define ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#include <memory>
#include <cstdint>
namespace robot_interface2
{
class MotorController
{
  public:
    class Impl;
    MotorController(uintptr_t gpioMmapPtr, uintptr_t spiMmapPtr);
    ~MotorController();
    std::unique_ptr<Impl> impl_;
};
} // namespace robot_interface2
#endif // ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
