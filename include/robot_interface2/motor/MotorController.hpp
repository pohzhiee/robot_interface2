#ifndef ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#define ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
#include <memory>
#include <cstdint>
class RpiSPIDriver;
namespace robot_interface2
{
class MotorController
{
  public:
    class Impl;
    MotorController(std::shared_ptr<RpiSPIDriver> spi);
    ~MotorController();
    std::unique_ptr<Impl> impl_;
};
} // namespace robot_interface2
#endif // ROBOT_INTERFACE2_MOTORCONTROLLER_HPP
