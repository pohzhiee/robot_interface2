#include "gtest/gtest.h"
#include <string>
#include "robot_interface2/motor/MotorController.hpp"
#include "dummies/dummyRpiSPIDriver.hpp"

namespace
{
TEST(SomeTest, TestName)
{
//    auto spi = std::make_shared<robot_interface2_test::DummyRpiSPIDriver>();
    std::string something = "robot_interface2";
    EXPECT_EQ(std::string("robot_interface2"), something);
}


} // namespace
