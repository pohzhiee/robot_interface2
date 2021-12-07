cmake_minimum_required(VERSION 3.14)

project(robot_interface2_tests LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(GTest REQUIRED)
#find_package(robot_interface2 REQUIRED)
enable_testing()

set(some_sources ${CMAKE_SOURCE_DIR}/src/motor/MotorController.cpp)

add_executable(robot_interface2_test src/test1.cpp
        src/MessageConversionTest.cpp
        ${some_sources})
target_link_libraries(robot_interface2_test PRIVATE GTest::GTest GTest::Main
        raspi_hardware robot_interface_protobuf robot_interface_utils
        eCAL::core)
target_include_directories(robot_interface2_test PRIVATE "source" ${CMAKE_SOURCE_DIR}/include)

add_test(NAME robot_interface2_test COMMAND robot_interface2_test)
gtest_discover_tests(robot_interface2_test)