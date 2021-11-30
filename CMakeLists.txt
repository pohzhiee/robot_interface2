cmake_minimum_required(VERSION 3.16)
project(robot_interface2)

set(CMAKE_CXX_STANDARD 17)
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

find_package(Threads)

include(GNUInstallDirs)
include(cmake/raspi_hardware.cmake)
include(cmake/xstypes.cmake)

find_package(eCAL REQUIRED)
find_package(robot_interface_protobuf REQUIRED)
add_executable(motor_controller src/MotorControllerExec.cpp src/motor/MotorController.cpp)
target_include_directories(
        motor_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)

add_executable(flysky_remote src/FlyskyReceiverExec.cpp src/Flysky/FlyskyRemoteReceiver.cpp src/Flysky/iBusParser.cpp)
target_include_directories(
        flysky_remote PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(flysky_remote PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)

add_executable(motor_controller_test src/MotorControllerTest.cpp)
target_include_directories(
        motor_controller_test PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller_test PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)