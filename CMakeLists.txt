cmake_minimum_required(VERSION 3.16)
project(robot_interface2)

set(CMAKE_CXX_STANDARD 17)
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
option(BUILD_ROS_STUFF "Build ROS2 stuff" OFF)
option(BUILD_TESTING "Build tests" OFF)

find_package(Threads)

include(GNUInstallDirs)
include(cmake/lint-targets.cmake)

include(cmake/detect-raspi.cmake)
include(cmake/raspi_hardware.cmake)
include(cmake/xstypes.cmake)

find_package(urdfdom REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(eCAL REQUIRED)
find_package(robot_interface_protobuf REQUIRED)

add_library(robot_interface_utils src/utils/MessageConversion.cpp)
target_include_directories(
        robot_interface_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(robot_interface_utils PUBLIC robot_interface_protobuf raspi_hardware)

add_executable(motor_controller src/motor/MotorControllerExec.cpp src/motor/MotorController.cpp)
target_include_directories(
        motor_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller PUBLIC eCAL::core robot_interface_protobuf raspi_hardware robot_interface_utils)

add_executable(flysky_remote src/flysky/FlyskyReceiverExec.cpp src/flysky/FlyskyRemoteReceiver.cpp src/flysky/iBusParser.cpp)
target_include_directories(
        flysky_remote PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(flysky_remote PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)

add_executable(mti_receiver src/xsens/MTiParser.cpp src/xsens/MTiReceiver.cpp src/xsens/MTiReceiverExec.cpp)
target_include_directories(
        mti_receiver PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mti_receiver PUBLIC eCAL::core robot_interface_protobuf raspi_hardware xstypes)

find_package(hyq_cheetah)
add_executable(movement_controller src/MovementController.cpp)
target_include_directories(
        movement_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(movement_controller PUBLIC eCAL::core robot_interface_protobuf MainController
        RecoveryStandController)

add_executable(data_recorder src/DataRecorder.cpp)
target_include_directories(
        data_recorder PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(data_recorder PUBLIC eCAL::core robot_interface_protobuf
        nlohmann_json::nlohmann_json spdlog::spdlog fmt::fmt)

add_executable(robot_system_pub src/RobotSystemPub.cpp)
target_include_directories(
        robot_system_pub PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(robot_system_pub PUBLIC eCAL::core robot_interface_protobuf Eigen3::Eigen)

add_executable(motor_controller_test src/MotorControllerTest.cpp)
target_include_directories(
        motor_controller_test PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller_test PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)

add_executable(test_spi_dma src/testSpiDma.cpp)
target_include_directories(
        test_spi_dma PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(test_spi_dma PUBLIC raspi_hardware)

add_executable(test_spi_dma2 src/testSpiDma2.cpp)
target_include_directories(
        test_spi_dma2 PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(test_spi_dma2 PUBLIC raspi_hardware)

find_package(iir REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(state_estimator_pub src/stateEstimator/StateEstimatorExec.cpp src/stateEstimator/StateEstimator.cpp
        src/stateEstimator/TrapzIntegrator.hpp)
target_include_directories(
        state_estimator_pub PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(state_estimator_pub PUBLIC eCAL::core robot_interface_protobuf
        Eigen3::Eigen
        iir::iir_static)

install(
        TARGETS motor_controller flysky_remote mti_receiver motor_controller_test state_estimator_pub
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if (BUILD_ROS_STUFF)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    add_executable(ros_joint_state_to_cmd src/RosJointStateToCmdPub.cpp)
    ament_target_dependencies(ros_joint_state_to_cmd rclcpp std_msgs sensor_msgs)
    target_link_libraries(ros_joint_state_to_cmd eCAL::core robot_interface_protobuf)
    add_executable(ros_joint_state_to_cmd_pid src/RosJointStateToCmdPubPid.cpp)
    ament_target_dependencies(ros_joint_state_to_cmd_pid rclcpp std_msgs sensor_msgs)
    target_link_libraries(ros_joint_state_to_cmd_pid eCAL::core robot_interface_protobuf)
endif ()

if (BUILD_TESTING)
    add_subdirectory(test)
endif ()