cmake_minimum_required(VERSION 3.16)
project(robot_interface2)

set(CMAKE_CXX_STANDARD 17)
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

find_package(Threads)

include(GNUInstallDirs)
include(cmake/lint-targets.cmake)

include(cmake/raspi_hardware.cmake)
include(cmake/xstypes.cmake)

find_package(eCAL REQUIRED)
find_package(robot_interface_protobuf REQUIRED)
add_executable(motor_controller src/motor/MotorControllerExec.cpp src/motor/MotorController.cpp)
target_include_directories(
        motor_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)

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
target_link_libraries(movement_controller PUBLIC eCAL::core robot_interface_protobuf MainController)


add_executable(motor_controller_test src/MotorControllerTest.cpp)
target_include_directories(
        motor_controller_test PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(motor_controller_test PUBLIC eCAL::core robot_interface_protobuf raspi_hardware)


find_package(iir REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(state_estimator_pub src/stateEstimator/StateEstimatorExec.cpp src/stateEstimator/StateEstimator.cpp src/stateEstimator/TrapzIntegrator.hpp)
target_include_directories(
        state_estimator_pub PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(state_estimator_pub PUBLIC eCAL::core robot_interface_protobuf
        Eigen3::Eigen
        iir::iir_static)

install(
        TARGETS motor_controller flysky_remote mti_receiver motor_controller_test state_estimator_pub
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})