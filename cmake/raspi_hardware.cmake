add_library(raspi_hardware ${CMAKE_SOURCE_DIR}/src/raspiHardware/mailbox.c
        ${CMAKE_SOURCE_DIR}/src/raspiHardware/SimultaneousSPI.cpp
        ${CMAKE_SOURCE_DIR}/src/raspiHardware/UARTDMAReader.cpp
        ${CMAKE_SOURCE_DIR}/src/raspiHardware/UncachedMem.cpp)
target_link_libraries(raspi_hardware PUBLIC Threads::Threads
        )
target_include_directories(raspi_hardware PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(raspi_hardware PROPERTIES
        POSITION_INDEPENDENT_CODE ON)

install(
        TARGETS raspi_hardware
        EXPORT raspi_hardwareTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT raspi_hardwareTargets
        NAMESPACE robot_interface2::
        DESTINATION "${CMAKE_INSTALL_DATADIR}/robot_interface2"
        )