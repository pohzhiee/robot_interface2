include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(package robot_interface2)

install(
        TARGETS xstypes_lib
        EXPORT robotInterface_xstypes
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})


install(
    TARGETS robotInterface_robotInterface
    EXPORT robotInterfaceTargets
    RUNTIME #
    COMPONENT robotInterface_Runtime
    LIBRARY #
    COMPONENT robotInterface_Runtime
    NAMELINK_COMPONENT robotInterface_Development
    ARCHIVE #
    COMPONENT robotInterface_Development
    INCLUDES #
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

write_basic_package_version_file(
    "${package}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

# Allow package maintainers to freely override the path for the configs
set(
    robotInterface_INSTALL_CMAKEDIR "${CMAKE_INSTALL_DATADIR}/${package}"
    CACHE PATH "CMake package config location relative to the install prefix"
)
mark_as_advanced(robotInterface_INSTALL_CMAKEDIR)

install(
    FILES cmake/install-config.cmake
    DESTINATION "${robotInterface_INSTALL_CMAKEDIR}"
    RENAME "${package}Config.cmake"
    COMPONENT robotInterface_Development
)

install(
    FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
    DESTINATION "${robotInterface_INSTALL_CMAKEDIR}"
    COMPONENT robotInterface_Development
)

install(
    EXPORT robotInterfaceTargets
    NAMESPACE robotInterface::
    DESTINATION "${robotInterface_INSTALL_CMAKEDIR}"
    COMPONENT robotInterface_Development
)

install(EXPORT robotInterface_xstypes
        NAMESPACE robotInterface::
        DESTINATION "${robotInterface_INSTALL_CMAKEDIR}"
        COMPONENT robotInterface_xstypes
        )

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
