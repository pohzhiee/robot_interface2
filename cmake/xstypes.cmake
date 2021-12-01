file(GLOB xstypes_src
        LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/src/xstypes/*.c" "${CMAKE_SOURCE_DIR}/src/xstypes/*.cpp")
add_library(xstypes STATIC ${xstypes_src})
target_include_directories(
        xstypes PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/xstypes>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/xstypes>)

install(
        TARGETS xstypes
        EXPORT xstypesTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT xstypesTargets
        NAMESPACE robot_interface2::
        DESTINATION "${CMAKE_INSTALL_DATADIR}/robot_interface2"
        )