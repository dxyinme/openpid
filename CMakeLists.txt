cmake_minimum_required(VERSION 3.16)

project(openpid VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(OPENPID_IS_TOP_LEVEL ON)
else()
    set(OPENPID_IS_TOP_LEVEL OFF)
endif()

option(OPENPID_BUILD_TESTS "Build the OpenPID unit tests" ${OPENPID_IS_TOP_LEVEL})
option(OPENPID_BUILD_EXAMPLES "Build the OpenPID examples" ${OPENPID_IS_TOP_LEVEL})

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(openpid INTERFACE)
add_library(openpid::openpid ALIAS openpid)

target_include_directories(openpid INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(openpid INTERFACE cxx_std_17)

install(TARGETS openpid EXPORT openpidTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT openpidTargets
    NAMESPACE openpid::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openpid
)

configure_package_config_file(
    cmake/openpidConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/openpidConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openpid
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/openpidConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/openpidConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/openpidConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openpid
)

function(openpid_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

if(OPENPID_BUILD_TESTS)
    enable_testing()
    add_executable(openpid_tests tests/test_controller.cpp)
    target_link_libraries(openpid_tests PRIVATE openpid)
    openpid_set_warnings(openpid_tests)
    add_test(NAME openpid_tests COMMAND openpid_tests)
endif()

if(OPENPID_BUILD_EXAMPLES)
    add_executable(openpid_first_order examples/first_order.cpp)
    target_link_libraries(openpid_first_order PRIVATE openpid)
    openpid_set_warnings(openpid_first_order)
endif()
