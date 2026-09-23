# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20)
project(joint_sdk VERSION 1.4.5 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(JOINT_SDK_BOOST_PREPROCESSOR_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/external/boost_preprocessor/include"
)

if(NOT EXISTS "${JOINT_SDK_BOOST_PREPROCESSOR_INCLUDE_DIR}/boost/preprocessor.hpp")
    message(FATAL_ERROR
        "Boost.Preprocessor submodule is missing. Run: git submodule update --init"
    )
endif()

include(cmake/JointSdkMotorDriver.cmake)
joint_sdk_resolve_motor_driver()

if(NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json REQUIRED)
endif()

option(JOINT_SDK_BUILD_TESTS "Build joint_sdk tests" OFF)
option(JOINT_SDK_BUILD_WASM_BINDINGS "Build joint_sdk Emscripten WASM bindings" OFF)
option(JOINT_SDK_BUILD_WASM_TESTS "Build and run joint_sdk WASM tests" OFF)

find_program(CLANG_FORMAT_BIN NAMES clang-format clang-format-18)
find_program(CLANG_TIDY_BIN NAMES clang-tidy clang-tidy-18)

add_custom_target(joint_sdk_format
    COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/run_clang_format.sh"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Formatting project sources with clang-format"
)

add_custom_target(joint_sdk_format_check
    COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/run_clang_format.sh" --check
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Checking project sources with clang-format"
)

add_custom_target(joint_sdk_tidy
    COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/run_clang_tidy.sh" ${CMAKE_BINARY_DIR}
            --extra-arg=-U_OPENMP
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-tidy on project translation units"
)

if(CLANG_FORMAT_BIN STREQUAL "CLANG_FORMAT_BIN-NOTFOUND")
    set_property(TARGET joint_sdk_format PROPERTY EXCLUDE_FROM_ALL TRUE)
    set_property(TARGET joint_sdk_format_check PROPERTY EXCLUDE_FROM_ALL TRUE)
endif()

if(CLANG_TIDY_BIN STREQUAL "CLANG_TIDY_BIN-NOTFOUND")
    set_property(TARGET joint_sdk_tidy PROPERTY EXCLUDE_FROM_ALL TRUE)
endif()

set(JOINT_SDK_COUPLE_JOINT_SOLVER_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/external/couple_joint_solver"
)

if(NOT EXISTS "${JOINT_SDK_COUPLE_JOINT_SOLVER_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "couple_joint_solver submodule is missing. Run: git submodule update --init"
    )
endif()

set(COUPLE_JOINT_BUILD_TESTS ${JOINT_SDK_BUILD_TESTS})
add_subdirectory("${JOINT_SDK_COUPLE_JOINT_SOLVER_DIR}")

add_library(joint_sdk INTERFACE)
add_library(joint_sdk::joint_sdk ALIAS joint_sdk)

target_compile_features(joint_sdk INTERFACE cxx_std_17)

target_include_directories(joint_sdk INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${JOINT_SDK_BOOST_PREPROCESSOR_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(joint_sdk INTERFACE
    $<BUILD_INTERFACE:Encos::encos_driver>
    $<INSTALL_INTERFACE:Encos::encos_driver>
    nlohmann_json::nlohmann_json
    $<BUILD_INTERFACE:couple_joint::solver>
    $<INSTALL_INTERFACE:couple_joint::couple_joint_solver>
)

if(NOT JOINT_SDK_BUILD_WASM_BINDINGS)
    find_package(Threads REQUIRED)
    target_link_libraries(joint_sdk INTERFACE Threads::Threads)
endif()

if(NOT JOINT_SDK_BUILD_WASM_BINDINGS)
    install(TARGETS joint_sdk EXPORT joint_sdkTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h"
    )

    install(DIRECTORY "${JOINT_SDK_BOOST_PREPROCESSOR_INCLUDE_DIR}/boost"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(EXPORT joint_sdkTargets
        FILE joint_sdkTargets.cmake
        NAMESPACE joint_sdk::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/joint_sdk
    )

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/joint_sdkConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/joint_sdkConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/joint_sdk
    )

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/joint_sdkConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/joint_sdkConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/joint_sdkConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/joint_sdk
    )
endif()

# CPack configuration for generating DEB packages
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CPACK_GENERATOR "DEB")
    set(CPACK_SOURCE_GENERATOR "")
    set(CPACK_BINARY_STGZ "OFF")
    set(CPACK_BINARY_TGZ "OFF")
    set(CPACK_BINARY_TZ "OFF")
    set(CPACK_BINARY_TBZ2 "OFF")
    set(CPACK_BINARY_TXZ "OFF")
    set(CPACK_BINARY_RPM "OFF")

    find_program(DPKG_DEB_EXECUTABLE dpkg-deb)
    if(DPKG_DEB_EXECUTABLE)
        set(CPACK_BINARY_DEB "ON")
    else()
        message(WARNING "dpkg-deb not found: DEB package generator disabled")
        set(CPACK_BINARY_DEB "OFF")
    endif()

    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/jointsdk")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/debian/copyright"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/jointsdk")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_PACKAGE_NAME "JointSDK")
    set(CPACK_PACKAGE_VENDOR "Encos")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
        "joint_sdk - C++17 header-only joint control library")
    set(CPACK_PACKAGE_DESCRIPTION
        "joint_sdk is a header-only C++17 library for joint control based on encos_driver. "
        "Provides three joint abstractions: RJoint (limited direct-drive rotary joint), "
        "ContinuousJoint (unlimited rotary joint), and DoubleCoupleJoint (dual-motor closed-chain "
        "joint). Features include PVT/position/speed/"
        "torque/current control modes, JSON schema generation, and built-in singularity protection.")
    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")

    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Encos Team <Sales002@encos.cn>")
    set(CPACK_DEBIAN_PACKAGE_SECTION "libdevel")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libencosdriver (>= 3.3.2), libeigen3-dev, nlohmann-json3-dev, libstdc++6, libc6")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://encos.cn")
    set(CPACK_DEBIAN_PACKAGE_SHLIBS OFF)

    execute_process(
        COMMAND dpkg --print-architecture
        OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT CPACK_DEBIAN_PACKAGE_ARCHITECTURE)
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    endif()

    set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
    set(CPACK_PACKAGE_FILE_NAME
        "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
endif()

if(JOINT_SDK_BUILD_WASM_BINDINGS AND NOT (EMSCRIPTEN OR CMAKE_SYSTEM_NAME STREQUAL "Emscripten"))
    message(FATAL_ERROR "JOINT_SDK_BUILD_WASM_BINDINGS is only valid under Emscripten")
endif()

if(JOINT_SDK_BUILD_WASM_TESTS AND NOT JOINT_SDK_BUILD_WASM_BINDINGS)
    message(FATAL_ERROR "JOINT_SDK_BUILD_WASM_TESTS requires JOINT_SDK_BUILD_WASM_BINDINGS=ON")
endif()

if(JOINT_SDK_BUILD_WASM_BINDINGS)
    add_executable(JointSDKWasm
        src/wasm/wasm_runtime.cc
        src/wasm/wasm_c_api.cc
    )

    target_include_directories(JointSDKWasm PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${encos_driver_SOURCE_DIR}
    )

    target_compile_options(JointSDKWasm PRIVATE
        "-fexceptions"
    )

    target_link_libraries(JointSDKWasm PRIVATE
        joint_sdk::joint_sdk
        Encos::encos_driver
        FakeAdapterSupport
        nlohmann_json::nlohmann_json
        couple_joint::solver
    )

    target_link_options(JointSDKWasm PRIVATE
        "-fexceptions"
        "-sASYNCIFY"
        "-sMODULARIZE=1"
        "-sEXPORT_ES6=1"
        "-sINCOMING_MODULE_JS_API=['locateFile','wasmBinary']"
        "-sENVIRONMENT=web,node"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sALLOW_TABLE_GROWTH=1"
        "-sEXPORTED_FUNCTIONS=['_malloc','_free','_joint_sdk_fake_enable_position_error']"
        "-sEXPORTED_RUNTIME_METHODS=['cwrap','ccall','UTF8ToString','getValue','setValue','addFunction','removeFunction','HEAPU8','HEAP32','HEAPF64']"
    )

    target_compile_definitions(JointSDKWasm PRIVATE "JOINT_SDK_VERSION=\"${PROJECT_VERSION}\"")

    set_target_properties(JointSDKWasm PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
        OUTPUT_NAME "JointSDKWasm"
        SUFFIX ".js"
    )
endif()

include(CPack)

if(JOINT_SDK_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)

    add_executable(joint_sdk_tests
        tests/joint_framework_test.cc
        tests/zero_damping_joint_test.cc
    )

    target_include_directories(joint_sdk_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )

    target_link_libraries(joint_sdk_tests PRIVATE
        joint_sdk::joint_sdk
        GTest::gtest_main
    )

    gtest_discover_tests(joint_sdk_tests)
endif()
