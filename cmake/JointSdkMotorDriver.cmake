# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

# cmake/JointSdkMotorDriver.cmake
#
# Resolves the encos_driver dependency for joint_sdk. Native builds use the
# installed config package; Emscripten builds fetch the driver source via
# CMake FetchContent and force the same static/constrained options used by
# encos_driver's own WASM builds.

include(FetchContent)

function(joint_sdk_resolve_motor_driver)
    if(EMSCRIPTEN OR CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        # Under Emscripten we build encos_driver from source so the WASM glue can
        # link it statically. Force the same constraints encos_driver uses for its
        # own WASM builds.
        set(ENCOS_STATIC_MODE ON CACHE BOOL "" FORCE)
        set(ENCOS_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
        if(NOT DEFINED ENCOS_ENABLE_SPDLOG)
            set(ENCOS_ENABLE_SPDLOG OFF CACHE BOOL "" FORCE)
        endif()
        set(ENCOS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(ENCOS_BUILD_WASM_BINDINGS OFF CACHE BOOL "" FORCE)
        set(ENCOS_BUILD_WASM_TESTS OFF CACHE BOOL "" FORCE)

        FetchContent_Declare(
            encos_driver
            GIT_REPOSITORY https://git.in.encos.cn/bismarckkk/motor_driver.git
            GIT_TAG origin/main
            GIT_SUBMODULES_RECURSE TRUE
        )
        FetchContent_MakeAvailable(encos_driver)

        if(NOT TARGET FakeAdapterSupport)
            message(FATAL_ERROR
                "Fetched encos_driver does not provide the FakeAdapterSupport target. "
                "The joint_sdk JavaScript Fake tools depend on it."
            )
        endif()

        # Make the joint_sdk link line portable: joint_sdk links
        # Encos::encos_driver, but the fetched build tree exposes the raw
        # EncosMotorDriver target. Create the alias if it is missing.
        if(NOT TARGET Encos::encos_driver)
            if(TARGET EncosMotorDriver)
                add_library(Encos::encos_driver ALIAS EncosMotorDriver)
            else()
                message(FATAL_ERROR
                    "Fetched encos_driver does not expose an encos_driver target."
                )
            endif()
        endif()

        # joint_sdk uses nlohmann_json directly. Under Emscripten it is not
        # installed, so fetch it from the Encos internal mirror (pushed from
        # GitHub with tags). Native builds continue to use find_package.
        if(NOT TARGET nlohmann_json::nlohmann_json)
            FetchContent_Declare(
                nlohmann_json
                GIT_REPOSITORY https://git.in.encos.cn/bismarckkk/json.git
                GIT_TAG v3.11.3
            )
            FetchContent_MakeAvailable(nlohmann_json)
        endif()
    else()
        find_package(encos_driver 3.3.2 REQUIRED CONFIG)
    endif()
endfunction()
