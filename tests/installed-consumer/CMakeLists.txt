# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20)
project(joint_sdk_installed_consumer LANGUAGES CXX)
find_package(joint_sdk REQUIRED CONFIG)
add_executable(joint_sdk_installed_consumer main.cc)
target_link_libraries(joint_sdk_installed_consumer PRIVATE joint_sdk::joint_sdk)
