cmake_minimum_required(VERSION 3.20)

project(couple_joint_solver LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(COUPLE_JOINT_BUILD_TESTS "Build native solver tests" OFF)
option(COUPLE_JOINT_BUILD_BENCHMARKS "Build native solver benchmarks" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(couple_joint_solver
  src/mechanism_solver.cc
  src/mechanism_solver_calibration.cc
  src/mechanism_solver_geometry.cc
  src/mechanism_solver_transmission.cc
  src/mechanism_utils.cc
)

add_library(couple_joint::solver ALIAS couple_joint_solver)

target_include_directories(couple_joint_solver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/couple_joint>
)

target_link_libraries(couple_joint_solver
  PUBLIC
    Eigen3::Eigen
)

target_compile_options(couple_joint_solver
  PRIVATE
    -Wall
    -Wextra
    -Werror
    -Wpedantic
)

set_target_properties(couple_joint_solver PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

if(COUPLE_JOINT_BUILD_TESTS)
  enable_testing()

  add_executable(mechanism_solver_test
    tests/mechanism_solver_test.cc
  )

  target_link_libraries(mechanism_solver_test
    PRIVATE
      couple_joint_solver
  )

  target_compile_options(mechanism_solver_test
    PRIVATE
      -Wall
      -Wextra
      -Werror
      -Wpedantic
  )

  add_test(NAME mechanism_solver_test COMMAND mechanism_solver_test)
endif()

if(COUPLE_JOINT_BUILD_BENCHMARKS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanobench/src/include/nanobench.h")
    message(FATAL_ERROR "nanobench submodule is missing. Run: git submodule update --init --recursive")
  endif()

  add_executable(mechanism_solver_bench
    benchmarks/mechanism_solver_bench.cc
    benchmarks/nanobench_impl.cc
  )

  target_include_directories(mechanism_solver_bench
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanobench/src/include
  )

  target_link_libraries(mechanism_solver_bench
    PRIVATE
      couple_joint_solver
  )

  target_compile_options(mechanism_solver_bench
    PRIVATE
      -Wall
      -Wextra
      -Werror
      -Wpedantic
  )
endif()

set(couple_joint_public_headers
  src/mechanism_solver.h
)

install(
  TARGETS couple_joint_solver
  EXPORT couple_joint_solverTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  FILES ${couple_joint_public_headers}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/couple_joint
)

install(
  EXPORT couple_joint_solverTargets
  NAMESPACE couple_joint::
  FILE couple_joint_solverTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/couple_joint_solver
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/couple_joint_solverConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/couple_joint_solverConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/couple_joint_solver
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/couple_joint_solverConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/couple_joint_solverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/couple_joint_solverConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/couple_joint_solver
)
