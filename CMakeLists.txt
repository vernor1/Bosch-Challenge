project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-Werror -Wall -std=c++11 -O3)

set(CXX_FLAGS "-Wall -Werror")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/coordinate_converter.cpp
            src/main.cpp
            src/helpers.cpp
            src/path_planner.cpp
            src/planner_state.cpp
            src/trajectory_estimator.cpp
            src/trajectory_generator.cpp
            src/vehicle.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
#link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)

# Makes boolean 'test' available
option(test "Build all tests" OFF)
# Testing
# ------------------------------------------------------------------------------
if (test)
  # Enable ExternalProject CMake module
  include(ExternalProject)

  # Download and install GoogleTest
  ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/release-1.8.0.tar.gz
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    # Disable install step
    INSTALL_COMMAND ""
  )

  # Get GTest source and binary directories from CMake project
  ExternalProject_Get_Property(gtest source_dir binary_dir)

  # Create a libgtest target to be used as a dependency by test programs
  add_library(libgtest IMPORTED STATIC GLOBAL)
  add_dependencies(libgtest gtest)

  # Set libgtest properties
  set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
  )

  # Create a libgtest_main target to be used as a dependency by test programs.
  add_library(libgtest_main IMPORTED STATIC GLOBAL)
  add_dependencies(libgtest_main gtest)

  # Set libgtest_main properties
  set_target_properties(libgtest_main PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest_main.a"
  )

  # Create a libgmock target to be used as a dependency by test programs.
  add_library(libgmock IMPORTED STATIC GLOBAL)
  add_dependencies(libgmock gtest)

  # Set libgmock properties
  set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
  )

  include_directories("${source_dir}/googletest/include"
                      "${source_dir}/googlemock/include")

  # Components under test
  # ----------------------------------------------------------------------------
  add_library(trajectory_cost_lib src/trajectory_estimator.cpp)
  add_library(vehicle_lib src/vehicle.cpp)
  add_library(helpers_lib src/helpers.cpp)

  enable_testing()

  # Unit Tests
  # ----------------------------------------------------------------------------
  add_executable(test_trajectory_cost unittests/test_trajectory_cost.cpp)

  # Standard linking to gtest stuff
  target_link_libraries(test_trajectory_cost libgtest libgtest_main)

  # Extra linking for the project
  target_link_libraries(test_trajectory_cost
                        trajectory_cost_lib vehicle_lib helpers_lib)

  # Make tests running through 'make test'
  add_test(NAME test_trajectory_cost COMMAND test_trajectory_cost)
endif()
