cmake_minimum_required(VERSION 3.5)
project(tracking)


# creating .gcda for coverage (lcov)
SET(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -Wall -W -fprofile-arcs -ftest-coverage")
SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

#default to c++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(afius_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(positioning_systems_api REQUIRED)

include_directories(include)

add_executable(tracking src/Tracking.cpp)
ament_target_dependencies(
        tracking
        rclcpp
        tf2
        tf2_ros
        afius_msgs
        )

target_link_libraries(tracking
        positioning_systems_api::rtls_driver
        )

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  include_directories(test)
  #include_directories(include)
  ament_add_gtest(tracking_test test/TestTracking.cpp)
  ament_target_dependencies(tracking_test rclcpp afius_msgs)
endif()

# add targets
install(TARGETS
        tracking
        DESTINATION lib/${PROJECT_NAME})

# install header files
install(DIRECTORY include/
        DESTINATION include
        )

# add launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

# add config file
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
