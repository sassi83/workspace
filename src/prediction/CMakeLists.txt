cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE Release)
project(prediction)


# creating .gcda for coverage (lcov)
set(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "-g -O0 -Wall -W -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

# default to c++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################### include tensorflow ################################################
#if (NOT EXISTS ${CMAKE_SOURCE_DIR}/tensorflow)
#  make_directory(${CMAKE_SOURCE_DIR}/tensorflow)
#endif()

#if (NOT EXISTS ${CMAKE_SOURCE_DIR}/tensorflow/lib/libtensorflow.so)
#  file(
#          DOWNLOAD
#          https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-gpu-linux-x86_64-2.4.0.tar.gz ${CMAKE_SOURCE_DIR}/tensorflow/linux.tar.gz
#          STATUS
#          status
#          LOG
#          log
#  )
#  list(GET status 0 status_code)
#  list(GET status 1 status_string)
#  if(NOT status_code EQUAL 0)
#    message(FATAL_ERROR "error downloading tensorflow lib: ${status_string}" "${log}")
#  endif()

#  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvzf ${CMAKE_SOURCE_DIR}/tensorflow/linux.tar.gz WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tensorflow)
#  file(REMOVE ${CMAKE_SOURCE_DIR}/tensorflow/linux.tar.gz)
#endif()


#include_directories(tensorflow/include)
#link_directories(${CMAKE_SOURCE_DIR}/tensorflow/lib)

find_library(tensorflow tensorflow)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(afius_msgs REQUIRED)



add_executable(prediction_cv src/PredictionCV.cpp)
add_executable(prediction_nn src/PredictionNN.cpp)
add_executable(subsciber_predicted_list dummy/subsciber_predicted_list.cpp)
add_executable(publisher_dummy_data dummy/publisher_dummy_data.cpp)


ament_target_dependencies(subsciber_predicted_list rclcpp afius_msgs)
ament_target_dependencies(prediction_cv rclcpp afius_msgs)
ament_target_dependencies(prediction_nn rclcpp afius_msgs)
ament_target_dependencies(publisher_dummy_data rclcpp afius_msgs)


# link tensorflow
target_link_libraries(prediction_nn tensorflow)


# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  include_directories(test)
  include_directories(include)
  ament_add_gtest(prediction_test test/TestPredictionCV.cpp)
  ament_target_dependencies(prediction_test rclcpp afius_msgs)
endif()

# add targets
install(TARGETS
        publisher_dummy_data
        prediction_cv
        prediction_nn
        subsciber_predicted_list
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