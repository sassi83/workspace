# Prediction

Prediction of moving objects in the environment of the bicycle/vehicle


**TODO** pre-/postprocessing  

**contents:**

- subscribes to topic "object_list" , Type "afius_msgs::msg::ObjectList"
- publishes topic "prediction_list", Type "afius_msgs::msg::PredictedObjectList"
- uses library "prediction.h"
- parameters: 
  - numTimesteps number of discrete time steps to predict
  - timestep distance between discrete time steps in seconds
  - for Testing update dummy_trajectory_path (default: "/mnt/d/eaasy/ros2_ws/src/prediction/dummy/dummy_trajectory.txt")


**dependencies**

- rclcpp
- std_msgs
- custom messages afius_msgs
- gtest [for unittests]
- lcov [for code coverage]

**build**

> colcon build --packages-select prediction

**launch:**

- start_launch.py for usage within bike config
> ros2 launch  prediction start_launch.py

- test_launch.py for test usage with mock up for topic "object_list" and "ego_state" 
and subscriber for topic "prediction_list" (see folder dummy/)
> ros2 launch  prediction test_launch.py

**test:**

- unitest see results under "build/prediction/test_results/" or "build/prediction/Testing/":
> colcon test --packages-select prediction
- coverage: install lcov
> sudo apt install lcov
- generate coverage.info (in ros2 workspace)
> lcov --capture --directory . --output-file coverage.info
- generate html from coverage.info in folder "out"
> genhtml coverage.info --output-directory out

