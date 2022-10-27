# Tracking

Position of courier 

**contents:**

- publishes topic "tracking", Type "afius_msgs::msg::Track"
- uses library "tracking.h" **ToDo**
- parameters:
  - publish_tf (default true): set TransformBroadcaster active/inactive 
  - ref_frame  (default "map"): name of reference frame
  - frame (default"tracker"): name of frame
  - portname (default "/dev/ttyUSB0"): portname for serial input
  - serial_timeout (default 800): timeout for serial port


**dependencies**

- rclcpp
- tf2
- std_msgs
- geometry_msgs
- rtls (terabee position system api) https://github.com/Terabee/positioning_systems_api
- custom messages afius_msgs
- gtest [for unittests]
- lcov [for code coverage]

NOTE: you need to add yourself to the dialout group (and do a full relog), with:

> sudo usermod -a -G dialout ${USER}

**build**

> colcon build --packages-select tracking

**launch:**

- start_launch.py for usage within bike config
> ros2 launch tracking start_launch.py


**test**

ToDo    


