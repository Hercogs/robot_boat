## README
This is my ROSJect documentation!

Autonomous boat ROSJect based on diff drive robot!

Feel free to donate some money for some beers is you consider this useful!

To use this project, install:

- Ubuntu 20.04

- ROS2 foxy

To run this project, the following should be done:

#### Clone project in your ROS2 workspace
```git clone https://github.com/Hercogs/boat_simulation_pkg.git```

#### Build and source your workspace
```colcon build && source install/setup.bash```

#### Launch Gazebo world
```source install/setup.bash && ros2 launch robot_boat_gazebo_pkg start_world.launch.py```

#### Spawn robot in Gazebo world
```source install/setup.bash && ros2 launch  robot_boat_gazebo_pkg spawn_robot_ros2_utm.launch.py```


Now you should see Gazebo world with fantastic boat

You can use following topics:

- robot_boat/cmd_vel
- robot_boat/gps
- robot_boat/laser_scan
- /robot_boat/gps_utm

#### To drive around manually with robot, execute folowing:
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=robot_boat/cmd_vel```