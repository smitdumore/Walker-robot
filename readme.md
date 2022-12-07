[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# Cleaner robot 

### Dependecies
- Ubuntu 22.04
- ROS2 Humble Hawksbill
- rclcpp
- ament 
- colcon
- turtlebot simulations

### How to use this repository
Clone this repo
```
cd <ROS_ws>/src
git clone https://github.com/smitdumore/Walker-robot.git
```

Build this repo
```
cd <ROS2_ws>
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select walker_robot
```

### How to run this repo
In a terminal launch turtlebot simulation
```
. install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In a new terminal
```
. install/setup.bash
ros2 launch walker_robot cleaner.launch.py
```

### recording and playing bag 

To enable recording bag file from the launch file, chnage the variable value "record_bag" to True.

To record a bag file manually 
```
ros2 bag record -a -o my_bag 
```

To inspect a bag file
```
ros2 bag info my_bag
```

To play a bag file
```
ros2 bag play my_bag
```

### Results

cppcheck
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```

cpplint
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```

google style guide
```
clang-format -style=Google -i filename.cpp
```