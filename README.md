# Advanced Cleaning Robot -- Robotics Project
A clean robot which can: slam, path planning and pick up garbage by using machine vision.

## How to use
### Install
for twist mux:
> sudo apt install ros-${ROS_DISTRO}-twist-mux


for path planning:
> sudo apt install ros-${ROS_DISTRO}-turtlebot3 ros-${ROS_DISTRO}-navigation ros-${ROS_DISTRO}-dwa-local-planner

### Config
chang the topics in 
> /opt/ros/${ROS_DISTRO}/share/twist_mux/config/twist_mux_topics.yaml

to

```
topics:
-
  name    : clean_robot
  topic   : next_goal_vel
  timeout : 0.5
  priority: 150
-
  name    : machine_vision
  topic   : detector_vel
  timeout : 0.5
  priority: 250
```

### Start Running
run twist mux:
> roslaunch twist_mux twist_mux.launch

run path planning node:
> roslaunch clean_robot clean_work.launch
  
run machine vision detector:
> roslaunch machine_vision detector.launch

start move:
> roslaunch machine_vision pick_move_place.launch
