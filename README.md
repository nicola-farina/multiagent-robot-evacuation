# Coordinated evacuation

This repository implemented a coordinated evacuation algorithm of three robots in an environment with
obstacles and one gate.

## Installation and usage
This project run in a ros2 humble environment. To install ros2, please refer to [ros2 installation guide](https://index.ros.org/doc/ros2/Installation/),
or download the docker image from [here](https://hub.docker.com/r/pla10/ros2_humble).

After the setup of the environment, move on it and clone the following repository:
```
git clone https://github.com/pla10/Shelfino_ROS2.git
```
 and move in the devel branch:
```
git checkout devel
```

The repository cloned contains a list of nodes that can be run in a ros2 environment.
The useful ros2 packages for the coordinated evacuation are:
1. Shelfino gazebo
2. Shelfino navigation

To run them, build the packages (`colcon build`, update the available bash commands (`source install/setup.bash`) and run the following:
```
ros2 launch shelfino_gazebo multi_shelfino.launch.py world:=hexagon rviz:=true gui:=true
```
and then
```
ros2 launch shelfino_navigation multi_shelfino_navigation.launch.py map:=hexagon use_sim_time:=true
```

Once the previous commands are executed, the environment is ready to run the coordinated evacuation algorithm.
In order to run it, open a new terminal, build the evacuation package and run the following:
```
ros2 run evacuation evacuation
```
