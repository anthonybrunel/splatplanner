# SplatPlanner

We address the problem of autonomous exploration of unknown environments using a Micro Aerial Vehicle (MAV) equipped with an active depth sensor. As such, the task consists in mapping the gradually discovered environment while planning the envisioned trajectories in real-time, using on-board computation only. To do so, we present SplatPlanner, an end-to-end autonomous planner that is based on a novel Permutohedral Frontier Filtering (PFF) which relies on a combination of highly efficient operations stemming from bilateral filtering using permutohedral lattices to guide the entire exploration. 

[![SplatPlanner](https://img.youtube.com/vi/DCcfA2HB1GI/0.jpg)](https://youtu.be/DCcfA2HB1GI)

## System requirements

Ubuntu (20.04) -  CMake: 3.15+ - C++17 compiler

[ROS](http://wiki.ros.org/ROS/Installation) Desktop-Full Install Recommended

[Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

PCL ```sudo update && sudo apt install libpcl-dev```

[TBB](https://github.com/oneapi-src/oneTBB) version 2020.1-2 ```sudo update && sudo apt install libtbb-dev```

```sudo apt-get install libgoogle-glog-dev```

## Installation

### Workspace Setup:

```
mkdir -p ~/$WORKSPACE/src
cd ~/$WORKSPACE/src
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin config --merge-devel
```

### External Packages Setup:
Package to simulate a standard depth camera

[Depth Camera](https://github.com/anthonybrunel/vulkan_depth_vision) 

Grid Mapping package with frontiers extraction and Euclidean distance mapping

[Mapping System](https://github.com/anthonybrunel/map_core) 

[glm](https://github.com/g-truc/glm) version 0.9.9.8 is requiered by Depth Camera and the Mapping System package


```
cd ~/WORKSPACE/src
wstool init . ./$WORKSPACE/splatplanner.rosinstall or wstool merge -t if already init
wstool update
rosdep install --from-paths $WORKSPACE --ignore-src --rosdistro=noetic
```

## Demo


## Citation

Please consider citing the following works if you use any of the contents provided by FLYBO:

```
@inproceedings{Brunel3DV2021,
  TITLE = {FLYBO: A Unified Benchmark Environment for Autonomous Flying Robots},
  AUTHOR = {Brunel, Anthony and Bourki, Amine and Strauss, Olivier and Demonceaux, C{\'e}dric},
  BOOKTITLE = {9th International Conference on 3D Vision},
  ADDRESS = {Online, United Kingdom},
  YEAR = {2021}
}
```
```
@inproceedings{brunel2021splatplanner,
  title={SplatPlanner: Efficient Autonomous Exploration via Permutohedral Frontier Filtering},
  author={Brunel, Anthony and Bourki, Amine and Demonceaux, C{\'e}dric and Strauss, Olivier},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA 2021)},
  year={2021}
}
```

