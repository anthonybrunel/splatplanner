# SplatPlanner

We address the problem of autonomous exploration of unknown environments using a Micro Aerial Vehicle (MAV) equipped with an active depth sensor. As such, the task consists in mapping the gradually discovered environment while planning the envisioned trajectories in real-time, using on-board computation only. To do so, we present SplatPlanner, an end-to-end autonomous planner that is based on a novel Permutohedral Frontier Filtering (PFF) which relies on a combination of highly efficient operations stemming from bilateral filtering using permutohedral lattices to guide the entire exploration. 
<p align="center">
[![SplatPlanner](https://img.youtube.com/vi/DCcfA2HB1GI/0.jpg)](https://youtu.be/DCcfA2HB1GI)
  </p>
## System requirements

Ubuntu (20.04) -  CMake: 3.15+ - C++14 compiler

[ROS](http://wiki.ros.org/ROS/Installation) Desktop-Full Install Recommended

[Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

PCL ```sudo update && sudo apt install libpcl-dev```

[TBB](https://github.com/oneapi-src/oneTBB) version 2020.1-2 ```sudo update && sudo apt install libtbb-dev```

```sudo apt-get install libgoogle-glog-dev```

## Installation

### Workspace Setup:

```
mkdir -p ~/WORKSPACE/src
cd ~/$WORKSPACE/src
catkin init
catkin config --extend /opt/ros/"your distrib"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### External Package required:
Package to simulate a standard depth camera
[Depth Camera](https://github.com/anthonybrunel/vulkan_depth_vision) 

Grid Mapping package with frontiers extraction and Euclidean distance mmaping
[Mapping System](https://github.com/anthonybrunel/map_core) 

[glm](https://github.com/g-truc/glm) version 0.9.9.8 is requiered by Depth Camera and the Mapping System package


```
cd ~/WORKSPACE/src
wstool init . ./$WORKSPACE/splatplanner.rosinstall or wstool merge -t if already init
wstool update
rosdep install --from-paths $WORKSPACE --ignore-src --rosdistro=noetic
```



