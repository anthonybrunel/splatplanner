# SplatPlanner


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
cd ~/flybo_ws
catkin init
catkin config --extend /opt/ros/"your distrib"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```


[Depth Vision](https://github.com/anthonybrunel/vulkan_depth_vision) 

[Mapping system](https://github.com/anthonybrunel/map_core) 

[glm](https://github.com/g-truc/glm) version 0.9.9.8 is requiered by Depth Vision and the Mapping system package



```rosdep install --from-paths WORKSPACE --ignore-src --rosdistro=ROSDISTRO```

