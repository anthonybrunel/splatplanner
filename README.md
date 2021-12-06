# SplatPlanner, Classic and Rapid Frontier

We address the problem of autonomous exploration of unknown environments using a Micro Aerial Vehicle (MAV) equipped with an active depth sensor. As such, the task consists in mapping the gradually discovered environment while planning the envisioned trajectories in real-time, using on-board computation only. To do so, we present SplatPlanner, an end-to-end autonomous planner that is based on a novel Permutohedral Frontier Filtering (PFF) which relies on a combination of highly efficient operations stemming from bilateral filtering using permutohedral lattices to guide the entire exploration. 

![SplatPlanner](https://github.com/anthonybrunel/splatplanner/blob/main/Splat.gif)

# Installation

Installation instructions are provided in the [FLYBO wiki](https://github.com/anthonybrunel/FLYBO/wiki). Feel free to create an issues if you have any questions or problems related to this package.

## Citation

Please consider citing the following works if you use any of the contents provided by this specific subpart of [FLYBO](https://github.com/anthonybrunel/FLYBO):

[IEEE](), [HAL](https://hal.archives-ouvertes.fr/hal-03380109)
```
@inproceedings{Brunel3DV2021,
  TITLE = {FLYBO: A Unified Benchmark Environment for Autonomous Flying Robots},
  AUTHOR = {Brunel, Anthony and Bourki, Amine and Strauss, Olivier and Demonceaux, C{\'e}dric},
  BOOKTITLE = {9th International Conference on 3D Vision},
  ADDRESS = {Online, United Kingdom},
  YEAR = {2021}
}
```
[IEEE](https://ieeexplore.ieee.org/document/9560896), [HAL](https://hal.archives-ouvertes.fr/hal-03175707), [Video](https://youtu.be/DCcfA2HB1GI)

```
@inproceedings{BrunelICRA2021,
  title={SplatPlanner: Efficient Autonomous Exploration via Permutohedral Frontier Filtering},
  author={Brunel, Anthony and Bourki, Amine and Demonceaux, C{\'e}dric and Strauss, Olivier},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA 2021)},
  year={2021}
}
```
```
@inproceedings{yamauchi1997frontier,
  title={A frontier-based approach for autonomous exploration},
  author={Yamauchi, Brian},
  booktitle={Proceedings 1997 IEEE International Symposium on Computational Intelligence in Robotics and Automation CIRA'97.'Towards New Computational Principles for Robotics and Automation'},
  pages={146--151},
  year={1997},
  organization={IEEE}
}
```
```
@inproceedings{cieslewski2017rapid,
  title={Rapid exploration with multi-rotors: A frontier selection method for high speed flight},
  author={Cieslewski, Titus and Kaufmann, Elia and Scaramuzza, Davide},
  booktitle={2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2135--2142},
  year={2017},
  organization={IEEE}
}
```

