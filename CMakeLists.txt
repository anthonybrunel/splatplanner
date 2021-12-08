cmake_minimum_required(VERSION 2.8.3)
project(splatplanner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14 -O3  -g -msse -msse2 -msse3 -msse4.2)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package( OpenCV REQUIRED )
find_package( TBB REQUIRED )

find_package(ompl REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)






cs_add_library(splatplanner
    include/map/mapsearchspace.h
    src/map/mapsearchspace.cpp


    include/map/frontierevaluator.h
    src/map/frontierevaluator.cpp

    include/map/viewutility.h
    src/map/viewutility.cpp


    include/utils/boundingvolume.h
    src/utils/boundingvolume.cpp


    include/planner/planner_base.h
    src/planner/planner_base.cpp

    include/planner/nearestfrontierplanner.h
    src/planner/nearestfrontierplanner.cpp


    include/planner/constraint.h
    src/planner/constraint.cpp
    include/planner/state_utils.h



    include/trajectory/velocitytrajinterface.h
    src/trajectory/velocitytrajinterface.cpp

    include/trajectory/trajectory.h
    src/trajectory/trajectory.cpp

    include/trajectory/polytrajinterface.h
    src/trajectory/polytrajinterface.cpp
    include/utils/motion_utils.h

    include/trajectory/spline.h
    src/trajectory/spline.cpp

    include/utils/permutohedral.h
    src/utils/permutohedral.cpp


    include/trajectory/trajectoryompl.h
    src/trajectory/trajectoryompl.cpp

    include/planner/splatplanner.h
    src/planner/splatplanner.cpp


    include/utils/nnfrontiers_utils.h
    src/utils/nnfrontiers_utils.cpp

    include/planner/plannerlogging.h
    src/planner/plannerlogging.cpp
    )
target_link_libraries(splatplanner  ${catkin_LIBRARIES} Eigen3::Eigen  ${OpenCV_LIBS} tbb  ${OMPL_LIBRARIES})




cs_add_executable(splatplanner_node
  node/splatplanner_node.cpp
)
target_link_libraries(splatplanner_node splatplanner ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES}  ${OpenCV_LIBS} tbb)




cs_add_executable(nfplanner_node
  node/nfplanner_node.cpp
)
target_link_libraries(nfplanner_node splatplanner ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES}  ${OpenCV_LIBS} tbb)





