#pragma once
#include <Eigen/Core>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include "../planner/constraint.h"

class PlannerBase;

class PolyTrajInterface
{
public:
    PolyTrajInterface();


    static  bool computeTrajectory(const Constraint& contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,
                                      mav_trajectory_generation::Trajectory* trajectory);



};

