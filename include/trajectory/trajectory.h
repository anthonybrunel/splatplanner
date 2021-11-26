#pragma once
#include <ros/ros.h>

#include "../include/planner/state_utils.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/default_topics.h>
#include "../include/planner/constraint.h"
#include <atomic>
#include <memory>
#include <Eigen/Geometry>
#include "../utils/motion_utils.h"

class Trajectory
{
public:
    Trajectory(const ros::NodeHandle& nh,
               const ros::NodeHandle& nh_private, Constraint *constraint_,
               std::atomic<PlannerState> *state_i, OdomData * odom_i);




    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher command_pub_;

    ros::Timer publish_timer_;
    ros::Time start_time_;


    double dt_;
    std::atomic<double> current_sample_time_;

    std::atomic<PlannerState> *state_;

    Constraint *constraint_;
    OdomData *odom_;

    std::atomic<TrajectoryState> traj_state_;
};

