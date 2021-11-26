#pragma once


#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>



class PolyTrajExecutor
{
public:
    PolyTrajExecutor(ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~PolyTrajExecutor();
    void polyTraj4DCallback(const mav_planning_msgs::PolynomialTrajectory4D& traj_msg_i);

    void commandTimerCallback(const ros::TimerEvent&);

    void processTraj();



private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;


    ros::Subscriber trajectory4D_sub_;

    ros::Publisher command_pub_;

    ros::Timer publish_timer_;
    ros::Time start_time_;


    double dt_;
    double current_sample_time_;

    mav_trajectory_generation::Trajectory trajectory_;
};

