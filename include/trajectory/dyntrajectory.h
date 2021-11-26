#ifndef DYNAMICTRAJECTORY_H
#define DYNAMICTRAJECTORY_H
#include "trajectory.h"
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mutex>

class DynTrajectory : public Trajectory
{
public:
    DynTrajectory(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, Constraint *constraint_i,
                  std::atomic<PlannerState> *state_i, OdomData *odom_i);



    void trajTimerCallback(const ros::TimerEvent&);


    bool getPredictedPose(Eigen::Vector4d & pos_o, Eigen::Vector4d & vel_o);

    bool getPredictedAccJerk(Eigen::Vector4d &acc_o, Eigen::Vector4d &jerk_o);


    bool startTrajectory(mav_trajectory_generation::Trajectory &trajectory)
    {
        mtx_.lock();//trajectory is start in exploration planner thread and trajectory is used in the main node
        if(trajectory.empty() || trajectory.getMaxTime() <= 0)
            return false;
        trajectory_ = trajectory;
        publish_timer_.start();
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();

        for(float t = 0;t<trajectory_.getMaxTime();t+=0.1){
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                        trajectory_, t, &trajectory_point);
            if (!success) {
                publish_timer_.stop();
            }
            yawStrategy(trajectory_point);
            std::cout << "pos: "<< trajectory_point.position_W.transpose() << " | " << trajectory_point.velocity_W.norm() << " | " << trajectory_point.acceleration_W.norm() << std::endl;
        }

        mtx_.unlock();

        (*state_) = PlannerState::IDLE;
        traj_state_ = TrajectoryState::RUN;

        return true;
    }


    void yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point);


    void publishTrajectoryVizualisation();


    std::mutex mtx_;
    bool useYaw_ = false;

    ros::Publisher pub_trajectory_vizu_;


    mav_trajectory_generation::Trajectory trajectory_;

};

#endif // DYNAMICTRAJECTORY_H
