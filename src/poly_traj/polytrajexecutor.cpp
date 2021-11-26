#include "../../include/poly_traj/polytrajexecutor.h"

PolyTrajExecutor::PolyTrajExecutor(ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private):nh_(nh),
    nh_private_(nh_private),
    dt_(0.01),
    current_sample_time_(0.0)
{
    nh_private_.param("dt", dt_, dt_);

    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    trajectory4D_sub_ = nh_.subscribe(
        "path_segments_4D", 10, &PolyTrajExecutor::polyTraj4DCallback, this);
    const bool oneshot = false;
    const bool autostart = false;
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                     &PolyTrajExecutor::commandTimerCallback,
                                     this, oneshot, autostart);

    publish_timer_.start();
    std::cout << "[Trajectory Executor] Initialized" <<std::endl;
}

PolyTrajExecutor::~PolyTrajExecutor()
{
    publish_timer_.stop();
}

void PolyTrajExecutor::polyTraj4DCallback(const mav_planning_msgs::PolynomialTrajectory4D &traj_msg_i)
{
    std::cout <<"[PolyTrajExecutor] recv new traj " <<std::endl;

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
                traj_msg_i, &trajectory_);
    if (!success) {
        return;
    }

    processTraj();

}

void PolyTrajExecutor::commandTimerCallback(const ros::TimerEvent &)
{
//    std::cout << "test ok" <<std::endl;

    if (current_sample_time_ <= trajectory_.getMaxTime()) {
      trajectory_msgs::MultiDOFJointTrajectory msg;
      mav_msgs::EigenTrajectoryPoint trajectory_point;

      bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
          trajectory_, current_sample_time_, &trajectory_point);
      if (!success) {
        publish_timer_.stop();
      }
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
      msg.points[0].time_from_start = ros::Duration(current_sample_time_);
      command_pub_.publish(msg);
      current_sample_time_ += dt_;
    } else {
      publish_timer_.stop();
    }

}

void PolyTrajExecutor::processTraj()
{

    publish_timer_.start();
    current_sample_time_ = 0.0;
    start_time_ = ros::Time::now();
}
