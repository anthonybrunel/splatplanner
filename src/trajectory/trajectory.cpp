#include "../../include/trajectory/trajectory.h"
#include <quadrotor_msgs/TrajectoryPoint.h>

Trajectory::Trajectory(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private, Constraint *constraint_i,
                       std::atomic<PlannerState> *state_i, OdomData * odom_i):nh_(nh),
    nh_private_(nh_private),
    dt_(0.01),
    current_sample_time_(0.0),
    state_(state_i)
{
    constraint_ = constraint_i;

    nh_private_.param("dt", dt_, dt_);

    command_pub_ = nh_private_.advertise<quadrotor_msgs::TrajectoryPoint>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    odom_ = odom_i;
    traj_state_ = TrajectoryState::WAITING;
}
