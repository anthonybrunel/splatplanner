#include "../../include/trajectory/dyntrajectory.h"

DynTrajectory::DynTrajectory(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, Constraint *constraint_i, std::atomic<PlannerState> *state_i, OdomData *odom_i):
    Trajectory (nh,nh_private,constraint_i,state_i,odom_i)
{
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                     &DynTrajectory::trajTimerCallback,
                                     this, false, false);


    std::cout << "[DynTrajectory] Initialized" <<std::endl;
    //    pub_markers_ =
    //            nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1000);

}

void DynTrajectory::trajTimerCallback(const ros::TimerEvent &)
{
    mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    if (current_sample_time_ <= trajectory_.getMaxTime()) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                    trajectory_, current_sample_time_, &trajectory_point);
        if (!success) {
            publish_timer_.stop();
        }
        yawStrategy(trajectory_point);

        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
        command_pub_.publish(msg);
    } else if(current_sample_time_ <= trajectory_.getMaxTime() + 2.){//time shift to go at goal
        current_sample_time_ = trajectory_.getMaxTime();
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                    trajectory_, current_sample_time_, &trajectory_point);


        if (!success) {
            publish_timer_.stop();
        }
        yawStrategy(trajectory_point);

        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    }else {
        publish_timer_.stop();
        trajectory_.clear();
        traj_state_ = TrajectoryState::WAITING;

//        (*state_) = PlannerState::IDLE;
    }



    mtx_.unlock();
}

bool DynTrajectory::getPredictedPose(Eigen::Vector4d &pos_o, Eigen::Vector4d &vel_o)
{
    mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                trajectory_, current_sample_time_, &trajectory_point);
    if(!success || trajectory_.empty() || trajectory_.getMaxTime() <= 0){
        std::cout << "nonononnoo" <<std::endl;
        pos_o.head(3) = odom_->current_pose_.translation();
        vel_o.head(3) = odom_->current_velocity_;
        pos_o(3) = mav_msgs::yawFromQuaternion(
                    (Eigen::Quaterniond)odom_->current_pose_.rotation());
        vel_o(3) = 0;//currently not managing yaw rate

        mtx_.unlock();
        return true;
    }
    double yaw,yaw_rate;
    if(useYaw_){
        yaw = trajectory_point.getYaw();
        yaw_rate = trajectory_point.getYawRate();
    }else{
        yaw = mav_msgs::yawFromQuaternion(
                    (Eigen::Quaterniond)odom_->current_pose_.rotation());
        yaw_rate = 0;

    }
    pos_o << trajectory_point.position_W,yaw;
    vel_o << trajectory_point.velocity_W,yaw_rate;
    mtx_.unlock();
    return true;
}

bool DynTrajectory::getPredictedAccJerk(Eigen::Vector4d &acc_o, Eigen::Vector4d &jerk_o)
{
    mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
                trajectory_, current_sample_time_, &trajectory_point);

    if(!success || trajectory_.empty() || trajectory_.getMaxTime() <= 0){
        acc_o = Eigen::Vector4d::Zero();
        jerk_o = Eigen::Vector4d::Zero();

        mtx_.unlock();
        return true;
    }
//    double yaw,yaw_rate;
//    if(useYaw_){
//        yaw = trajectory_point.getYaw();
//        yaw_rate = trajectory_point.getYawRate();
//    }else{
//        yaw = mav_msgs::yawFromQuaternion(
//                    (Eigen::Quaterniond)odom_->current_pose_.rotation());
//        yaw_rate = 0;

//    }
    acc_o << trajectory_point.acceleration_W,0;
    jerk_o << trajectory_point.jerk_W,0;
    mtx_.unlock();
    return true;
}

void DynTrajectory::yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point)
{
    if(useYaw_ & trajectory_point.velocity_W.norm() > 0)
        return;

    float goal_yaw =std::atan2(trajectory_point.velocity_W.x(),
                               -trajectory_point.velocity_W.y()) - M_PI/2.;

    if (goal_yaw > M_PI)        { goal_yaw -= 2 * M_PI; }
    else if (goal_yaw <= -M_PI) { goal_yaw += 2 * M_PI; }

    trajectory_point.setFromYaw(goal_yaw);
    trajectory_point.timestamp_ns = ros::Time::now().toNSec();


}

void DynTrajectory::publishTrajectoryVizualisation()
{
    // send trajectory as markers to display them in RVIZ
    if(trajectory_.empty())
        return;
    visualization_msgs::MarkerArray markers;
    double distance =
            0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mtx_.lock();

    mav_trajectory_generation::drawMavTrajectory(trajectory_,
                                                 distance,
                                                 frame_id,
                                                 &markers);
    mtx_.unlock();

    pub_trajectory_vizu_.publish(markers);
}

