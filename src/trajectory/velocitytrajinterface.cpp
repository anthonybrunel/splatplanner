#include "../../include/trajectory/velocitytrajinterface.h"
#include <visualization_msgs/Marker.h>
#include "../utils/timer.hpp"

#include <quadrotor_msgs/TrajectoryPoint.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
VelocityTrajInterface::VelocityTrajInterface(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private,
                                             Constraint *constraint_i, std::atomic<PlannerState> *state_i,
                                             OdomData *odom_i, MapSearchSpace::Ptr map_i):
    Trajectory (nh,nh_private,constraint_i,state_i,odom_i)
{
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                     &VelocityTrajInterface::commandTimerCallback,
                                     this, false, false);


    velocity_pub_ = nh_private_.advertise<geometry_msgs::TwistStamped>(
                "autopilot/velocity_command", 10);

    force_hover_pub_ = nh_private_.advertise<std_msgs::Empty>(
                "autopilot/force_hover", 10);




    map_=map_i;
    std::cout << "[PolyTrajInterface] Initialized" <<std::endl;
    predicted_acc_ = Eigen::Vector3d(0,0,0);
    //    pub_markers_ =
    //            nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1000);
    estimate_posistion_ = odom_->current_pose_.translation();
    predicted_velocity_ = odom_->current_velocity_;

    visu_vector_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planner/vel_vector", 10);
    path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planner/path_vel", 10);

    traj_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planner/optim_traj_", 10);
}

void VelocityTrajInterface::commandTimerCallback(const ros::TimerEvent &)
{
    mtx_.lock();
    double dt = current_sample_time_;
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    dt = current_sample_time_ - dt;

    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;


    if(traj_state_ == BRAKING){
        //do bracking untill  current_sample_time_ >= profile_.T_
        //then send 0 velocity and traj_state become waiting and state = idle
        if(odom_->current_velocity().squaredNorm() < 0.04*0.04 ){
            traj_state_ =WAITING;
            (*state_) = PlannerState::IDLE;
            mtx_.unlock();

            publish_timer_.stop();

        }
        mtx_.unlock();

        return;


    }else{

        //execute velocity trajectory until bracking requested
        if(current_sample_time_ >= profile_.T_){
            //velocitykeeping
            Eigen::Vector3f vel,acc;
            profile_.getVelAcc(profile_.T_,vel,acc);
            if(vel.norm()> constraint_->max_v_){
                vel = vel.normalized()*constraint_->max_v_;
            }
            trajectory_point.position_W = estimate_posistion_ + (vel.cast<double>()*dt+acc.cast<double>()*0.5*dt*dt);

            trajectory_point.velocity_W = vel.cast<double>();
            trajectory_point.acceleration_W = acc.cast<double>();

        }else{
            //get velocity ta u(t)
            Eigen::Vector3f vel,acc;
            profile_.getVelAcc(current_sample_time_,vel,acc);
            if(vel.norm()> constraint_->max_v_){
                vel = vel.normalized()*constraint_->max_v_;
            }
            trajectory_point.position_W = estimate_posistion_ + (vel.cast<double>()*dt+acc.cast<double>()*0.5*dt*dt);
            trajectory_point.velocity_W = vel.cast<double>();
            trajectory_point.acceleration_W = acc.cast<double>();

        }
        mtx_.unlock();

    }
    mtx_.lock();

    estimate_posistion_ = odom_->current_pose().translation();//trajectory_point.position_W;
    predicted_velocity_ = trajectory_point.velocity_W;
    predicted_acc_ = trajectory_point.acceleration_W;
    float goal_yaw = yaw_;

    float odom_yaw = mav_msgs::yawFromQuaternion(
                (Eigen::Quaterniond)odom_->current_pose().rotation());

    if(trajectory_point.velocity_W.norm() > 0 && traj_state_ == TrajectoryState::RUN){
        float goal_yaw =std::atan2(trajectory_point.velocity_W.y(),trajectory_point.velocity_W.x());
        if (goal_yaw < M_PI)        { goal_yaw += 2 * M_PI; }

        yaw_ = goal_yaw;

    }else{
        goal_yaw = odom_yaw;
        yaw_ = odom_yaw;
    }

    if (odom_yaw < 0) { odom_yaw += 2 * M_PI; }
    while(std::fabs(yaw_-odom_yaw) > M_PI){
        if(yaw_ < odom_yaw){
            odom_yaw -= 2 * M_PI;
        }else{
            odom_yaw += 2 * M_PI;
        }
    }
    //    trajectory_point.acceleration_W = Eigen::Vector3d::Zero();
    trajectory_point.setFromYaw(yaw_);
    trajectory_point.timestamp_ns = ros::Time::now().toNSec();
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    mtx_.unlock();

    geometry_msgs::TwistStamped velocity_command;
    velocity_command.header.stamp = ros::Time::now();
    velocity_command.twist.linear.x = predicted_velocity_.x();
    velocity_command.twist.linear.y = predicted_velocity_.y();
    velocity_command.twist.linear.z = predicted_velocity_.z();
    double yaw_err = yaw_-odom_yaw;
    double yaw_rate = odom_->current_angular_velocity().z();

    double sign = 1;
    if(yaw_err<0)
        sign = -1;
    yaw_err = std::abs(yaw_err);
    velocity_command.twist.angular.z = std::min(yaw_err*0.5,constraint_->max_yaw_vel_)*sign;
//    velocity_command.twist.angular.z = std::min(std::min(yaw_rate+constraint_->max_yaw_acc_*0.01,
//                                                         velocity_command.twist.angular.z),constraint_->max_yaw_vel_)*sign;

    velocity_pub_.publish(velocity_command);

//    quadrotor_msgs::TrajectoryPoint quad_msg;

//    quad_msg.pose.position.x = trajectory_point.position_W.x();
//    quad_msg.pose.position.y = trajectory_point.position_W.y();
//    quad_msg.pose.position.z = trajectory_point.position_W.z();

//    quad_msg.velocity.linear.x = trajectory_point.velocity_W.x();
//    quad_msg.velocity.linear.y = trajectory_point.velocity_W.y();
//    quad_msg.velocity.linear.z = trajectory_point.velocity_W.z();

//    quad_msg.acceleration.linear.x = trajectory_point.acceleration_W.x();
//    quad_msg.acceleration.linear.y = trajectory_point.acceleration_W.y();
//    quad_msg.acceleration.linear.z = trajectory_point.acceleration_W.z();


//    quad_msg.jerk.linear.x = trajectory_point.jerk_W.x();
//    quad_msg.jerk.linear.y = trajectory_point.jerk_W.y();
//    quad_msg.jerk.linear.z = trajectory_point.jerk_W.z();

//    quad_msg.snap.linear.x = trajectory_point.snap_W.x();
//    quad_msg.snap.linear.y = trajectory_point.snap_W.y();
//    quad_msg.snap.linear.z = trajectory_point.snap_W.z();


//    quad_msg.heading = trajectory_point.getYaw();
//    quad_msg.heading_rate = trajectory_point.getYawRate();
//    quad_msg.heading_acceleration = trajectory_point.getYawAcc();
//    command_pub_.publish(quad_msg);
    //    publishVelVector();

    //    publishVelTraj();
    //    publishTraj();

}

void VelocityTrajInterface::startTrajectory(const Eigen::Vector3d &goal_vel, const float goal_yaw,
                                            const bool useYaw,MapSearchSpace::Ptr &map_i)
{

    if(!publish_timer_.hasStarted()){
        estimate_posistion_ = odom_->current_pose_.translation();
        predicted_velocity_ = odom_->current_velocity_;
        predicted_acc_ = Eigen::Vector3d::Zero();
        computeVelocityProfile(goal_vel,map_i);
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        publish_timer_.start();

    }else{
        mtx_.lock();
        current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
        mtx_.unlock();

        computeVelocityProfile(goal_vel,map_i);
        mtx_.lock();
        current_sample_time_ = 0.0;
        mtx_.unlock();
    }
    (*state_) = PlannerState::IDLE;
    traj_state_ = TrajectoryState::RUN;
    yaw_ = goal_yaw;
    useYaw_ = useYaw;


}

bool VelocityTrajInterface::startTrajectory(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                                            const Eigen::Vector3d &acc, const Eigen::Vector3d &goal_vel,
                                            const float goal_yaw, MapSearchSpace::Ptr &map_i)
{
    if(!publish_timer_.hasStarted()){
        estimate_posistion_ = pos;
        bool success = computeVelocityProfile(pos,vel,acc,goal_vel);
        if(!success) return false;
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        yaw_ = goal_yaw;
        publish_timer_.start();
    }else{
        Eigen::Vector3d start_pos,start_vel,start_acc;
        getEstimatePosition(start_pos);
        getEstimateVelocity(start_vel);
        getEstimateAcc(start_acc);
        bool success = computeVelocityProfile(start_pos,start_vel,start_acc,goal_vel);
        if(!success) return false;
        mtx_.lock();
        start_time_ = ros::Time::now()-ros::Duration(0.005);
        current_sample_time_ = 0;
        mtx_.unlock();


    }
    traj_state_ = TrajectoryState::RUN;
    return true;
}

void VelocityTrajInterface::stopTrajectory()
{
    //    std::cout << "Stopping trajectory _________________________________ " <<std::endl;
    mtx_.lock();
    if(TrajectoryState::WAITING == traj_state_){
        mtx_.unlock();
        return;
    }


    if(traj_state_ == BRAKING){
        mtx_.unlock();
        return;
    }
    traj_state_ = TrajectoryState::BRAKING;
    mtx_.unlock();

    force_hover_pub_.publish(std_msgs::Empty());

    return;

    Eigen::Vector3d start_pos,start_vel,start_acc;
    getEstimatePosition(start_pos);//odometry position (state bracking)
    getEstimateVelocity(start_vel);
    getEstimateAcc(start_acc);
    float dt = 0.2;
    std::cout << start_vel.transpose() << std::endl;
    start_pos = start_pos +(estimate_posistion_-start_pos)*0.2+ (start_vel*dt+start_acc*0.5*dt*dt);

    estimate_posistion_ = start_pos;
    computeVelocityProfile(start_pos,start_vel,start_acc,Eigen::Vector3d::Zero());
    mtx_.lock();

    start_time_ = ros::Time::now();
    current_sample_time_ = 0.0;

    //    predicted_velocity_.setZero();
    //    predicted_acc_.setZero();
    useYaw_ = false;
    mtx_.unlock();
    publish_timer_.start();

}

bool VelocityTrajInterface::computeVelocityProfile(const Eigen::Vector3d &goal_vel, MapSearchSpace::Ptr &map_i)
{
    Eigen::Vector3f goal_vel_tmp = goal_vel.cast<float>();
    Timer t;
    VelProfile tmpProfile_;

    float min_t = 0.03;//need to find an heuristic here
    float max_t = 3.;//need to find an heuristic here
    float dt = 0.01;
    Eigen::Vector3f tmp(0,0,0);
    Eigen::Vector3f start_vel = predicted_velocity_.cast<float>();
    Eigen::Vector3f start_acc = predicted_acc_.cast<float>();
    Eigen::Vector3f start_jerk(0,0,0);
    ros::Time now = ros::Time::now();
    if(publish_timer_.hasStarted()){
        current_sample_time_ = ros::Duration(now-start_time_).toSec();

        if(current_sample_time_ < profile_.T_){
            profile_.getVelAcc(current_sample_time_,start_vel,start_acc);
            start_vel = predicted_velocity_.cast<float>();
            start_acc = predicted_acc_.cast<float>();
            start_jerk << profile_.qx_.ft_dd(current_sample_time_),
                    profile_.qy_.ft_dd(current_sample_time_),
                    profile_.qz_.ft_dd(current_sample_time_);
        }

    }
    int num_try = 0;
    float angle = 0.2;
    float scale = 0.95;
    while(num_try < 100){
        if(num_try%10 == 0){
            goal_vel_tmp = goal_vel.cast<float>();
            goal_vel_tmp *= scale;
            scale-=0.05;
        }
        for(float T = min_t; T < max_t; T+=dt){

            tmpProfile_.init(start_vel,start_acc.cast<float>(),start_jerk,
                             goal_vel_tmp.cast<float>(),tmp,tmp,
                             T);
            float half_T = T *0.5;
            Eigen::Vector3f max_acc; max_acc << tmpProfile_.qx_.ft_d(half_T),
                    tmpProfile_.qy_.ft_d(half_T),
                    tmpProfile_.qz_.ft_d(half_T);
            Eigen::Vector3f vel,acc;
            tmpProfile_.getVelAcc(T,vel,acc);
            //        for(float t = 0; t < T; t+=dt){
            //            Eigen::Vector3f vel; vel << tmpProfile_.qx_.ft(t),
            //                    tmpProfile_.qy_.ft(t),
            //                    tmpProfile_.qz_.ft(t);
            //            if(vel.norm()>1.5){
            //                std::cout << vel.norm() <<std::endl;
            //            }
            //        }

            if(constraint_->max_a_ > max_acc.norm() ){

                mtx_.lock();
                profile_ = tmpProfile_;
                profile_.start_pos_ = estimate_posistion_.cast<float>();
                start_time_ = now;
                mtx_.unlock();
                Eigen::Vector3f pos(estimate_posistion_.cast<float>());
                bool ok = true;
                for(float t = 0; t < std::fmax(0.1,T); t+=dt){
                    break;
                    float tmp_t = std::fmax(t,tmpProfile_.T_);
                    Eigen::Vector3f vel; vel << tmpProfile_.qx_.ft(tmp_t),
                            tmpProfile_.qy_.ft(tmp_t),
                            tmpProfile_.qz_.ft(tmp_t);
                    Eigen::Vector3f acc; acc << tmpProfile_.qx_.ft_d(tmp_t),
                            tmpProfile_.qy_.ft_d(tmp_t),
                            tmpProfile_.qz_.ft_d(tmp_t);
                    pos = pos + (vel*dt+acc*0.5*dt*dt);
                    float dist;
                    map_i->getDistance(pos,dist);
                    if(dist< 0.8){
                        Eigen::Vector3f grad;
                        if(!map_i->is_inside(pos)){
                            std::cout << "not inside" << pos.transpose() <<std::endl;
                            num_try+=num_try%10;
                            ok = false;
                            break;
                        }
                        map_i->getDistanceGrad(pos,dist,grad);
                        if(grad.norm() <= 0.00001){
                            std::cout << "bug norm" <<std::endl;
                            ok = false;
                            break;
                        }
                        Eigen::Vector3f rot_axis = goal_vel.cast<float>().normalized().cross(grad.normalized());
                        std::cout << "pb :))" << goal_vel_tmp.transpose() <<std::endl;
                        goal_vel_tmp = Eigen::AngleAxisf(-angle,rot_axis)*goal_vel_tmp.cast<float>();

                        std::cout << "pb :))" << goal_vel_tmp.transpose() <<std::endl;
                        ok = false;
                        break;
                    }

                }

                if(ok){
                    //                    TrajOptim optim(map_i,*constraint_);
                    //                    optim.init(profile_);
                    //                    optim.optimize();
                    //                    traj_ = optim.pts();
                    std::cout << "velocity_profile time: " << t.elapsed_micro() <<std::endl;
                    return true;
                }
                break;
            }
        }
        num_try++;
    }
    std::cerr << "No traj found" <<std::endl;
    return false;
}

bool VelocityTrajInterface::isRunning()
{

    return publish_timer_.hasStarted();

}

bool VelocityTrajInterface::computeVelocityProfile(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d& start_acc , const Eigen::Vector3d &goal_vel)
{
    Eigen::Vector3f goal_vel_tmp = goal_vel.cast<float>();
    Timer t;
    VelProfile tmpProfile_;

    float min_t = 0.002;//need to find an heuristic here
    float max_t = 6.;//need to find an heuristic here
    float dt = 0.002;
    Eigen::Vector3f tmp(0,0,0);
    Eigen::Vector3d start_jerk(0,0,0);
    getEstimateJerk(start_jerk);
    int num_try = 0;
    float angle = 0.2;
    float scale = 0.95;
    tmpProfile_.start_pos_ = start_pos.cast<float>();
    while(num_try < 100){
        if(num_try%10 == 0){
            goal_vel_tmp = goal_vel.cast<float>();
            goal_vel_tmp *= scale;
            scale-=0.05;
        }
        for(float T = min_t; T < max_t; T+=dt){

            tmpProfile_.init(start_vel.cast<float>(),start_acc.cast<float>(),start_jerk.cast<float>(),
                             goal_vel_tmp.cast<float>(),tmp,tmp,
                             T);
            float half_T = T *0.5;
            Eigen::Vector3f max_acc; max_acc << tmpProfile_.qx_.ft_d(half_T),
                    tmpProfile_.qy_.ft_d(half_T),
                    tmpProfile_.qz_.ft_d(half_T);
            Eigen::Vector3f vel,acc;
            tmpProfile_.getVelAcc(T,vel,acc);
            //        for(float t = 0; t < T; t+=dt){
            //            Eigen::Vector3f vel; vel << tmpProfile_.qx_.ft(t),
            //                    tmpProfile_.qy_.ft(t),
            //                    tmpProfile_.qz_.ft(t);
            //            if(vel.norm()>1.5){
            //                std::cout << vel.norm() <<std::endl;
            //            }
            //        }
            if(constraint_->max_a_ > max_acc.norm() && constraint_->max_v_ > vel.norm()){
                Eigen::Vector3f pos(start_pos.cast<float>());
                bool ok = true;
                tmpProfile_.getVelAcc(half_T,vel,acc);

                //                for(float t = 0; t < std::fmax(0.1,T); t+=dt){
                //                    break;
                //                    float tmp_t = std::fmax(t,tmpProfile_.T_);
                //                    Eigen::Vector3f vel; vel << tmpProfile_.qx_.ft(tmp_t),
                //                            tmpProfile_.qy_.ft(tmp_t),
                //                            tmpProfile_.qz_.ft(tmp_t);
                //                    Eigen::Vector3f acc; acc << tmpProfile_.qx_.ft_d(tmp_t),
                //                            tmpProfile_.qy_.ft_d(tmp_t),
                //                            tmpProfile_.qz_.ft_d(tmp_t);
                //                    pos = pos + (vel*dt+acc*0.5*dt*dt);
                //                    float dist;
                //                    map_i->getDistance(pos,dist);
                //                    if(dist< 0.8){
                //                        Eigen::Vector3f grad;
                //                        if(!map_i->is_inside(pos)){
                //                            std::cout << "not inside" << pos.transpose() <<std::endl;
                //                            num_try+=num_try%10;
                //                            ok = false;
                //                            break;
                //                        }
                //                        map_i->getDistanceGrad(pos,dist,grad);
                //                        if(grad.norm() <= 0.00001){
                //                            std::cout << "bug norm" <<std::endl;
                //                            ok = false;
                //                            break;
                //                        }
                //                        Eigen::Vector3f rot_axis = goal_vel.cast<float>().normalized().cross(grad.normalized());
                //                        std::cout << "pb :))" << goal_vel_tmp.transpose() <<std::endl;
                //                        goal_vel_tmp = Eigen::AngleAxisf(-angle,rot_axis)*goal_vel_tmp.cast<float>();

                //                        std::cout << "pb :))" << goal_vel_tmp.transpose() <<std::endl;
                //                        ok = false;
                //                        break;
                //                    }

                //                }

                if(ok){
                    //                    TrajOptim optim(map_i,*constraint_);
                    //                    optim.init(tmpProfile_);
                    //                    optim.optimize();
                    //                    traj_ = optim.pts();
                    profile_ = tmpProfile_;

//                    std::cout << "velocity_profile time & T: " << t.elapsed_micro() << " " << T << " curr_vel: " << start_vel.norm() <<std::endl;
                    return true;
                }
                break;
            }
        }
        num_try++;
    }
    std::cout << "compute vel profile" << start_pos.transpose() <<" "
              << start_vel.transpose()<<" "
              << start_acc.transpose()<<" "
              << goal_vel.transpose()<< std::endl;
    std::cerr << "No traj found" <<std::endl;
    stopTrajectory();
    return false;
}

void VelocityTrajInterface::brake()
{

    if(TrajectoryState::WAITING == traj_state_){
        return;
    }


    if(traj_state_ == BRAKING){
        return;
    }
    traj_state_ = TrajectoryState::BRAKING;


    force_hover_pub_.publish(std_msgs::Empty());


    return;

    if(traj_state_ == BRAKING){
        return;
    }
    traj_state_ = TrajectoryState::BRAKING;

    publish_timer_.stop();

    Eigen::Vector3d start_pos,start_vel,start_acc;
    getEstimatePosition(start_pos);//odometry position (state bracking)
    getEstimateVelocity(start_vel);
    getEstimateAcc(start_acc);
    float dt = 0.2;
    start_pos = start_pos + (start_vel*dt+start_acc*0.5*dt*dt);

    computeVelocityProfile(start_pos,start_vel,start_acc,Eigen::Vector3d::Zero());
    mtx_.lock();
    estimate_posistion_ = start_pos;

    start_time_ = ros::Time::now();
    current_sample_time_ = 0.0;

    //    predicted_velocity_.setZero();
    //    predicted_acc_.setZero();
    useYaw_ = false;
    mtx_.unlock();
    publish_timer_.start();

}

bool VelocityTrajInterface::computePath()
{

}

void VelocityTrajInterface::getPredictedPose(Eigen::Vector4d &pos_o, Eigen::Vector4d &vel_o)
{
    std::unique_lock<std::mutex> lck(mtx_);
    if(traj_state_ == TrajectoryState::RUN){

        pos_o.head(3) = estimate_posistion_;
        vel_o.head(3) = predicted_velocity_;
        pos_o(3) = yaw_;

        return;
    }


    pos_o.head(3) = odom_->current_pose_.translation();
    Eigen::Matrix3d R = odom_->current_pose().rotation();
    vel_o.head(3) = R*odom_->current_velocity_;
    pos_o(3) = mav_msgs::yawFromQuaternion(
                (Eigen::Quaterniond)odom_->current_pose_.rotation());

}

void VelocityTrajInterface::getEstimatePosition(Eigen::Vector3d &pos_o)
{
    mtx_.lock();
    if(traj_state_ == TrajectoryState::RUN){
        pos_o = estimate_posistion_;
        mtx_.unlock();
        return;
    }
    mtx_.unlock();
    pos_o = odom_->current_pose().translation();
    estimate_posistion_ = pos_o;
}

void VelocityTrajInterface::getEstimateVelocity(Eigen::Vector3d &vel_o)
{
    mtx_.lock();
    if(traj_state_ == TrajectoryState::RUN){
        vel_o = predicted_velocity_;
        mtx_.unlock();
        return;
    }
    mtx_.unlock();
    vel_o = odom_->current_velocity();
    vel_o = odom_->current_pose().rotation()*vel_o;//odometry robot frame to world frame R
    predicted_velocity_ = vel_o;

}

void VelocityTrajInterface::getEstimateAcc(Eigen::Vector3d &acc_o)
{
    mtx_.lock();
    if(traj_state_ == TrajectoryState::RUN){
        acc_o = predicted_acc_;
        mtx_.unlock();
        return;
    }
    mtx_.unlock();
    acc_o = Eigen::Vector3d::Zero();
    predicted_acc_ = acc_o;
}

void VelocityTrajInterface::getEstimateJerk(Eigen::Vector3d &jerk_o)
{
    mtx_.lock();
    if(traj_state_ == TrajectoryState::RUN){
        jerk_o = predicted_acc_;
        mtx_.unlock();
        return;
    }
    mtx_.unlock();
    jerk_o = Eigen::Vector3d::Zero();
}

void VelocityTrajInterface::getEstimateYaw(float &yaw)
{
    mtx_.lock();
    if(traj_state_ == TrajectoryState::RUN){
        yaw = yaw_;
        mtx_.unlock();
        return;
    }
    mtx_.unlock();
    yaw = mav_msgs::yawFromQuaternion(
                (Eigen::Quaterniond)odom_->current_pose_.rotation());

}

void VelocityTrajInterface::publishVelVector()
{
    float dist;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "grad_dist";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.pose.position.x = estimate_posistion_.x();
    marker.pose.position.y = estimate_posistion_.y();
    marker.pose.position.z = estimate_posistion_.z();


    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;


    marker.points.resize(2);
    marker.points[0].x = 0;
    marker.points[0].y = 0;
    marker.points[0].z = 0;
    marker.points[1].x = predicted_velocity_.x();
    marker.points[1].y = predicted_velocity_.y();
    marker.points[1].z = predicted_velocity_.z();

    marker.color.r = 1.f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.f;

    marker.scale.x = 0.05;
    marker.scale.y = 0.08;
    marker.scale.z = 0.1;

    visu_vector_pub_.publish(marker);
}

void VelocityTrajInterface::publishVelTraj()
{
    if(profile_.T_ <= 0)
        return;


    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::LINE_LIST;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = 0;
    path_pub_.publish(mk);

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.color.a = 1;
    mk.scale.x = 0.05;

    geometry_msgs::Point pt;


    Eigen::Vector3f pos = profile_.start_pos_;
    Eigen::Vector3f prev_pos;
    float dt = 0.01f;
    for(float t = dt; t < profile_.T_+2; t+=dt){
        prev_pos = pos;
        float tmp_t = std::fmin(t,profile_.T_);
        Eigen::Vector3f vel; vel << profile_.qx_.ft(tmp_t),
                profile_.qy_.ft(tmp_t),
                profile_.qz_.ft(tmp_t);
        Eigen::Vector3f acc; acc << profile_.qx_.ft_d(tmp_t),
                profile_.qy_.ft_d(tmp_t),
                profile_.qz_.ft_d(tmp_t);
        pos = prev_pos + (vel*dt+acc*0.5*dt*dt);
        pt.x = prev_pos(0);
        pt.y = prev_pos(1);
        pt.z = prev_pos(2);
        mk.points.push_back(pt);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        mk.points.push_back(pt);
    }


    path_pub_.publish(mk);
}

void VelocityTrajInterface::publishTraj()
{
    if(traj_.size() <= 0)
        return;


    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::LINE_LIST;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = 1 ;
    path_pub_.publish(mk);

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.color.a = 1;
    mk.scale.x = 0.05;

    geometry_msgs::Point pt;

    for(int i = 0; i < traj_.size()-3; i+=3){
        pt.x = traj_(i);
        pt.y = traj_(i+1);
        pt.z = traj_(i+2);
        mk.points.push_back(pt);

        pt.x = traj_(i+3);
        pt.y = traj_(i+4);
        pt.z = traj_(i+5);
        mk.points.push_back(pt);
    }


    traj_pub_.publish(mk);
}



