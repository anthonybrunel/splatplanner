#include "../../include/trajectory/leecontrollertrajectory.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include "rotors_control/parameters_ros.h"
LeeControllerTrajectory::LeeControllerTrajectory(const ros::NodeHandle &nh_i, const ros::NodeHandle &nh_private_i, Constraint *constraint_i, std::atomic<PlannerState> *state_i, OdomData *odom_i):Trajectory (nh_i,nh_private_i,constraint_i,state_i,odom_i)
{

    ROS_INFO("InitializeParams");

    motor_velocity_reference_pub_ = nh_private_.advertise<mav_msgs::Actuators>(
                mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    rotors_control::GetRosParameter(nh_private_, "position_gain/x",
                                    lee_position_controller_.controller_parameters_.position_gain_.x(),
                                    &lee_position_controller_.controller_parameters_.position_gain_.x());
    rotors_control::GetRosParameter(nh_private_, "position_gain/y",
                                    lee_position_controller_.controller_parameters_.position_gain_.y(),
                                    &lee_position_controller_.controller_parameters_.position_gain_.y());
    rotors_control::GetRosParameter(nh_private_, "position_gain/z",
                                    lee_position_controller_.controller_parameters_.position_gain_.z(),
                                    &lee_position_controller_.controller_parameters_.position_gain_.z());
    rotors_control::GetRosParameter(nh_private_, "velocity_gain/x",
                                    lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                                    &lee_position_controller_.controller_parameters_.velocity_gain_.x());
    rotors_control::GetRosParameter(nh_private_, "velocity_gain/y",
                                    lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                                    &lee_position_controller_.controller_parameters_.velocity_gain_.y());
    rotors_control::GetRosParameter(nh_private_, "velocity_gain/z",
                                    lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                                    &lee_position_controller_.controller_parameters_.velocity_gain_.z());
    rotors_control::GetRosParameter(nh_private_, "attitude_gain/x",
                                    lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                                    &lee_position_controller_.controller_parameters_.attitude_gain_.x());
    rotors_control::GetRosParameter(nh_private_, "attitude_gain/y",
                                    lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                                    &lee_position_controller_.controller_parameters_.attitude_gain_.y());
    rotors_control::GetRosParameter(nh_private_, "attitude_gain/z",
                                    lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                                    &lee_position_controller_.controller_parameters_.attitude_gain_.z());
    rotors_control::GetRosParameter(nh_private_, "angular_rate_gain/x",
                                    lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
    rotors_control::GetRosParameter(nh_private_, "angular_rate_gain/y",
                                    lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
    rotors_control::GetRosParameter(nh_private_, "angular_rate_gain/z",
                                    lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
    rotors_control::GetVehicleParameters(nh_private_, &lee_position_controller_.vehicle_parameters_);
    lee_position_controller_.InitializeParameters();


}

void LeeControllerTrajectory::computeTrajectory(const Constraint &contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps)
{
    trajectory_mtx_.lock();

    std::cout << "New trajectory: " << start_pos.transpose() <<" " << goal_pos.transpose() <<std::endl;
    if(trajectories_.empty())
        start_time_ = ros::Time::now();
    if(wps.size() > 0){
        {
            LinearTrajectory traj;
            traj.generateMotionProfile(start_pos.cast<float>(),wps[0].cast<float>(),constraint_->max_v_,constraint_->max_a_,constraint_->max_yaw_vel_);
            trajectories_.push(traj);
            std::cout << traj.start_.transpose() << "->" << traj.end_.transpose() <<std::endl;

        }
        for (size_t i = 0;i <wps.size()-1; ++i) {
            LinearTrajectory traj;
            traj.generateMotionProfile(wps[i].cast<float>(),wps[i+1].cast<float>(),constraint_->max_v_,constraint_->max_a_,constraint_->max_yaw_vel_);
            trajectories_.push(traj);
            std::cout << traj.start_.transpose() << "->" << traj.end_.transpose() <<std::endl;

        }

        {
            LinearTrajectory traj;

            traj.generateMotionProfile(wps[wps.size()-1].cast<float>(),goal_pos.cast<float>(),constraint_->max_v_,constraint_->max_a_,constraint_->max_yaw_vel_);

            trajectories_.push(traj);
            std::cout << traj.start_.transpose() << "->" << traj.end_.transpose() <<std::endl;

        }


    }else{
        LinearTrajectory traj;
        traj.generateMotionProfile(start_pos.cast<float>(),goal_pos.cast<float>(),constraint_->max_v_,constraint_->max_a_,constraint_->max_yaw_vel_);

        trajectories_.push(traj);

    }
    trajectory_mtx_.unlock();

}



void LeeControllerTrajectory::execute(const nav_msgs::OdometryConstPtr &odom_i)
{

    trajectory_mtx_.lock();

    rotors_control::EigenOdometry odometry;
    rotors_control::eigenOdometryFromMsg(odom_i, &odometry);
    trajectory_msgs::MultiDOFJointTrajectory msg;

    if(!isInit){
        std::cout << "Lee controller is Init" <<std::endl;

        command_trajectory_.position_W = odometry.position;

        command_trajectory_.velocity_W = odometry.velocity;

        command_trajectory_.acceleration_W = Eigen::Vector3d::Zero();

        double last_yaw = mav_msgs::yawFromQuaternion(odometry.orientation);

        command_trajectory_.setFromYaw(last_yaw);
        command_trajectory_.setFromYawRate(0);
        command_trajectory_.setFromYawAcc(0);

        lee_position_controller_.SetTrajectoryPoint(command_trajectory_);
        lee_position_controller_.SetOdometry(odometry);

        Eigen::VectorXd ref_rotor_velocities;
        lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        actuator_msg->angular_velocities.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = odom_i->header.stamp;

        //        motor_velocity_reference_pub_.publish(actuator_msg);
        isInit = true;
    }else{
        if(trajectories_.empty()){
            command_trajectory_.velocity_W = Eigen::Vector3d::Zero();
            command_trajectory_.acceleration_W = Eigen::Vector3d::Zero();
            command_trajectory_.setFromYawAcc(0);
            command_trajectory_.setFromYawRate(0);
            lee_position_controller_.SetTrajectoryPoint(command_trajectory_);
            lee_position_controller_.SetOdometry(odometry);

            Eigen::VectorXd ref_rotor_velocities;
            lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

            mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
            actuator_msg->angular_velocities.clear();
            for (int i = 0; i < ref_rotor_velocities.size(); i++)
                actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
            actuator_msg->header.stamp = odom_i->header.stamp;

            mav_msgs::msgMultiDofJointTrajectoryFromEigen(command_trajectory_, &msg);

            command_pub_.publish(msg);

            //            motor_velocity_reference_pub_.publish(actuator_msg);
        }else{
            //exec traj

            while(!trajectories_.empty()){
                LinearTrajectory &traj = trajectories_.front();
                current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
                if(traj.max_time_ < current_sample_time_){
                    if(traj.max_time_ < current_sample_time_-1){
                        traj.getTrajectoryPoint(traj.max_time_+1,command_trajectory_);
                    }else{
                        trajectories_.pop();


                        if(trajectories_.empty()){
                            traj.getTrajectoryPoint(traj.max_time_+1,command_trajectory_);
                        }
                        else{
                            LinearTrajectory &next_traj = trajectories_.front();
                            Eigen::Vector4d odom_start; odom_start <<  odometry.position,mav_msgs::yawFromQuaternion(odometry.orientation);
                            Eigen::Vector4d odom_vel; odom_vel.head(3)= odometry.orientation.toRotationMatrix()*odometry.velocity;
                            odom_vel.w() = 0;
                            next_traj.replan(odom_start.cast<float>(),odom_vel.cast<float>(),constraint_->max_a_,constraint_->max_v_,constraint_->max_yaw_vel_);
                            start_time_ = ros::Time::now();//reset timer for next segment
                        }
                    }
                }else{
                    traj.getTrajectoryPoint(current_sample_time_,command_trajectory_);
                    break;
                }

                std::cout <<  "Traj: "  << trajectories_.size() <<" " << current_sample_time_ <<std::endl;


            }
            //            command_trajectory_.velocity_W.setZero();
            //            command_trajectory_.acceleration_W.setZero();

            //             std::cout <<  "Pos: "  << command_trajectory_.position_W.transpose() <<std::endl;
                        std::cout <<  "Pos: "  << command_trajectory_.getYaw()<<std::endl;

            //            std::cout <<  "Vel: "  << odometry.position.transpose()  <<" " << command_trajectory_.position_W.transpose() <<std::endl;

            //            std::cout <<  "Vel: "  << odometry.velocity.norm()  <<" " << command_trajectory_.velocity_W.norm() <<std::endl;
            //            double last_yaw = mav_msgs::yawFromQuaternion(odometry.orientation);
            //            std::cout <<  "Vel: "  << odometry.angular_velocity.z()  <<" " << command_trajectory_.getYaw() << " " << command_trajectory_.getYawRate() <<std::endl;
            //            command_trajectory_.setFromYawAcc(0);
            //            command_trajectory_.setFromYawRate(0);
            lee_position_controller_.SetTrajectoryPoint(command_trajectory_);
            lee_position_controller_.SetOdometry(odometry);
            Eigen::VectorXd ref_rotor_velocities;
            lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
            mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
            actuator_msg->angular_velocities.clear();
            for (int i = 0; i < ref_rotor_velocities.size(); i++)
                actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
            actuator_msg->header.stamp = odom_i->header.stamp;
            mav_msgs::msgMultiDofJointTrajectoryFromEigen(command_trajectory_, &msg);

            //            motor_velocity_reference_pub_.publish(actuator_msg);

            //            msg.points[0].time_from_start = ros::Duration(current_sample_time_);
            command_pub_.publish(msg);

            if(trajectories_.empty()){
                (*state_) = PlannerState::IDLE;
            }
        }
    }
    trajectory_mtx_.unlock();

}

void LeeControllerTrajectory::predictPosVel(Eigen::Vector4d &pos, Eigen::Vector4d &vel)
{

    //    pos.head(3) = odom_->current_pose().translation();
    //    pos.w() =  command_trajectory_.getYaw();
    //    Eigen::Matrix3d R = odom_->current_pose().rotation();
    //    vel.head(3) = R*odom_->current_velocity();
    //    vel.w() = command_trajectory_.getYawRate();

    trajectory_mtx_.lock();
    pos.head(3) = command_trajectory_.position_W;
    pos.w() = command_trajectory_.getYaw();

    vel.head(3) = command_trajectory_.velocity_W;
    vel.w() = command_trajectory_.getYawRate();

    trajectory_mtx_.unlock();

}
