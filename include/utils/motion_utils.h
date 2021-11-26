#pragma once
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <mutex>
//need to made it thread safe
struct OdomData{

    ros::Time t;
    Eigen::Affine3d current_pose_ = Eigen::Affine3d::Identity();
    Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_angular_velocity_ = Eigen::Vector3d::Zero();




    std::mutex pos_mutex_;
    std::mutex vel_mutex_;
    std::mutex angular_mutex_;
    std::mutex time_mutex_;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d current_velocity(){
        std::unique_lock<std::mutex> lock(vel_mutex_);
        return current_velocity_;
    }

    void setCurrent_velocity(const Eigen::Vector3d &current_velocity)
    {
        std::unique_lock<std::mutex> lock(vel_mutex_);
        current_velocity_ = current_velocity;
    }

    Eigen::Vector3d current_angular_velocity()
    {
        std::unique_lock<std::mutex> lock(angular_mutex_);
        return current_angular_velocity_;
    }

    void setCurrent_angular_velocity(const Eigen::Vector3d &current_angular_velocity)
    {
        std::unique_lock<std::mutex> lock(angular_mutex_);
        current_angular_velocity_ = current_angular_velocity;
    }

    Eigen::Affine3d current_pose()
    {
        std::unique_lock<std::mutex> lock(pos_mutex_);
        return current_pose_;
    }

    void setCurrent_pose(const Eigen::Affine3d &current_pose)
    {
        std::unique_lock<std::mutex> lock(pos_mutex_);
        current_pose_ = current_pose;
    }

    void setTime(const ros::Time &t_i){
        std::unique_lock<std::mutex> lock(time_mutex_);
        t = t_i;

    }

    ros::Time getTime(){
        std::unique_lock<std::mutex> lock(time_mutex_);
        return t;

    }};

inline float degreeDifference(const int angle1_i,const int angle2_i){
    float diff = ( angle1_i - angle2_i) % 360;
    return std::abs(std::min(360-diff,diff));
};


