#pragma once

#include "trajectory.h"
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "spline.h"
#include <mutex>
#include <map/mapsearchspace.h>




class VelocityTrajInterface : public Trajectory
{
public:




    VelocityTrajInterface(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, Constraint *constraint_i,
                          std::atomic<PlannerState> *state_i, OdomData *odom_i, MapSearchSpace::Ptr map_i);

    void commandTimerCallback(const ros::TimerEvent&);

    void startTrajectory(const Eigen::Vector3d &goal_vel, const float goal_yaw, const bool useYaw,MapSearchSpace::Ptr &map_i);

    bool startTrajectory(const Eigen::Vector3d &pos,const Eigen::Vector3d &vel,const Eigen::Vector3d &acc, const Eigen::Vector3d &goal_vel, const float goal_yaw, MapSearchSpace::Ptr &map_i);

    void stopTrajectory();
    bool computeVelocityProfile(const Eigen::Vector3d &goal_vel, MapSearchSpace::Ptr &map_i);

    bool isRunning();
    bool computeVelocityProfile(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, const Eigen::Vector3d &goal_vel);

    void brake();

    bool computePath();
    void getPredictedPose(Eigen::Vector4d & pos_o, Eigen::Vector4d & vel_o);

    void getEstimatePosition(Eigen::Vector3d & pos_o);
    void getEstimateVelocity(Eigen::Vector3d & vel_o);
    void getEstimateAcc(Eigen::Vector3d & acc_o);
    void getEstimateJerk(Eigen::Vector3d & jerk_o);
    void getEstimateYaw(float &yaw);

    void publishVelVector();
    void publishVelTraj();

    void publishTraj();


    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> trajs;


    ros::Publisher visu_vector_pub_;
    ros::Publisher path_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher force_hover_pub_;
    Eigen::VectorXf traj_ = Eigen::VectorXf::Zero(0);


    Eigen::Vector3d  predicted_velocity_;
    Eigen::Vector3d predicted_acc_;
    Eigen::Vector3d  estimate_posistion_;
    float yaw_;
    Eigen::Vector3d velocity_target_;

    ros::Publisher velocity_pub_;

    VelProfile profile_;

    MapSearchSpace::Ptr map_;

    std::mutex mtx_;

    bool useYaw_ = false;
    bool newProfile = false;


};

