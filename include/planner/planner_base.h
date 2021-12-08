#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>


#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <eigen_conversions/eigen_msg.h>


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/trajectory/polytrajinterface.h"
#include "constraint.h"

#include <thread>
#include "state_utils.h"

#include <atomic>
#include <memory>


#include "../include/utils/boundingvolume.h"

#include "map/mapsearchspace.h"

#include "../map/viewutility.h"
#include "../utils/motion_utils.h"
#include "map_core/../../utils/timer.hpp"

#include "plannerlogging.h"

class PlannerBase
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    enum ExplorationState{
        YAWTURN,
        EXPLORE
    };

    PlannerBase(const ros::NodeHandle& nh, const ros::NodeHandle &nh_private_i);

    virtual ~PlannerBase();


    void execute(const ros::TimerEvent&);

    bool waitUntilReady();

    void uavOdomCallback(const nav_msgs::OdometryConstPtr &odom_i);


    void depthOdomCallback(const nav_msgs::OdometryConstPtr &odom_i);
    void addGoals(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                  const Eigen::Vector4d &goal_pos,const Eigen::Vector4d &goal_vel);


    void publish_grad_esdf();

    virtual void run_exploration();

    //implement in subclass
    virtual void exploration_behavior();

    virtual void publish_visualization();

    visualization_msgs::Marker drawCamera(const Eigen::Vector3d pos, const Eigen::Vector3d &dir, int id);

    bool validate_path_polytraj();
    bool validate_segment(const Eigen::Vector3f &start,const  Eigen::Vector3f &end, float d = 0.05,float rad = 0.01){

        Eigen::Vector3f dir(end-start);
        float dist = dir.norm();
        dir.normalize();
        float step = 0.05f;//STEP 5CM
        float incr = d;
        float obs_dist;
        while(incr < dist){
            Eigen::Vector3f tmp = start+dir*incr;

            if(!searchspace_->is_inside(tmp)){
                return false;
            }
            searchspace_->getDistance(tmp,obs_dist);
            if(obs_dist < rad){
                return false;
            }
            incr+=step;

        }
        return true;
    }
protected:

    Timer t;
    std::atomic<PlannerState> state_;
    ExplorationState exploreState_;


    bool depthOdomReady = false;
    bool mavOdomReady = false;
    bool isReady_ = false;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer execute_timer_;


    ros::Subscriber sub_odom_;

    ros::Subscriber sub_depth_odom_;


    OdomData odom_data_;

//    Eigen::Affine3d current_pose_;
//    Eigen::Vector3d current_velocity_;
//    Eigen::Vector3d current_angular_velocity_;


    Constraint constraint_;

    bool useLeeController_ = false;

    PolyTrajInterface trajInterface_;
//    LeeControllerTrajectory leeTrajInterface;


    ros::Publisher command_pub_;
    ros::Publisher visu_vector_pub_;
    ros::Publisher saved_path2_pub2_;
    std::thread *explore_thread_;



    MapSearchSpace::Ptr searchspace_;


    Eigen::Vector3d repforce = Eigen::Vector3d::Zero();

    PlannerLogging planner_logging_;
};

