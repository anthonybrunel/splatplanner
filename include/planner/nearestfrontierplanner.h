#pragma once
#include "planner_base.h"

#include "../../include/trajectory/velocitytrajinterface.h"
#include "../map/frontierevaluator.h"

#include "trajectory/trajectoryompl.h"
#include "utils/nnfrontiers_utils.h"

class NearestFrontierPlanner : public PlannerBase

{
public:
    NearestFrontierPlanner(const ros::NodeHandle& nh, const ros::NodeHandle &nh_private_i);

    void exploration_behavior() override;


    void publish_visualization();
    void publish_path();
    void publish_mapping();



private:

    std::mutex trajvizu;
    mav_trajectory_generation::Trajectory test_traj;

    ViewUtility::Ptr view_utility;

    int startlookAround_ = 0;

    FrontierEvaluator frontiersEvaluator_;
    VelocityTrajInterface vel_traj;




    ros::Publisher path_pub_;
    //point cloud publisher to debug mapping planner
    ros::Publisher mapping_pub_;

    TrajectoryOMPL traj_ompl;


    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> path_;


    nnfrontiers_utils::Ptr nndjikstra;



    bool useRapid_ = false;


    //djikstra time (min - max) std
    //distance
    //avg speed

};

