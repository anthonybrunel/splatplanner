#pragma once
#include "planner_base.h"

#include "../../include/trajectory/velocitytrajinterface.h"
#include "../map/frontierevaluator.h"

#include "trajectory/trajectoryompl.h"


#include <algorithm>
#include <random>


enum BestPathRessult{
    OMPLFailed,
    NotFound,
    Found
};

class SplatPlanner : public PlannerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SplatPlanner(const ros::NodeHandle& nh, const ros::NodeHandle &nh_private_i);

    void exploration_behavior() override;

    bool backwardPrevTraj();
    void computeGoals(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &goals);
    BestPathRessult computeBestPath(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &goals, mav_trajectory_generation::Trajectory &traj);


    void publish_visualization() override;
    void publish_path();

    void publish_executed_path();

    void local_planning();

    static bool sortDistanceGoals(const std::pair<int,float> &rhs, const std::pair<int,float> &lhs){
        return rhs.second < lhs.second;
    }
    double norm_angle_2pi(double angle){
        double norm_angle = angle;
        while (norm_angle > 2*M_PI)
          norm_angle -= 2.*M_PI;
        while (norm_angle < 0)
          norm_angle += 2.*M_PI;
        return norm_angle;
    }
    void printTimeComparison();
private:

    std::mutex trajvizu;
    ViewUtility::Ptr view_utility;
    int startlookAround_ = 0;
    FrontierEvaluator frontiersEvaluator_;
    VelocityTrajInterface vel_traj;


    std::random_device rd {};
    std::default_random_engine rng_ = std::default_random_engine { rd() };

    ros::Publisher path_pub_;
    ros::Publisher saved_paths_pub_;
    ros::Publisher saved_paths_pub2_;
    ros::Publisher goals_pub_;

    ros::Publisher mapping_pub_;
    TrajectoryOMPL traj_ompl;


    std::vector<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>> saved_path_ompl_;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> path_ompl_;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> goals_;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> poly_path_;
    std::vector<double> poly_time_;


    std::vector<std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>>> paths_;
    std::vector<std::vector<double>> paths_yaw_;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> prev_states_;


    std::vector<Eigen::Vector4d , Eigen::aligned_allocator<Eigen::Vector4d>> lee_waypoints_;

    visualization_msgs::MarkerArray markers_paths_;

    int best_idx_ = 0;

    std::vector<double> times;
    double std_times = 0;
    Timer globalt;
};

