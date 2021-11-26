#pragma once

#include <ros/ros.h>
#include "mapsearchspace.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../utils/motion_utils.h"
#include <tbb/concurrent_queue.h>
#include "utils/permutohedral.h"

#include <thread>
#include <mutex>


class ViewUtility
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ViewUtility> Ptr;

    ViewUtility();
    ViewUtility(const ros::NodeHandle &nh_i,const ros::NodeHandle &nh_private_i, MapSearchSpace::Ptr& map_i);
    ViewUtility(const ViewUtility & v);
    ~ViewUtility(){

    }
    float computeViewUtility(const Eigen::Vector3f &pos_i,float &angle);

    bool computeRapidFrontier(const Eigen::Vector3f &pos_i, const Eigen::Vector3f &v_curr, float yaw, Eigen::Vector3f &resultVelocity);


    float rayUtility(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, Eigen::Vector3i &hit);

    float rayFrontiereTagger(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, Eigen::Vector3i &hit);



    float getMaximumAngleGain(const Eigen::ArrayXXf &im, double &max_score);

    float getBestVerticalAngle(int start_angle);

    void drawViewUtility(const Eigen::Vector2i& target);

    int selectNearestHorizontalFrontier(int x);

    std::vector<float> scores;


    void computeDistribution(int x, int y);


    void computeLocalDistribution(const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);

    tbb::concurrent_queue<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> path_;



    std::mutex lock_lattice_map;
    phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> lattice_map;

    std::list<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > free_space_;
    void insertRemovedFrontiers();
    void computeGaussianMap(std::vector<pcl::PointXYZI> &map);


    int getHorizontal_angle() const;

private:
    ros::NodeHandle nh_;
    MapSearchSpace::Ptr map_;
    Eigen::ArrayXXf im;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> hits_;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers_;

    Eigen::Vector3f target_;
    cv::Mat cvim;
    int height_= 75;
    float vertical_angle_ = height_*M_PI/180.f;
    float angle_step_ = 1*M_PI/180.f;

    float tan_a = std::tan(angle_step_/2.);
    float cos_a = std::cos(angle_step_/2.);

    int horizontal_angle = 90;

    int width = 360;


    float k1=1;
    float k2=0.01;
    float k3=1;

    float max_range_ = 5.;
    float max_v_=1.5;
    float safety_radius_=.3;

    Eigen::Vector3f t_oc_= Eigen::Vector3f(0,0,0);
    Eigen::Quaternionf r_oc_;

    Eigen::Vector2i cv_circle_ = Eigen::Vector2i(0,0);

};

