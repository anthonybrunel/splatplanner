#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "map_core/mapcoreserver.h"
class MapSearchSpace
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<MapSearchSpace> Ptr;


    MapSearchSpace();
    MapSearchSpace(const ros::NodeHandle &nh_i, const ros::NodeHandle &nh_prvate_i);

    void init(ros::NodeHandle &nh_i);



    float evaluate_node(Eigen::Vector3f pos_i);


    bool is_free(const Eigen::Vector3f& pos_i);// check if pose is free esdf

    bool is_free(const Eigen::Vector3i& pos_i);


    bool checkOcclusion(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal);

    bool is_inside(const Eigen::Vector3f &p){
        return mapServer_->is_inside(p);
    }
    bool is_inside(const Eigen::Vector3i &p){
        return mapServer_->is_inside(p);
    }
    bool isInsideBuf(const Eigen::Vector3i &p){
        return mapServer_->isInsideBuf(p);
    }
    void convert(const Eigen::Vector3f& in, Eigen::Vector3i& out){
        mapServer_->convert(in,out);
    }


    void convert(const Eigen::Vector3i& in, Eigen::Vector3f& out){
        mapServer_->convert(in,out);
    }

    BoundingMap3D<float> getBoudingMapReal(){
        return mapServer_->getBoudingMapReal();
    }

    void getDistanceGrad(const Eigen::Vector3f &pos, float & dist, Eigen::Vector3f &grad);

    void getDistance(const Eigen::Vector3f &pos, float &dist){
        mapServer_->getDistance(pos,dist);
    }
    void getDistance(const Eigen::Vector3i &pos, float &dist){
        mapServer_->getDistance(pos,dist);
    }
    void getFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &frontiers){
        mapServer_->getFrontiers(frontiers);
    }

    void getRemovedFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& removed_frontiers){
        mapServer_->getRemovedFrontiers(removed_frontiers);
    }

    void getMapState(std::vector<uint8_t>& map_){
        mapServer_->getMapState(map_);
    }

    void getEdtMap(std::vector<float>& edt_i){
        mapServer_->getEdtMap(edt_i);
    }

    void clamp(Eigen::Vector3i &coord){
        mapServer_->clamp(coord);
    }

    void clamp(Eigen::Vector3f &coord){
        mapServer_->clamp(coord);
    }
    bool clampGridRay(const Eigen::Vector3i &origin, const Eigen::Vector3f &dir, Eigen::Vector3i &result){
        return mapServer_->clampRay(origin, dir, result);
    }
    inline uint8_t getState(const Eigen::Vector3i& p){
        return mapServer_->getState(p);
    }
    inline uint8_t getState(const Eigen::Vector3f& p){
        return mapServer_->getState(p);
    }
    inline uint8_t getState(const size_t i){
        return mapServer_->getState(i);
    }
    void get3DGridSize(Eigen::Vector3i &size_o){
        mapServer_->get3DGridSize(size_o);
    }
    inline size_t get_idx(const Eigen::Vector3i &coord){
        return mapServer_->get_idx(coord);
    }

    void setFrontierFeatures(const Eigen::Vector3i &coord,const Features &f){
        mapServer_->setFrontierFeatures(coord,f);
    }


    void getFrontierFeatures(const Eigen::Vector3i &coord,Features &f){
        mapServer_->getFrontierFeatures(coord,f);
    }


    float getResolution(){
        return mapServer_->getResolution();
    }

    void logExploration(){
        mapServer_->logExploration();
    }
private:

    //Constrait robot
    Eigen::Vector3f robot_size_;



    MapCoreServer::Ptr mapServer_;

    ros::NodeHandle nh_;

    ros::NodeHandle nh_privaite_;

    std::vector<float> _edf_buffer_;



};
