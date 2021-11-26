#ifndef NNFRONTIERS_UTILS_H
#define NNFRONTIERS_UTILS_H

#include <map/mapsearchspace.h>
#include "map_core/../../utils/timer.hpp"

class nnfrontiers_utils
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<nnfrontiers_utils> Ptr;

    nnfrontiers_utils(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private_i, MapSearchSpace::Ptr map_i);
    ~nnfrontiers_utils(){

    }
    int validity_check(Eigen::Vector3i p, std::vector<uint8_t> &states, std::vector<float> &edt);
    bool nn_frontier_djikstra(const Eigen::Vector3i &start, std::vector<uint8_t> &states, std::vector<bool> &target_frontier);



    bool djiktra_priority_queue(const Eigen::Vector3i &start, std::vector<uint8_t> &states, std::vector<bool> &target_frontier);

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> getSimplifiedPath();
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> getPath();

    void convert_simplify_path();

    float line_pts_angle(Eigen::Vector3f p1,Eigen::Vector3f p2,Eigen::Vector3f x);
    Eigen::Vector3f getGoal() const;
    size_t shiftStart(const Eigen::Vector3i &start, std::vector<float> &edt,std::vector<uint8_t> &map_state,
                      Eigen::Vector3i &shiftedStart);


    BoundingMap3D<float> djikstra_bounds;


    void updateFrontierMap(const std::vector<uint8_t> &map_states);
    std::vector<uint8_t>& getFrontier_map(){
        return frontier_map_;
    }

private:


    //maintain a frontier set
    //0 mean unmapped, 1 mean frontier, 2 mean already check
    std::vector<uint8_t> frontier_map_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    MapSearchSpace::Ptr map_;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> position_buffer_;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> path_;


    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> simplified_path_;
    Eigen::Vector3f goal;


    //djikstra navigation max and min
    Eigen::Vector3i grid_max_;
    Eigen::Vector3i grid_min_;

    float radius_;
    float fovx_deg_;
    float fovy_deg_;


    float fovx_rad_;
    float fovy_rad_;
};

#endif // NNFRONTIERS_UTILS_H
