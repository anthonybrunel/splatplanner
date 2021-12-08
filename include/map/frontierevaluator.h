#pragma once
#include "mapsearchspace.h"
#include <pcl/point_types_conversion.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/common/transforms.h>

#include "../utils/permutohedral.h"
#include <Eigen/Dense>

#include <unordered_map>

class FrontierEvaluator
{
public:
    FrontierEvaluator();

    FrontierEvaluator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private_i, MapSearchSpace::Ptr& map_i);

    void localFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered);
    MapSearchSpace::Ptr map_;



    void sampling(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &goals);


    void buildPermuto(const Eigen::Matrix<float, 3, Eigen::Dynamic> &pts);

    void sampleView(const Eigen::Matrix<float, 3, Eigen::Dynamic> & pts, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &goals);


    void sampleTarget(const Eigen::Matrix<float, 3, Eigen::Dynamic> & pts, std::vector<Aggregatept2pt> &aggregate_results);


    void findAccessiblePose(const Eigen::Matrix<float, 3, Eigen::Dynamic> &target, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &goals);

    void radiusSelection(const std::vector<Aggregatept2pt> &aggregate_results, std::vector<Aggregatept2pt> &selectedTarget);


    static bool sortAgregateResult(const Aggregatept2pt &rhs, const Aggregatept2pt &lhs){
        return rhs._M0 > lhs._M0;
    }


    void superpointclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, std::vector<bool> &target_frontiers);

    void nn_clustering(std::vector<bool> &target_frontiers, std::vector<uint8_t> &frontier_map, float mav_radius);

    void julia_cluster(std::vector<bool> &target_frontiers, std::vector<uint8_t> &frontier_map);
    bool julia_clustering(const Eigen::Vector3i &p, std::unordered_map<size_t,std::pair<int,bool>>& visited_map,
                          plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &seq,
                         bool backfront);

    phmap::flat_hash_map<LatticeCoordKey, LatticeInfo, Hasher> lattice_map_;


    GMMParameters params;
    float _cluster_radius = 1.5f;
};

