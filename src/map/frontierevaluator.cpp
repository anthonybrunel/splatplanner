#include "../../include/map/frontierevaluator.h"
#include "../utils/timer.hpp"
#include "../external/plf_list.hpp"
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <random>
#include <pcl/filters/convolution_3d.h>
#include <pcl/segmentation/supervoxel_clustering.h>

FrontierEvaluator::FrontierEvaluator()
{

}

FrontierEvaluator::FrontierEvaluator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private_i,MapSearchSpace::Ptr &map_i)
{
    map_ = map_i;

    nh_private_i.param("cluster_radius", _cluster_radius, _cluster_radius);

}


void FrontierEvaluator::localFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    kdtree->setInputCloud(cloud_filtered);




    Timer t;
    float maxr=0.,maxg=0., maxb=0.;
    for(auto &p: cloud_filtered->points){
        std::vector<int> idx;
        std::vector<float> dist;
        kdtree->nearestKSearch(p,50,idx,dist);
        Eigen::Matrix3f cov;
        Eigen::Vector4f centroid(p.x,p.y,p.z,1);
        pcl::PointXYZ average_centroid;

        pcl::computeCentroid(*cloud_filtered,idx,average_centroid);
        pcl::computeCovarianceMatrixNormalized(*cloud_filtered,idx,centroid,cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov, Eigen::ComputeEigenvectors);
        float verticality = eigen_solver.eigenvectors().col(0).dot(Eigen::Vector3f::UnitZ());
        std::vector<float> ev = {eigen_solver.eigenvalues()[0],eigen_solver.eigenvalues()[1],eigen_solver.eigenvalues()[2]};
        std::vector<int> indices(3);
        std::size_t n(0);
        std::generate(std::begin(indices), std::end(indices), [&]{ return n++; });
        std::sort(std::begin(indices),std::end(indices),
                  [&](int i1, int i2) { return ev[i1] > ev[i2]; } );


        std::vector<float> lambda = {(std::max(ev[indices[0]],0.f)),
                                     (std::max(ev[indices[1]],0.f)),
                                     (std::max(ev[indices[2]],0.f))};
        float planarity  = (sqrtf(lambda[1]) - sqrtf(lambda[2])) / sqrtf(lambda[0]);

        float entropy = 0;
        for(auto v : lambda){
            if(v == 0)
                continue;
            entropy += sqrtf(v)*std::log(sqrtf(v));
        }

        entropy *= -1;

        float scattering = sqrtf(lambda[2]) / sqrtf(lambda[0]);


        float omni = cbrtf(sqrtf(lambda[0]) * sqrtf(lambda[1]) * sqrtf(lambda[2]));

        float anisotropic = ((sqrtf(lambda[0]) - sqrtf(lambda[2])) / sqrtf(lambda[0]));


        float sphericity = (sqrtf(lambda[2]) / sqrtf(lambda[0]));

        //        p.r = planarity*255;
        //        p.g = scattering*255;
        //        p.b = entropy*255;

        //        if(maxr < anisotropic){
        //            maxr = entropy;
        //        }
        //        if(maxg < planarity){
        //            maxg = scattering;
        //        }
        //        if(maxb < planarity){
        //            maxb = entropy;
        //        }

        float coeff_centroid = (centroid.head(3)-average_centroid.getVector3fMap()).norm();
        if(maxr < coeff_centroid){
            maxr = coeff_centroid;
        }
        Features f;
        coeff_centroid = 1-std::fmax(0,std::fmin(coeff_centroid,1));
        f.tmp = (std::fmax(anisotropic,planarity))*coeff_centroid*10;
        if(fabs(verticality) > 0.5){
            f.tmp = 0;
        }
        //        f.tmp = 1;

        Eigen::Vector3i coord;
        map_->convert(p.getArray3fMap(),coord);
        map_->setFrontierFeatures(coord,f);
    }



    std::cout << "Features computing: " << t.elapsed_ms()  <<" " << maxr <<std::endl;

}

void FrontierEvaluator::sampling(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &goals)
{
    Timer t;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers;

    map_->getFrontiers(frontiers);

    if(frontiers.size()<3){
        return;
    }
    Eigen::Matrix<float, 3, Eigen::Dynamic> pts(3,frontiers.size());

    for(int i = 0;i < frontiers.size(); ++i){
        Eigen::Vector3f p;
        map_->convert(frontiers[i],p);
        pts.col(i) = p;
    }

    buildPermuto(pts);

    std::vector<Aggregatept2pt> aggregate_results;
    sampleTarget(pts,aggregate_results);

    std::cout << "[FrontierEVAL] t: " << t.elapsed_ms() << std::endl;
    std::sort(aggregate_results.begin(),aggregate_results.end(),FrontierEvaluator::sortAgregateResult);


    std::vector<Aggregatept2pt> sampled_target;

    radiusSelection(aggregate_results,sampled_target);

    //sample pos



    //vosu only
    {
        for(size_t i = 0; i < aggregate_results.size(); ++i){
            Eigen::Vector3i coord;
            map_->convert(aggregate_results[i].pos_,coord);


            Features f;

            f.planarity = aggregate_results[i]._M0;
            map_->setFrontierFeatures(coord,f);
        }
        goals.resize(sampled_target.size());
        for(size_t i = 0; i < sampled_target.size(); ++i){
            Eigen::Vector3i coord;
            map_->convert(sampled_target[i].pos_,coord);


            //            Features f;

            //            f.planarity = 0.;
            //            map_->setFrontierFeatures(coord,f);

            goals[i] = sampled_target[i].pos_.head(3);
            goals[i].w() = sampled_target[i]._M0;
        }

    }

    std::cout << "[FrontierEVAL] t: " << t.elapsed_ms() << " " << pts.size()  <<" " << sampled_target.size() <<std::endl;
}

void FrontierEvaluator::buildPermuto(const Eigen::Matrix<float, 3, Eigen::Dynamic> & pts)
{

    Eigen::Vector3i size_grid;

    map_->get3DGridSize(size_grid);
    float resolution = map_->getResolution();
    int step = (int)(0.6f/resolution);
    Eigen::Matrix<float, 3, Eigen::Dynamic> free_pts;
    int estimatee_volume = (int) size_grid.x()/step + (int)size_grid.y()/step + (int)size_grid.z()/step;
    free_pts.resize(3,estimatee_volume);
    Eigen::Vector3i p(step*2,step*2,step*2);
    int cpt = 0;
    std::cout << size_grid.transpose() << " " <<map_->getResolution() <<std::endl;

    for(; p.x() <size_grid.x()-step*2; p.x()+=step){
        for(; p.y()<size_grid.y()-step*2; p.y()+=step){
            for(;p.z()< size_grid.z()-step*2; p.z()+=step){
                if(map_->getState(p) == 0){
                    Eigen::Vector3f p_free;
                    map_->convert(p,p_free);
                    free_pts.col(cpt++) = p_free;
                }
            }
        }
    }
    free_pts.conservativeResize(3,cpt);
    lattice_map_.reserve(pts.cols()*4+estimatee_volume*4);
    Eigen::Isometry3f T;

    std::cout << params.init_invsigma.transpose() <<std::endl;

    buildPermutohedralpt2pt(pts, lattice_map_, params.init_invsigma);

    updatePermutoWeight(free_pts, lattice_map_, params.init_invsigma);


    blur(lattice_map_, false);
    //        blur(lattice_map_, true);

}

void FrontierEvaluator::sampleView(const Eigen::Matrix<float, 3, Eigen::Dynamic> &pts, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &goals)
{

    std::vector<Aggregatept2pt> aggregate_results;
    aggregate_results.resize(pts.cols());

    Eigen::Matrix<float, 3, Eigen::Dynamic> pts_tmp(3,pts.cols());
    computeTargetpt2pt(pts, lattice_map_, params.init_invsigma, aggregate_results,params);

    int cpt = 0;
    for(size_t i = 0; i < aggregate_results.size(); ++i){
        if(aggregate_results[i]._M0 > 0.02)
            pts_tmp.col(cpt++) = aggregate_results[i]._M1;

    }
    if(cpt == 0)
        return;

    pts_tmp.conservativeResize(3,cpt);
    aggregate_results.resize(pts_tmp.cols());
    goals.resize(aggregate_results.size());

    computeTargetpt2pt(pts_tmp, lattice_map_, params.init_invsigma, aggregate_results,params);
    for(size_t i = 0; i < aggregate_results.size(); ++i){
        goals[i] << aggregate_results[i].pos_, aggregate_results[i]._M0;
        std::cout << goals[i].transpose() <<std::endl;
    }


}

void FrontierEvaluator::sampleTarget(const Eigen::Matrix<float, 3, Eigen::Dynamic> & pts, std::vector<Aggregatept2pt> &aggregate_results)
{

    Eigen::Isometry3f T;


    aggregate_results.resize(pts.cols());

    computeTargetpt2pt(pts, lattice_map_, params.init_invsigma, aggregate_results,params);


    for(size_t i = 0; i < aggregate_results.size();++i){

        aggregate_results[i]._M0 = (aggregate_results[i]._M0);
    }

}

void FrontierEvaluator::findAccessiblePose(const Eigen::Matrix<float, 3, Eigen::Dynamic> &target, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &goals)
{
    Eigen::Vector3i size_grid;

    map_->get3DGridSize(size_grid);
    float resolution = map_->getResolution();
    int step = (int)(1);
    int min_dist = 1./resolution;
    int max_dist = 3.6/resolution;
    int cpt = 0;

    goals.resize(target.cols());
    Eigen::Vector3f tmp_vec3;
    for(int i = 0; i < target.cols();++i){

        bool found = false;
        Eigen::Vector3i p_center;
        Eigen::Vector3i p(0,0,0);
        float dist;
        map_->convert(target.col(i),p_center);
        //neg
        for(p.x() = std::max(p_center.x()-max_dist,size_grid.x()); p.x() < std::min(p_center.x()-min_dist,size_grid.x()) && !found; p.x()+=step){
            for(p.y() = std::max(p_center.y()-max_dist,size_grid.y()); p.y() < std::min(p_center.y()-min_dist,size_grid.y())&& !found; p.y()+=step){
                for(p.z() = std::max(p_center.z()-max_dist,size_grid.z()); p.z() < std::min(p_center.z()-min_dist,size_grid.z())&& !found; p.z()+=step){
                    map_->getDistance(p,dist);
                    if(map_->getState(p) != 0 && dist > 0.6f){

                        map_->convert(p,tmp_vec3);
                        float dot = (tmp_vec3-target.col(i).head(3)).normalized().z();
                        float angle = acos(dot);

                        if(dot < 0){
                            angle = angle-M_PI_2;
                        }else{
                            angle = M_PI_2-angle;
                        }

                        if(angle <0.4){
                            goals[i] << tmp_vec3,1;
                            found = true;
                        }
                    }
                }
            }
        }



        //positive


        for(p.x() = std::min(p_center.x()+min_dist,size_grid.x()); p.x() < std::min(p_center.x()-min_dist,size_grid.x()) && !found; p.x()+=step){
            for(p.y() = std::min(p_center.y()+min_dist,size_grid.y()); p.y() < std::min(p_center.y()-min_dist,size_grid.y())&& !found; p.y()+=step){
                for(p.z() = std::min(p_center.z()+min_dist,size_grid.z()); p.z() < std::min(p_center.z()-min_dist,size_grid.z())&& !found; p.z()+=step){
                    map_->getDistance(p,dist);
                    if(map_->getState(p) != 0 && dist > 0.6f){
                        float dot = (tmp_vec3-target.col(i).head(3)).normalized().z();
                        float angle = acos(dot);

                        if(dot < 0){
                            angle = angle-M_PI_2;
                        }else{
                            angle = M_PI_2-angle;
                        }

                        if(angle <0.4){
                            goals[i] << tmp_vec3,1;
                            found = true;
                        }

                    }
                }
            }
        }
    }

}

void FrontierEvaluator::radiusSelection(const std::vector<Aggregatept2pt> &aggregate_results, std::vector<Aggregatept2pt> &selectedTarget)
{
    if(aggregate_results.size()<0)
        return;
    selectedTarget.reserve(aggregate_results.size()*0.2);

    size_t j;
    //insert first pt
    selectedTarget.push_back(aggregate_results[0]);
    float sq_min_radius = 2.5*2.5;
    //    NanoFlannCloud cloud;
    //    cloud.pts_ = &selectedTarget;

    //    typedef nanoflann::KDTreeSingleIndexAdaptor<
    //            nanoflann::L2_Simple_Adaptor<float, NanoFlannCloud > ,
    //            NanoFlannCloud,
    //            3> my_kd_tree_t;


    //    size_t num_results = 1;
    //    std::vector<size_t>   ret_index(num_results);
    //    std::vector<float> out_dist_sqr(num_results);
    //    my_kd_tree_t   index(3 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
    //    index.buildIndex();

    //    float mean = 0;
    //    for(size_t i = 0; i < aggregate_results.size(); ++i)
    //    {
    //        mean+=aggregate_results[i]._M0;
    //    }
    //    mean/=(float)aggregate_results.size();

    float thresh = aggregate_results[0]._M0*0.6;

    for(size_t i = 1; i < aggregate_results.size(); ++i){
        //        float query_pt[3] = { aggregate_results[i].pos_.x(),aggregate_results[i].pos_.y(),aggregate_results[i].pos_.z()};

        //        num_results = index.knnSearch(query_pt, num_results, &ret_index[0], &out_dist_sqr[0]);
        //        if(num_results>0){
        //            if(sq_min_radius > out_dist_sqr[0]){
        //                selectedTarget.push_back(aggregate_results[i]);
        //                Timer t;
        //                index.buildIndex();
        //                std::cout <<"build time :" << t.elapsed_micro() <<std::endl;
        //            }
        //        }
        if(selectedTarget.size() > 50 && aggregate_results[i]._M0 < thresh)
            break;
        for(j = 0;j < selectedTarget.size();++j){
            if((selectedTarget[j].pos_-aggregate_results[i].pos_).squaredNorm() < sq_min_radius){
                break;
            }
        }
        if(j == selectedTarget.size()){
            selectedTarget.push_back(aggregate_results[i]);
        }

        if(selectedTarget.size()>350){
            break;
        }
    }


}



void FrontierEvaluator::superpointclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, std::vector<bool> &target_frontiers)
{

    std::cout << "SuperVoxel clustering\n";
    typedef pcl::PointXYZ PointT;



    float voxel_resolution = map_->getResolution()*2;
    float seed_resolution = _cluster_radius;
    float spatial_importance = 1.f;

    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setInputCloud (cloud);
    super.setSpatialImportance (spatial_importance);

    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
    super.extract (supervoxel_clusters);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    super.setInputCloud (cloud);
    super.setColorImportance (0);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (0);
    kdtree.setInputCloud (cloud);

    std::cout << "Number of cluster: " << supervoxel_clusters.size() << std::endl;

    for (auto itr = supervoxel_clusters.begin (); itr != supervoxel_clusters.end (); itr++){
        if(itr->second->voxels_->size()<3){
            continue;
        }
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        pcl::PointXYZRGBA centroid_rgba;
        itr->second->getCentroidPoint(centroid_rgba);
        pcl::PointXYZ centroid(centroid_rgba.x,centroid_rgba.y,centroid_rgba.z);

        if ( kdtree.nearestKSearch(centroid, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

            Features f;
            Eigen::Vector3i c;
            map_->convert(cloud->points[pointIdxNKNSearch[0]].getVector3fMap()  ,c);
            map_->getFrontierFeatures(c,f);
            f.planarity = 1.;// / max for normalized information
            map_->setFrontierFeatures(c,f);
            int idx = map_->get_idx(c);

            target_frontiers[idx] = true;
        }

    }
    std::cout << "Number of cluster: " << supervoxel_clusters.size() << std::endl;

}

void FrontierEvaluator::nn_clustering(std::vector<bool> &target_frontiers, std::vector<uint8_t> &frontier_map, float mav_radius)
{
    Timer t;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers;
    map_->getFrontiers(frontiers);
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers_list;
    frontiers_list.reserve(frontiers.size());

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1,10);
    for(size_t i = 0;i<frontiers.size();++i){
        size_t idx = map_->get_idx(frontiers[i]);
        if(frontier_map[idx] == 1){
            frontiers_list.push_back(frontiers[i]);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //last update time and update range
    //
    cloud->resize(frontiers_list.size());
    Eigen::Vector3i size;
    map_->get3DGridSize(size);
    target_frontiers.resize(size.x()*size.y()*size.z(),false);

    //    std::vector<bool> visited(frontiers.size(),0);



    std::unordered_map<size_t,std::pair<int,bool>> visited;
    for(int i = 0;i < frontiers_list.size(); ++i){
        visited.insert(std::pair<size_t,std::pair<int,bool>>(map_->get_idx(frontiers_list[i]),
                                                             std::pair<int,bool>(i,false)));
        Eigen::Vector3f p;
        map_->convert(frontiers_list[i],p);
        cloud->points[i].x = p.x();
        cloud->points[i].y = p.y();
        cloud->points[i].z = p.z();
    }
    superpointclustering(cloud, target_frontiers);

      /*
    do{
        //        size_t tmp;
        //        enqueue(key);
        const auto &it = visited.begin();

        if(it == visited.end())
            break;
        Eigen::Vector3i p;
        p=frontiers_list[it->second.first];

        int count = 1;
        plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> seq;
        julia_clustering(p,visited,seq,true);
        if(seq.size() > 1./(map_->getResolution()*map_->getResolution())){
            int k;
            plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>::iterator its,its1;
            Eigen::Vector3f centroid(0,0,0);
            for(its = seq.begin(); its != seq.end();  its++ ){
                centroid+=(*its).cast<float>();
            }
            centroid *= 1.f/(static_cast<float>(seq.size()));

            Eigen::Vector3i center;
            float dist = -1;
            for(its = seq.begin(); its != seq.end();  its++ ){

                if(dist < 0){
                    float d = 0;
                    map_->getDistance(*its,d);
                    if(mav_radius+map_->getResolution()*0.5f < d){
                        center = *its;
                        dist = ((*its).cast<float>()-centroid).squaredNorm();
                    }
                }else{
                    Eigen::Vector3f pts = (*its).cast<float>();

                    if((pts-centroid).squaredNorm() < dist){
                        dist = (pts-centroid).squaredNorm();
                        center = *its;
                    }
                }

            }
            if(dist<0){
                continue;
            }

            Features f;

            //maybe check distance obstacle
            int idx = map_->get_idx(center);

            target_frontiers[idx] = true;

            int color = distribution(generator);
            for(its1 = seq.begin(); its1 != seq.end();its1++){
                target_frontiers[map_->get_idx((*its1))] = true;

                map_->getFrontierFeatures(*its1,f);
                f.planarity =color/10.;// / max for normalized information
                map_->setFrontierFeatures(*its1,f);
            }

            map_->getFrontierFeatures(center,f);
            f.planarity = 1.;// / max for normalized information
            map_->setFrontierFeatures(center,f);


            //add target
        }


    }while(true);

    std::cout << "Julia frontier clustering number: " << target_frontiers.size() << std::endl;
*/
}

void FrontierEvaluator::julia_cluster(std::vector<bool> &target_frontiers,std::vector<uint8_t> &frontier_map)
{
    Timer t;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers;
    map_->getFrontiers(frontiers);
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers_list;
    frontiers_list.reserve(frontiers.size());

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1,10);
    for(size_t i = 0;i<frontiers.size();++i){
        size_t idx = map_->get_idx(frontiers[i]);
        if(frontier_map[idx] == 1){
            frontiers_list.push_back(frontiers[i]);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //last update time and update range
    //
    cloud->resize(frontiers_list.size());
    Eigen::Vector3i size;
    map_->get3DGridSize(size);
    target_frontiers.resize(size.x()*size.y()*size.z(),false);

    //    std::vector<bool> visited(frontiers.size(),0);
    std::unordered_map<size_t,std::pair<int,bool>> visited;
    for(int i = 0;i < frontiers_list.size(); ++i){
        visited.insert(std::pair<size_t,std::pair<int,bool>>(map_->get_idx(frontiers_list[i]),
                                                             std::pair<int,bool>(i,false)));
        Eigen::Vector3f p;
        map_->convert(frontiers_list[i],p);
        cloud->points[i].x = p.x();
        cloud->points[i].y = p.y();
        cloud->points[i].z = p.z();
    }
    std::cout << "Start clustering" <<std::endl;
    do{
        //        size_t tmp;
        //        enqueue(key);
        const auto &it = visited.begin();

        if(it == visited.end())
            break;
        Eigen::Vector3i p;
        p=frontiers_list[it->second.first];

        int count = 1;
        plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> seq;
        julia_clustering(p,visited,seq,true);
        if(seq.size() > 1./(map_->getResolution()*map_->getResolution())){
            int k;
            plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>::iterator its,its1;
            for( k = 0, its = seq.begin(); its != seq.end() && k <= seq.size()/2; its++ , k++);

            Features f;

            //maybe check distance obstacle

            int color = distribution(generator);
            for(its1 = seq.begin(); its1 != seq.end();its1++){
                int idx = map_->get_idx(*its1);
                //                target_frontiers[idx] = true;
                map_->getFrontierFeatures(*its1,f);
                f.planarity =color/11.;// / max for normalized information
                map_->setFrontierFeatures(*its1,f);
            }
            int idx = map_->get_idx(*its);

            target_frontiers[idx] = true;
            map_->getFrontierFeatures(*its,f);
            f.planarity = 1;// / max for normalized information
            map_->setFrontierFeatures(*its,f);

            std::cout << "new cluster " << seq.size() <<" " << its->transpose() <<std::endl;

            //add target
        }
        if(it == visited.end())
            break;

    }while(true);

    std::cout << "Julia frontier clustering number: " << target_frontiers.size() << std::endl;
}

bool FrontierEvaluator::julia_clustering(const Eigen::Vector3i &p,
                                         std::unordered_map<size_t, std::pair<int, bool> > &visited_map,
                                         plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &seq, bool backfront)
{
    if(!map_->isInsideBuf(p))
        return false;

    static constexpr int indices_array[3][18] = {{1,0,0,-1,0,0, 1,-1,1,-1,1,-1,1,-1,0,0,0},
                                                 {0,1,0,0,-1,0, 1,1,-1,-1,0,0,0,0,1,-1,1,-1},
                                                 {0,0,1,0,0,-1, 0,0,0,0,1,1,-1,-1,1,1,-1,-1},
                                                };

    int idx = map_->get_idx(p);
    if(visited_map.find(idx) != visited_map.end()){

        if(!visited_map[idx].second){

            visited_map.erase(idx);

            if(backfront)seq.push_back(p);
            else seq.push_front(p);


            //check 6 faces

            for(int i = 0; i < 18; ++i){

                if(julia_clustering(p+Eigen::Vector3i(indices_array[0][i],indices_array[1][i],indices_array[2][i]),visited_map,seq,backfront)){
                    backfront = true;
                }

            }



            return true;

        }
        return false;
    }
    return false;


}

