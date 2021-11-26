#include "../../include/planner/nearestfrontierplanner.h"
#include "map_core/../../utils/timer.hpp"
NearestFrontierPlanner::NearestFrontierPlanner(const ros::NodeHandle& nh, const ros::NodeHandle &nh_private_i)
    :PlannerBase(nh,nh_private_i)
    ,frontiersEvaluator_(nh,nh_private_i,searchspace_)
    ,vel_traj(nh,nh_private_i,&constraint_,&state_,&odom_data_,searchspace_)
    //    trajectoryManager_(nh,nh_private_i,&constraint_,&state_,&odom_data_)
{

    view_utility.reset(new ViewUtility(nh,nh_private_i,searchspace_));
    nndjikstra.reset(new nnfrontiers_utils(nh,nh_private_i,searchspace_));
    nh_private_.param("useRapid", useRapid_, useRapid_);

    path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planner/djik_path", 10);
    mapping_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/planner/planner_mapping", 10);

    traj_ompl = TrajectoryOMPL(nh,nh_private_i,searchspace_,&constraint_);


}

void NearestFrontierPlanner::exploration_behavior()
{

    Eigen::Affine3d current_pose_ = odom_data_.current_pose();
    Eigen::Vector3d current_velocity_ = odom_data_.current_velocity();

    //Make a yaw turn to initialize exploration space
    if(startlookAround_ < 1){
        float yaw = mav_msgs::yawFromQuaternion(
                    (Eigen::Quaterniond)current_pose_.rotation());
        startlookAround_=1;
        Eigen::Vector4d start_pos_4d, start_vel_4d, goal_pos_4d, goal_vel_4d;
        start_pos_4d << current_pose_.translation(),
                mav_msgs::yawFromQuaternion(
                    (Eigen::Quaterniond)current_pose_.rotation());
        start_vel_4d << current_velocity_, 0.0;
        goal_pos_4d << current_pose_.translation(),M_PI*1.7;
        goal_vel_4d << current_velocity_, 0.0;
        mav_trajectory_generation::Trajectory trajectory;
        trajInterface_.computeTrajectoryAndYaw(constraint_,start_pos_4d, start_vel_4d,goal_pos_4d,Eigen::Vector4d::Zero(),
                                               std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                               &trajectory);
        trajInterface_.setTrajectory(trajectory);
        (state_) = PlannerState::YAWTURN;
        trajInterface_.startTrajectory();
        return;
    }


    if(state_ == PlannerState::YAWTURN)
        return;
    Timer t0;

    t0.restart();
    Eigen::Vector3d pos = odom_data_.current_pose().translation();
    float yaw = mav_msgs::yawFromQuaternion(
                (Eigen::Quaterniond)current_pose_.rotation());
    if (yaw < 0) { yaw += 2 * M_PI; }

    float yaw_rate = odom_data_.current_angular_velocity().z();
    //    std::cout << current_velocity_.norm() <<std::endl;
    if(useRapid_){
        //    frontiersEvaluator_.clustering(astar_,pos.cast<float>());
        Eigen::Vector3f desiredRapidVel;
        Eigen::Vector3d vel_wr = (current_pose_.rotation()*current_velocity_);
        Eigen::Vector4d pos4d_wr, vel4d_wr;
        float predicted_yaw;
        if(trajInterface_.isRunning()){
            trajInterface_.getPredictedPose(pos4d_wr,vel4d_wr);
        }else{
            vel_traj.getPredictedPose(pos4d_wr,vel4d_wr);
        }
        //        if(view_utility->computeRapidFrontier(pos4d_wr.head(3).cast<float>(),vel4d_wr.head(3).cast<float>(),pos4d_wr(3),desiredRapidVel)){
        if(view_utility->computeRapidFrontier(pos.head(3).cast<float>(),vel_wr.cast<float>(),yaw,desiredRapidVel)){
            //                        std::cout << "Has found rapid frontier: " << desiredRapidVel.transpose() << std::endl;
            if(trajInterface_.isRunning())
                trajInterface_.stopTrajectory();
            if(!vel_traj.isRunning() && yaw_rate > 0.05)
                return;// waiting yaw_rate stabilization
            //wait yaw rate slow down before sending new rapid trajectory
            //            BoundingMap3D<float> bounds_map = searchspace_->getBoudingMapReal();

            //            if((bounds_map.max_.z() - 0.5 - pos.z()) < 0.1 )
            //                desiredRapidVel.z() = 0;

            //            if((bounds_map.min_.z() - 0.5) < 0.1 )
            //                desiredRapidVel.z() = 0;

            bool sucess = vel_traj.startTrajectory(pos.head(3),vel_wr,Eigen::Vector3d::Zero(),desiredRapidVel.cast<double>(),yaw,searchspace_);
            if(!sucess){
                vel_traj.stopTrajectory();
            }
            //                        std::cout << "Elapsed time: " <<   t0.elapsed_ms() << std::endl;
            Eigen::Vector2i target(0,0);

            view_utility->drawViewUtility(target);

            return;
        }else{
            Eigen::Vector2i target(0,0);

            view_utility->drawViewUtility(target);

            vel_traj.stopTrajectory();
        }
        if(current_velocity_.norm() > 0.1){
            return;
        }
    }

    if(!validate_path_polytraj()){
        //stop trajectory
        std::cout << "[Planner] Trajectory too close of an obstacle! break the trajectory"<< std::endl;
        trajInterface_.stopTrajectory();
        if(vel_traj.isRunning()){
            state_ = PlannerState::TRAJ;
            vel_traj.stopTrajectory();
        }
    }
    if(state_ != PlannerState::IDLE)
        return;
    std::vector<bool> target_frontier;
    std::vector<float> edt;
    std::vector<uint8_t> states_map;
    t0.restart();
    searchspace_->getEdtMap(edt);

    searchspace_->getMapState(states_map);
    nndjikstra->updateFrontierMap(states_map);

    frontiersEvaluator_.nn_clustering(target_frontier,nndjikstra->getFrontier_map(),constraint_.margin_);

    Eigen::Vector3i pos_int;
    searchspace_->convert(pos.cast<float>(),pos_int);
    //    bool found_path = nndjikstra.nn_frontier_djikstra(pos_int,states_map,target_frontier);
    bool found_path = nndjikstra->djiktra_priority_queue(pos_int,states_map,target_frontier);

    std::cout << "Pipeline t(ms)" << t0.elapsed_ms() <<std::endl;


    if(!found_path)
    {
        std::cout << "No cluster centroid mutual visibility path found\n";
        target_frontier.clear();
        target_frontier.resize(states_map.size(),false);

        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers;
        searchspace_->getFrontiers(frontiers);
        bool has_frontier = false;
        for(size_t i = 0;i<frontiers.size();++i){

            size_t idx = searchspace_->get_idx(frontiers[i]);
            float d;
            searchspace_->getDistance(frontiers[i],d);
            if(d>constraint_.margin_+searchspace_->getResolution() && nndjikstra->getFrontier_map()[i] == 1){
                target_frontier[idx] = true;
                has_frontier = true;
            }

        }
        if(!has_frontier){
            std::cout << "Planner finish no frontier/path found\n";
            state_ = PlannerState::STOP;
            return;
        }

        found_path = nndjikstra->djiktra_priority_queue(pos_int,states_map,target_frontier);
        if(!found_path){
            std::cout << "Planner finish no frontier/path found\n";
            state_ = PlannerState::STOP;
            return;
        }
    }
    trajvizu.lock();
    path_ = nndjikstra->getSimplifiedPath();
    Eigen::Vector3f goal = nndjikstra->getGoal();
    trajvizu.unlock();

    current_pose_ = odom_data_.current_pose();
    current_velocity_ = odom_data_.current_velocity();
    yaw = mav_msgs::yawFromQuaternion(
                (Eigen::Quaterniond)current_pose_.rotation());
    if (yaw < 0) { yaw += 2 * M_PI; }

    Eigen::Vector4d start_pos_4d, start_vel_4d, goal_pos_4d, goal_vel_4d;
    start_pos_4d << current_pose_.translation(),yaw;
    start_vel_4d << current_velocity_, 0.0;

    //compute heading angle

    mav_trajectory_generation::Trajectory trajectory;
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> path_4d;
    float angle_radians;

    if(path_.size()> 0){
        goal_pos_4d.head(3) = path_[path_.size()-1].cast<double>();
        Eigen::Vector3f dir = (path_[1]-start_pos_4d.cast<float>().head(3)).normalized();
        if(abs(dir.y()) < 0.0001 && abs(dir.x()) < 0.00001){
            //no rotation
            angle_radians = yaw;

        }else{
            angle_radians = atan2(dir.y(),dir.x());
            if (angle_radians < 0) { angle_radians += 2 * M_PI; }
            while(std::fabs(yaw-angle_radians) > M_PI){
                if(yaw < angle_radians){
                    angle_radians -= 2 * M_PI;
                }else{
                    angle_radians += 2 * M_PI;
                }
            }
            yaw = angle_radians;
        }
        path_4d.push_back(Eigen::Vector4d(start_pos_4d.x(),start_pos_4d.y(),start_pos_4d.z(),angle_radians));

    }else{
        goal_pos_4d.head(3) = start_pos_4d.head(3);

    }

    if(path_.size()>0)
        for(size_t i = 1; i < path_.size()-1; ++i){

            Eigen::Vector3f dir = (path_[i+1]-path_[i].cast<float>().head(3)).normalized();
            if(abs(dir.y()) < 0.0001 && abs(dir.x()) < 0.00001){
                //no rotation
                angle_radians = yaw;
            }else{
                angle_radians = atan2(dir.y(),dir.x());
                if (angle_radians < 0) { angle_radians += 2 * M_PI; }
                while(std::fabs(yaw-angle_radians) > M_PI){
                    if(yaw < angle_radians){
                        angle_radians -= 2 * M_PI;
                    }else{
                        angle_radians += 2 * M_PI;
                    }
                }
            }

            yaw = angle_radians;

            path_4d.push_back(Eigen::Vector4d(path_[i].x(),path_[i].y(),path_[i].z(),angle_radians));
        }

    Eigen::Vector3f dir_goal_frontier = (goal-goal_pos_4d.cast<float>().head(3)).normalized();
    if(abs(dir_goal_frontier.y()) < 0.0001 && abs(dir_goal_frontier.x()) < 0.00001){
        angle_radians = yaw;
    }else{
        angle_radians = atan2(dir_goal_frontier.y(),dir_goal_frontier.x());
        if (angle_radians < 0) { angle_radians += 2 * M_PI; }
        while(std::fabs(yaw-angle_radians) > M_PI){
            if(yaw < angle_radians){
                angle_radians -= 2 * M_PI;
            }else{
                angle_radians += 2 * M_PI;
            }
        }
    }
    goal_pos_4d.w() = angle_radians;

    {
        float angle_tmp;
        view_utility->computeViewUtility(goal_pos_4d.cast<float>().head(3),angle_tmp);

        float angle_radians  = float(((int)angle_tmp+view_utility->getHorizontal_angle()/2)%360)*M_PI/180.f;

        if (angle_radians < 0) { angle_radians += 2 * M_PI; }
        while(std::fabs(yaw-angle_radians) > M_PI){
            if(yaw < angle_radians){
                angle_radians -= 2 * M_PI;
            }else{
                angle_radians += 2 * M_PI;
            }
        }
        goal_pos_4d.w() = angle_radians;
    }

    trajInterface_.computeTrajectoryAndYaw(constraint_,start_pos_4d, start_vel_4d,goal_pos_4d,Eigen::Vector4d::Zero(),path_4d,
                                           &trajectory);

    trajInterface_.setTrajectory(trajectory);
    (state_) = PlannerState::TRAJ;// djikstra traj is similar to idle state as rapid could get the control
    trajInterface_.startTrajectory();

    planner_logging_.addTime(t0.elapsed_ms());

    //    path_ = nndjikstra.getPath();
    //        while(!collisionChecker_->checkCollisionLine(current_pose_.translation().cast<float>(),posf)){
    //            if(try_count > 100 || mag < 0.3){
    //                std::cout << "No path found" << posf.transpose() <<std::endl;
    //                found = false;
    //                break;
    //            }
    //            try_count++;
    //            mag /= try_count;
    //            posf = (current_pose_.translation() + dir*mag).cast<float>();
    //            bounding_volume_.clamp(posf);

    //        }



}


void NearestFrontierPlanner::publish_visualization()
{
    PlannerBase::publish_visualization();
    publish_path();
    publish_mapping();
    trajvizu.lock();
    //    trajInterface_.publishVizualization(test_traj);
    trajvizu.unlock();

    //    trajectoryManager_.publishTrajectoryVizualisation();
}

void NearestFrontierPlanner::publish_path()
{
    float dist;
    Eigen::Vector3f dir;

    int id = 0;


    trajvizu.lock();

    if(path_.empty()){
        trajvizu.unlock();
        return;
    }

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::LINE_STRIP;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = 1 ;
    path_pub_.publish(mk);


    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 0.29;
    mk.color.g = 0.6;
    mk.color.b = 0.;
    mk.color.a = 1;
    mk.scale.x = 0.2;



    geometry_msgs::Point pt;

    for(int i = 0; i < path_.size()-1; i++){
        pt.x = path_[i].x();
        pt.y = path_[i].y();
        pt.z = path_[i].z();
        mk.points.push_back(pt);

        pt.x = path_[i+1].x();
        pt.y = path_[i+1].y();
        pt.z = path_[i+1].z();
        mk.points.push_back(pt);
    }
    path_pub_.publish(mk);

    trajvizu.unlock();


    //    if(vel_traj.traj_.size() == 0){
    //        std::cout <<"testsdqqqqqqqqqqqqq\n";
    //        return;

    //    }
    //    astar_.reset();


    //    Timer t;
    //    int state = astar_.search(odom_data_.current_pose_.translation(), Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0),
    //                               Eigen::Vector3d(2,-2,1.2),Eigen::Vector3d(0,0,0), false);

    //    if(state)
    //    {
    //        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> path = astar_.getKinoTraj(0.02);
    //        if(path.size() <4)
    //            return;
    //        std::vector<double> times(path.size());
    //        times[0] = 0;
    //        for(size_t i = 1; i < path.size();++i){
    //            times[i] = times[i-1] +0.02;
    //        }

    //        TrajOptim<9,double> optim = TrajOptim<9,double>(searchspace_, constraint_);
    //        optim.fitting(path,times,Eigen::MatrixXd(0,0));
    //        path.clear();
    //        for(double dt = 0; dt < times[times.size()-1]; dt+=0.02){
    //            path.push_back(optim.getPoint(dt));
    //        }

    //        std::cout << "ELAPSED TIME KINOASTAR: " << t.elapsed_ms() <<" " << path.size() <<" " << times[times.size()-1] <<std::endl;

    //        visualization_msgs::Marker mk;
    //        mk.header.frame_id = "world";
    //        mk.header.stamp    = ros::Time::now();
    //        mk.type            = visualization_msgs::Marker::LINE_LIST;
    //        mk.action          = visualization_msgs::Marker::DELETE;
    //        mk.id              = 1 ;
    //        path_pub_.publish(mk);


    //        mk.action             = visualization_msgs::Marker::ADD;
    //        mk.pose.orientation.x = 0.0;
    //        mk.pose.orientation.y = 0.0;
    //        mk.pose.orientation.z = 0.0;
    //        mk.pose.orientation.w = 1.0;

    //        mk.color.r = 1;
    //        mk.color.g = 0;
    //        mk.color.b = 0;
    //        mk.color.a = 1;
    //        mk.scale.x = 0.05;

    //        geometry_msgs::Point pt;

    //        for(int i = 0; i < path.size()-1; i+=3){
    //            pt.x = path[i].x();
    //            pt.y = path[i].y();
    //            pt.z = path[i].z();
    //            mk.points.push_back(pt);

    //            pt.x = path[i+1].x();
    //            pt.y = path[i+1].y();
    //            pt.z = path[i+1].z();
    //            mk.points.push_back(pt);
    //        }

    //        path_pub_.publish(mk);


    //    }else{
    //        std::cout << "Error trajectory" << std::endl;
    //    }

}

void NearestFrontierPlanner::publish_mapping()
{

    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::vector<pcl::PointXYZI> slice;
    view_utility->computeGaussianMap(slice);
    cloud.points.reserve(slice.size());
    if(slice.size() == 0)
        return;
    for (auto & p: slice) {
        cloud.points.push_back(p);
    }


    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";



    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    mapping_pub_.publish(cloud_msg);
}


