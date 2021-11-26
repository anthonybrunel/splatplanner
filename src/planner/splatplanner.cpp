#include "../../include/planner/splatplanner.h"
#include <nav_msgs/Path.h>
SplatPlanner::SplatPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private_i)
    :PlannerBase(nh,nh_private_i),
      frontiersEvaluator_(nh,nh_private_i,searchspace_),
      vel_traj(nh,nh_private_i,&constraint_,&state_,&odom_data_,searchspace_)

    //    trajectoryManager_(nh,nh_private_i,&constraint_,&state_,&odom_data_)
{
    view_utility.reset(new ViewUtility(nh_,nh_private_i,searchspace_));


    path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/planner/path", 10);
    saved_paths_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/planner/saved_path", 1);
    saved_paths_pub2_ = nh_private_.advertise<nav_msgs::Path>("/planner/saved_path2", 1);


    mapping_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/planner/planner_mapping", 10);

    traj_ompl = TrajectoryOMPL(nh,nh_private_i,searchspace_,&constraint_);
}

void SplatPlanner::exploration_behavior()
{
    Eigen::Affine3d current_pose_ = odom_data_.current_pose();
    Eigen::Vector3d current_velocity_ = odom_data_.current_velocity();

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
        if(!useLeeController_){
            mav_trajectory_generation::Trajectory trajectory;
            trajInterface_.computeTrajectoryAndYaw(constraint_,start_pos_4d, start_vel_4d,goal_pos_4d,Eigen::Vector4d::Zero(),
                                                   std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                                   &trajectory);
            //            trajInterface_.computeTrajectory(constraint_, start_pos_4d, start_vel_4d, goal_pos_4d,goal_vel_4d, &trajectory);
            trajInterface_.setTrajectory(trajectory);
            (state_) = PlannerState::TRAJ;
            trajInterface_.startTrajectory();
        }else{

            //            leeTrajInterface.computeTrajectory(constraint_, start_pos_4d, start_vel_4d, goal_pos_4d, goal_vel_4d);
            //            (state_) = PlannerState::TRAJ;

        }
        globalt.restart();
        return;
    }
    //    cv::waitKey(1);

    //    float angle;
    //    view_utility->computeViewUtility(Eigen::Vector3f(0,10,1.2),angle);

    //    view_utility->drawViewUtility(Eigen::Vector2i(0,0));

    //    return;

    if(state_ != IDLE){

        return;

    }
    float score;

    if(globalt.elapsed_ms()>31*60*1000){
        state_ = STOP;
        return;
    }
    //    sleep(2);
    Timer t;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> goals;
    mav_trajectory_generation::Trajectory traj;
    //exit if node kill
    if(!nh_.ok()){
        state_=STOP;
        return;
    }
    computeGoals(goals);
    if(goals.empty()){
        state_ = STOP;

        return;
    }

    //exit if node kill
    if(!nh_.ok()){
        state_=STOP;
        return;
    }

    BestPathRessult result_best_path = computeBestPath(goals, traj);
    if(result_best_path == OMPLFailed){
        backwardPrevTraj();
        return;
    }

    double compute_time = t.elapsed_ms();
    if(result_best_path == NotFound or !nh_.ok() ){
        //force sleep to avoid thread lock
        std::cout << "[Planner] Trajectory gain too low (will retr) or node has been interupt" <<std::endl;

        if(!nh_.ok()){
            state_=STOP;
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>(250)));
            if(!nh_.ok()){
                state_=STOP;
            }
        }
        return;

    }
    //    if(!found){
    //        std::cout << "No available path found..." << std::endl;
    //        for (int i = 0; i < 20; ++i){
    //            found = computeBestPath(goals, traj);

    //            if (found)
    //                break;
    //        }
    //        if(!found){
    //            return;
    //        }
    //    }
    planner_logging_.addTime(compute_time);



    std::cout << "Planning behavior take: " << t.elapsed_ms()*.001f << "(s)" <<std::endl;

    //    sleep(2);

    if(!useLeeController_){
        trajInterface_.setTrajectory(traj);
        trajInterface_.startTrajectory();
    }else{
        //        leeTrajInterface.computeTrajectory(constraint_,lee_start_,lee_vel_,lee_goal_,Eigen::Vector4d::Zero(),lee_waypoints_);
    }
    state_ = TRAJ;
//    trajvizu.lock();


//    for(size_t j = 1;j < saved_path_ompl_.back().size(); ++j){
//        Eigen::Vector3f dir = (saved_path_ompl_.back()[j]-saved_path_ompl_.back()[j-1]);
//        poly_path_.push_back(saved_path_ompl_.back()[j-1].cast<double>());

//        float len = dir.norm();
//        dir.normalize();
//        float incr = 0.3f;
//        float s = incr;

//        while (s < len) {
//            Eigen::Vector3f p = saved_path_ompl_.back()[j-1] + dir*s;
//            poly_path_.push_back(p.cast<double>());
//            s+=incr;
//        }
//        if(j == saved_path_ompl_.back().size() -1)
//            poly_path_.push_back((saved_path_ompl_.back()[j]).cast<double>());
//        else
//            poly_path_.push_back((saved_path_ompl_.back()[j]+dir*0.15).cast<double>());

//    }
//    trajvizu.unlock();

    float distance = 0;
    for (size_t i = 0;i< saved_path_ompl_.size(); ++i) {
        if(saved_path_ompl_[i].empty()){
            continue;
        }
        for(size_t j = 1;j < saved_path_ompl_[i].size(); ++j){
            distance += (saved_path_ompl_[i][j]-saved_path_ompl_[i][j-1]).norm();
        }
    }



    std::cout << ros::Time::now() <<  " Distance done: " << distance << "(m)" << std::endl;
    times.push_back(t.elapsed_ms());
    printTimeComparison();
}

bool SplatPlanner::backwardPrevTraj(){


    std::cout <<
                 "[Planner] The planner is probably near an occupied cell therefore ompl failed, we  search the closest available trajectory"<<std::endl;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    int iter = 0;
    Eigen::Vector3d pos = odom_data_.current_pose().translation();
    float dist;

    {
        // Set up random number generators
        std::mt19937 generator (seed);
        std::uniform_real_distribution<double> uniform01(0.0, 1.0);
        std::uniform_real_distribution<double> uniform_scale(0.1, 0.8);

        while(iter++ <1000){

            // incorrect way
            double theta = 2 * M_PI * uniform01(generator);
            double phi = M_PI * uniform01(generator);
            double scale = uniform01(generator);
            double x = sin(phi) * cos(theta);
            double y = sin(phi) * sin(theta);
            double z = cos(phi);

            Eigen::Vector3f target = (pos + Eigen::Vector3d(x,y,z)*scale).cast<float>();

            searchspace_->getDistance(target,dist);
            if(dist > constraint_.margin_){

                if(validate_segment(pos.cast<float>(),target)){

                    Eigen::Vector4d start;
                    Eigen::Vector4d start_vel(0,0,0,0);

                    trajInterface_.getPredictedPose(start,start_vel);

                    Eigen::Vector4d goal;
                    goal << target.cast<double>(),start.w();
                    mav_trajectory_generation::Trajectory traj;
                    trajInterface_.computeTrajectoryAndYaw(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
                                                           std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                                           &traj);

                    trajInterface_.setTrajectory(traj);
                    trajInterface_.startTrajectory();
                    state_ = TRAJ;

                    return true;
                }
            }

        }
    }

    std::mt19937 generator (seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    std::uniform_real_distribution<double> uniform_scale(0.6, 1.5);

    while(iter++ <1000){

        // incorrect way
        double theta = 2 * M_PI * uniform01(generator);
        double phi = M_PI * uniform01(generator);
        double scale = uniform01(generator);
        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi);

        Eigen::Vector3f target = (pos + Eigen::Vector3d(x,y,z)*scale).cast<float>();

        searchspace_->getDistance(target,dist);
        if(dist > constraint_.margin_){

            if(validate_segment(pos.cast<float>(),target)){

                Eigen::Vector4d start;
                Eigen::Vector4d start_vel(0,0,0,0);

                trajInterface_.getPredictedPose(start,start_vel);

                Eigen::Vector4d goal;
                goal << target.cast<double>(),start.w();
                mav_trajectory_generation::Trajectory traj;
                trajInterface_.computeTrajectoryAndYaw(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
                                                       std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                                       &traj);

                trajInterface_.setTrajectory(traj);
                trajInterface_.startTrajectory();
                state_ = TRAJ;

                return true;
            }
        }

    }

    return false;

}

void SplatPlanner::computeGoals(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &goals)
{
    int num_goals = 20;
    frontiersEvaluator_.sampling(goals);
    goals_.clear();
    goals_.assign(goals.begin(),goals.end());

    if(goals.size()< num_goals){
        return;
    }

    int half_max = num_goals/2;
    int quarter = num_goals/4;





    std::vector<std::pair<int,float> > dist_goals(goals.size()-quarter);
    Eigen::Vector3f pos = odom_data_.current_pose().translation().cast<float>();

    //insert dist goal from start + quarter (we keep the first quarter because the score is high)
    size_t j = 0;
    for(int i = quarter; i < dist_goals.size(); ++i,++j){
        dist_goals[j].first = i;
        dist_goals[j].second = (pos-goals[i].head<3>()).squaredNorm();
    }
    std::sort(dist_goals.begin(),dist_goals.end(),sortDistanceGoals);


    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> goals_tmp(goals.size()-quarter);

    for(size_t i = 0; i < dist_goals.size(); ++i){
        goals_tmp[i] = goals[dist_goals[i].first];
    }
    //shuffle last
    std::shuffle(goals_tmp.begin()+half_max, goals_tmp.end(), rng_);

    j = quarter;//5
    goals.resize(num_goals);
    for(size_t i = 0; i < quarter+half_max; ++i, ++j){
        goals[j] = goals_tmp[i];
    }

}

BestPathRessult SplatPlanner::computeBestPath(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &goals, mav_trajectory_generation::Trajectory &traj)
{
    trajvizu.lock();
    lee_waypoints_.clear();
    path_ompl_.clear();
    Eigen::Vector4d tmp_p,tmp_v;
    if(!useLeeController_)
        trajInterface_.getPredictedPose(tmp_p,tmp_v);
    else{
        //        leeTrajInterface.predictPosVel(tmp_p,tmp_v);
    }
    Eigen::Vector3d pos = tmp_p.head(3);
    Eigen::Vector3d vel = tmp_v.head(3);


    paths_ = std::vector<std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>>>(goals.size());
    paths_yaw_ = std::vector<std::vector<double>>(goals.size());
    std::cout << "[Planner]: OMPL planning search path toward "<< goals.size() << " goals" << std::endl;

    if(!traj_ompl.setStart(pos.cast<float>())){
        trajvizu.unlock();
        return OMPLFailed;
    }

    ob::PlannerStatus::StatusType omplStatus;
    int traj_count = 0;
    for(size_t i = 0; i < paths_.size();++i){
        bool found_path = false;;
        traj_ompl.setNewGoal(goals[i].head(3));
        omplStatus = traj_ompl.solvePath(paths_[i]);
        if(omplStatus != ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION &&
                omplStatus != ob::PlannerStatus::StatusType::EXACT_SOLUTION){
            paths_[i].clear();
            if(omplStatus == ob::PlannerStatus::StatusType::INVALID_START){
                trajvizu.unlock();
                return OMPLFailed;

            }
        }else{
            traj_count++;
        }
    }
    std::cout << "[Planner]: Found " << traj_count << " trajectory, computing best trajectory" << std::endl;

    size_t best_path=0;
    float min_time = 2200000;
    bool found = 0;
    float best_score = 0;
    float best_angle=0;
    for(size_t i = 0; i < paths_.size();++i){
        if(paths_[i].size() < 2){
            continue;
        }
        float duration = 0;
        float score = 0;

        float score_along_path = 0;


        float u_vol = 0;
        for(size_t j = 1; j < paths_[i].size();++j){

            const double distance = (paths_[i][j] - paths_[i][j-1]).norm();

            double acc = 2.5;
            const double acc_time = constraint_.max_v_ / acc;
            const double acc_distance = 0.5 * constraint_.max_v_ * acc_time;
            if (distance < 2.0 * acc_distance) {
                duration += 2.0 * std::sqrt(distance / acc);
            } else {
                duration += 2.0 * acc_time + (distance - 2.0 * acc_distance) / constraint_.max_v_;
            }
        }
        std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > dir_views;

        Eigen::Matrix<float,3,Eigen::Dynamic> checkpts(3,paths_[i].size()-1);
        for(size_t j = 1; j < paths_[i].size();++j){
            checkpts.col(j-1) = paths_[i][j].cast<float>();
        }


        float angle_tmp = 0;
        score = view_utility->computeViewUtility(paths_[i][paths_[i].size()-1],angle_tmp);

        float angle_cam  = float(((int)angle_tmp+view_utility->getHorizontal_angle()/2)%360)*M_PI/180.f;

        paths_yaw_[i].push_back(angle_cam);

        //        score *=score;

        //        //use frontier score to guide exploration
        //        if(score > 0 && goals[i].w() > 0){

        //            score = goals[i].w()*score/(goals[i].w()+score)+1;

        //        }else{
        //            score = 0;
        //        }
        if(score < 0.01)
            continue;
        score /=(1+duration);
        if(best_score < score+0.00001){
            best_score = score;
            best_path = i;
            best_angle = angle_tmp;
            found = true;
        }
    }


    if(!found){
        trajvizu.unlock();
        return NotFound;
    }
    std::cout << "[Planner]: Beest trajectory extracted computing polynomial trajectory..." << std::endl;
    best_idx_ = best_path;

    path_ompl_.assign(paths_[best_path].begin(),paths_[best_path].end());
    saved_path_ompl_.push_back(path_ompl_);//save for vizualization


    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> path_4d;
    path_4d.reserve(path_ompl_.size());
    Eigen::Vector4d goal(path_ompl_[path_ompl_.size()-1].x(),path_ompl_[path_ompl_.size()-1].y(),
            path_ompl_[path_ompl_.size()-1].z(),0);

    //set yaw
    //    goal.w() = std::atan2(goals[best_path].x()-goal.x(), -goals[best_path].y() +goal.y()) - M_PI/2.;
    Eigen::Vector4d start;
    Eigen::Vector4d start_vel(0,0,0,0);
    trajInterface_.getPredictedPose(start,start_vel);
    start.w() = norm_angle_2pi(start.w());
    float yaw = start.w();
    float angle_tmp;

    if(path_ompl_.size() > 2){

        for(size_t i = 1; i < path_ompl_.size()-1; ++i){

            view_utility->computeViewUtility(path_ompl_[i],angle_tmp);

            float angle_radians  = float(((int)angle_tmp+view_utility->getHorizontal_angle()/2)%360)*M_PI/180.f;
            if (angle_radians < 0) { angle_radians += 2 * M_PI; }
            while(std::fabs(yaw-angle_radians) > M_PI){
                if(yaw < angle_radians){
                    angle_radians -= 2 * M_PI;
                }else{
                    angle_radians += 2 * M_PI;
                }
            }
            path_4d.push_back(Eigen::Vector4d(path_ompl_[i].x(),path_ompl_[i].y(),path_ompl_[i].z(),angle_radians));
            yaw = angle_radians;
        }

        trajvizu.unlock();

        float angle_radians  = float(((int)best_angle+view_utility->getHorizontal_angle()/2)%360)*M_PI/180.f;
        if (angle_radians < 0) { angle_radians += 2 * M_PI; }
        while(std::fabs(yaw-angle_radians) > M_PI){
            if(yaw < angle_radians){
                angle_radians -= 2 * M_PI;
            }else{
                angle_radians += 2 * M_PI;
            }
        }

        goal.w() = angle_radians;

        trajInterface_.computeTrajectoryAndYaw(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
                                               path_4d,&traj);


        //        trajInterface_.computeTrajectory(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
        //                                         path_4d,&traj);
    }else{
        trajvizu.unlock();


        float angle_radians  = float(((int)best_angle+view_utility->getHorizontal_angle()/2)%360)*M_PI/180.f;
        if (angle_radians < 0) { angle_radians += 2 * M_PI; }
        while(std::fabs(start.w()-angle_radians) > M_PI){
            if(start.w() < angle_radians){
                start.w() += M_PI*2;
            }else{
                angle_radians += 2 * M_PI;
            }
        }
        goal.w() = angle_radians;
        trajInterface_.computeTrajectoryAndYaw(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
                                               std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                               &traj);
        //        trajInterface_.computeTrajectory(constraint_,start,start_vel,goal,Eigen::Vector4d::Zero(),
        //                                         &traj);
    }
    return Found;
}


void SplatPlanner::publish_visualization()
{

    PlannerBase::publish_visualization();
    //    publish_path();
    publish_executed_path();
    //    trajvizu.lock();
    //    trajInterface_.publishVizualization(test_traj);
    //    trajvizu.unlock();

    //    trajectoryManager_.publishTrajectoryVizualisation();
}

void SplatPlanner::publish_path()
{
    Eigen::Vector3f dir;
    visualization_msgs::MarkerArray markers;

    int id = 0;


    trajvizu.lock();

    path_pub_.publish(markers);
    for(size_t j = 0; j < paths_.size();++j){
        if(paths_[j].empty()){
            continue;
        }
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp    = ros::Time::now();
        mk.type            = visualization_msgs::Marker::LINE_STRIP;
        mk.id              = j ;


        mk.action             = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.lifetime = ros::Duration(1.);
        if(best_idx_ == j){
            mk.color.r = 50/255.;
            mk.color.g = 205/255.;
            mk.color.b = 50/255.;
            mk.scale.x = 0.15;

        }else{
            mk.color.r = 1.0;
            mk.color.g = 0.49;
            mk.color.b = 0.;
            mk.scale.x = 0.1;

        }
        mk.color.a = 1;
        ;


        geometry_msgs::Point pt;

        for(int i = 0; i < paths_[j].size()-1; i++){
            pt.x = paths_[j][i].x();
            pt.y = paths_[j][i].y();
            pt.z = paths_[j][i].z();
            mk.points.push_back(pt);

            pt.x = paths_[j][i+1].x();
            pt.y = paths_[j][i+1].y();
            pt.z = paths_[j][i+1].z();
            mk.points.push_back(pt);

        }


        Eigen::Vector3d dir = Eigen::Vector3d(cos(paths_yaw_[j][0]),sin(paths_yaw_[j][0]),0);
        visualization_msgs::Marker cams_mk = drawCamera(paths_[j].back().cast<double>(),dir,j);

        if(best_idx_ == j){
            cams_mk.color.r = 34/255.;
            cams_mk.color.g = 139/255.;
            cams_mk.color.b = 34/255.;
            cams_mk.scale.x = 0.15;

        }else{
            cams_mk.color.r = 1.0;
            cams_mk.color.g = 0.0;
            cams_mk.color.b = 0.;
            cams_mk.scale.x = 0.05;

        }
        markers.markers.push_back(cams_mk);

        markers.markers.push_back(mk);
    }

    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= "world";
    sphere_list.header.stamp= ros::Time::now();
    sphere_list.ns= "spheres";
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;

    sphere_list.id = 0;

    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;


    sphere_list.scale.x = 0.8;
    sphere_list.scale.y = 0.8;
    sphere_list.scale.z = 0.8;
    sphere_list.color.r = 0.19;
    sphere_list.color.g = 0.55;
    sphere_list.color.b = 0.91;
    sphere_list.lifetime = ros::Duration(1.5);
    sphere_list.color.a = 1.;
    //    sphere_list.colors.resize(goals_.size())
    for(size_t j = 0; j < goals_.size();++j){
        geometry_msgs::Point p;
        p.x = goals_[j].x();
        p.y = goals_[j].y();
        p.z = goals_[j].z();

        sphere_list.points.push_back(p);
    }

    markers.markers.push_back(sphere_list);

    path_pub_.publish(markers);




    trajvizu.unlock();
}

void SplatPlanner::publish_executed_path()
{

    visualization_msgs::MarkerArray markers_paths;
    trajvizu.lock();

    float max_r = 229/255.,max_g = 122/255.,max_b = 39./255.;

    //    float max_r = 20/255.,max_g = 70/255.,max_b = 1.;
    float step_r = (1.-max_r)/((float)saved_path_ompl_.size()-1.f),
            step_g = (1.-max_g)/((float)saved_path_ompl_.size()-1.f),
            step_b = (1.-max_b)/((float)saved_path_ompl_.size()-1.f);

    saved_paths_pub_.publish(markers_paths);
    nav_msgs::Path path;
    if(poly_path_.size()<1){
        poly_path_.push_back(odom_data_.current_pose().translation().cast<double>());
    }else{
        Eigen::Vector3d p = odom_data_.current_pose().translation().cast<double>();
        if((poly_path_.back()-p).norm()>0.01){
            poly_path_.push_back(odom_data_.current_pose().translation().cast<double>());
        }
    }
    if(poly_path_.size()<=1){
        trajvizu.unlock();
        return;
    }
    if(poly_path_.empty()){
        trajvizu.unlock();
        return;
    }
    double size = 20;
    double old = 300;
    path.poses.reserve(3500);
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    visualization_msgs::Marker mk;





    markers_paths.markers.clear();

    for(int i = 1; i < poly_path_.size(); i++){

        double alpha = 1.;

        if(poly_path_.size()-i > old){
            continue;
        }else{
//            if(poly_path_.size()-i>size){
//                double path_size = std::min((double)poly_path_.size(),old+size);
//                alpha = (old-(path_size-size)+i)/old;
//            }
            double path_size = std::min((double)poly_path_.size(),old);
            alpha = (path_size-(poly_path_.size()-i))/path_size;
            mk.action   = visualization_msgs::Marker::ADD;
        }
        if(alpha < 0.00001){
            continue;
        }
        mk.header.frame_id = "world";
        mk.header.stamp    = ros::Time::now();
        mk.type            = visualization_msgs::Marker::LINE_LIST;
        mk.id              =  i;
        mk.ns              =  "traj";


        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
//        mk.lifetime = ros::Duration(60.);

        mk.color.r = 229./255.;
        mk.color.g = 43/255.;
        mk.color.b = 80/255.;
        //        mk.color.r = 49./255.;
        //        mk.color.g = 140./255.;
        //        mk.color.b = 231./255.;
        //        mk.color.r = 0.0;
        //        mk.color.g = 127./255.;
        //        mk.color.b = 1.;
        mk.color.a = alpha;
        mk.scale.x = 0.20;


        geometry_msgs::Point pt;



        pt.x = poly_path_[i-1].x();
        pt.y = poly_path_[i-1].y();
        pt.z = poly_path_[i-1].z();
        mk.points.push_back(pt);



        pt.x = poly_path_[i].x();
        pt.y = poly_path_[i].y();
        pt.z = poly_path_[i].z();
        mk.points.push_back(pt);
        markers_paths.markers.push_back(mk);
        mk.points.clear();

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = pt.z;
        pose.pose.orientation.w = 1.;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        path.poses.push_back(pose);

    }

//    for(int i = 1; i < poly_path_.size(); i++){

//        double alpha = 1.;

//        if(poly_path_.size()-i > old+size){
//            mk.action   = visualization_msgs::Marker::DELETE;
//        }else{
//            if(poly_path_.size()-i>size){
//                double path_size = std::min((double)poly_path_.size(),old+size);
//                alpha = (old-(path_size-size)+i)/old;
//            }
//            mk.action   = visualization_msgs::Marker::ADD;
//        }

//        //        mk.color.r = 49./255.;
//        //        mk.color.g = 140./255.;
//        //        mk.color.b = 231./255.;
//        //        mk.color.r = 0.0;
//        //        mk.color.g = 127./255.;
//        //        mk.color.b = 1.;



//        geometry_msgs::Point pt;




//        mk.header.frame_id = "world";
//        mk.header.stamp    = ros::Time::now();
//        mk.type            = visualization_msgs::Marker::POINTS;
//        mk.id              =  i+100000;
//        mk.ns       = "pts";

//        mk.pose.orientation.x = 0.0;
//        mk.pose.orientation.y = 0.0;
//        mk.pose.orientation.z = 0.0;
//        mk.pose.orientation.w = 1.0;
////        mk.lifetime = ros::Duration(60.);

//        mk.color.r = 229./255.;
//        mk.color.g = 43/255.;
//        mk.color.b = 80/255.;
//        mk.color.a = alpha;
//        mk.scale.x = 0.20;
//        mk.scale.y = 0.20;



//        pt.x = poly_path_[i-1].x();
//        pt.y = poly_path_[i-1].y();
//        pt.z = poly_path_[i-1].z();
//        mk.points.push_back(pt);


//    }

    saved_paths_pub2_.publish(path);

    saved_paths_pub_.publish(markers_paths);
    trajvizu.unlock();

}

void SplatPlanner::local_planning()
{
    //    Eigen::Vector3d pos = odom_data_.current_pose().translation();
    //    int h_angle;
    //    float score = view_utility->computeViewFrontiers(pos,h_angle);

    //            int shift_angle = view_utility->getHorizontal_angle();
    //            float vel_factor = 1.;
    //            h_angle  = (h_angle+shift_angle)%360;
    //            float angle_radians = (h_angle+shift_angle)*M_PI/180.f;
    //            //        std::cout << "[NNPlanner] yaw: " << angle_degree <<std::endl;
    //            float yaw = pos(3);
    //            //        vel_traj.getEstimateYaw(yaw);
    //            if (yaw < 0) { yaw += 2 * M_PI; }

    //            float yaw_degree = yaw * 180.f/M_PI;
    //            bool front_motion = false;
    //            float max_score = 0;
    //            int start_angle = 0;
    //            for(size_t i = 0; i < view_utility->scores.size(); ++i){

    //                if(degreeDifference(yaw_degree,(float)((i+shift_angle)%360)) < 40.f){
    //                    //                std::cout << i << " " << view_utility->scores[i] << " " << yaw_degree << " " << degreeDifference(yaw_degree,((i+shift_angle)%360)) << " " << (i)%360 <<std::endl;
    //                    if(view_utility->scores[i] > 35){
    //                        front_motion = true;
    //                        if(max_score < view_utility->scores[i]){
    //                            Eigen::Vector3d pt;
    //                            max_score = view_utility->scores[i];
    //                            angle_radians  = (i+shift_angle)%360*M_PI/180.f;
    //                            angle_degree = i+shift_angle;
    //                            start_angle = (i+shift_angle)%360;
    //                        }
    //                    }
    //                }
    //            }

    //            float h =  view_utility->getBestVerticalAngle(start_angle);

    //            start_angle  = view_utility->selectNearestHorizontalFrontier(start_angle);

    //            angle_radians  = (start_angle)%360*M_PI/180.f;
    //            view_utility->drawViewUtility(Eigen::Vector2i(start_angle,h));
    //            //        if(degreeDifference(yaw_degree,(float)(((int)angle_degree)%360)) > 10)
    //            //            vel_factor = 0.5;
    //            //        std::cout << "vel factor: " << vel_factor <<std::endl;
    //            if (angle_radians < 0) { angle_radians += 2 * M_PI; }

    //            if(std::fabs(yaw-angle_radians) > M_PI){
    //                if(yaw < angle_radians){
    //                    yaw += M_PI*2;
    //                }else{
    //                    angle_radians += 2 * M_PI;
    //                }
    //            }

    //            //        std::cout << yaw << " " << angle_degree <<std::endl;

    //            Eigen::Vector4d start_pos_4d, start_vel_4d, goal_pos_4d, goal_vel_4d;
    //            start_pos_4d << pos,yaw;

    //            start_vel_4d << vel, 0.0;

    //            Eigen::Vector3d dir;
    //            Eigen::Vector3f posf;

    //            float vertical_angle = h;
    //            vertical_angle-=30;//center
    //            vertical_angle*=(1/6.);
    //            //        std::cout << "Vertical angle: " << h <<std::endl;
    //            vertical_angle=std::fmod(h,360);// mod 360
    //            //        h = 10;
    //            vertical_angle = vertical_angle*M_PI/180.;
    //            bool lock_bot = false;
    //            bool lock_up = false;
    //            if(pos.z() <=1.4){
    //                vertical_angle = 0;
    //                lock_bot = true;
    //            }
    //            if(pos.z()>1.4){
    //                lock_up = true;
    //                vertical_angle=0;
    //            }
    //            dir << std::cos(angle_radians)*std::cos(vertical_angle),
    //                    std::sin(angle_radians),
    //                    -std::sin(vertical_angle);
    //            float mag = 2.f;
    //            posf = (pos + dir*mag).cast<float>();
    //            bool found = true;
    //            int try_count =1;
    //            bool use_yaw = false;

    //            if(front_motion){
    //                view_utility->computeDistribution(start_angle,(int)h);

    //                float max_height = 1.6;
    //                //            Eigen::Vector3d h_dir(0,0,max_height - current_pose_.translation().z());
    //                //            dir += h_dir;
    //                dir.normalize();
    //                float dist;
    //                Eigen::Vector3f grad;
    //                float current_dist;
    //                searchspace_->getDistanceGrad(current_pose_.translation().cast<float>(),current_dist,grad);

    //                searchspace_->getDistanceGrad(pos.cast<float>(),dist,grad);
    //                float min_dist = 0.7;
    //                float max_dist = 1.5;
    //                if(dist < max_dist)
    //                {

    //                    float dot = vel.dot(grad.cast<double>());
    //                    //                if(dot <= 0){

    //                    //                    std::cout << pos.transpose() << " "<< dist <<std::endl;
    //                    if(current_dist <=min_dist){
    //                        std::cout << "Obstacle to close: " << current_dist << std::endl;
    //                    }
    //                    if(dist <= min_dist){
    //                        dist = min_dist+0.001;
    //                    }
    //                    float v = vel.norm();
    //                    if(v < 0.51)
    //                        v = 0.51;
    //                    float alpha_obs = (1./(dist-min_dist)-1./max_dist);
    //                    float alpha_vel = (1./(v-0.5)-1./1.5);
    //                    alpha_vel *=alpha_vel*0.5;
    //                    alpha_obs *=alpha_obs*0.5;
    //                    alpha_obs *= 0.005;
    //                    //                    alpha_vel *= 0.000005;
    //                    float influence= grad.cast<double>().dot(vel)/(v*max_dist);
    //                    influence *=influence;
    //                    Eigen::Vector3d normal = grad.cast<double>().cross(vel);
    //                    normal.normalize();
    //                    Eigen::Vector3d max_lateral_direction = vel.cross(normal).normalized();

    //                    Eigen::Vector3d target = dir;
    //                    repforce = max_lateral_direction;
    //                    dir = /*alpha_vel**/target + repforce.cast<double>()*alpha_obs;

    //                    //                std::cout << "Repulse: " << alpha_obs << " " << influence <<std::endl;
    //                    //                    if(target.dot(dir) <= 0){
    //                    //                        dir = max_lateral_direction;
    //                    //                    }
    //                    float ka = 0.5;
    //                    if(influence > 0.01)
    //                        vel_factor = fmax(fmin(sqrt(2*(dist-min_dist)*constraint_.max_a_)*ka,1.5),0.4);
    //                    use_yaw = false;
    //                    //                }
    //                }
    //                if(lock_up){
    //                    dir.z() = 1.9 - current_pose_.translation().z();
    //                    //                std::cout << "lock up" <<std::endl;
    //                }
    //                if(lock_bot){
    //                    dir.z() = 1.2 - current_pose_.translation().z();
    //                    //                std::cout << "lock bot" <<std::endl;
    //                }
    //                dir.normalize();
    //                vel_traj.startTrajectory(dir*vel_factor*1,angle_radians,use_yaw,searchspace_);
    //                std::cout << "[NNPlanner]: " << t.elapsed_ms() <<std::endl;

}

void SplatPlanner::printTimeComparison()
{
    if(times.size()==0)
        return;
    std_times = 0;
    float mean = 0;
    for(size_t i = 0; i < times.size(); ++i){
        mean += times[i];
    }
    mean /= (double)times.size();
    for(size_t i = 0; i < times.size(); ++i){
        std_times += pow(times[i] - mean, 2);
    }
    std_times /= (double)times.size();
    std_times = sqrt(std_times);


    std::cout<< "Iteration " << times.size() << " " << t.elapsed_ms()/1000. << " " << mean <<" " << std_times <<std::endl;
}
