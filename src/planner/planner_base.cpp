#include "../../include/planner/planner_base.h"

#include <visualization_msgs/Marker.h>
#include <random>
PlannerBase::PlannerBase(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private_i):
    nh_(nh),
    nh_private_(nh_private_i),
    trajInterface_(nh,nh_private_i,&constraint_,&state_,&odom_data_)
//    leeTrajInterface(nh,nh_private_i,&constraint_,&state_,&odom_data_)
{

    constraint_.max_v_ = (1.5);
    constraint_.max_a_ = (2.5);
    constraint_.max_yaw_vel_ = (M_PI / 2.0);
    constraint_.max_yaw_acc_ = (M_PI / 2.0);

    nh_private_.param("max_v", constraint_.max_v_, constraint_.max_v_);
    nh_private_.param("max_a", constraint_.max_a_, constraint_.max_a_);
    nh_private_.param("max_yaw_vel", constraint_.max_yaw_vel_, constraint_.max_yaw_vel_);
    nh_private_.param("max_yaw_acc", constraint_.max_yaw_acc_, constraint_.max_yaw_acc_);
    nh_private_.param("safety_radius", constraint_.margin_, constraint_.margin_);

    nh_private_.param("planner/save_log_folder", planner_logging_.save_path_, planner_logging_.save_path_);

    nh_private_.param("useLeeController", useLeeController_, false);

    std::cout << "[PlannerBase] Vitesse planner: " << constraint_.max_v_ << std::endl;


    // subscriber for Odometry
    sub_odom_ =
            nh_private_.subscribe<nav_msgs::Odometry>("uav_odom", 1, &PlannerBase::uavOdomCallback, this);

    sub_depth_odom_ =
            nh_private_.subscribe<nav_msgs::Odometry>("depth_odom", 1, &PlannerBase::depthOdomCallback, this);


    visu_vector_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planner/grad_arrow", 10);


    searchspace_.reset(new MapSearchSpace(nh_,nh_private_));
    state_= PlannerState::INIT;
    exploreState_ = ExplorationState::YAWTURN;
    explore_thread_ = new std::thread(&PlannerBase::run_exploration,this);



    execute_timer_ = nh_.createTimer(ros::Duration(0.02),
                                     &PlannerBase::execute,
                                     this);
//    collisionChecker_.reset(new CollisionChecker(searchspace_));
    std::cout << "[PlannerBase] Initialized" <<std::endl;
}

PlannerBase::~PlannerBase()
{
    state_ = PlannerState::STOP;
    std::cout << "Saving mapping volume and planner data\n" << std::endl;
    searchspace_->logExploration();
    planner_logging_.saveData(planner_logging_.save_path_);
    if(explore_thread_){
        if(explore_thread_->joinable()){
            explore_thread_->join();
        }
        delete explore_thread_;

    }

    //    std::cout << "Exploraing time: " << t.elapsed_ms() << std::endl;


    //crash occure here: terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    //what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument


}

void PlannerBase::execute(const ros::TimerEvent &)
{

    switch(state_){
    case INIT:
//                        ROS_INFO("[Planner] State: INIT"); // takeoff is done here
        waitUntilReady();
        break;
    case TAKEOFF:

        ROS_INFO("[Planner] State: TAKEOFF");
//        if(odom_data_.current_pose().translation().z()>1. && odom_data_.current_velocity().norm() < 0.1)
            state_ = PlannerState::IDLE;//assume that the planner has take off

        break;
    case TRAJ:
        //                ROS_INFO("[Planner] State: TRAJ");
        //        publish_grad_esdf();
        publish_visualization();
        break;

    case IDLE:
        //                ROS_INFO("[Planner] State: IDLE");
        //        publish_grad_esdf();
        publish_visualization();
        break;

    case STOP:
        ros::shutdown();
        break;
    default:
        //        ROS_INFO("[Planner] State: NOT HANDLED");

        break;
    }
}

bool PlannerBase::waitUntilReady()
{
    if(mavOdomReady){
        std::cout << "[Planner]: take off" << std::endl;
        state_ = PlannerState::TAKEOFF;
//        Eigen::Vector4d start_pos_4d, start_vel_4d;
//        Eigen::Affine3d pose = odom_data_.current_pose();
//        Eigen::Vector3d vel = odom_data_.current_velocity();
//        start_pos_4d << pose.translation(),0;
//        //                mav_msgs::yawFromQuaternion(
//        //                    (Eigen::Quaterniond)pose.rotation());
//        start_vel_4d << vel, 0.0;
//        addGoals(start_pos_4d, start_vel_4d, Eigen::Vector4d(pose.translation().x(),pose.translation().y(),1.35,0),Eigen::Vector4d(0,0,0,0));
        t.restart();
    }

    return true;
}

void PlannerBase::uavOdomCallback(const nav_msgs::OdometryConstPtr &odom_i)
{

    if(!depthOdomReady)
        return;
    Eigen::Affine3d pose;
    Eigen::Vector3d vel;
    Eigen::Vector3d ang_vel;

    tf::poseMsgToEigen(odom_i->pose.pose, pose);
    tf::vectorMsgToEigen(odom_i->twist.twist.linear, vel);
    tf::vectorMsgToEigen(odom_i->twist.twist.angular, ang_vel);
    odom_data_.setCurrent_pose(pose);
    odom_data_.setCurrent_velocity(vel);
    odom_data_.setCurrent_angular_velocity(ang_vel);

    odom_data_.setTime(odom_i->header.stamp);
//    if(useLeeController_)
//        leeTrajInterface.execute(odom_i);
    mavOdomReady = true;
//        std::cout << "[Planner] Current velocity :" << odom_data_.current_velocity().norm() <<std::endl;

    float speed = odom_data_.current_velocity().norm();
    planner_logging_.log_dist_speed(speed,odom_data_.current_pose().translation());
    //    std::cout << "Current pose :" << odom_data_.current_pose().translation().transpose() <<std::endl;
}

void PlannerBase::depthOdomCallback(const nav_msgs::OdometryConstPtr &odom_i)
{
    depthOdomReady = true;
}



void PlannerBase::addGoals(const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                           const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel)
{

    if(!useLeeController_){
        mav_trajectory_generation::Trajectory trajectory;


        Timer t;
        trajInterface_.computeTrajectoryAndYaw(constraint_,start_pos, start_vel, goal_pos, goal_vel,
                                              std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(),
                                              &trajectory);
//        trajInterface_.computeTrajectory(constraint_, start_pos, start_vel, goal_pos, goal_vel, &trajectory);
        //    std::cout << t.elapsed_micro() <<std::endl;
        trajInterface_.setTrajectory(trajectory);
        trajInterface_.startTrajectory();
    }else{
//        leeTrajInterface.computeTrajectory(constraint_, start_pos, start_vel, goal_pos, goal_vel);
    }
}

void PlannerBase::publish_grad_esdf()
{
    float dist;
    Eigen::Vector3f dir;
    searchspace_->getDistanceGrad(odom_data_.current_pose_.translation().cast<float>(),dist,dir);
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "grad_dist";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.pose.position.x = odom_data_.current_pose_.translation().x();
    marker.pose.position.y = odom_data_.current_pose_.translation().y();
    marker.pose.position.z = odom_data_.current_pose_.translation().z();


    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;


    marker.points.resize(2);
    marker.points[0].x = 0;
    marker.points[0].y = 0;
    marker.points[0].z = 0;
    marker.points[1].x = repforce.x();
    marker.points[1].y = repforce.y();
    marker.points[1].z = repforce.z();

    marker.color.r = 0.0f;
    marker.color.g = 0.6f;
    marker.color.b = 0.6f;
    marker.color.a = 1.f;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    visu_vector_pub_.publish(marker);
}






void PlannerBase::run_exploration()
{
    //    addGoals(Eigen::Vector4d(2.,2.,1.5,0.3),Eigen::Vector4d(1.,1.,0.,0).normalized());
    //tempo loop until the drone has takeoff
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>(4000)));

    Timer t1;
    Timer tstart;
    double elapsed_time;
    double rate = 20;//[50hz]
    while(state_ != IDLE){
        elapsed_time = t1.elapsed_ms() ;
        while (elapsed_time < rate && state_ != IDLE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>(rate-elapsed_time)));
            elapsed_time = t1.elapsed_ms() ;

        }
        t1.restart();
    }

    std::cout << "[Planner] Starting exploration behavior\n";

    rate =20;//50hz
    while(state_ != STOP && nh_.ok()){
        t1.restart();
        switch(state_){
        case IDLE:
            //            std::cout << "Ready" << std::endl;
            break;
        case TRAJ:
            //            std::cout << "Ready but executing traj" << std::endl;
            break;

        default:
//            std::cout << "NOT READY" << std::endl;
            break;
        }

        exploration_behavior();

        elapsed_time = t1.elapsed_ms() ;
        while (elapsed_time < rate && state_ != STOP) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>(rate-elapsed_time)));
            elapsed_time = t1.elapsed_ms() ;

        }



    }

    std::cout << "Explorer thread finish properly" << std::endl;

}

void PlannerBase::exploration_behavior()
{
    //do random thing : )
    if(state_ != IDLE)
        return;


}

void PlannerBase::publish_visualization()
{

}

visualization_msgs::Marker PlannerBase::drawCamera(const Eigen::Vector3d pos, const Eigen::Vector3d &dir, int id)
{
//    nearHeight_ = 2. * tan(fovRadians_/ 2.) * near_;
//    farHeight_ = 2. * tan(fovRadians_ / 2.) * far_;
//    nearWidth_ = nearHeight_ * ratio;
//    farWidth_ = farHeight_ * ratio;

    auto eigen_to_pt = [](const Eigen::Vector3d & point){
        geometry_msgs::Point pt;
        pt.x =point.x();
        pt.y =point.y();
        pt.z =point.z();
        return pt;
    };

    double far = 1;
    double fovRadians = 65.*M_PI/180.;
    double ratio = 640./480.;
    double farHeight_ = 2*tan(fovRadians*0.5)*far;

    double farWidth_ = farHeight_*ratio;


    visualization_msgs::Marker mk_cams;

    mk_cams.header.frame_id = "world";
    mk_cams.header.stamp    = ros::Time::now();
    mk_cams.ns = "cams";
    mk_cams.type            = visualization_msgs::Marker::LINE_LIST;
    mk_cams.id              = id ;


    mk_cams.action             = visualization_msgs::Marker::ADD;
    mk_cams.pose.orientation.x = 0.0;
    mk_cams.pose.orientation.y = 0.0;
    mk_cams.pose.orientation.z = 0.0;
    mk_cams.pose.orientation.w = 1.0;

    mk_cams.color.r = 0.;
    mk_cams.color.g = 0.;
    mk_cams.color.b = 1.;


    mk_cams.color.a = 1;
    mk_cams.scale.x = 0.05;

    mk_cams.lifetime = ros::Duration(1.);

    Eigen::Vector3d nearCenter = pos;
    Eigen::Vector3d farCenter = pos+dir*far;
    Eigen::Vector3d left = -dir.cross(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d topleft = farCenter + Eigen::Vector3d::UnitZ() * (farHeight_*0.5f) + left * (farWidth_*0.5f);
    Eigen::Vector3d topright = farCenter + Eigen::Vector3d::UnitZ()  * (farHeight_*0.5f) - left * (farWidth_*0.5f);
    Eigen::Vector3d bottomleft = farCenter - Eigen::Vector3d::UnitZ()  * (farHeight_*0.5f) + left * (farWidth_*0.5f);
    Eigen::Vector3d bottomright = farCenter - Eigen::Vector3d::UnitZ()  * (farHeight_*0.5f) - left * (farWidth_*0.5f);



    mk_cams.points.push_back(eigen_to_pt(topleft));
    mk_cams.points.push_back(eigen_to_pt(topright));

    mk_cams.points.push_back(eigen_to_pt(topleft));
    mk_cams.points.push_back(eigen_to_pt(bottomleft));

    mk_cams.points.push_back(eigen_to_pt(topright));
    mk_cams.points.push_back(eigen_to_pt(bottomright));


    mk_cams.points.push_back(eigen_to_pt(bottomright));
    mk_cams.points.push_back(eigen_to_pt(bottomleft));


    mk_cams.points.push_back(eigen_to_pt(pos));
    mk_cams.points.push_back(eigen_to_pt(bottomleft));

    mk_cams.points.push_back(eigen_to_pt(pos));
    mk_cams.points.push_back(eigen_to_pt(bottomright));

    mk_cams.points.push_back(eigen_to_pt(pos));
    mk_cams.points.push_back(eigen_to_pt(topleft));

    mk_cams.points.push_back(eigen_to_pt(pos));
    mk_cams.points.push_back(eigen_to_pt(topright));

    return mk_cams;

}

bool PlannerBase::validate_path_polytraj()
{

    if(trajInterface_.publish_timer_.hasStarted()){

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> states;
        double start = trajInterface_.current_sample_time_;
        trajInterface_.get3DTraj(start,0.02,states);

        float dist;
        for(size_t i = 0; i < states.size();++i){
            Eigen::Vector3f pos = states[i].cast<float>();
            searchspace_->getDistance(pos,dist);
            if(dist < 0.1){
                return false;
            }

        }

    }
    return true;
}





