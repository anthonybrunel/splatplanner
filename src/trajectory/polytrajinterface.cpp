#include "../../include/trajectory/polytrajinterface.h"
#include <quadrotor_msgs/TrajectoryPoint.h>
PolyTrajInterface::PolyTrajInterface(const ros::NodeHandle &nh,
                                     const ros::NodeHandle& nh_private, Constraint *constraint_i,
                                     std::atomic<PlannerState> *state_i, OdomData * odom_i):
    Trajectory (nh,nh_private,constraint_i,state_i,odom_i), setup_start(false)
{
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                     &PolyTrajInterface::commandTimerCallback,
                                     this, false, false);

    std::cout << "[PolyTrajInterface] Initialized" <<std::endl;

    pub_markers_ =
            nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1000);



}


bool PolyTrajInterface::computeTrajectory(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                          const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,


                                          mav_trajectory_generation::Trajectory *trajectory)
{


    const int dimension = goal_pos.size();
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    std::vector<Eigen::Vector4d> pts;
    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,start_pos);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    //    for(size_t i = 1; i < pts.size()-1; ++i){

    //        Eigen::Vector4d &next_pts = pts[i];
    //        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,next_pts);
    ////        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
    ////                          Eigen::Vector4d(0,0,0.3,0));


    //        vertices.push_back(p);

    //    }
    //    Eigen::Vector4d half_path;half_path << (goal_pos.x() - start_pos_4d.x())/2.,(goal_pos.y() - start_pos_4d.y())/2.,(goal_pos.z() - start_pos_4d.z())/2.,M_PI/2.;
    //    p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,half_path);
    //    vertices.push_back(p);


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector4d(0,0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, 2, contraint_i.max_a_);
    for(size_t i =0; i < segment_times.size(); ++i){
        if(segment_times[i] < 0.001)
            segment_times[i] = 0.1;
    }

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, contraint_i.max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, contraint_i.max_a_);

    // solve trajectory
    opt.optimize();

    //    trajectory->scaleSegmentTimes(
    //    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    //    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    //    opt.solveLinear();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    //    trajectory->scaleSegmentTimesToMeetConstraints(contraint_i.max_v_, contraint_i.max_a_);

    return true;

}


bool PolyTrajInterface::computeTrajectory(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                          const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> &wps,


                                          mav_trajectory_generation::Trajectory *trajectory)
{

    //    std::cout << "New Trajectory with waypoints" << start_pos.transpose() << " "<< start_vel.transpose() << " " << goal_pos .transpose()<< " " << goal_vel.transpose() <<std::endl;

    const int dimension = goal_pos.size();
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,start_pos);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    for(size_t i = 0; i < wps.size(); ++i){
        const Eigen::Vector4d &next_pts = wps[i];
        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,next_pts);
        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        Eigen::Vector4d::Zero());
        p.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                        Eigen::Vector4d(0,0,0,0));
        vertices.push_back(p);
    }


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector4d(0,0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, contraint_i.max_v_, contraint_i.max_a_);

    for(size_t i =0; i < segment_times.size(); ++i){
        if(segment_times[i] < 0.001)
            segment_times[i] = 0.1;
    }

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 2000.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, contraint_i.max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, contraint_i.max_a_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, 20.);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, 20.);

    // solve trajectory
    opt.optimize();

    //    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    //    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    //    opt.solveLinear();
    opt.getTrajectory(&(*trajectory));
    //    trajectory->scaleSegmentTimesToMeetConstraints(contraint_i.max_v_, contraint_i.max_a_);

    return true;

}

bool PolyTrajInterface::computeTrajectoryAndYaw(const Constraint &contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps, mav_trajectory_generation::Trajectory *trajectory)
{
    //    std::cout << "New Trajectory 3D with waypoints" << start_pos.transpose() << " "<< start_vel.transpose() << " " << goal_pos .transpose()<< " " << goal_vel.transpose() <<std::endl;

    mav_trajectory_generation::Segment::Vector segment_xyz;

    //    if((start_pos.head(3)-goal_pos.head(3)).squaredNorm()> 0.0001)
    //    {
    const int dimension = 3;
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(start_pos.head<3>()));
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel.head(3));
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    for(size_t i = 0; i < wps.size(); ++i){
        const Eigen::Vector4d &next_pts = wps[i];
        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(next_pts.head<3>()));
        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        Eigen::Vector3d::Zero());
        p.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                        Eigen::Vector3d(0,0,0));
        vertices.push_back(p);
    }


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos.head<3>());
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel.head(3));
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector3d(0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, contraint_i.max_v_, contraint_i.max_a_);

    for(size_t i =0; i < segment_times.size(); ++i){
        if(segment_times[i] < 0.001)
            segment_times[i] = 0.1;
    }

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 2000.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, contraint_i.max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, contraint_i.max_a_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, 20.);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, 20.);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(contraint_i.max_v_, contraint_i.max_a_);



    trajectory->getSegments(&segment_xyz);
    //    }

    std::vector<Quintic> segments_yaw;
    float prev_yaw = start_pos.w();
    float prev_vel_yaw = start_vel.w();
    double prev_time = 0;
    if(wps.size() > 0){
        for(size_t i = 0; i < wps.size(); ++i){
            const float next_yaw = wps[i].w();
            Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,next_yaw,0,0;
            double t = fabs(prev_yaw-next_yaw)/constraint_->max_yaw_vel_;
            Quintic q(state,t);
            //applyconstraint
            q.applyConstraint(constraint_->max_yaw_vel_,constraint_->max_yaw_acc_);
            segments_yaw.push_back(q);

            prev_yaw = next_yaw;

            prev_vel_yaw = 0;

        }
    }

    Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,goal_pos.w(),0,0;
    Quintic q(state,fabs(prev_yaw-goal_pos.w())/constraint_->max_yaw_vel_);
    //apply constraint
    q.applyConstraint(constraint_->max_yaw_vel_,constraint_->max_yaw_acc_);
    segments_yaw.push_back(q);
    //    std::cout << goal_pos.w() <<std::endl;

    //    std::cout << q.t_ <<std::endl;
    trajectory_4D_ = SegmentManager4D(start_pos.head(3),segment_xyz,segments_yaw);
    return true;

}

void PolyTrajInterface::scaleTimeTrajectory()
{
    //    double new_max_time = 0.0;
    //    double scaling_inverse = 1.0 / scaling;

    //    Segments segment
    //    trajectory_.getSegments(segments)
    //    for (size_t i = 0; i < segments_.size(); i++) {
    //      double new_time = segments_[i].getTime() * scaling;
    //      for (int d = 0; d < segments_[i].D(); d++) {
    //        (segments_[i])[d].scalePolynomialInTime(scaling_inverse);
    //      }
    //      segments_[i].setTime(new_time);
    //      new_max_time += new_time;
    //    }
    //    max_time_ = new_max_time;
}



void PolyTrajInterface::commandTimerCallback(const ros::TimerEvent &)
{
    //    trajectory_mtx_.lock();
    //    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    //    if (current_sample_time_ <= trajectory_.getMaxTime()) {
    //        trajectory_msgs::MultiDOFJointTrajectory msg;
    //        mav_msgs::EigenTrajectoryPoint trajectory_point;
    //        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
    //                    trajectory_, current_sample_time_, &trajectory_point);
    //        if (!success) {
    //            publish_timer_.stop();
    //        }

    //        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    //        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    //        command_pub_.publish(msg);
    //    } else if(current_sample_time_ <= trajectory_.getMaxTime() + 0.5){//time shift to go at goal
    //        current_sample_time_ = trajectory_.getMaxTime();
    //        trajectory_msgs::MultiDOFJointTrajectory msg;
    //        mav_msgs::EigenTrajectoryPoint trajectory_point;
    //        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
    //                    trajectory_, current_sample_time_, &trajectory_point);
    //        if (!success) {
    //            publish_timer_.stop();
    //        }
    //        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    //        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    //    }else {
    //        publish_timer_.stop();
    //        //        trajectory_.clear();
    //        (*state_) = PlannerState::IDLE;
    //    }
    //    trajectory_mtx_.unlock();



    if(setup_start){
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        setup_start = false;
    }
    trajectory_mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();

    if (current_sample_time_ <= trajectory_4D_.getMaxTime()) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = trajectory_4D_.sampleTrajectoryAtTime(
                    current_sample_time_, &trajectory_point);
        trajectory_mtx_.unlock();
//        std::cout << ros::Duration(ros::Time::now()-odom_->getTime()) .toSec() << std::endl;
        if((trajectory_point.position_W-odom_->current_pose().translation()).norm()>0.49 && odom_->current_velocity().norm() < 0.02){
            std::cout << "[Planner]: Error current pose too far from trajectory point\n" <<std::endl;
            stopTrajectory();
            return;
        }
        if (!success) {
            std::cout << "[Planner]: trajectory failed\n" <<std::endl;
            stopTrajectory();
            return;
        }

        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        //        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
        quadrotor_msgs::TrajectoryPoint quad_msg;

        quad_msg.pose.position.x = trajectory_point.position_W.x();
        quad_msg.pose.position.y = trajectory_point.position_W.y();
        quad_msg.pose.position.z = trajectory_point.position_W.z();

        quad_msg.velocity.linear.x = trajectory_point.velocity_W.x();
        quad_msg.velocity.linear.y = trajectory_point.velocity_W.y();
        quad_msg.velocity.linear.z = trajectory_point.velocity_W.z();

        quad_msg.acceleration.linear.x = trajectory_point.acceleration_W.x();
        quad_msg.acceleration.linear.y = trajectory_point.acceleration_W.y();
        quad_msg.acceleration.linear.z = trajectory_point.acceleration_W.z();


        quad_msg.jerk.linear.x = trajectory_point.jerk_W.x();
        quad_msg.jerk.linear.y = trajectory_point.jerk_W.y();
        quad_msg.jerk.linear.z = trajectory_point.jerk_W.z();

        quad_msg.snap.linear.x = trajectory_point.snap_W.x();
        quad_msg.snap.linear.y = trajectory_point.snap_W.y();
        quad_msg.snap.linear.z = trajectory_point.snap_W.z();


        quad_msg.heading = trajectory_point.getYaw();
        quad_msg.heading_rate = trajectory_point.getYawRate();
        quad_msg.heading_acceleration = trajectory_point.getYawAcc();
        command_pub_.publish(quad_msg);
    } else if(current_sample_time_ <= trajectory_4D_.getMaxTime()){//time shift to go at goal
        current_sample_time_ = trajectory_4D_.getMaxTime();
        trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = trajectory_4D_.sampleTrajectoryAtTime(
                    current_sample_time_, &trajectory_point);
        trajectory_mtx_.unlock();

        if((trajectory_point.position_W-odom_->current_pose().translation()).norm()>0.49 && odom_->current_velocity().norm() < 0.02){
            std::cout << "[Planner]: Error current pose too far from trajectory point\n" <<std::endl;
            stopTrajectory();
            return;
        }
        if (!success) {
            std::cout << "[Planner]: trajectory failed\n" <<std::endl;
            stopTrajectory();
            return;
        }
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        quadrotor_msgs::TrajectoryPoint quad_msg;

        quad_msg.pose.position.x = trajectory_point.position_W.x();
        quad_msg.pose.position.y = trajectory_point.position_W.y();
        quad_msg.pose.position.z = trajectory_point.position_W.z();

        quad_msg.velocity.linear.x = trajectory_point.velocity_W.x();
        quad_msg.velocity.linear.y = trajectory_point.velocity_W.y();
        quad_msg.velocity.linear.z = trajectory_point.velocity_W.z();

        quad_msg.acceleration.linear.x = trajectory_point.acceleration_W.x();
        quad_msg.acceleration.linear.y = trajectory_point.acceleration_W.y();
        quad_msg.acceleration.linear.z = trajectory_point.acceleration_W.z();


        quad_msg.jerk.linear.x = trajectory_point.jerk_W.x();
        quad_msg.jerk.linear.y = trajectory_point.jerk_W.y();
        quad_msg.jerk.linear.z = trajectory_point.jerk_W.z();

        quad_msg.snap.linear.x = trajectory_point.snap_W.x();
        quad_msg.snap.linear.y = trajectory_point.snap_W.y();
        quad_msg.snap.linear.z = trajectory_point.snap_W.z();


        quad_msg.heading = trajectory_point.getYaw();
        quad_msg.heading_rate = trajectory_point.getYawRate();
        quad_msg.heading_acceleration = trajectory_point.getYawAcc();
        command_pub_.publish(quad_msg);
        //        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    }else {
        trajectory_mtx_.unlock();

        stopTrajectory();
    }
}

void PolyTrajInterface::yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point)
{
    if(useYaw & trajectory_point.velocity_W.norm() > 0)
        return;

    float goal_yaw =std::atan2(trajectory_point.velocity_W.x(),
                               -trajectory_point.velocity_W.y()) - M_PI/2.;

    if (goal_yaw > M_PI)        { goal_yaw -= 2 * M_PI; }
    else if (goal_yaw <= -M_PI) { goal_yaw += 2 * M_PI; }

    trajectory_point.setFromYaw(goal_yaw);
    trajectory_point.timestamp_ns = ros::Time::now().toNSec();

}

bool PolyTrajInterface::checkTrajectory(Eigen::Vector4d request_pos)
{
    Eigen::Vector3d pos_o = odom_->current_pose_.translation();
    double diff_p = (pos_o-request_pos.head(3)).norm();

    return diff_p <0.49;
}

void PolyTrajInterface::startTrajectory()
{
    //    if(trajectory_.getMaxTime() <= 0)
    //        return;
    setup_start = true;
    publish_timer_.start();

}

void PolyTrajInterface::stopTrajectory()
{
    current_sample_time_ = trajectory_4D_.getMaxTime()+10;

    publish_timer_.stop();

    //        trajectory_.clear();
    (*state_) = PlannerState::IDLE;
}

bool PolyTrajInterface::getPredictedPose(Eigen::Vector4d &pos_o, Eigen::Vector4d &vel_o)
{
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();

    std::unique_lock<std::mutex>  m(trajectory_mtx_);
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = false;
    if(current_sample_time_ >= trajectory_4D_.getMaxTime()){
        success = trajectory_4D_.sampleTrajectoryAtTime(trajectory_4D_.getMaxTime()-0.01, &trajectory_point);

    }else if(trajectory_4D_.getMaxTime()>0){
        success = trajectory_4D_.sampleTrajectoryAtTime(current_sample_time_, &trajectory_point);

    }

    if(success){
        pos_o << trajectory_point.position_W,trajectory_point.getYaw();
        vel_o << trajectory_point.velocity_W,trajectory_point.getYawRate();

    }

    if(!checkTrajectory(pos_o)){
        pos_o.head(3) = odom_->current_pose_.translation();
        Eigen::Matrix3d R = odom_->current_pose().rotation();
        vel_o.head(3) = R*odom_->current_velocity_;
        pos_o(3) = mav_msgs::yawFromQuaternion(
                    (Eigen::Quaterniond)odom_->current_pose_.rotation());

    }
    return success;

}

void PolyTrajInterface::setTrajectory(const mav_trajectory_generation::Trajectory &trajectory)
{
    trajectory_mtx_.lock();
    publish_timer_.stop();
    trajectory_.clear();
    trajectory_ = trajectory;
    trajectory_mtx_.unlock();

}




void PolyTrajInterface::publishVizualization(const mav_trajectory_generation::Trajectory &trajectory)
{
    if(trajectory.empty())
        return;
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance =
            0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";


    mav_trajectory_generation::drawMavTrajectory(trajectory,
                                                 distance,
                                                 frame_id,
                                                 &markers);
    pub_markers_.publish(markers);
}
