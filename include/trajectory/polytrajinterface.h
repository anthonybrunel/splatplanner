#pragma once
#include "trajectory.h"
#include <Eigen/Core>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include "../planner/constraint.h"
#include <mutex>

#include "spline.h"

class SegmentManager4D{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SegmentManager4D(){

    }
    SegmentManager4D(const Eigen::Vector3d &start_position, mav_trajectory_generation::Segment::Vector xyz_segment, std::vector<Quintic> yaw_segments){
        segments_xyz_=xyz_segment;
        segments_yaw_=yaw_segments;
        size_t size = std::max(xyz_segment.size(),yaw_segments.size());
        start_position_ = start_position;
        //set max time, set maxtime segment
        segments_max_time_.resize(size);
        segments_xyz_time_.resize(size);
        segments_yaw_time_.resize(size);
        for(int i = 0;i < size;++i){
            segments_max_time_[i] = std::fmax(xyz_segment[i].getTime(),segments_yaw_[i].t_);
            segments_xyz_time_[i] = xyz_segment[i].getTime();
            segments_yaw_time_[i] = segments_yaw_[i].t_;
            max_time_ += segments_max_time_[i];
        }

//        std::cout << max_time_<<std::endl;
    }

    bool sampleTrajectoryAtTime(double t, mav_msgs::EigenTrajectoryPoint *p){
        if(t > max_time_)
            return false;
        Eigen::Vector4d pos = evaluate(t,0);
        Eigen::Vector4d vel = evaluate(t,1);
        Eigen::Vector4d acc = evaluate(t,2);
        p->setFromYaw(pos.w());
        p->setFromYawRate(vel.w());
        p->setFromYawAcc(acc.w());


        p->position_W = pos.head(3);
        p->velocity_W = vel.head(3);
        p->acceleration_W = acc.head(3);

        p->degrees_of_freedom = mav_msgs::MavActuation::DOF4;
        return true;
    }

    Eigen::Vector4d evaluate(double t,int der){
        Eigen::Vector4d res;
        double accumulated_time = 0;
        size_t i;
        for(i = 0; i < segments_max_time_.size();++i){
            accumulated_time += segments_max_time_[i];
            if (accumulated_time > t) {
                break;
            }

        }

        if (i >= segments_max_time_.size()) {
            i = segments_max_time_.size() - 1;
        }
        float relative_t = accumulated_time;
        relative_t -= segments_max_time_[i];
        relative_t = t - relative_t;
        if(segments_xyz_.empty()){
            res.head(3) = start_position_;
        }else{
            if(relative_t > segments_xyz_time_[i]){
                res.head(3) = segments_xyz_[i].evaluate(segments_xyz_[i].getTime()-0.0001,der).head(3);
            }else{
                res.head(3) = segments_xyz_[i].evaluate(relative_t,der).head(3);
            }
        }

        if(relative_t > segments_yaw_time_[i]){
            res(3) = segments_yaw_[i].evaluate(segments_yaw_time_[i],der);
        }else{
            res(3) = segments_yaw_[i].evaluate(relative_t,der);
        }
        return res;
    }

    void sample3D(double start,double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points){
        points.reserve((int) max_time_/dt+1);
        double t = start;
        while(t < max_time_){

            points.push_back(evaluate(t,0).head(3));
            t+=dt;
        }

    }

    double getMaxTime() const{
        return max_time_;
    }

    void reset(){
        max_time_=0;
        segments_xyz_.clear();
        segments_max_time_.clear();
        segments_yaw_.clear();
        segments_xyz_time_.clear();
        segments_yaw_time_.clear();
    }
    double max_time_=0;

    mav_trajectory_generation::Segment::Vector segments_xyz_;
    std::vector<Quintic> segments_yaw_;

    std::vector<double> segments_max_time_;
    std::vector<double> segments_xyz_time_;
    std::vector<double> segments_yaw_time_;
    Eigen::Vector3d start_position_;
};

class PolyTrajInterface : public Trajectory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PolyTrajInterface(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private, Constraint *constraint_i, std::atomic<PlannerState> *state_i, OdomData *odom_i);


    bool computeTrajectory(const Constraint& contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,
                           mav_trajectory_generation::Trajectory* trajectory);


    bool computeTrajectory(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                           const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps,
                           mav_trajectory_generation::Trajectory *trajectory);

    bool computeTrajectoryAndYaw(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                 const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps,
                                 mav_trajectory_generation::Trajectory *trajectory);


    void computeYawTraj();

    void scaleTimeTrajectory();

    void commandTimerCallback(const ros::TimerEvent&);
    void yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point);

    double angle_diff(double a1, double a2);
    bool checkTrajectory(Eigen::Vector4d request_pos);
    void publishVizualization(const mav_trajectory_generation::Trajectory &trajectory);

    void startTrajectory();
    void stopTrajectory();

    float remainingTime(){
        trajectory_mtx_.lock();

        current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
        float t = current_sample_time_ - trajectory_.getMaxTime();
        trajectory_mtx_.unlock();
        return t;

    }

    bool isRunning(){
        trajectory_mtx_.lock();
        bool tmp = publish_timer_.hasStarted();
        trajectory_mtx_.unlock();

        return tmp;

    }

    bool inputFeasability(const mav_trajectory_generation::Trajectory *trajectory)
    {
        for(const auto &segment: trajectory->segments()){
            if (feasibility_check_.checkInputFeasibility(segment) !=
                    mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                return false;
            }
        }
        return true;
    }



    bool getPredictedPose(Eigen::Vector4d & pos_o, Eigen::Vector4d & vel_o);


    void setTrajectory(const mav_trajectory_generation::Trajectory &trajectory);


    void get3DTraj(double start, double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points){
//        std::unique_lock<std::mutex>  m(trajectory_mtx_);
        trajectory_mtx_.lock();
        trajectory_4D_.sample3D(start,dt,points);
        trajectory_mtx_.unlock();
    }


    std::vector<Quintic> yaw_traj_;

    mav_trajectory_generation::Trajectory trajectory_;
    ros::Publisher pub_markers_;


    mav_trajectory_generation::Trajectory current_trajectory_;
    mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;

    bool useYaw = true;

    std::mutex trajectory_mtx_;

    std::atomic_bool setup_start;

    Eigen::Vector3d  predicted_velocity_;
    Eigen::Vector3d predicted_acc_;
    Eigen::Vector3d predicted_pos_;

    SegmentManager4D trajectory_4D_;
};
