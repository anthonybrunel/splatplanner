#pragma once
#include <ros/ros.h>
#include "rotors_control/lee_position_controller.h"
#include "trajectory.h"
#include "spline.h"

#include "queue"

class MotionProfile{
public:
    float T_ = 0.;
    Eigen::Vector3f start_pos_ = Eigen::Vector3f::Zero();

    Quintic qx_;
    Quintic qy_;
    Quintic qz_;

    void init(const Eigen::Vector3f &pos_start , const Eigen::Vector3f &vel_start, const Eigen::Vector3f &acc_start,
              const Eigen::Vector3f &pos_goal , const Eigen::Vector3f &vel_goal, const Eigen::Vector3f &acc_goal,float t){

        T_ = t;
        Eigen::Matrix<float,6,1> state;

        state <<  pos_start(0),vel_start(0),acc_start(0),pos_goal(0),vel_goal(0),acc_goal(0);
        qx_ = Quintic(state,t);
        state <<  pos_start(1),vel_start(1),acc_start(1),pos_goal(1),vel_goal(1),acc_goal(1);
        qy_ = Quintic(state,t);

        state <<  pos_start(2),vel_start(2),acc_start(2),pos_goal(2),vel_goal(2),acc_goal(2);
        qz_ = Quintic(state,t);
    }


    void getPosVelAcc(float t,Eigen::Vector3f& pos, Eigen::Vector3f& vel, Eigen::Vector3f& acc){
        pos << qx_.ft(t),qy_.ft(t),qz_.ft(t);
        vel << qx_.ft_d(t),qy_.ft_d(t),qz_.ft_d(t);
        acc << qx_.ft_dd(t),qy_.ft_dd(t),qz_.ft_dd(t);
    }
};

class LinearTrajectory{

public:
    LinearTrajectory(){

    }
    Eigen::Vector4f start_,end_;

    void generateMotionProfile(Eigen::Vector4f start, Eigen::Vector4f end, float max_a,float max_v, float max_yaw_rate){
        start_=start;
        end_ = end;

//        Eigen::Vector3f max_traj = (end.head(3)-start.head(3));
//        max_traj *= max_traj;
//        int idx_max;
//        max_traj.maxCoeff(&idx_max);




        {
            float tmp_max_time = 0;
            //motion 3d
            const double distance = (end_.head(3) - start_.head(3)).norm();
            float t=0;
            double acc = max_a;
            const double acc_time = max_v / acc;
            const double acc_distance = 0.5 * max_v * acc_time;
            if(distance > 0){
                if (distance < 2.0 * acc_distance) {
                    t = 2.0 * std::sqrt(distance / acc);

                    MotionProfile motion;
                    motion.init(start.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),
                                end_.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),t);
                    profile_.push_back(motion);
                    tmp_max_time = t;
                    //generate only one quintic 3d
                } else {
                    //generate only three quintic 3d
                    Eigen::Vector3f dir = (end_.head(3) - start.head(3)).normalized();
                    MotionProfile motion1;
                    motion1.init(start.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),
                                 start.head(3)+acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),acc_time);

                    profile_.push_back(motion1);
                    tmp_max_time+=acc_time;

                    MotionProfile motion2;
                    motion2.init(start.head(3)+acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),
                                 end.head(3)-acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),(distance - 2.0 * acc_distance) / max_v);

                    profile_.push_back(motion2);
                    tmp_max_time+=(distance - 2.0 * acc_distance) / max_v;


                    MotionProfile motion3;

                    motion3.init(end.head(3)-acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),
                                 end.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),acc_time);

                    profile_.push_back(motion3);

                    tmp_max_time+=acc_time;
                }
                max_time_ = tmp_max_time;
            }
        }


        {
            //yaw motion
            float tmp_max_time = 0;

            const double distance = std::abs(end_.w() - start_.w());
            float t=0;
            double acc = 2.0;
            const double acc_time = max_yaw_rate / acc;
            const double acc_distance = 0.5 * max_yaw_rate * acc_time;
            if(distance > 0){
                if (distance < 2.0 * acc_distance) {
                    t = 2.0 * std::sqrt(distance / acc);
                    //generate only one quintic
                    Eigen::Matrix<float,6,1> coeffs;
                    coeffs << start.w(),0,0,end.w(),0,0;
                    yaw_profile_.push_back(Quintic(coeffs,t));
                    tmp_max_time = t;

                } else {
                    //generate only three quintic

                    Eigen::Matrix<float,6,1> coeffs;
                    float dir = end_.w() - start_.w();
                    dir /= (abs(dir));
                    coeffs << start_.w(),max_yaw_rate,0,start_.w()+acc_distance*dir,max_yaw_rate,0;

                    yaw_profile_.push_back(Quintic(coeffs, acc_time));
                    tmp_max_time+=acc_time;
                    coeffs << start_.w()+acc_distance*dir,max_yaw_rate,0,end.w()-acc_distance*dir,max_yaw_rate,0;
                    yaw_profile_.push_back(Quintic(coeffs,(distance - 2.0 * acc_distance) / max_yaw_rate));
                    tmp_max_time+=max_yaw_rate;

                    coeffs << end.w()-acc_distance*dir,max_yaw_rate,0,end.w(),0,0;
                    yaw_profile_.push_back(Quintic(coeffs, acc_time));
                    tmp_max_time+=acc_time;
                }
            }
            max_time_ = std::max(max_time_,tmp_max_time);
        }
    }

    float max_time_ = 0;



    void replan(const Eigen::Vector4f &start,const Eigen::Vector4f &start_vel, float max_a,float max_v, float max_yaw_rate){
        start_=start;
        {
            float tmp_max_time = 0;
            //motion 3d
            const double distance = (end_.head(3) - start_.head(3)).norm();
            float t=0;
            double acc = max_a;
            const double acc_time = max_v / acc;
            const double acc_distance = 0.5 * max_v * acc_time;
            if(distance > 0){
                if (distance < 2.0 * acc_distance) {
                    t = 2.0 * std::sqrt(distance / acc);

                    MotionProfile motion;
                    motion.init(start.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),
                                end_.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),t);
                    profile_.push_back(motion);
                    tmp_max_time = t;
                    //generate only one quintic 3d
                } else {
                    //generate only three quintic 3d
                    Eigen::Vector3f dir = (end_.head(3) - start.head(3)).normalized();
                    MotionProfile motion1;
                    motion1.init(start.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),
                                 start.head(3)+acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),acc_time);

                    profile_.push_back(motion1);
                    tmp_max_time+=acc_time;

                    MotionProfile motion2;
                    motion2.init(start.head(3)+acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),
                                 end_.head(3)-acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),(distance - 2.0 * acc_distance) / max_v);

                    profile_.push_back(motion2);
                    tmp_max_time+=(distance - 2.0 * acc_distance) / max_v;


                    MotionProfile motion3;

                    motion3.init(end_.head(3)-acc_distance*dir,dir*max_v,Eigen::Vector3f::Zero(),
                                 end_.head(3),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero(),acc_time);

                    profile_.push_back(motion3);

                    tmp_max_time+=acc_time;
                }
                max_time_ = tmp_max_time;
            }
        }


        {
            //yaw motion
            float tmp_max_time = 0;

            const double distance = std::abs(end_.w() - start_.w());
            float t=0;
            double acc = 2.0;
            const double acc_time = max_yaw_rate / acc;
            const double acc_distance = 0.5 * max_yaw_rate * acc_time;
            if(distance > 0){
                if (distance < 2.0 * acc_distance) {
                    t = 2.0 * std::sqrt(distance / acc);
                    //generate only one quintic
                    Eigen::Matrix<float,6,1> coeffs;
                    coeffs << start.w(),0,0,end_.w(),0,0;
                    yaw_profile_.push_back(Quintic(coeffs,t));
                    tmp_max_time = t;

                } else {
                    //generate only three quintic

                    Eigen::Matrix<float,6,1> coeffs;
                    float dir = end_.w() - start_.w();
                    dir /= (abs(dir));
                    coeffs << start_.w(),max_yaw_rate,0,start_.w()+acc_distance*dir,max_yaw_rate,0;

                    yaw_profile_.push_back(Quintic(coeffs, acc_time));
                    tmp_max_time+=acc_time;
                    coeffs << start_.w()+acc_distance*dir,max_yaw_rate,0,end_.w()-acc_distance*dir,max_yaw_rate,0;
                    yaw_profile_.push_back(Quintic(coeffs,(distance - 2.0 * acc_distance) / max_yaw_rate));
                    tmp_max_time+=max_yaw_rate;

                    coeffs << end_.w()-acc_distance*dir,max_yaw_rate,0,end_.w(),0,0;
                    yaw_profile_.push_back(Quintic(coeffs, acc_time));
                    tmp_max_time+=acc_time;
                }
            }
            max_time_ = std::max(max_time_,tmp_max_time);
        }

    }

    bool getTrajectoryPoint(float t,mav_msgs::EigenTrajectoryPoint& point){
        if(t >= max_time_){
            point.position_W = end_.head(3).cast<double>();
            point.velocity_W.setZero();
            point.acceleration_W.setZero();
            float yaw,yaw_rate,yaw_acc;
            yaw = end_.w();
            yaw_rate = 0;
            yaw_acc = 0;
            point.setFromYaw(std::fmod(yaw, M_PI*2.));
            if(point.getYaw() > M_PI){
                point.setFromYaw(point.getYaw()-M_PI*2.);
            }
            point.setFromYawRate(yaw_rate);
            point.setFromYawAcc(yaw_acc);

            return false;
        }
        float tmp_t = 0;
        for(size_t i = 0; i< profile_.size();++i){
            if(profile_[i].T_>t-tmp_t){
                Eigen::Vector3f p,v,a;
                profile_[i].getPosVelAcc(t-tmp_t,p,v,a);
                point.position_W = p.cast<double>();
                point.velocity_W = v.cast<double>();
                point.acceleration_W = a.cast<double>();
                break;
            }else{
                tmp_t += profile_[i].T_;
                if(i == profile_.size()-1){
                    Eigen::Vector3f p,v,a;
                    profile_[i].getPosVelAcc(profile_[i].T_,p,v,a);
                    point.position_W = p.cast<double>();
                    point.velocity_W = v.cast<double>();
                    point.acceleration_W = a.cast<double>();

                }
            }
        }

        tmp_t = 0;

        for(size_t i = 0; i< yaw_profile_.size();++i){
            if(yaw_profile_[i].t_>t-tmp_t){
                float yaw,yaw_rate,yaw_acc;
                yaw = yaw_profile_[i].ft(t-tmp_t);
                yaw_rate = yaw_profile_[i].ft_d(t-tmp_t);
                yaw_acc = yaw_profile_[i].ft_dd(t-tmp_t);
                point.setFromYaw(yaw);
                point.setFromYawRate(yaw_rate);
                point.setFromYawAcc(yaw_acc);
                break;
            }else{
                tmp_t += yaw_profile_[i].t_;
                if(i == profile_.size()-1){

                    float yaw,yaw_rate,yaw_acc;
                    yaw = yaw_profile_[i].ft(yaw_profile_[i].t_);
                    yaw_rate = yaw_profile_[i].ft_d(yaw_profile_[i].t_);
                    yaw_acc = yaw_profile_[i].ft_dd(yaw_profile_[i].t_);
                    point.setFromYaw(std::fmod(yaw, M_PI*2.));
                    if(point.getYaw() > M_PI){
                        point.setFromYaw(point.getYaw()-M_PI*2.);
                    }
                    point.setFromYawRate(yaw_rate);
                    point.setFromYawAcc(yaw_acc);
                }

            }
        }
        return true;
    }


    std::vector<MotionProfile> profile_;
    std::vector<Quintic> yaw_profile_;
};

class LeeControllerTrajectory: public Trajectory
{
public:
    LeeControllerTrajectory(const ros::NodeHandle &nh_i, const ros::NodeHandle &nh_private_i, Constraint *constraint_i, std::atomic<PlannerState> *state_i, OdomData *odom_i);



    void computeTrajectory(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                           const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,
                           const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps = std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>());


    void execute(const nav_msgs::OdometryConstPtr &odom_i);


    void predictPosVel(Eigen::Vector4d &pos, Eigen::Vector4d &vel);
    rotors_control::LeePositionController lee_position_controller_;
    ros::Publisher motor_velocity_reference_pub_;



    std::mutex trajectory_mtx_;
    mav_msgs::EigenTrajectoryPoint command_trajectory_;

    bool isInit = false;

    std::queue<LinearTrajectory> trajectories_;

};

