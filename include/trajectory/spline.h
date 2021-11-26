#pragma once

#include <Eigen/Dense>
#include<Eigen/SparseQR>

#include <iostream>
class spline
{
public:
    spline();
};



class Quintic{
public:
    Eigen::Matrix<float,6,1> coeffs_;
    Eigen::Matrix<float,5,1> coeffs_d_;
    Eigen::Matrix<float,4,1> coeffs_dd_;
    Eigen::Matrix<float,3,1> coeffs_ddd_;
    float t_=0;
    Quintic(){

    }
    Quintic(Eigen::Matrix<float,6,1> coeffs , float t){

        t_ = t;
        coeffs(2) *=0.5f;

        coeffs_ = coeffs;
        if(fabs(coeffs(3)-coeffs(0))<0.0001){
            t_ = 0;
            return;
        }
        Eigen::Matrix<float,3,3> A;

        float t_sq = t*t;
        float t_th = t_sq*t;
        float t_fo = t_th*t;
        float t_fi = t_fo*t;
        A << t_th,t_fo,t_fi,
                3*t_sq,4*t_th,5*t_fo,
                6*t,12*t_sq,20*t_th;

        Eigen::Matrix<float,3,1> b;
        b<< coeffs(3)-coeffs(0)-coeffs(1)*t-coeffs(2)*t_sq,
                coeffs(4)-coeffs(1)-2*coeffs_(2)*t,
                coeffs(5) - 2*coeffs_(2);
        coeffs_.tail(3) = A.inverse() * b;

        coeffs_d_(2) = coeffs_(3) *3;
        coeffs_d_(3) = coeffs_(4) * 4;
        coeffs_d_(4) = coeffs_(5) * 5;

        coeffs_dd_(1) =coeffs_(3) * 6;
        coeffs_dd_(2) =coeffs_(4) * 12;
        coeffs_dd_(3) =coeffs_(5) * 20;

        coeffs_ddd_(0) = coeffs_dd_(1);
        coeffs_ddd_(1) =coeffs_(4)* 24;
        coeffs_ddd_(2) =coeffs_(5)* 60;
    }

    float ft(float t){

        return coeffs_(0)+coeffs_(1)*t+coeffs_(2)*t*t+coeffs_(3)*t*t*t + coeffs_(4)*t*t*t*t+coeffs_(5)*t*t*t*t*t;
    }

    float ft_d(float t){
        return coeffs_(1) + 2 * coeffs_(2) * t +
                3 * coeffs_(3) * t *t + 4 * coeffs_(4) * t *t*t + 5 * coeffs_(5) * t *t*t*t;
    }

    float ft_dd(float t){
        //        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return  2 * coeffs_(2) +
                6 * coeffs_(3) * t  + 12 * coeffs_(4) * t *t + 20 * coeffs_(5) * t *t*t;

    }

    double evaluate(double t, int der){
        if(t_ < 0.0001){
            return coeffs_(der);
        }
        switch(der){
        case 0:
            return ft(t);
        case 1:
            return ft_d(t);
        case 2:
            return ft_dd(t);
        }

        return 0;
    }

    float ft_ddd(float t){
        Eigen::Matrix<float,3,1> t_v; t_v << 1,t;
        return coeffs_ddd_.dot(t_v);
    }

    // https://github.com/ethz-asl/mav_trajectory_generation/
    //C. Richter, A. Bry, and N. Roy, “Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments,”
    //in International Journal of Robotics Research, Springer, 2016.
    void applyConstraint(double constraint_v,double constraint_a){
        if(t_ == 0){
            return;
        }
        double delta_t = 0.02;
        constexpr double kTolerance = 1e-3;
        constexpr size_t kMaxCounter = 20;

        bool within_range = false;
        for(size_t i = 0; i < kMaxCounter; ++i){
            // From Liu, Sikang, et al. "Planning Dynamically Feasible Trajectories for
            // Quadrotors Using Safe Flight Corridors in 3-D Complex Environments." IEEE
            // Robotics and Automation Letters 2.3 (2017).
            double maxv=0,maxa=0;
            for(double t = 0; t <= t_; t+=delta_t){
                double v = ft_d(t);
                double a = ft_dd(t);
                v*=v;
                a*=a;
                if(v > maxv){
                    maxv = v;
                }

                if(a > maxa){
                    maxa = a;
                }
            }
            maxv = sqrt(maxv);
            maxa = sqrt(maxa);
            double velocity_violation = maxv / constraint_v;
            double acceleration_violation = maxa / constraint_a;

            within_range = velocity_violation <= 1.0 + kTolerance &&
                    acceleration_violation <= 1.0 + kTolerance;
            if (within_range) {
                break;
            }
            double violation_scaling = std::max(
                        1.0, std::max(velocity_violation, sqrt(acceleration_violation)));
            if(violation_scaling>1.8)
                violation_scaling*=0.6;
            double violation_scaling_inverse = 1.0 / violation_scaling;
            scaleQuintic(violation_scaling_inverse);

            t_*=violation_scaling;
        }

    }

    void scaleQuintic(double scale_factor){
        double scale =1.0;
        for(int i = 0; i<coeffs_.size();++i){
            coeffs_[i] *= scale;
            scale *=scale_factor;
        }
    }

};


class Cubic{
public:
    Cubic(){

    }
    Eigen::Matrix<float,4,1> coeffs_;
    float t_ = 1;
    Cubic(float s, float vs, float g, float gs, float t){
        t_ =t;
        coeffs_(0) = s;
        coeffs_(1) = vs;
        Eigen::Matrix2f A;
        A << t*t,t*t*t,2*t,3*t*t;
        Eigen::Matrix<float,2,1> b;
        b<< gs - (s+vs*t) , gs-vs;
        coeffs_.tail(2) =  A.inverse() * b;
    }

    float ft(float t){

        return coeffs_(0)+coeffs_(1)*t+coeffs_(2)*t*t+coeffs_(3)*t*t*t;
    }

    float ft_d(float t){
        return  2 * coeffs_(2) * t +
                3 * coeffs_(3) * t *t;
    }

    float ft_dd(float t){
        //        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return   6 * coeffs_(3) * t;

    }


    void applyConstraint(double constraint_v,double constraint_a){
        if(t_ == 0){
            return;
        }
        double delta_t = 0.02;
        constexpr double kTolerance = 1e-3;
        constexpr size_t kMaxCounter = 20;

        bool within_range = false;
        for(size_t i = 0; i < kMaxCounter; ++i){
            double maxv=0,maxa=0;
            for(double t = 0; t <= t_; t+=delta_t){
                double v = ft_d(t);
                double a = ft_dd(t);
                v*=v;
                a*=a;
                if(v > maxv){
                    maxv = v;
                }

                if(a > maxa){
                    maxa = a;
                }
            }
            maxv = sqrt(maxv);
            maxa = sqrt(maxa);
            double velocity_violation = maxv / constraint_v;
            double acceleration_violation = maxa / constraint_a;

            within_range = velocity_violation <= 1.0 + kTolerance &&
                    acceleration_violation <= 1.0 + kTolerance;
            if (within_range) {
                break;
            }
            double violation_scaling = std::max(
                        1.0, std::max(velocity_violation, sqrt(acceleration_violation)));
            if(violation_scaling>1.8)
                violation_scaling*=0.6;
            double violation_scaling_inverse = 1.0 / violation_scaling;
            scale(violation_scaling_inverse);

            t_*=violation_scaling;
        }
    }

    void scale(double scale_factor){
        double scale =1.0;
        for(int i = 0; i<coeffs_.size();++i){
            coeffs_[i] *= scale;
            scale *=scale_factor;
        }
    }
};

class VelProfile{
public:

    float T_ = 1.;
    Eigen::Vector3f start_pos_ = Eigen::Vector3f::Zero();

    Quintic qx_;
    Quintic qy_;
    Quintic qz_;

    void init(const Eigen::Vector3f &vel_start , const Eigen::Vector3f &acc_start, const Eigen::Vector3f &jerk_start,
              const Eigen::Vector3f &vel_goal , const Eigen::Vector3f &acc_goal, const Eigen::Vector3f &jerk_goal,float t){

        T_ = t;
        Eigen::Matrix<float,6,1> state;

        state <<  vel_start(0),acc_start(0),jerk_start(0),vel_goal(0),acc_goal(0),jerk_goal(0);
        qx_ = Quintic(state,t);
        state <<  vel_start(1),acc_start(1),jerk_start(1),vel_goal(1),acc_goal(1),jerk_goal(1);
        qy_ = Quintic(state,t);

        state <<  vel_start(2),acc_start(2),jerk_start(2),vel_goal(2),acc_goal(2),jerk_goal(2);
        qz_ = Quintic(state,t);
    }


    void getVelAcc(float t, Eigen::Vector3f& vel, Eigen::Vector3f& acc){
        vel << qx_.ft(t),qy_.ft(t),qz_.ft(t);
        acc << qx_.ft_d(t),qy_.ft_d(t),qz_.ft_d(t);
    }

};


