#pragma once
#include <Eigen/Geometry>
#include <iostream>




class BezierCurve2D{
public:
    BezierCurve2D(){
        switch(degree){
        case 3:
            dcoeff_ = const_d3_;
            ddcoeff_ = const_dd3_;
            break;
        case 4:
            dcoeff_ = const_d4_;
            ddcoeff_ = const_dd4_;

            break;
        default:
            std::cout << "Degree curve not define" <<std::endl;
            break;
        }
    }

    Eigen::Vector2d computePts(float t){
        Eigen::VectorXd tv(ctrlPts_.rows());
        tv[0] = 1;
        for(size_t i = 1; i < ctrlPts_.rows();++i){
            tv[i] = tv[i-1]*t;
        }
        return Eigen::Vector2d(tv.transpose()*coefficient_*ctrlPts_.col(0),
                               tv.transpose()*coefficient_*ctrlPts_.col(1));

    }

    void compute_t_vec(Eigen::VectorXd &tv, float t, int order){
        tv[0] = 1;
        for(size_t i = 1; i < degree+1-order; ++i){
            tv[i] = t*tv[i-1];
        }
    }

    void t_dt(Eigen::VectorXd & dt_i, float t){
        compute_t_vec(dt_i,t,1);
        for(size_t i = 1; i<dt_i.rows();++i){
            dt_i[i] *= dcoeff_[i];
        }
    }
    void t_ddt(Eigen::VectorXd & ddt_i, float t){
        compute_t_vec(ddt_i,t,2);
        ddt_i[0] *= ddcoeff_[0];

        for(size_t i = 1; i < ddt_i.rows();++i){
            ddt_i[i] *= ddcoeff_[i];
        }
    }

    Eigen::Vector2d b_dt(float t){

        Eigen::VectorXd dt(ctrlPts_.rows()-1);
        t_dt(dt,t);
        return Eigen::Vector2d(dt.transpose()*coefficient_.block(1,0,dt.rows(),ctrlPts_.rows())*ctrlPts_.col(0),
                               dt.transpose()*coefficient_.block(1,0,dt.rows(),ctrlPts_.rows())*ctrlPts_.col(1));

    }
    Eigen::Vector2d b_ddt(float t){
        Eigen::VectorXd ddt(ctrlPts_.rows()-2);
        t_ddt(ddt,t);
        std::cout << ddt<< std::endl;
        std::cout << coefficient_.block(2,0,ddt.rows(),ctrlPts_.rows())*ctrlPts_.col(0) <<std::endl;
        std::cout << coefficient_.block(2,0,ddt.rows(),ctrlPts_.rows())  <<std::endl;
//        std::cout << 6*(1-t)*(ctrlPts_(0,0)-2*ctrlPts_(1,0)+ctrlPts_(2,0)) + 6*(ctrlPts_(0,0)-2*ctrlPts_(1,0)+ctrlPts_(2,0))
//                  <<std::endl;

        return Eigen::Vector2d(ddt.transpose()*coefficient_.block(2,0,ddt.rows(),ctrlPts_.rows())*ctrlPts_.col(0),
                               ddt.transpose()*coefficient_.block(2,0,ddt.rows(),ctrlPts_.rows())*ctrlPts_.col(1));

    }

    float compute1D(float t,int coord){
        Eigen::VectorXd tv(ctrlPts_.rows());
        tv[0] = 1;
        for(size_t i = 1; i < ctrlPts_.rows();++i){
            tv[i] = tv[i-1]*t;
        }
        return tv.transpose()*coefficient_*ctrlPts_.col(coord);
    }


    double approximateBezierLength2(){
        double t;
        double half_t;
        int i;
        int steps = 10;
        Eigen::Vector2d dot;
        Eigen::Vector2d previous_dot;
        double length = 0.0;


        double C1=	0.6521451548625461,t1 =	-0.3399810435848563;
        double C2=	0.6521451548625461,t2 =	0.3399810435848563;
        double C3=	0.3478548451374538,t3 =	-0.8611363115940526;
        double C4=	0.3478548451374538,t4 =	0.8611363115940526;

        t = (double) i / (double) steps;
        half_t = 0.5*t;
        float distx = C1*compute1D(0.5*t1-0.5,0)+C2*compute1D(0.5*t2-0.5,0)+C3*compute1D(0.5*t3-0.5,0)+C4*compute1D(0.5*t4-0.5,0);
        float disty = C1*compute1D(0.5*t1-0.5,1)+C2*compute1D(0.5*t2-0.5,1)+C3*compute1D(0.5*t3-0.5,1)+C4*compute1D(0.5*t4-0.5,1);
        return distx+disty;
    }

    double approximateBezierLength(){
        double t;
        int i;
        int steps = 2000;
        Eigen::Vector2d dot;
        Eigen::Vector2d previous_dot;

        Eigen::Vector2d velocity;
        Eigen::Vector2d acc;

        double length = 0.0;
        for (i = 0; i <= steps; i++) {
            t = (double) i / (double) steps;
            dot = computePts(t);
            if (i > 0) {
                double x_diff = dot.x() - previous_dot.x();
                double y_diff = dot.y() - previous_dot.y();
                length += sqrt (x_diff * x_diff + y_diff * y_diff);
                velocity = b_dt(t);
                acc = b_ddt(t);
                std::cout << "vel :" << velocity.norm() <<std::endl;
                std::cout << "acc :" << acc.norm() <<std::endl;

                double w =std::atan2(dot.x() - previous_dot.x(),
                        -dot.y() + previous_dot.y()) - M_PI/2.;

                if (w > M_PI)        { w -= 2 * M_PI; }
                else if (w <= -M_PI) { w+= 2 * M_PI; }
                std::cout << "angle: " << w <<std::endl;
            }
            previous_dot = dot;
        }
        return length;
    }

    int *dcoeff_;
    int *ddcoeff_;
    int *dddcoeff_;

    int const_d3_[3] =  {1,2,3};
    int const_dd3_[2] =  {2,6};
    int const_d4_[4] =  {1,2,3,4};
    int const_dd4_[3] =  {2,6,12};



    Eigen::MatrixXd ctrlPts_; //4x2 matrix

    Eigen::MatrixXd coefficient_;

    int degree = 3;

};


class OptimalSmallTraj
{
public:
    OptimalSmallTraj(const Eigen::Vector4d &start_pos_i,const Eigen::Vector4d &start_vel_i,
                     const Eigen::Vector4d &end_pos_i,const Eigen::Vector4d &end_vel_i);


    void computeTrajPoints(float resolution_i, std::vector<Eigen::Vector4d> &pts);


    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    double max_yaw_vel_;
    double max_yaw_acc_;


    Eigen::Vector4d start_pos_;
    Eigen::Vector4d end_pos_;

    Eigen::Vector2d start_vel_;
    Eigen::Vector2d end_vel_;

    BezierCurve2D curve_;

};



