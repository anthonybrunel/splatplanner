#include "../../include/poly_traj/optimalsmalltraj.h"
#include <iostream>


OptimalSmallTraj::OptimalSmallTraj(const Eigen::Vector4d &start_pos_i, const Eigen::Vector4d &start_vel_i, const Eigen::Vector4d &end_pos_i, const Eigen::Vector4d &end_vel_i)
{

    Eigen::Vector2d dir1,dir2;
    dir1 = Eigen::AngleAxisd(start_pos_i.w(),Eigen::Vector3d(0,0,1)).matrix().col(0).head(2);

    dir2 = Eigen::AngleAxisd(end_pos_i.w(),Eigen::Vector3d(0,0,-1)).matrix().col(0).head(2);

    dir1.normalize();
    dir2.normalize();

    start_pos_ = start_pos_i;
    end_pos_ = end_pos_i;
    std::cout << "Start pos " << start_pos_.transpose() <<std::endl;
    std::cout << "End pos " << end_pos_.transpose() <<std::endl;

    curve_.ctrlPts_.resize(4,2);
    curve_.ctrlPts_.row(0) = start_pos_i.head(2);
    curve_.ctrlPts_.row(3) = end_pos_i.head(2);

    double one_third = 1/3.;
    Eigen::Vector2d segment_dir = (curve_.ctrlPts_.row(3)  - curve_.ctrlPts_.row(0)).transpose();
    double dist = segment_dir.norm()*one_third;
    segment_dir.normalize();
    double velocity = 0.5;
    curve_.ctrlPts_.row(1) = curve_.ctrlPts_.row(0)  +  (dir1*(dist+velocity)*segment_dir.dot(dir1)).transpose();
    curve_.ctrlPts_.row(2) = curve_.ctrlPts_.row(3)  +  (dir2*(-(dist+velocity))*segment_dir.dot(dir2)).transpose();

//    float alpha,beta;

//    beta = 1/
//    curve_.ctrlPts_.row(2) = invdir2*((1/3.)*curve_.ctrlPts_.row(0).transpose()+
//                                      2*(curve_.ctrlPts_.row(0).transpose()+0.5*invdir1*curve_.ctrlPts_.row(0).transpose()+
//                                         curve_.ctrlPts_.row(2).transpose())+4/3.*curve_.ctrlPts_.row(3).transpose());



//    curve_.ctrlPts_.row(1) = (invdir1*(1.f/2.f) *
//                              (-curve_.ctrlPts_.row(0).transpose() + curve_.ctrlPts_.row(2).transpose())).transpose();

    curve_.coefficient_.resize(4,4);
    curve_.coefficient_ << 1 ,0,0,0,
            -3,3,0,0,
            3,-6,3,0,
            -1,3,-3,1;

    std::cout << curve_.ctrlPts_ <<std::endl;
////    std::cout << curve_.b_dt(0).transpose() <<std::endl;
//    std::cout << curve_.b_ddt(0).transpose() <<std::endl;


//    std::cout << curve_.b_dt(1).transpose() <<std::endl;
    std::cout << curve_.b_ddt(0.9).transpose().norm() <<std::endl;
    std::cout << curve_.b_ddt(1).transpose().norm() <<std::endl;



//        std::cout <<"Len " <<  curve_.approximateBezierLength() <<std::endl;
    //    std::vector<Eigen::Vector4d> pts;
    //    computeTrajPoints(0.01,pts);
}

void OptimalSmallTraj::computeTrajPoints(float resolution_i, std::vector<Eigen::Vector4d>& pts)
{

    int N = 1./resolution_i;
    pts.resize(N+1);
    pts[0] = start_pos_;
    pts[N] = end_pos_;
    double t;
    for(int i = 1; i < N; ++i){
        t = resolution_i*i;
        if((start_pos_.head(2)-end_pos_.head(2)).norm() < 0.002){
            pts[i].head(2) = pts[i-1].head(2);
            pts[i](3) = (1 - t)*start_pos_(3) + t*end_pos_(3); // linear interpolation

        }else{
            pts[i].head(2) = curve_.computePts(t);
            pts[i](3) =std::atan2(pts[i].x() - pts[i-1].x(),
                    -pts[i].y() + pts[i-1].y()) - M_PI/2.;

            if (pts[i](3) > M_PI)        { pts[i](3) -= 2 * M_PI; }
            else if (pts[i](3) <= -M_PI) { pts[i](3) += 2 * M_PI; }
        }
        pts[i](2) = (1 - t)*start_pos_(2) + t*end_pos_(2);


        std::cout << t << " " << pts[i].transpose() <<std::endl;
    }


}

