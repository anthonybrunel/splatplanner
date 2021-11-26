#pragma once
#include <Eigen/Core>
#include <vector>
using namespace  std;
using namespace  Eigen;

class QuinticBezierOpt
{
public:
    QuinticBezierOpt(const Vector4d &start_pos_i,const Vector4d &start_vel_i,
                     const Vector4d &end_pos_i,const Vector4d &end_vel_i);


    void approximateTrajTime();


    void pathSmoothnessCost(const vector<Vector3d>& q, double& cost,
                            vector<Vector3d>& gradient);

    void wayPointsAccCost(const vector<Vector3d>& q, double& cost,
                          vector<Vector3d>& gradient);
    void wayPointsJerkCost(const vector<Vector3d>& q, double& cost,
                           vector<Vector3d>& gradient);




    std::vector<Eigen::Vector3d> ctrl_pts_;//6 pts



};

