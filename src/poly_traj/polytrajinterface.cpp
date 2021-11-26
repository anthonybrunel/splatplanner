#include "../../include/poly_traj/polytrajinterface.h"


PolyTrajInterface::PolyTrajInterface(ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    :
    Trajectory(nh,nh_private)

{
}

bool PolyTrajInterface::computeTrajectory(const Constraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel,


                                          mav_trajectory_generation::Trajectory *trajectory)
{
    const int dimension = goal_pos.size();
    // Array for all waypoints and their constraints
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    Eigen::Vector4d start_pos_4d, start_vel_4d;

    std::vector<Eigen::Vector4d> pts;
//    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,start_pos_4d);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_4d);
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
    segment_times = estimateSegmentTimes(vertices, contraint_i.max_v_, contraint_i.max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, contraint_i.max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, contraint_i.max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(contraint_i.max_v_, contraint_i.max_a_);

    return true;

}


