#ifndef TRAJECTORYOMPL_H
#define TRAJECTORYOMPL_H


#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include "../map/mapsearchspace.h"
#include "../planner/constraint.h"
namespace ob = ompl::base;;
namespace og = ompl::geometric;
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si,MapSearchSpace::Ptr &map, Constraint &constraint) :
        ob::StateValidityChecker(si) {

        map_ = map;
        margin_ = constraint.margin_;

    }

    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) >= margin_;
    }

    double clearance(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state3D =
                state->as<ob::RealVectorStateSpace::StateType>();

        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];
        float d = 0;
        map_->getDistance(Eigen::Vector3f(x,y,z),d);

        return d;
    }

    MapSearchSpace::Ptr map_;
    double margin_ = 0.3;
};


class FrontierGoal : public ob::GoalState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrontierGoal(const ob::SpaceInformationPtr &si, MapSearchSpace::Ptr &map,const Eigen::Vector3f &goal, const float maxVerticalAngle = 0.4, const float maxDist = 2.5 ) : ob::GoalState(si), map_(map), goal_(goal)
    {
        maxDist_ = maxDist;
        angle_cam = maxVerticalAngle;
        goal_ = goal;
        goal_normalized_ = goal.normalized();
        ob::ScopedState<> goal_st(si_);
        goal_st->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal.x();
        goal_st->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal.y();
        goal_st->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal.z();
        setState (goal_st);
        setThreshold(maxDist_);
    }

    virtual bool isSatisfied(const ob::State *st) const
    {

        Eigen::Vector3f state(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                st->as<ob::RealVectorStateSpace::StateType>()->values[2]
                );


        Eigen::Vector3f dir = goal_-state;
        //validate if close
        if(dir.norm() < maxDist_){
            //check angle less  20 degree z  is forward
            float dot = dir.normalized().z();/*Eigen::Vector3f::UnitZ().dot(dir.normalized());*/
            float angle = acos(dot);
            if(dot < 0){
                angle = angle-M_PI_2;
            }else{
                angle = M_PI_2-angle;
            }
            //25deg
            if(angle < angle_cam){
                if(map_->checkOcclusion(goal_,state)){
                    return true;
                }
            }
        }

        return false;
    }

    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        bool result = isSatisfied(st);
        Eigen::Vector3f state(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                st->as<ob::RealVectorStateSpace::StateType>()->values[2]
                );

        if (distance != NULL)
        {
            if (!result)
            {
                *distance = (state-goal_).norm();
            }
            else
            {
                *distance = (state-goal_).norm();
            }
        }
        return result;
    }

    virtual double distanceGoal(const ob::State *st) const
    {
        Eigen::Vector3f state(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                st->as<ob::RealVectorStateSpace::StateType>()->values[2]
                );
        return (state-goal_).norm();

    }
    MapSearchSpace::Ptr map_;
    Eigen::Vector3f goal_;
    Eigen::Vector3f goal_normalized_;

    //2.5 maze 5m cam
    //3.5 powerplant
    float maxDist_ = 2.5;
    float angle_cam = 0.40;
};



class TrajectoryOMPL
{
public:
    TrajectoryOMPL(){

    }

    TrajectoryOMPL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, MapSearchSpace::Ptr &map, Constraint *constraint);


    bool shiftStart(const Eigen::Vector3f &start, Eigen::Vector3f &result);

    bool setStart(const Eigen::Vector3f &start);

    void setNewGoal(const Eigen::Vector3f &start);

    ompl::base::PlannerStatus::StatusType solvePath(std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> &result);

    bool findPath(const Eigen::Vector3f &start, const Eigen::Vector3f &goal,std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> &result);

    float evaluateTime(std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> &result);

    void reset();

    MapSearchSpace::Ptr map_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    Constraint *constraint_;

    ob::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr  optimizingPlanner_;

    BoundingMap3D<float> bounds_map_;

    //goal parameters
    float maxDist = 2.5;
    float mavVerticalAngle = 67;

};

#endif // TRAJECTORYOMPL_H
