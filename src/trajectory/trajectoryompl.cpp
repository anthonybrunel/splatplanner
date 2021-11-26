#include "../../include/trajectory/trajectoryompl.h"

#include "../../../../map_core/utils/boundingmap.hpp"
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include  <ompl/geometric/planners/informedtrees/AITstar.h>
#include "../../../../map_core/utils/timer.hpp"
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
TrajectoryOMPL::TrajectoryOMPL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, MapSearchSpace::Ptr &map, Constraint *constraint):
    nh_(nh),
    nh_private_(nh_private)
{
    map_=map;

    nh_.param("RRT_max_distance", maxDist, maxDist);

    nh_private_.param("camera/fovy", mavVerticalAngle, mavVerticalAngle);
    mavVerticalAngle-=20;
    mavVerticalAngle = ((mavVerticalAngle * M_PI))/180.;
    mavVerticalAngle /=2.;

    std::cout << "OMPL DIST = " << maxDist << " ANGLE: " << mavVerticalAngle << std::endl;
    OMPL_DEBUG("level 0 - warning");

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    constraint_ = constraint;
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
    ob::RealVectorBounds bounds(3);
    bounds_map_ = map_->getBoudingMapReal();
    bounds_map_.min_.z() += constraint_->margin_;
    bounds_map_.max_.z() -= constraint_->margin_;



    bounds.low[0] = bounds_map_.min_(0);
    bounds.low[1] = bounds_map_.min_(1);
    bounds.low[2] = bounds_map_.min_(2);

    bounds.high[0] = bounds_map_.max_(0);
    bounds.high[1] = bounds_map_.max_(1);
    bounds.high[2] = bounds_map_.max_(2);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);


    si_= ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si_,map_,*constraint_)));
    si_->setup();

    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));


    optimizingPlanner_ = ob::PlannerPtr(new og::RRTstar(si_));
    optimizingPlanner_->as<og::RRTstar>()->setRange(maxDist-1.);
    optimizingPlanner_->setProblemDefinition(pdef_);
    optimizingPlanner_->setup();
}

bool TrajectoryOMPL::shiftStart(const Eigen::Vector3f &start, Eigen::Vector3f &result)
{


    Eigen::Vector3i start_i;
    map_->convert(start,start_i);
    Eigen::Vector3i step = ((1./map_->getResolution()) * Eigen::Vector3f(0.4,0.4,0.4)+Eigen::Vector3f(0.5,0.5,0.5)).cast<int>();

    for(int x = -step.x(); x<=step.x(); ++x)   {
        for(int y = -step.y(); y<=step.y(); ++y)   {
            for(int z = -step.z(); z<=step.z(); ++z)   {

                Eigen::Vector3i pt_in_grid = start_i+Eigen::Vector3i(x,y,z);
                if(!map_->isInsideBuf(pt_in_grid))
                    continue;
                map_->convert(pt_in_grid,result);
                float d;
                map_->getDistance(pt_in_grid,d);
                uint8_t flag = map_->getState(pt_in_grid);

                if(d > constraint_->margin_  && (result-start).norm() < 0.49 && bounds_map_.is_inside(result) &&
                        flag !=0 && ((flag &VoxelBuffer::occupied_flag) != 0)){
                    std::cout << "found state: " << d  << " " << result.transpose() << std::endl;
                    return true;
                }
            }
        }
    }
    std::cout << "error no valid state!!" << std::endl;
    return false;
}

bool TrajectoryOMPL::setStart(const Eigen::Vector3f &start)
{
    float d;
    map_->getDistance(start,d);
    Eigen::Vector3f start_tmp = start;

    if(d <= constraint_->margin_ || !bounds_map_.is_inside(start)){
        std::cout << "shift start invalid: " << d  << " " << start.transpose() << std::endl;

        if(!shiftStart(start,start_tmp))
            return false;
    }

    optimizingPlanner_->clear();
    pdef_->clearStartStates();
    pdef_->clearGoal();
    pdef_->clearSolutionPaths();

    ob::ScopedState<> start_ompl(si_);
    start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_tmp.x();
    start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_tmp.y();
    start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_tmp.z();
    pdef_->setStartAndGoalStates(start_ompl,start_ompl);

    return true;

}

void TrajectoryOMPL::setNewGoal(const Eigen::Vector3f &goal)
{

    pdef_->clearGoal();
    pdef_->clearSolutionPaths();
    optimizingPlanner_->clearQuery();
    ob::GoalPtr goalptr(new FrontierGoal(si_,map_,goal));
    pdef_->setGoal(goalptr);
}

ob::PlannerStatus::StatusType TrajectoryOMPL::solvePath(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &result)
{


    Timer t;

//    std::cout << "Searching path toward goal..." <<std::endl;

    //    pdef_->setGoalState(goal_ompl);
    ob::PlannerStatus solved = optimizingPlanner_->solve(plannerOrTerminationCondition
                                                         (ob::exactSolnPlannerTerminationCondition (pdef_),
                                                          ob::timedPlannerTerminationCondition(0.005))
                                                         );

    ob::PlannerStatus::StatusType type = solved.operator ompl::base::PlannerStatus::StatusType();
    int iter = 0;
    //    while(type != ob::PlannerStatus::StatusType::EXACT_SOLUTION){
    //        solved = optimizingPlanner_->solve(plannerOrTerminationCondition
    //                                           (ob::exactSolnPlannerTerminationCondition (pdef_),
    //                                             ob::timedPlannerTerminationCondition(0.001))
    //                                                                         );
    //        type = solved.operator ompl::base::PlannerStatus::StatusType();
    //        if(iter++ > 1)
    //            break;
    //    }
    if(type==ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION){
        //try on more time
        solved = optimizingPlanner_->solve(plannerOrTerminationCondition
                                                             (ob::exactSolnPlannerTerminationCondition (pdef_),
                                                              ob::timedPlannerTerminationCondition(0.005))
                                                             );
        type = solved.operator ompl::base::PlannerStatus::StatusType();

    }

    switch(type){
    case ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
        //        std::cout << "APprox sol" <<std::endl;
        //        std::cout << "[TrajOMPL] t: " << t.elapsed_ms()  <<" "  << result.size() <<std::endl;
//                return false;
        break;
    case ob::PlannerStatus::StatusType::EXACT_SOLUTION:
        //        std::cout << "Exact sol" <<std::endl;
        break;
    default:
//        std::cout << "No solution" <<std::endl;
//        std::cout << "[TrajOMPL] t: " << t.elapsed_ms()  <<" "  << result.size() <<std::endl;
        return type;
    }


    og::PathGeometric path( dynamic_cast< const og::PathGeometric& >( *pdef_->getSolutionPath()));

    og::PathSimplifier simplifier(si_);

    if(!simplifier.simplify(path,0.001)){
//        std::cout << "OMPL Planner no simplification found" <<std::endl;
    }
    const std::vector< ob::State* > &states = path.getStates();
    ob::State *state;
    result.resize(states.size());
    for( size_t i = 0 ; i < states.size( ) ; ++i )
    {
        state = states[ i ]->as< ob::State >( );
        result[i].x() = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        result[i].y() = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        result[i].z() = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
    }

//    std::cout << "[TrajOMPL] t: " << t.elapsed_ms()  <<" "  << result.size() <<std::endl;
    return type;
}


float TrajectoryOMPL::evaluateTime(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &result)
{

}

void TrajectoryOMPL::reset()
{
    optimizingPlanner_->clearQuery();
}
