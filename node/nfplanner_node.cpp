#include <ros/ros.h>
#include "planner/nearestfrontierplanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nfplanner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    std::cout << "[Planner] Sleep for 2s" <<std::endl;
    ros::Duration(2.0).sleep();


    NearestFrontierPlanner planner(nh,nh_private);


    ros::spin();

}
