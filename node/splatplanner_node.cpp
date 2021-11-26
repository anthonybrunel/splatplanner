#include <ros/ros.h>
#include <ros/rate.h>
#include <nav_msgs/Path.h>
#include "planner/nearestfrontierplanner.h"
#include "planner/splatplanner.h"
#include <glog/logging.h>
int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    std::cout << "[Planner] Sleep for 2s" <<std::endl;
    ros::Duration(2.0).sleep();
    SplatPlanner planner(nh,nh_private);


    ros::spin();

}
