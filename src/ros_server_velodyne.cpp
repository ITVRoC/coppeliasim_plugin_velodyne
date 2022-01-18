#include "../include/coppeliasim_plugin_velodyne/ros_server_velodyne.h"
#include "../include/v_repLib.h"
#include <vector>

ros::NodeHandle *ROS_server::node = NULL;

// Publishers:
std::vector<ros::Publisher> publisher;

bool ROS_server::initialize()
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "CoppeliaSim_velodyne");
    if (!ros::master::check())
        return (false);
    node = new ros::NodeHandle();
    return (true);
}

ros::Publisher ROS_server::createPublisher(std::string topic)
{
    auto pub = node->advertise<sensor_msgs::PointCloud2>(topic, 1);
    publisher.push_back(pub);
    return pub;
}

void ROS_server::shutDown()
{
    // Disable the publishers:
    for (auto &a: publisher)
        a.shutdown();
    // Shut down:
    ros::shutdown();
}
