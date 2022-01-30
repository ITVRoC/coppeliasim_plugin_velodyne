#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ROS_server
{
	public:
		static bool initialize();
		static void shutDown();

        static ros::Publisher getPublisher();
        static ros::Publisher createPublisher(std::string);

	private:
        ROS_server() {}
		
		static ros::NodeHandle* node;

};

#endif
