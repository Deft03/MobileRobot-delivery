#include "ros/ros.h"
#include "your_package_name/MergedCoordinates.h"  // Replace "your_package_name" with the actual name of your package

void processCallback(const your_package_name::MergedCoordinates::ConstPtr& msg)
{
    // Process the received merged coordinate data
    float x_coordinate = msg->x;
    float y_coordinate = msg->y;
    float z_coordinate = msg->z;
    // Add your processing logic here

    ROS_INFO("Received merged coordinates: x=%f, y=%f, z=%f", x_coordinate, y_coordinate, z_coordinate);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_node");
    ros::NodeHandle nh;

    // Subscribe to the merged coordinates topic
    ros::Subscriber sub = nh.subscribe("merged_coordinates", 10, processCallback);

    ros::spin();

    return 0;
}