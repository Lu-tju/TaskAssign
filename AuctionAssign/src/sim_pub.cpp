#include "AuctionAssign/DetectInfo.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/Header.h"
#include "ros/time.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_publisher");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::Publisher pub = nh.advertise<AuctionAssign::DetectInfo>("/detect_info", 10);

    double x_coord = 0;
    double y_coord = 0;
    double z_coord = 0;
    if (argc == 4)
    {
        x_coord = std::stod(argv[1]);
        y_coord = std::stod(argv[2]);
        z_coord = std::stod(argv[3]);
    }

    ROS_INFO("Custom message published");

    while (ros::ok())
    {
        AuctionAssign::DetectInfo custom_msg;
        custom_msg.header = std_msgs::Header();
        custom_msg.category.push_back(0);               // Replace 1 with your desired category value
        custom_msg.location.push_back(geometry_msgs::Point());
        custom_msg.location[0].x = x_coord; // Replace 0.0 with your desired x-coordinate value
        custom_msg.location[0].y = y_coord; // Replace 0.0 with your desired y-coordinate value
        custom_msg.location[0].z = z_coord; // Replace 0.0 with your desired z-coordinate value

        custom_msg.category.push_back(1);               // Replace 1 with your desired category value
        custom_msg.location.push_back(geometry_msgs::Point());
        custom_msg.location[1].x = x_coord + 10; // Replace 0.0 with your desired x-coordinate value
        custom_msg.location[1].y = y_coord; // Replace 0.0 with your desired y-coordinate value
        custom_msg.location[1].z = z_coord; // Replace 0.0 with your desired z-coordinate value

        pub.publish(custom_msg);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
