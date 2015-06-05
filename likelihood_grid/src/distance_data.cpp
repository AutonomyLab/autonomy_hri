#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_data");
    ros::NodeHandle n;
    ros::Rate looprate (10);

    while(ros::ok())
    {
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
