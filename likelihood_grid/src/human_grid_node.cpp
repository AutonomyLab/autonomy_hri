#include "grid.h"
#include "chumangrid.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_node_grid");
    ros::NodeHandle n;
    int loop_rate;
    ros::param::param("~/loop_rate", loop_rate, 10);
    ros::Rate looprate(loop_rate);

    CHumanGrid human_grid(n);

    bool get_leg_grid = true;
    bool get_sound_grid = true;
    bool get_torso_grid = true;

//    ros::param::param("leg_node", get_leg_grid, true);
    ros::param::param("leg_weight", human_grid.leg_weight, 1);

//    ros::param::param("sound_node", get_sound_grid, true);
    ros::param::param("sound_weight", human_grid.sound_weight, 1);

//    ros::param::param("torsi_node", get_torso_grid, true);
    ros::param::param("torso_weight", human_grid.torso_weight, 1);


        ros::Subscriber leg_grid_sub = n.subscribe("leg_probability", 10,
                                                   &CHumanGrid::legCallBack,
                                                   &human_grid);

        ros::Subscriber sound_grid_sub = n.subscribe("sound_probability", 10,
                                                     &CHumanGrid::soundCallBack,
                                                     &human_grid);


        ros::Subscriber torso_grid_sub = n.subscribe("torso_probability", 10,
                                                     &CHumanGrid::torsoCallBack,
                                                     &human_grid);

    while(ros::ok())
    {
        human_grid.average();

        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_ERROR("Not enough time left");

        ros::spinOnce();
    }

    return 0;
}
