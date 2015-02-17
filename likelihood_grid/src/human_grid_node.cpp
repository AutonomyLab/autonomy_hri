#include "grid.h"
#include "chumangrid.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_node_grid");
    ros::NodeHandle n;
    int loop_rate;
    ros::param::param("~/loop_rate", loop_rate, 5);
    ros::Rate looprate(loop_rate);

    CHumanGrid human_grid(n);

    bool get_leg_grid = true;
    bool get_sound_grid = true;
    bool get_torso_grid = true;

    //    ros::param::param("/human_grid_node/leg_node", get_leg_grid, true);
    //    ros::param::param("/human_grid_node/torso_node", get_torso_grid, true);
    //    ros::param::param("/human_grid_node/sound_node", get_sound_grid, true);
    ros::param::param("/human_grid_node/sound_weight", human_grid.sound_weight, 3.7);
    ros::param::get("/human_grid_node/sound_weight", human_grid.sound_weight);

    ros::param::param("/human_grid_node/torso_weight", human_grid.torso_weight, 3.8);
    ros::param::get("/human_grid_node/torso_weight", human_grid.torso_weight);

    ros::param::param("/human_grid_node/leg_weight", human_grid.leg_weight, 2.5);
    ros::param::get("/human_grid_node/leg_weight", human_grid.leg_weight);


    ros::Subscriber leg_grid_sub = n.subscribe("leg_probability", 10,
                                               &CHumanGrid::legCallBack,
                                               &human_grid);

    ros::Subscriber sound_grid_sub = n.subscribe("sound_probability", 10,
                                                 &CHumanGrid::soundCallBack,
                                                 &human_grid);


    ros::Subscriber torso_grid_sub = n.subscribe("torso_probability", 10,
                                                 &CHumanGrid::torsoCallBack,
                                                 &human_grid);

    ros::Subscriber encoder_sub = n.subscribe("encoder", 10,
                                              &CHumanGrid::encoderCallBack, &human_grid);

    ros::Subscriber weights_sub = n.subscribe("/weights", 10,
                                              &CHumanGrid::weightsCallBack, &human_grid);

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
