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


//    bool get_leg_grid = true;
//    bool get_sound_grid = true;
//    bool get_torso_grid = true;

    double lw, sw, tw;

    //    ros::param::param("/human_grid_node/leg_node", get_leg_grid, true);
    //    ros::param::param("/human_grid_node/torso_node", get_torso_grid, true);
    //    ros::param::param("/human_grid_node/sound_node", get_sound_grid, true);
    ros::param::param("/human_grid_node/sound_weight", sw, 4);
    ros::param::get("/human_grid_node/sound_weight", sw);

    ros::param::param("/human_grid_node/torso_weight",tw, 3);
    ros::param::get("/human_grid_node/torso_weight", tw);

    ros::param::param("/human_grid_node/leg_weight", lw, 2);
    ros::param::get("/human_grid_node/leg_weight", lw);
    CHumanGrid human_grid(n,lw, sw, tw);


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
        human_grid.integrateProbabilities();

        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_ERROR("Not enough time left");

        ros::spinOnce();
    }

    return 0;
}
