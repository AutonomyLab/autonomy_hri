#include "lkgrid_interface.h"

LikelihoodGridInterface::LikelihoodGridInterface():
    update_rate(0),
    update_time_ratio(0),
    free_cell_probability(0)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}



LikelihoodGridInterface::LikelihoodGridInterface(ros::NodeHandle _n,
                                                 GridFOV_t _globalGridFOV,
                                                float _update_rate,
                                                float _update_time_ratio,
                                                float _free_cell_probability):
    n(_n),
    globalGridFOV(_globalGridFOV),
    update_rate(_update_rate),
    update_time_ratio(_update_time_ratio),
    free_cell_probability(_free_cell_probability)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
    legsLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("legLKGrid",10);
}

void LikelihoodGridInterface::update_legs(GridFOV_t sensorGridFOV)
{
    legGrid = new LikelihoodGrid(sensorGridFOV, globalGridFOV, free_cell_probability);
}


void LikelihoodGridInterface::legs_cb(const geometry_msgs::PoseArray& msg)
{
    std::vector<geometry_msgs::Pose> legs;
    std::vector<PolarPose> legPoses;

    if(!msg.poses.empty()){
        lastLegsTime = ros::Time::now();
        legs = msg.poses;
        leg_frame = msg.header.frame_id;
        legPoses.clear();
        PolarPose tempPos;
        // TODO: before this step, make sure they are in the same coordinate system.
        for(size_t i = 0; i < legs.size(); i++){
            tempPos.fromCart(legs.at(i).position.x, legs.at(i).position.y);
            legPoses.push_back(tempPos);
        }
        assert(legPoses.size() == legs.size());
        legGrid->assign(legPoses);
        legPoses.clear();
    }
}

void LikelihoodGridInterface::spin(float cycleTime)
{
    //humanGrid.set(humanGrid.data, 1.0);

    diffLegs = ros::Time::now() - lastLegsTime;
    legGrid->flag = !(diffLegs.toSec() > update_time_ratio*cycleTime);
    legGrid->update(update_rate);

    // difffaces = ros::Time::now() - lastFaceTime;
    // faceGrid.flag = !(difffaces.toSec() >  _update_time_ratio*looprate.cycleTime().toSec());
    // faceGrid.update(_update_rate);

    // humanGrid.fuse(legGrid.data);
    // humanGrid.fuse(faceGrid.data);
    // humanGrid.normalize();

    publish();
}

void LikelihoodGridInterface::publish()
{
    geometry_msgs::Point32 points;
    float r, t;
    sensor_msgs::PointCloud legLKGrid;

    legLKGrid.points.clear();
    for(size_t i = 0; i < legGrid->globalGridSize; i++){
        r = legGrid->data[i].range;
        t = legGrid->data[i].angle;
        points.x = r*cos(t);
        points.y = r*sin(t);
        points.z = legGrid->data[i].probability;
        legLKGrid.points.push_back(points);
    }
    legLKGrid.header.stamp = ros::Time::now();
    legLKGrid.header.frame_id = leg_frame;
    legsLKGrid_pub.publish(legLKGrid);
}

LikelihoodGridInterface::~LikelihoodGridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    delete legGrid;
}
