#include "chumangrid.h"

CHumanGrid::CHumanGrid()
{
}

CHumanGrid::CHumanGrid(ros::NodeHandle n):
    n_(n)
{
    init();
}

void CHumanGrid::init()
{
    human_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("human_grid", 10);
    highest_point_pub_ = n_.advertise<geometry_msgs::PointStamped>("highest_probability_point", 10) ;
    local_maxima_pub_ = n_.advertise<geometry_msgs::PoseArray>("local_maxima",10);

    initGrid();
    leg_prob_.poses.resize(1600); //TODO: FIX THIS
    sound_prob_.poses.resize(1600);
    torso_prob_.poses.resize(1600);
    occupancy_grid_.data.resize(grid_->grid_size, 0.0);
    occupancy_grid_.info.height = occupancy_grid_.info.width = 40;
    occupancy_grid_.info.resolution = 0.5;
    occupancy_grid_.info.origin.position.x = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.info.origin.position.y = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.header.frame_id = "base_footprint";
    hp_.header.frame_id = "base_footprint";
}

void CHumanGrid::initGrid()
{
    CellProbability_t cp;
    cp.free = 0.1;
    cp.human = 0.9;
    cp.unknown = 0.5;

    SensorFOV_t sfov;
    sfov.range.max = 10.0;
    sfov.range.min = 0.0;
    sfov.angle.max = angles::from_degrees(180.0);
    sfov.angle.min = angles::from_degrees(-180.0);

    try
    {
        grid_ = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new human Grid: bad_alloc caught: " << ba.what() << '\n';
    }
    prob_.poses.resize(grid_->grid_size);

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.x = grid_->map.cell.at(i).cartesian.x;
        prob_.poses.at(i).position.y = grid_->map.cell.at(i).cartesian.y;
        prob_.poses.at(i).position.z = 0.0;
    }
}

void CHumanGrid::legCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    leg_prob_.poses.clear();
    leg_prob_.poses = msg->poses;

    leg_max = 0.0;
    for(size_t i = 0; i < leg_prob_.poses.size(); i++)
    {
        leg_max = (leg_max > leg_prob_.poses.at(i).position.z)
                ? leg_max : leg_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::soundCallBack(const geometry_msgs::PoseArrayConstPtr& msg)
{
    sound_prob_.poses.clear();
    sound_prob_.poses = msg->poses;

    sound_max = 0.0;
    for(size_t i = 0; i < sound_prob_.poses.size(); i++)
    {
        sound_max = (sound_max > sound_prob_.poses.at(i).position.z)
                ? sound_max : sound_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::torsoCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    torso_prob_.poses.clear();
    torso_prob_.poses = msg->poses;

    torso_max = 0.0;
    for(size_t i = 0; i < torso_prob_.poses.size(); i++)
    {
        torso_max = (torso_max > torso_prob_.poses.at(i).position.z)
                ? torso_max : torso_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::weightsCallBack(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    leg_weight = msg->data.at(0);
    sound_weight = msg->data.at(1);
    torso_weight = msg->data.at(2);

}


void CHumanGrid::average()
{
    float maxw = std::max(std::max(leg_max, sound_max), torso_max);

    lw_  = (leg_max > 0.0) ? (maxw / leg_max) : 0.0;
    sw_ = (sound_max > 0.0) ? (maxw / sound_max) : 0.0;
    tw_ = (torso_max > 0.0) ? (maxw / torso_max) : 0.0;

    float num = 0.0;
    float denum = (lw_ * leg_weight) + (sw_ * sound_weight) + (tw_ * torso_weight); //

    std::vector<float> temp;
    temp.resize(prob_.poses.size());

    float max = -1000;

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        num =   lw_ * leg_weight * leg_prob_.poses.at(i).position.z +
                sw_ * sound_weight * sound_prob_.poses.at(i).position.z +
                tw_ * torso_weight * torso_prob_.poses.at(i).position.z;

        prob_.poses.at(i).position.z = num/denum;
        grid_->posterior.at(i) = num/denum;
        temp.at(i) = num/denum;
        max = std::max(max, temp.at(i));
    }

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        occupancy_grid_.data.at(i) = (int) 100 * (temp.at(i)) / max;
    }

    occupancy_grid_.header.stamp = ros::Time::now();
    human_grid_pub_.publish(occupancy_grid_);
    std::vector<float>::iterator it = std::max_element(temp.begin(), temp.end());

    ROS_INFO("max probability: %.4f ", max);

    hp_.header.stamp = ros::Time::now();
    hp_.point = prob_.poses.at(it - temp.begin()).position;
    hp_.point.z = prob_.poses.at(it - temp.begin()).position.z;

    // Using Local Maxima
//    hp_.point = grid_->highest_prob_point.point;
    highest_point_pub_.publish(hp_);

    grid_->updateLocalMaximas();
    grid_->local_maxima_poses.header.stamp = ros::Time::now();
    grid_->local_maxima_poses.header.frame_id = "base_footprint";
    local_maxima_pub_.publish(grid_->local_maxima_poses);
}

CHumanGrid::~CHumanGrid()
{
    ROS_INFO("Deconstructing the constructed Human Grid.");
    delete grid_;
}
