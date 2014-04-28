#include "grid_interface.h"



GridInterface::GridInterface():
    update_rate(0),
    update_time_ratio(0)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

GridInterface::GridInterface(ros::NodeHandle _n,
                                                 tf::TransformListener *_tf_listener,
                                                 GridFOV_t _global_fov,
                                                double _update_rate,
                                                double _update_time_ratio,
                                                 CellProbability_t _cell_probability):
    n(_n),
    tf_listener(_tf_listener),
    global_fov(_global_fov),
    update_rate(_update_rate),
    update_time_ratio(_update_time_ratio),
    cell_probability(_cell_probability)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");

    legs_grid_pub = n.advertise<sensor_msgs::PointCloud>("leg_likelihood_grid",10);
    face_grid_pub = n.advertise<sensor_msgs::PointCloud>("face_likelihood_grid",10);
    human_grid_pub = n.advertise<sensor_msgs::PointCloud>("human_likelihood_grid",10);
    try
    {
        tf_listener = new tf::TransformListener();
        world_base = new PointRAP_t[global_fov.getSize()];
        world_odom = new PointRAP_t[global_fov.getSize()];
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Likelihood Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }

    initWorldGrids();
}

void GridInterface::initWorldGrids()
{
    size_t i,r,c;
    i = 0;
    for(r = 0; r < global_fov.getRowSize(); r++){
        for(c = 0; c < global_fov.getColSize(); c++){
            world_base[i].range = global_fov.range.min + global_fov.range.resolution*r;
            world_base[i].angle = global_fov.angle.min + global_fov.angle.resolution*c;
            world_base[i].probability = 1.0;
            world_odom[i].range = global_fov.range.min + global_fov.range.resolution*r;
            world_odom[i].angle = global_fov.angle.min + global_fov.angle.resolution*c;
            world_odom[i].probability = 1.0;
            i++;
        }
    }
    ROS_ASSERT(i == global_fov.getSize());
    ROS_ASSERT(fabs(world_odom[global_fov.getSize()-1].range - global_fov.range.max) < 0.001);
    ROS_ASSERT(fabs(world_odom[global_fov.getSize()-1].angle - global_fov.angle.max) < 0.0001);
}


bool GridInterface::transformToBase(geometry_msgs::PointStamped& source_point, geometry_msgs::PointStamped &target_point, bool debug)
{
    bool can_transform;
    try
    {
//        listener.waitForTransform("base_footprint", tmp_point.header.frame_id, ros::Time(0),
//                    ros::Duration(5.0));
        tf_listener->transformPoint("base_footprint", source_point, target_point);
        if (debug) {
            tf::StampedTransform _t;
            tf_listener->lookupTransform("base_footprint", source_point.header.frame_id, ros::Time(0), _t);
            ROS_INFO("From %s to bfp: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
                     source_point.header.frame_id.c_str(),
                     _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
                     _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(), _t.getRotation().getW());
        }
        can_transform = true;
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_footprint\": %s", source_point.header.frame_id.c_str(), ex.what());
        can_transform = false;
    }
    return can_transform;
}

bool GridInterface::transformToOdom(geometry_msgs::PointStamped& source_point, geometry_msgs::PointStamped target_point, bool debug)
{
    bool can_transform;
    try
    {
        tf_listener->transformPoint("odom", source_point, target_point);
        if (debug) {
            tf::StampedTransform _t;
            tf_listener->lookupTransform("odom", source_point.header.frame_id, ros::Time(0), _t);
            ROS_INFO("From %s to odom: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
                     source_point.header.frame_id.c_str(),
                     _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
                     _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(), _t.getRotation().getW());
        }
        can_transform = true;
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"odom\": %s", source_point.header.frame_id.c_str(), ex.what());
        can_transform = false;
    }
    return can_transform;
}


void GridInterface::initLegs(GridFOV_t sensor_fov)
{
    try
    {
        leg_grid = new Grid(sensor_fov, global_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void GridInterface::legCallBack(const geometry_msgs::PoseArray& msg)
{
    std::vector<geometry_msgs::Pose> legs_laser;
    std::vector<PolarPose> legs_polar_base;

    if(!msg.poses.empty()){
        legs_polar_base.clear();
        last_leg_time = ros::Time::now();

        // FRAME TRANSFORM: /laser to /base_footprint
        legs_laser = msg.poses;
        geometry_msgs::PointStamped tmp_laser, tmp_base;
        PolarPose tmp_polar;
        for(size_t i = 0; i < legs_laser.size(); i++){
            tmp_laser.header = msg.header;
            tmp_laser.point = legs_laser.at(i).position;
            if(!transformToBase(tmp_laser, tmp_base)){
                ROS_WARN("Can not transform from laser to base_footprint");
                return;
            }
            tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
            legs_polar_base.push_back(tmp_polar);
        }

        //ROS_ASSERT(legs_polar_base.size() == legs_laser.size());
        leg_frame_id = "base_footprint";
        leg_grid->computeLikelihood(legs_polar_base, leg_grid->new_data);
        legs_polar_base.clear();
    }
}

void GridInterface::initFaces(GridFOV_t sensor_fov)
{
    try
    {
        face_grid = new Grid(sensor_fov,global_fov,cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new faceGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void GridInterface::faceCallBack(const autonomy_human::human& msg)
{
    std::vector<PolarPose> face_polar_base;
    if(msg.numFaces > 0)
    {
        last_face_time = ros::Time::now();
        geometry_msgs::PointStamped tmp_camera, tmp_base;
        PolarPose tmp_polar;
        double theta, x;
        double image_width = 640.0;

        x = msg.faceROI.x_offset + msg.faceROI.width/2.0;
        theta = ((image_width/2.0-x)/image_width/2.0)*toRadian(65.0/2.0);
        tmp_polar.angle = theta;

        double r = face_grid->sensor_fov.range.min;
        while(r <= face_grid->sensor_fov.range.max){
            tmp_polar.range = r;
            r += face_grid->sensor_fov.range.resolution;
            face_polar_base.push_back(tmp_polar);
        }
        face_frame_id = "base_footprint";
        face_grid->computeLikelihood(face_polar_base, face_grid->new_data);
        face_polar_base.clear();
    }

}

void GridInterface::initHuman()
{
    try
    {
        human_grid = new Grid(global_fov,global_fov,cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void GridInterface::spin()
{

    // LEGS
    diff_leg_time = ros::Time::now() - last_leg_time;
    leg_grid->flag = !(diff_leg_time.toSec() > 2.0);
    leg_grid->sensorUpdate(update_rate);


    // FACES
    diff_face_time = ros::Time::now() - last_face_time;
    face_grid->flag = !(diff_face_time.toSec() > 2.0);
    face_grid->sensorUpdate(update_rate);

    // HUMAN
    human_grid->setProbability(human_grid->new_data, cell_probability.human);
    human_grid->fuse(leg_grid->data);
    human_grid->fuse(face_grid->data);
    //human_grid->output();

/*
    // transform worldGrid_odom to worldGrid_base

    geometry_msgs::PointStamped tmp_odom, tmp_base;
    tmp_odom.header.frame_id = "odom";
    tmp_base.header.frame_id = "base_footprint";
    tmp_odom.header.stamp = ros::Time(0);
    tmp_base.header.stamp = ros::Time(0);

    tmp_odom.point.z = 0.0;
    tmp_base.point.z = 0.0;

    size_t i;

    for(i = 0; i < global_fov.getSize(); i++){
        tmp_odom.point.x = world_odom[i].range * cos(world_odom[i].angle);
        tmp_odom.point.y = world_odom[i].range * sin(world_odom[i].angle);
        if(!transformToBase(tmp_odom, tmp_base, false)){
            ROS_WARN("Can not transform from odom to base_footprint");
            return;
        }
        world_base[i].angle = atan2(tmp_base.point.y, tmp_base.point.x);
        world_base[i].range = sqrt(pow(tmp_base.point.y,2)+pow(tmp_base.point.x,2));
        world_base[i].probability = world_odom[i].probability;
    }

    i = global_fov.getSize() -2000;
    ROS_INFO("---------------------------------------------");
    tmp_odom.point.x = world_odom[i].range * cos(world_odom[i].angle);
    tmp_odom.point.y = world_odom[i].range * sin(world_odom[i].angle);
    if(!transformToBase(tmp_odom, tmp_base, false)){
        ROS_WARN("Can not transform from odom to base_footprint");
        return;
    }
    world_base[i].angle = atan2(tmp_base.point.y, tmp_base.point.x);
    world_base[i].range = sqrt(pow(tmp_base.point.y,2)+pow(tmp_base.point.x,2));
    world_base[i].probability = world_odom[i].probability;
    ROS_INFO("[%4lu] world_odom: from range= %.2lf  and angle= %.2lf", i, world_odom[i].range, world_odom[i].angle);
    ROS_INFO("[%4lu] temp_odom : x= %.2f  y= %.2f", i, tmp_odom.point.x, tmp_odom.point.y);
    ROS_INFO("[%4lu] temp_base : x= %.2f  y= %.2f",i, tmp_base.point.x, tmp_base.point.y);
    ROS_INFO("[%4lu] world_base: range= %.2lf  and angle= %.2lf", i, world_base[i].range, world_base[i].angle);
    */

    human_grid->worldUpdate(world_base, 0.0);

    /*

    for(i = 0; i < global_fov.getSize(); i++){
        tmp_base.point.x = human_grid->data[i].range * cos(human_grid->data[i].angle);
        tmp_base.point.y = human_grid->data[i].range * sin(human_grid->data[i].angle);
        if(!transformToOdom(tmp_base, tmp_odom, false))
        {
            ROS_WARN("Can not transform from base to odom");
            return;
        }
        world_odom[i].angle = atan2(tmp_odom.point.y, tmp_odom.point.x);
        world_odom[i].range = sqrt(pow(tmp_odom.point.y,2)+pow(tmp_odom.point.x,2));
        world_odom[i].probability = human_grid->data[i].probability;
    }
    i = global_fov.getSize() -2000;
    tmp_base.point.x = human_grid->data[i].range * cos(human_grid->data[i].angle);
    tmp_base.point.y = human_grid->data[i].range * sin(human_grid->data[i].angle);
    if(!transformToOdom(tmp_base, tmp_odom, false))
    {
        ROS_WARN("Can not transform from base to odom");
        return;
    }
    world_odom[i].angle = atan2(tmp_odom.point.y, tmp_odom.point.x);
    world_odom[i].range = sqrt(pow(tmp_odom.point.y,2)+pow(tmp_odom.point.x,2));
    world_odom[i].probability = human_grid->data[i].probability;
    ROS_INFO("[%4lu] human_grid: from range= %.2lf  and angle= %.2lf", i, human_grid->data[i].range, human_grid->data[i].angle);
    ROS_INFO("[%4lu] temp_base : x= %.2f  y= %.2f", i, tmp_base.point.x, tmp_base.point.y);
    ROS_INFO("[%4lu] temp_odom : x= %.2f  y= %.2f",i, tmp_odom.point.x, tmp_odom.point.y);
    ROS_INFO("[%4lu] world_odom: range= %.2lf  and angle= %.2lf", i, world_odom[i].range, world_odom[i].angle);

*/
    publish();
}

void GridInterface::publish()
{

    //LEGS
    sensor_msgs::PointCloud leg_pointcloud_grid = GridInterface::pointCloudGrid(leg_grid);
    leg_pointcloud_grid.header.stamp = ros::Time::now();
    leg_pointcloud_grid.header.frame_id = "base_footprint";
    legs_grid_pub.publish(leg_pointcloud_grid);

    // FACES
    sensor_msgs::PointCloud face_pointcloud_grid = pointCloudGrid(face_grid);
    face_pointcloud_grid.header.stamp = ros::Time::now();
    face_pointcloud_grid.header.frame_id = "base_footprint";
    face_grid_pub.publish(face_pointcloud_grid);

    // HUMAN
    sensor_msgs::PointCloud human_pointcloud_grid = pointCloudGrid(human_grid);
    human_pointcloud_grid.header.stamp = ros::Time::now();
    human_pointcloud_grid.header.frame_id = "base_footprint";
    human_grid_pub.publish(human_pointcloud_grid);
    //humanGrid->output();
}

sensor_msgs::PointCloud GridInterface::pointCloudGrid(Grid* polar_grid)
{
    double r, t, prior_threshold;
    geometry_msgs::Point32 points;
    sensor_msgs::ChannelFloat32 probability_channel, threshold_channel;
    sensor_msgs::PointCloud pointcloud_grid;
    probability_channel.name = "probability";
    threshold_channel.name = "threshold";
    prior_threshold = pow(cell_probability.unknown, 2);

    for(size_t i = 0; i < polar_grid->global_fov.getSize(); i++){
        r = polar_grid->data[i].range;
        t = polar_grid->data[i].angle;
        points.x = r*cos(t);
        points.y = r*sin(t);
        points.z = polar_grid->data[i].probability;
        probability_channel.values.push_back(polar_grid->data[i].probability);
        threshold_channel.values.push_back(prior_threshold);
        pointcloud_grid.points.push_back(points);
    }
    pointcloud_grid.channels.push_back(probability_channel);
    pointcloud_grid.channels.push_back(threshold_channel);
    return pointcloud_grid;
}

GridInterface::~GridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    delete leg_grid;
    delete face_grid;
    delete human_grid;
    delete tf_listener;
    delete[] world_base;
    delete[] world_odom;
}
