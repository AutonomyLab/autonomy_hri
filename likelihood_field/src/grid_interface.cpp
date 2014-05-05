#include "grid_interface.h"



GridInterface::GridInterface()
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

GridInterface::GridInterface(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
    n(_n),
    tf_listener(_tf_listener)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
    init();
}

void GridInterface::init()
{
    ros::param::param("~/LikelihoodGrid/grid_angle_min",global_fov.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",global_fov.angle.max, M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_resolution",global_fov.angle.resolution, M_PI/18);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf: resolution: %.2lf",
             global_fov.angle.min,
             global_fov.angle.max,
             global_fov.angle.resolution);

    ros::param::param("~/LikelihoodGrid/grid_range_min",global_fov.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",global_fov.range.max, 20.0);
    ros::param::param("~/LikelihoodGrid/grid_range_resolution",global_fov.range.resolution, 0.5);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf: resolution: %.2lf",
             global_fov.range.min,
             global_fov.range.max,
             global_fov.range.resolution);

    ros::param::param("~/LikelihoodGrid/update_rate",update_rate, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",update_rate);

    ros::param::param("~/LikelihoodGrid/human_cell_probability",cell_probability.human, 1.0);
    ROS_INFO("/LikelihoodGrid/human_cell_probability is set to %.2lf",cell_probability.human);

    ros::param::param("~/LikelihoodGrid/free_cell_probability",cell_probability.free, 0.1);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %.2lf",cell_probability.free);

    ros::param::param("~/LikelihoodGrid/unknown_cell_probability",cell_probability.unknown, 0.5);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability is set to %.2lf",cell_probability.unknown);


    ros::param::param("~/LikelihoodGrid/sensitivity",sensitivity, 1);
    ROS_INFO("/LikelihoodGrid/sensitivity is set to %u",sensitivity);

    ros::param::param("~/update_time_ratio",update_time_ratio, 10.0);
    ROS_INFO("/update_time_ratio is set to %.2lf",update_time_ratio);

    ros::param::param("~/leg_detection_enable",leg_detection_enable, true);
    ROS_INFO("/leg_detection_enable to %d",leg_detection_enable);
    ros::param::param("~/torso_detection_enable",torso_detection_enable, true);
    ROS_INFO("/torso_detection_enable to %d",torso_detection_enable);
    ros::param::param("~/sound_detection_enable",sound_detection_enable, true);
    ROS_INFO("/sound_detection_enable to %d",sound_detection_enable);
    ros::param::param("~/laser_detection_enable",laser_detection_enable, true);
    ROS_INFO("/laser_detection_enable to %d",laser_detection_enable);

    number_of_sensors = (leg_detection_enable) + (torso_detection_enable)
            + (sound_detection_enable) + (laser_detection_enable);
    ROS_INFO("number_of_sensors is set to %u",number_of_sensors);

    ROS_ASSERT(sensitivity <= number_of_sensors);

    /* Calculating the prior */
    double upper_bound, lower_bound;
    upper_bound = pow(cell_probability.free, number_of_sensors - sensitivity) * pow(cell_probability.human,sensitivity);
    lower_bound = pow(cell_probability.unknown, number_of_sensors - sensitivity +1) * pow(cell_probability.human,sensitivity - 1);
    //cell_probability.unknown = pow((upper_bound + lower_bound)/2.0, (1.0/number_of_sensors));

    prior_threshold = (upper_bound + lower_bound)/2.0;
    ROS_INFO("Threshold is  %lf",prior_threshold);



    if(leg_detection_enable){
        GridFOV_t legs_fov = global_fov;
        legs_fov.range.max = 20.0;
        legs_fov.angle.min = toRadian(-135.0);//-2.35619449615;
        legs_fov.angle.max = toRadian(135.0);//2.35619449615;
        initLegs(legs_fov);
        legs_grid_pub = n.advertise<sensor_msgs::PointCloud>("leg_likelihood_grid",10);
        leg_counter = 0;
    }

    if(torso_detection_enable){
        GridFOV_t camera_fov = global_fov;
        camera_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.range.max = 10.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.angle.min = toRadian(-65.0/2);
        camera_fov.angle.max = toRadian(65.0/2);
        initFaces(camera_fov);
        face_grid_pub = n.advertise<sensor_msgs::PointCloud>("face_likelihood_grid",10);
        torso_counter = 0;
    }

    if(sound_detection_enable){
        GridFOV_t mic_fov = global_fov;
        mic_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        mic_fov.range.max = 10.00;
        mic_fov.angle.min = toRadian(-90);
        mic_fov.angle.max = toRadian(90);
        initSound(mic_fov);
        sound_grid_pub = n.advertise<sensor_msgs::PointCloud>("sound_likelihood_grid",10);
        sound_counter = 0;
    }

    if(laser_detection_enable){
        GridFOV_t laser_fov = global_fov;
        laser_fov.range.max = 20.0;
        laser_fov.angle.min = toRadian(-135.0);//-2.35619449615;
        laser_fov.angle.max = toRadian(135.0);//2.35619449615;
        initLaser(laser_fov);
        laser_grid_pub = n.advertise<sensor_msgs::PointCloud>("laser_likelihood_grid",10);
        laser_counter = 0;
    }

    initHuman();
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
    if(!leg_detection_enable) return;
    if(leg_counter++ < 5) return;
    else{
        leg_counter = 0;
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
    if(!torso_detection_enable) return;
    if(torso_counter++ < 5) return;
    else{
        torso_counter = 0;
        std::vector<PolarPose> face_polar_base;
        if(msg.numFaces)
        {
            last_face_time = ros::Time::now();
            //geometry_msgs::PointStamped tmp_camera, tmp_base;
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
}

void GridInterface::initSound(GridFOV_t sensor_fov)
{
    try
    {
        sound_grid = new Grid(sensor_fov, global_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new sound_Grid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void GridInterface::soundCallBack(const hark_msgs::HarkSource& msg)
{
    if(!sound_detection_enable) return;
    if(sound_counter++ < 5) return;
    else{
        sound_counter = 0;
        std::vector<hark_msgs::HarkSourceVal> sound_src;
        std::vector<PolarPose> sound_polar_base;
        if(!msg.src.empty()){
            last_sound_time = ros::Time::now();
            sound_src = msg.src;
            PolarPose tmp_polar;
            for(size_t i = 0; i < sound_src.size(); i++){
                tmp_polar.angle = toRadian(sound_src[i].azimuth);
                double r = sound_grid->sensor_fov.range.min;
                while(r <= sound_grid->sensor_fov.range.max){
                    tmp_polar.range = r;
                    r += sound_grid->sensor_fov.range.resolution;
                    sound_polar_base.push_back(tmp_polar);
                }
                sound_frame_id = "base_footprint";
                sound_grid->computeLikelihood(sound_polar_base, sound_grid->new_data);
                sound_polar_base.clear();
            }
        }
    }

}


void GridInterface::initLaser(GridFOV_t sensor_fov)
{
    try
    {
        laser_grid = new Grid(sensor_fov, global_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new laserGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void GridInterface::laserCallBack(const sensor_msgs::LaserScan& msg)
{
    if(!laser_detection_enable) return;
    if(laser_counter++ < 10) return;
    else {
        laser_counter = 0;
        std::vector<PolarPose> laser_polar_base;
        if(!msg.ranges.empty()){
            laser_polar_base.clear();
            last_laser_time = ros::Time::now();
            geometry_msgs::PointStamped tmp_laser, tmp_base;
            PolarPose tmp_polar;
            tmp_laser.header = msg.header;
            float r = 0;
            float t;
            //float t_offset = msg.angle_min + msg.angle_increment * global_fov.angle.resolution;
            float t_offset = msg.angle_min;

            for(size_t i = 0; i < msg.ranges.size(); i++){
                if(msg.ranges.at(i) > global_fov.range.max || msg.ranges.at(i) < 0.5 || (i%2 == 1))
                    continue;
                r = msg.ranges.at(i);
                t =  msg.angle_min + msg.angle_increment*i;
                tmp_laser.point.x = r * cos(t);
                tmp_laser.point.y = r * sin(t);
                if(!transformToBase(tmp_laser, tmp_base)){
                    ROS_WARN("Can not transform from laser to base_footprint");
                    return;
                }
                tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
                laser_polar_base.push_back(tmp_polar);
            }

//            for(size_t c = 0; c < global_fov.getColSize(); c++){
//                float global_angle = global_fov.angle.min + global_fov.angle.resolution*c;
//                int index = round((global_angle - t_offset)/msg.angle_increment);
//                if(index >= 0 && index < msg.ranges.size()){

//                    if(msg.ranges.at(index) > global_fov.range.max || msg.ranges.at(index) < 0.5)
//                        continue;

//                    r = msg.ranges.at(index);
//                    t = global_angle;
//                    tmp_laser.point.x = r * cos(t);
//                    tmp_laser.point.y = r * sin(t);
//                    if(!transformToBase(tmp_laser, tmp_base)){
//                        ROS_WARN("Can not transform from laser to base_footprint");
//                        return;
//                    }
//                    tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
//                    laser_polar_base.push_back(tmp_polar);
//                }
//            }

            laser_frame_id = "base_footprint";
            laser_grid->computeLikelihood(laser_polar_base, laser_grid->new_data);
            laser_polar_base.clear();
        }
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

sensor_msgs::PointCloud GridInterface::pointCloudGrid(Grid* polar_grid)
{
    double r, t;
    geometry_msgs::Point32 points;
    sensor_msgs::ChannelFloat32 probability_channel, threshold_channel;
    sensor_msgs::PointCloud pointcloud_grid;

    probability_channel.name = "probability";
    threshold_channel.name = "threshold";

    //prior_threshold = pow(cell_probability.unknown, number_of_sensors);

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

void GridInterface::publish()
{
    //LEGS
    if(leg_detection_enable){
        sensor_msgs::PointCloud leg_pointcloud_grid = GridInterface::pointCloudGrid(leg_grid);
        leg_pointcloud_grid.header.stamp = ros::Time::now();
        leg_pointcloud_grid.header.frame_id = "base_footprint";
        legs_grid_pub.publish(leg_pointcloud_grid);
    }

    // FACES
    if(torso_detection_enable){
        sensor_msgs::PointCloud face_pointcloud_grid = pointCloudGrid(face_grid);
        face_pointcloud_grid.header.stamp = ros::Time::now();
        face_pointcloud_grid.header.frame_id = "base_footprint";
        face_grid_pub.publish(face_pointcloud_grid);
    }

    // Sound
    if(sound_detection_enable){
        sensor_msgs::PointCloud sound_pointcloud_grid = pointCloudGrid(sound_grid);
        sound_pointcloud_grid.header.stamp = ros::Time::now();
        sound_pointcloud_grid.header.frame_id = "base_footprint";
        sound_grid_pub.publish(sound_pointcloud_grid);
    }

    //LASER
    if(laser_detection_enable){
        sensor_msgs::PointCloud laser_pointcloud_grid = GridInterface::pointCloudGrid(laser_grid);
        laser_pointcloud_grid.header.stamp = ros::Time::now();
        laser_pointcloud_grid.header.frame_id = "base_footprint";
        laser_grid_pub.publish(laser_pointcloud_grid);
    }

    // HUMAN
    sensor_msgs::PointCloud human_pointcloud_grid = pointCloudGrid(human_grid);
    human_pointcloud_grid.header.stamp = ros::Time::now();
    human_pointcloud_grid.header.frame_id = "base_footprint";
    human_grid_pub.publish(human_pointcloud_grid);


}


void GridInterface::spin()
{
    // LEGS
    if(leg_detection_enable){
        diff_leg_time = ros::Time::now() - last_leg_time;
        leg_grid->flag = !(diff_leg_time.toSec() > 2.0);
        leg_grid->sensorUpdate(update_rate);
    }

    // FACES
    if(torso_detection_enable){
        diff_face_time = ros::Time::now() - last_face_time;
        face_grid->flag = !(diff_face_time.toSec() > 2.0);
        face_grid->sensorUpdate(update_rate);
    }

    // SOUND
    if(sound_detection_enable){
        diff_sound_time = ros::Time::now() - last_sound_time;
        sound_grid->flag = !(diff_sound_time.toSec() > 2.0);
        sound_grid->sensorUpdate(update_rate);
    }

    // LASER
    if(laser_detection_enable){
        diff_laser_time = ros::Time::now() - last_laser_time;
        laser_grid->flag = !(diff_laser_time.toSec() > 2.0);
        laser_grid->sensorUpdate(update_rate);
    }
    // HUMAN
    human_grid->setProbability(human_grid->new_data, cell_probability.human);
    if(leg_detection_enable)
        human_grid->fuse(leg_grid->data);
    if(torso_detection_enable)
        human_grid->fuse(face_grid->data);
    if(sound_detection_enable)
        human_grid->fuse(sound_grid->data);
    if(laser_detection_enable)
        human_grid->fuse(laser_grid->data);
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



GridInterface::~GridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    if(leg_detection_enable) delete leg_grid;
    if(torso_detection_enable) delete face_grid;
    if(sound_detection_enable) delete sound_grid;
    if(laser_detection_enable) delete laser_grid;
    delete human_grid;
    delete tf_listener;
    delete[] world_base;
    delete[] world_odom;
}
