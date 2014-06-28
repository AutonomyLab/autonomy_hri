#include "cartesian_grid_interface.h"


CartesianGridInterface::CartesianGridInterface()
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

CartesianGridInterface::CartesianGridInterface(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
    n(_n),
    tf_listener(_tf_listener)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
    init();
}

void CartesianGridInterface::init()
{


    ros::param::param("~/LikelihoodGrid/grid_angle_min",fov.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",fov.angle.max, M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_resolution",fov.angle.res, M_PI/18);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf: resolution: %.2lf",
             fov.angle.min,
             fov.angle.max,
             fov.angle.res);

    ros::param::param("~/LikelihoodGrid/grid_range_min",fov.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",fov.range.max, 20.0);
    ros::param::param("~/LikelihoodGrid/grid_range_resolution",fov.range.res, 0.5);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf: resolution: %.2lf",
             fov.range.min,
             fov.range.max,
             fov.range.res);

    //ros::param::param("~/CartesianLikelihoodGrid/map_size",map_size, 2);

    ros::param::param("~/CartesianLikelihoodGrid/resolution",map_resolution, 0.5);
    map_size = fov.range.max * 2 / map_resolution;

    ROS_INFO("/CartesianLikelihoodGrid size: %d resolution: %.2lf",
             map_size, map_resolution);

    ros::param::param("~/LikelihoodGrid/update_rate",update_rate, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",update_rate);

    ros::param::param("~/LikelihoodGrid/human_cell_probability",cell_probability.human, 100);
    ROS_INFO("/LikelihoodGrid/human_cell_probability is set to %d",cell_probability.human);

    ros::param::param("~/LikelihoodGrid/free_cell_probability",cell_probability.free, 0);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %d",cell_probability.free);

    ros::param::param("~/LikelihoodGrid/unknown_cell_probability",cell_probability.unknown, -1);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability is set to %d",cell_probability.unknown);


    ros::param::param("~/LikelihoodGrid/sensitivity",sensitivity, 1);
    ROS_INFO("/LikelihoodGrid/sensitivity is set to %u",sensitivity);

    ros::param::param("~/loop_rate",loop_rate, 10);
    ROS_INFO("/loop_rate is set to %d",loop_rate);

    ros::param::param("~/leg_detection_enable",leg_detection_enable, false);
    ROS_INFO("/leg_detection_enable to %d",leg_detection_enable);

    ros::param::param("~/torso_detection_enable",torso_detection_enable, false);
    ROS_INFO("/torso_detection_enable to %d",torso_detection_enable);

    ros::param::param("~/sound_detection_enable",sound_detection_enable, false);
    ROS_INFO("/sound_detection_enable to %d",sound_detection_enable);

    ros::param::param("~/laser_detection_enable",laser_detection_enable, false);
    ROS_INFO("/laser_detection_enable to %d",laser_detection_enable);

    number_of_sensors = (leg_detection_enable) + (torso_detection_enable)
            + (sound_detection_enable) + (laser_detection_enable);

    ROS_INFO("number_of_sensors is set to %u",number_of_sensors);

    //ROS_ASSERT(sensitivity <= number_of_sensors);

////////// [???????]
//    /* Calculating the prior */
//    double upper_bound, lower_bound;
//    upper_bound = pow(cell_probability.free, number_of_sensors - sensitivity) * pow(cell_probability.human,sensitivity);
//    lower_bound = pow(cell_probability.free, number_of_sensors - sensitivity +1) * pow(cell_probability.human,sensitivity - 1);
//    cell_probability.unknown = pow((upper_bound + lower_bound)/2.0, (1.0/number_of_sensors));

//    prior_threshold = (upper_bound + lower_bound)/2.0;
//    ROS_INFO("Threshold is  %lf",prior_threshold);

//    ROS_INFO("/LikelihoodGrid/unknown_cell_probability has been changed to %.2lf",cell_probability.unknown);


    accept_counter = 0;
    reject_counter = 0;

    if(leg_detection_enable){
        SensorFOV_t legs_fov = fov;
        //legs_fov.range.max = 20.0;
        legs_fov.range.max = (fov.range.max < 20.0 ? fov.range.max:20.0);
        legs_fov.angle.min = toRadian(-135.0);//-2.35619449615;
        legs_fov.angle.max = toRadian(135.0);//2.35619449615;
        initLegs(legs_fov);
        legs_grid_pub = n.advertise<sensor_msgs::PointCloud>("leg_likelihood_grid",10);
        leg_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("leg_occupancy_grid",10);
        leg_counter = 0;
    }

    if(torso_detection_enable){
        SensorFOV_t camera_fov = fov;
        camera_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.range.max = 10.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.angle.min = toRadian(-65.0/2);
        camera_fov.angle.max = toRadian(65.0/2);
        initFaces(camera_fov);
        face_grid_pub = n.advertise<sensor_msgs::PointCloud>("face_likelihood_grid",10);
        face_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("face_occupancy_grid",10);
        torso_counter = 0;
    }

    if(sound_detection_enable){
        SensorFOV_t mic_fov = fov;
        mic_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        mic_fov.range.max = 10.00;
        mic_fov.angle.min = toRadian(-90);
        mic_fov.angle.max = toRadian(90);
        initSound(mic_fov);
        sound_grid_pub = n.advertise<sensor_msgs::PointCloud>("sound_likelihood_grid",10);
        sound_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("sound_occupancy_grid",10);
        sound_counter = 0;
    }

    if(laser_detection_enable){
        SensorFOV_t laser_fov = fov;
        laser_fov.range.max = 20.0;
        laser_fov.angle.min = toRadian(-135.0);//-2.35619449615;
        laser_fov.angle.max = toRadian(135.0);//2.35619449615;
        initLaser(laser_fov);
        laser_grid_pub = n.advertise<sensor_msgs::PointCloud>("laser_likelihood_grid",10);
        laser_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("laser_occupancy_grid",10);
        laser_counter = 0;
    }
    initHuman();
    human_grid_pub = n.advertise<sensor_msgs::PointCloud>("human_likelihood_grid",10);
    human_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("human_occupancy_grid",10);

    try
    {
        tf_listener = new tf::TransformListener();
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Likelihood Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }

}

bool CartesianGridInterface::transformToBase(geometry_msgs::PointStamped& source_point, geometry_msgs::PointStamped &target_point, bool debug)
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



void CartesianGridInterface::initLegs(SensorFOV_t sensor_fov)
{
    try
    {
        leg_grid = new CartesianGrid(map_size, map_resolution, sensor_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::legCallBack(const geometry_msgs::PoseArray& msg)
{
    if(!leg_detection_enable) return;

    //if(leg_counter++ < 5) return;
    leg_counter = 0;

    //std::vector<geometry_msgs::Pose> legs_laser; // *** Can be removed ***
    std::vector<PolarPose> legs_polar_base;
    leg_frame_id = "base_footprint";

    if(!msg.poses.empty()){

        if(!legs_polar_base.empty())
            legs_polar_base.clear();

        last_leg_time = ros::Time::now();
        //legs_laser = msg.poses; // *** Can be removed ***

        // FRAME TRANSFORM: /laser to /base_footprint       
        geometry_msgs::PointStamped tmp_laser, tmp_base;
        PolarPose tmp_polar;

        tmp_laser.header = msg.header;

        for(size_t i = 0; i < msg.poses.size(); i++){

            tmp_laser.point = msg.poses.at(i).position;

            if(!transformToBase(tmp_laser, tmp_base)){
                ROS_WARN("Can not transform from laser to base_footprint");
                return;
            }

            tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
            legs_polar_base.push_back(tmp_polar);
        }

        ROS_ASSERT(legs_polar_base.size() == msg.poses.size());

        leg_grid->computeLikelihood(legs_polar_base,
                                    leg_grid->new_data,
                                    sqrt(map_resolution),
                                    sqrt(map_resolution)/4);

        if(!legs_polar_base.empty())
            legs_polar_base.clear();
    }
}

void CartesianGridInterface::initFaces(SensorFOV_t sensor_fov)
{
    try
    {
        face_grid = new CartesianGrid(map_size, map_resolution, sensor_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new faceGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void CartesianGridInterface::faceCallBack(const autonomy_human::human& msg)
{
    if(!torso_detection_enable) return;
   // if(torso_counter++ < 5) return;
    torso_counter = 0;
    std::vector<PolarPose> face_polar_base;
    face_frame_id = "base_footprint";

    if(msg.numFaces)
    {
        last_face_time = ros::Time::now();
        //geometry_msgs::PointStamped tmp_camera, tmp_base;
        PolarPose tmp_polar;
        double theta, x, person_height, person_range;
        double image_width = 640.0;

        x = msg.faceROI.x_offset + msg.faceROI.width/2.0;
        theta = ((image_width/2.0-x)/image_width/2.0)*toRadian(65.0/2.0);
        person_height = msg.faceROI.height;
        person_range = 0.0031*person_height*person_height - 0.8 * person_height + 53.02;
        tmp_polar.angle = theta;
        tmp_polar.range = person_range;

        face_polar_base.push_back(tmp_polar);
        face_grid->computeLikelihood(face_polar_base, face_grid->new_data, sqrt(map_resolution), sqrt(map_resolution));
        face_polar_base.clear();
    }
}

void CartesianGridInterface::initSound(SensorFOV_t sensor_fov)
{
    try
    {
        sound_grid = new CartesianGrid(map_size, map_resolution, sensor_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new sound_Grid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void CartesianGridInterface::soundCallBack(const hark_msgs::HarkSource& msg)
{
    if(!sound_detection_enable) return;
   // if(sound_counter++ < 5) return;
    sound_counter = 0;
    std::vector<hark_msgs::HarkSourceVal> sound_src;
    std::vector<PolarPose> sound_polar_base;
    sound_frame_id = "base_footprint";

    if(!msg.src.empty()){
        last_sound_time = ros::Time::now();
        sound_src = msg.src;
        PolarPose tmp_polar;
        for(size_t i = 0; i < sound_src.size(); i++){
            tmp_polar.angle = toRadian(sound_src[i].azimuth);
            double r = sound_grid->sensor_fov.range.min;
            // FIX THIS PART [????]
//            while(r <= sound_grid->sensor_fov.range.max){
//                tmp_polar.range = r;
//                r += sound_grid->sensor_fov.range.resolution;
//                sound_polar_base.push_back(tmp_polar);
//            }
            sound_grid->computeLikelihood(sound_polar_base, sound_grid->new_data, sqrt(map_resolution), sqrt(map_resolution));
            sound_polar_base.clear();
        }
    }
}


void CartesianGridInterface::initLaser(SensorFOV_t sensor_fov)
{
    try
    {
        laser_grid = new CartesianGrid(map_size, map_resolution, sensor_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new laserGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::laserCallBack(const sensor_msgs::LaserScan& msg)
{
    if(!laser_detection_enable) return;
    laser_frame_id = "base_footprint";

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
                if(msg.ranges.at(i) > fov.range.max || msg.ranges.at(i) < 0.5 || (i%2 == 1))
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

            laser_grid->computeLikelihood(laser_polar_base, laser_grid->new_data, sqrt(map_resolution), sqrt(map_resolution));
            laser_polar_base.clear();
        }
    }
}
void CartesianGridInterface::initHuman()
{
    try
    {
        human_grid = new CartesianGrid(map_size, map_resolution, fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::pointCloudGrid(CartesianGrid* grid, sensor_msgs::PointCloud* pointcloud_grid)
{


    if(!pointcloud_grid->header.frame_id.size()){
        pointcloud_grid->header.frame_id = "base_footprint";
    }else{
        pointcloud_grid->points.clear();
        pointcloud_grid->channels.clear();
    }

    geometry_msgs::Point32 points;
    sensor_msgs::ChannelFloat32 probability_channel, threshold_channel;

    for(size_t i = 0; i < grid->grid_size; i++){

        points.x = grid->map.cell_cart_pos[i].x;
        points.y = grid->map.cell_cart_pos[i].y;
        points.z = grid->data[i];
        probability_channel.values.push_back(grid->data[i]);
        threshold_channel.values.push_back(prior_threshold);
        pointcloud_grid->points.push_back(points);

    }

    probability_channel.name = "probability";
    threshold_channel.name = "threshold";

    pointcloud_grid->channels.push_back(probability_channel);
    pointcloud_grid->channels.push_back(threshold_channel);
}

void CartesianGridInterface::occupancyGrid(CartesianGrid* grid, nav_msgs::OccupancyGrid *occupancy_grid)
{
    if(!occupancy_grid->header.frame_id.size()){
        occupancy_grid->info.height = grid->map.height;
        occupancy_grid->info.width = grid->map.width;
        occupancy_grid->info.origin = grid->map.origin;
        occupancy_grid->info.resolution = grid->map.resolution;
        occupancy_grid->header.frame_id = "base_footprint";
    }else{
        occupancy_grid->data.clear();
    }

    for(size_t i = 0; i < grid->grid_size; i++){
        occupancy_grid->data.push_back(grid->new_data[i]);
    }
}


void CartesianGridInterface::publish()
{
    //LEGS
    if(leg_detection_enable){
//        pointCloudGrid(leg_grid, &leg_pointcloud_grid); //sensor_msgs::PointCloud
//        leg_pointcloud_grid.header.stamp = ros::Time::now();
//        legs_grid_pub.publish(leg_pointcloud_grid);

        occupancyGrid(leg_grid, &leg_occupancy_grid); //nav_msgs::OccupancyGrid
        leg_occupancy_grid.header.stamp = ros::Time::now();
        leg_occupancy_grid_pub.publish(leg_occupancy_grid);
    }

    // FACES
    if(torso_detection_enable){
//        pointCloudGrid(face_grid, &face_pointcloud_grid); //sensor_msgs::PointCloud
//        face_pointcloud_grid.header.stamp = ros::Time::now();
//        face_grid_pub.publish(face_pointcloud_grid);

        occupancyGrid(face_grid, &face_occupancy_grid); //nav_msgs::OccupancyGrid
        face_occupancy_grid.header.stamp = ros::Time::now();
        face_occupancy_grid_pub.publish(face_occupancy_grid);
    }

    // Sound
    if(sound_detection_enable){
//        pointCloudGrid(sound_grid, &sound_pointcloud_grid); //sensor_msgs::PointCloud
//        sound_pointcloud_grid.header.stamp = ros::Time::now();
//        sound_grid_pub.publish(sound_pointcloud_grid);

        occupancyGrid(sound_grid, &sound_occupancy_grid); //nav_msgs::OccupancyGrid
        sound_occupancy_grid.header.stamp = ros::Time::now();
        sound_occupancy_grid_pub.publish(sound_occupancy_grid);
    }

    //LASER
    if(laser_detection_enable){
//        pointCloudGrid(laser_grid, &laser_pointcloud_grid); //sensor_msgs::PointCloud
//        laser_pointcloud_grid.header.stamp = ros::Time::now();
//        laser_grid_pub.publish(laser_pointcloud_grid);

        occupancyGrid(laser_grid, &laser_occupancy_grid); //nav_msgs::OccupancyGrid
        laser_occupancy_grid.header.stamp = ros::Time::now();
        laser_occupancy_grid_pub.publish(laser_occupancy_grid);
    }

    // HUMAN
//    pointCloudGrid(human_grid, &human_pointcloud_grid); // sensor_msgs::PointCloud
//    human_pointcloud_grid.header.stamp = ros::Time::now();
//    human_grid_pub.publish(human_pointcloud_grid);

    occupancyGrid(human_grid, &human_occupancy_grid); //nav_msgs::OccupancyGrid
    human_occupancy_grid.header.stamp = ros::Time::now();
    human_occupancy_grid_pub.publish(human_occupancy_grid);
}


void CartesianGridInterface::spin()
{
    // LEGS
    if(leg_detection_enable){
        diff_leg_time = ros::Time::now() - last_leg_time;
        leg_grid->flag = !(diff_leg_time.toSec() > 2.0);
//        leg_grid->nonlinear_negative_data = true;
//        leg_grid->sensorUpdate(update_rate);
    }

    // FACES
    if(torso_detection_enable){
        diff_face_time = ros::Time::now() - last_face_time;
        face_grid->flag = !(diff_face_time.toSec() > 2.0);
//        face_grid->nonlinear_negative_data = false;
//        face_grid->sensorUpdate(update_rate);
    }

    // SOUND
    if(sound_detection_enable){
        diff_sound_time = ros::Time::now() - last_sound_time;
        sound_grid->flag = !(diff_sound_time.toSec() > 2.0);
//        sound_grid->nonlinear_negative_data = false;
//        sound_grid->sensorUpdate(update_rate);
    }

    // LASER
    if(laser_detection_enable){
        diff_laser_time = ros::Time::now() - last_laser_time;
        laser_grid->flag = !(diff_laser_time.toSec() > 2.0);
        //laser_grid->nonlinear_negative_data = false;
        //laser_grid->sensorUpdate(update_rate);
    }
    // HUMAN
    //human_grid->setProbability(human_grid->new_data, cell_probability.human);
    if(number_of_sensors){
        if(leg_detection_enable){
//            leg_grid->scaleProbability(leg_grid->data,0.5);
//            human_grid->fuse(leg_grid->data);
        }
        if(torso_detection_enable)
            human_grid->fuse(face_grid->data);
        if(sound_detection_enable)
            human_grid->fuse(sound_grid->data);
        if(laser_detection_enable){
            laser_grid->scaleProbability(laser_grid->data,0.2);
            human_grid->fuse(laser_grid->data);
        }
    }

//    for(size_t i = 0; i < human_grid->grid_size; i++){
//        ROS_INFO("----------");
//        ROS_INFO("Cell Number: %lu",i);
//        ROS_INFO("Probability:  %d",human_grid->new_data[i]);
//        ROS_INFO("Cell Position: x: %.2f    y: %.2f", human_grid->map.cell_cart_pos[i].x, human_grid->map.cell_cart_pos[i].y);
//        ROS_INFO("----------");
//    }
    publish();
}

CartesianGridInterface::~CartesianGridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    if(leg_detection_enable) delete leg_grid;
    if(torso_detection_enable) delete face_grid;
    if(sound_detection_enable) delete sound_grid;
    if(laser_detection_enable) delete laser_grid;
    delete human_grid;
    delete tf_listener;
}
