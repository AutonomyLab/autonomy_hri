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
    ros::param::param("~/LikelihoodGrid/grid_angle_min",FOV.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",FOV.angle.max, M_PI);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf",
             FOV.angle.min,
             FOV.angle.max);

    ros::param::param("~/LikelihoodGrid/grid_range_min",FOV.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",FOV.range.max, 20.0);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf",
             FOV.range.min,
             FOV.range.max);

    ros::param::param("~/CartesianLikelihoodGrid/resolution",MAP_RESOLUTION, 0.5);
    MAP_SIZE = FOV.range.max * 2 / MAP_RESOLUTION;

    ROS_INFO("/CartesianLikelihoodGrid size: %d resolution: %.2lf",
             MAP_SIZE, MAP_RESOLUTION);

    ros::param::param("~/LikelihoodGrid/update_rate",UPDATE_RATE, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",UPDATE_RATE);

    ros::param::param("~/LikelihoodGrid/human_cell_probability",CELL_PROBABILITY.human, 0.95);
    ROS_INFO("/LikelihoodGrid/human_cell_probability is set to %.2f",CELL_PROBABILITY.human);

    ros::param::param("~/LikelihoodGrid/free_cell_probability",CELL_PROBABILITY.free, 0.05);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %.2f",CELL_PROBABILITY.free);

    ros::param::param("~/LikelihoodGrid/unknown_cell_probability",CELL_PROBABILITY.unknown, 0.5);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability is set to %.2f",CELL_PROBABILITY.unknown);

    ros::param::param("~/LikelihoodGrid/target_detection_probability",TARGET_DETETION_PROBABILITY, 0.9);
    ROS_INFO("/LikelihoodGrid/target_detection_probability is set to %.2f",TARGET_DETETION_PROBABILITY);

    ros::param::param("~/LikelihoodGrid/false_positive_probability",FALSE_POSITIVE_PROBABILITY, 0.01);
    ROS_INFO("/LikelihoodGrid/false_positive_probability is set to %.2f",FALSE_POSITIVE_PROBABILITY);


    ros::param::param("~/LikelihoodGrid/sensitivity",SENSITIVITY, 1);
    ROS_INFO("/LikelihoodGrid/sensitivity is set to %u",SENSITIVITY);

    ros::param::param("~/loop_rate",LOOP_RATE, 10);
    ROS_INFO("/loop_rate is set to %d",LOOP_RATE);

    ros::param::param("~/motion_model_enable",MOTION_MODEL_ENABLE, true);
    ROS_INFO("/motion_model_enable to %d",MOTION_MODEL_ENABLE);

    ros::param::param("~/leg_detection_enable",LEG_DETECTION_ENABLE, true);
    ROS_INFO("/leg_detection_enable to %d",LEG_DETECTION_ENABLE);

    ros::param::param("~/torso_detection_enable",TORSO_DETECTION_ENABLE, true);
    ROS_INFO("/torso_detection_enable to %d",TORSO_DETECTION_ENABLE);

    ros::param::param("~/sound_detection_enable",SOUND_DETECTION_ENABLE, true);
    ROS_INFO("/sound_detection_enable to %d",SOUND_DETECTION_ENABLE);

    ros::param::param("~/fuse_multiply",FUSE_MULTIPLY, false);
    ROS_INFO("/fuse_multiply to %d",FUSE_MULTIPLY);

    number_of_sensors = (LEG_DETECTION_ENABLE) + (TORSO_DETECTION_ENABLE)
            + (SOUND_DETECTION_ENABLE);

    ROS_INFO("number_of_sensors is set to %u",number_of_sensors);

    accept_counter = 0;
    reject_counter = 0;

    encoder_last_time = ros::Time::now();
    if(LEG_DETECTION_ENABLE){
        SensorFOV_t LEG_DETECTOR_FOV = FOV;
        LEG_DETECTOR_FOV.range.max = (FOV.range.max < 10.0 ? FOV.range.max : 10.0);
        LEG_DETECTOR_FOV.range.min = FOV.range.min;
        LEG_DETECTOR_FOV.angle.min = toRadian(-120.0);//-2.35619449615;
        LEG_DETECTOR_FOV.angle.max = toRadian(120.0);//2.35619449615;

        initLegGrid(LEG_DETECTOR_FOV);

        ros::param::param("~/LikelihoodGrid/leg_range_stdev",leg_grid->stdev.range, 0.1);
        ROS_INFO("/LikelihoodGrid/leg_range_stdev is set to %.2f",leg_grid->stdev.range);
        ros::param::param("~/LikelihoodGrid/leg_angle_stdev",leg_grid->stdev.angle, 1.0);
        ROS_INFO("/LikelihoodGrid/leg_angle_stdev is set to %.2f",leg_grid->stdev.angle);

        leg_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("leg_occupancy_grid",10);
        predicted_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("predicted_legs",10);
        last_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("last_legs",10);
        current_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("legs_basefootprint",10);
        leg_counter = 0;
    }

    if(TORSO_DETECTION_ENABLE){
        SensorFOV_t TORSO_DETECTOR_FOV = FOV;
        TORSO_DETECTOR_FOV.range.min = 2.0;
        TORSO_DETECTOR_FOV.range.max = 8.0;
        TORSO_DETECTOR_FOV.angle.min = toRadian(-65.0/2);
        TORSO_DETECTOR_FOV.angle.max = toRadian(65.0/2);

        initTorsoGrid(TORSO_DETECTOR_FOV);

        ros::param::param("~/LikelihoodGrid/torso_range_stdev",torso_grid->stdev.range, 0.5);
        ROS_INFO("/LikelihoodGrid/torso_range_stdev is set to %.2f",torso_grid->stdev.range);
        ros::param::param("~/LikelihoodGrid/torso_angle_stdev",torso_grid->stdev.angle, 1.0);
        ROS_INFO("/LikelihoodGrid/torso_angle_stdev is set to %.2f",torso_grid->stdev.angle);
        torso_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("torso_occupancy_grid",10);
        torso_counter = 0;
    }

    if(SOUND_DETECTION_ENABLE){
        SensorFOV_t SOUND_DETECTOR_FOV = FOV;
        SOUND_DETECTOR_FOV.range.min = 1.00;
        SOUND_DETECTOR_FOV.range.max = (FOV.range.max < 10.0 ? FOV.range.max : 10.0);
        SOUND_DETECTOR_FOV.angle.min = toRadian(-90.0);
        SOUND_DETECTOR_FOV.angle.max = toRadian(90.0);

        initSoundGrid(SOUND_DETECTOR_FOV);

        ros::param::param("~/LikelihoodGrid/sound_range_stdev",sound_grid->stdev.range, 0.5);
        ROS_INFO("/LikelihoodGrid/sound_range_stdev is set to %.2f",sound_grid->stdev.range);
        ros::param::param("~/LikelihoodGrid/sound_angle_stdev",sound_grid->stdev.angle, 1.0);
        ROS_INFO("/LikelihoodGrid/sound_angle_stdev is set to %.2f",sound_grid->stdev.angle);
        sound_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("sound_occupancy_grid",10);
        sound_counter = 0;
    }
    initHumanGrid(FOV);
    human_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("human_occupancy_grid",10);
    local_maxima_pub = n.advertise<geometry_msgs::PoseArray>("local_maxima",10);
    max_local_maxima_pub = n.advertise<geometry_msgs::PointStamped>("maximum_probability",10);
    try
    {
        tf_listener = new tf::TransformListener();
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Likelihood Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }

}

bool CartesianGridInterface::transformToBase(geometry_msgs::PointStamped& source_point,
                                             geometry_msgs::PointStamped& target_point,
                                             bool debug)
{
    bool can_transform;
    try
    {
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

bool CartesianGridInterface::transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                                             geometry_msgs::PoseArray& target,
                                             bool debug)
{
    bool can_transform = true;

    geometry_msgs::PointStamped source_point, target_point;
    geometry_msgs::Pose source_pose, target_pose;

    target_point.header = target.header;
    source_point.header = source->header;

    for(size_t i = 0; i < source->poses.size(); i++){
        source_point.point = source->poses.at(i).position;
        try
        {
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

        target_pose.position = target_point.point;
        target_pose.position.z = 0.0;
        target.poses.push_back(target_pose);
    }

    return can_transform;
}


void CartesianGridInterface::initLegGrid(SensorFOV_t _FOV)
{
    try
    {
        leg_grid = new CartesianGrid(MAP_SIZE, _FOV, MAP_RESOLUTION, CELL_PROBABILITY,
                                      TARGET_DETETION_PROBABILITY,
                                      FALSE_POSITIVE_PROBABILITY);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void CartesianGridInterface::initTorsoGrid(SensorFOV_t _FOV)
{
    try
    {
        torso_grid = new CartesianGrid(MAP_SIZE, _FOV, MAP_RESOLUTION, CELL_PROBABILITY,
                                       TARGET_DETETION_PROBABILITY,
                                       FALSE_POSITIVE_PROBABILITY);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new torsoGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::initSoundGrid(SensorFOV_t _FOV)
{
    try
    {
        sound_grid = new CartesianGrid(MAP_SIZE, _FOV, MAP_RESOLUTION, CELL_PROBABILITY, TARGET_DETETION_PROBABILITY,
                                       FALSE_POSITIVE_PROBABILITY);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new sound_Grid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::initHumanGrid(SensorFOV_t _FOV)
{
    try
    {
        human_grid = new CartesianGrid(MAP_SIZE, _FOV, MAP_RESOLUTION, CELL_PROBABILITY,
                                       TARGET_DETETION_PROBABILITY,
                                       FALSE_POSITIVE_PROBABILITY);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CartesianGridInterface::syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg_crtsn,
                                          const nav_msgs::OdometryConstPtr& encoder_msg,
                                          const autonomy_human::raw_detectionsConstPtr torso_msg)
{
//    ----------   ENCODER CALLBACK   ----------
    if(MOTION_MODEL_ENABLE){
        robot_velocity.linear = sqrt( pow(encoder_msg->twist.twist.linear.x,2) + pow(encoder_msg->twist.twist.linear.y,2) );
        robot_velocity.angular = encoder_msg->twist.twist.angular.z;
        encoder_diff_time = ros::Time::now() - encoder_last_time;
        encoder_last_time = ros::Time::now();
    }

//    ----------   LEG DETECTION CALLBACK   ----------
    if(LEG_DETECTION_ENABLE){

        leg_grid->crtsn_array.current.poses.clear();

        if(!transformToBase(leg_msg_crtsn, leg_grid->crtsn_array.current)){
            ROS_WARN("Can not transform from laser to base_footprint");
            leg_grid->crtsn_array.current.poses.clear();
        } else{
            ROS_ASSERT(leg_msg_crtsn->poses.size() == leg_grid->crtsn_array.current.poses.size());
            leg_grid->getPose(leg_grid->crtsn_array.current);
            ROS_ASSERT(leg_grid->polar_array.current.size() == leg_msg_crtsn->poses.size());
        }
//        ROS_INFO("LEG");
        leg_grid->diff_time = encoder_diff_time;
        leg_grid->predict(robot_velocity);

        /* FOR RVIZ */
        leg_grid->crtsn_array.current.header.frame_id = "base_footprint";
        current_leg_base_pub.publish(leg_grid->crtsn_array.current);

        leg_grid->polar2Crtsn(leg_grid->polar_array.predicted, leg_grid->crtsn_array.predicted);
        predicted_leg_base_pub.publish(leg_grid->crtsn_array.predicted);

        leg_grid->polar2Crtsn(leg_grid->polar_array.past, leg_grid->crtsn_array.past);
        last_leg_base_pub.publish(leg_grid->crtsn_array.past);
        /* ******* */

        leg_grid->bayesOccupancyFilter();

        //PUBLISH LEG OCCUPANCY GRID
        occupancyGrid(leg_grid, &leg_occupancy_grid);
        leg_occupancy_grid.header.stamp = ros::Time::now();
        leg_occupancy_grid_pub.publish(leg_occupancy_grid);
    }

    //    ----------   TORSO DETECTION CALLBACK   ----------
    if(TORSO_DETECTION_ENABLE){
//        ROS_INFO("TORSO");
        torso_grid->getPose(torso_msg);
        torso_grid->diff_time = encoder_diff_time;
        torso_grid->predict(robot_velocity);
        torso_grid->bayesOccupancyFilter();

        //PUBLISH TORSO OCCUPANCY GRID
        occupancyGrid(torso_grid, &torso_occupancy_grid);
        torso_occupancy_grid.header.stamp = ros::Time::now();
        torso_occupancy_grid_pub.publish(torso_occupancy_grid);
    }

    human_grid->diff_time = encoder_diff_time;
    human_grid->fuse(sound_grid->posterior, leg_grid->posterior, torso_grid->posterior, FUSE_MULTIPLY);
    human_grid->predict(robot_velocity); // TODO: FIX THIS

    //PUBLISH LOCAL MAXIMA
    human_grid->updateLocalMaximas();
    local_maxima_pub.publish(human_grid->local_maxima_poses);

    //PUBLISH HIGHEST PROBABILITY OF INTEGRATED GRID
    maximum_probability = human_grid->highest_prob_point;
    maximum_probability.header.frame_id = "base_footprint";
    maximum_probability.header.stamp = ros::Time::now();
    max_local_maxima_pub.publish(maximum_probability);

    //PUBLISH INTEGRATED OCCUPANCY GRID
    occupancyGrid(human_grid, &human_occupancy_grid);
    human_occupancy_grid.header.stamp = ros::Time::now();
    human_occupancy_grid_pub.publish(human_occupancy_grid);
}


void CartesianGridInterface::soundCallBack(const hark_msgs::HarkSourceConstPtr &sound_msg)
{
    if(SOUND_DETECTION_ENABLE){
//        ROS_ERROR("SOUND");
        sound_grid->getPose(sound_msg);
        sound_grid->diff_time = ros::Time::now() - sound_grid->last_time;
        sound_grid->last_time = ros::Time::now();
        sound_grid->predict(robot_velocity);
        sound_grid->bayesOccupancyFilter();

        //PUBLISH SOUND OCCUPANCY GRID
        occupancyGrid(sound_grid, &sound_occupancy_grid);
        sound_occupancy_grid.header.stamp = ros::Time::now();
        sound_occupancy_grid_pub.publish(sound_occupancy_grid);
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

        points.x = grid->map.cell[i].cartesian.x;
        points.y = grid->map.cell[i].cartesian.y;
        points.z = grid->posterior[i];
        probability_channel.values.push_back(grid->posterior[i]);
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
        uint cell_prob = (uint) 100 * grid->posterior[i];
        occupancy_grid->data.push_back(cell_prob);
    }
}


void CartesianGridInterface::spin()
{
}

CartesianGridInterface::~CartesianGridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    if(LEG_DETECTION_ENABLE) delete leg_grid;
    if(TORSO_DETECTION_ENABLE) delete torso_grid;
    if(SOUND_DETECTION_ENABLE) delete sound_grid;
    delete human_grid;
    delete tf_listener;
}
