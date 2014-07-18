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
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf",
             fov.angle.min,
             fov.angle.max);

    ros::param::param("~/LikelihoodGrid/grid_range_min",fov.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",fov.range.max, 20.0);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf",
             fov.range.min,
             fov.range.max);

    ros::param::param("~/CartesianLikelihoodGrid/resolution",map_resolution, 0.5);
    map_size = fov.range.max * 2 / map_resolution;

    ROS_INFO("/CartesianLikelihoodGrid size: %d resolution: %.2lf",
             map_size, map_resolution);

    ros::param::param("~/LikelihoodGrid/update_rate",update_rate, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",update_rate);

    ros::param::param("~/LikelihoodGrid/human_cell_probability",cell_probability.human, 0.95);
    ROS_INFO("/LikelihoodGrid/human_cell_probability is set to %.2f",cell_probability.human);

    ros::param::param("~/LikelihoodGrid/free_cell_probability",cell_probability.free, 0.05);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %.2f",cell_probability.free);

    ros::param::param("~/LikelihoodGrid/unknown_cell_probability",cell_probability.unknown, 0.5);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability is set to %.2f",cell_probability.unknown);


    ros::param::param("~/LikelihoodGrid/sensitivity",sensitivity, 1);
    ROS_INFO("/LikelihoodGrid/sensitivity is set to %u",sensitivity);

    ros::param::param("~/loop_rate",loop_rate, 10);
    ROS_INFO("/loop_rate is set to %d",loop_rate);

    ros::param::param("~/motion_model_enable",motion_model_enable, false);
    ROS_INFO("/motion_model_enable to %d",motion_model_enable);

    ros::param::param("~/leg_detection_enable",leg_detection_enable, false);
    ROS_INFO("/leg_detection_enable to %d",leg_detection_enable);

    ros::param::param("~/torso_detection_enable",torso_detection_enable, false);
    ROS_INFO("/torso_detection_enable to %d",torso_detection_enable);

    ros::param::param("~/sound_detection_enable",sound_detection_enable, false);
    ROS_INFO("/sound_detection_enable to %d",sound_detection_enable);

    number_of_sensors = (leg_detection_enable) + (torso_detection_enable)
            + (sound_detection_enable);

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

    last_time_encoder = ros::Time::now();

    if(leg_detection_enable){
        SensorFOV_t legs_fov = fov;
        //legs_fov.range.max = 20.0;
        legs_fov.range.max = (fov.range.max < 20.0 ? fov.range.max:20.0);
        legs_fov.angle.min = toRadian(-120.0);//-2.35619449615;
        legs_fov.angle.max = toRadian(120.0);//2.35619449615;
        initLegs(legs_fov);
        legs_grid_pub = n.advertise<sensor_msgs::PointCloud>("leg_likelihood_grid",10);
        leg_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("leg_occupancy_grid",10);
        predicted_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("predicted_legs",10);
        last_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("last_legs",10);
        current_leg_base_pub = n.advertise<geometry_msgs::PoseArray>("legs_basefootprint",10);
        leg_counter = 0;
    }

    if(torso_detection_enable){
        SensorFOV_t camera_fov = fov;
        camera_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.range.max = 10.00; // TODO: MAKE SURE OF THE REAL FOV
        camera_fov.angle.min = toRadian(-65.0/2);
        camera_fov.angle.max = toRadian(65.0/2);
        initTorso(camera_fov);
        torso_grid_pub = n.advertise<sensor_msgs::PointCloud>("torso_likelihood_grid",10);
        torso_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("torso_occupancy_grid",10);
        torso_counter = 0;
    }

    if(sound_detection_enable){
        SensorFOV_t mic_fov = fov;
        mic_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
        mic_fov.range.max = 10.00;
        mic_fov.angle.min = toRadian(-90.0);
        mic_fov.angle.max = toRadian(90.0);
        initSound(mic_fov);
        sound_grid_pub = n.advertise<sensor_msgs::PointCloud>("sound_likelihood_grid",10);
        sound_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("sound_occupancy_grid",10);
        sound_counter = 0;
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


void CartesianGridInterface::syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg_crtsn,
                                          const nav_msgs::OdometryConstPtr& encoder_msg,
                                          const autonomy_human::humanConstPtr& torso_msg,
                                          const hark_msgs::HarkSourceConstPtr& sound_msg)
{
//    ROS_INFO("I received data");
//    ----------   ENCODER CALLBACK   ----------
    if(motion_model_enable){
        current_time_encoder = ros::Time::now();
        robot_linear_velocity = sqrt( pow(encoder_msg->twist.twist.linear.x,2) + pow(encoder_msg->twist.twist.linear.y,2) );
        robot_angular_velocity = encoder_msg->twist.twist.angular.z/2;
        diff_time = current_time_encoder.toSec() - last_time_encoder.toSec();
        last_time_encoder = current_time_encoder;
    }

//    ----------   LEG DETECTION CALLBACK   ----------
    if(leg_detection_enable){
//        if(!leg_msg_crtsn->poses.empty()) last_leg_time = ros::Time::now();
        geometry_msgs::Pose current_leg_pose_base_crtsn;
        geometry_msgs::PoseArray current_leg_pose_array_base_crtsn;
        current_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
        current_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();
        if(!current_leg_pose_array_base_crtsn.poses.empty()) current_leg_pose_array_base_crtsn.poses.clear();
        leg_grid->flag = true;

        geometry_msgs::PointStamped current_leg_point_laser_crtsn, current_leg_point_base_crtsn;
        PolarPose current_leg_pose_base_polar;
        current_leg_point_laser_crtsn.header = leg_msg_crtsn->header;
        current_leg_point_laser_crtsn.header.frame_id = "laser";

        if(!current_leg_pose_array_base_polar.empty()) current_leg_pose_array_base_polar.clear();

        for(size_t i = 0; i < leg_msg_crtsn->poses.size(); i++){
            current_leg_point_laser_crtsn.point = leg_msg_crtsn->poses.at(i).position;

            if(!transformToBase(current_leg_point_laser_crtsn, current_leg_point_base_crtsn)){
                ROS_WARN("Can not transform from laser to base_footprint");
                return;
            }
            current_leg_pose_base_crtsn.position = current_leg_point_base_crtsn.point;
            current_leg_pose_base_crtsn.position.z = 0.0;
            current_leg_pose_array_base_crtsn.poses.push_back(current_leg_pose_base_crtsn);

            current_leg_pose_base_polar.fromCart(current_leg_point_base_crtsn.point.x, current_leg_point_base_crtsn.point.y);
            current_leg_pose_array_base_polar.push_back(current_leg_pose_base_polar);
        }

        current_leg_base_pub.publish(current_leg_pose_array_base_crtsn);

        ROS_ASSERT(current_leg_pose_array_base_polar.size() == leg_msg_crtsn->poses.size());


        PolarPose predicted_leg_pose_base_polar;
        geometry_msgs::Pose predicted_leg_pose_base_crtsn, last_leg_pose_base_crtsn;
        geometry_msgs::PoseArray last_leg_pose_array_base_crtsn, predicted_leg_pose_array_base_crtsn;
        float diff_range;

        leg_diff_time = ros::Time::now() - last_leg_time;
        last_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
        last_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();

        predicted_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
        predicted_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();

        if(!predicted_leg_pose_array_base_crtsn.poses.empty()) predicted_leg_pose_array_base_crtsn.poses.clear();

        if(!last_leg_pose_array_base_crtsn.poses.empty()) last_leg_pose_array_base_crtsn.poses.clear();

        if(!predicted_leg_pose_array_base_polar.empty())    predicted_leg_pose_array_base_polar.clear();

        for(size_t p = 0 ; p < last_leg_pose_array_base_polar.size(); p++){

            last_leg_pose_array_base_polar.at(p).toCart(last_leg_pose_base_crtsn.position.x,last_leg_pose_base_crtsn.position.y);
            last_leg_pose_base_crtsn.position.z = 0.0;
            last_leg_pose_array_base_crtsn.poses.push_back(last_leg_pose_base_crtsn);

            double leg_angular_velocity;
            double leg_linear_velocity;

            leg_angular_velocity = (robot_linear_velocity * sin(last_leg_pose_array_base_polar.at(p).angle) /
                                    last_leg_pose_array_base_polar.at(p).range) - robot_angular_velocity;
            leg_linear_velocity = -robot_linear_velocity * cos(last_leg_pose_array_base_polar.at(p).angle);

            predicted_leg_pose_base_polar.range = leg_linear_velocity * diff_time + last_leg_pose_array_base_polar.at(p).range;
            predicted_leg_pose_base_polar.angle = leg_angular_velocity * diff_time + last_leg_pose_array_base_polar.at(p).angle;

            predicted_leg_pose_base_crtsn.position.x = predicted_leg_pose_base_polar.range * cos(predicted_leg_pose_base_polar.angle);
            predicted_leg_pose_base_crtsn.position.y = predicted_leg_pose_base_polar.range * sin(predicted_leg_pose_base_polar.angle);
            predicted_leg_pose_base_crtsn.position.z = 0.0;
            predicted_leg_pose_array_base_crtsn.poses.push_back(predicted_leg_pose_base_crtsn);

            diff_range = predicted_leg_pose_base_polar.range - last_leg_pose_array_base_polar.at(p).range;

            predicted_leg_pose_array_base_polar.push_back(predicted_leg_pose_base_polar);

        }

        predicted_leg_base_pub.publish(predicted_leg_pose_array_base_crtsn);
        last_leg_base_pub.publish(last_leg_pose_array_base_crtsn);

        leg_grid->setFreeProbability(leg_grid->likelihood, 0.01);
        leg_grid->setFreeProbability(leg_grid->predicted_likelihood, 0.01);


        leg_grid->computeLikelihood(predicted_leg_pose_array_base_polar,
                                    leg_grid->predicted_likelihood,
                                    0.2,
                                    3.0*M_PI/180.0);
        leg_grid->updateGridProbability(leg_grid->prior,
                                        leg_grid->predicted_likelihood,
                                        leg_grid->predicted_posterior);



        if(leg_diff_time.toSec() < 2.0){
            leg_grid->computeLikelihood(current_leg_pose_array_base_polar,
                                        leg_grid->likelihood,
                                        0.1,
                                        1.0*M_PI/180.0);
        }

        leg_grid->updateGridProbability(leg_grid->predicted_posterior,
                                        leg_grid->likelihood,
                                        leg_grid->posterior);

        leg_grid->prior = leg_grid->posterior;
        last_leg_pose_array_base_polar = current_leg_pose_array_base_polar;
        last_leg_time = ros::Time::now();

        // Publish
        occupancyGrid(leg_grid, &leg_occupancy_grid);
        leg_occupancy_grid.header.stamp = ros::Time::now();
        leg_occupancy_grid_pub.publish(leg_occupancy_grid);
    }

//    ----------   TORSO DETECTION CALLBACK   ----------
    if(torso_detection_enable){

//            double theta, x, person_height, person_range;

//            person_height = msg.faceROI.height;
//            person_range = 0.0031*person_height*person_height - 0.8 * person_height + 53.02;
//            tmp_polar.angle = theta;
//            tmp_polar.range = person_range;

        //*************************************8

        std::vector<PolarPose> torso_pose_array_base_polar;
        torso_frame_id = "base_footprint";
        torso_diff_time = ros::Time::now() - last_torso_time;
        PolarPose torso_pose_base_polar;
        torso_pose_base_polar.range = -1.0;
        torso_pose_base_polar.angle = 2 * M_PI;
        geometry_msgs::Point torso_position_in_image;
        double image_width = 640.0;
        double average_person_height = 1.69; //meter
        double camera_fov = 65.0; // degree

        for(size_t i = 0; i < torso_msg->numFaces; i++){
            last_torso_time = ros::Time::now();
            torso_position_in_image.x = torso_msg->faceROI.x_offset + 0.5 * torso_msg->faceROI.width;
            torso_pose_base_polar.angle = atan((image_width/2.0-torso_position_in_image.x)* tan(toRadian(camera_fov/2.0)) * 2.0 / image_width) ;
            ROS_INFO("angle: %.2f   x offset: %.2f ",torso_pose_base_polar.angle, torso_position_in_image.x);
            torso_pose_array_base_polar.push_back(torso_pose_base_polar);
        }

        torso_grid->setFreeProbability(torso_grid->likelihood, 0.01);

        if(torso_diff_time.toSec() < 1.0){

            torso_grid->computeLikelihood(torso_pose_array_base_polar,
                                          torso_grid->likelihood,
                                          -1.0,
                                          1.0*M_PI/180.0);
        }

        torso_grid->updateGridProbability(torso_grid->prior,
                                          torso_grid->likelihood,
                                          torso_grid->posterior);

        torso_grid->prior = torso_grid->posterior;
        torso_pose_array_base_polar.clear();

        occupancyGrid(torso_grid, &torso_occupancy_grid);
        torso_occupancy_grid.header.stamp = ros::Time::now();
        torso_occupancy_grid_pub.publish(torso_occupancy_grid);
    }

//    ----------   SOUND DETECTION CALLBACK   ----------
    if(sound_detection_enable){

        std::vector<PolarPose> sound_src_array_polar;
        sound_frame_id = "base_footprint";
        sound_diff_time = ros::Time::now() - last_sound_time;
        PolarPose sound_src_polar;
        sound_src_polar.range = -1.0;
        sound_src_polar.angle = 2 * M_PI;

        for(size_t i = 0; i < sound_msg->src.size(); i++){
            if(fabs(sound_msg->src[i].y) > 0.0){
                sound_src_polar.angle = toRadian(sound_msg->src[i].azimuth);
                last_sound_time = ros::Time::now();
                sound_src_array_polar.push_back(sound_src_polar);
            }
        }


        sound_grid->setFreeProbability(sound_grid->likelihood, 0.01);

        if(sound_diff_time.toSec() < 1.0){

            sound_grid->computeLikelihood(sound_src_array_polar,
                                          sound_grid->likelihood,
                                          -1.0,
                                          2.0*M_PI/180.0);
        }

        sound_grid->updateGridProbability(sound_grid->prior,
                                          sound_grid->likelihood,
                                          sound_grid->posterior);
        sound_grid->prior = sound_grid->posterior;
        sound_src_array_polar.clear();

        occupancyGrid(sound_grid, &sound_occupancy_grid);
        sound_occupancy_grid.header.stamp = ros::Time::now();
        sound_occupancy_grid_pub.publish(sound_occupancy_grid);
    }
}

void CartesianGridInterface::initTorso(SensorFOV_t sensor_fov)
{
    try
    {
        torso_grid = new CartesianGrid(map_size, map_resolution, sensor_fov, cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new torsoGrid: bad_alloc caught: " << ba.what() << '\n';
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

        points.x = grid->map.cell_pos_crtsn[i].x;
        points.y = grid->map.cell_pos_crtsn[i].y;
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

//        uint temp_data = (int) cell_probability.unknown * 50;
        uint temp_data = 50;

        if(grid->map.cell_inFOV.at(i)){
            temp_data = (uint) 100 * grid->posterior[i];
        }

        occupancy_grid->data.push_back(temp_data);
    }
}


void CartesianGridInterface::publish()
{

}


void CartesianGridInterface::spin()
{
}

CartesianGridInterface::~CartesianGridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    if(leg_detection_enable) delete leg_grid;
    if(torso_detection_enable) delete torso_grid;
    if(sound_detection_enable) delete sound_grid;
    delete human_grid;
    delete tf_listener;
}
