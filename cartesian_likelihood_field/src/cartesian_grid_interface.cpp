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
        initFaces(camera_fov);
        face_grid_pub = n.advertise<sensor_msgs::PointCloud>("face_likelihood_grid",10);
        face_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("face_occupancy_grid",10);
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

    if(laser_detection_enable){
        SensorFOV_t laser_fov = fov;
        laser_fov.range.max = 20.0;
        laser_fov.angle.min = toRadian(-120.0);//-2.35619449615;
        laser_fov.angle.max = toRadian(120.0);//2.35619449615;
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

void CartesianGridInterface::encoderCallBack(const nav_msgs::Odometry& encoder_msg)
{
//    ROS_INFO("Received encoder msgs ***%.6f", encoder_msg.header.stamp.toSec());
////    if(!motion_model_enable) return;
//    current_time_encoder = ros::Time::now();
//    velocity = encoder_msg.twist.twist;
//    diff_time = 0.10;
////    diff_time = current_time_encoder.toSec() - last_time_encoder.toSec();
//    diff_pose.position.x = -velocity.linear.x * diff_time; // dx = vx * dt
//    diff_pose.position.y = -velocity.linear.y * diff_time;
//    diff_pose.orientation.z = -velocity.angular.z/2 * diff_time;
//    last_time_encoder = current_time_encoder;
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


void CartesianGridInterface::legCallBack(const geometry_msgs::PoseArray& leg_msg)
{
//    ROS_INFO("Received leg msgs %.6f", leg_msg.header.stamp.toSec());
//    if(!leg_detection_enable) return;
////    if(leg_counter++ < 10) return;
////    leg_counter = 0;
//    leg_frame_id = "base_footprint";
//    leg_grid->flag = true;

//    if(!current_leg_poses.empty())
//        current_leg_poses.clear();

//    geometry_msgs::PointStamped tmp_laser, tmp_base;
//    PolarPose tmp_polar;
//    last_leg_time = ros::Time::now();
//    tmp_laser.header = leg_msg.header;
//    tmp_laser.header.frame_id = "laser";

//    for(size_t i = 0; i < leg_msg.poses.size(); i++){
//        tmp_laser.point = leg_msg.poses.at(i).position;

//        if(!transformToBase(tmp_laser, tmp_base)){
//            ROS_WARN("Can not transform from laser to base_footprint");
//            return;
//        }
//        tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
//        current_leg_poses.push_back(tmp_polar);
    }

//    ROS_ASSERT(current_leg_poses.size() == leg_msg.poses.size());
//    last_leg_poses = current_leg_poses;
//}

void CartesianGridInterface::syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg_crtsn, const nav_msgs::OdometryConstPtr& encoder_msg)
{
//    ----------   ENCODER CALLBACK   ----------

//    if(!motion_model_enable) return;
    geometry_msgs::Point test_input_point, test_output_point;
    current_time_encoder = ros::Time::now();
    robot_linear_velocity = sqrt( pow(encoder_msg->twist.twist.linear.x,2) + pow(encoder_msg->twist.twist.linear.y,2) );
    robot_angular_velocity = encoder_msg->twist.twist.angular.z/2;
    diff_time = current_time_encoder.toSec() - last_time_encoder.toSec();   // dt = t1 - t0
//    ROS_INFO("diff time: %.4f",diff_time);

//    velocity.linear is robot's linear velocity in (m/s) and velocity.angular is robot's angular velocity in (Radian/sec)
//    diff_pose_crtsn.position.x = -robot_linear_velocity.linear.x * diff_time;  // dx = -vx * dt (m)
//    diff_pose_crtsn.position.y = -robot_linear_velocity.linear.y * diff_time;  // dy = -vy * dt (m)
//    diff_pose_crtsn.position.z = 0.0;
//    diff_pose_crtsn.orientation.z = -robot_linear_velocity.angular.z/2 * diff_time;    // dtheta = -vtetha * dt (radian)
    last_time_encoder = current_time_encoder;

    // TEST ***********************
//    test_input_point.x = 0.0;
//    test_input_point.y = 0.0;
//    test_input_point.z = 0.0;

//    test_output_point.x = -encoder_msg->pose.pose.x * diff_time + test_input_point.x;
//    test_output_point.y = -encoder_msg->pose.pose.y * diff_time + test_input_point.y;
//    test_output_point.z = 0.0 + test_input_point.z;

//    ROS_INFO("From: x: %.2f     y: %.2f     z: %.2f",test_input_point.x, test_input_point.y, test_input_point.z );
//    ROS_INFO("To:   x: %.2f     y: %.2f     z: %.2f",test_output_point.x, test_output_point.y, test_output_point.z);



//    ROS_INFO("      dx: %.4f    dy:%.4f     dtheta: %.4f", diff_pose_crtsn.position.x, diff_pose_crtsn.position.y, diff_pose_crtsn.orientation.z);

//    ------------------------------------------

//    ----------   LEG DETECTION CALLBACK   ----------

//    ROS_INFO("Received leg msgs %.6f", leg_msg->header.stamp.toSec());
    if(!leg_detection_enable) return;
    geometry_msgs::Pose current_leg_pose_base_crtsn;
    geometry_msgs::PoseArray current_leg_pose_array_base_crtsn;
    current_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
    current_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();
    if(!current_leg_pose_array_base_crtsn.poses.empty()) current_leg_pose_array_base_crtsn.poses.clear();

    leg_frame_id = "base_footprint";
    leg_grid->flag = true;

    geometry_msgs::PointStamped current_leg_point_laser_crtsn, current_leg_point_base_crtsn;
    PolarPose current_leg_pose_base_polar;
    last_leg_time = ros::Time::now();
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
//    last_leg_poses = current_leg_poses;

//    ------------------------------------------------
    ROS_INFO("*******************************************************************");

    PolarPose predicted_leg_pose_base_polar;
    geometry_msgs::Pose predicted_leg_pose_base_crtsn, last_leg_pose_base_crtsn;
    geometry_msgs::PoseArray last_leg_pose_array_base_crtsn, predicted_leg_pose_array_base_crtsn;
    float diff_range;

    leg_diff_time = ros::Time::now() - last_leg_time;

    last_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
    last_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();

    predicted_leg_pose_array_base_crtsn.header.frame_id = "base_footprint";
    predicted_leg_pose_array_base_crtsn.header.stamp = ros::Time::now();

    if(!predicted_leg_pose_array_base_crtsn.poses.empty()) predicted_leg_pose_array_base_crtsn.poses.clear(); // Pl = {}
    if(!last_leg_pose_array_base_crtsn.poses.empty()) last_leg_pose_array_base_crtsn.poses.clear();

    if(!predicted_leg_pose_array_base_polar.empty())    predicted_leg_pose_array_base_polar.clear();

    for(size_t p = 0 ; p < last_leg_pose_array_base_polar.size(); p++){ // {l0_0, ..., l0_P}

        last_leg_pose_array_base_polar.at(p).toCart(last_leg_pose_base_crtsn.position.x,last_leg_pose_base_crtsn.position.y);   // (x0,y0)_p
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

//        ROS_INFO("predicted leg pose: range: %.4f   angle: %.4f", predicted_leg_pose_base_polar.range,
//                 predicted_leg_pose_base_polar.angle);

//        ROS_INFO("diff (predicted - last): range: %.4f    angle: %.4f", diff_range, diff_pose_crtsn.orientation.z);
//        predicted_leg_pose_array_base_polar.push_back(predicted_leg_pose_base_polar);

    }
    predicted_leg_base_pub.publish(predicted_leg_pose_array_base_crtsn);
    last_leg_base_pub.publish(last_leg_pose_array_base_crtsn);

    for(size_t pi = 0; pi < predicted_leg_pose_array_base_polar.size(); pi++){
        for(size_t ci = 0; ci < current_leg_pose_array_base_polar.size(); ci++){
//            ROS_INFO("Diff (Current %lu - Prediction %lu)  range: %.4f   angle: %.4f ", ci, pi,
//                     current_leg_pose_array_base_polar.at(ci).range - predicted_leg_pose_array_base_polar.at(pi).range,
//                     current_leg_pose_array_base_polar.at(ci).angle - predicted_leg_pose_array_base_polar.at(pi).angle);
        }
    }

    for(size_t li = 0; li < last_leg_pose_array_base_polar.size(); li++){
        for(size_t ci = 0; ci < current_leg_pose_array_base_polar.size(); ci++){
//            ROS_ERROR("Diff (Current %lu - last %lu)  range: %.4f   angle: %.4f ", ci, li,
//                     current_leg_pose_array_base_polar.at(ci).range - last_leg_pose_array_base_polar.at(li).range,
//                     current_leg_pose_array_base_polar.at(ci).angle - last_leg_pose_array_base_polar.at(li).angle);
        }
    }

    ROS_INFO("-----------------------------------------------------------------------------");

    leg_grid->setFreeProbability(leg_grid->likelihood, 0.01);
    leg_grid->setFreeProbability(leg_grid->prediction, 0.01);

    leg_grid->computeLikelihood(predicted_leg_pose_array_base_polar,
                                leg_grid->prediction,
                                0.2,
                                toRadian(60.0*M_PI/180.0));
    leg_grid->updateGridProbability(leg_grid->prior,
                                    leg_grid->prediction,
                                    leg_grid->predicted_posterior);

    if(leg_diff_time.toSec() < 2.0){

        leg_grid->computeLikelihood(current_leg_pose_array_base_polar,
                                    leg_grid->likelihood,
                                    0.1,
                                    toRadian(45.0*M_PI/180.0));
    }

    leg_grid->updateGridProbability(leg_grid->predicted_posterior,
                                    leg_grid->likelihood,
                                    leg_grid->posterior);
    leg_grid->prior = leg_grid->posterior;
    last_leg_pose_array_base_polar = current_leg_pose_array_base_polar;

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
        face_grid->computeLikelihood(face_polar_base, face_grid->likelihood, sqrt(map_resolution), sqrt(map_resolution));
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
            sound_grid->computeLikelihood(sound_polar_base, sound_grid->likelihood, sqrt(map_resolution), sqrt(map_resolution));
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

            laser_grid->computeLikelihood(laser_polar_base, laser_grid->likelihood, sqrt(map_resolution), sqrt(map_resolution));
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

        uint temp_data = 50;

        if(grid->map.cell_inFOV.at(i)){
            temp_data = (uint) 100 * grid->posterior[i];
        }

        occupancy_grid->data.push_back(temp_data);
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

//        PolarPose polar_pose_tmp;
//        geometry_msgs::Pose pose_new, pose_old;
//        geometry_msgs::PoseArray pr_legs, last_legs;
//        float x_old, y_old, x_new, y_new, diff_range;
//        leg_diff_time = ros::Time::now() - last_leg_time;
//        pr_legs.header.frame_id = "base_footprint";
//        pr_legs.header.stamp = ros::Time::now();

//        last_legs.header.frame_id = "base_footprint";
//        last_legs.header.stamp = ros::Time::now();

//        if(!predict_leg_poses.empty()) predict_leg_poses.clear(); // Pl = {}
//        if(!pr_legs.poses.empty()) pr_legs.poses.clear(); // Pl = {}
//        if(!last_legs.poses.empty()) last_legs.poses.clear();

//        for(size_t p = 0 ; p < last_leg_poses.size(); p++){ // {l0_0, ..., l0_P}

//            last_leg_poses.at(p).toCart(x_old,y_old);   // (x0,y0)_p
//            x_new = x_old + diff_pose.position.x;   // x1_p = x0_p + dx
//            y_new = y_old + diff_pose.position.y;   // y1_p = y0_p + dy
//            pose_new.position.x = x_new;
//            pose_new.position.y = y_new;
//            pose_new.position.z = 0.0;
//            pr_legs.poses.push_back(pose_new);

//            pose_old.position.x = x_old;
//            pose_old.position.y = y_old;
//            pose_old.position.z = 0.0;
//            last_legs.poses.push_back(pose_old);
////            ROS_INFO("X1: %.4f      X0 = %.4f", x_new, x_old);
////            ROS_INFO("Y1: %.4f      Y0 = %.4f", y_new, y_old);
//            polar_pose_tmp.range = sqrt(x_new * x_new + y_new * y_new);    // r1 = sqrt( x1_p*x1_p + y1_p*y1_p )
//            polar_pose_tmp.angle = last_leg_poses.at(p).angle + diff_pose.orientation.z;   // theta1_p = theta0_p + dtheta
//            diff_range = polar_pose_tmp.range - last_leg_poses.at(p).range;

////            diff_range = (x_old*diff_pose.position.x + y_old*diff_pose.position.y)/last_leg_poses.at(p).range;
////            tmp.angle = last_leg_poses.at(p).angle + diff_pose.orientation.z;
////            tmp.range = last_leg_poses.at(p).range + diff_range;


//            if( abs(diff_range) > 1.5 || abs(diff_pose.orientation.z) > 4.0){
//                ROS_INFO("difference between two legs are too large: diff range: %.4f   diff angle: %.4f", diff_range, diff_pose.orientation.z);

//            } else{
//                ROS_ERROR("diff: (Prediction - last) range: %.4f   angle: %.4f   number of legs: %lu", diff_range, diff_pose.orientation.z, last_leg_poses.size());
//                predict_leg_poses.push_back(polar_pose_tmp);
//            }
//        }
//        predicted_leg_pub.publish(pr_legs);
//        last_leg_pub.publish(last_legs);

//        for(size_t pi = 0; pi < predict_leg_poses.size(); pi++){
//            for(size_t ci = 0; ci < current_leg_poses.size(); ci++){
//                ROS_INFO("Diff (Current %lu - Prediction %lu)  range: %.4f   angle: %.4f ", ci, pi,
//                         current_leg_poses.at(ci).range - predict_leg_poses.at(pi).range,
//                         current_leg_poses.at(ci).angle - predict_leg_poses.at(pi).angle);
//            }
//        }

//        for(size_t li = 0; li < last_leg_poses.size(); li++){
//            for(size_t ci = 0; ci < current_leg_poses.size(); ci++){
//                ROS_ERROR("Diff (Current %lu - last %lu)  range: %.4f   angle: %.4f ", ci, li,
//                         current_leg_poses.at(ci).range - last_leg_poses.at(li).range,
//                         current_leg_poses.at(ci).angle - last_leg_poses.at(li).angle);
//            }
//        }

//        ROS_INFO("-----------------------------------------------------------------------------");

//        leg_grid->setFreeProbability(leg_grid->likelihood, 0.01);
//        leg_grid->setFreeProbability(leg_grid->prediction, 0.01);

//        leg_grid->computeLikelihood(predict_leg_poses,
//                                    leg_grid->prediction,
//                                    0.2,
//                                    toRadian(60.0*M_PI/180.0));
//        leg_grid->updateGridProbability(leg_grid->prior,
//                                        leg_grid->prediction,
//                                        leg_grid->predicted_posterior);

//        if(leg_diff_time.toSec() < 2.0){

//            leg_grid->computeLikelihood(current_leg_poses,
//                                        leg_grid->likelihood,
//                                        0.1,
//                                        toRadian(45.0*M_PI/180.0));
//        }

//        leg_grid->updateGridProbability(leg_grid->predicted_posterior,
//                                        leg_grid->likelihood,
//                                        leg_grid->posterior);
//        leg_grid->prior = leg_grid->posterior;
//        last_leg_poses = current_leg_poses;
    }

    // FACES
    if(torso_detection_enable){
        diff_face_time = ros::Time::now() - last_face_time;
        face_grid->flag = !(diff_face_time.toSec() > 2.0);
//        face_grid->nonlinear_negative_data = false;
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
    //human_grid->setProbability(human_grid->likelihood, cell_probability.human);
    if(number_of_sensors){
        if(leg_detection_enable){
//            leg_grid->scaleProbability(leg_grid->posterior,0.5);
//            human_grid->fuse(leg_grid->posterior);
        }
        if(torso_detection_enable)
//            human_grid->fuse(face_grid->posterior);
        if(sound_detection_enable)
//            human_grid->fuse(sound_grid->posterior);
        if(laser_detection_enable){
//            laser_grid->scaleProbability(laser_grid->posterior,0.2);
//            human_grid->fuse(laser_grid->posterior);
        }
    }
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
