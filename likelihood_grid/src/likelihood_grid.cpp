#include "likelihood_grid.h"


CLikelihoodGrid::CLikelihoodGrid()
{
  ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

CLikelihoodGrid::CLikelihoodGrid(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
  n_(_n),
  tf_listener_(_tf_listener)
{
  ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
  init();
}

void CLikelihoodGrid::init()
{
  ros::param::param("~/LikelihoodGrid/grid_angle_min", FOV_.angle.min, (float) - M_PI);
  ros::param::param("~/LikelihoodGrid/grid_angle_max", FOV_.angle.max, (float) M_PI);

  ros::param::param("~/LikelihoodGrid/grid_range_min", FOV_.range.min, (float) 0.0);
  ros::param::param("~/LikelihoodGrid/grid_range_max", FOV_.range.max, (float) 20.0);

  ros::param::param("~/LikelihoodGrid/resolution", MAP_RESOLUTION_, (float) 0.5);
  MAP_SIZE_ = FOV_.range.max * 2 / MAP_RESOLUTION_;

  ros::param::param("~/LikelihoodGrid/update_rate", UPDATE_RATE_, (float) 0.5);
  ros::param::param("~/LikelihoodGrid/human_cell_probability", CELL_PROBABILITY_.human, (float) 0.95);
  ros::param::param("~/LikelihoodGrid/free_cell_probability", CELL_PROBABILITY_.free, (float) 0.05);
  ros::param::param("~/LikelihoodGrid/unknown_cell_probability", CELL_PROBABILITY_.unknown, (float) 0.25);

  ros::param::param("~/LikelihoodGrid/target_detection_probability", TARGET_DETETION_PROBABILITY_, (float) 0.9);
  ros::param::param("~/LikelihoodGrid/false_positive_probability", FALSE_POSITIVE_PROBABILITY_, (float) 0.01);

  ros::param::param("~/loop_rate", LOOP_RATE_, 10);
  ros::param::param("~/motion_model_enable", MOTION_MODEL_ENABLE_, true);
  ros::param::param("~/leg_detection_enable", LEG_DETECTION_ENABLE_, true);
  ros::param::param("~/torso_detection_enable", TORSO_DETECTION_ENABLE_, true);
  ros::param::param("~/sound_detection_enable", SOUND_DETECTION_ENABLE_, true);
  ros::param::param("~/periodic_gesture_detection_enable", PERIODIC_GESTURE_DETECTION_ENABLE_, false);

  ros::param::param("~/LikelihoodGrid/probability_projection_step", PROJECTION_ANGLE_STEP, 1);


  number_of_sensors_ = (LEG_DETECTION_ENABLE_) + (TORSO_DETECTION_ENABLE_)
                       + (SOUND_DETECTION_ENABLE_) + (PERIODIC_GESTURE_DETECTION_ENABLE_);
  ROS_INFO("number_of_sensors is set to %u", number_of_sensors_);

  encoder_last_time_ = ros::Time::now();

  if (PERIODIC_GESTURE_DETECTION_ENABLE_)
  {
    FOV_.range.max = 30.0;
    SensorFOV_t PERIODIC_DETECTOR_FOV = FOV_;
    PERIODIC_DETECTOR_FOV.range.min = 10.00;
    PERIODIC_DETECTOR_FOV.angle.min = angles::from_degrees(-65.0 / 2);
    PERIODIC_DETECTOR_FOV.angle.max = angles::from_degrees(65.0 / 2);

    initPeriodicGrid(PERIODIC_DETECTOR_FOV);

    ros::param::param("~/LikelihoodGrid/periodic_range_stdev", periodic_grid_->stdev.range, (float) 1.0);
    ros::param::param("~/LikelihoodGrid/periodic_angle_stdev", periodic_grid_->stdev.angle, (float) 1.0);
    periodic_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("periodic_occupancy_grid", 10);
  }

  if (LEG_DETECTION_ENABLE_)
  {

    SensorFOV_t LEG_DETECTOR_FOV = FOV_;

    ros::param::param("~/LikelihoodGrid/leg_range_min", LEG_DETECTOR_FOV.range.min, (float) 1.0);
    ros::param::param("~/LikelihoodGrid/leg_range_max", LEG_DETECTOR_FOV.range.max, (float) 10.0);

    ros::param::param("~/LikelihoodGrid/leg_angle_min", LEG_DETECTOR_FOV.angle.min, (float) angles::from_degrees(-120.0));
    ros::param::param("~/LikelihoodGrid/leg_angle_max", LEG_DETECTOR_FOV.angle.max, (float) angles::from_degrees(120.0));

    ros::param::param("~/LikelihoodGrid/leg_human_cell_probability", LEG_CELL_PROBABILITY_.human, (float) 0.95);
    ros::param::param("~/LikelihoodGrid/leg_free_cell_probability", LEG_CELL_PROBABILITY_.free, (float) 0.05);
    ros::param::param("~/LikelihoodGrid/leg_unknown_cell_probability", LEG_CELL_PROBABILITY_.unknown, (float) 0.25);

    initLegGrid(LEG_DETECTOR_FOV);

    ros::param::param("~/LikelihoodGrid/leg_range_stdev", leg_grid_->stdev.range, (float) 0.1);
    ros::param::param("~/LikelihoodGrid/leg_angle_stdev", leg_grid_->stdev.angle, (float) 0.1);

    leg_grid_->projection_angle_step = PROJECTION_ANGLE_STEP;

    legs_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("leg/occupancy_grid", 10);
    predicted_leg_base_pub_ = n_.advertise<geometry_msgs::PoseArray>("predicted_legs", 10);
    last_leg_base_pub_ = n_.advertise<geometry_msgs::PoseArray>("last_legs", 10);
    current_leg_base_pub_ = n_.advertise<geometry_msgs::PoseArray>("legs_basefootprint", 10);
  }

  if (TORSO_DETECTION_ENABLE_)
  {
    SensorFOV_t TORSO_DETECTOR_FOV = FOV_;

    ros::param::param("~/LikelihoodGrid/torso_range_min", TORSO_DETECTOR_FOV.range.min, (float)2.0);
    ros::param::param("~/LikelihoodGrid/torso_range_max", TORSO_DETECTOR_FOV.range.max, (float)8.0);

    ros::param::param("~/LikelihoodGrid/torso_angle_min", TORSO_DETECTOR_FOV.angle.min, (float)angles::from_degrees(-70.0 / 2));
    ros::param::param("~/LikelihoodGrid/torso_angle_max", TORSO_DETECTOR_FOV.angle.max, (float)angles::from_degrees(70.0 / 2));

    ros::param::param("~/LikelihoodGrid/torso_human_cell_probability", TORSO_CELL_PROBABILITY_.human, (float)0.95);
    ros::param::param("~/LikelihoodGrid/torso_free_cell_probability", TORSO_CELL_PROBABILITY_.free, (float)0.05);
    ros::param::param("~/LikelihoodGrid/torso_unknown_cell_probability", TORSO_CELL_PROBABILITY_.unknown, (float)0.25);

    initTorsoGrid(TORSO_DETECTOR_FOV);

    ros::param::param("~/LikelihoodGrid/torso_range_stdev", torso_grid_->stdev.range, (float)0.2);
    ros::param::param("~/LikelihoodGrid/torso_angle_stdev", torso_grid_->stdev.angle, (float)1.0);
    torso_grid_->projection_angle_step = PROJECTION_ANGLE_STEP;

    torso_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("torso/occupancy_grid", 10);
  }

  if (SOUND_DETECTION_ENABLE_)
  {
    SensorFOV_t SOUND_DETECTOR_FOV = FOV_;

    ros::param::param("~/LikelihoodGrid/sound_range_min", SOUND_DETECTOR_FOV.range.min, (float)1.0);
    ros::param::param("~/LikelihoodGrid/sound_range_max", SOUND_DETECTOR_FOV.range.max, (float)10.0);

    ros::param::param("~/LikelihoodGrid/sound_angle_min", SOUND_DETECTOR_FOV.angle.min, (float)angles::from_degrees(-90.0));
    ros::param::param("~/LikelihoodGrid/sound_angle_max", SOUND_DETECTOR_FOV.angle.max, (float)angles::from_degrees(90.0));

    ros::param::param("~/LikelihoodGrid/sound_human_cell_probability", SOUND_CELL_PROBABILITY_.human, (float)0.95);
    ros::param::param("~/LikelihoodGrid/sound_free_cell_probability", SOUND_CELL_PROBABILITY_.free, (float)0.05);
    ros::param::param("~/LikelihoodGrid/sound_unknown_cell_probability", SOUND_CELL_PROBABILITY_.unknown, (float)0.25);

    initSoundGrid(SOUND_DETECTOR_FOV);

    ros::param::param("~/LikelihoodGrid/sound_range_stdev", sound_grid_->stdev.range, (float) 0.5);
    ros::param::param("~/LikelihoodGrid/sound_angle_stdev", sound_grid_->stdev.angle, (float) 5.0);
    sound_grid_->projection_angle_step = PROJECTION_ANGLE_STEP;
    sound_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("sound/occupancy_grid", 10);
  }

  initHumanGrid(FOV_);
  human_grid_->projection_angle_step = PROJECTION_ANGLE_STEP;
  human_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("human/occupancy_grid", 10);
  local_maxima_pub_ = n_.advertise<geometry_msgs::PoseArray>("local_maxima", 10);
  max_prob_pub_ = n_.advertise<geometry_msgs::PointStamped>("maximum_probability", 10);

  try
  {
    tf_listener_ = new tf::TransformListener();
  }
  catch (std::bad_alloc& ba)
  {
    std::cerr << "In Likelihood Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
  }
}

bool CLikelihoodGrid::transformToBase(geometry_msgs::PointStamped& source_point,
                                      geometry_msgs::PointStamped& target_point,
                                      bool debug)
{
  bool can_transform;
  try
  {
    tf_listener_->transformPoint("base_footprint", source_point, target_point);
    if (debug)
    {
      tf::StampedTransform _t;
      tf_listener_->lookupTransform("base_footprint", source_point.header.frame_id, ros::Time(0), _t);
      ROS_INFO("From %s to bfp: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
               source_point.header.frame_id.c_str(),
               _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
               _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(), _t.getRotation().getW());
    }
    can_transform = true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_footprint\": %s", source_point.header.frame_id.c_str(), ex.what());
    can_transform = false;
  }
  return can_transform;
}

bool CLikelihoodGrid::transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                                      geometry_msgs::PoseArray& target,
                                      bool debug)
{
  bool can_transform = true;

  geometry_msgs::PointStamped source_point, target_point;
  geometry_msgs::Pose source_pose, target_pose;

  target_point.header = target.header;
  source_point.header = source->header;

  for (size_t i = 0; i < source->poses.size(); i++)
  {
    source_point.point = source->poses.at(i).position;
    try
    {
      tf_listener_->transformPoint("base_footprint", source_point, target_point);
      if (debug)
      {
        tf::StampedTransform _t;
        tf_listener_->lookupTransform("base_footprint", source_point.header.frame_id, ros::Time(0), _t);
        ROS_INFO("From %s to bfp: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
                 source_point.header.frame_id.c_str(),
                 _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
                 _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(), _t.getRotation().getW());
      }
      can_transform = true;
    }
    catch (tf::TransformException& ex)
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


void CLikelihoodGrid::initLegGrid(SensorFOV_t _FOV)
{

  try
  {

    leg_grid_ = new CGrid(MAP_SIZE_, _FOV, MAP_RESOLUTION_, LEG_CELL_PROBABILITY_,
                          TARGET_DETETION_PROBABILITY_, FALSE_POSITIVE_PROBABILITY_,
                          PROJECTION_ANGLE_STEP);

  }
  catch (std::bad_alloc& ba)
  {

    std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
  }
}

void CLikelihoodGrid::initTorsoGrid(SensorFOV_t _FOV)
{
  try
  {
    torso_grid_ = new CGrid(MAP_SIZE_, _FOV, MAP_RESOLUTION_, TORSO_CELL_PROBABILITY_,
                            TARGET_DETETION_PROBABILITY_,
                            FALSE_POSITIVE_PROBABILITY_,
                            PROJECTION_ANGLE_STEP);
  }
  catch (std::bad_alloc& ba)
  {
    std::cerr << "In new torsoGrid: bad_alloc caught: " << ba.what() << '\n';
  }
}


void CLikelihoodGrid::initSoundGrid(SensorFOV_t _FOV)
{
  try
  {
    sound_grid_ = new CGrid(MAP_SIZE_, _FOV, MAP_RESOLUTION_, SOUND_CELL_PROBABILITY_, TARGET_DETETION_PROBABILITY_,
                            FALSE_POSITIVE_PROBABILITY_, PROJECTION_ANGLE_STEP);
  }
  catch (std::bad_alloc& ba)
  {
    std::cerr << "In new sound_Grid: bad_alloc caught: " << ba.what() << '\n';
  }
}

void CLikelihoodGrid::initPeriodicGrid(SensorFOV_t _FOV)
{
  try
  {
    periodic_grid_ = new CGrid(MAP_SIZE_, _FOV, MAP_RESOLUTION_, CELL_PROBABILITY_, TARGET_DETETION_PROBABILITY_,
                               FALSE_POSITIVE_PROBABILITY_,
                               PROJECTION_ANGLE_STEP);
  }
  catch (std::bad_alloc& ba)
  {
    std::cerr << "In new periodic_grid: bad_alloc caught: " << ba.what() << '\n';
  }
}


void CLikelihoodGrid::initHumanGrid(SensorFOV_t _FOV)
{
  try
  {
    human_grid_ = new CGrid(MAP_SIZE_, _FOV, MAP_RESOLUTION_, CELL_PROBABILITY_,
                            TARGET_DETETION_PROBABILITY_,
                            FALSE_POSITIVE_PROBABILITY_,
                            PROJECTION_ANGLE_STEP);
  }
  catch (std::bad_alloc& ba)
  {
    std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
  }
}

void CLikelihoodGrid::syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg_crtsn,
                                   const nav_msgs::OdometryConstPtr& encoder_msg)
{
//    ----------   ENCODER CALLBACK   ----------
  if (MOTION_MODEL_ENABLE_)
  {
    robot_velocity_.linear = sqrt(pow(encoder_msg->twist.twist.linear.x, 2) + pow(encoder_msg->twist.twist.linear.y, 2));
    robot_velocity_.lin.x = encoder_msg->twist.twist.linear.x;
    robot_velocity_.lin.y = encoder_msg->twist.twist.linear.y;
    robot_velocity_.angular = encoder_msg->twist.twist.angular.z;
    encoder_diff_time_ = ros::Time::now() - encoder_last_time_;
    encoder_last_time_ = ros::Time::now();
  }

//    ----------   LEG DETECTION CALLBACK   ----------
  if (LEG_DETECTION_ENABLE_)
  {

    leg_grid_->crtsn_array.current.poses.clear();

    if (!transformToBase(leg_msg_crtsn, leg_grid_->crtsn_array.current))
    {
      ROS_WARN("Can not transform from laser to base_footprint");
      leg_grid_->crtsn_array.current.poses.clear();
    }
    else
    {
      ROS_ASSERT(leg_msg_crtsn->poses.size() == leg_grid_->crtsn_array.current.poses.size());
      leg_grid_->getPose(leg_grid_->crtsn_array.current);
      ROS_ASSERT(leg_grid_->polar_array.current.size() == leg_msg_crtsn->poses.size());
    }
  }
}

void CLikelihoodGrid::torsoCallBack(const autonomy_human::raw_detectionsConstPtr &torso_msg)
{
  if (TORSO_DETECTION_ENABLE_)
    torso_grid_->getPose(torso_msg);
}


void CLikelihoodGrid::soundCallBack(const hark_msgs::HarkSourceConstPtr &sound_msg)
{
  if (SOUND_DETECTION_ENABLE_)
    sound_grid_->getPose(sound_msg);
}

void CLikelihoodGrid::periodicCallBack(const autonomy_human::raw_detectionsConstPtr &periodic_msg)
{
  if (PERIODIC_GESTURE_DETECTION_ENABLE_)
    periodic_grid_->getPose(periodic_msg);
}

void CLikelihoodGrid::occupancyGrid(CGrid* grid, nav_msgs::OccupancyGrid *occupancy_grid)
{
  if (!occupancy_grid->header.frame_id.size())
  {
    occupancy_grid->info.height = grid->map.height;
    occupancy_grid->info.width = grid->map.width;
    occupancy_grid->info.origin = grid->map.origin;
    occupancy_grid->info.resolution = grid->map.resolution;
    occupancy_grid->header.frame_id = "base_footprint";
  }
  else
  {
    occupancy_grid->data.clear();
  }

  for (size_t i = 0; i < grid->grid_size; i++)
  {
    uint cell_prob = (uint) 100 * grid->posterior[i];
    occupancy_grid->data.push_back(cell_prob);
  }
}


void CLikelihoodGrid::spin()
{
  if (LEG_DETECTION_ENABLE_)
  {
    leg_grid_->diff_time = ros::Time::now() - last_time_;

    leg_grid_->predict(robot_velocity_);

    /* FOR RVIZ */
    leg_grid_->crtsn_array.current.header.frame_id = "base_footprint";
    current_leg_base_pub_.publish(leg_grid_->crtsn_array.current);

    leg_grid_->polar2Crtsn(leg_grid_->polar_array.predicted, leg_grid_->crtsn_array.predicted);
    predicted_leg_base_pub_.publish(leg_grid_->crtsn_array.predicted);

    leg_grid_->polar2Crtsn(leg_grid_->polar_array.past, leg_grid_->crtsn_array.past);
    last_leg_base_pub_.publish(leg_grid_->crtsn_array.past);
    /* ******* */

    leg_grid_->bayesOccupancyFilter();


    //PUBLISH LEG OCCUPANCY GRID
    occupancyGrid(leg_grid_, &leg_occupancy_grid_);
    leg_occupancy_grid_.header.stamp = ros::Time::now();
    legs_grid_pub_.publish(leg_occupancy_grid_);
  }
  //---------------------------------------------

  if (TORSO_DETECTION_ENABLE_)
  {
    torso_grid_->diff_time = ros::Time::now() - last_time_;;
    torso_grid_->predict(robot_velocity_);
    torso_grid_->bayesOccupancyFilter();

    //PUBLISH TORSO OCCUPANCY GRID
    occupancyGrid(torso_grid_, &torso_occupancy_grid_);
    torso_occupancy_grid_.header.stamp = ros::Time::now();
    torso_grid_pub_.publish(torso_occupancy_grid_);
  }
  //---------------------------------------------

  if (SOUND_DETECTION_ENABLE_)
  {
    sound_grid_->diff_time = ros::Time::now() - last_time_;;
    sound_grid_->predict(robot_velocity_);
    sound_grid_->bayesOccupancyFilter();

    //PUBLISH SOUND OCCUPANCY GRID
    occupancyGrid(sound_grid_, &sound_occupancy_grid_);
    sound_occupancy_grid_.header.stamp = ros::Time::now();
    sound_grid_pub_.publish(sound_occupancy_grid_);

  }

  //---------------------------------------------

  human_grid_->diff_time = ros::Time::now() - last_time_;

  //TODO: Should not depend on three vectors
  human_grid_->fuse(sound_grid_->posterior, leg_grid_->posterior, torso_grid_->posterior,
                    FUSE_MULTIPLY_);
  human_grid_->predict(robot_velocity_); // TODO: FIX THIS

  //PUBLISH LOCAL MAXIMA
  human_grid_->updateLocalMaximas();
  local_maxima_pub_.publish(human_grid_->local_maxima_poses);

  //PUBLISH HIGHEST PROBABILITY OF INTEGRATED GRID
  maximum_probability_ = human_grid_->highest_prob_point;
  maximum_probability_.header.frame_id = "base_footprint";
  maximum_probability_.header.stamp = ros::Time::now();
  max_prob_pub_.publish(maximum_probability_);

  //PUBLISH INTEGRATED OCCUPANCY GRID
  occupancyGrid(human_grid_, &human_occupancy_grid_);
  human_occupancy_grid_.header.stamp = ros::Time::now();
  human_grid_pub_.publish(human_occupancy_grid_);
  last_time_ = ros::Time::now();
}

CLikelihoodGrid::~CLikelihoodGrid()
{
  ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
  if (LEG_DETECTION_ENABLE_) delete leg_grid_;
  if (TORSO_DETECTION_ENABLE_) delete torso_grid_;
  if (SOUND_DETECTION_ENABLE_) delete sound_grid_;
  if (PERIODIC_GESTURE_DETECTION_ENABLE_) delete periodic_grid_;
  delete human_grid_;
  delete tf_listener_;
}
