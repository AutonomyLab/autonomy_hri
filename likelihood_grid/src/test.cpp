#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <likelihood_grid/TestConfig.h>
#include <nav_msgs/Odometry.h>
#include <autonomy_human/raw_detections.h>
#include <hark_msgs/HarkSource.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::PoseArray legs;
nav_msgs::Odometry encoder;
autonomy_human::raw_detections torso;
hark_msgs::HarkSource sound;
std_msgs::Float32MultiArray weights;


bool show_sound;

void callback(likelihood_grid::TestConfig &config, uint32_t level)
{

  if (!legs.poses.empty()) legs.poses.clear();
  if (!torso.detections.empty()) torso.detections.clear();
  if (!sound.src.empty()) sound.src.clear();
  if (!weights.data.empty()) weights.data.clear();

  geometry_msgs::Pose l;
  sensor_msgs::RegionOfInterest d;
  hark_msgs::HarkSourceVal s;

  if (config.show_leg1)
  {
    l.position.x = config.leg1_x;
    l.position.y = config.leg1_y;
    legs.poses.push_back(l);
  }

  if (config.show_leg2)
  {
    l.position.x = config.leg2_x;
    l.position.y = config.leg2_y;
    legs.poses.push_back(l);
  }

  if (config.show_leg3)
  {
    l.position.x = config.leg3_x;
    l.position.y = config.leg3_y;
    legs.poses.push_back(l);
  }

  if (config.show_torso)
  {
    d.height = config.torso_height;
    d.width = config.torso_width;
    d.x_offset = config.torso_x;
    d.y_offset = config.torso_y;
    torso.detections.push_back(d);
  }

  if (config.show_sound)
  {
    s.azimuth = config.sound_dir;
    s.power = config.sound_pwr;
    s.y = config.sound_y;
    sound.src.push_back(s);
  }

  weights.data.push_back(config.leg_weight);
  weights.data.push_back(config.sound_weight);
  weights.data.push_back(config.torso_weight);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<likelihood_grid::TestConfig> server;
  dynamic_reconfigure::Server<likelihood_grid::TestConfig>::CallbackType f;

  std::string frame_id = "base_footprint";
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ros::Rate loop_rate(10);

  ros::Publisher leg_pub = n.advertise<geometry_msgs::PoseArray>("legs", 10);
  ros::Publisher sound_pub = n.advertise<hark_msgs::HarkSource>("/HarkSource", 10);
  ros::Publisher torso_pub = n.advertise<autonomy_human::raw_detections>("/person_detection/people", 10);
  ros::Publisher encoder_pub = n.advertise<nav_msgs::Odometry>("/encoder", 10);
  ros::Publisher weights_pub = n.advertise<std_msgs::Float32MultiArray>("/weights", 10);

  legs.header.frame_id = frame_id;
  encoder.header.frame_id = frame_id;
  torso.header.frame_id = frame_id;
  sound.header.frame_id = frame_id;

  encoder.twist.twist.linear.x = 0.0;
  encoder.twist.twist.angular.z = 0.0;

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    legs.header.stamp = now;
    encoder.header.stamp = now;
    torso.header.stamp = now;
    sound.header.stamp = now;

    leg_pub.publish(legs);
    encoder_pub.publish(encoder);
    torso_pub.publish(torso);
    sound_pub.publish(sound);
    weights_pub.publish(weights);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
