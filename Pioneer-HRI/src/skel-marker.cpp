#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "pi_tracker/Skeleton.h"

// Subscribe to the skeleton topic.
  void skeleton_cb(const pi_tracker::Skeleton &msg)
  {
     //last_skel = msg.header.stamp;
      last_skel = ros::Time::now();
      pi_tracker::Skeleton Persons, mean_person;
      bool allConf = true;
      Persons = msg;
      vector<string> Names;
      unsigned int counter = 10;
      Names = Persons.name;
      //float depth_min, depth_max, height_min, height_max;
      for(unsigned int i = 0; i < JOINTS_NUM ; i++)
      {
          allConf = allConf && Persons.confidence.at(i);
      }
      if (Persons.confidence.size() > 0)
      {
          if (allConf)
          {
              if (past_persons.size() < counter)
              {
                  past_persons.push_back(Persons);
              } else
              {
                  past_persons.pop_front();
                  past_persons.push_back(Persons);
              }

              if (past_persons.size() == counter)
              {
                  mean_p(past_persons, mean_person);
                  person_edges(mean_person, depth_min, depth_max, height_min, height_max);
                  hand_gesture(mean_person);
              }
          }
          else
              pub_gesture.publish(no_gesture);
      }
  } // End of skeleton_cb

int main( int argc, char** argv )
{
  ros::init(argc, argv, "skel_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("skel_marker", 1);



  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

  while (ros::ok())
  {
      visualization_msgs::MarkerArray marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "skel_marker";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD and DELETE
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    marker_pub.publish(marker);
// %EndTag(PUBLISH)%

    // Cycle between different shapes
// %Tag(CYCLE_SHAPES)%
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
// %EndTag(CYCLE_SHAPES)%

// %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%

