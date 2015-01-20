
#include "leg_grid.h"

LegGrid::LegGrid()
{
    ROS_INFO("Constructing an instance of Leg Grid.");
}

void LegGrid::init()
{

}

void LegGrid::syncCallBack(const geometry_msgs::PoseArrayConstPtr &leg_msg,
                           const nav_msgs::OdometryConstPtr &encoder_msg)
{
    ros::Time now = ros::Time::now();
    /*** Transform detected legs from laser frame to base_footprint frame ***/

    if(!transformToBase(leg_msg, grid->cartesian_array)){

        ROS_WARN("Can not transform from laser to base_footprint");

    } else{
        ROS_ASSERT(leg_msg->poses.size() == grid->cartesian_array.poses.size());
        grid->getPose(grid->cartesian_array);
        ROS_ASSERT(grid->polar_array.current.size() == leg_msg->poses.size());
    }

    /*** For predicting the human target, we assume that robot is stationary ***/
    /*** and human is moving with the robot's opposite velocity.  ***/

    leg_velocity_.linear = - sqrt( pow(encoder_msg->twist.twist.linear.x,2) + pow(encoder_msg->twist.twist.linear.y,2) );
    leg_velocity_.angular = - encoder_msg->twist.twist.angular.z;
    diff_time_ = now - last_time_;
    last_time_ = now;
}

bool LegGrid::transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
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
            tf_listener_->transformPoint("base_footprint", source_point, target_point);
            if (debug) {
                tf::StampedTransform _t;
                tf_listener_->lookupTransform("base_footprint", source_point.header.frame_id, ros::Time(0), _t);
                ROS_INFO("From %s to bfp: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
                         source_point.header.frame_id.c_str(),
                         _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
                         _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(),
                         _t.getRotation().getW());
            }
            can_transform = true;
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_footprint\": %s",
                      source_point.header.frame_id.c_str(), ex.what());
            can_transform = false;
        }

        target_pose.position = target_point.point;
        target_pose.position.z = 0.0;
        target.poses.push_back(target_pose);
    }

    return can_transform;
}


void LegGrid::spin()
{
    uint8_t eye2[2][2];
    eye2[0][0] = eye2[1][1] = 1;
    eye2[0][1] = eye2[1][0] = 0;

    float F[2][2];
    float B[2][2];
    float Q[2][2];
    float H[2][2];
    float R[2][2];
}

void LegGrid::kalmanFilter()
{

}
