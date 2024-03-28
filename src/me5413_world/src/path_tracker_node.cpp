/** path_tracker_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to track a given path
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double LOOKAHEAD_DISTANCE;
bool PARAMS_UPDATED;

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // Pure Pursuit
  LOOKAHEAD_DISTANCE = config.lookahead_distance;

  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate lookahead point
  geometry_msgs::Pose lookahead_point = findLookaheadPoint(path);

  // Compute control outputs
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, lookahead_point));

  return;
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

geometry_msgs::Pose PathTrackerNode::findLookaheadPoint(const nav_msgs::Path::ConstPtr& path)
{
  // Iterate through path to find lookahead point
  for (size_t i = 0; i < path->poses.size(); ++i)
  {
    geometry_msgs::Pose pose = path->poses[i].pose;

    // Calculate distance to current pose
    tf2::Vector3 point_robot, point_goal;
    tf2::fromMsg(this->odom_world_robot_.pose.pose.position, point_robot);
    tf2::fromMsg(pose.position, point_goal);
    const double distance = (point_goal - point_robot).length();

    // Check if distance exceeds lookahead distance
    if (distance >= LOOKAHEAD_DISTANCE)
      return pose;
  }

  // If no suitable lookahead point found, return last point
  return path->poses.back().pose;
}

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& lookahead_point)
{
  // Calculate desired heading towards lookahead point
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  tf2::fromMsg(lookahead_point.orientation, q_goal); // Using the orientation of the lookahead point
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = unifyAngleRange(yaw_goal - yaw_robot); // Calculating the heading_error
  
  // Compute angular velocity (steering)
  double angular_velocity = K_PURE_PURSUIT * heading_error;

  // Compute linear velocity (constant for now)
  const double linear_velocity = SPEED_TARGET;

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear_velocity;
  cmd_vel.angular.z = angular_velocity;

  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
