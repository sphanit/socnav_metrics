// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#ifndef METRICS_API_H
#define METRICS_API_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Polygon.h>
#include <socnav_metrics/AgentsData.h>
#include <socnav_metrics/MetricsData.h>
#include <socnav_metrics/socialnavmetrics.h>
// #include <socnav_metrics/HRMetrics.h>
// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2/utils.h>

namespace socnav_metrics{

class SocialNavROS
{
public:
  SocialNavROS();
  ~SocialNavROS();
  void initialize();
private:
  void getMap(const nav_msgs::OccupancyGrid &grid);
  void publishMetrics(const AgentsData &agents);

  // geometry_msgs::PoseStamped robot_pose_;
  tf2_ros::Buffer tf_;
  ros::Subscriber map_sub_, agent_data_sub_;
  ros::Publisher metrics_pub_;
  SocialNavMetrics SNMetrics;

  // AgentsData agents_;
  ros::Time last_time_;
  int eps_id_;
  bool goal_reached_;
  
  AgentsData agents_;
  Trajectory robot_traj;

};



}// namespace socnav_metrics

 #endif  // METRICS_API_H