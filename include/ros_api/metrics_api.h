// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#ifndef METRICS_API_H
#define METRICS_API_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>

namespace socnav_metrics{
class Metrics
{
public:
  Metrics();
  ~Metrics();
  void initialize();
private:
  bool detectCollision(const ros::TimerEvent& event);
  void getMap(const nav_msgs::OccupancyGrid &grid);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  unsigned int getIndex(unsigned int mx, unsigned int my) const;

  ros::Timer get_robot_pose;
  geometry_msgs::PoseStamped robot_pose_;
  tf2_ros::Buffer tf_;
  ros::Subscriber map_sub_;
  ros::Publisher scan_pub_;
  nav_msgs::OccupancyGrid map_;
  // sensor_msgs::LaserScan map_scan_;
  int samples, size_x_, size_y_; 
  double angle_increment;
  double range_min, range_max;
  double origin_x_, origin_y_, resolution_;
  double robot_radius_;
  geometry_msgs::Polygon robot_shape_;

};


}// namespace socnav_metrics

 #endif  // METRICS_API_H