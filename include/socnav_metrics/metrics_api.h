// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#ifndef METRICS_API_H
#define METRICS_API_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <socnav_metrics/AgentsData.h>
#include <socnav_metrics/MetricsData.h>
#include <socnav_metrics/HRMetrics.h>
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
  bool detectObsCollision(geometry_msgs::Pose robot_pose);
  void getMap(const nav_msgs::OccupancyGrid &grid);
  void calculateMetrics(const socnav_metrics::AgentsData &agents);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  unsigned int getIndex(unsigned int mx, unsigned int my) const;

  geometry_msgs::PoseStamped robot_pose_;
  tf2_ros::Buffer tf_;
  ros::Subscriber map_sub_, agent_data_sub_;
  ros::Publisher metrics_pub_;
  nav_msgs::OccupancyGrid map_;
  int samples, size_x_, size_y_; 
  double angle_increment;
  double range_min, range_max;
  double origin_x_, origin_y_, resolution_;
  double robot_radius_;
  geometry_msgs::Polygon robot_shape_;
  socnav_metrics::AgentsData agents_;
  ros::Time last_time_;

  //For episodic metrics
  int obs_colls;
  std::map<int, int> hr_colls;
  std::map<int, std::vector<double>> hr_dists;
  
  typedef struct{
    geometry_msgs::Pose pose;
    double dt; 
  }PoseVel;

  std::vector<PoseVel> robot_traj;


};


}// namespace socnav_metrics

 #endif  // METRICS_API_H