// Author: Phani Teja Singamaneni
// SocialNavMetrics Calculation

#ifndef SOCIALNAVMETRICS_H
#define SOCIALNAVMETRICS_H
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <socnav_metrics/AgentsData.h>
#include <socnav_metrics/MetricsData.h>
#include <socnav_metrics/HRMetrics.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>
#define Pi 3.141592653589793

namespace socnav_metrics{

typedef struct{
  geometry_msgs::Pose pose;
  double dt; 
}PoseTime;

using Trajectory = std::vector<PoseTime> ;

class SocialNavMetrics
{
public:
    SocialNavMetrics();
    ~SocialNavMetrics();
    void getStepMetrics(const AgentsData &agents, MetricsData &metrics);
    void getEpisodeMetrics(const AgentsData &agents, Trajectory traj, MetricsData &metrics);
    bool detectObsCollision(geometry_msgs::Pose robot_pose);
    bool setMap(const nav_msgs::OccupancyGrid &grid);
    inline double normalize_theta( double theta ){
        if (theta >= -Pi && theta < Pi)
            return theta;
            
        double multiplier = floor(theta / (2*Pi));
        theta = theta - multiplier*2*Pi;
        if (theta >= Pi)
            theta -= 2*Pi;
        if (theta < -Pi)
            theta += 2*Pi;
        return theta;
    }

private:
    // Methods
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    unsigned int getIndex(unsigned int mx, unsigned int my) const;

    // Variables
    nav_msgs::OccupancyGrid map_;
    int samples, size_x_, size_y_; 
    double angle_increment;
    double range_min, range_max;
    double origin_x_, origin_y_, resolution_;
    double robot_radius_;
    geometry_msgs::Polygon robot_shape_;


    AgentsData agents_data_;

    //For episodic metrics
    int obs_colls;
    std::map<int, int> hr_colls;
    std::map<int, std::vector<double>> hr_dists;
};

}// namespace socnav_metrics

#endif  // SOCIALNAVMETRICS_H