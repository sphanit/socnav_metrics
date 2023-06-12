// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#include <socnav_metrics/metrics_api.h>
#define Pi 3.141592653589793

namespace socnav_metrics
{
  Metrics::Metrics(){
    initialize();
  }

  Metrics::~Metrics(){}

  void Metrics::initialize(){
    ros::NodeHandle nh("~/");
    tf2_ros::TransformListener tfListener(tf_);
    map_sub_ = nh.subscribe("/map", 1, &Metrics::getMap, this);
    agent_data_sub_ = nh.subscribe("Agents", 1, &Metrics::calculateMetrics, this);
    metrics_pub_ = nh.advertise<socnav_metrics::MetricsData>("metrics",1);
    obs_colls=0;
    hr_colls.clear();
    hr_dists.clear();
    robot_traj.clear();
    ros::spin();

  }

  // const ros::TimerEvent& event
  bool Metrics::detectObsCollision(geometry_msgs::Pose robot_pose){

    auto theta = tf2::getYaw(robot_pose.orientation);

    // Set the parameters for raytracing
    range_min = 0.01;
    range_max = robot_radius_;
    samples = 360;
    angle_increment = (2*Pi)/samples;
    int scan_resolution = robot_radius_*100;
    auto ang = -Pi;
    double increment_ = range_max/scan_resolution;
    std::vector<geometry_msgs::Point> possible_collsions;

    Eigen::Vector2d robot_vec{cos(theta),sin(theta)};

    for(int i=0;i<samples;i++){
      if(map_.data.empty()){
      continue;
      }
      double ray_ = range_min;
      Eigen::Vector2d r_dir{robot_vec.x()*cos(ang)-robot_vec.y()*sin(ang),
                            +robot_vec.x()*sin(ang)+robot_vec.y()*cos(ang)};

      for(int j=0;j<scan_resolution;j++){
        auto rx = robot_pose.position.x + ray_*r_dir.x();
        auto ry = robot_pose.position.y + ray_*r_dir.y();
        int mx, my;

        if(!worldToMap(rx,ry,mx,my)){
          continue;
        }
        auto idx = getIndex(mx,my);

        if(int(map_.data[idx]) >= 100 && int(map_.data[idx]) <=254){
          geometry_msgs::Point point;
          point.x = rx;
          point.y = ry;
          possible_collsions.push_back(point);
          break;
        }
        ray_ += increment_;
      }
      ang += angle_increment;
    }


    int nv = robot_shape_.points.size();
    auto vertices = robot_shape_.points;
    bool inside;

    for (auto const &point : possible_collsions){
      inside = false;
      for(int i = 0, j = nv - 1; i < nv; j = i++) {
        if( ( (vertices[i].y >= point.y ) != (vertices[j].y >= point.y) ) &&
          (point.x <= (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x)
        )
        inside = !inside;
        break;
      }
    }

    return inside;

  }

  void Metrics::getMap(const nav_msgs::OccupancyGrid &grid){
    map_ = grid;
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    resolution_ = map_.info.resolution;
    size_x_ = map_.info.width;
    size_y_ = map_.info.height;
  }

  void Metrics::calculateMetrics(const socnav_metrics::AgentsData &agents)
  {
    agents_ = agents;
    robot_radius_ = agents_.robot.radius;
    
    socnav_metrics::MetricsData metrics;
    metrics.type = socnav_metrics::MetricsData::STEP;
    metrics.t_name = "step";

    if(detectObsCollision(agents_.robot.pose))
    {
      metrics.obs_collisions = 1;
      obs_colls += 1;
    }

    for(auto const &human: agents_.humans){
      socnav_metrics::HRMetrics hr_metrics;
      double hr_dist = std::hypot(human.pose.position.x-agents_.robot.pose.position.x,
                                    human.pose.position.y-agents_.robot.pose.position.y);
      
      if((agents.robot.radius+human.radius)>= hr_dist)
      {
        hr_metrics.hr_collisions = 1;
        hr_colls[human.track_id] += 1;

      }

      hr_metrics.hr_dist = hr_dist;
      hr_dists[human.track_id].push_back(hr_dist);

      metrics.social_metrics.push_back(hr_metrics);
    }

    metrics_pub_.publish(metrics);

    PoseVel pv;
    pv.pose = agents_.robot.pose;
    pv.dt = (ros::Time::now()-last_time_).toSec();
    robot_traj.push_back(pv);

    last_time_ = ros::Time::now();

  }


  bool Metrics::worldToMap(double wx, double wy, int& mx, int& my) const{
    if(wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  unsigned int Metrics::getIndex(unsigned int mx, unsigned int my) const{
   return my * size_x_ + mx;
 }

}