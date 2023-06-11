// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#include <ros_api/metrics_api.h>
#define Pi 3.141592653589793

// Metrics to implement
// Success rate, SPL, Collision, human-robot distance, movement jerk

namespace socnav_metrics
{
  Metrics::Metrics(){
    initialize();
  }

  Metrics::~Metrics(){}

  void Metrics::initialize(){
    ros::NodeHandle nh("~/");
    tf2_ros::TransformListener tfListener(tf_);
    get_robot_pose = nh.createTimer(ros::Duration(0.01), &Metrics::detectCollision, this);
    // scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("map_scan",1);
    map_sub_ = nh.subscribe("/map", 1, &Metrics::getMap, this);

    robot_radius_ = nh.parameter....


    range_min = 0.01;
    range_max = robot_radius_;

    ros::spin();

  }

  bool Metrics::detectCollision(const ros::TimerEvent& event){

    // Get Robot Pose
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tf_.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    robot_pose_.header = transformStamped.header;
    robot_pose_.pose.position.x = transformStamped.transform.translation.x;
    robot_pose_.pose.position.y = transformStamped.transform.translation.y;
    robot_pose_.pose.position.z = transformStamped.transform.translation.z;
    robot_pose_.pose.orientation = transformStamped.transform.rotation;
    auto theta = tf2::getYaw(robot_pose_.pose.orientation);

    // Set the parameters for raytracing
    samples = 360;
    angle_increment = (2*Pi)/samples;
    // std::vector<float> ranges(samples, 0.0);
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

      // ranges[i] = range_max;

      for(int j=0;j<scan_resolution;j++){
        auto rx = robot_pose_.pose.position.x + ray_*r_dir.x();
        auto ry = robot_pose_.pose.position.y + ray_*r_dir.y();
        int mx, my;

        if(!worldToMap(rx,ry,mx,my)){
          continue;
        }
        auto idx = getIndex(mx,my);

        if(int(map_.data[idx]) >= 100 && int(map_.data[idx]) <=254){
          // ranges[i] = ray_;
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


    int nv = robot_shape_.polygon.points.size();
    auto vertices = robot_shape_.polygon.points;
    bool inside;
    // std::vector<bool> coll_check;

    for (auto const &point : possible_collsions){
      inside = false;
      for(int i = 0, j = nv - 1; i < nv; j = i++) {
        if( ( (vertices[i].y >= point.y ) != (vertices[j].y >= point.y) ) &&
          (point.x <= (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x)
        )
        inside = !inside;
        break;
      }
      // coll_check.push_back(inside);
    }

    return inside;

  }

  void Metrics::getMap(const nav_msgs::OccupancyGrid &grid){
    map_ = grid;
    // std::cout << map_.data.size() << "\n";
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    resolution_ = map_.info.resolution;
    size_x_ = map_.info.width;
    size_y_ = map_.info.height;
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