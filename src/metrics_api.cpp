// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#include <socnav_metrics/metrics_api.h>


namespace socnav_metrics
{

  // ROS API Class
  SocialNavROS::SocialNavROS(){
    initialize();
  }

  SocialNavROS::~SocialNavROS(){}

  void SocialNavROS::initialize(){
    ros::NodeHandle nh("~/");
    tf2_ros::TransformListener tfListener(tf_);
    map_sub_ = nh.subscribe("/map", 1, &SocialNavROS::getMap, this);
    agent_data_sub_ = nh.subscribe("Agents", 1, &SocialNavROS::publishMetrics, this);
    metrics_pub_ = nh.advertise<MetricsData>("metrics",1);
    robot_traj.clear();
    goal_reached_=false;
    eps_id_ = -1;
    ros::spin();

  }


  void SocialNavROS::getMap(const nav_msgs::OccupancyGrid &grid){
    SNMetrics.setMap(grid);
  }


  void SocialNavROS::publishMetrics(const AgentsData &agents)
  {
    
    agents_ = agents;

    // Handle the episode termination
    if(eps_id_!=agents_.eps_id && !goal_reached_){
      // Publish metrics for the unfinished goal
      MetricsData metrics;
      metrics.type = MetricsData::EPISODE;
      metrics.t_name = "episode";
      metrics.time_stamp = ros::Time::now();
      metrics.success = 0;
      SNMetrics.getEpisodeMetrics(agents, robot_traj, metrics);
      metrics_pub_.publish(metrics);
    }

    if(abs(agents_.robot.pose.position.x - agents_.robot.start.position.x) < 0.1 && abs(agents_.robot.pose.position.y - agents_.robot.start.position.y) < 0.1){
      last_time_ = ros::Time::now();
      eps_id_ = agents_.eps_id;
      goal_reached_ = false;
    }

    // Publish Step wise metrics
    MetricsData metrics;
    metrics.type = MetricsData::STEP;
    metrics.t_name = "step";
    metrics.time_stamp = ros::Time::now();
    metrics.success = -1;
    metrics.spl = -1;
    metrics.min_movement_jerk = std::numeric_limits<float>::infinity();
    SNMetrics.getStepMetrics(agents, metrics);
    metrics_pub_.publish(metrics);

    // Store the trajectory of the robot
    PoseTime pt;
    pt.pose = agents_.robot.pose;
    pt.dt = (ros::Time::now()-last_time_).toSec();
    robot_traj.push_back(pt);
    last_time_ = ros::Time::now();

    // Check if the robot has reached the desired goal
    double theta_current = tf2::getYaw(agents_.robot.pose.orientation);
    double theta_goal = tf2::getYaw(agents_.robot.goal.orientation);

    if(fabs(agents_.robot.pose.position.x - agents_.robot.start.position.x) < 0.1 && fabs(agents_.robot.pose.position.y - agents_.robot.start.position.y) < 0.1
        && fabs(SNMetrics.normalize_theta(theta_goal-theta_current))< 0.2){
      
      goal_reached_ = true;

      // Publish Episode metrics for successful goal
      MetricsData metrics;
      metrics.type = MetricsData::EPISODE;
      metrics.t_name = "episode";
      metrics.time_stamp = ros::Time::now();
      metrics.success = 1;
      SNMetrics.getEpisodeMetrics(agents, robot_traj, metrics);
      metrics_pub_.publish(metrics);
    }

    

  }

}