// Author: Phani Teja Singamaneni
// ROS_API for calculating metrics

#include <socnav_metrics/socialnavmetrics.h>

namespace socnav_metrics
{
    //SocialNav Metrics Class

    SocialNavMetrics::SocialNavMetrics(){}

    SocialNavMetrics::~SocialNavMetrics(){}

    void SocialNavMetrics::getStepMetrics(const AgentsData &agents, MetricsData &metrics){
        std::cout << "Step metrics" << "\n";
        agents_data_ = agents;

        // Collision Detection with static obstacles in map
        if(detectObsCollision(agents_data_.robot.pose))
        {
            metrics.obs_collisions = 1;
            
            // For Episode
            obs_colls += 1;
        }

        // Human-Robot Distance and Collision Checks
        for(auto const &human: agents_data_.humans){
            HRMetrics hr_metrics;
            double hr_dist = std::hypot(human.pose.position.x-agents_data_.robot.pose.position.x,
                                            human.pose.position.y-agents_data_.robot.pose.position.y);
            
            if((agents.robot.radius+human.radius)>= hr_dist)
            {
                hr_metrics.hr_collisions = 1;
                
                // For Episode
                hr_colls[human.track_id] += 1;

            }

            hr_metrics.hr_dist = hr_dist;
            hr_metrics.min_hr_dist = -1;
            
            // For Episode
            hr_dists[human.track_id].push_back(hr_dist);

            metrics.social_metrics.push_back(hr_metrics);
        }
    }

    void SocialNavMetrics::getEpisodeMetrics(const AgentsData &agents, Trajectory traj, MetricsData &metrics){
        std::cout << "Episode metrics" << "\n";

        metrics.obs_collisions = obs_colls;

        for(auto const &human: agents_data_.humans){
            HRMetrics hr_metrics;
            hr_metrics.hr_collisions = hr_colls[human.track_id];
            hr_metrics.min_hr_dist = *std::min_element(hr_dists[human.track_id].begin(),hr_dists[human.track_id].end());        
            hr_metrics.hr_dist = hr_dists[human.track_id].back();
            metrics.social_metrics.push_back(hr_metrics);
        }    

        //Reset Data before the next episode
        obs_colls=0;
        hr_colls.clear();
        hr_dists.clear();
    }

    // const ros::TimerEvent& event
    bool SocialNavMetrics::detectObsCollision(geometry_msgs::Pose robot_pose){

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

    bool SocialNavMetrics::setMap(const nav_msgs::OccupancyGrid &grid){
        map_ = grid;
        origin_x_ = map_.info.origin.position.x;
        origin_y_ = map_.info.origin.position.y;
        resolution_ = map_.info.resolution;
        size_x_ = map_.info.width;
        size_y_ = map_.info.height;
    }
    

    bool SocialNavMetrics::worldToMap(double wx, double wy, int& mx, int& my) const{
        if(wx < origin_x_ || wy < origin_y_)
        return false;

        mx = (int) ((wx - origin_x_) / resolution_);
        my = (int) ((wy - origin_y_) / resolution_);

        if(mx < size_x_ && my < size_y_)
        return true;

        return false;
    }

    unsigned int SocialNavMetrics::getIndex(unsigned int mx, unsigned int my) const{
    return my * size_x_ + mx;
    }
}