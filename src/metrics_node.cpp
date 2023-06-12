#include <socnav_metrics/metrics_api.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "metrics_node");
  socnav_metrics::Metrics metrics;
  return 0;
}