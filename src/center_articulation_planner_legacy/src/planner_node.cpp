#include <ros/ros.h>
#include "center_articulation_planner/ros/planner_ros.hpp"
#include "center_articulation_planner/ros/ros_io.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "center_articulation_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  jhzx::center_articulation_planner::PlannerROS planner;
  jhzx::center_articulation_planner::RosIo ros_io(nh, pnh, planner);  
  ros::spin();
  return 0;
}
