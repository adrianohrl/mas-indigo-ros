// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <mas_lib/hello_world.h>

// Standard C++ entry point
int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "hello_world_node");
  ros::NodeHandle nh;

  // Call our library function
  say_hello();

  // Wait for SIGINT/Ctrl-C
  ros::spin();
  return 0;
}
