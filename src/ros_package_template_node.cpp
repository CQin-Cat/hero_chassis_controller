#include "hero_chassis_controller/RosPackageTemplate.hpp"
#include <ros/ros.h>
void callback(int32_t){

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hero_chassis_controller");
  ros::NodeHandle nodeHandle("~");

  ros_package_template::RosPackageTemplate rosPackageTemplate(nodeHandle);

  ros::spin();
  return 0;
}
