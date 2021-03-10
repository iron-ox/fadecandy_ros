#include <ros/init.h>
#include <ros/node_handle.h>

#include "./fadecandy_driver_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fadecandy_driver");
  ros::NodeHandle local_nh("~");
  fadecandy_driver::FadecandyDriverRos Node;

  double restart_patience = 1.;
  while (ros::ok()) {
    try {
      if (!Node.initialized) {
        ROS_INFO("Connecting to Fadecandy device ..");
        Node.connect();
      }

    } catch (const std::exception &e) {
      ROS_ERROR("Exception: %s", e.what());
      ROS_INFO("Restarting driver in %.2f seconds ..", restart_patience);

      ros::Duration(restart_patience).sleep();
    }
    ros::spinOnce();

    ros::Duration(.1).sleep();
  }
  return 0;
}
