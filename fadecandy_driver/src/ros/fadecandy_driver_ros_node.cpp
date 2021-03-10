//
// Copyright (c) 2021 Eurotec
//

#include <ros/init.h>
#include <ros/node_handle.h>

#include "./fadecandy_driver_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fadecandy_driver");
  ros::NodeHandle local_nh("~");
  double restart_patience = local_nh.param("restart_patience", 1.);

  fadecandy_driver::FadecandyDriverRos node;

  while (ros::ok())
  {
    try
    {
      if (!node.initialized_)
      {
        ROS_INFO("Connecting to Fadecandy device ..");
        node.run();
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Exception: %s", e.what());
      ROS_INFO("Restarting driver in %.2f seconds ..", restart_patience);

      ros::Duration(restart_patience).sleep();
    }
    ros::spinOnce();
    ros::Duration(.1).sleep();
  }
  return 0;
}
