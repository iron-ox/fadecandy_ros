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
  node.run(restart_patience);
  return 0;
}
