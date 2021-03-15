//
// Copyright (c) 2020 Eurotec
//
// @author Jad Haj Mustafa
//

#include "./fadecandy_driver_ros.h"
#include <fadecandy_msgs/LEDArray.h>

namespace fadecandy_driver
{
FadecandyDriverRos::FadecandyDriverRos()
{
  ros::NodeHandle nh;

  timer_ = nh.createTimer(ros::Duration(1.), &FadecandyDriverRos::timerCallback, this);

  diagnostic_updater_.add("Info", this, &FadecandyDriverRos::diagnosticsCallback);

  led_subscriber_ = nh.subscribe<fadecandy_msgs::LEDArray>("set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
  initialized_ = false;
}

void FadecandyDriverRos::connect()
{
  if (fadecandy_device_ != NULL)
  {
    releaseInterface();
    fadecandy_device_ = NULL;
  }
  if (!initialized_)
  {
    initialized_ = FadecandyDriverRos::intialize();
    ROS_INFO("Connected to Fadecandy device");
    diagnostic_updater_.setHardwareID(serial_number_);
  }
}

void FadecandyDriverRos::run(double restart_patience)
{
  while (ros::ok())
  {
    try
    {
      if (!initialized_)
      {
        ROS_INFO("Connecting to Fadecandy device ..");
        connect();
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Exception: %s", e.what());
      ROS_INFO("Restarting driver in %.2f seconds ..", restart_patience);

      ros::Duration(restart_patience).sleep();
    }
    ros::spinOnce();
  }
}

void FadecandyDriverRos::setLedsCallback(const fadecandy_msgs::LEDArrayConstPtr& led_array_msg)
{
  std::vector<std::vector<Color>> led_array_colors;
  for (size_t i = 0; i < led_array_msg->strips.size(); ++i)
  {
    std::vector<Color> led_strip_colors;
    for (size_t j = 0; j < led_array_msg->strips[i].colors.size(); ++j)
    {
      led_strip_colors.push_back({ (int)(led_array_msg->strips[i].colors[j].r * 255),
                                   (int)(led_array_msg->strips[i].colors[j].g * 255),
                                   (int)(led_array_msg->strips[i].colors[j].b * 255) });
    }
    led_array_colors.push_back(led_strip_colors);
  }
  try
  {
    FadecandyDriver::setColors(led_array_colors);
  }
  catch (const std::exception& e)
  {
    initialized_ = false;
    return;
  }
};

void FadecandyDriverRos::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  if (FadecandyDriver::fadecandy_device_ != NULL)
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected");
  }
  else
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected");
  }
}

void FadecandyDriverRos::timerCallback(const ros::TimerEvent& e)
{
  diagnostic_updater_.force_update();
}
}  // namespace fadecandy_driver
