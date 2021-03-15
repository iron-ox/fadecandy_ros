/*
 * Copyright (c) 2021 Eurotec, Netherlands
 * All rights reserved.
 *
 * Author: Jad Haj Mustafa
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <fadecandy_msgs/LEDArray.h>

#include "./fadecandy_driver_ros.h"

namespace fadecandy_driver
{
FadecandyDriverRos::FadecandyDriverRos()
{
  ros::NodeHandle nh;

  timer_ = nh.createTimer(ros::Duration(1.), &FadecandyDriverRos::timerCallback, this);

  diagnostic_updater_.add("Info", this, &FadecandyDriverRos::diagnosticsCallback);

  led_subscriber_ = nh.subscribe<fadecandy_msgs::LEDArray>("set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
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
      led_strip_colors.emplace_back(static_cast<int>(led_array_msg->strips[i].colors[j].r * 255),
                                    static_cast<int>(led_array_msg->strips[i].colors[j].g * 255),
                                    static_cast<int>(led_array_msg->strips[i].colors[j].b * 255));
    }
    led_array_colors.push_back(led_strip_colors);
  }
  try
  {
    setColors(led_array_colors);
  }
  catch (const std::exception& e)
  {
    initialized_ = false;
    return;
  }
};

void FadecandyDriverRos::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  if (fadecandy_device_ != NULL)
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
