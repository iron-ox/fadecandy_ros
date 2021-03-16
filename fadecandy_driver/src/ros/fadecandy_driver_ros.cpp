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
FadecandyDriverRos::FadecandyDriverRos(double restart_patience)
{
  ros::NodeHandle nh;

  restart_patience_ = restart_patience;

  timer_ = nh.createTimer(ros::Duration(restart_patience_), &FadecandyDriverRos::timerCallback, this);

  connectionCheckTimer_ =
      nh.createTimer(ros::Duration(restart_patience_), &FadecandyDriverRos::connectionCheckTimerCallback, this);

  diagnostic_updater_.add("Info", this, &FadecandyDriverRos::diagnosticsCallback);

  led_subscriber_ = nh.subscribe<fadecandy_msgs::LEDArray>("set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
}

void FadecandyDriverRos::run()
{
  ros::spin();
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
  if (isConnected_)
  {
    try
    {
      setColors(led_array_colors);
    }
    catch (const std::exception& e)
    {
      isConnected_ = false;
      ROS_ERROR("Error occured: %s ", e.what());
    }
  }
  else
  {
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

void FadecandyDriverRos::connectionCheckTimerCallback(const ros::TimerEvent& e)
{
  if (!isConnected_)
  {
    try
    {
      isConnected_ = intialize();
      if (isConnected_)
      {
        diagnostic_updater_.setHardwareID(serial_number_);
        ROS_INFO("Connected to Fadecandy device");
      }
    }
    catch (const std::exception& e)
    {
      ROS_WARN("Failed to connect to Fadecandy device %s; will retry every %f second", serial_number_.c_str(),
               restart_patience_);
    }
  }
  else
  {
    return;
  }
}

}  // namespace fadecandy_driver
