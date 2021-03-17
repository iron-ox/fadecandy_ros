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
FadecandyDriverROS::FadecandyDriverROS(double restart_patience) : restart_patience_(restart_patience)
{
  ros::NodeHandle nh;

  diagnostic_updater_.add("Info", this, &FadecandyDriverROS::diagnosticsCallback);
  diagnostics_timer_ = nh.createTimer(ros::Duration(1.), &FadecandyDriverROS::diagnosticsTimerCallback, this);

  connect_timer_ = nh.createTimer(ros::Duration(restart_patience_), &FadecandyDriverROS::connectTimerCallback, this);

  led_subscriber_ = nh.subscribe<fadecandy_msgs::LEDArray>("set_leds", 1, &FadecandyDriverROS::setLedsCallback, this);
}

void FadecandyDriverROS::run()
{
  ros::spin();
}

void FadecandyDriverROS::setLedsCallback(const fadecandy_msgs::LEDArrayConstPtr& led_array_msg)
{
  if (!driver_.isConnected())
  {
    return;
  }

  std::vector<std::vector<Color>> led_array_colors;
  for (const auto& strip : led_array_msg->strips)
  {
    std::vector<Color> led_strip_colors;
    for (const auto& color : strip.colors)
    {
      led_strip_colors.emplace_back(static_cast<int>(color.r * 255), static_cast<int>(color.g * 255),
                                    static_cast<int>(color.b * 255));
    }
    led_array_colors.push_back(led_strip_colors);
  }

  try
  {
    driver_.setColors(led_array_colors);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error occured: %s ", e.what());
  }
};

void FadecandyDriverROS::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  if (driver_.isConnected())
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected");
  }
  else
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected");
  }
}

void FadecandyDriverROS::diagnosticsTimerCallback(const ros::TimerEvent& e)
{
  diagnostic_updater_.force_update();
}

void FadecandyDriverROS::connectTimerCallback(const ros::TimerEvent& e)
{
  if (driver_.isConnected())
  {
    return;
  }

  try
  {
    auto serial_number = driver_.connect();
    diagnostic_updater_.setHardwareID(serial_number);
    ROS_INFO("Fadecandy device is connected.");
  }
  catch (const std::exception& e)
  {
    ROS_WARN_ONCE("Failed to connect to device: %s; will retry every %f seconds", e.what(), restart_patience_);
  }
}

}  // namespace fadecandy_driver
