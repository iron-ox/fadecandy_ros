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

#include <diagnostic_updater/diagnostic_updater.h>
#include <fadecandy_msgs/LEDArray.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "../fadecandy_driver.h"

namespace fadecandy_driver
{
//!
//! \brief The FadecandyDriverRos class wraps ROS
//!
class FadecandyDriverRos : public FadecandyDriver
{
public:
  //!
  //! \brief FadecandyDriverRos fadecandy driver ROS wrapper
  //!
  FadecandyDriverRos();

  //!
  //! \brief connect Construct the connection with the driver
  //!
  void connect();

  //!
  //! \brief run Listen to LED messages and publishes the diagnostic of the driver
  //! \param restart_patience Restart patience
  //!
  void run(double restart_patience);

private:
  //!
  //! \brief setLedsCallback Fired when a new LEDArray message is received
  //!
  void setLedsCallback(const fadecandy_msgs::LEDArrayConstPtr& msg);

  //!
  //! \brief diagnosticsCallback Diagnostics callback
  //! \param diagnostic_status Status that should be updated
  //!
  void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);

  //!
  //! \brief timer_ Periodic timer
  //!
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent& e);

  //!
  //! \brief diagnostic_updater_ Diagnostic updater
  //!
  diagnostic_updater::Updater diagnostic_updater_;

  //!
  //! \brief led_subscriber_ LED messages subscriber
  //!
  ros::Subscriber led_subscriber_;

  //!
  //! \brief initialized_ Whether the driver is initialized
  //!
  bool initialized_ = false;
};
}  // namespace fadecandy_driver
