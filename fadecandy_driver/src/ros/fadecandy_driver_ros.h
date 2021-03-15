//
// Copyright (c) 2020 Eurotec
//
// @author Jad Haj Mustafa
//

#include "../fadecandy_driver.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <fadecandy_msgs/LEDArray.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

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

  bool initialized_;

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
};

}  // namespace fadecandy_driver
