#include "../fadecandy_driver.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <fadecandy_msgs/LEDArray.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace fadecandy_driver
{
class FadecandyDriverRos : public FadecandyDriver
{
  //!
  //! \brief The FadecandyDriverRos class wraps ROS
  //!
  //!
public:
  FadecandyDriverRos();
  void spin();
  void connect();
  bool initialized;

private:
  //!
  //! \brief setLedsCallback fired when a new LEDArray message is received
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

  ros::Subscriber led_subscriber;
};

}  // namespace fadecandy_driver
