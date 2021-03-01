#include "../fadecandy_driver.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <fadecandy_msgs/LEDArray.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace fadecandy_driver {

class FadecandyDriverRos : public FadecandyDriver {
  //!
  //! \brief The FadecandyDriverRos class wraps ROS
  //!
  //! Publishes diagnostics
  //!
public:
  FadecandyDriverRos();
  void spin();
  void connect();

private:
  bool initialized;
  //!
  //! \brief setLedsCallback Fired when a new BatteryState is received
  //! \param state State
  //!
  void setLedsCallback(const fadecandy_msgs::LEDArrayConstPtr &msg);

  //!
  //! \brief diagnosticsCallback Diagnostics callback
  //! \param diagnostic_status Status that should be updated
  //!
  void diagnosticsCallback(
      diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status);

  //!
  //! \brief diagnostic_updater_ Diagnostic updater
  //!
  diagnostic_updater::Updater diagnostic_updater_;

  ros::Subscriber led_subscriber;
};

} // namespace fadecandy_driver
