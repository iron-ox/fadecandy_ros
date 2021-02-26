#include "./fadecandy_driver_ros.h"
#include <fadecandy_msgs/LEDArray.h>

namespace fadecandy_driver {

FadecandyDriverRos::FadecandyDriverRos() {
  ros::NodeHandle nh;
  diagnostic_updater_.add("Info", this,
                          &FadecandyDriverRos::diagnosticsCallback);
  led_subscriber = nh.subscribe<fadecandy_msgs::LEDArray>(
      "/set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
  FadecandyDriverRos::intialize();
}

void FadecandyDriverRos::connect() {
  ROS_INFO("Connecting to Fadecandy device ..");
  FadecandyDriver::release();
  FadecandyDriverRos::intialize();
  if (FadecandyDriver::fadecandy_device == NULL) {
    FadecandyDriver::release();
    FadecandyDriverRos::intialize();
  }
  ROS_INFO("Connecting to Fadecandy device ..");

  bool exception_caught = true;
  double connection_retry_rate = 1.0;
  ROS_INFO("Connecting to Fadecandy device ..");
  while (ros::ok()) {
    ros::spin();
    try {
      ROS_INFO("Connected to Fadecandy device");
      exception_caught = false;
    } catch (const std::exception &e) {
      ROS_ERROR("Exception: %s", e.what());
      ROS_INFO("Restarting driver in %.2f seconds ..", connection_retry_rate);
      ros::Duration(connection_retry_rate).sleep();
    }
    if (!exception_caught) {
      ROS_INFO("Connected to Fadecandy device ..");
      break;
      //      unsigned char string[256];
      //      libusb_get_string_descriptor_ascii(
      //          dev_handle,
      //      fadecandy_device_descriptor.iSerialNumber,
      //      string,
      //      //          sizeof(string));
      //      //      std::string *s = (std::string *)string;
      //      //      diagnostic_updater_.setHardwareID(*s);
    }
  }
}

void FadecandyDriverRos::setLedsCallback(
    const fadecandy_msgs::LEDArrayConstPtr &led_array_msg) {
  ROS_INFO("Call back ..");
  if (FadecandyDriver::fadecandy_device == NULL)
    return;
  std::vector<std::vector<colors>> led_array_colors;
  std::vector<colors> led_strip_colors;
  for (size_t i = 0; i < led_array_msg->strips.size(); ++i) {
    led_strip_colors.clear();
    for (size_t j = 0; j < led_array_msg->strips[i].colors.size(); ++j) {
      led_strip_colors.push_back({led_array_msg->strips[i].colors[i].r * 255,
                                  led_array_msg->strips[i].colors[i].g * 255,
                                  led_array_msg->strips[i].colors[i].b * 255});
    }
    led_array_colors.push_back(led_strip_colors);
  }
  //  FadecandyDriver::set_colors(led_array_colors);
  try {
    FadecandyDriver::set_colors(led_array_colors);
  } catch (const std::exception &e) {
    ROS_ERROR("Exception: %s", e.what());
    FadecandyDriverRos::connect();
  }
};
void FadecandyDriverRos::diagnosticsCallback(
    diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status) {
  if (FadecandyDriver::fadecandy_device == NULL) {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                              "Disconnected");
  } else {
    diagnostic_status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                               "Connected");
  }
}

} // namespace fadecandy_driver
