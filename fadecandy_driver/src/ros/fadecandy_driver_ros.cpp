#include "./fadecandy_driver_ros.h"
#include <fadecandy_msgs/LEDArray.h>

namespace fadecandy_driver {

FadecandyDriverRos::FadecandyDriverRos() {
  ros::NodeHandle nh;
  diagnostic_updater_.add("Info", this,
                          &FadecandyDriverRos::diagnosticsCallback);
  led_subscriber = nh.subscribe<fadecandy_msgs::LEDArray>(
      "/set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
  initialized = false;
}

void FadecandyDriverRos::connect() {

  if (fadecandy_device != NULL) {
    release();
    fadecandy_device = NULL;
  }

  double connection_retry_rate = 1.0;
  ROS_INFO("Connecting to Fadecandy device ..");
  while (ros::ok()) {

    if (!initialized) {
      initialized = FadecandyDriverRos::intialize();
      if (initialized) {
        ROS_INFO("Connected to Fadecandy device");
        ros::spin();
      }
    }
    ROS_INFO("Re connecting in %f sec ", connection_retry_rate);

    ros::Duration(connection_retry_rate).sleep();
  }
}

void FadecandyDriverRos::setLedsCallback(
    const fadecandy_msgs::LEDArrayConstPtr &led_array_msg) {
  if (fadecandy_device == NULL) {
    return;
  }
  std::vector<std::vector<colors>> led_array_colors;
  std::vector<colors> led_strip_colors;
  for (size_t i = 0; i < led_array_msg->strips.size(); ++i) {
    led_strip_colors.clear();
    for (size_t j = 0; j < led_array_msg->strips[i].colors.size(); ++j) {
      led_strip_colors.push_back(
          {(int)(led_array_msg->strips[i].colors[j].r * 255),
           (int)(led_array_msg->strips[i].colors[j].g * 255),
           (int)(led_array_msg->strips[i].colors[j].b * 255)});
    }
    led_array_colors.push_back(led_strip_colors);
  }
  try {
    FadecandyDriver::setColors(led_array_colors);
  } catch (const std::exception &e) {
    ROS_ERROR("Exception: %s", e.what());
    initialized = false;
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
