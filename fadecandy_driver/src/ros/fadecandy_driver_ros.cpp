#include "./fadecandy_driver_ros.h"
#include <fadecandy_msgs/LEDArray.h>

namespace fadecandy_driver
{
FadecandyDriverRos::FadecandyDriverRos()
{
  ros::NodeHandle nh;

  timer_ = nh.createTimer(ros::Duration(1.), &FadecandyDriverRos::timerCallback, this);

  diagnostic_updater_.add("Info", this, &FadecandyDriverRos::diagnosticsCallback);

  led_subscriber = nh.subscribe<fadecandy_msgs::LEDArray>("/set_leds", 1, &FadecandyDriverRos::setLedsCallback, this);
  initialized = false;
}

void FadecandyDriverRos::connect()
{
  if (fadecandy_device != NULL)
  {
    release();
    fadecandy_device = NULL;
  }

  if (!initialized)
  {
    initialized = FadecandyDriverRos::intialize();
    if (initialized)
    {
      ROS_INFO("Connected to Fadecandy device");
      unsigned char serial_number[64];
      libusb_get_string_descriptor_ascii(dev_handle, fadecandy_device_descriptor.iSerialNumber, serial_number, 64);
      diagnostic_updater_.setHardwareID((char*)serial_number);
    }
    else
    {
      throw std::runtime_error("Device not found! Could not write on the driver.");
    }
  }
}

void FadecandyDriverRos::setLedsCallback(const fadecandy_msgs::LEDArrayConstPtr& led_array_msg)
{
  std::vector<std::vector<colors>> led_array_colors;
  std::vector<colors> led_strip_colors;
  for (size_t i = 0; i < led_array_msg->strips.size(); ++i)
  {
    led_strip_colors.clear();
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
    initialized = false;
    return;
  }
};
void FadecandyDriverRos::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  if (FadecandyDriver::fadecandy_device != NULL)
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
