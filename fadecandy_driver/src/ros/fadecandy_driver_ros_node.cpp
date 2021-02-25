#include "../fadecandy_driver.h"
#include <iostream>
#include <libusb-1.0/libusb.h>

int main() {
  fadecandy_driver::FadecandyDriver fadecandy;
  //  fadecandy.findUsb();
  //  std::vector<int> arra;
  //  std::vector<fadecandy_driver::FadecandyDriver::colors> led_array_color;
  //  fadecandy.make_video_usb_packets(led_array_color);
  //  int size = sizeof(arra);
  //  std::cout << size << std::endl;
  //  std::cout << arra.size() << std::endl;
  //    arra = fadecandy.make_default_lookup_table();
  //    for (int row = 0; row < arra.size(); row++) {
  //      std::cout << arra.at(row) << std::endl;
  //    }
  //    fadecandy.make_lookup_table_packets(arra, arra, arra);
  std::vector<std::vector<fadecandy_driver::FadecandyDriver::colors>>
      all_led_colors;
  std::vector<fadecandy_driver::FadecandyDriver::colors> temp;

  for (int i = 0; i < 8; i++) {
    temp.clear();
    for (int j = 0; j < 21; j++) {
      temp.push_back({0, 1, 1});
    }
    all_led_colors.push_back(temp);
  }
  fadecandy.FadecandyDriver::make_video_usb_packets(all_led_colors);
  fadecandy.FadecandyDriver::intialize();

  return 0;
}
