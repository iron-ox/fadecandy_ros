#include <array>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <vector>

#define LEDS_PER_PACKET 21
#define LEDS_PER_PACKET 21
#define LOOKUP_VALUES_PER_PACKET 31
#define LOOKUP_VALUES_PER_CHANNEL 257
#define USB_PACKET_SIZE 64
#define PACKET_TYPE_VIDEO 0x00
#define PACKET_TYPE_LUT 0x40
#define FINAL_PACKET_BIT 0x20

#define USB_PRODUCT_ID 0x607a
#define USB_VENDOR_ID 0x1d50
#define USB_ENDPOINT 1

#define LEDS_PER_STRIP 64
#define NUM_STRIPS 8

namespace fadecandy_driver {
//!
//! \brief The FadecandyDriver class
//!
class FadecandyDriver {
public:
  struct colors {
    double r;
    double g;
    double b;
  };
  libusb_device *fadecandy_device = NULL;
  libusb_device_descriptor fadecandy_device_descriptor;
  libusb_context *context = NULL;
  libusb_device_handle *dev_handle = NULL;

  void findUsb();
  void release();
  std::vector<std::vector<unsigned char>>
      make_video_usb_packets(std::vector<std::vector<colors>>);
  std::vector<std::vector<unsigned char>>
  make_lookup_table_packets(std::vector<int> red_lookup_values,
                            std::vector<int> green_lookup_values,
                            std::vector<int> blue_lookup_values);
  std::vector<int> make_default_lookup_table();
  void set_colors(std::vector<std::vector<colors>> led_colors);
  void intialize();
};

} // namespace fadecandy_driver
