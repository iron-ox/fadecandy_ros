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
#define INTERFACE_NO 0x01

#define LEDS_PER_STRIP 64
#define NUM_STRIPS 8

namespace fadecandy_driver {
//!
//! \brief The FadecandyDriver class
//!
class FadecandyDriver {
public:
  //!
  //! \brief The color struct contains the r,g and b colors
  //!
  struct colors {
    double r;
    double g;
    double b;
  };
  libusb_device *fadecandy_device = NULL;
  libusb_device_descriptor fadecandy_device_descriptor;
  libusb_context *context = NULL;
  libusb_device_handle *dev_handle = NULL;
  char *serial_number = NULL;
  //!
  //! \brief findUsbDevice search the fadcandy device with particular vendor and
  //! product id
  //!
  void findUsbDevice();

  //!
  //! \brief release fadecandy device interface
  //!
  void release();

  //!
  //! \brief initialize fadecandy device interface
  //!
  bool intialize();

  //!
  //! \brief initialize fadecandy device interface
  //!
  void setColors(std::vector<std::vector<colors>> led_colors);

private:
  //!
  //! \brief makeVideoUsbPackets constructs the USB packets to set all LED
  //! strips to the given colors.
  //! To simplify things, we always send values for all 8 * 64 LEDs. If the
  //! physical strips are shorter, or there are less then 8 strips, the extra
  //! data doesn't do anything.
  //! If the user gives us values for less than the total number of strips, or
  //! less than the total number of LEDs in any given strip, all unspecified
  //! LEDs are left dark.
  //!
  std::vector<std::vector<unsigned char>>
      makeVideoUsbPackets(std::vector<std::vector<colors>>);

  //!
  //! \brief makeLookupTablePackets creates USB packets for a simple color
  //! lookup table. The entire red lookup table comes first, then the entire
  //! green channel, then the entire red channel.
  //!

  std::vector<std::vector<unsigned char>>
  makeLookupTablePackets(std::vector<int> red_lookup_values,
                         std::vector<int> green_lookup_values,
                         std::vector<int> blue_lookup_values);

  //!
  //! \brief makeLookupTablePackets returns lookup tables as 3 lists of lookup
  //! values - one for the red channel, one for the green channel, and one for
  //! the blue channel.
  //!

  std::vector<int> makeDefaultLookupTable();
};

} // namespace fadecandy_driver
