//
// Copyright (c) 2021 Eurotec
//

#include <array>
#include <libusb-1.0/libusb.h>
#include <vector>
#include <iostream>

constexpr int LEDS_PER_PACKET = 21;
constexpr int LOOKUP_VALUES_PER_PACKET = 31;
constexpr int LOOKUP_VALUES_PER_CHANNEL = 257;
constexpr int USB_PACKET_SIZE = 64;
constexpr int PACKET_TYPE_VIDEO = 0x00;
constexpr int PACKET_TYPE_LUT = 0x40;
constexpr int FINAL_PACKET_BIT = 0x20;

constexpr int USB_PRODUCT_ID = 0x607a;
constexpr int USB_VENDOR_ID = 0x1d50;
constexpr int USB_ENDPOINT = 1;
constexpr int INTERFACE_NO = 0x01;

constexpr int LEDS_PER_STRIP = 64;
constexpr int NUM_STRIPS = 8;

namespace fadecandy_driver
{
//!
//! \brief The FadecandyDriver class
//!
class FadecandyDriver
{
public:
  //!
  //! \brief The color struct contains the r,g and b colors
  //!
  struct Color
  {
    double r;
    double g;
    double b;
  };

  libusb_device* fadecandy_device_ = NULL;
  libusb_device_descriptor fadecandy_device_descriptor_;
  libusb_context* context_ = NULL;
  libusb_device_handle* dev_handle_ = NULL;
  std::string serial_number_;

  //!
  //! \brief findUsbDevice search the fadcandy device with particular vendor and
  //! product id
  //!
  void findUsbDevice();

  //!
  //! \brief releaseInterface release fadecandy device interface
  //!
  void releaseInterface();

  //!
  //! \brief initialize fadecandy device interface
  //!
  bool intialize();

  //!
  //! \brief initialize fadecandy device interface
  //!
  void setColors(std::vector<std::vector<Color>> led_colors);

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
  std::vector<std::vector<unsigned char>> makeVideoUsbPackets(const std::vector<std::vector<Color>>&);

  //!
  //! \brief makeLookupTablePackets creates USB packets for a simple color
  //! lookup table. The entire red lookup table comes first, then the entire
  //! green channel, then the entire red channel.
  //!
  std::vector<std::vector<unsigned char>> makeLookupTablePackets(const std::vector<int>& red_lookup_values,
                                                                 const std::vector<int>& green_lookup_values,
                                                                 const std::vector<int>& blue_lookup_values);
  //!
  //! \brief makeLookupTablePackets returns lookup tables as 3 lists of lookup
  //! values - one for the red channel, one for the green channel, and one for
  //! the blue channel.
  //!
  std::vector<int> makeDefaultLookupTable();
};

}  // namespace fadecandy_driver
