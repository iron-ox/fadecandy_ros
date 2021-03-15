//
// Copyright (c) 2020 Eurotec
//
// @author Jad Haj Mustafa
//

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <list>
#include <math.h>
#include <vector>

#include "./fadecandy_driver.h"

namespace fadecandy_driver
{
std::vector<unsigned char> intToCharArray(int in, const size_t bytes_per_int)
{
  if (in > pow(2, bytes_per_int * 8))
    throw std::overflow_error("Overflow error while converting integer " + std::to_string(in) + " to char array of " +
                              std::to_string(bytes_per_int) + " bytes");
  unsigned char buffer[bytes_per_int];
  std::vector<unsigned char> char_array;
  for (size_t i = 0; i < bytes_per_int; i++)
  {
    size_t shift = 8 * (bytes_per_int - 1 - i);
    buffer[i] = (in >> shift) & 0xff;
    char_array.push_back(buffer[i]);
  }
  std::reverse(char_array.begin(), char_array.end());
  return char_array;
}

void FadecandyDriver::findUsbDevice()
{
  libusb_device** list = nullptr;
  int rc = 0;
  unsigned count = 0;

  rc = libusb_init(&context_);
  if (rc < 0)
  {
    throw std::runtime_error("Could not create USB session.");
  }
  count = libusb_get_device_list(context_, &list);
  for (size_t idx = 0; idx < count; ++idx)
  {
    libusb_device* device = list[idx];
    libusb_device_descriptor desc = { 0 };

    rc = libusb_get_device_descriptor(device, &desc);
    if (rc < 0)
    {
      throw std::runtime_error("Could not get device descriptor.");
    }
    if (desc.idVendor == USB_VENDOR_ID && desc.idProduct == USB_PRODUCT_ID)
    {
      fadecandy_device_ = device;
      fadecandy_device_descriptor_ = desc;
    }
  }
  libusb_free_device_list(list, count);
  libusb_exit(context_);
}

std::vector<std::vector<unsigned char>>
FadecandyDriver::makeVideoUsbPackets(const std::vector<std::vector<Color>>& led_array_colors)
{
  int led_index = 0;
  std::vector<Color> all_led_colors(LEDS_PER_STRIP * NUM_STRIPS, { 0, 0, 0 });
  for (size_t i = 0; i < led_array_colors.size(); i++)
  {
    for (size_t j = 0; j < led_array_colors[i].size(); j++)
    {
      led_index = (i * LEDS_PER_STRIP) + j;
      all_led_colors[led_index] = led_array_colors[i][j];
    }
  }
  std::vector<std::vector<unsigned char>> packets;
  std::vector<Color> packet_leds;
  int control;

  while (all_led_colors.size() > 0)
  {
    std::vector<int> color_bytes;
    std::vector<unsigned char> packet;

    if (all_led_colors.size() < LEDS_PER_PACKET)
    {
      packet_leds.assign(all_led_colors.begin(), all_led_colors.end());
      all_led_colors.erase(all_led_colors.begin(), all_led_colors.end());
    }
    else
    {
      packet_leds.assign(all_led_colors.begin(), all_led_colors.begin() + LEDS_PER_PACKET);
      all_led_colors.erase(all_led_colors.begin(), all_led_colors.begin() + LEDS_PER_PACKET);
    }

    control = packets.size() | PACKET_TYPE_VIDEO;
    if (all_led_colors.size() == 0)
    {
      control |= FINAL_PACKET_BIT;
    }

    for (size_t i = 0; i < packet_leds.size(); i++)
    {
      color_bytes.push_back(packet_leds[i].r);
      color_bytes.push_back(packet_leds[i].g);
      color_bytes.push_back(packet_leds[i].b);
    }
    // construnt USB packet and leave the first byte for the control byte
    if ((USB_PACKET_SIZE - 1) - color_bytes.size() > 0)
    {
      int j = (USB_PACKET_SIZE - 1) - color_bytes.size();
      for (int i = 0; i < j; i++)
      {
        color_bytes.push_back(0);
      }
    }

    for (size_t i = 0; i < color_bytes.size(); i++)
    {
      int bytes_per_int = 1;
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(color_bytes[i], bytes_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }

    // add control byte
    packet.insert(packet.begin(), static_cast<unsigned char>(control));

    assert(packet.size() == USB_PACKET_SIZE);
    packets.push_back(packet);
  }
  return packets;
}

std::vector<std::vector<unsigned char>> FadecandyDriver::makeLookupTablePackets(
    const std::vector<int>& red_lookup_values, const std::vector<int>& green_lookup_values,
    const std::vector<int>& blue_lookup_values)
{
  assert(red_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  assert(green_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  assert(blue_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  std::vector<std::vector<unsigned char>> packets;
  std::vector<int> remaining_lookup_values;
  std::vector<int> packet_lookup_values;
  int control;

  remaining_lookup_values.insert(remaining_lookup_values.begin(), red_lookup_values.begin(), red_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(), green_lookup_values.begin(), green_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(), blue_lookup_values.begin(), blue_lookup_values.end());

  while (remaining_lookup_values.size() > 0)
  {
    std::vector<unsigned char> packet;

    if (remaining_lookup_values.size() < LOOKUP_VALUES_PER_PACKET)
    {
      packet_lookup_values.assign(remaining_lookup_values.begin(), remaining_lookup_values.end());
      remaining_lookup_values.erase(remaining_lookup_values.begin(), remaining_lookup_values.end());
    }
    else
    {
      packet_lookup_values.assign(remaining_lookup_values.begin(),
                                  remaining_lookup_values.begin() + LOOKUP_VALUES_PER_PACKET);
      remaining_lookup_values.erase(remaining_lookup_values.begin(),
                                    remaining_lookup_values.begin() + LOOKUP_VALUES_PER_PACKET);
    }
    control = packets.size() | PACKET_TYPE_LUT;
    if (remaining_lookup_values.size() == 0)
    {
      control |= FINAL_PACKET_BIT;
    }

    if (LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size() > 0)
    {
      int j = LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size();
      for (int i = 0; i < j; i++)
      {
        packet_lookup_values.push_back(0);
      }
    }

    for (size_t i = 0; i < packet_lookup_values.size(); i++)
    {
      int bytes_per_int = 2;
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(packet_lookup_values[i], bytes_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }
    // add control byte
    packet.insert(packet.begin(), static_cast<unsigned char>(0));
    packet.insert(packet.begin(), static_cast<unsigned char>(control));

    assert(packet.size() == USB_PACKET_SIZE);
    packets.push_back(packet);
  }
  return packets;
}

std::vector<int> FadecandyDriver::makeDefaultLookupTable()
{  // color correction curve borrowed from the USB example in the main fadecandy repo:
  //
  // https://github.com/scanlime/fadecandy/blob/master/examples/python/usb-lowlevel.py
  //
  std::vector<int> lookup_values;
  for (int row = 0; row < 257; row++)
  {
    lookup_values.push_back(std::min(0xFFFF, int(pow(row / 256.0, 2.2) * 0x10000)));
  }
  return lookup_values;
}

void FadecandyDriver::setColors(std::vector<std::vector<Color>> led_colors)
{
  if (fadecandy_device_ != NULL)
  {
    uint r = libusb_init(&context_);
    int actual_written;
    const int timeout = 10000;
    std::vector<std::vector<unsigned char>> usb_packets = FadecandyDriver::makeVideoUsbPackets(led_colors);
    for (size_t i = 0; i < usb_packets.size(); i++)
    {
      r = libusb_bulk_transfer(dev_handle_, USB_ENDPOINT, usb_packets[i].data(), USB_PACKET_SIZE, &actual_written,
                               timeout);
      if (r != 0 || actual_written != USB_PACKET_SIZE)
      {
        throw std::runtime_error("Could not write on the driver.");
      }
    }
  }
  else
  {
    throw std::runtime_error("Device not found! Could not write on the driver.");
  }
}

void FadecandyDriver::releaseInterface()
{
  uint r = libusb_init(&context_);
  r = libusb_release_interface(dev_handle_, INTERFACE_NO);
  if (r < 0)
  {
    throw std::runtime_error("Could not release device.");
  }
  libusb_close(dev_handle_);
  libusb_exit(context_);
}

bool FadecandyDriver::intialize()
{
  // Find usb device.
  findUsbDevice();
  if (!fadecandy_device_)
  {
    throw std::runtime_error("Could not find device.");
  }
  uint r = libusb_init(&context_);
  if (r < 0)
  {
    throw std::runtime_error("Could not create USB session.");
  }
  dev_handle_ = libusb_open_device_with_vid_pid(context_, USB_VENDOR_ID, USB_PRODUCT_ID);

  if (dev_handle_ == NULL)
  {
    throw std::runtime_error("Could not open device.");
  }

  // Check if kernel driver, detach
  if (libusb_kernel_driver_active(dev_handle_, INTERFACE_NO) == 1)
  {
    if (libusb_detach_kernel_driver(dev_handle_, INTERFACE_NO) != 0)
    {
      throw std::runtime_error("Could not detach kernel driver.");
    }
  }

  // Claim interface
  r = libusb_claim_interface(dev_handle_, INTERFACE_NO);
  if (r < 0)
  {
    throw std::runtime_error("Could not claim device interface.");
  }

  unsigned char serial[64];
  libusb_get_string_descriptor_ascii(dev_handle_, fadecandy_device_descriptor_.iSerialNumber, serial, USB_PACKET_SIZE);
  serial_number_ = reinterpret_cast<char*>(serial);

  // Prepare command
  int actual_written;
  const int timeout = 10000;

  std::vector<int> array = makeDefaultLookupTable();
  std::vector<std::vector<unsigned char>> packets = makeLookupTablePackets(array, array, array);
  for (size_t i = 0; i < packets.size(); i++)
  {
    r = libusb_bulk_transfer(dev_handle_, USB_ENDPOINT, packets[i].data(), USB_PACKET_SIZE, &actual_written, timeout);
    if (r != 0 && actual_written != USB_PACKET_SIZE)
    {
      throw std::runtime_error("Failed to write data on device.");
    }
  }
  return true;
}
}  // namespace fadecandy_driver
