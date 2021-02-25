#include "./fadecandy_driver.h"
#include <cassert>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <list>
#include <math.h>
#include <vector>

#define BITS 8
#include <cstring>
#include <iomanip>
#define INTERFACE_NO 0x01
namespace fadecandy_driver {

std::vector<unsigned char> intToCharArray(unsigned char *buffer, int in,
                                          const size_t byts_per_int) {
  std::vector<unsigned char> char_array;
  for (size_t i = 0; i < byts_per_int; i++) {
    size_t shift = 8 * (byts_per_int - 1 - i);
    buffer[i] = (in >> shift) & 0xff;
    char_array.push_back(buffer[i]);
  }
  return char_array;
}

void FadecandyDriver::findUsb() {
  libusb_device **list = NULL;
  int rc = 0;
  unsigned count = 0;

  rc = libusb_init(&context);
  assert(rc == 0);

  count = libusb_get_device_list(context, &list);
  assert(count > 0);

  for (size_t idx = 0; idx < count; ++idx) {
    libusb_device *device = list[idx];
    libusb_device_descriptor desc = {0};

    rc = libusb_get_device_descriptor(device, &desc);
    assert(rc == 0);
    if (desc.idVendor == USB_VENDOR_ID && desc.idProduct == USB_PRODUCT_ID) {
      fadecandy_device = device;
      fadecandy_device_descriptor = desc;
      printf("Vendor:Device = %04x:%04x\n", desc.idVendor, desc.idProduct);
    }
  }
}

std::vector<std::vector<unsigned char>> FadecandyDriver::make_video_usb_packets(
    std::vector<std::vector<colors>> led_array_colors) {
  int led_index = 0;
  std::vector<colors> all_led_colors(LEDS_PER_STRIP * NUM_STRIPS, {0, 0, 0});
  for (size_t i = 0; i < led_array_colors.size(); i++) {
    for (size_t j = 0; j < led_array_colors[i].size(); j++) {

      led_index = (i * LEDS_PER_STRIP) + j;
      all_led_colors[led_index] = led_array_colors[i][j];
    }
  }
  assert(all_led_colors.size() == LEDS_PER_STRIP * NUM_STRIPS);
  std::vector<std::vector<unsigned char>> packets;
  std::vector<unsigned char> packet;
  std::vector<colors> packet_leds;
  std::vector<colors> remaining_leds = all_led_colors;
  std::vector<int> color_bytes;
  int control;

  while (remaining_leds.size() > 0) {
    color_bytes.clear();
    packet.clear();

    if (remaining_leds.size() < LEDS_PER_PACKET) {
      packet_leds.assign(remaining_leds.begin(), remaining_leds.end());
      remaining_leds.erase(remaining_leds.begin(), remaining_leds.end());
    } else {
      packet_leds.assign(remaining_leds.begin(),
                         remaining_leds.begin() + LEDS_PER_PACKET);
      remaining_leds.assign(remaining_leds.begin() + LEDS_PER_PACKET,
                            remaining_leds.end());
    }

    control = packets.size() | PACKET_TYPE_VIDEO;
    if (remaining_leds.size() == 0)
      control |= FINAL_PACKET_BIT;

    for (size_t i = 0; i < packet_leds.size(); i++) {
      color_bytes.push_back(packet_leds[i].r);
      color_bytes.push_back(packet_leds[i].g);
      color_bytes.push_back(packet_leds[i].b);
    }

    if (63 - color_bytes.size() > 0) {
      int j = 63 - color_bytes.size();
      for (int i = 0; i < j; i++) {
        color_bytes.push_back(0);
      }
    }

    color_bytes.insert(color_bytes.begin(), control);

    for (size_t i = 0; i < color_bytes.size(); i++) {
      int byts_per_int = 1;
      unsigned char buffer[byts_per_int];
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(buffer, color_bytes[i], byts_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }

    assert(packet.size() == USB_PACKET_SIZE);
    packets.push_back(packet);
  }
  return packets;
}

std::vector<std::vector<unsigned char>>
FadecandyDriver::make_lookup_table_packets(
    std::vector<int> red_lookup_values, std::vector<int> green_lookup_values,
    std::vector<int> blue_lookup_values) {
  assert(red_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  assert(green_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  assert(blue_lookup_values.size() == LOOKUP_VALUES_PER_CHANNEL);
  std::vector<unsigned char> packet;
  std::vector<std::vector<unsigned char>> packets;
  std::vector<int> remaining_lookup_values;
  std::vector<int> packet_lookup_values;
  int control;

  remaining_lookup_values.insert(remaining_lookup_values.begin(),
                                 red_lookup_values.begin(),
                                 red_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(),
                                 red_lookup_values.begin(),
                                 red_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(),
                                 red_lookup_values.begin(),
                                 red_lookup_values.end());

  while (remaining_lookup_values.size() > 0) {
    packet.clear();
    if (remaining_lookup_values.size() < LOOKUP_VALUES_PER_PACKET) {
      packet_lookup_values.assign(remaining_lookup_values.begin(),
                                  remaining_lookup_values.end());
      remaining_lookup_values.erase(remaining_lookup_values.begin(),
                                    remaining_lookup_values.end());
    } else {
      packet_lookup_values.assign(remaining_lookup_values.begin(),
                                  remaining_lookup_values.begin() +
                                      LOOKUP_VALUES_PER_PACKET);
      remaining_lookup_values.assign(remaining_lookup_values.begin() +
                                         LOOKUP_VALUES_PER_PACKET,
                                     remaining_lookup_values.end());
    }
    control = packets.size() | PACKET_TYPE_LUT;
    if (remaining_lookup_values.size() == 0)
      control |= FINAL_PACKET_BIT;
    if (LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size() > 0) {
      int j = LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size();
      for (int i = 0; i < j; i++) {
        packet_lookup_values.push_back(0);
      }
    }

    // add control byte
    packet_lookup_values.insert(packet_lookup_values.begin(), control);

    for (size_t i = 0; i < packet_lookup_values.size(); i++) {
      int byts_per_int = 2;
      unsigned char buffer[byts_per_int];
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(buffer, packet_lookup_values[i], byts_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }
    assert(packet.size() == USB_PACKET_SIZE);
    packets.push_back(packet);
  }
  return packets;
}

std::vector<int> FadecandyDriver::make_default_lookup_table() {
  std::vector<int> lookup_values;
  for (int row = 0; row < 257; row++) {
    lookup_values.push_back(
        std::min(0xFFFF, int(pow(row / 256.0, 2.2) * 0x10000)));
  }
  return lookup_values;
}

void FadecandyDriver::set_colors(std::vector<std::vector<colors>> led_colors) {
  uint r = libusb_init(&context);
  int actual_written;

  std::vector<std::vector<unsigned char>> usb_packets =
      FadecandyDriver::make_video_usb_packets(led_colors);
  for (size_t i = 0; i < usb_packets.size(); i++) {
    r = libusb_bulk_transfer(dev_handle, USB_ENDPOINT, usb_packets[i].data(),
                             64, &actual_written, 10000);
    if (r == 0 && actual_written == 64) {
      std::cout << "Succesfully written!";
    } else {
      std::cout << "||" << r << "||" << actual_written << "||"
                << "Could not write.";
    }
  }
}

void FadecandyDriver::release() {
  // Releasing interface
  uint r = libusb_init(&context);
  r = libusb_release_interface(dev_handle, INTERFACE_NO);
  if (r < 0) {
    printf("Could not release interface.");
  }
  printf("Interface released.");

  libusb_close(dev_handle);
  libusb_exit(context);
}
void FadecandyDriver::intialize() {
  // Find usb device.
  findUsb();
  if (!fadecandy_device) {
    printf("No Fadecandy interfaces found");
    throw;
  }
  uint r = libusb_init(&context);
  if (r < 0) {
    printf("Error");
  }
  //  libusb_set_debug(context, 4);
  dev_handle =
      libusb_open_device_with_vid_pid(context, USB_VENDOR_ID, USB_PRODUCT_ID);

  if (dev_handle == NULL) {
    printf("Could not open device.");
  }

  // Check if kernel driver, detach
  if (libusb_kernel_driver_active(dev_handle, INTERFACE_NO) == 1) {
    std::cout << "Kernel Driver Active";
    if (libusb_detach_kernel_driver(dev_handle, INTERFACE_NO) == 0) {
      std::cout << "Kernel Driver Detached";
    }
  }
  // Claim interface
  r = libusb_claim_interface(dev_handle, INTERFACE_NO);
  if (r < 0) {
    std::cout << "Could not claim interface.";
  }
  printf("Interface claimed.");
  // Prepare command
  int actual_written;

  std::vector<int> array = make_default_lookup_table();
  std::vector<std::vector<unsigned char>> packets =
      make_lookup_table_packets(array, array, array);

  for (size_t i = 0; i < packets.size(); i++) {
    r = libusb_bulk_transfer(dev_handle, USB_ENDPOINT, packets[i].data(), 64,
                             &actual_written, 10000);
    if (r == 0 && actual_written == 64) {
      std::cout << "Succesfully written!";
    } else {
      std::cout << "||" << r << "||" << actual_written << "||"
                << "Could not write.";
    }
  }
}
} // namespace fadecandy_driver
