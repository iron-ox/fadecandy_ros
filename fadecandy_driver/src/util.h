#include <vector>

namespace fadecandy_driver
{
//!
//! \brief The color struct contains the r,g and b colors
//!
struct Color
{
  Color(int r, int g, int b);

  int r_;
  int g_;
  int b_;
};

//!
//! \brief makeVideoUsbPackets Construct the USB packets to set all LED
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
//! \brief makeLookupTablePackets Create USB packets for a simple color
//! lookup table. The entire red lookup table comes first, then the entire
//! green channel, then the entire red channel.
//!
std::vector<std::vector<unsigned char>> makeLookupTablePackets(const std::vector<int>& red_lookup_values,
                                                               const std::vector<int>& green_lookup_values,
                                                               const std::vector<int>& blue_lookup_values);
//!
//! \brief makeLookupTablePackets Return lookup tables as 3 lists of lookup
//! values - one for the red channel, one for the green channel, and one for
//! the blue channel.
//!
std::vector<int> makeDefaultLookupTable();
}  // namespace fadecandy_driver
