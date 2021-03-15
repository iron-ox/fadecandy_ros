###############################################################################
# Find libusb
#
# This sets the following variables:
# LIBUSB_FOUND - True if LIBUSB was found.
# LIBUSB_INCLUDE_DIRS - Directories containing the LIBUSB include files.
# LIBUSB_LIBRARIES - Libraries needed to use LIBUSB.

message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
find_package(PkgConfig)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)

find_path(LIBUSB_INCLUDE_DIR libusb.h
    HINTS ${PC_LIBUSB_INCLUDEDIR} ${PC_LIBUSB_INCLUDE_DIRS})

find_library(LIBUSB_LIBRARY NAMES usb-1.0
    HINTS ${PC_LIBUSB_LIBDIR} ${PC_LIBUSB_LIBRARY_DIRS})

set(LIBUSB_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
set(LIBUSB_LIBRARIES ${LIBUSB_LIBRARY})
