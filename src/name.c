#include "usb_names.h"

#define MIDI_NAME   {'A','X','-','8','0'}
#define MIDI_NAME_LEN  5

#define MANUFACTURER_NAME  {'C','r','a','i','g',' ','B','a','r','n','e','s'}
#define MANUFACTURER_NAME_LEN 12

struct usb_string_descriptor_struct usb_string_product_name = {
  2 + MIDI_NAME_LEN * 2,
  3,
  MIDI_NAME
};

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
  2 + MANUFACTURER_NAME_LEN * 2,
  3,
  MANUFACTURER_NAME
};
