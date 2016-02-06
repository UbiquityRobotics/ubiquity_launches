#!/usr/bin/env python

import os
import os.path

def main():
    is_raspberry_pi = os.path.exists("/dev/ttyAMA0")
    #print("is_raspberry_pi={0}".format(is_raspberry_pi))
    has_usb_serial = os.path.exists("/dev/ttyUSB0")
    #print("has_usb_serial={0}".format(has_usb_serial))

    # This is really kudgey (technical term) for now:
    platform = "stage"
    if is_raspberry_pi:
	platform = "loki"
	if has_usb_serial:
	    platform = "magni"
    print("robot_base:={0}".format(platform))

if __name__ == "__main__":
    main()
