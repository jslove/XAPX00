# XAPX00
Python Module to control ClearOne Converge Pro v1 or XAP 400/800 Audio Conferencing System

This module facilitates control of a CleanOne system connected to a telnet or serial port.

This code is adapted from a javascript library written by Dan Rudman.

The module was created to enable the use of an XAP 800 as part of a home audio system, so some enhancements have ben added to facilitate stereo inputs and outputs.  Otherwise it provides access to essentially all of the serial commands available on the system (and almost everything can be done through serial commands on this thing!)

The basic use is:

```
import XAPX00
xap = XAPX00.XAPX00("/dev/ttyUSB0")
xap.connect()


xap.getUniqueID() #or whatever
```

The XAP unit(s) can be completely configured through this module, with a few small exceptions such as setting the unit#.   However, it may be easiest to do initial configuration with the ClearOne G-Ware application.  There are a lot of nuanced settings that could be missed otherwise.

Using a network connection (natively on Converge or by a serial adaptre on XAP) is the easiest way to connect.  If a direct serial connection is used, you will want to make sure that your usb to serial connection is always in the same place.  The linux udev system offers a way to create a rule so that a device is given a peristent name.  This blog entry offers a good summary:
http://rolfblijleven.blogspot.nl/2015/02/howto-persistent-device-names-on.html




