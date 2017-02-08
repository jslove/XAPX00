# XAPX00
Python Module to control ClearOne Genter XAP 400/800 Audio Conferencing System


This module facilitates control of a XAP system connected to a serial port (probably through a USB connection.)

This code is adapted from a javascript library written by Dan Rudman.

The module was created to enable the use of an XAP 800 as part of a home audio system, so some enhancements have ben added to facilitate stereo inputs and outputs.  Otherwise it provides access to essentially all of the serial commands available on the system (and almost everything can be done through serial commands on this thing!)

The basic use is:

```
import XAPX00
xap = XAPX00.XAPX00("/dev/ttyUSB0")
xap.connect()


xap.getUniqueID() #or whatever
```

The XAP unit can be completely configured through this module, with a few small exceptions such as setting the unit#.  (I have not added code to handle multiple units gracefully, but it should work by specifying the unit number in each method call.)  However, it may be easiest to do initial configuration with the ClearOne G-Ware application.  There are a lot of nuanced settings that could be missed otherwise.


