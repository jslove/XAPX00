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





## MAXGAIN is enforced in software, here

The hardware does not enforce it. The Converge Pro 880 reference gives `GAIN`'s
only bound as the internal range (−65…20 dB), documents no interaction with
`MAX`, and gives `MAX` that same range. Written to −15.00 with `GAIN` left at
−7.50, the unit leaves the level above its own stated maximum.

So an absolute `setGain` or `setPropGain` above a channel's `MAXGAIN` is **held
at the ceiling and logged**, on inputs and outputs alike. The ceiling read is
cached, so this costs no extra round trip on the volume path.

**Relative writes are bounded too**, and are the case the ceiling matters most
for. A delta cannot be clamped as a delta: shrinking it so `current + delta`
lands exactly on the ceiling does not help, because `XAPCommand` retries after a
telnet no-response and a retried *relative* write applies its delta a second
time. So `isAbsolute=0` reads the current level, resolves the delta against it,
clamps, and writes **absolute** — which bounds it and makes the write idempotent,
so the retry that caused the problem becomes harmless. The cost is one extra
round trip, paid only by relative writes.

One thing this does not cover:

- **Anything that bypasses the library** — a raw command, G-Ware, the front
  panel — is unaffected. `GAIN` with no `A`/`R` token is *relative*, so
  `GAIN 7 O -15` sent by hand drops the channel 15 dB rather than setting it.
