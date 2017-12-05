# PSVRD - An user space daemon for interfacing with a PS VR headset.

PSVRD is an user space driver for controlling the PlayStation VR headset. The PSVRD
driver is written as an user space driver by using libusb, and Unix domain sockets
for IPC communication to allow.

This driver is completely unofficial and does not have any kind of support from part
of Sony Computer Entertainment. USE AT YOUR OWN RISK

Any firmware update to the PS VR processing unit could render this software uselesss.

## Components
- psvrd - The psvrd daemon
- libpsvrd-client - Library to communicate with the daemon.
- psvrd_control - Utility for sending commands to the processing box. Written as a wrapper for libpsvrd-client

## psvrd_control commands

- headset [on|off] - Turns on or off the PS VR headset.
- cinematic - Enter into cinematic mode.
- vr - Enter into VR mode.
- recenter - Enter into VR mode.
- calibrate - Calibrate the sensors.
- poweroff - Turns off the PS VR processing unit box.
- readsensor - Starts reading the sensor data.

## Acknowledgements
We thank Agustín Gimenez Bernad for making PSVRFramework ( https://github.com/gusmanb/PSVRFramework ),
and giving us permission on using PSVRFramework as documentation for the low-level
USB protocol used by the PS VR processing unit box.
