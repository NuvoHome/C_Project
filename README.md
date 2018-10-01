[![Build Status](https://travis-ci.org/NuvoHome/notify_c.svg?branch=master)](https://travis-ci.org/NuvoHome/notify_c)
# Nuvo Presence Sensor
The Notify Presence sensor can detect occupancy in a room with up to 99.99% accuracy.

The cloud-based sensor is also constantly learning to identify different activities occuring in its environment. These activties can be used to build robust automations for your smart home.
# Quick Start
* [PlatformIO](https://platformio.org/) for VSCode
``` 
git clone https://github.com/NuvoHome/notify_c.git
cd notify_c
platformio lib install https://github.com/256dpi/lwmqtt.git
```
Edit the upload_port in the platformio.ini file to the serial port your esp32 device is connected to.
```
platformio run
```
Note: if the pio or platformio command fails in terminal check your PATH to make sure it includes PlaformIO
