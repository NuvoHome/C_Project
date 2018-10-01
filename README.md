# Nuvo Presence Sensor
(Description)
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
