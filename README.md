An ESP32-based electronic compass, based on a "ESP32 DEVKIT V1" board and an Adafruit BNO055 sensor board.
Outputs both NMEA0183 heading and NMEA2000 heading and attitude.  My hardware design also
includes a SN65VHD230 CAN Bus transceiver board and a 12-to-5 volt voltage regulator board.

Data can be read and various settings can be configured via BLE.
I use a simple BLE scanner app on my Android phone to access that data.

