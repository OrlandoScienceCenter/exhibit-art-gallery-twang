# Board mac addresses
 
* Attached to LED strip: 40:22:D8:EA:1F:98
* Sender in pedestal:  40:22:D8:EA:4B:E8

# Wiring:

## LED Strip

* Attach the green wire from the LED strip to pin D13 on the ESP32
* Attach the yellow wire from the LED strip to pin D14 on the ESP32

# Accelerometer/IMU

BMI160

## IMU Wiring

* BMI160 GND = ESP32 pin Gnd (Black wire on breadboard)  [Orange/White from accelerometer]
* BMI160 SCL = ESP32 pin D22 (Green wire on breadboard)  [Green Solid from accelerometer]
* BMI160 SDA = ESP32 pin D21 (Blue wire on breadboard)   [Green/White from accelerometer]
* BMI160 3v3 = ESP32 pin 3v3 (Yellow wire on breadboard) [Orange from accelerometer]

# Folders

twang_for_esp32_on_ledstrip

* This sketch goes on an ESP32 attached to the LED strip. It gets data sent from the esp32 in the pedestal.

accelerometer_sender_for_esp32_in_pedestal

* This is the sketch for the ESP32 housed in the white pedestal with the joystick on top, it sends the joystick data

get_esp_mac_address

* Shows the mac address of a esp8266 or ESP32 that this sketch is run on. Useful for setting up ESP-NOW


# Libraries

* https://github.com/hanyazou/BMI160-Arduino
* https://github.com/RobTillaart/RunningMedian
* https://github.com/FastLED/FastLED

# Serial

All boards programmed by David are 115200 baud in the Serial Monitor in the Arduino IDE

The ESP-32 (x2) in line wobbler need a USB A to C cable to connect to a computer. A USB C > C cable will not work.