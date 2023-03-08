# Board mac addresses

* Accelerometer data server ESP8266: B4:E6:2D:53:D5:79
* ESP8266 that communicates with the trinket that drives the LED strip: A0:20:A6:18:4E:3F


# Wiring:

BMI160 SCL = Wemos pin D1
BMI160 SDA = Wemos pin D2
BMI160 3v3 = Wemos pin 3v3
BMI160 GND = Wemos pin G

# Folders

basic_readings_working_with_wavgat

* Accelerometer + ESP8266 working example

esp8266_accelerometer_receiver

* This is the sketch for the esp8266 connected to the teensy that drives the LED strip

esp8266_accelerometer_sender

* This is the sketch for the ESP8266 housed in the white pedestal with the joystick on top, it sends the joystick data

get_esp8266_mac_address

* Shows the mac address of a esp8266 that this sketch is run on. Useful for setting up ESP-NOW

twang4809_teensey_accel_client_with_ledstrip_WORKING

* This sketch goes on the teensey. It gets data sent from the esp8266 in the pedestal, that is then received by the esp8266 with the esp8266_accelerometer_receiver sketch, which then finally gives the data to the teensey. It does not let you "steer" the green dot, but you can attack and it matches the signage.

twang4809_teensey_accel_client_with_ledstrip_in_progress

* In progress code. Do not use unless your name is David.