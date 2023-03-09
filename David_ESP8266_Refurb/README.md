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

# Libraries

https://github.com/hanyazou/BMI160-Arduino

# Accelerometer Values:

These are known values from some tests of the joystick. Using these to update the code so that the joystick controls the player dot correctly:

Directions are given as if you're directly facing the wall with exhibit mounted:


Joystick straight up parallel with wall:

aX: 6241, aY: -15578, aZ: 707
gX: 6251, gY: -15567, gZ: 740

Joystick bent right 90 to parallel with floor:

aX: 14775, aY: 5880, aZ: -4572
gX: 14775, gY: 5880, gZ: -4572

Joystick bent left 90 to parallel with floor:

aX: -14657, aY: -3578, aZ: 5921
gX: -62, gY: 139, gZ: -352

Joystick bent towards wall 90 to parallel with floor: (Impossible when mounted)

aX: -75, aY: 1603, aZ: 16597
gX: 107, gY: 33, gZ: 126

Joystick bent away from wall 90 to parallel with floor:

aX: -3946, aY: -1340, aZ: -15039
gX: -127, gY: 88, gZ: -132

While wobbling 1:

aX: 10949, aY: 6705, aZ: -3829
gX: 10949, gY: 6705, gZ: -3829

While wobbling 2:

aX: 12365, aY: 7279, aZ: -3253
gX: 12365, gY: 7279, gZ: -3253

While wobbling 3:

aX: 19980, aY: 13460, aZ: -10230
gX: 14092, gY: 7649, gZ: -4139

TEST 01:

accelGyroData.gx = 15000;
accelGyroData.gy = 0;
accelGyroData.gz = 0;

accelGyroData.ax = 100;
accelGyroData.ay = 100;
accelGyroData.az = 100;

RESULT: Dot is still

TEST 02:

accelGyroData.gx = 000;
accelGyroData.gy = 15000;
accelGyroData.gz = 0;

accelGyroData.ax = 100;
accelGyroData.ay = 100;
accelGyroData.az = 100;

RESULT: Dot moves to the right

TEST 03:

accelGyroData.gx = 0;
accelGyroData.gy = 0;
accelGyroData.gz = 15000;

accelGyroData.ax = 100;
accelGyroData.ay = 100;
accelGyroData.az = 100;

RESULT: 