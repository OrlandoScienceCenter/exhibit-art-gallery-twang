#include <BMI160Gen.h>
 
const int select_pin = 10;
const int i2c_addr = 0x69;
 
void setup() {
  Serial.begin(115200); // initialize Serial communication
 
  // initialize device
  //BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
}
 
void loop() {
  int gx, gy, gz;         // raw gyro values
 
  // read raw gyro measurements from device
  BMI160.readGyro(gx, gy, gz);
  BMI160.read
 
  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();
 
  delay(500);
}