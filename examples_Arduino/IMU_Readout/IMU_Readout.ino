#include <Wire.h>
#include <Adafruit_Sensor.h> //install in library manager, name "Adafruit Unified Sensor"
#include <Adafruit_BNO055.h> //install in library manager, name "Adafruit BNO055"
#include <utility/imumaths.h>

void printQuat(imu::Quaternion quat);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

void setup() {
  Serial.begin(9600);
  if(!bno1.begin() or !bno2.begin()) Serial.print("Failed to initialize");

  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true); //higher accuracy
}

void loop() {

  imu::Quaternion quat1 = bno1.getQuat();
  imu::Quaternion quat2 = bno2.getQuat();
  Serial.print("Quaternion of IMU 1:");                         //address 0x28
  printQuat(quat1);
  Serial.print("Quaternion of IMU 2:");                         //address 0x29
  printQuat(quat2);  
  delay(100);                                                   //sensor runs at 100Hz, minimal readout time 10ms
}


void printQuat(imu::Quaternion quat){                           //prints the quaternion to serial in csv: w,x,y,z
  
  Serial.print(quat.w(),2);
  Serial.print(",");
  Serial.print(quat.x(),2);
  Serial.print(",");
  Serial.print(quat.y(),2);
  Serial.print(",");
  Serial.println(quat.z(),2);

 
}
