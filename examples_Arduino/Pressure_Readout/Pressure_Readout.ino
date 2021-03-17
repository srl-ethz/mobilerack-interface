#include <Wire.h>

uint32_t timer = 0;
bool run = 0;


void setup() {
  Wire.begin();
  Serial.begin(9600);

}



void loop() {
  switch (Serial.read()) {
    case '1': run=!run; break;                                  //write 1 to serial terminal to start/stop readouts
    case '2': timer=0; break;                                   //write 2 to serial terminal to reset timer
  }

  Wire.requestFrom(0x28,2);
  int byte1 = Wire.read();
  int byte2 = Wire.read();
  
  if (byte1>63) Serial.print("Sensor error, discard result");   //first two significant bits indicate error
  int n = byte1*256+byte2;                                      //conversion from singular bytes to 16-bit integer
  double presbar = (n/16384.0 - 0.1)*10/0.8;                    //conversion from value to SI units
                                                                //transfer function found on page 11 of honeywell SSC datasheet
                                                                //this is transfer function for pressure laid out for 0-10bar
  if (run) {
  Serial.print(timer);
  Serial.print(",");
  Serial.print(presbar*1000);                                   //measurements are printed to serial output in csv format: time[ms],pressure[mbar]
  Serial.println("");
  if (run) timer=timer+33;                                      //33ms is smallest timestep possible
  }
  delay(13);
}
