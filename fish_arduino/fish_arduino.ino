
/*
 Script to sync festo valve data with LED
 and to record data from i2c load cell amplifier.
*/

#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int LED = 12;
int starttime = 0;

// calibration values
float calibration_offset = -0.12;
float calibration_sensitivity = -5.3781e-6;

bool waitForSync = true;

HX711 scale;

void setup() {
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.println("Setup done");
  starttime = millis();
}

void loop() {
  handleSerial();
}

void handleSerial() {
  while (Serial.available() > 0) {
    char incomingCharacter = Serial.read();
    switch (incomingCharacter) {
    case 's':
      waitForSync = false;
      digitalWrite(LED, HIGH);
      starttime = millis();
      break;
    case 'e':
      digitalWrite(LED, LOW);
      break;
    }
  }
  // read from load cell and write to serial
  if (scale.is_ready()) {
    long reading = scale.read();
    float force = reading*calibration_sensitivity + calibration_offset;
    int reading_time = millis()-starttime;
    Serial.print(force, 4);
    Serial.println(reading_time);
  }
}
