/*
Sample sketch for Arduino on how to send data over serial. Use together with example_SerialInterface.cpp
Arduino resets after communication?
*/

float sensor_vals[6];

int num = sizeof(sensor_vals) / sizeof(sensor_vals[0]);

void setup() {
  // mock sensor values
  sensor_vals[0] = 0;
  sensor_vals[1] = 3.14;
  sensor_vals[2] = 2.7;
  sensor_vals[3] = 3;
  sensor_vals[4] = 4;
  sensor_vals[5] = 5;

  Serial.begin(38400);
}

void loop() {
  // set sensor values
  sensor_vals[0] += 1;

  Serial.print('a');
  // send
  for (int i = 0; i< num; i++){
    Serial.print(sensor_vals[i]);
    if (i != num-1)
      Serial.print(","); // comma is used for separator
  }
  Serial.println();
}
