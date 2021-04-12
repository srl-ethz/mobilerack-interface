/*
Sample sketch for Arduino on how to send and receive data over serial. Use together with example_SerialInterface.cpp
*/

float sensor_vals[6];

int num = sizeof(sensor_vals) / sizeof(sensor_vals[0]);
int led = 13;

void setup() {
  // mock sensor values
  sensor_vals[0] = 0;
  sensor_vals[1] = 3.14;
  sensor_vals[2] = 2.7;
  sensor_vals[3] = 3;
  sensor_vals[4] = 4;
  sensor_vals[5] = 5;

  pinMode(led, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  // set sensor values
  sensor_vals[0] += 1;

  // send data
  for (int i = 0; i< num; i++){
    Serial.print(sensor_vals[i]);
    if (i != num-1)
      Serial.print(","); // comma is used for separator
  }
  Serial.println();
  
  Serial.flush(); // wait until all data is sent

  // receive data
  if (Serial.available() > 0){
    char receivedByte = Serial.read();
    if (receivedByte == 'H')
      digitalWrite(led, HIGH);
    else if (receivedByte == 'L')
      digitalWrite(led, LOW);
  }
  delay(10);
}
