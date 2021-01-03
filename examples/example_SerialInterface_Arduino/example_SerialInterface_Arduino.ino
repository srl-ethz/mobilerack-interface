/*
Sample sketch for Arduino on how to send data over serial. Use together with example_SerialInterface.cpp
Arduino resets after communication?
*/

// 6 * [4 bytes for float], [2 bytes for marking end of transmission]
#define DATA_SIZE 26

// data array to send
char data[DATA_SIZE];

// mock sensor values
float sensor1 = 0;
float sensor2 = 3.1415;
float sensor3 = 3;
float sensor4 = 4;
float sensor5 = 5;
float sensor6 = 6;

void setup() {
  Serial.begin(38400);
  
  // set footer bytes
  data[DATA_SIZE - 2] = '\r';
  data[DATA_SIZE - 1] = '\n';
}

void loop() {
  // set sensor values
  sensor1 += 1;
  sensor3 -= 1;

  // copy values to data array
  memcpy(&data[0], &sensor1, sizeof(sensor1));
  memcpy(&data[4], &sensor2, sizeof(sensor2));
  memcpy(&data[8], &sensor3, sizeof(sensor3));
  memcpy(&data[12], &sensor4, sizeof(sensor4));
  memcpy(&data[16], &sensor5, sizeof(sensor5));
  memcpy(&data[20], &sensor6, sizeof(sensor6));

  // send
  Serial.write(data, DATA_SIZE);
}
