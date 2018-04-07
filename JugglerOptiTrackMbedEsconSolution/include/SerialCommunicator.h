#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include <string>
#include "serial/serial.h"

using std::string;

//#include <mutex>

#define RPM_MAX 10000.0f
#define SHORTSIZE 2
#define MESSAGESIZE SHORTSIZE+1
#define MESSAGESIZE_WRITE MESSAGESIZE
#define LOOPCOUNTS_INT 300

typedef union {
    uint16_t unsignedShort;
    uint8_t binary[SHORTSIZE];
} binaryUShort;

class SerialCommunicator
{
  public:
    SerialCommunicator();
    ~SerialCommunicator();
	
	void sendMotorRadPerSec(float rad_s);
	void sendMotorRpm(float rpm);
	float readMotorRpm();
	float readMotorRadPerSec();

    void enumerate_ports();
    void print_usage();

	static uint16_t convertRadSToMessage(float rad_per_second);
	static uint16_t convertRpmToMessage(float rpm);
	static float convertMessageToRpm(uint16_t message);
	static float convertMessageToRadS(uint16_t message);


protected:
	//void readThread();
	void sendMessage(uint16_t message);
	void getNewMessage();

  private:
    std::string port;
    unsigned long baud;
    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial;

    binaryUShort sentNumber;
    binaryUShort receivedNumber;

    uint8_t incomingData[1030];
    uint8_t endMessage = 0x0A;

	//std::chrono::milliseconds duraWrite(LOOPRATE_MS);
	//my_serial.flushOutput();

	uint8_t bytesToBeSent[MESSAGESIZE_WRITE];
	binaryUShort sentNumberLast;
	uint64_t writeCount;
	size_t bytes_wrote;
	size_t whatIsAvailable;
	size_t bytes_read;
	uint64_t readCount;
	//mutex mutexRec;
	//std::chrono::microseconds durationReadThreadSleepUS;

};

#endif
