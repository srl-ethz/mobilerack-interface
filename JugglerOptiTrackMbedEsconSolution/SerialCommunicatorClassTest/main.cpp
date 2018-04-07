#include <string>
#include <iostream>
#include <cstdio>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

//#include <windows.h>
//#include <math.h>
#include "SerialCommunicator.h"


int main()
{	
	SerialCommunicator serialComm;
	float rpmMeasured = 0;
	float rpmSent = 0;
	//serialComm.sendMotorRpm(0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(2));

	while (true)
	{
		serialComm.sendMotorRpm(rpmSent);
		//printf("Diff: %.4f, Meas: %.4f, Sent: %.4f \n", (rpmMeasured - rpmSent), rpmMeasured, rpmSent);
		if (rpmSent > RPM_MAX)
			rpmSent = -RPM_MAX;
		else
			rpmSent = rpmSent + 1;
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		rpmMeasured = serialComm.readMotorRpm();
	}
	return 0;
}