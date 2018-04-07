#pragma once

#define maxStrSize 100
#define PI 3.14159265358979

#include "Definitions.h"
#include <mutex>

class MotorDriver
{
private:
	HANDLE keyHandle;
	DWORD m_ulErrorCode;
	WORD m_usNodeId = 1;

	char strDeviceName[maxStrSize];
	char strProtocolStackName[maxStrSize];
	char strInterfaceName[maxStrSize];
	char strPortName[maxStrSize];

	BOOL endOfSel = FALSE;
	DWORD errorCode = 0;

	__int8 m_bMode = -1;
	long m_lStartPosition = 0;
	double m_desiredMotorPositionRad;

	std::mutex mutexDesired;

	//char* strDeviceName = "EPOS4";
	//char* strProtocolStackName = "CANopen";
	//char* strInterfaceName = "USB";
	//char* strPortName = "USB0";

public:
	
	MotorDriver();
	~MotorDriver();

	void initMotor();

	void motor_control_thread_function(int const & motorSetPosition);

	long getStartPosition();
	
	void setDesiredMotorPosition(double const & desiredMotorPositionRad);

	void getDesiredMotorPosition(double* desiredMotorPositionRad);


protected:
	// protected methods
	void setStartPosition(long startPosition);

	BOOL getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration,
		DWORD* pProfileDeceleration, DWORD* pErrorCode);
	BOOL setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration,
		DWORD ProfileDeceleration, DWORD* pErrorCode);

	BOOL getPosition(long* pPosition);
	BOOL getPositionRad(double* pPositionRad);
	BOOL moveToPosition(long moveToPosition, BOOL Absolute, BOOL Immediately);
	BOOL moveToPositionRad(double moveToPositionRad, BOOL Absolute, BOOL Immediately);

	BOOL getMovementState(BOOL* pTargetReached, DWORD* pErrorCode);

	BOOL getProtocolSettings(DWORD* pBaudrate, DWORD* pTimeOut, DWORD* pErrorCode);

	long rad2qc(double angleInRadians);
	double qc2rad(long angleInQuadratureCount);

};