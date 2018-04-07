#include "motorDriver.h"
#include <thread>
#include <iostream>


MotorDriver::MotorDriver()
{
	char deviceName[maxStrSize] = "EPOS4";
	char protocolStackName[maxStrSize] = "MAXON SERIAL V2"; // Can be "MAXON_RS232", "MAXON SERIAL V2" OR "CANopen"
	char interfaceName[maxStrSize] = "USB"; // Can be "USB" "RS232"
	char portName[maxStrSize] = "USB0"; // Can also be COM1, COM2, ..., USB0, USB1, ..., CAN0, CAN1, ...

	// strDeviceName = "EPOS4";
	//strProtocolStackName = "CANopen"; // Can be "MAXON_RS232", "MAXON SERIAL V2" OR "CANopen"
	//strInterfaceName = "USB"; // Can be "USB" "RS232"
	//strPortName = "USB0"; // Can also be COM1, COM2, ..., USB0, USB1, ..., CAN0, CAN1, ...

	// get first device name
	if (VCS_GetDeviceNameSelection(TRUE, strDeviceName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next device name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetDeviceNameSelection(FALSE, strDeviceName, maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first protocol stack name
	if (VCS_GetProtocolStackNameSelection(strDeviceName, TRUE, strProtocolStackName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next protocol stack name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetProtocolStackNameSelection(strDeviceName, FALSE, strProtocolStackName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first interface name
	if (VCS_GetInterfaceNameSelection(strDeviceName, strProtocolStackName, TRUE, strInterfaceName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next interface name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetInterfaceNameSelection(strDeviceName, strProtocolStackName, FALSE, strInterfaceName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first port name
	if (VCS_GetPortNameSelection(strDeviceName, strProtocolStackName, strInterfaceName,
		TRUE, strPortName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next port name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetPortNameSelection(strDeviceName, strProtocolStackName, strInterfaceName,
				FALSE, strPortName, maxStrSize, &endOfSel, &errorCode);
		}
	}	

	// Open Device
	//keyHandle = VCS_OpenDevice(strDeviceName, strProtocolStackName, strInterfaceName,
		//strPortName, &errorCode);
	keyHandle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName,
		portName, &errorCode);

	// Set Enable State
	VCS_SetEnableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Set Operation Mode
	VCS_SetOperationMode(keyHandle, m_usNodeId, m_bMode, &m_ulErrorCode);

	initMotor();
}

MotorDriver::~MotorDriver()
{
	// Set Disable State
	VCS_SetDisableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Close Device
	VCS_CloseDevice(keyHandle, &errorCode);
}

void MotorDriver::initMotor()
{
	m_lStartPosition = 0;
	setDesiredMotorPosition(0);
}

long MotorDriver::getStartPosition()
{
	return m_lStartPosition;
}

void MotorDriver::setStartPosition(long startPosition)
{
	m_lStartPosition = startPosition;
}

BOOL MotorDriver::getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration, 
	DWORD* pProfileDeceleration, DWORD* pErrorCode)
{
	return VCS_GetPositionProfile(keyHandle, m_usNodeId, pProfileVelocity,
		pProfileAcceleration, pProfileDeceleration, pErrorCode);
}

BOOL MotorDriver::setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration, DWORD ProfileDeceleration, DWORD * pErrorCode)
{
	return VCS_SetPositionProfile(keyHandle, m_usNodeId, ProfileVelocity, 
		ProfileAcceleration, ProfileDeceleration, pErrorCode);
}

BOOL MotorDriver::getPosition(long* pPosition)
{
	return VCS_GetPositionIs(keyHandle, m_usNodeId, pPosition, &m_ulErrorCode);
}

BOOL MotorDriver::getPositionRad(double * pPositionRad)
{
	long posQC = 0;
	if (VCS_GetPositionIs(keyHandle, m_usNodeId, &posQC, &m_ulErrorCode))
	{
		*pPositionRad = qc2rad(posQC);
		return TRUE;
	}
	else
		return FALSE;
}


BOOL MotorDriver::moveToPosition(long moveToPosition, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, moveToPosition, Absolute, Immediately, &m_ulErrorCode);
}

BOOL MotorDriver::moveToPositionRad(double moveToPositionRad, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, rad2qc(moveToPositionRad), Absolute, Immediately, &m_ulErrorCode);
}

BOOL MotorDriver::getMovementState(BOOL * pTargetReached, DWORD * pErrorCode)
{
	return VCS_GetMovementState(keyHandle, m_usNodeId, pTargetReached, pErrorCode);
}

BOOL MotorDriver::getProtocolSettings(DWORD * pBaudrate, DWORD * pTimeOut, DWORD * pErrorCode)
{
	return VCS_GetProtocolStackSettings(keyHandle, pBaudrate, pTimeOut, pErrorCode);
}

long MotorDriver::rad2qc(double angleInRadians)
{
	return (long) (20000 / 2 / PI * (angleInRadians));
}

double MotorDriver::qc2rad(long angleInQuadratureCount)
{
	return (double) (2 * PI / 20000 * (angleInQuadratureCount));
}

void MotorDriver::motor_control_thread_function(int const & motorSetPosition)
{

	int & y = const_cast<int &>(motorSetPosition);
	y++;
	std::cout << "Motor Thread :: ID = " << std::this_thread::get_id() << std::endl;
	// call motor
	std::cout << "Motor Thread :: motorSetPosition = " << motorSetPosition << std::endl;
	for (int i = 0; i < 10000; i++);
	double newTargetPositionRad;
	// get the new target position
	this->getDesiredMotorPosition(&newTargetPositionRad);

	std::cout << "Motor Control Thread Finished" << std::endl;
}

void MotorDriver::setDesiredMotorPosition(double const & desiredMotorPositionRad)
{
	mutexDesired.lock();

	m_desiredMotorPositionRad = desiredMotorPositionRad;

	mutexDesired.unlock();

}

void MotorDriver::getDesiredMotorPosition(double * desiredMotorPositionRad)
{
	mutexDesired.lock();
	
	*desiredMotorPositionRad = m_desiredMotorPositionRad;

	mutexDesired.unlock();

}