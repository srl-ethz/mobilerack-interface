#pragma once

#define maxStrSize 100

#include "Definitions.h"

class motorDriver
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

public:
	motorDriver();
	~motorDriver();

	long getStartPosition();
	void setStartPosition(long startPosition);

	BOOL getPosition(long* pPosition);
	BOOL moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately);

};