#pragma once

#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>

#include "NatNetTypes.h"
#include "NatNetClient.h"

#pragma warning( disable : 4996 )

class OptiVision
{
private:
	NatNetClient* theClient;
	FILE* fp;
	
	unsigned int MyServersDataPort = 3130;
	unsigned int MyServersCommandPort = 3131;
	int iConnectionType = ConnectionType_Multicast;

	char szMyIPAddress[128] = "";
	char szServerIPAddress[128] = "";

	int analogSamplesPerMocapFrame = 0;

public:
	OptiVision(int argc, char *argv[]);
	~OptiVision();

	int CreateClient(int iConnectionType);
	void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
	void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
	void _WriteFooter(FILE* fp);

	void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
	// void (OptiVision::*DataHandlerHandle)(); // <- declare by saying what class it is a pointer to

	friend void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
	void resetClient();

	void acquireFrame(int iResult, void* response, int nBytes);
};