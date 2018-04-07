#include "OptiTrack.h"

#define DTTMFMT L"%Y-%m-%d_%H-%M-%S"
#define DTTMSZ 21

#include <time.h>
#include <iostream>

//#include <string>
#include <stdio.h>
#include <conio.h>

/* Constructor */
OptiTrack::OptiTrack():
	m_initialized(false),
	xVelLowPassFilter(FILTER_CUT_OFF_FREQUENCY ,FILTER_SAMPLE_PERIOD),
	zVelLowPassFilter(FILTER_CUT_OFF_FREQUENCY , FILTER_SAMPLE_PERIOD)
{
}

int OptiTrack::initialize()
{
	if (m_initialized)
	{
		printf("Already initialized.  Exiting");
		return 1;
	}

	int iResult;

	// Create NatNet Client
	iResult = CreateClient(iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = m_theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = m_theClient->GetDataDescriptions(&pDataDefs);
	if (!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
		printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
		for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
			if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				// MarkerSet
				sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				printf("MarkerSet Name : %s\n", pMS->szName);
				for (int i = 0; i < pMS->nMarkers; i++)
					printf("%s\n", pMS->szMarkerNames[i]);

			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				// RigidBody
				sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				printf("RigidBody Name : %s\n", pRB->szName);
				printf("RigidBody ID : %d\n", pRB->ID);
				printf("RigidBody Parent ID : %d\n", pRB->parentID);
				printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else
			{
				printf("Unknown data type.");
				// Unknown
			}
		}
	}

	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");

	m_initialized = true;
	
	return ErrorCode_OK;
}
/* Destructor */
OptiTrack::~OptiTrack()
{
	this->stopWriteDataToFile();
	m_theClient->Uninitialize();
	delete m_theClient;
}


// Establish a NatNet Client connection
int OptiTrack::CreateClient(int iConnectionType)
{
	// release previous server
	if (m_theClient)
	{
		m_theClient->Uninitialize();
		delete m_theClient;
	}

	// create NatNet client
	m_theClient = new NatNetClient(iConnectionType);

	// set the callback handlers
	m_theClient->SetVerbosityLevel(Verbosity_Warning);
	m_theClient->SetMessageCallback(::messageCallback);
	m_theClient->SetDataCallback(::dataCallback, this);	// this function will receive data from the server
														// [optional] use old multicast group
														//theClient->SetMulticastAddress("224.0.0.1");

														// print version info
	unsigned char ver[4];
	m_theClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Init Client and connect to NatNet server
	// to use NatNet default port assignments
	int retCode = m_theClient->Initialize(szMyIPAddress, szServerIPAddress);
	// to use a different port for commands and/or data:
	//int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// get # of analog samples per mocap frame of data
		void* pResult;
		int ret = 0;
		int nBytes = 0;
		ret = m_theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			analogSamplesPerMocapFrame = *((int*)pResult);
			printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
		}

		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		m_theClient->GetServerDescription(&ServerDescription);
		if (!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		printf("[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", szMyIPAddress);
		printf("Server IP:%s\n", szServerIPAddress);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;

}

int OptiTrack::enterMenuMode()
{
	void* response;
	int nBytes;
	int c;
	int iResult = ErrorCode_OK;
	bool bExit = false;
	while (c = _getch())
	{
		switch (c)
		{
		case 'q':
			bExit = true;
			break;
		case 'r':
			this->resetClient();
			break;
		case 'p':
			sServerDescription ServerDescription;
			memset(&ServerDescription, 0, sizeof(ServerDescription));
			m_theClient->GetServerDescription(&ServerDescription);
			if (!ServerDescription.HostPresent)
			{
				printf("Unable to connect to server. Host not present. Exiting.");
				return 1;
			}
			break;
		case 'f':
		{
			sFrameOfMocapData* pData = m_theClient->GetLastFrameOfData();
			printf("Most Recent Frame: %d", pData->iFrame);
		}
		break;
		case 'm':	                        // change to multicast
			iConnectionType = ConnectionType_Multicast;
			iResult = CreateClient(iConnectionType);
			if (iResult == ErrorCode_OK)
				printf("Client connection type changed to Multicast.\n\n");
			else
				printf("Error changing client connection type to Multicast.\n\n");
			break;
		case 'u':	                        // change to unicast
			iConnectionType = ConnectionType_Unicast;
			iResult = CreateClient(iConnectionType);
			if (iResult == ErrorCode_OK)
				printf("Client connection type changed to Unicast.\n\n");
			else
				printf("Error changing client connection type to Unicast.\n\n");
			break;
		case 'c':                          // connect
			iResult = CreateClient(iConnectionType);
			break;
		case 'd':                          // disconnect
										   // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
			iResult = m_theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
			if (iResult == ErrorCode_OK)
				printf("[SampleClient] Disconnected");
			break;
		case CTRL('r'):
			writeDataToFile();
			break;
		case CTRL('s'):
			stopWriteDataToFile();
			break;
		default:
			break;
		}
		if (bExit)
			break;
		// ADDED A SLEEP HERE
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return iResult;
}

void __cdecl dataCallback(sFrameOfMocapData* data, void* pUserData)
{
	OptiTrack* optiTrackPointer = (OptiTrack*)pUserData;
	optiTrackPointer->dataCallback(data);
}


// DataHandler receives data from the server
void OptiTrack::dataCallback(sFrameOfMocapData* data)
{
	//printf("CALLBACK!\n");
	int i = 0;

	// printf("FrameID : %d\n", data->iFrame);
	//printf("Timestamp :  %3.2lf\n", data->fTimestamp);
	//printf("Latency :  %3.2lf\n", data->fLatency);

	// FrameOfMocapData params
	bool bIsRecording = ((data->params & 0x01) != 0);
	bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
	if (bIsRecording)
		printf("RECORDING\n");
	if (bTrackedModelsChanged)
		printf("Models Changed.\n");

	// get frame rate from host
	void* pResult;
	int ret = 0;
	int nBytes = 0;
	ret = m_theClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
	if (ret == ErrorCode_OK)
	{
		fRate = *((float*)pResult);
		if (fRate != 0.0f)
			expectedFramePeriod = (1 / fRate);
	}
	if (expectedFramePeriod == 0.0)
		printf("Error establishing Frame Rate.");

	// xPos /= data->nLabeledMarkers;
	// zPos /= data->nLabeledMarkers;

	sDataDescriptions* pDataDefs = NULL;
	int nBodies = m_theClient->GetDataDescriptions(&pDataDefs);

	for (i = 0; i < pDataDefs->nDataDescriptions; i++)
	{
		if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
		{
			sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
			if (!strcmp(PUCK_NAME, pRB->szName)) // Puck
			{
				xPos = data->RigidBodies[i].x;
				zPos = data->RigidBodies[i].z;

				if (xPos > xPosMax)
					xPos = xPosMax;
				else if (xPos < xPosMin)
					xPos = xPosMin;

				if (zPos > zPosMax)
					zPos = zPosMax;
				else if (zPos < zPosMin)
					zPos = zPosMin;

				xVel = xVelLowPassFilter.calculate(xPos);
				zVel = zVelLowPassFilter.calculate(zPos);

				//xVel = (xPos - xPosOld) * fRate;
				//zVel = (zPos - zPosOld) * fRate;

				xPosOld = xPos;
				zPosOld = zPos;
			}
			else if (!strcmp("Paddle", pRB->szName)) // Paddle
			{
				q0 = data->RigidBodies[i].qw;
				q1 = data->RigidBodies[i].qx;
				q2 = data->RigidBodies[i].qy;
				q3 = data->RigidBodies[i].qz;
				psi = atan2(-2 * (q1*q3 + q0*q2), pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2));
			}
		}
	}

	// log data (if active)
	this->_WriteFrame(data);

	/*printf("\t%3.3f\t%3.3f\t%3.2f\t%3.2f\t%3.3f\n",
		xPos, zPos, xVel, zVel, psi);*/

	this->setPaddlePosition(psi);
	this->setBallPosition(xPos, zPos);
	this->setBallVelocity(xVel, zVel);
}

// MessageHandler receives NatNet error/debug messages
void __cdecl messageCallback(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

void OptiTrack::resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = m_theClient->Uninitialize();
	if (iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = m_theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if (iSuccess != 0)
		printf("error re-initting Client\n");


}

// SHARE DATA VARIABLES
double OptiTrack::getPaddlePosition()
{
	std::lock_guard<std::mutex> guard(m_mutexPaddlePos);
	return m_paddlePositionOptiTrack;
}

void OptiTrack::setPaddlePosition(const double& paddlePos)
{
	std::lock_guard<std::mutex> guard(m_mutexPaddlePos);
	m_paddlePositionOptiTrack = paddlePos;
}

Vector2d OptiTrack::getBallPosition()
{
	std::lock_guard<std::mutex> guard(m_mutexBallPos);
	return m_ballPosOptiTrack;
}

void OptiTrack::setBallPosition(const double& x, const double& z)
{
	std::lock_guard<std::mutex> guard(m_mutexBallPos);
	m_ballPosOptiTrack(0) = x;
	m_ballPosOptiTrack(1) = z;
}

Vector2d OptiTrack::getBallVelocity()
{
	std::lock_guard<std::mutex> guard(m_mutexBallVel);
	return m_ballVelOptiTrack;
}

void OptiTrack::setBallVelocity(const double& xp, const double& zp)
{
	std::lock_guard<std::mutex> guard(m_mutexBallVel);
	m_ballVelOptiTrack(0) = xp;
	m_ballVelOptiTrack(1) = zp;
}

/*


static char * OptiTrack::getDtTm(char *buff) {
	time_t t = time(0);
	
	return buff;
}
*/



void OptiTrack::writeDataToFile()
{
	if (!fp)
	{
		wchar_t buff[DTTMSZ];
		time_t ltime; /* calendar time */
		ltime = time(NULL); /* get current cal time */
		wcsftime(buff, DTTMSZ, DTTMFMT, localtime(&ltime));

		// start writing
		std::size_t size = MAX_PATH;
		GetCurrentDirectory(MAX_PATH, szFolder);
		swprintf(szFile, size, L"%s\\Juggler-Experiment_%s.pts", szFolder,buff); 
		wprintf(L"Writing to: %s\n", szFile);
		fp = _wfopen(szFile, L"w");
		if (!fp)
		{
			wprintf(L"error opening output file %s.  Exiting.", szFile);
			exit(1);
		}
		else
		{
			printf("Starting to Write to File...\n");
			_WriteHeader();
		}
	}
	else {
		printf("Already Writing to File!\n");
	}
}

void OptiTrack::stopWriteDataToFile()
{
	if (fp)
	{
		printf("Closing File...\n");
		fclose(fp);
		fp = nullptr;
		printf("File closed...\n");
	}
	else {
		printf("Not writing yet!!!\n");
	}
}


void OptiTrack::_WriteFrame(sFrameOfMocapData* data)
{
	if (fp) // Write data only if the file stream is open
	{
		if (writeCount > nSamplesToWait)
		{
			fprintf(fp, "%d", data->iFrame);

			sDataDescriptions* pDataDefs = NULL;
			int nBodies = m_theClient->GetDataDescriptions(&pDataDefs);

			fprintf(fp, "\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f", xPos, zPos, xVel, zVel, psi);
			fprintf(fp, "\n");
		}
		writeCount++;
	}
}

void OptiTrack::_WriteHeader()
{
	fprintf(fp, "%s\t%s\t%s\t%s\t%s\t%s", "Frame     ", "x     ", "z    ", "xd      ", "zd     ", "psi");
	fprintf(fp, "\n");

}