#include "OptiVision.h"

OptiVision::OptiVision(int argc, char *argv[])
{
	int iResult;

	// parse command line args
	if (argc>1)
	{
		strcpy(szServerIPAddress, argv[1]);	// specified on command line
		printf("Connecting to server at %s...\n", szServerIPAddress);
	}
	else
	{
		strcpy(szServerIPAddress, "");		// not specified - assume server is local machine
		printf("Connecting to server at LocalMachine\n");
	}
	if (argc>2)
	{
		strcpy(szMyIPAddress, argv[2]);	    // specified on command line
		printf("Connecting from %s...\n", szMyIPAddress);
	}
	else
	{
		strcpy(szMyIPAddress, "");          // not specified - assume server is local machine
		printf("Connecting from LocalMachine...\n");
	}

	// Create NatNet Client
	iResult = CreateClient(iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
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
			else
			{
				printf("Unknown data type.");
				// Unknown
			}
		}
	}

	// Create data file for writing received stream into
	char szFile[MAX_PATH];
	wchar_t szFolder[MAX_PATH] = {};
	GetCurrentDirectory(MAX_PATH, szFolder);

	if (argc > 3)
		sprintf(szFile, "%s\\%s", szFolder, argv[3]);
	else
		sprintf(szFile, "%s:\Client-output.pts", szFolder);
	fp = fopen(szFile, "w");
	if (!fp)
	{
		printf("error opening output file %s.  Exiting.", szFile);
		exit(1);
	}
	if (pDataDefs)
		_WriteHeader(fp, pDataDefs);

}

OptiVision::~OptiVision()
{
	theClient->Uninitialize();
	_WriteFooter(fp);
	fclose(fp);
}

int OptiVision::CreateClient(int iConnectionType)
{
	// release previous server
	if (theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create NatNet client
	theClient = new NatNetClient(iConnectionType);



	// set the callback handlers
	theClient->SetVerbosityLevel(Verbosity_Warning);
	theClient->SetMessageCallback(MessageHandler);
	
	/* This gives an error, so it is commented out for the time being */
	/*
	theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
														// [optional] use old multicast group
														//theClient->SetMulticastAddress("224.0.0.1");

														// print version info
	*/

	unsigned char ver[4];
	theClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Init Client and connect to NatNet server
	// to use NatNet default port assignments
	int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
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
		ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			analogSamplesPerMocapFrame = *((int*)pResult);
			printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
		}

		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);
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

/* File writing routines */
void OptiVision::_WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs)
{
	int i = 0;

	if (!pBodyDefs->arrDataDescriptions[0].type == Descriptor_MarkerSet)
		return;

	sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

	fprintf(fp, "<MarkerSet>\n\n");
	fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

	fprintf(fp, "<Markers>\n");
	for (i = 0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
	}
	fprintf(fp, "</Markers>\n\n");

	fprintf(fp, "<Data>\n");
	fprintf(fp, "Frame#\t");
	for (i = 0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
	}
	fprintf(fp, "\n");
}

void OptiVision::_WriteFooter(FILE* fp)
{
	fprintf(fp, "</Data>\n\n");
	fprintf(fp, "</MarkerSet>\n");
}

void OptiVision::_WriteFrame(FILE* fp, sFrameOfMocapData* data)
{
	fprintf(fp, "%d", data->iFrame);

	/*for(int i =0; i < data->MocapData->nMarkers; i++)
	{
	fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
	}*/

	for (int i = 0; i < data->nLabeledMarkers; i++)
	{
		sMarker marker = data->LabeledMarkers[i];
		fprintf(fp, "\t%.5f\t%.5f\t%.5f", marker.x, marker.y, marker.z);
	}

	fprintf(fp, "\n");
}

void OptiVision::resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if (iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if (iSuccess != 0)
		printf("error re-initting Client\n");


}

void OptiVision::acquireFrame(int iResult, void* response, int nBytes)
{
		// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;
	while (c = _getch())
	{
		switch (c)
		{
		case 'q':
			bExit = true;
			break;
		case 'r':
			resetClient();
			break;
		case 'p':
			sServerDescription ServerDescription;
			memset(&ServerDescription, 0, sizeof(ServerDescription));
			theClient->GetServerDescription(&ServerDescription);
			if (!ServerDescription.HostPresent)
			{
				printf("Unable to connect to server. Host not present. Exiting.");
			}
			break;
		case 'f':
		{
			sFrameOfMocapData* pData = theClient->GetLastFrameOfData();
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
			iResult = theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
			if (iResult == ErrorCode_OK)
				printf("[SampleClient] Disconnected");
			break;
		default:
			break;
		}
		if (bExit)
			break;
	}
}


// DataHandler receives data from the server
void __cdecl OptiVision::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*)pUserData;

	if (fp)
		_WriteFrame(fp, data);

	int i = 0;

	// printf("FrameID : %d\n", data->iFrame);
	// printf("Timestamp :  %3.2lf\n", data->fTimestamp);
	// printf("Latency :  %3.2lf\n", data->fLatency);

	// FrameOfMocapData params
	bool bIsRecording = ((data->params & 0x01) != 0);
	bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
	if (bIsRecording)
		printf("RECORDING\n");
	if (bTrackedModelsChanged)
		printf("Models Changed.\n");

	/* // Other Markers
	printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	for(i=0; i < data->nOtherMarkers; i++)
	{
	printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n",
	i,
	data->OtherMarkers[i][0],
	data->OtherMarkers[i][1],
	data->OtherMarkers[i][2]);
	}*/


	// labeled markers
	bool bOccluded;     // marker was not visible (occluded) in this frame
	bool bPCSolved;     // reported position provided by point cloud solve
	bool bModelSolved;  // reported position provided by model solve
	printf("Labeled Markers [Count=%d]\n", data->nLabeledMarkers);
	for (i = 0; i < data->nLabeledMarkers; i++)
	{
		bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
		bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
		bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
		sMarker marker = data->LabeledMarkers[i];
		int modelID, markerID;
		theClient->DecodeID(marker.ID, &modelID, &markerID);
		printf("Labeled Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
			modelID, markerID, bOccluded, bPCSolved, bModelSolved, marker.size, marker.x, marker.y, marker.z);
	}
}

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}