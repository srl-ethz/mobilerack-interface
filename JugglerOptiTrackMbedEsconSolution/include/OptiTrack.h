#ifndef OPTITRACK_H
#define OPTITRACK_H

#ifndef CTRL
#define CTRL(c) ((c) & 037)
#endif

#pragma warning(disable:4996)

#define M_PI 3.14159265358979323846f
#define PUCK_NAME "PuckHeavy"
//#define PUCKNAME "Puck"
#define FILTER_CUT_OFF_FREQUENCY 62.83
#define FILTER_SAMPLE_PERIOD 0.0083333 // = 1/120

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include <mutex>
#include <Eigen/Dense>
#include "LowPassFilter.h"
#include <winsock2.h>
using namespace Eigen;

class OptiTrack
{
public:
	OptiTrack();
	~OptiTrack();
	int initialize();
	void resetClient();
	int enterMenuMode();

	double getPaddlePosition();
	Vector2d getBallPosition();
	Vector2d getBallVelocity();

	void writeDataToFile();
	void stopWriteDataToFile();
	void dataCallback(sFrameOfMocapData* data);

protected:
	int CreateClient(int iConnectionType);

	void setPaddlePosition(const double&);
	void setBallPosition(const double&, const double&);
	void setBallVelocity(const double&, const double&);

private:
	bool m_initialized;
	unsigned int MyServersDataPort = 3130;
	//unsigned int MyServersDataPort = 3883;
	unsigned int MyServersCommandPort = 3131;
	int iConnectionType = ConnectionType_Multicast; //ConnectionType_Unicast;

	NatNetClient* m_theClient;

	char szMyIPAddress[128] = "";
	char szServerIPAddress[128] = "";

	int analogSamplesPerMocapFrame = 0;

	// start - Writing to File
	FILE* fp;
	wchar_t szFile[MAX_PATH];
	wchar_t szFolder[MAX_PATH];
	int writeCount = 0;
	int nSamplesToWait = (int) floor(2 * (1/ FILTER_SAMPLE_PERIOD) * (1 / (FILTER_CUT_OFF_FREQUENCY / 2 / M_PI)));
	void _WriteHeader();
	void _WriteFrame(sFrameOfMocapData* data);
	// end - Writing to File
	
	double fRate = 0.0;
	double expectedFramePeriod = 0.0;

	double xPos = 0.0;
	double zPos = 0.0;
	double xPosMax = 0.38;
	double xPosMin = -0.08;

	double zPosMax = 0.555;
	double zPosMin = -0.125;

	double xPosOld = 0.0;
	double zPosOld = 0.0;

	double xVel = 0.0;
	LowPassFilter xVelLowPassFilter;
	double zVel = 0.0;
	LowPassFilter zVelLowPassFilter;

	double q0 = 1;
	double q1 = 0;
	double q2 = 0;
	double q3 = 0;
	double psi = 0;

	// DONT TOUCH THOSE OTHER THAN BY GET SET FUNCTIONS
	Vector2d m_ballPosOptiTrack;
	Vector2d m_ballVelOptiTrack;
	double m_paddlePositionOptiTrack = 0.0;
	 
	std::mutex m_mutexPaddlePos, m_mutexBallPos, m_mutexBallVel;

};

//void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
//void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages

void __cdecl dataCallback(sFrameOfMocapData *, void *);
void __cdecl messageCallback(int, char*);

#endif //OPTITRACK_H
