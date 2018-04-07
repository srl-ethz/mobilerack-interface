#include "OptiTrack.h"

int main()
{	
	OptiTrack optiTrack;
	
	int errorCode = optiTrack.initialize();
	if (errorCode != ErrorCode_OK)
	{
		printf("Something went wrong initializing OptiTrack.");
		return 1;
	}

	errorCode = optiTrack.enterMenuMode();
		if (errorCode != ErrorCode_OK)
		{
			printf("Something went wrong in menu mode of OptiTrack.");
			return 1;
		}
	return ErrorCode_OK;

}