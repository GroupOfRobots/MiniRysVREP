#include "rys.h"
extern "C" {
    #include "extApi.h"
}

rys::rys(int portNb)
{
	clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
}

rys::~rys()
{
	simxFinish(clientID);
}

void rys::getData(char* lMH, char* rMH, char* cH, char* gH)
{
	simxGetObjectHandle(clientID, lMH, &leftMotorHandle, simx_opmode_blocking);
    simxGetObjectHandle(clientID, rMH, &rightMotorHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, cH, &cuboidHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, gH, &goalHandle, simx_opmode_blocking);
}
void rys::moveAndRotate(float linVel, float angVel)
{
       simxSetFloatSignal(clientID,"angVel",angVel,simx_opmode_oneshot);
       simxSetFloatSignal(clientID,"linVel",linVel,simx_opmode_oneshot);
 }

void rys::stop()
{
	simxSetIntegerSignal(clientID,"stop",1,simx_opmode_blocking);
}

void rys::layDown()
{
	simxSetIntegerSignal(clientID,"pozycja",2,simx_opmode_blocking);
	int state;
	simxGetIntegerSignal(clientID,"pozycja",&state,simx_opmode_streaming);
	while(state!=1)
	{
		simxGetIntegerSignal(clientID,"pozycja",&state,simx_opmode_buffer);	
	}
}

void rys::standUp()
{

	simxSetIntegerSignal(clientID,"pozycja",3,simx_opmode_blocking);
	int state;
	simxGetIntegerSignal(clientID,"pozycja",&state,simx_opmode_streaming);
	while(state)
	{
		simxGetIntegerSignal(clientID,"pozycja",&state,simx_opmode_buffer);	
	}
}

void rys::setTarget()
{
	simxGetObjectPosition(clientID,goalHandle,-1,goalPosition,simx_opmode_oneshot_wait);
}
