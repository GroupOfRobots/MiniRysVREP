#include "rys.h"
#include <iostream>
extern "C" {
    #include "extApi.h"
}

rys::rys(int portNb, const std::string& nazwa) 
{
	
	clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	
	linVelSignal=nazwa+"_linVel";
	angVelSignal=nazwa+"_angVel";
	positionSignal=nazwa+"_pozycja";
	stopSignal=nazwa+"_stop";
	sensorBackSignal=nazwa+"_sensorBack";
	sensorFrontSignal=nazwa+"_sensorFront";
	sensorUpSignal=nazwa+"_sensorUp";
	
}

rys::~rys()
{
	simxFinish(clientID);
}

void rys::getData(const std::string & lMH, const std::string & rMH, const std::string & cH, const std::string & gH)
{
	simxGetObjectHandle(clientID, lMH.c_str(), &leftMotorHandle, simx_opmode_blocking);
    simxGetObjectHandle(clientID, rMH.c_str(), &rightMotorHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, cH.c_str(), &cuboidHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, gH.c_str(), &goalHandle, simx_opmode_blocking);
}
void rys::moveAndRotate(float linVel, float angVel)
{
       simxSetFloatSignal(clientID,angVelSignal.c_str(),angVel,simx_opmode_oneshot);
       simxSetFloatSignal(clientID,linVelSignal.c_str(),linVel,simx_opmode_oneshot);
 }

void rys::stop()
{
	simxSetIntegerSignal(clientID,stopSignal.c_str(),1,simx_opmode_blocking);
}

void rys::layDown()
{
	simxSetIntegerSignal(clientID,positionSignal.c_str(),2,simx_opmode_blocking);
	int state;
	simxGetIntegerSignal(clientID,positionSignal.c_str(),&state,simx_opmode_streaming);
	while(state!=1)
	{
		simxGetIntegerSignal(clientID,positionSignal.c_str(),&state,simx_opmode_buffer);	
	}
}

void rys::standUp()
{

	simxSetIntegerSignal(clientID,positionSignal.c_str(),3,simx_opmode_blocking);
	int state;
	simxGetIntegerSignal(clientID,positionSignal.c_str(),&state,simx_opmode_streaming);
	while(state)
	{
		simxGetIntegerSignal(clientID,positionSignal.c_str(),&state,simx_opmode_buffer);	
	}
}

void rys::setTarget()
{
	simxGetObjectPosition(clientID,goalHandle,-1,goalPosition,simx_opmode_oneshot_wait);
}

void rys::readSensors()
{
	simxGetFloatSignal(clientID,sensorFrontSignal.c_str(), &sensorFrontVal,simx_opmode_streaming);
	simxGetFloatSignal(clientID,sensorBackSignal.c_str(), &sensorBackVal,simx_opmode_streaming);
	simxGetFloatSignal(clientID,sensorUpSignal.c_str(), &sensorUpVal,simx_opmode_streaming);
}
