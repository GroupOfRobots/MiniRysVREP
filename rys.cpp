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
	
	/*linVelSignal=lv.c_str();
	std::cout << linVelSignal << std::endl;
	angVelSignal=av.c_str();
	
	positionSignal=p.c_str();
	std::cout << "DUPA\n";
	std::cout << s << std::endl;
	stopSignal=s.c_str();
	std::cout << stopSignal <<std::endl;
	std::cout << "aaaa" <<std::endl;*/
	
}

rys::~rys()
{
	simxFinish(clientID);
}

void rys::getData(const char* lMH, const char* rMH, const char* cH, const char* gH)
{
	simxGetObjectHandle(clientID, lMH, &leftMotorHandle, simx_opmode_blocking);
    simxGetObjectHandle(clientID, rMH, &rightMotorHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, cH, &cuboidHandle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, gH, &goalHandle, simx_opmode_blocking);
}
void rys::moveAndRotate(float linVel, float angVel)
{
       std::cout << linVelSignal << std::endl;
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
