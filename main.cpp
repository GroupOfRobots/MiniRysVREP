// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.1 Rev1 on May 17th 2016

// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono>
#include <iostream>
#define DEBUG
#include "robot.h"

extern "C" {
    #include "extApi.h"
}

int MoveandRotate(float linVel, float angVel, int clientID)
{
       simxSetFloatSignal(clientID,"angVel",angVel,simx_opmode_streaming);
       simxSetFloatSignal(clientID,"linVel",linVel,simx_opmode_streaming);
}

int MovetoPoint(float *GoalPosition, float minDistance, int clientID, int leftMotorHandle, int rightMotorHandle, int cuboidHandle)
{
	float radius=0.05;
	float axis=0.075;
	float P=0.06;
	float LinVel=0.4; // [m/s]
	float AngVel;
	int state;

	float ObjectPosition[3];
	float ObjectOrientation[3]; //rotation about Z --> ObjectOrientation[2]
	float OrientationError;
	float lMPosition[3];
	float rMPosition[3];

	simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);
	
	float distance=sqrt(pow(ObjectPosition[0]-GoalPosition[0],2)+pow(ObjectPosition[1]-GoalPosition[1],2)+pow(ObjectPosition[2]-GoalPosition[2],2));
	auto snow = std::chrono::system_clock::now();
	auto sduration = snow.time_since_epoch();
	auto smillis = std::chrono::duration_cast<std::chrono::milliseconds>(sduration).count();
	
	while (distance>minDistance)
	{
		simxGetObjectOrientation(clientID, cuboidHandle, -1, ObjectOrientation, simx_opmode_streaming);
		simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);

		//simxGetIntegerSignal(clientID,"pozycja",&state,simx_opmode_streaming);
		//if (state==0)
		//	OrientationError=orientationError(ObjectPosition[0], ObjectPosition[1], ObjectOrientation[2], GoalPosition[0], GoalPosition[1]);
		//else
		simxGetObjectPosition(clientID,leftMotorHandle,-1, lMPosition, simx_opmode_oneshot_wait);
		simxGetObjectPosition(clientID,rightMotorHandle,-1, rMPosition, simx_opmode_oneshot_wait);
		OrientationError=orientationError2(ObjectPosition[0], ObjectPosition[1], lMPosition, rMPosition, GoalPosition[0], GoalPosition[1]);
		AngVel=P*OrientationError*180/M_PI; // [deg/s]
		MoveandRotate(LinVel, AngVel, clientID);
		printf("Distance: %f Robot2: %f Robot0: %f Error: %f\n", distance, ObjectOrientation[2], ObjectOrientation[1], OrientationError); 

		distance=sqrt(pow(ObjectPosition[0]-GoalPosition[0],2)+pow(ObjectPosition[1]-GoalPosition[1],2));
		
		//printing data to file
		
		auto now = std::chrono::system_clock::now();
		auto duration = now.time_since_epoch();
		auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
		
		//std::cout << millis-smillis << "\t" << ObjectPosition[0] << "\t" << ObjectPosition[1] << std::endl;
		
		
	}
    simxSetFloatSignal(clientID,"angVel",0,simx_opmode_blocking);
    simxSetFloatSignal(clientID,"linVel",0,simx_opmode_blocking);
}

int LayDown(int clientID)
{
		
	simxSetFloatSignal(clientID,"linVel",-5,simx_opmode_streaming);
	simxSetFloatSignal(clientID,"angVel",0,simx_opmode_streaming);
	simxSetIntegerSignal(clientID, "pozycja", 1, simx_opmode_oneshot);	

}

int StandUp(int clientID)
{
	int stoi;
	simxGetIntegerSignal(clientID,"stoi",&stoi,simx_opmode_streaming);
	while (!stoi)
	{
		simxSetFloatSignal(clientID,"angVel",0,simx_opmode_streaming);
		simxSetFloatSignal(clientID, "linVel",1000, simx_opmode_streaming);
		simxGetIntegerSignal(clientID,"stoi",&stoi,simx_opmode_streaming);
		std::cout << "la";
	}
	simxSetIntegerSignal(clientID, "pozycja", 0, simx_opmode_streaming);	
}



int main(int argc,char* argv[])
{
	int portNb=0;
	int leftMotorHandle;
	int rightMotorHandle;
	int cuboidHandle;
	int goalHandle;
	int pureMagicMotorHandle;

	if (argc>=6)
	{
		portNb=atoi(argv[1]);
		leftMotorHandle=atoi(argv[2]);
		rightMotorHandle=atoi(argv[3]);
		cuboidHandle=atoi(argv[4]);
		goalHandle=atoi(argv[5]);
	}
	else
	{
		extApi_sleepMs(5000);
		return 0;
	}

	int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if (clientID!=-1)
	{
        simxSetFloatSignal(clientID,"linVel",0.,simx_opmode_oneshot);
        simxSetFloatSignal(clientID,"andVel",0.,simx_opmode_oneshot);
		int driveBackStartTime=-99000;
		float motorSpeeds[2];
		float leftMotorAngle;
		float ObjectPosition[3];
		float GoalPosition[3];
		float ObjectOrientation[3];
		float minDistance=0.05;

		if (simxGetConnectionId(clientID)!=-1)
		{  
			simxUChar sensorTrigger=0;

			simxGetObjectHandle(clientID, argv[2], &leftMotorHandle, simx_opmode_blocking);
            simxGetObjectHandle(clientID, argv[3], &rightMotorHandle, simx_opmode_blocking);
			simxGetObjectHandle(clientID, argv[4], &cuboidHandle, simx_opmode_blocking);
			simxGetObjectHandle(clientID, argv[5], &goalHandle, simx_opmode_blocking);


			simxGetObjectPosition(clientID,goalHandle,-1,GoalPosition,simx_opmode_oneshot_wait);
			LayDown(clientID);
			MovetoPoint(GoalPosition, minDistance, clientID, leftMotorHandle, rightMotorHandle, cuboidHandle);
			//StandUp(clientID);
			

			extApi_sleepMs(5);

		}
		printf("Fin!\n");
		simxFinish(clientID);
	}
	return(0);
}

