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

extern "C" {
    #include "extApi.h"
}

int MoveandRotate(float LinVel, float AngVel, float radius, float lengthWheelAxis, int clientID, int leftMotorHandle, int rightMotorHandle)
{
       float leftMotorAngVel=(LinVel-AngVel*(lengthWheelAxis/2))/radius;
       float rightMotorAngVel=(LinVel+AngVel*(lengthWheelAxis/2))/radius;
       simxSetJointTargetVelocity(clientID,leftMotorHandle,leftMotorAngVel,simx_opmode_oneshot);			
       simxSetJointTargetVelocity(clientID,rightMotorHandle,rightMotorAngVel,simx_opmode_oneshot);
}



int main(int argc,char* argv[])
{
	int portNb=0;
	int leftMotorHandle;
	int rightMotorHandle;
	int sensorHandle;

	if (argc>=5)
	{
		portNb=atoi(argv[1]);
		leftMotorHandle=atoi(argv[2]);
		rightMotorHandle=atoi(argv[3]);
		sensorHandle=atoi(argv[4]);
	}
	else
	{
		extApi_sleepMs(5000);
		return 0;
	}

	int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if (clientID!=-1)
	{
		int driveBackStartTime=-99000;
		float motorSpeeds[2];
		float leftMotorAngle;
		float ObjectPosition[3];
		

		while (simxGetConnectionId(clientID)!=-1)
		{  
			simxUChar sensorTrigger=0;
			motorSpeeds[0]=-3.1415f*0.5f;
			motorSpeeds[1]=-3.1415f*0.25f;
			simxGetObjectHandle(clientID, argv[2], &leftMotorHandle, simx_opmode_blocking);
                        simxGetObjectHandle(clientID, argv[3], &rightMotorHandle, simx_opmode_blocking);
			//printf("LMH: %d\n", leftMotorHandle);
			simxGetJointPosition(clientID,leftMotorHandle,&leftMotorAngle,simx_opmode_streaming);
			//printf("%f\n", leftMotorAngle);
			simxGetObjectPosition(clientID,leftMotorHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);
			printf("%f %f %f\n", ObjectPosition[0], ObjectPosition[1], ObjectPosition[2]);
			//simxSetJointTargetVelocity(clientID,leftMotorHandle,motorSpeeds[0],simx_opmode_oneshot);			
			//simxSetJointTargetVelocity(clientID,rightMotorHandle,motorSpeeds[1],simx_opmode_oneshot);	
                        MoveandRotate(0.1,1.,0.25,0.5,clientID,leftMotorHandle,rightMotorHandle);	
			extApi_sleepMs(5);
		}
		printf("Fin!\n");
		simxFinish(clientID);
	}
	return(0);
}

