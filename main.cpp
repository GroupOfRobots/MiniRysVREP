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
#include "rys.h"

extern "C" {
    #include "extApi.h"
}

int main()
{
	rys rozowy(19999);

	if (rozowy.clientID!=-1)
	{

		float minDistance=0.05;

		if (simxGetConnectionId(rozowy.clientID)!=-1)
		{  
			simxUChar sensorTrigger=0;
			
			rozowy.getData("RysLeftMotor", "RysRightMotor", "Rys", "Goal");
			rozowy.moveToPoint(minDistance);
			std::cout << "Done!\n";


			extApi_sleepMs(5);

		}
		else
		printf("Fin!\n");
	}
	return(0);
}

