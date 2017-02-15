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
	rys rozowy(19999, "rozowy");
	//rys niebieski(20000, "niebieski");
	//rys zielony(20001, "zielony");
	//rys fioletowy(20002, "fioletowy");
	//rys zolty(20003, "zolty");
	//rys pomaranczowy(20004, "pomaranczowy");

	if (rozowy.valid())
	{
		float minDistance=0.05;
		rozowy.getData("RysLeftMotor", "RysRightMotor", "Rys", "Goal");
		/*niebieski.getData("RysLeftMotor2", "RysRightMotor2", "Rys2", "Goal2");
		zielony.getData("RysLeftMotor3", "RysRightMotor3", "Rys3", "Goal3");
		fioletowy.getData("RysLeftMotor4", "RysRightMotor4", "Rys4", "Goal4");
		zolty.getData("RysLeftMotor5", "RysRightMotor5", "Rys5", "Goal5");
		pomaranczowy.getData("RysLeftMotor6", "RysRightMotor6", "Rys6", "Goal6");*/
			

		if (rozowy.connected())
		{  
		


			//rozowy.layDown();
			rozowy.moveToPointAndStop(0.1);
			//rozowy.layDown();
			//rozowy.standUp();
			//rozowy.layDown();
			//rozowy.moveAndRotate(-0.4,0);


			extApi_sleepMs(5);

		}
		
	}
	return(0);
}

