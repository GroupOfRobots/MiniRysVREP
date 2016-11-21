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
#include <thread>

extern "C" {
    #include "extApi.h"
}

void rysiu_zyj(int port, const std::string & col, const std::string & suffix) {
	rys robot(port, col);

	if (robot.valid()) {
		robot.getData("RysLeftMotor" + suffix, "RysRightMotor" + suffix, "Rys" + suffix, "Goal" + suffix);

		if(robot.valid2()) {
			robot.moveToPoint(0.05);	
			extApi_sleepMs(5);
		}
	}
}






int main()
{	
	std::thread t1(rysiu_zyj, 19999, "rozowy", "");
	std::thread t2(rysiu_zyj, 20000, "niebieski", "2");
	std::thread t3(rysiu_zyj, 20001, "zielony", "3");
	std::thread t4(rysiu_zyj, 20001, "fioletowy", "4");
	t1.join();
	t2.join();
			
	return(0);
}

