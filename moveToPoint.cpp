#include "rys.h"
#include "robot.h"
#include <math.h>
extern "C" {
    #include "extApi.h"
}

void rys::moveToPointFront(float minDistance)
{
	setTarget();
	float radius=0.043;
	float axis=0.112;
	float P=0.05;
	float LinVel=0.4; // [m/s]
	float AngVel;
	int state;

	float ObjectPosition[3];
	float ObjectOrientation[3]; //rotation about Z --> ObjectOrientation[2]
	float OrientationError;
	float lMPosition[3];
	float rMPosition[3];

	simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);
	
	float distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2)+pow(ObjectPosition[2]-goalPosition[2],2));

	while (distance>minDistance)
	{
		setTarget();
		simxGetObjectOrientation(clientID, cuboidHandle, -1, ObjectOrientation, simx_opmode_streaming);
		simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);


		simxGetObjectPosition(clientID,leftMotorHandle,-1, lMPosition, simx_opmode_oneshot_wait);
		simxGetObjectPosition(clientID,rightMotorHandle,-1, rMPosition, simx_opmode_oneshot_wait);
		OrientationError=orientationError2(ObjectPosition[0], ObjectPosition[1], lMPosition, rMPosition, goalPosition[0], goalPosition[1]);
		AngVel=P*OrientationError*180/M_PI; // [deg/s]
		moveAndRotate(LinVel, AngVel);
		distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2));
		if (isTheObstacleAhead(0.15))
			break;
		
	}
    stop();
}

void rys::moveToPoint(float minDistance)
{
	setTarget();
	float radius=0.043;
	float axis=0.112;
	float P=0.05;
	float LinVel=0.4; // [m/s]
	float AngVel;
	int state;

	float ObjectPosition[3];
	float ObjectOrientation[3]; //rotation about Z --> ObjectOrientation[2]
	float OrientationError;
	float lMPosition[3];
	float rMPosition[3];

	simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);
	
	float distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2)+pow(ObjectPosition[2]-goalPosition[2],2));

	while (distance>minDistance)
	{
		setTarget();
		simxGetObjectOrientation(clientID, cuboidHandle, -1, ObjectOrientation, simx_opmode_streaming);
		simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);


		simxGetObjectPosition(clientID,leftMotorHandle,-1, lMPosition, simx_opmode_oneshot_wait);
		simxGetObjectPosition(clientID,rightMotorHandle,-1, rMPosition, simx_opmode_oneshot_wait);
		OrientationError=orientationError2(ObjectPosition[0], ObjectPosition[1], lMPosition, rMPosition, goalPosition[0], goalPosition[1]);
		if (OrientationError>-M_PI/2 && OrientationError<M_PI/2)
		{
			AngVel=P*OrientationError*180/M_PI; // [deg/s]
			moveAndRotate(LinVel, AngVel);
		}
		else if (OrientationError>=M_PI/2)
		{
			AngVel=P*(M_PI/2-OrientationError)*180/M_PI;
			moveAndRotate(-LinVel,AngVel);
		}
		else
		{
			AngVel=P*(OrientationError+M_PI)*180/M_PI;
			moveAndRotate(-LinVel,AngVel);
		}
		distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2));
		if (isTheObstacleAhead(0.15))
		{
			std::cout << "przeszkoda" << ObjectOrientation[1]*180/M_PI << std::endl;
			break;
		}
		
	}
    stop();
}

void rys::moveToPointAndStop(float minDistance)
{
	setTarget();
	float radius=0.043;
	float axis=0.112;
	float P=0.05;
	float LinVel=0.3; // [m/s]
	float AngVel;
	int state=0;

	float ObjectPosition[3];
	float ObjectOrientation[3]; //rotation about Z --> ObjectOrientation[2]
	float OrientationError;
	float lMPosition[3];
	float rMPosition[3];

	simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);
	
	float distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2)+pow(ObjectPosition[2]-goalPosition[2],2));
	simxGetObjectOrientation(clientID, cuboidHandle, -1, ObjectOrientation, simx_opmode_streaming);

	while (distance>minDistance)
	{
		
		setTarget();
		simxGetObjectOrientation(clientID, cuboidHandle, -1, ObjectOrientation, simx_opmode_streaming);
		simxGetObjectPosition(clientID,cuboidHandle,-1,ObjectPosition,simx_opmode_oneshot_wait);


		simxGetObjectPosition(clientID,leftMotorHandle,-1, lMPosition, simx_opmode_oneshot_wait);
		simxGetObjectPosition(clientID,rightMotorHandle,-1, rMPosition, simx_opmode_oneshot_wait);
		OrientationError=orientationError2(ObjectPosition[0], ObjectPosition[1], lMPosition, rMPosition, goalPosition[0], goalPosition[1]);
		if (OrientationError>-M_PI/2 && OrientationError<M_PI/2)
		{
			AngVel=P*OrientationError*180/M_PI; // [deg/s]
			moveAndRotate(LinVel, AngVel);
			state=1;
		}
		else if (OrientationError>=M_PI/2)
		{
			AngVel=P*(M_PI/2-OrientationError)*180/M_PI;
			moveAndRotate(-LinVel,AngVel);
			state=2;
		}
		else
		{
			AngVel=P*(OrientationError+M_PI)*180/M_PI;
			moveAndRotate(-LinVel,AngVel);
			state=2;
		}
		distance=sqrt(pow(ObjectPosition[0]-goalPosition[0],2)+pow(ObjectPosition[1]-goalPosition[1],2));
		if (isTheObstacleAhead(0.1))
		{
			break;
			std::cout << "O hejka" << std::endl;
		}
		
	}
	moveAndRotate(pow((-1),state)*LinVel,0);
	extApi_sleepMs(5000);
    stop();
}
