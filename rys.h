#ifndef RYS_H_
#define RYS_H_
#include <iostream>

class rys{
	public:
		//konstruktor
		int clientID;
		
		rys(int);
		~rys();
		
		void getData(char*, char*, char*, char*);
		float goalPosition[3];
		
		//high-level functions
		void moveToPoint(float);
		void layDown();
		void standUp();
	private:
		//getData()
		int leftMotorHandle;
		int rightMotorHandle;
		int cuboidHandle;
		int goalHandle;
		
		//low-level functions
		void moveAndRotate(float linVel, float angVel);
		void stop();
		void setTarget();
		
};
#endif
