#ifndef RYS_H_
#define RYS_H_
#include <iostream>
extern "C" {
    #include "extApi.h"
}
class rys{
	public:
		
				
		rys(int,const std::string&);
		~rys();
		
		/**
		 * Funkcja zwraca dane bieżącego robota.
		 * 
		 * \param lMH name of the left motor handle
		 * \param rMH name of the right motor handle
		 * \param cH name of the body of the robot handle
		 * \param gH name of the goal handle
		 */
		void getData(const char* lMH, const char* rMH, const char* cH, const char* gH);
		
		float goalPosition[3];
		
		/**
		 * Funkcja sprawiająca, że robot jedzie do punktu położenia piłeczki
		 * 
		 * \param minDistance minimalna odległość od piłeczki, na którą ma dojechać robot
		 */
		 
		void moveToPoint(float minDistance);
		void layDown();
		void standUp();
		
		bool valid() {
				return (clientID != -1);
		}
		bool valid2() {
				return (simxGetConnectionId(clientID)!=-1);
		}
		void moveAndRotate(float linVel, float angVel);
	private:
		int clientID;
		
		std::string linVelSignal;
		std::string angVelSignal;
		std::string positionSignal;
		std::string stopSignal;
		
		int leftMotorHandle;
		int rightMotorHandle;
		int cuboidHandle;
		int goalHandle;
		
		//low-level functions
		
		void stop();
		void setTarget();
		
};
#endif