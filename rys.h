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
		float sensorUpVal;
		float sensorFrontVal;
		float sensorBackVal;
		
		
		/**
		 * Funkcja zwraca dane bieżącego robota.
		 * 
		 * \param lMH name of the left motor handle
		 * \param rMH name of the right motor handle
		 * \param cH name of the body of the robot handle
		 * \param gH name of the goal handle
		 */
		void getData(const std::string & lMH, const std::string & rMH, const std::string & cH, const std::string & gH);
		
		float goalPosition[3];
		
		/**
		 * Funkcja sprawiająca, że robot jedzie do punktu położenia piłeczki
		 * 
		 * \param minDistance minimalna odległość od piłeczki, na którą ma dojechać robot
		 */
		 
		void moveToPoint(float minDistance);
		void layDown();
		void standUp();
		bool isTheObstacleAhead(float distance);

		
		bool valid() {
				return (clientID != -1);
		}
		bool connected() {
				return (simxGetConnectionId(clientID)!=-1);
		}
	private:
		int clientID;
		
		std::string linVelSignal;
		std::string angVelSignal;
		std::string positionSignal;
		std::string stopSignal;
		std::string sensorUpSignal;
		std::string sensorFrontSignal;
		std::string sensorBackSignal;
		
		int leftMotorHandle;
		int rightMotorHandle;
		int cuboidHandle;
		int goalHandle;
		
		
		//low-level functions
		void moveAndRotate(float linVel, float angVel);
		void stop();
		void setTarget();
		void readSensors();
};
#endif
