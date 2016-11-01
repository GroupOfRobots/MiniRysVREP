#ifndef ROBOT_H_
#define ROBOT_H_

#include <cmath>

#ifdef DEBUG
#include <iostream>
#define SHOW(x) std::cout << x << std::endl;
#else
#define SHOW(x) 
#endif

float orientationError(float obj_x, float obj_y, float obj_or, float goal_x, float goal_y) {
	float GoalOrientation=atan2((goal_y-obj_y),(goal_x-obj_x));
	float res = 0;
	if (GoalOrientation-obj_or < -M_PI) {
		SHOW(1)
		res = 2*M_PI + (GoalOrientation - obj_or);
	} else if (GoalOrientation-obj_or > M_PI) {
		SHOW(2)
		res = -2*M_PI + (GoalOrientation - obj_or);
	} else {
		SHOW(3)
		res = GoalOrientation - obj_or;
	}
	
#ifdef DEBUG
	std::cout << "OR: " << obj_or << " GO: " << GoalOrientation << " ER: " << res << "\n";
#endif
	
	return res;
}

float orientationError2(float obj_x, float obj_y, float lPosition[3], float rPosition[3], float goal_x, float goal_y) {
	float GoalOrientation=atan2((goal_y-obj_y),(goal_x-obj_x));
	float WheelsOrientation=atan2((rPosition[1]-lPosition[1]),(rPosition[0]-lPosition[0]))+M_PI_2;
	float res = 0;
	if (GoalOrientation-WheelsOrientation < -M_PI) {
		SHOW(1)
		res = 2*M_PI + (GoalOrientation - WheelsOrientation);
	} else if (GoalOrientation-WheelsOrientation > M_PI) {
		SHOW(2)
		res = -2*M_PI + (GoalOrientation - WheelsOrientation);
	} else {
		SHOW(3)
		res = GoalOrientation - WheelsOrientation;
	}

#ifdef DEBUG
	std::cout << "OR: " << WheelsOrientation << " GO: " << GoalOrientation << " ER: " << res << "\n";
#endif

	return -res;
}
#endif
