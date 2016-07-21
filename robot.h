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

#endif
