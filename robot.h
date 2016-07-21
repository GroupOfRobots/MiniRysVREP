#ifndef ROBOT_H_
#define ROBOT_H_

#include <cmath>

float orientationError(float obj_x, float obj_y, float obj_or, float goal_x, float goal_y) {
	float GoalOrientation=atan2((goal_y-obj_y),(goal_x-obj_x));
	if (GoalOrientation-obj_or < -M_PI)
		return 2*M_PI + (GoalOrientation - obj_or);
	else if (GoalOrientation-obj_or > M_PI)
		return 2*M_PI - (GoalOrientation - obj_or);
	else
		return GoalOrientation - obj_or;
	

}

#endif
