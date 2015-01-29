#include <math.h>
#include "movements.h"

void driveMotors(int speeLeft, int speedRight);


typedef struct{
	double SpeedL;
	double SpeedR;
} MovementsInternalValues;

MovementsInternalValues movements_last_values;

void rotateRel_naive(double deltaAngle)
{
	double x, y, t;
	double targetAngle;
	double error;
	int cmdVel;

	getRobotPos(&x, &y, &t);
	targetAngle = normalizeAngle(t + deltaAngle);
	error = normalizeAngle(targetAngle - t);
	if(error < 0)
		cmdVel = -SPEED_ROTATE_SLOW;
	else
		cmdVel = SPEED_ROTATE_SLOW;
		
	driveMotors(-cmdVel, cmdVel);
	
	while (fabs(error) > 0.01 && (error * cmdVel) > 0){
		refresh_sensorReadings(STATE_DEFAULT);
		getRobotPos(&x, &y, &t);
		error = normalizeAngle(targetAngle - t);
	}
	driveMotors(0, 0);
}

void movement_go_forward(int speed){
	driveMotors(speed, speed);
}
void movement_stop(){
	driveMotors(0, 0);
}

void driveMotors(int speedLeft, int speedRight){
	//buffer movement to smooth out the roots movement for the purpous of smaller errors in the position sensors
	speedLeft = (int)floor((double)(movements_last_values.SpeedL + speedLeft) / 2.0);
	speedRight = (int)floor((double)(movements_last_values.SpeedR + speedRight) / 2.0);
	setVel2(speedLeft, speedRight);
	movements_last_values.SpeedL = speedLeft;
	movements_last_values.SpeedR = speedRight;
	//setVel2(0, 0);
}
void movement_rotate(int speed, bool direction){
	if(direction == LEFT)
		driveMotors(-speed, speed);
	else
		driveMotors(speed, -speed);
}

void movement_travel_in_curve_radius(int speed, double radius){
	double radiusLarge = 0.0; 
	double radiusSmall = 0.0; 
	if(radius > 0){
		radiusLarge = radius + ROBOT_WIDTH/2;
		radiusSmall = radius - ROBOT_WIDTH/2;
	}else{
		radiusLarge = radius - ROBOT_WIDTH/2;
		radiusSmall = radius + ROBOT_WIDTH/2;
	}
	double speedLarge = speed*(radiusLarge/radius);
	double speedSmall = speed*(radiusSmall/radius);
	if(radius > 0){
		driveMotors(speedLarge, speedSmall);
	}else{
		driveMotors(speedSmall, speedLarge);
	}
}

void movement_travel_in_curve_radius_direction(int speed, double radius, bool direction){
	if(direction == RIGHT){
		movement_travel_in_curve_radius(speed, radius);
	}else{
		movement_travel_in_curve_radius(speed, -radius);
	}
}


