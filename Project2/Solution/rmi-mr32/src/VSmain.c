#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define SPEED_NORMAL 50
#define SPEED_FAST 50
#define SPEED_SLOW 50

#define ROBOT_WHEEL_DISTANCE 20

void rotateRel_naive(double deltaAngle);

int main(void)
{
	int groundSensor;

	initPIC32();
	closedLoopControl( true );
	setVel2(0, 0);

	printf("RMI-example, robot %d\n\n\n", ROBOT);

	while(1)
	{
		printf("Press start to continue\n");
		while(!startButton());
		enableObstSens();

		rotateRel_naive(M_PI / 2);
		do
		{
			waitTick40ms();						// Wait for next 40ms tick
			readAnalogSensors();				// Fill in "analogSensors" structure
			groundSensor = readLineSensors(0);	// Read ground sensor
			printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d, Ground_sens=", analogSensors.obstSensLeft,
							analogSensors.obstSensFront, analogSensors.obstSensRight, analogSensors.batteryVoltage);

			printInt(groundSensor, 2 | 5 << 16);	// System call
			printf("\n");
		} while(!stopButton());
		disableObstSens();
		rotateRel_naive(-M_PI / 2);
	}
	return 0;
}

void stop(){
	setVel2(0, 0);
}

void moveForward(int speed){
	setVel2(speed, speed);
}

void moveInArc(int speed, int radius_cm){
		
}

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
		cmdVel = -30;
	else
		cmdVel = 30;

	setVel2(-cmdVel, cmdVel);

	do
	{
		getRobotPos(&x, &y, &t);
		error = normalizeAngle(targetAngle - t);
	} while (fabs(error) > 0.01 && (error * cmdVel) > 0);
	setVel2(0, 0);
}

