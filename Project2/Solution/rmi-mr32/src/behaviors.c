#include "behaviors.h"
#include "movements.h"
#include "config_constants.h"

#define STOP_AT_BEACON_ID 	0
#define AVOID_COLISION_ID 	1
#define FOLLOW_WALL_ID 		2
#define FOLLOW_BEACON_ID 	3
#define WOUNDER_ID 			4

#define LOW_PRIORITY 				1
#define HIGH_PRIORITY 				2

#define STOP_AT_BEACON_PRIORITY 	4
#define AVOID_COLISION_PRIORITY 	3
#define FOLLOW_WALL_PRIORITY 		2
#define FOLLOW_BEACON_PRIORITY 		1
#define WOUNDER_PRIORITY 			0

#define NUMBER_OF_BEHAVIORS		5

bool test_avoid_colision();
void avoid_colision();
bool test_follow_wall();
void follow_wall();
bool test_follow_beacon();
void follow_beacon();
bool test_stop_at_beacon();
void stop_at_beacon();
bool test_wounder();
void wounder();

void reset_default_priority_list();

typedef struct
{
	bool direction;
	bool isActive;
} Behavior_generic_vals;

SensorReadings behavior_sensorReadings;

typedef struct
{
	int default_priority_list[NUMBER_OF_BEHAVIORS];
	int priority_list[NUMBER_OF_BEHAVIORS];
} Behavior_struct_values;
Behavior_struct_values behavior_struct;
bool temp1;
void execute_behavior(SensorReadings sensorReading)
{
	//todo: array of beghaviors, array of priorities...	
	behavior_sensorReadings = sensorReading;
	if(test_avoid_colision())
	{
		avoid_colision();
	}
	else if(test_stop_at_beacon())
	{
		stop_at_beacon();
	}
	else if(test_follow_beacon())
	{
		follow_beacon();
	}
	else if(test_follow_wall() || temp1)
	{
		follow_wall();
		temp1 = true;
	}
	else
	{
		wounder();
	}
}

void check_behaviors(){
	
}

Behavior_generic_vals beh_av_col;
bool test_avoid_colision(){
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_COLISION_FRONT 
	|| behavior_sensorReadings.obstacleSensor.right >= DISTANCE_COLISION_SIDES
	|| behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_COLISION_SIDES)
	{
		return true;
	}
	else
	{
		beh_av_col.isActive = false;		
	}
	return false;
}
void avoid_colision(){
	if(beh_av_col.isActive == false)
	{
		beh_av_col.direction = (behavior_sensorReadings.obstacleSensor.right > behavior_sensorReadings.obstacleSensor.left) ? LEFT : RIGHT;
		beh_av_col.isActive = true;
	} 	
	movement_rotate(SPEED_ROTATE, beh_av_col.direction);
}


typedef struct
{
	Behavior_generic_vals turningVals;
	bool wall_side;
	bool isActive;
	bool isGoingAroundCorner;
	int goingAroundCornerCounter;
} Behavior_follow_wall_vals;
Behavior_follow_wall_vals beh_fol_wall;
bool test_follow_wall(){
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_FOLLOW_WALL_FRONT 
	|| behavior_sensorReadings.obstacleSensor.right >= DISTANCE_FOLLOW_WALL_SIDES
	|| behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_FOLLOW_WALL_SIDES)
	{
		return true;
	}
	return false;
}
void follow_wall(){
	if(beh_fol_wall.isActive == false)
	{
		beh_fol_wall.wall_side = (behavior_sensorReadings.obstacleSensor.right > behavior_sensorReadings.obstacleSensor.left) ? RIGHT : LEFT;
		beh_fol_wall.isActive = true;
	}
	//beh_fol_wall.wall_side = (behavior_sensorReadings.obstacleSensor.right > behavior_sensorReadings.obstacleSensor.left) ? RIGHT : LEFT;
		
	//printf("beh_fol_wall.isActive=%d, beh_fol_wall.wall_side=%d", beh_fol_wall.isActive, beh_fol_wall.wall_side);
	//printf("\n");	
	
	int distance_sensor_value = (beh_fol_wall.wall_side == RIGHT) ? behavior_sensorReadings.obstacleSensor.right : behavior_sensorReadings.obstacleSensor.left ;
	//printf("distance_sensor_value = %03d",distance_sensor_value);
	//printf("\n");	
	
	printf("front=%d, distance_sensor_value=%d", behavior_sensorReadings.obstacleSensor.front,distance_sensor_value);
	printf("\n");
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_FOLLOW_WALL_FRONT || distance_sensor_value >= DISTANCE_COLISION_SIDES)
	{			
		beh_fol_wall.turningVals.direction = !beh_fol_wall.wall_side;
		beh_fol_wall.turningVals.isActive = true;
		printf("WALL in front");
		printf("\n");	
		movement_rotate(SPEED_ROTATE, beh_fol_wall.turningVals.direction);
		return;
	}
	
	if(distance_sensor_value >= DISTANCE_FOLLOW_WALL_IDEAL_MAX && distance_sensor_value <= DISTANCE_FOLLOW_WALL_IDEAL_MIN)
	{
		movement_go_forward(SPEED_SLOW);
	}
	else
	{
		if(distance_sensor_value >= DISTANCE_FOLLOW_WALL_IDEAL_MIN)
		{
			printf("TURN_RADIUS_SMALL  ------beh_fol_wall.wall_side\n");
			movement_travel_in_curve_radius_direction(SPEED_FOLLOW_WALL, TURN_RADIUS_SMALL, !beh_fol_wall.wall_side);
		}
		else
		{
			printf("TURN_RADIUS_MEDIUM  ++++++beh_fol_wall.wall_side\n");
			movement_travel_in_curve_radius_direction(SPEED_FOLLOW_WALL, TURN_RADIUS_MEDIUM, beh_fol_wall.wall_side);
			//todo malo poboljšati ovo			
		}
	}
	//printf("\n");
}

bool test_follow_beacon(){
	if(behavior_sensorReadings.beaconSensor.isVisible == true)
	{
		return true;
	}
	return false;
}
void follow_beacon(){
	if(fabs(behavior_sensorReadings.beaconSensor.relative_direction) > (M_PI/3))//30 degrees
	{
		bool direction = (behavior_sensorReadings.beaconSensor.relative_direction > 0) ? RIGHT : LEFT;
		movement_travel_in_curve_radius_direction(SPEED_MEDIUM, TURN_RADIUS_SMALL, direction);
	}
	else
	{
		movement_go_forward(SPEED_MEDIUM);
	}
}

bool test_stop_at_beacon(){
	if(behavior_sensorReadings.atBeaconArea == true)
	{
		return true;
	}
	return false;
}
void stop_at_beacon(){
	printf("..............STOP_AT_BEACON_AREA.........\n");
	/*victory dance*/
	rotateRel_naive(-M_PI/4);
	rotateRel_naive(M_PI/2);
	rotateRel_naive(-M_PI/4);
	movement_stop();
}

bool test_wounder(){
	return true;
}
void wounder(){
	movement_go_forward(SPEED_MEDIUM);
}

void reset_default_priority_list()
{
	int i;
	for(i=0; i<NUMBER_OF_BEHAVIORS; i++)
	{
		behavior_struct.priority_list[i] = behavior_struct.default_priority_list[i];
	}
}

void behaviors_init(){
	
	behavior_struct.default_priority_list[STOP_AT_BEACON_ID] = STOP_AT_BEACON_PRIORITY;
	behavior_struct.default_priority_list[AVOID_COLISION_ID] = AVOID_COLISION_PRIORITY;
	behavior_struct.default_priority_list[FOLLOW_WALL_ID] = FOLLOW_WALL_PRIORITY;
	behavior_struct.default_priority_list[FOLLOW_BEACON_ID] = FOLLOW_BEACON_PRIORITY;
	behavior_struct.default_priority_list[WOUNDER_ID] = WOUNDER_PRIORITY;
	
	reset_default_priority_list();
	
	beh_fol_wall.isActive = false;
	beh_fol_wall.isActive = false;
	beh_fol_wall.turningVals.isActive = false;
	beh_fol_wall.isGoingAroundCorner = false;
	beh_fol_wall.goingAroundCornerCounter = 0;
}
void behaviors_finish(){
	movement_stop();
};



