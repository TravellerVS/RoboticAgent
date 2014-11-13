#include <math.h>
#include "behaviors.h"
#include "movements.h"
#include "config_constants.h"

#define STOP_AT_BEACON_ID 	0
#define AVOID_COLISION_ID 	1
#define FOLLOW_BEACON_ID 	2
#define FOLLOW_WALL_ID 		3
#define WOUNDER_ID 			4

#define STOP_AT_STARTING_POINT_ID		5
#define FOLLOW_STARTING_POINT_ID		6

//lower value means a high priority
#define HIGH_PRIORITY 				1
#define LOW_PRIORITY 				3

#define STOP_AT_BEACON_PRIORITY 	0
#define AVOID_COLISION_PRIORITY 	1
#define FOLLOW_BEACON_PRIORITY 		2
#define AVOID_COLISION_PRIORITY2 	3
#define FOLLOW_WALL_PRIORITY 		4
#define WOUNDER_PRIORITY 			5

#define NUMBER_OF_BEHAVIORS		8



bool test_avoid_colision();
void avoid_colision();
void reset_avoid_colision();

bool test_follow_wall();
void follow_wall();
void reset_follow_wall();

bool test_follow_beacon();
void follow_beacon();
void reset_follow_beacon();

bool test_stop_at_beacon();
void stop_at_beacon();
void reset_stop_at_beacon();

bool test_wounder();
void wounder();
void reset_wounder();

void reset_stop_at_starting_point();
bool test_stop_at_starting_point();
void stop_at_starting_point();

void reset_follow_starting_point();
bool test_follow_starting_point();
void follow_starting_point();


void reset_default_priority_list();
void set_return_priority_list();
int get_priority_from_id(int id);
void change_behavior_priority(int behavior_id, int new_priority);
void signal_long_LED();
void signal_short_LED();
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
	PositionSensor follow_wall_starting_point;//x and y from robot but t from the beacon direction
	int at_beacon_area_counter;
	
} Behavior_struct_values;
Behavior_struct_values behavior_struct;

bool temp1;
int behaviors_state = STATE_SEARCHING_FOR_BEACON;

double next_point_rel_dir = 0.0;
int execute_behavior(int behavior_control)
{
	bool can_execute_list[NUMBER_OF_BEHAVIORS];
	
	behavior_sensorReadings = get_new_sensorReadings(behaviors_state);;
	
	can_execute_list[STOP_AT_BEACON_ID] = false;
	can_execute_list[FOLLOW_BEACON_ID] = false;
	can_execute_list[FOLLOW_WALL_ID] = false;
	
	//for demonstarion
	//for changing behaviors from all to avoid colisio, to follow wall and to follow and stop at beacon
	if(behavior_control == 1)
	{
		//only avoid collision behavior
		behaviors_state = STATE_RETURNING_HOME;
	}
	else if(behavior_control == 2)
	{
		can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
		behaviors_state = STATE_RETURNING_HOME;
	}
	else if(behavior_control == 3)
	{
		can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
		can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
		behaviors_state = STATE_SEARCHING_FOR_BEACON;
	}
	else
	{
		can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
		can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
		can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
	}
	
	if(behaviors_state == STATE_RETURNING_HOME)
	{
		can_execute_list[STOP_AT_BEACON_ID] = false;
		can_execute_list[FOLLOW_BEACON_ID] = false;
	}
	//can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
	can_execute_list[AVOID_COLISION_ID] = test_avoid_colision();
	//can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
	//can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
	can_execute_list[WOUNDER_ID] = test_wounder();
	can_execute_list[STOP_AT_STARTING_POINT_ID] = test_stop_at_starting_point();
	can_execute_list[FOLLOW_STARTING_POINT_ID] = test_follow_starting_point();
	
	int follow_wall_priority = get_priority_from_id(FOLLOW_WALL_ID);
	//check if there is a follow wall and follow beacon can execute and if the wall is between the robot and the beacon
	
	if(behaviors_state == STATE_RETURNING_HOME)
	{
		can_execute_list[STOP_AT_BEACON_ID] = false;
		can_execute_list[FOLLOW_BEACON_ID] = false;
	}
	else
	{
		can_execute_list[STOP_AT_STARTING_POINT_ID] = false;
		can_execute_list[FOLLOW_STARTING_POINT_ID] = false;
	}
	
	
	/**
	 * changing the priority list of the behaviors
	 * */
	
	/*
	if(can_execute_list[FOLLOW_WALL_ID] == true && can_execute_list[AVOID_COLISION_ID] == true && follow_wall_priority!= HIGH_PRIORITY
		&& next_point_rel_dir!=0.0){		
		change_behavior_priority(FOLLOW_WALL_ID, HIGH_PRIORITY);//promote behavior
		behavior_struct.follow_wall_starting_point.x = behavior_sensorReadings.positionSensor.x;
		behavior_struct.follow_wall_starting_point.y = behavior_sensorReadings.positionSensor.y;
		behavior_struct.follow_wall_starting_point.t = next_point_rel_dir;
		printf("CHANGE_PRIORITY start_dir=%f\n",behavior_struct.follow_wall_starting_point.t);
		signal_short_LED();
	}else*/
	if(can_execute_list[FOLLOW_WALL_ID] == true && can_execute_list[FOLLOW_BEACON_ID] == true && follow_wall_priority!= HIGH_PRIORITY
		&&(	(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_NEAR_FRONT && fabs(behavior_sensorReadings.beaconSensor.relative_direction) <= (M_PI/12))//from-30 to 30
			|| (behavior_sensorReadings.obstacleSensor.right >= DISTANCE_NEAR_SIDES && behavior_sensorReadings.beaconSensor.relative_direction >= (M_PI/4))//>=30
			|| (behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_NEAR_SIDES && behavior_sensorReadings.beaconSensor.relative_direction <= -(M_PI/4))//<=-30
			)
		){
		change_behavior_priority(FOLLOW_WALL_ID, HIGH_PRIORITY);//promote behavior
		behavior_struct.follow_wall_starting_point.x = behavior_sensorReadings.positionSensor.x;
		behavior_struct.follow_wall_starting_point.y = behavior_sensorReadings.positionSensor.y;
		behavior_struct.follow_wall_starting_point.t = behavior_sensorReadings.beaconSensor.apsolute_direction;
		printf("CHANGE_PRIORITY start_dir=%f\n",behavior_struct.follow_wall_starting_point.t);		
		signal_short_LED();	
		
	}else if(follow_wall_priority == HIGH_PRIORITY 
		//&& 
		//(behavior_struct.follow_wall_starting_point.t < (aprox_angle * 1.2) 
		//&& behavior_struct.follow_wall_starting_point.t > (aprox_angle * 0.8))
		){
		double dx = (behavior_sensorReadings.positionSensor.x - behavior_struct.follow_wall_starting_point.x);
		double dy = (behavior_sensorReadings.positionSensor.y - behavior_struct.follow_wall_starting_point.y);
		double dz = sqrt(dx*dx+dy*dy);
		double aprox_angle = atan2(dy,dx);
		//double angle = behavior_struct.follow_wall_starting_point.t;
		//double start_point_angle = (behavior_struct.follow_wall_starting_point.t<0) ? (behavior_struct.follow_wall_starting_point.t + 2*M_PI) : behavior_struct.follow_wall_starting_point.t;
		//aprox_angle = (aprox_angle<0) ? (aprox_angle + 2*M_PI) : aprox_angle;
		
		aprox_angle = behavior_struct.follow_wall_starting_point.t- aprox_angle;
		while(aprox_angle<-M_PI)
			aprox_angle += 2*M_PI;
		while(aprox_angle>M_PI)
			aprox_angle -= 2*M_PI;
		printf("start_point_angle=%f, aprox_angle=%f  , dz=%f \n",behavior_struct.follow_wall_starting_point.t, aprox_angle, dz);		
		//signal_long_LED();
		
		//if(start_point_angle < (aprox_angle * 1.2) 
		//&& start_point_angle > (aprox_angle * 0.8))
		if(fabs(aprox_angle) <= (M_PI/12) && dz>=DISTANCE_XY_CLOSE){
			printf("..................RETURN_PRIORITY..................\n");
			signal_short_LED();			
			change_behavior_priority(FOLLOW_WALL_ID, FOLLOW_WALL_PRIORITY);//return normal priority
			reset_follow_wall();
			if(behavior_sensorReadings.beaconSensor.isVisible == true){
				rotateRel_naive(behavior_sensorReadings.beaconSensor.relative_direction);
			}else{
				aprox_angle = behavior_struct.follow_wall_starting_point.t - behavior_sensorReadings.positionSensor.t;
				rotateRel_naive(aprox_angle);
			}			
		}else{
			printf("..................CONINUE..................\n");
			//continue and wait until something changes eg. when follow wall finishes
		}
	}else{
		//continue and wait until something changes eg. when follow wall finishes
	}
	int priority;
	for(priority=0;priority<NUMBER_OF_BEHAVIORS;priority++){
		if(can_execute_list[behavior_struct.priority_list[priority]] == true){
			leds(behavior_struct.priority_list[priority]);
			switch ( behavior_struct.priority_list[priority] ) {				
				case STOP_AT_BEACON_ID:
					stop_at_beacon();
					//if stopped at beacon  exchange this behavior for stop at starting point
					behavior_struct.at_beacon_area_counter++;
					if(behavior_struct.at_beacon_area_counter>10)
					{
						/**
						 * preparing for returning home
						 * */
						//signal that beacon 
						signal_long_LED();		
						set_return_priority_list();
						behaviors_state = STATE_RETURNING_HOME;
					}					
					break;
				case STOP_AT_STARTING_POINT_ID:
					stop_at_starting_point();
					signal_long_LED();
					return 1;
					break;
				case AVOID_COLISION_ID:
					avoid_colision();
					break;
				case FOLLOW_BEACON_ID:
					follow_beacon();
					reset_follow_wall();
					break;
				case FOLLOW_STARTING_POINT_ID:
					follow_starting_point();
					reset_follow_wall();
					break;
				case FOLLOW_WALL_ID:
					follow_wall();				
					break;
				case WOUNDER_ID:
					wounder();
					break;
				default:
					wounder();
					break;
			}
			break;
		}
	}
	return 0;
}

Behavior_generic_vals beh_av_col;
void reset_avoid_colision(){
	beh_av_col.isActive = false;	
}
bool test_avoid_colision(){
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_COLISION_FRONT 
	|| behavior_sensorReadings.obstacleSensor.right >= DISTANCE_COLISION_SIDES
	|| behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_COLISION_SIDES)
	{
		return true;
	}else{
		reset_avoid_colision();
	}
	return false;
}
void avoid_colision(){
	printf("AVOID_COLLISION \n");
	if(beh_av_col.isActive == false){
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
	int state;
	double wallAngle;
	bool wallAngleSet;
	int curveCounter;
} Behavior_follow_wall_vals;
Behavior_follow_wall_vals beh_fol_wall;
void reset_follow_wall(){
	beh_fol_wall.isActive = false;
	beh_fol_wall.isActive = false;
	beh_fol_wall.turningVals.isActive = false;
	beh_fol_wall.state = 0;
	beh_fol_wall.wallAngle = 0.0;
	beh_fol_wall.wallAngleSet = false;
	beh_fol_wall.curveCounter = 0;
}
bool test_follow_wall(){
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_FOLLOW_WALL_FRONT 
	|| behavior_sensorReadings.obstacleSensor.right >= DISTANCE_FOLLOW_WALL_SIDES
	|| behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_FOLLOW_WALL_SIDES
	|| beh_fol_wall.isActive == true )
	{
		return true;
	}
	return false;
}
void follow_wall(){
	printf("FOLLOW_WALL \n");
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
	
	//printf("front=%d, distance_sensor_value=%d", behavior_sensorReadings.obstacleSensor.front,distance_sensor_value);
	//printf("\n");
	if(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_FOLLOW_WALL_FRONT /*&& distance_sensor_value >= DISTANCE_COLISION_SIDES*/)
	{			
		beh_fol_wall.turningVals.direction = !beh_fol_wall.wall_side;
		beh_fol_wall.turningVals.isActive = true;
		//printf("WALL in front");
		//printf("\n");
		movement_rotate(SPEED_ROTATE, beh_fol_wall.turningVals.direction);
		return;
	}
	
	if(beh_fol_wall.state!=2 && beh_fol_wall.wallAngleSet)
	{
		beh_fol_wall.curveCounter = 0;
	}
	
	if(distance_sensor_value >= DISTANCE_FOLLOW_WALL_IDEAL_MAX && distance_sensor_value <= DISTANCE_FOLLOW_WALL_IDEAL_MIN)
	{
		beh_fol_wall.state=1;
		beh_fol_wall.wallAngleSet = true;
		beh_fol_wall.wallAngle=behavior_sensorReadings.positionSensor.t;
		movement_go_forward(SPEED_FOLLOW_WALL);		
	}
	else
	{		
		if(distance_sensor_value >= DISTANCE_FOLLOW_WALL_IDEAL_MIN)
		{
			beh_fol_wall.wallAngleSet = false;
			beh_fol_wall.state=0;
			//printf("TURN_RADIUS_SMALL  ------beh_fol_wall.wall_side\n");
			movement_travel_in_curve_radius_direction(SPEED_FOLLOW_WALL, TURN_RADIUS_SMALL, !beh_fol_wall.wall_side);
		}
		else
		{
			if(beh_fol_wall.state!=2 && beh_fol_wall.wallAngleSet)
			{
				beh_fol_wall.state=2;
				double angle = (beh_fol_wall.wall_side == RIGHT) ? (beh_fol_wall.wallAngle+M_PI/8) : (beh_fol_wall.wallAngle-M_PI/8);
				beh_fol_wall.wallAngleSet = false;				
				angle = angle - behavior_sensorReadings.positionSensor.t;
				rotateRel_naive(angle);
			}
			beh_fol_wall.curveCounter++;
			if(beh_fol_wall.curveCounter>=200)
			{
				reset_follow_wall();
			}
			//printf("TURN_RADIUS_MEDIUM  ++++++beh_fol_wall.wall_side\n");
			movement_travel_in_curve_radius_direction(SPEED_FOLLOW_WALL, TURN_RADIUS_GO_ROUND_CORNER, beh_fol_wall.wall_side);
			//todo malo poboljšati ovo			
		}
	}
	//printf("\n");
}
void reset_follow_beacon(){
	
}
bool test_follow_beacon(){
	if(behavior_sensorReadings.beaconSensor.isVisible == true)
	{
		return true;
	}
	return false;
}
void follow_beacon(){
	printf("FOLLOW_BEACON \n");
	if(fabs(behavior_sensorReadings.beaconSensor.relative_direction) > (M_PI/6))//30 degrees
	{
		bool direction = (behavior_sensorReadings.beaconSensor.relative_direction > 0) ? RIGHT : LEFT;
		movement_travel_in_curve_radius_direction(SPEED_MEDIUM, TURN_RADIUS_SMALL, direction);
		//rotateRel_naive(behavior_sensorReadings.beaconSensor.relative_direction);
	}
	else
	{
		movement_go_forward(SPEED_MEDIUM);
	}
}

void reset_stop_at_beacon(){
	
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
	movement_stop();
}

void reset_wounder(){
	
}
bool test_wounder(){
	return true;
}
void wounder(){
	//printf("WOUNDER \n");
	movement_go_forward(SPEED_MEDIUM);
}

void reset_stop_at_starting_point(){
	
}
bool test_stop_at_starting_point(){	
	behavior_sensorReadings.startingPosition.x = 0.0;
	behavior_sensorReadings.startingPosition.y = 0.0;
	double dx = fabs(behavior_sensorReadings.startingPosition.x - behavior_sensorReadings.positionSensor.x);
	double dy = fabs(behavior_sensorReadings.startingPosition.y - behavior_sensorReadings.positionSensor.y);
	
	//double dx = fabs(0.0 - behavior_sensorReadings.positionSensor.x);
	//double dy = fabs(0.0 - behavior_sensorReadings.positionSensor.y);
	double dz = sqrt(dx*dx+dy*dy);
	
	if(dz<=STARTING_POINT_BUFFER){
		return true;
	}	
	return false;
}
void stop_at_starting_point(){
	printf("..............STOP_AT_STARTING_POINT.........\n");
	/*victory dance*/
	movement_stop();	
}
int follow_starting_point_last_point;
void reset_follow_starting_point(){
	follow_starting_point_last_point = behavior_sensorReadings.last_position_index;
}
bool test_follow_starting_point(){
	return true;
}
void follow_starting_point(){	
	//follow_starting_point_last_point = 0;
	double dest_x;
	double dest_y;
	if(follow_starting_point_last_point<=0)
	{
		dest_x = behavior_sensorReadings.startingPosition.x;
		dest_y = behavior_sensorReadings.startingPosition.y;
		follow_starting_point_last_point= 0;
	}
	else
	{
		dest_x = behavior_sensorReadings.positionsHistory[follow_starting_point_last_point].x;
		dest_y = behavior_sensorReadings.positionsHistory[follow_starting_point_last_point].y;	
	}
	double dx = behavior_sensorReadings.positionSensor.x - dest_x;
	double dy = behavior_sensorReadings.positionSensor.y - dest_y;
	double dz = sqrt(dx*dx+dy*dy);
	int i=0;
	for(i= follow_starting_point_last_point-1;i>0; i--)
	{
		double tdx = dest_x - behavior_sensorReadings.positionsHistory[i].x;
		double tdy = dest_x - behavior_sensorReadings.positionsHistory[i].y;
		double tdz = sqrt(tdx*tdx+tdy*tdy);
		if(tdz<OTHER_POINT_BUFFER){
			follow_starting_point_last_point = i;
			follow_starting_point();
			return;
		}		
	}
	
	if(dz<=OTHER_POINT_BUFFER && follow_starting_point_last_point!=0){
		follow_starting_point_last_point--;
		follow_starting_point();
		return;
	}
	
	double relative_direction = atan2(dy, dx) - behavior_sensorReadings.positionSensor.t;
	relative_direction = (relative_direction>M_PI) ? relative_direction-2*M_PI : relative_direction;
	relative_direction = (relative_direction<-M_PI) ? relative_direction+2*M_PI : relative_direction;	
	next_point_rel_dir=relative_direction;
	printf("follow_starting_point_last_point =%d, x=%5.3f, y=%5.3f, t=%5.3f, relAngle=%5.3f , startX=%5.3f, startY=%5.3f \n",
		follow_starting_point_last_point,
		behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, behavior_sensorReadings.positionSensor.t, 
		relative_direction, 
		dest_x, dest_y );
	
	//printf("FOLLOW_STARTING_POINT \n");
	if(fabs(relative_direction) > (M_PI/6))//30 degrees
	{
		printf("ROTATE \n");
		//rotateRel_naive(relative_direction);
		bool direction = (relative_direction > 0) ? RIGHT : LEFT;
		movement_travel_in_curve_radius_direction(SPEED_MEDIUM, TURN_RADIUS_SMALL, direction);
	}
	else
	{
		printf("FORWARD \n");
		movement_go_forward(SPEED_MEDIUM);
	}
}

void signal_long_LED(){
	movement_stop();
	leds(15);
	wait(2);
	leds(0);
	wait(2);
	leds(15);
	wait(2);
	leds(0);
}
void signal_short_LED(){
	movement_stop();
	leds(15);
	wait(2);
	leds(0);
}

int get_priority_from_id(int id)
{
	int priority;
	for(priority=0; priority<NUMBER_OF_BEHAVIORS; priority++){		
		if(behavior_struct.priority_list[priority] == id){
			return priority;	
		}
	}
	return -1;
}
void change_behavior_priority(int behavior_id, int new_priority)
{
	int priority;
	int current_priority=-1;
	for(priority=0; priority<NUMBER_OF_BEHAVIORS; priority++){		
		if(behavior_struct.priority_list[priority] == behavior_id){
			current_priority = priority;
			break;			
		}
	}
	//printf("current_priority:%d, behavior_id:%d, new_priority:%d \n",current_priority, behavior_id,new_priority);
	if(current_priority==-1)
		return;
	if(current_priority<new_priority){		
		for(priority=current_priority; priority<new_priority ; priority++){	
			//reverse values of element at index==priority and index==priority+1    hack->reversing values without using a temp variable for storing a value a=2,b=5; a=a+b; b=a-b; a= a-b; -> a=5,b=2
			behavior_struct.priority_list[priority] += behavior_struct.priority_list[priority+1];
			behavior_struct.priority_list[priority+1] = behavior_struct.priority_list[priority] - behavior_struct.priority_list[priority+1];
			behavior_struct.priority_list[priority] -= behavior_struct.priority_list[priority+1];
			//printf("change1:%d %d\n",behavior_struct.priority_list[priority], behavior_struct.priority_list[priority+1]);
		}
	}else if(current_priority>new_priority){
		for(priority=current_priority; priority>new_priority ; priority--){	
			//reverse values of element at index==priority and index==priority-1    hack->reversing values without using a temp variable for storing a value
			behavior_struct.priority_list[priority] += behavior_struct.priority_list[priority-1];
			behavior_struct.priority_list[priority-1] = behavior_struct.priority_list[priority] - behavior_struct.priority_list[priority-1];
			behavior_struct.priority_list[priority] -= behavior_struct.priority_list[priority-1]; 
			//printf("change2:%d %d\n",behavior_struct.priority_list[priority], behavior_struct.priority_list[priority-1]);
		}
	}else{
		//allredy at that priority
		return;
	}
	/*
	for(priority=0; priority<NUMBER_OF_BEHAVIORS; priority++){
		printf("%d",behavior_struct.priority_list[priority]);
	}
	 * */
}

void reset_default_priority_list()
{
	int priority;
	for(priority=0; priority<NUMBER_OF_BEHAVIORS; priority++){
		behavior_struct.priority_list[priority] = behavior_struct.default_priority_list[priority];
	}
}

void set_return_priority_list()
{
	behavior_struct.priority_list[0] = STOP_AT_STARTING_POINT_ID;
	behavior_struct.priority_list[1] = AVOID_COLISION_ID;	
	behavior_struct.priority_list[2] = FOLLOW_STARTING_POINT_ID;
	behavior_struct.priority_list[3] = -1;//AVOID_COLISION_ID;	
	behavior_struct.priority_list[4] = -1;//FOLLOW_WALL_ID;
	behavior_struct.priority_list[5] = WOUNDER_ID;
	reset_stop_at_starting_point();	
	reset_avoid_colision();
	reset_follow_starting_point();
	reset_follow_wall();
	reset_wounder();	
}

void behaviors_init(){
	behaviors_state = STATE_SEARCHING_FOR_BEACON;
	behavior_struct.default_priority_list[STOP_AT_BEACON_PRIORITY] = STOP_AT_BEACON_ID;
	behavior_struct.default_priority_list[AVOID_COLISION_PRIORITY] = AVOID_COLISION_ID;	
	behavior_struct.default_priority_list[FOLLOW_BEACON_PRIORITY] = FOLLOW_BEACON_ID;
	//behavior_struct.default_priority_list[AVOID_COLISION_PRIORITY2] = AVOID_COLISION_ID;	
	behavior_struct.default_priority_list[FOLLOW_WALL_PRIORITY] = FOLLOW_WALL_ID;
	behavior_struct.default_priority_list[WOUNDER_PRIORITY] = WOUNDER_ID;
	
	reset_default_priority_list();
	
	reset_avoid_colision();
	reset_follow_wall();
	reset_follow_beacon();
	reset_stop_at_beacon();
	reset_wounder();	
}
void behaviors_finish(){
	movement_stop();
};



