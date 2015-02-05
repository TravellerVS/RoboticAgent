#include <math.h>
#include "behaviors.h"
#include "movements.h"
#include "config_constants.h"
#include "map.h"
#include "sensors.h"
#include "helperFunctions.h"

/** \brief pseudo enumeration of the behaviors
 * */
#define STOP_AT_BEACON_ID 	0
#define AVOID_COLISION_ID 	1
#define FOLLOW_BEACON_ID 	2
#define FOLLOW_WALL_ID 		3
#define WOUNDER_ID 			4

#define STOP_AT_STARTING_POINT_ID	5
#define FOLLOW_STARTING_POINT_ID	6

#define LABYRINTH_EXPLORE_ID 			7
#define LABYRINTH_RETURN_HOME_ID		8
#define FOLLOW_PATH_ID					9
#define CORRECT_POSITION_ID				10

#define NUMBER_OF_BEHAVIORS		11

/** \brief pseudo enumeration for priority, detailed lower value means a high priority
 * */
#define HIGH_PRIORITY 				1
#define LOW_PRIORITY 				3

#define STOP_AT_BEACON_PRIORITY 	0
#define AVOID_COLISION_PRIORITY 	1
#define FOLLOW_BEACON_PRIORITY 		2
#define FOLLOW_WALL_PRIORITY 		3
#define WOUNDER_PRIORITY 			4

/** \brief list of usefull advanced movement functions that require reading the sensor data
 * \return they return true if the movement is acomplished and false in case it still needs to be called in the next cycle
 * \example rotate_to_angle will return false unltil the angle position of the robot is reached (with a small error window)
 * 			when the angle is reachet the function reurns true indicating that the destination angle is reached
 * */ 
bool rotate_to_angle(double targetAngle, double tolerance);
bool move_to_point(double dest_x, double dest_y);

/** \brief list of behavior functions
 * \details each behaviour has a test, reset and execution function
 * 			-the test function returns if the behavior is able to execut acording to the sensor readings and other conditions
 * 			-the reset function resets the behavior to its initial state, must be called when priority list has changed
 * 			-the execution function executes the behaviour calling the apropriate movment function to get the apropriate result
 * */
bool test_avoid_colision();
void avoid_colision();
void reset_avoid_colision();

bool test_stop_at_beacon();
void stop_at_beacon();
void reset_stop_at_beacon();

bool test_wounder();
void wounder();
void reset_wounder();

bool test_stop_at_starting_point();
void stop_at_starting_point();
void reset_stop_at_starting_point();

bool test_follow_starting_point();
void follow_starting_point();
void reset_follow_starting_point();

bool test_labyrinth_explore();
void labyrinth_explore();
void reset_labyrinth_explore();

bool test_labyrinth_return_home();
void labyrinth_return_home();
void reset_labyrinth_return_home();

void set_follow_path_to_unexplored_field();
void set_follow_path_to_return_home();
void reset_follow_path();
bool test_follow_path();
void follow_path();

bool check_current_position_offset();
bool test_correct_position();
void correct_position();
void reset_correct_position();

void reset_default_priority_list();
void set_return_priority_list();
int get_priority_from_id(int id);
void change_behavior_priority(int behavior_id, int new_priority);
void signal_long_LED();
void signal_short_LED();

void signal_ERROR();

typedef struct
{
	bool direction;
	bool isActive;
} Behavior_generic_vals;

static SensorReadings behavior_sensorReadings;

typedef struct
{
	int default_priority_list[NUMBER_OF_BEHAVIORS];
	int priority_list[NUMBER_OF_BEHAVIORS];
	PositionSensor follow_wall_starting_point;//x and y from robot but t from the beacon direction
	int at_beacon_area_counter;
	int init_state_counter;
	
} Behavior_struct_values;
static Behavior_struct_values behavior_struct;

static int behaviors_state = STATE_INIT;

int execute_behavior(int behavior_control)
{
	//~ printf("behavior BEGIN\n");
	bool can_execute_list[NUMBER_OF_BEHAVIORS];
	behavior_sensorReadings = get_new_filteredSensorReadings(behaviors_state);
	
	//~ printf("behavior_sensorReadings: x=%5.3f, y=%5.3f, t=%5.3f,\n",behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, behavior_sensorReadings.positionSensor.t);
	//~ behavior_sensorReadings = get_new_sensorReadings(behaviors_state);
	//~ return 0;
	can_execute_list[AVOID_COLISION_ID] = false;//test_avoid_colision();
	
	//~ printf("behaviors_state = %d  \n",behaviors_state);
		
	//debug
	//behaviors_state = STATE_SEARCHING_FOR_BEACON_AREA;
	if(behaviors_state == STATE_RETURNING_HOME){
		can_execute_list[STOP_AT_STARTING_POINT_ID] = test_stop_at_starting_point();
		can_execute_list[STOP_AT_BEACON_ID] = false;
		can_execute_list[LABYRINTH_EXPLORE_ID] = false;
		can_execute_list[CORRECT_POSITION_ID] = test_correct_position();
		can_execute_list[FOLLOW_PATH_ID] = test_follow_path();
		can_execute_list[LABYRINTH_RETURN_HOME_ID] = test_labyrinth_return_home();
	}else if(behaviors_state == STATE_INIT){
		can_execute_list[STOP_AT_STARTING_POINT_ID] = false;
		can_execute_list[STOP_AT_BEACON_ID] = false;
		can_execute_list[CORRECT_POSITION_ID] = false;
		can_execute_list[FOLLOW_PATH_ID] = false;
		can_execute_list[LABYRINTH_EXPLORE_ID] = false;		
		can_execute_list[LABYRINTH_RETURN_HOME_ID] = false;
	}else if(behaviors_state == STATE_SEARCHING_FOR_BEACON_AREA){
		can_execute_list[STOP_AT_STARTING_POINT_ID] = false;
		can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
		can_execute_list[CORRECT_POSITION_ID] = test_correct_position();
		can_execute_list[FOLLOW_PATH_ID] = test_follow_path();
		can_execute_list[LABYRINTH_EXPLORE_ID] = test_labyrinth_explore(); //true		
		can_execute_list[LABYRINTH_RETURN_HOME_ID] = false;
	}else{
		printf("ERROR - behaviour state is wrong \n");
		signal_ERROR();
	}
	/**
	 * changing the priority list of the behaviors
	 * */
	int priority;
	
	for(priority=0;priority<NUMBER_OF_BEHAVIORS;priority++){
		if(can_execute_list[behavior_struct.priority_list[priority]] == true){
			//~ leds(behavior_struct.priority_list[priority]);
			switch ( behavior_struct.priority_list[priority] ) {				
				case STOP_AT_BEACON_ID:
					//if stopped at beacon  exchange this behavior for stop at starting point
					behavior_struct.at_beacon_area_counter++;
					if(behavior_struct.at_beacon_area_counter>10){
						stop_at_beacon();
						/**
						 * preparing for returning home
						 * */
						//signal that beacon is found 
						double distance = 0.0;
						PositionXY position;
						position.x = behavior_sensorReadings.positionSensor.x;
						position.y = behavior_sensorReadings.positionSensor.y;
						MapField *beaconAreaField;
						beaconAreaField = get_closest_field(position, &distance);
						set_GoalField(beaconAreaField);
						
						signal_long_LED();		
						set_return_priority_list();
						behaviors_state = STATE_RETURNING_HOME;
					}					
					break;
				case STOP_AT_STARTING_POINT_ID:
					stop_at_starting_point();
					print_out_map();
					signal_long_LED();
					signal_long_LED();
					signal_long_LED();
					signal_long_LED();
					return 1;
					break;
				case LABYRINTH_EXPLORE_ID:
					labyrinth_explore();
					break;
				case CORRECT_POSITION_ID:
					correct_position();
					break;
				case FOLLOW_PATH_ID:
					follow_path();
					break;
				case LABYRINTH_RETURN_HOME_ID:
					labyrinth_return_home();
					break;			
				case AVOID_COLISION_ID:
					avoid_colision();
					break;
				default:
					printf("ERROR - no state can execute \n");
					signal_ERROR();
					break;
			}
			break;
		}
		else{
			//~ printf("behaviour (id=%d, priority=%d) cannot execute \n",behavior_struct.priority_list[priority], priority);
		}
	}
	return 0;
}

static Behavior_generic_vals beh_av_col;
void reset_avoid_colision(){
	beh_av_col.isActive = false;	
}
bool test_avoid_colision(){
	//debug
	//~ return false;
	if(behavior_sensorReadings.obstacleSensor.front <= DISTANCE_COLISION_FRONT 
	|| behavior_sensorReadings.obstacleSensor.right <= DISTANCE_COLISION_SIDES
	|| behavior_sensorReadings.obstacleSensor.left  <= DISTANCE_COLISION_SIDES)
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
		beh_av_col.direction = (behavior_sensorReadings.obstacleSensor.right < behavior_sensorReadings.obstacleSensor.left) ? LEFT : RIGHT;
		beh_av_col.isActive = true;
	} 	
	movement_rotate(SPEED_ROTATE, beh_av_col.direction);
}

bool move_to_point(double dest_x, double dest_y)
{
	double distance = distance_between_points(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, dest_x, dest_y);
	bool result = (distance<=DISTANCE_XY_CLOSE) ? true : false;
	//~ printf("move_to_point values: x=%f, y=%f, t=%f, destx=%f, desty=%f \n",behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, behavior_sensorReadings.positionSensor.t, dest_x, dest_y);
	if(result){
		//~ printf(" stop \n");	
		//~ movement_stop();
		//~ wait(1);
	}else{
		leds(0);
		double absolute_angle = angle_between_points(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, dest_x, dest_y);
		double angleDifference = angle_difference(behavior_sensorReadings.positionSensor.t,absolute_angle);
		
		//~ printf(" forward %5.3f\n", speed);
		
		if(rotate_to_angle(absolute_angle,(2*M_PI)/36)){
			//buffered movement -> it slows down if when it's close to the target
			//~ printf(" forward ");
			int speed = (int)((distance/DISTANCE_BETWEEN_POINTS)*((double)SPEED_FAST));
			if(speed<SPEED_BEFORE_STOP){
				speed=SPEED_BEFORE_STOP;
			}else if(speed>SPEED_MEDIUM){
				speed=SPEED_MEDIUM;
			}
			double tolerance = (2*M_PI)/72;//5 degrees
			if(fabs(angleDifference) < tolerance){
				movement_go_forward(speed);
			}
			else{
				double radius = -(((M_PI/2)/angleDifference)*(distance/30)); //minus is because negative values need to turn right and positive need to turn left
				movement_travel_in_curve_radius(speed, radius);
			}
		}		
	}
	return result;
}

bool rotate_to_angle(double targetAngle, double tolerance){
	//~ double tolerance = (2*M_PI)/144;//10 degrees
	double angleDifference = angle_difference(behavior_sensorReadings.positionSensor.t,targetAngle);
	bool result = (fabs(angleDifference) < tolerance) ? true : false;
	if(result==true){
		//~ movement_stop();
	}else{
		leds(15);
		//~ printf("rotate, angleDifference=%5.3f , targetAngle=%5.3f \n", angleDifference, targetAngle);
		bool direction = (angleDifference < 0) ? RIGHT : LEFT;
		
		int speed = (int)((fabs(angleDifference)/M_PI)*((double)SPEED_ROTATE));
		if(speed<SPEED_ROTATE_BEFORE_STOP){
			speed=SPEED_ROTATE_BEFORE_STOP;
		}else if(speed>SPEED_ROTATE){
			speed=SPEED_ROTATE;
		}	
		movement_rotate(speed, direction);
	}
	return result;
}

typedef struct{
	PositionXY destination;
	int state;
	int next_direction;
	Behavior_generic_vals generic;
	MapField *next_field;
} Behavior_labyrinth_explore_vals;
static Behavior_labyrinth_explore_vals beh_labyrinth_explore;

#define LAB_BEH_STATE_MOVE_TO_NEXT_POINT 	0
#define LAB_BEH_FIND_NEXT_UNDEF_FIELD 		1
#define LAB_BEH_MAP_FIELD_AND_DECIDE 		2
#define LAB_BEH_MAP_CORRECT_POSTION			3

bool test_labyrinth_explore(){
	return true;
}

/*
 * WARNING: function has multiple exit poits.
 * */
void labyrinth_explore(){
	//~ printf("LABYRINTH_EXPLORE STATE=%d \n", beh_labyrinth_explore.state);
	//debug
	//~ beh_labyrinth_explore.state = 0;
	leds(beh_labyrinth_explore.state+1);
	if(beh_labyrinth_explore.state == LAB_BEH_STATE_MOVE_TO_NEXT_POINT)
	{
		//~ printf("move to next point \n");
		//~ printf("move_to_point values: x=%f, y=%f, t=%f, destx=%f, desty=%f \n",behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, behavior_sensorReadings.positionSensor.t, (*beh_labyrinth_explore.next_field).position.x, (*beh_labyrinth_explore.next_field).position.y);
		double absolute_angle = angle_between_points(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, (*beh_labyrinth_explore.next_field).position.x,(*beh_labyrinth_explore.next_field).position.y);
		double angleDifference = angle_difference(behavior_sensorReadings.positionSensor.t,absolute_angle);
		double angle_tolerance = (2*M_PI)/36;//20 degrees taking in account left and right
		int closest_direction = get_closest_direction( behavior_sensorReadings.positionSensor.t , MAP_FIELD_NUM_CONNECTIONS);
		double distance = distance_between_points(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, (*beh_labyrinth_explore.next_field).position.x,(*beh_labyrinth_explore.next_field).position.y);
		
		double distance_tolerance = 20.0;//mm
		double wanted_measurement_point = DISTANCE_BETWEEN_POINTS*3/5;
		if(fabs(angleDifference) < angle_tolerance && fabs(wanted_measurement_point- distance) < distance_tolerance){
			//left neighbour
			int left_direction = (closest_direction+1)%MAP_FIELD_NUM_CONNECTIONS;		
			if((*beh_labyrinth_explore.next_field).connections[left_direction].state == MAP_STATE_UNDEFINED )
			{
				//~ int new_field_state = 
				double left_angle = left_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);
				PositionXY neighbour_position;
				new_points_from_distance_and_angle((*beh_labyrinth_explore.next_field).position.x, (*beh_labyrinth_explore.next_field).position.y, &neighbour_position.x, &neighbour_position.y, left_angle, DISTANCE_BETWEEN_POINTS);
				//~ if(behavior_sensorReadings.obstacleSensor.left >= DISTANCE_NEXT_FRONT_POINT_SIDES_FREE)
				//~ {
					//~ add_field(beh_labyrinth_explore.next_field, left_direction, MAP_STATE_FREE, MAP_STATE_FREE, neighbour_position);
					//~ printf("LEFT NIGHBOUR FREE\n");
				//~ }
				if(behavior_sensorReadings.obstacleSensor.left < DISTANCE_NEXT_FRONT_POINT_SIDES_OCCUPIED)
				{
					add_field(beh_labyrinth_explore.next_field, left_direction, MAP_STATE_OCCUPIED, MAP_STATE_OCCUPIED, neighbour_position);
					printf("LEFT NIGHBOUR OCCUPIED\n");
				}
			}
				
			int right_direction = (closest_direction-1)%MAP_FIELD_NUM_CONNECTIONS;
			if((*beh_labyrinth_explore.next_field).connections[right_direction].state == MAP_STATE_UNDEFINED)
			{
				double right_angle = right_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);
				PositionXY neighbour_position;
				new_points_from_distance_and_angle((*beh_labyrinth_explore.next_field).position.x, (*beh_labyrinth_explore.next_field).position.y, &neighbour_position.x, &neighbour_position.y, right_angle, DISTANCE_BETWEEN_POINTS);
				
				//~ if(behavior_sensorReadings.obstacleSensor.right >= DISTANCE_NEXT_FRONT_POINT_SIDES_FREE)
				//~ {
					//~ add_field(beh_labyrinth_explore.next_field, right_direction, MAP_STATE_FREE, MAP_STATE_FREE, neighbour_position);
					//~ printf("RIGHT NIGHBOUR FREE\n");
				//~ }	
				if(behavior_sensorReadings.obstacleSensor.right < DISTANCE_NEXT_FRONT_POINT_SIDES_OCCUPIED)
				{
					add_field(beh_labyrinth_explore.next_field, right_direction, MAP_STATE_OCCUPIED, MAP_STATE_OCCUPIED, neighbour_position);
					printf("RIGHT NIGHBOUR OCCUPIED\n");
				}
			}
		}		
		//move to next point
		if(move_to_point((*beh_labyrinth_explore.next_field).position.x,(*beh_labyrinth_explore.next_field).position.y)){
			movement_stop();
			beh_labyrinth_explore.state = LAB_BEH_FIND_NEXT_UNDEF_FIELD;
			set_CurrentField(beh_labyrinth_explore.next_field);
			//~ printf("FINISHED \n");
			signal_short_LED();
		}
	}
	else if(beh_labyrinth_explore.state == LAB_BEH_FIND_NEXT_UNDEF_FIELD)
	{
		movement_stop();
		printf("find next undefined state \n");
		//find next undefined state
		int next_field_state = MAP_STATE_UNDEFINED;
		int i = 0;
		
		int closest_direction = get_closest_direction( behavior_sensorReadings.positionSensor.t , MAP_FIELD_NUM_CONNECTIONS);
		
		for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)
		{
			int next_direction = (i+closest_direction)%MAP_FIELD_NUM_CONNECTIONS;
			//~ printf("for i = %d, next_direction = %d", i, next_direction);
			next_field_state = (*currentField).connections[next_direction].state;
			//~ next_field_state = (*currentField).connections[i];
			//~ printf("state = %d ", next_field_state);
			if(next_field_state == MAP_STATE_UNDEFINED){
				//~ printf("next_field_state is undefined \n");
				beh_labyrinth_explore.state = LAB_BEH_MAP_FIELD_AND_DECIDE;
				beh_labyrinth_explore.next_direction = next_direction;
				break;
			}
			else{
				//printf("next_field_state is undefined \n");
				//~ MapField tempMapField = *(*currentField).connections[i].field;
				//~ printf("\n !!!!!!!!!!!neighbour: %d, X=%5.3f, Y=%5.3f --  MY Position: X=%5.3f, Y=%5.3f --  Current: %d, X=%5.3f, Y=%5.3f\n", (*currentField).connections[i].field, tempMapField.position.x, tempMapField.position.y, behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, currentField, (*currentField).position.x, (*currentField).position.y );
			}
		}
		if(next_field_state != MAP_STATE_UNDEFINED){
			
			//calculate next path 
			printf("find next free destination \n");
			reset_labyrinth_explore();
			set_follow_path_to_unexplored_field();			
			
		}				
		//beh_labyrinth_explore.state = LAB_BEH_STATE_MOVE_TO_NEXT_POINT;
	}
	else if(beh_labyrinth_explore.state == LAB_BEH_MAP_FIELD_AND_DECIDE)
	{
		//rotate to next undefined neighbour
		double angle = beh_labyrinth_explore.next_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);
		//angle = angle_difference(behavior_sensorReadings.positionSensor.t, angle);
		if( rotate_to_angle(angle, (2*M_PI)/144) == true ){// 5 degrees
			
			
			movement_fullstop();
			behavior_sensorReadings = get_accurate_sensor_reading();
			
			printf("calculate position and distance \n");	
				
			//get more accurate reeadings to decide more accuratly		
			
			//calculate position and distance
			double distance = DISTANCE_BETWEEN_POINTS;
			PositionXY neighbour_position;
			new_points_from_distance_and_angle((*currentField).position.x, (*currentField).position.y, &neighbour_position.x, &neighbour_position.y, angle, distance);
			//new_points_from_distance_and_angle(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, &neighbour_position.x, &neighbour_position.y, behavior_sensorReadings.positionSensor.t, distance);
			//calculate state
			int neighbour_state = (
											behavior_sensorReadings.obstacleSensor.front >= DISTANCE_NEXT_POINT_FRONT
										//~ && behavior_sensorReadings.obstacleSensor.left >= DISTANCE_NEXT_POINT_SIDES
										//~ && behavior_sensorReadings.obstacleSensor.right >= DISTANCE_NEXT_POINT_SIDES
									) ? MAP_STATE_FREE : MAP_STATE_OCCUPIED;
			MapField *neighbour_field;
			
			//~ printf("BEHAVIOR: currentField=%p ,  startingField=%p\n", currentField, startingField);
			
			neighbour_field = add_field(currentField, beh_labyrinth_explore.next_direction, neighbour_state, neighbour_state, neighbour_position);
			if(neighbour_state == MAP_STATE_FREE && (*neighbour_field).num_visits == 0)
			{
				//~ printf("go to neighbour \n");
				//go to neighbour
				
				
				if(	behavior_sensorReadings.obstacleSensor.left <= DISTANCE_CLOSE_SIDES || behavior_sensorReadings.obstacleSensor.right <= DISTANCE_CLOSE_SIDES)
				{
					//wall is to close so add that as an occupied field int the next call and make the correction 
					int step = (behavior_sensorReadings.obstacleSensor.left > behavior_sensorReadings.obstacleSensor.right) ? 1:-1;
					beh_labyrinth_explore.next_direction = (beh_labyrinth_explore.next_direction + step)%MAP_FIELD_NUM_CONNECTIONS;
					printf("next step is to correct step= %d",step); 
				}
				else
				{
					//advance to the next state
					beh_labyrinth_explore.next_field = neighbour_field;
					beh_labyrinth_explore.state = LAB_BEH_STATE_MOVE_TO_NEXT_POINT;
				}			
			}
			else
			{
				//~ printf("field not free \n");
				//~ printf("f=%5.3f ,l=%5.3f ,r=%5.3f \n", behavior_sensorReadings.obstacleSensor.front, behavior_sensorReadings.obstacleSensor.left ,behavior_sensorReadings.obstacleSensor.right);
				if(check_current_position_offset()){
					beh_labyrinth_explore.state = LAB_BEH_FIND_NEXT_UNDEF_FIELD;	
				}				
					
				//~ printf("field not free \n");
				//~ //rotate to next neighbour
				//~ rotateRel_naive((2*M_PI)/MAP_FIELD_NUM_CONNECTIONS);
				//~ beh_labyrinth_explore.next_direction++;
				//~ beh_labyrinth_explore.next_direction = beh_labyrinth_explore.next_direction%MAP_FIELD_NUM_CONNECTIONS;
			}
		}
		else
		{
			//~ printf("rotating to next undefined neighbour destination=%d, angle=%f, t=%f\n",beh_labyrinth_explore.next_direction,angle,behavior_sensorReadings.positionSensor.t);
		}		
	}
	//~ printf("End of function \n");
	return;
}

void reset_labyrinth_explore(){
	beh_labyrinth_explore.next_direction = 0;
	beh_labyrinth_explore.generic.isActive = false;
	beh_labyrinth_explore.state = LAB_BEH_STATE_MOVE_TO_NEXT_POINT;
	beh_labyrinth_explore.next_field = currentField;
	//~ beh_labyrinth_explore.destination.x = 200.0;
	//~ beh_labyrinth_explore.destination.y = 200.0;
}

typedef struct{
	int current_field_index;
	int next_field_index;
	MapField *path[MAX_MAP_FIELDS];
	int end_field_index;
} Behavior_follow_path_vals;
static Behavior_follow_path_vals beh_follow_path;

void set_follow_path_to_unexplored_field(){
	printf("set_follow_path_to_unexplored_field\n");
	reset_follow_path();
	get_path_to_unexplored_field(beh_follow_path.path, &beh_follow_path.end_field_index);
}

void set_follow_path_to_return_home(){
	printf("set_follow_path_to_return_home\n");
	reset_follow_path();
	get_path_to_starting_field(beh_follow_path.path, &beh_follow_path.end_field_index);
}

void reset_follow_path(){
	printf("reset_follow_path\n");
	beh_follow_path.current_field_index = -1;
	beh_follow_path.next_field_index = beh_follow_path.current_field_index;
	*beh_follow_path.path = NULL;
	//test
	//~ get_path_to_unexplored_field(beh_follow_path.path, &beh_follow_path.end_field_index);
	//~ printf("end_index = %d\n",beh_follow_path.end_field_index);
	//~ int i = 0;
	//~ for(i=0; i<beh_follow_path.end_field_index; i++)
	//~ {
		//~ printf(" index(%d) = %p \n", i, beh_follow_path.path[i]);
	//~ }
}

bool test_follow_path(){
	//~ return false;
	bool result = false;
	//if path is set and the robot has not yet reached the destination return true
	//else return false	
	if(*beh_follow_path.path != NULL && beh_follow_path.current_field_index < beh_follow_path.end_field_index){
		result = true;
		//~ printf("test_follow_path - true\n");
	}else{
		result = false;
		//~ printf("test_follow_path - false\n");
	}
	
	return result;
}
void follow_path(){
	int step = 1;
	while(true){
		//~ printf("beh_follow_path.current_field_index %d and step  %d\n",beh_follow_path.current_field_index, step );
		if( (beh_follow_path.current_field_index >= 0 ) && (beh_follow_path.current_field_index+step) < beh_follow_path.end_field_index  && points_are_in_line((*beh_follow_path.path[beh_follow_path.current_field_index]).position, (*beh_follow_path.path[beh_follow_path.current_field_index+step]).position, (*beh_follow_path.path[beh_follow_path.current_field_index+step+1]).position))
		{
			step++;
			continue;
		}
		break;
	}	
	if(move_to_point((*beh_follow_path.path[beh_follow_path.current_field_index+step]).position.x ,  (*beh_follow_path.path[beh_follow_path.current_field_index+step]).position.y )){
		movement_stop();
		int closest_direction = get_closest_direction( behavior_sensorReadings.positionSensor.t , MAP_FIELD_NUM_CONNECTIONS);
		double angle = closest_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);	
		if( rotate_to_angle(angle, (2*M_PI)/144) == true ){
			beh_follow_path.current_field_index += step;// beh_follow_path.next_field_index;
			set_CurrentField(beh_follow_path.path[beh_follow_path.current_field_index]);
			check_current_position_offset();
			if(beh_follow_path.current_field_index >= beh_follow_path.end_field_index){
				reset_labyrinth_explore();
				reset_follow_path();
				signal_long_LED();		
			}	
		}		
	}	
}

Behavior_generic_vals beh_correct_position;

bool check_current_position_offset(){
	int closest_direction = get_closest_direction( behavior_sensorReadings.positionSensor.t , MAP_FIELD_NUM_CONNECTIONS);
	printf("check_current_position_offset direction = %d, STATE = %d\n",closest_direction, (*currentField).connections[closest_direction].state);
	bool result = true;
	double angle = closest_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);
	if((*currentField).connections[closest_direction].state == MAP_STATE_UNDEFINED )
	{
		if( rotate_to_angle(angle, (2*M_PI)/72) == true ){
			movement_fullstop();
			get_accurate_sensor_reading();
			double distance = DISTANCE_BETWEEN_POINTS;
			PositionXY neighbour_position;
			new_points_from_distance_and_angle((*currentField).position.x, (*currentField).position.y, &neighbour_position.x, &neighbour_position.y, angle, distance);
			int neighbour_state = (behavior_sensorReadings.obstacleSensor.front >= DISTANCE_NEXT_POINT_FRONT) ? MAP_STATE_FREE : MAP_STATE_OCCUPIED;
			MapField *neighbour_field;
			neighbour_field = add_field(currentField, closest_direction, neighbour_state, neighbour_state, neighbour_position);
		}
		else{
			result = false;
		}
	}
	if((*currentField).connections[closest_direction].state == MAP_STATE_OCCUPIED )
	{
		printf("front field is occupied \n");
		double angle = closest_direction*((2.0*M_PI)/(double)MAP_FIELD_NUM_CONNECTIONS);	
		
		double measurement_correction = (DISTANCE_FROM_CENTER_TO_WALL_FRONT - behavior_sensorReadings.obstacleSensor.front)*10;//cm into mm						
		if(fabs(measurement_correction) >= DISTANCE_FROM_CENTER_TO_WALL_TOLERANCE){
			reset_correct_position();
			double x = measurement_correction*cos(angle);
			double y = measurement_correction*sin(angle);
			double t = 0.0;	
			add_position_correction( x,  y,  t);
			printf("CURRENT CORRECTION: x=%5.3f, y=%5.3f, t=%5.3f \n", x, y, t);
			beh_correct_position.isActive = true;
		}
	}
	return result;
}

bool test_correct_position(){
	return beh_correct_position.isActive;
}
void correct_position(){
	//~ printf("MOVING TO CORRECT POSITION \n");
	if(move_to_point((*currentField).position.x,(*currentField).position.y)){
		movement_stop();
		beh_correct_position.isActive = false;
		//~ signal_short_LED();
	}
}
void reset_correct_position(){
	beh_correct_position.isActive = false;
	//~ beh_correct_position.direction = LEFT;
}


bool test_labyrinth_return_home(){
	return true;
}
void labyrinth_return_home(){
	set_follow_path_to_return_home();
	printf("LABYRINTH_RETURN_HOME \n");
}
void reset_labyrinth_return_home(){
	set_follow_path_to_return_home();
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
	double dz = distance_between_points(behavior_sensorReadings.positionSensor.x, behavior_sensorReadings.positionSensor.y, behavior_sensorReadings.startingPosition.x, behavior_sensorReadings.startingPosition.y);
	if(dz<=STARTING_POINT_BUFFER){
		return true;
	}	
	return false;
}
void stop_at_starting_point(){
	printf("..............STOP_AT_STARTING_POINT.........\n");
	/*victory dance*/
	movement_fullstop();	
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
		//~ printf("reset priority=%d ID=%d \n",priority, behavior_struct.priority_list[priority] );
	}
}

void set_return_priority_list()
{
	behavior_struct.priority_list[0] = STOP_AT_STARTING_POINT_ID;
	behavior_struct.priority_list[1] = AVOID_COLISION_ID;
	behavior_struct.priority_list[2] = CORRECT_POSITION_ID;
	behavior_struct.priority_list[3] = FOLLOW_PATH_ID;
	behavior_struct.priority_list[4] = LABYRINTH_RETURN_HOME_ID;
	
	reset_stop_at_starting_point();	
	reset_avoid_colision();
	//~ reset_follow_starting_point();
	//~ reset_follow_wall();
	reset_wounder();
	reset_correct_position();
	reset_follow_path();
	reset_labyrinth_explore();
}

void behaviors_init(){
	SensorReadings tempSensorReadings = get_filteredSensorReadings();
	PositionXY position;
	position.x = tempSensorReadings.startingPosition.x;
	position.y = tempSensorReadings.startingPosition.y;
	//~ position.x = 0.0;
	//~ position.y = 0.0;
	map_init(position);
	
	movement_fullstop();
	behaviors_state = STATE_SEARCHING_FOR_BEACON_AREA;
	behavior_struct.default_priority_list[0] = STOP_AT_BEACON_ID;
	behavior_struct.default_priority_list[1] = AVOID_COLISION_ID;
	behavior_struct.default_priority_list[2] = CORRECT_POSITION_ID;
	behavior_struct.default_priority_list[3] = FOLLOW_PATH_ID;
	behavior_struct.default_priority_list[4] = LABYRINTH_EXPLORE_ID;
	
	reset_default_priority_list();
	reset_correct_position();
	reset_follow_path();
	reset_labyrinth_explore();
	reset_avoid_colision();
	//~ reset_follow_wall();
	//~ reset_follow_beacon();
	reset_stop_at_beacon();
	reset_wounder();	
}
void behaviors_finish(){
	movement_fullstop();
};

void signal_long_LED(){
	movement_fullstop();
	leds(15);
	wait(2);
	leds(0);
	wait(2);
	leds(15);
	wait(2);
	leds(0);
}
void signal_short_LED(){
	movement_fullstop();
	leds(15);
	wait(2);
	leds(0);
}
void signal_ERROR(){
	printf("WARNING: An ERROR occured!! \n");	
	movement_fullstop();
	while(1){
		leds(9);
		wait(1);
		leds(6);
		wait(1);	
	}	
}




/*
 * ########################################################################################################################################################################
 * ########################################################################################################################################################################
 * ########################################################################################################################################################################
 * ##################################--------DEPRICATED PART - old code, old behaviors--------#############################################################################
 * ########################################################################################################################################################################
 * ########################################################################################################################################################################
 * ########################################################################################################################################################################
 */

bool test_follow_wall();
void follow_wall();
void reset_follow_wall();

bool test_follow_beacon();
void follow_beacon();
void reset_follow_beacon();


static int follow_starting_point_last_point;
static double next_point_rel_dir = 0.0;
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
//~ 
//~ 


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
static Behavior_follow_wall_vals beh_fol_wall;
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


/**
 * DEPRICATED FUNCTION - new one replaced this one for the purpouse of the PathFinder Final Project for the "Inteligent and mobile robotics" course 2014./2015. 
 * */

//~ int execute_behavior(int behavior_control)
//~ {
	//~ bool can_execute_list[NUMBER_OF_BEHAVIORS];
	//~ 
	//~ behavior_sensorReadings = get_new_sensorReadings(behaviors_state);;
	//~ 
	//~ can_execute_list[STOP_AT_BEACON_ID] = false;
	//~ can_execute_list[FOLLOW_BEACON_ID] = false;
	//~ can_execute_list[FOLLOW_WALL_ID] = false;
	//~ 
	//~ //for demonstration
	//~ //for changing behaviors from all to avoid colisio, to follow wall and to follow and stop at beacon
	//~ if(behavior_control == 1)
	//~ {
		//~ //only avoid collision behavior
		//~ behaviors_state = STATE_RETURNING_HOME;
	//~ }
	//~ else if(behavior_control == 2)
	//~ {
		//~ //only follow wall behavior
		//~ can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
		//~ behaviors_state = STATE_RETURNING_HOME;
	//~ }
	//~ else if(behavior_control == 3)
	//~ {
		//~ //only stop at beacon and stop at beacon
		//~ can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
		//~ can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
		//~ behaviors_state = STATE_SEARCHING_FOR_BEACON;
	//~ }
	//~ else
	//~ {
		//~ //all behaviours
		//~ can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
		//~ can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
		//~ can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
	//~ }
	//~ 
	//~ if(behaviors_state == STATE_RETURNING_HOME)
	//~ {
		//~ can_execute_list[STOP_AT_BEACON_ID] = false;
		//~ can_execute_list[FOLLOW_BEACON_ID] = false;
	//~ }
	//~ 
	//~ //can_execute_list[STOP_AT_BEACON_ID] = test_stop_at_beacon();
	//~ can_execute_list[AVOID_COLISION_ID] = test_avoid_colision();
	//~ //can_execute_list[FOLLOW_BEACON_ID] = test_follow_beacon();
	//~ //can_execute_list[FOLLOW_WALL_ID] = test_follow_wall();
	//~ can_execute_list[WOUNDER_ID] = test_wounder();
	//~ can_execute_list[STOP_AT_STARTING_POINT_ID] = test_stop_at_starting_point();
	//~ can_execute_list[FOLLOW_STARTING_POINT_ID] = test_follow_starting_point();
	//~ 
	//~ int follow_wall_priority = get_priority_from_id(FOLLOW_WALL_ID);
	//~ //check if there is a follow wall and follow beacon can execute and if the wall is between the robot and the beacon
	//~ 
	//~ if(behaviors_state == STATE_RETURNING_HOME)
	//~ {
		//~ can_execute_list[STOP_AT_BEACON_ID] = false;
		//~ can_execute_list[FOLLOW_BEACON_ID] = false;
	//~ }
	//~ else
	//~ {
		//~ can_execute_list[STOP_AT_STARTING_POINT_ID] = false;
		//~ can_execute_list[FOLLOW_STARTING_POINT_ID] = false;
	//~ }
	//~ 
	//~ /**
	 //~ * changing the priority list of the behaviors
	 //~ * */
	//~ 
	//~ /*
	//~ if(can_execute_list[FOLLOW_WALL_ID] == true && can_execute_list[AVOID_COLISION_ID] == true && follow_wall_priority!= HIGH_PRIORITY
		//~ && next_point_rel_dir!=0.0){		
		//~ change_behavior_priority(FOLLOW_WALL_ID, HIGH_PRIORITY);//promote behavior
		//~ behavior_struct.follow_wall_starting_point.x = behavior_sensorReadings.positionSensor.x;
		//~ behavior_struct.follow_wall_starting_point.y = behavior_sensorReadings.positionSensor.y;
		//~ behavior_struct.follow_wall_starting_point.t = next_point_rel_dir;
		//~ printf("CHANGE_PRIORITY start_dir=%f\n",behavior_struct.follow_wall_starting_point.t);
		//~ signal_short_LED();
	//~ }else*/
	//~ if(can_execute_list[FOLLOW_WALL_ID] == true && can_execute_list[FOLLOW_BEACON_ID] == true && follow_wall_priority!= HIGH_PRIORITY
		//~ &&(	(behavior_sensorReadings.obstacleSensor.front >= DISTANCE_NEAR_FRONT && fabs(behavior_sensorReadings.beaconSensor.relative_direction) <= (M_PI/12))//from-30 to 30
			//~ || (behavior_sensorReadings.obstacleSensor.right >= DISTANCE_NEAR_SIDES && behavior_sensorReadings.beaconSensor.relative_direction >= (M_PI/4))//>=30
			//~ || (behavior_sensorReadings.obstacleSensor.left  >= DISTANCE_NEAR_SIDES && behavior_sensorReadings.beaconSensor.relative_direction <= -(M_PI/4))//<=-30
			//~ )
		//~ ){
		//~ change_behavior_priority(FOLLOW_WALL_ID, HIGH_PRIORITY);//promote behavior
		//~ behavior_struct.follow_wall_starting_point.x = behavior_sensorReadings.positionSensor.x;
		//~ behavior_struct.follow_wall_starting_point.y = behavior_sensorReadings.positionSensor.y;
		//~ behavior_struct.follow_wall_starting_point.t = behavior_sensorReadings.beaconSensor.apsolute_direction;
		//~ printf("CHANGE_PRIORITY start_dir=%f\n",behavior_struct.follow_wall_starting_point.t);		
		//~ signal_short_LED();	
		//~ 
	//~ }else if(follow_wall_priority == HIGH_PRIORITY 
		//~ //&& 
		//~ //(behavior_struct.follow_wall_starting_point.t < (aprox_angle * 1.2) 
		//~ //&& behavior_struct.follow_wall_starting_point.t > (aprox_angle * 0.8))
		//~ ){
		//~ double dx = (behavior_sensorReadings.positionSensor.x - behavior_struct.follow_wall_starting_point.x);
		//~ double dy = (behavior_sensorReadings.positionSensor.y - behavior_struct.follow_wall_starting_point.y);
		//~ double dz = sqrt(dx*dx+dy*dy);
		//~ double aprox_angle = atan2(dy,dx);
		//~ //double angle = behavior_struct.follow_wall_starting_point.t;
		//~ //double start_point_angle = (behavior_struct.follow_wall_starting_point.t<0) ? (behavior_struct.follow_wall_starting_point.t + 2*M_PI) : behavior_struct.follow_wall_starting_point.t;
		//~ //aprox_angle = (aprox_angle<0) ? (aprox_angle + 2*M_PI) : aprox_angle;
		//~ 
		//~ aprox_angle = behavior_struct.follow_wall_starting_point.t- aprox_angle;
		//~ while(aprox_angle<-M_PI)
			//~ aprox_angle += 2*M_PI;
		//~ while(aprox_angle>M_PI)
			//~ aprox_angle -= 2*M_PI;
		//~ printf("start_point_angle=%f, aprox_angle=%f  , dz=%f \n",behavior_struct.follow_wall_starting_point.t, aprox_angle, dz);		
		//~ //signal_long_LED();
		//~ 
		//~ //if(start_point_angle < (aprox_angle * 1.2) 
		//~ //&& start_point_angle > (aprox_angle * 0.8))
		//~ if(fabs(aprox_angle) <= (M_PI/12) && dz>=DISTANCE_XY_CLOSE){
			//~ printf("..................RETURN_PRIORITY..................\n");
			//~ signal_short_LED();			
			//~ change_behavior_priority(FOLLOW_WALL_ID, FOLLOW_WALL_PRIORITY);//return normal priority
			//~ reset_follow_wall();
			//~ if(behavior_sensorReadings.beaconSensor.isVisible == true){
				//~ rotateRel_naive(behavior_sensorReadings.beaconSensor.relative_direction);
			//~ }else{
				//~ aprox_angle = behavior_struct.follow_wall_starting_point.t - behavior_sensorReadings.positionSensor.t;
				//~ rotateRel_naive(aprox_angle);
			//~ }			
		//~ }else{
			//~ printf("..................CONINUE..................\n");
			//~ //continue and wait until something changes eg. when follow wall finishes
		//~ }
	//~ }else{
		//~ //continue and wait until something changes eg. when follow wall finishes
	//~ }
	//~ int priority;
	//~ for(priority=0;priority<NUMBER_OF_BEHAVIORS;priority++){
		//~ if(can_execute_list[behavior_struct.priority_list[priority]] == true){
			//~ leds(behavior_struct.priority_list[priority]);
			//~ switch ( behavior_struct.priority_list[priority] ) {				
				//~ case STOP_AT_BEACON_ID:
					//~ stop_at_beacon();
					//~ //if stopped at beacon  exchange this behavior for stop at starting point
					//~ behavior_struct.at_beacon_area_counter++;
					//~ if(behavior_struct.at_beacon_area_counter>10)
					//~ {
						//~ /**
						 //~ * preparing for returning home
						 //~ * */
						//~ //signal that beacon 
						//~ signal_long_LED();		
						//~ set_return_priority_list();
						//~ behaviors_state = STATE_RETURNING_HOME;
					//~ }					
					//~ break;
				//~ case STOP_AT_STARTING_POINT_ID:
					//~ stop_at_starting_point();
					//~ signal_long_LED();
					//~ return 1;
					//~ break;
				//~ case AVOID_COLISION_ID:
					//~ avoid_colision();
					//~ break;
				//~ case FOLLOW_BEACON_ID:
					//~ follow_beacon();
					//~ reset_follow_wall();
					//~ break;
				//~ case FOLLOW_STARTING_POINT_ID:
					//~ follow_starting_point();
					//~ reset_follow_wall();
					//~ break;
				//~ case FOLLOW_WALL_ID:
					//~ follow_wall();				
					//~ break;
				//~ case WOUNDER_ID:
					//~ wounder();
					//~ break;
				//~ default:
					//~ wounder();
					//~ break;
			//~ }
			//~ break;
		//~ }
	//~ }
	//~ return 0;
//~ }
//~ 




















