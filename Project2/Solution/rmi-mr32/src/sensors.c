#include <math.h>
#include "config_constants.h"
#include "sensors.h"
#include "helperFunctions.h"
#include "rmi_compass.h"
#include <detpic32.h>

/** \brief returns the value of the beacon snsor and also controlls te servo that is responsible for directing (the behaviour) the beacon sensor
 * */
void readBeaconSensor();
void readGroundSensor();
void readCompassSensor();
void readObstacleSensors();
void readOtherSensors();
void readPositionSensors();

/** \brief saves the current sensor readings to a buffer array of readings
 * */
void save_reading_to_buffer();
/** \brief calculates values that may be usfull to the robot agent taking in account other sensor values 
 * */
void calculate_extra_values();
/** \brief maintains a field of last known positions of the robot
 * */
void save_breadcrumbs();
/** \brief returns normalized aproxiate value of the obstacle sensors in cm
 * */
double normalize_obstacle_sensor(int sensor_reading);
/** \brief refreshes the internal sensor readings and stores it internally
 * */
void refresh_sensorReadings(int state);

SensorReadings sensor_sensorReadings;


typedef struct{
	ObstacleSensor obstacleSensor; ///< analog sensors structure
	BeaconSensor beaconSensor;
	PositionSensor positionSensor;
	int groundSensor;
} SensorBufferReadings;

SensorBufferReadings sensor_filteredSensorReadings;

#define READINGS_BUFFER_SIZE	10

typedef struct{
	bool analog_sensors_updated;
	int num_readings;
	SensorBufferReadings sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE];
} SensorInternalValues;
SensorInternalValues internal_values;

void sensors_init(){
	enableObstSens();
	enableGroundSens();
	initCompass();
	sensor_sensorReadings.last_position_index = -1;	
	internal_values.analog_sensors_updated = false;
	internal_values.num_readings = 0;
}
void sensors_finish(){
	disableObstSens();
	disableGroundSens();
}

void refresh_sensorReadings(int state){	
	readPositionSensors();
	readCompassSensor();
	readObstacleSensors();
	readOtherSensors();
	
	if(state == STATE_SEARCHING_FOR_BEACON){
		readBeaconSensor();
		save_breadcrumbs();
		readGroundSensor();
		readCompassSensor();
	}else if(state == STATE_SEARCHING_FOR_BEACON_AREA){
		save_breadcrumbs();
		readGroundSensor();
		readCompassSensor();
	}
	
	calculate_extra_values();
	
	save_reading_to_buffer();
	
	
	if(internal_values.num_readings < 1000){
		internal_values.num_readings++;
	}
	
	//double compass = readCompassSensor();
	//printf("Obst_front=%5.3f, Obst_left=%5.3f, Obst_right=%5.3f compass=%5.3f\n", front, left, right, compass);
	
	printf("x=%5.3f, y=%5.3f, \n", sensor_sensorReadings.positionSensor.x, sensor_sensorReadings.positionSensor.y);
	//~ 
	//~ printf("Obst_left=%03d, Obst_front=%03d, Obst_right=%03d, Bat_voltage=%03d, Ground_sens=%d, Beacon_visible=%d, Ground_sensors=", 
	//~ sensor_sensorReadings.obstacleSensor.left, sensor_sensorReadings.obstacleSensor.front, sensor_sensorReadings.obstacleSensor.right, sensor_sensorReadings.batteryVoltage, sensor_sensorReadings.groundSensor,
					//~ sensor_sensorReadings.beaconSensor.isVisible);
	//~ printInt(sensor_sensorReadings.groundSensor, 2 | 5 << 16);	// System call
	//~ printf("\n");	
	//~ 
	//~ printf("Beacon_visible=%d, Beacon_direction=%03d", sensor_sensorReadings.beaconSensor.isVisible, sensor_sensorReadings.beaconSensor.direction);
	//~ printf("\n");
}

void readGroundSensor(){
	sensor_sensorReadings.groundSensor = readLineSensors(0);
	sensor_sensorReadings.atBeaconArea = (sensor_sensorReadings.groundSensor > 0);
}

void check_analog_sensors(){
	if(internal_values.analog_sensors_updated == NO){
		readAnalogSensors();// Fill in "analogSensors" structure
		internal_values.analog_sensors_updated = YES;
	}
}

void readPositionSensors(){
	getRobotPos(&sensor_sensorReadings.positionSensor.x, &sensor_sensorReadings.positionSensor.y, &sensor_sensorReadings.positionSensor.t);  
}

void readObstacleSensors(){
	check_analog_sensors();
	sensor_sensorReadings.obstacleSensor.front = normalize_obstacle_sensor(analogSensors.obstSensFront);
	sensor_sensorReadings.obstacleSensor.left = normalize_obstacle_sensor(analogSensors.obstSensLeft);
	sensor_sensorReadings.obstacleSensor.right = normalize_obstacle_sensor(analogSensors.obstSensRight);
}

void readOtherSensors(){
	check_analog_sensors();
	sensor_sensorReadings.batteryVoltage = analogSensors.batteryVoltage;
}

void readCompassSensor(){
	check_analog_sensors();
	int compass_reading = getCompassValue();
	sensor_sensorReadings.positionSensor.compass_direction = deg_to_rad((double)compass_reading);	
}

double normalize_obstacle_sensor(int sensor_reading){
	double reading = (double) sensor_reading - 80;
	if(reading<0){
		reading = 1;
	}
	double result_value = 6200/reading;
	return result_value;
}

#define BEACON_SENSOR_SERVO_LIMIT 15
#define BEACON_SENSOR_SERVO_BUFFER 3

int beacon_servo_pos = 0;
int beacon_servo_dir = LEFT;
int beacon_servo_counter = 0;
int beacon_servo_sensor_limit_left = -BEACON_SENSOR_SERVO_LIMIT;
int beacon_servo_sensor_limit_right = BEACON_SENSOR_SERVO_LIMIT;

void readBeaconSensor(){
	if(readBeaconSens()){
		sensor_sensorReadings.beaconSensor.isVisible = true;
		sensor_sensorReadings.beaconSensor.relative_direction = (beacon_servo_pos/15.0*M_PI/2);//normalizing angle in radians
		beacon_servo_counter = 0;
		beacon_servo_sensor_limit_left = ((beacon_servo_pos - BEACON_SENSOR_SERVO_BUFFER) >= -BEACON_SENSOR_SERVO_LIMIT) ? (beacon_servo_pos - BEACON_SENSOR_SERVO_BUFFER) : -BEACON_SENSOR_SERVO_LIMIT;
		beacon_servo_sensor_limit_right = ((beacon_servo_pos + BEACON_SENSOR_SERVO_BUFFER) <= BEACON_SENSOR_SERVO_LIMIT) ? (beacon_servo_pos + BEACON_SENSOR_SERVO_BUFFER) :  BEACON_SENSOR_SERVO_LIMIT;
	}
	if(sensor_sensorReadings.beaconSensor.isVisible == true && beacon_servo_counter > (BEACON_SENSOR_SERVO_BUFFER*3)){
		beacon_servo_sensor_limit_left = -BEACON_SENSOR_SERVO_LIMIT;
		beacon_servo_sensor_limit_right = BEACON_SENSOR_SERVO_LIMIT;
		sensor_sensorReadings.beaconSensor.isVisible = false;
		beacon_servo_counter = 0;
	}else{
		beacon_servo_counter++;
	}
	if(beacon_servo_pos <= beacon_servo_sensor_limit_left || beacon_servo_pos >= beacon_servo_sensor_limit_right){
		beacon_servo_dir = !beacon_servo_dir;//change direction
	}
	if(beacon_servo_dir == LEFT){
		beacon_servo_pos-=2;
	}else{
		beacon_servo_pos+=2;
	}	
	setServoPos(beacon_servo_pos);
}



void calculate_extra_values(){
	if(sensor_sensorReadings.startingPosition.x == 0.0 && sensor_sensorReadings.startingPosition.y == 0.0 && sensor_sensorReadings.startingPosition.t == 0.0)
	{
		sensor_sensorReadings.startingPosition.x =  sensor_sensorReadings.positionSensor.x;
		sensor_sensorReadings.startingPosition.y =  sensor_sensorReadings.positionSensor.y;
		sensor_sensorReadings.startingPosition.t =  sensor_sensorReadings.positionSensor.t;
	}
	//~ sensor_sensorReadings.beaconSensor.apsolute_direction = (sensor_sensorReadings.beaconSensor.relative_direction + sensor_sensorReadings.positionSensor.t);
	//~ //printf("beaconSensor.apsolute_direction=%f\n",sensor_sensorReadings.beaconSensor.apsolute_direction);
	double dx = (sensor_sensorReadings.startingPosition.x - sensor_sensorReadings.positionSensor.x);
	double dy = (sensor_sensorReadings.startingPosition.y - sensor_sensorReadings.positionSensor.y);
	double angle = atan2(dy, dx);
	sensor_sensorReadings.startingPosition.relative_direction =  angle_difference( angle , sensor_sensorReadings.positionSensor.t);
}

int save_breadcrumbs_counter = 0;
void save_breadcrumbs()
{
	if(save_breadcrumbs_counter%10 == 0)
	{
		sensor_sensorReadings.last_position_index++;
		if(sensor_sensorReadings.last_position_index>=MAX_NUM_POSITIONS)
			sensor_sensorReadings.last_position_index=MAX_NUM_POSITIONS-1;
		if(sensor_sensorReadings.last_position_index<0)
			sensor_sensorReadings.last_position_index=0;
		save_breadcrumbs_counter = save_breadcrumbs_counter%10;
		sensor_sensorReadings.positionsHistory[sensor_sensorReadings.last_position_index].x = sensor_sensorReadings.positionSensor.x;
		sensor_sensorReadings.positionsHistory[sensor_sensorReadings.last_position_index].y = sensor_sensorReadings.positionSensor.y;
		sensor_sensorReadings.positionsHistory[sensor_sensorReadings.last_position_index].t = sensor_sensorReadings.positionSensor.t;
		//printf("last_position_index=%d \n", sensor_sensorReadings.last_position_index);
	}		
	save_breadcrumbs_counter++;
}

void save_reading_to_buffer(){
	
	if(internal_values.num_readings == 0)
	{
		for(i = READINGS_BUFFER_SIZE-1;i>0;i--){
			internal_values.sensor_sensorReadings_buffer[i] = sensor_sensorReadings;
		}
	}
	
	int i;
	for(i = READINGS_BUFFER_SIZE-1;i>0;i--){
		internal_values.sensor_sensorReadings_buffer[i-1] = internal_values.sensor_sensorReadings_buffer[i];
	}
	internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1] = sensor_sensorReadings;
	//~ internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].obstacleSensor = sensor_sensorReadings.obstacleSensor;
	//~ internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].beaconSensor = sensor_sensorReadings.beaconSensor;
	//~ internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].positionSensor = sensor_sensorReadings.positionSensor;
	//~ internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].groundSensor = sensor_sensorReadings.groundSensor;
}

void calculate_filteredSensorReadings(){
	int i;
	sensor_filteredSensorReadings.obstacleSensor.front = 0;
	sensor_filteredSensorReadings.obstacleSensor.left = 0;
	sensor_filteredSensorReadings.obstacleSensor.right = 0;
	sensor_filteredSensorReadings.positionSensor.x = 0;
	sensor_filteredSensorReadings.positionSensor.y = 0;
	sensor_filteredSensorReadings.positionSensor.t = 0;
	sensor_filteredSensorReadings.positionSensor.compass_direction = 0;
	sensor_filteredSensorReadings.groundSensor = 0;
	int sum_k = 0;
	for(i = READINGS_BUFFER_SIZE-1;i>=0;i--){
		int k = (i+1)/2;
		k = (k==0) ? 1 : k;
		sum_k += k;
		sensor_filteredSensorReadings.obstacleSensor.front += (double)k * internal_values.sensor_sensorReadings_buffer[i].obstacleSensor.front;
		sensor_filteredSensorReadings.obstacleSensor.left += (double)k * internal_values.sensor_sensorReadings_buffer[i].obstacleSensor.left;
		sensor_filteredSensorReadings.obstacleSensor.right += (double)k * internal_values.sensor_sensorReadings_buffer[i].obstacleSensor.right;
		sensor_filteredSensorReadings.positionSensor.x += (double)k * internal_values.sensor_sensorReadings_buffer[i].positionSensor.x;
		sensor_filteredSensorReadings.positionSensor.y += (double)k * internal_values.sensor_sensorReadings_buffer[i].positionSensor.y;
		sensor_filteredSensorReadings.positionSensor.t += (double)k * internal_values.sensor_sensorReadings_buffer[i].positionSensor.compass_direction;
		sensor_filteredSensorReadings.positionSensor.compass_direction += (double)k * internal_values.sensor_sensorReadings_buffer[i].obstacleSensor.front;
		sensor_filteredSensorReadings.groundSensor += k * internal_values.sensor_sensorReadings_buffer[i].groundSensor;
	}
	sensor_filteredSensorReadings.obstacleSensor.front = sensor_filteredSensorReadings.obstacleSensor.front / (double)sum_k;
	sensor_filteredSensorReadings.obstacleSensor.left = sensor_filteredSensorReadings.obstacleSensor.left / (double)sum_k;
	sensor_filteredSensorReadings.obstacleSensor.right = sensor_filteredSensorReadings.obstacleSensor.right / (double)sum_k;
	sensor_filteredSensorReadings.positionSensor.x = sensor_filteredSensorReadings.positionSensor.x / (double)sum_k;
	sensor_filteredSensorReadings.positionSensor.y = sensor_filteredSensorReadings.positionSensor.y / (double)sum_k;
	sensor_filteredSensorReadings.positionSensor.t = sensor_filteredSensorReadings.positionSensor.t / (double)sum_k;
	sensor_filteredSensorReadings.positionSensor.compass_direction = sensor_filteredSensorReadings.positionSensor.compass_direction / (double)sum_k;
	sensor_filteredSensorReadings.groundSensor = sensor_filteredSensorReadings.groundSensor / sum_k;
}

SensorReadings get_sensorReadings(){
	return sensor_sensorReadings;
}
SensorReadings get_new_sensorReadings(int state){
	refresh_sensorReadings(state);
	return sensor_sensorReadings;
}

SensorBufferReadings get_new_filteredSensorReadings(int state){
	refresh_sensorReadings(state);
	calculate_filteredSensorReadings();
	return sensor_filteredSensorReadings;
}

