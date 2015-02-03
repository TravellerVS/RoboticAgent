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

/** \brief calculates the avrage value from the buffer of sensor readings and sotres them in the sensor_filteredSensorReadings
 * */
void calculate_filteredSensorReadings();

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



void refresh_buffer();


SensorReadings sensor_sensorReadings;

SensorBufferReadings sensor_filteredSensorReadings;

#define READINGS_BUFFER_SIZE	10

typedef struct{
	bool analog_sensors_updated;
	int num_readings;
	double compas_offset;
	SensorBufferReadings sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE];
} SensorInternalValues;
SensorInternalValues internal_values;

void sensors_init(){
	enableObstSens();
	enableGroundSens();
	initCompass();
	sensor_sensorReadings.last_position_index = -1;	
	internal_values.analog_sensors_updated = NO;
	internal_values.num_readings = 0;
	internal_values.compas_offset = 0.0;
	refresh_buffer();
	refresh_buffer();
	refresh_buffer();
	refresh_buffer();
	internal_values.compas_offset = sensor_filteredSensorReadings.positionSensor.compass_direction;
	refresh_buffer();
}

SensorReadings get_accurate_sensor_reading(){
	refresh_buffer();
	return get_filteredSensorReadings();
}

void refresh_buffer(){
	int i;
	for(i=0; i<READINGS_BUFFER_SIZE;i++){
		waitTick40ms();
		refresh_sensorReadings(STATE_DEFAULT);
	}	
}

void sensors_finish(){
	disableObstSens();
	disableGroundSens();
}

void refresh_sensorReadings(int state){
	internal_values.analog_sensors_updated = NO;
	readPositionSensors();
	readCompassSensor();
	readObstacleSensors();
	readOtherSensors();
	
	if(state == STATE_SEARCHING_FOR_BEACON){
		readBeaconSensor();
		//save_breadcrumbs();
		readGroundSensor();
		readCompassSensor();
	}else if(state == STATE_SEARCHING_FOR_BEACON_AREA){
		//save_breadcrumbs();
		readGroundSensor();
		readCompassSensor();
	}
	else if(state == STATE_DEFAULT)
	{
		readGroundSensor();
		readCompassSensor();
	}
	
	calculate_extra_values();
	
	save_reading_to_buffer();
	
	if(internal_values.num_readings < 1000){
		internal_values.num_readings++;
	}
	
	//print the current readings
	//double compass = readCompassSensor();
	
	calculate_filteredSensorReadings();
	//~ printf("\n");
	//~ printf("  Position: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_sensorReadings.positionSensor.x        , sensor_sensorReadings.positionSensor.y        , sensor_sensorReadings.positionSensor.t        );
	printf("F_Position: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_filteredSensorReadings.positionSensor.x, sensor_filteredSensorReadings.positionSensor.y, sensor_filteredSensorReadings.positionSensor.t);
	//~ printf("  ObstSensors : Obst_front=%5.3f, Obst_left=%5.3f, Obst_right=%5.3f \n", sensor_sensorReadings.obstacleSensor.front, sensor_sensorReadings.obstacleSensor.left, sensor_sensorReadings.obstacleSensor.right);
	printf("F_ObstSensors: front=%5.3f, left=%5.3f, right=%5.3f, \n", sensor_filteredSensorReadings.obstacleSensor.front, sensor_filteredSensorReadings.obstacleSensor.left, sensor_filteredSensorReadings.obstacleSensor.right);
	
	//~ printf("Obst_left=%03f, Obst_front=%03f, Obst_right=%03f, Bat_voltage=%03d, Ground_sens=%d, Beacon_visible=%d, Ground_sensors=", 
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
	double front_reading = normalize_obstacle_sensor(analogSensors.obstSensFront);
	double left_reading = normalize_obstacle_sensor(analogSensors.obstSensLeft);
	double right_reading = normalize_obstacle_sensor(analogSensors.obstSensRight);
	
	//if readings are inconclusive the reading should is not taken into cosideration and is assigned the value of the last filtered value
	sensor_sensorReadings.obstacleSensor.front = ( front_reading > 0 ) ?  front_reading : sensor_filteredSensorReadings.obstacleSensor.front;
	sensor_sensorReadings.obstacleSensor.left = ( left_reading > 0 ) ?  left_reading : sensor_filteredSensorReadings.obstacleSensor.left;
	sensor_sensorReadings.obstacleSensor.right = ( right_reading > 0 ) ?  right_reading : sensor_filteredSensorReadings.obstacleSensor.right;
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
	double constant = 6200.0;
	double result_value = 0.0;
	double result_maximum = 70.0;
	double result_minimum = 0.0;
	if(reading<0){
		//strange reading
		//usually occurs when obstacle is very far away.
		result_value = result_maximum;
	}else{
		result_value = constant/reading;
	}
	//set upper limit because of unreliable readings
	if(result_value>result_maximum){		
		result_value = result_maximum;
	}
	//set lower limit because of unreliable readings
	if(result_value<result_minimum){		
		result_value = result_minimum;
	}
	//~ printf("s_reading %d", sensor_reading);
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
	int i;
	//if bufer is empty
	if(internal_values.num_readings == 0)
	{
		//fill the buffer with the same values, values read from the last reading
		for(i = READINGS_BUFFER_SIZE-1;i>0;i--){
			//~ internal_values.sensor_sensorReadings_buffer[i] = sensor_sensorReadings;
			internal_values.sensor_sensorReadings_buffer[i].obstacleSensor = sensor_sensorReadings.obstacleSensor;
			internal_values.sensor_sensorReadings_buffer[i].beaconSensor = sensor_sensorReadings.beaconSensor;
			internal_values.sensor_sensorReadings_buffer[i].positionSensor = sensor_sensorReadings.positionSensor;
			internal_values.sensor_sensorReadings_buffer[i].groundSensor = sensor_sensorReadings.groundSensor;
		}
	}
	//move over all the readings by one position
	for(i = READINGS_BUFFER_SIZE-1;i>0;i--){
		internal_values.sensor_sensorReadings_buffer[i-1] = internal_values.sensor_sensorReadings_buffer[i];
	}
	//put the last reading on the last position
	//~ internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1] = sensor_sensorReadings;
	internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].obstacleSensor = sensor_sensorReadings.obstacleSensor;
	internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].beaconSensor = sensor_sensorReadings.beaconSensor;
	internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].positionSensor = sensor_sensorReadings.positionSensor;
	internal_values.sensor_sensorReadings_buffer[READINGS_BUFFER_SIZE-1].groundSensor = sensor_sensorReadings.groundSensor;
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
		
		double val_t = internal_values.sensor_sensorReadings_buffer[i].positionSensor.t;
		normalize_angle_to_positive(&val_t);
		sensor_filteredSensorReadings.positionSensor.t += (double)k * val_t;
		
		double val_compass_direction = internal_values.sensor_sensorReadings_buffer[i].positionSensor.compass_direction;
		normalize_angle_to_positive(&val_compass_direction);
		sensor_filteredSensorReadings.positionSensor.compass_direction += (double)k * val_compass_direction;
		
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

SensorReadings get_filteredSensorReadings(){
	//~ printf("filteredSensorReadings: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_filteredSensorReadings.positionSensor.x, sensor_filteredSensorReadings.positionSensor.y, sensor_filteredSensorReadings.positionSensor.t);
	//copy data from sensor readings
	SensorReadings readings;
	
	readings.startingPosition = sensor_sensorReadings.startingPosition;
	readings.atBeaconArea = sensor_sensorReadings.atBeaconArea;
	readings.batteryVoltage = sensor_sensorReadings.batteryVoltage;
	
	int i = 0;
	for(i=0; (i<readings.last_position_index) && (i < MAX_NUM_POSITIONS);i++){
		readings.positionsHistory[i] = sensor_sensorReadings.positionsHistory[i];
	}
	
	readings.last_position_index = sensor_sensorReadings.last_position_index;
	
	//~ printf("filteredSensorReadings: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_filteredSensorReadings.positionSensor.x, sensor_filteredSensorReadings.positionSensor.y, sensor_filteredSensorReadings.positionSensor.t);
	
	
	//overwrite buffered data
	readings.obstacleSensor = sensor_filteredSensorReadings.obstacleSensor;
	readings.beaconSensor = sensor_filteredSensorReadings.beaconSensor;
	readings.positionSensor = sensor_filteredSensorReadings.positionSensor;
	readings.groundSensor = sensor_filteredSensorReadings.groundSensor;
	//~ printf("filteredSensorReadings: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_filteredSensorReadings.positionSensor.x, sensor_filteredSensorReadings.positionSensor.y, sensor_filteredSensorReadings.positionSensor.t);
	//~ 
	//~ printf("readings: x=%5.3f, y=%5.3f, t=%5.3f \n", readings.positionSensor.x, readings.positionSensor.y, readings.positionSensor.t);
	return readings;
}


SensorReadings get_new_sensorReadings(int state){
	refresh_sensorReadings(state);
	return sensor_sensorReadings;
}

SensorReadings get_new_filteredSensorReadings(int state){		
	refresh_sensorReadings(state);
	//~ printf("before get_filteredSensorReadings: x=%5.3f, y=%5.3f, t=%5.3f \n", sensor_filteredSensorReadings.positionSensor.x, sensor_filteredSensorReadings.positionSensor.y, sensor_filteredSensorReadings.positionSensor.t);
	return get_filteredSensorReadings();
}

