#ifndef __RMI_FP_SENSORS_H
#define __RMI_FP_SENSORS_H

/** \brief maximum number of points contained in breadcrums (robots memory for past positions)
 * */
#define MAX_NUM_POSITIONS		1000

#define SENSORS_INIT_POSITION_X		0.0
#define SENSORS_INIT_POSITION_Y		0.0
#define SENSORS_INIT_POSITION_T		0.0

/** \brief structures containing normalized sensor readings for the agents easier enterpretation
 * */
typedef struct{
	bool isVisible;
	double relative_direction;
	double apsolute_direction;
} BeaconSensor;

typedef struct
{
	double x;
	double y;
	double t;
	double compass_direction;
} PositionSensor;

typedef struct
{
	double x;
	double y;
	double t;
	double relative_direction;
} StartingPosition;

typedef struct
{
	double front;
	double left;
	double right;
} ObstacleSensor;

typedef struct
{
	ObstacleSensor obstacleSensor; ///< analog sensors structure
	BeaconSensor beaconSensor;
	PositionSensor positionSensor;
	StartingPosition startingPosition;
	int groundSensor;
	bool atBeaconArea;	
	int batteryVoltage;
	
	PositionSensor positionsHistory[MAX_NUM_POSITIONS];
	int last_position_index;	
} SensorReadings;

typedef struct{
	ObstacleSensor obstacleSensor; ///< analog sensors structure
	BeaconSensor beaconSensor;
	PositionSensor positionSensor;
	int groundSensor;
} SensorBufferReadings;

/** \brief inititalisation function, must be called before reading sensor data 
 * */
void sensors_init();

/** \brief feinitialisation function, call when robot finishes it's activity
 * */
void sensors_finish();

/** \brief returns SensorReadings structure containing last read sensor data
 * */
SensorReadings get_sensorReadings();

/** \brief returns SensorReadings structure containing last read filtered sensor data
 * */
SensorReadings get_filteredSensorReadings();

/** \brief returns SensorReadings structure containing new read sensor data
 * \details it refreshes the sensor readings before returning them and depending on the state parameter
 * it may refres all sensors or a part of them
 * \param state - determins what sensor readings are going to be refreshed
 * */
SensorReadings get_new_sensorReadings(int state);

/** \brief returns SensorBufferReadings structure containing new and filtered sensor data
 * \details it refreshes the sensor readings before returning them and depending on the state parameter
 * it may refres all sensors or a part of them
 * \param state - determins what sensor readings are going to be refreshed
 * */
SensorReadings get_new_filteredSensorReadings(int state);

/** \brief refreshes the internal sensor readings and stores it internally
 * */
void refresh_sensorReadings(int state);

SensorReadings get_accurate_sensor_reading();

#endif
