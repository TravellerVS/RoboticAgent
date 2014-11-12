#ifndef __RMI_P2_SENSORS
#define __RMI_P2_SENSORS


#define MAX_NUM_POSITIONS		1000

typedef struct
{
	bool isVisible;
	double relative_direction;
	double apsolute_direction;
} BeaconSensor;

typedef struct
{
	double x;
	double y;
	double t;
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
	int front;
	int left;
	int right;
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


void sensors_init();
void sensors_finish();

void refresh_sensorReadings(int state);
SensorReadings get_sensorReadings();
SensorReadings get_new_sensorReadings(int state);

#endif
