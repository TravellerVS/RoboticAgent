#ifndef __RMI_P2_SENSORS
#define __RMI_P2_SENSORS

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
} SensorReadings;


void sensors_init();
void sensors_finish();

void refresh_sensorReadings();
SensorReadings get_sensorReadings();
SensorReadings get_new_sensorReadings();

#endif
