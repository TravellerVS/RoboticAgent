#include "rmi-mr32.c"
//#include "bluetooth_comm.h"
#include "bluetooth_comm.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>

//#include "rmi-mr32.c"
#include "behaviors.c"
#include "movements.c"
#include "sensors.c"


int main(void){
	initPIC32();
	
	//configBTUart(3, 115200); // Configure Bluetooth UART
    //bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART
	
	closedLoopControl( true );
	movement_stop();
	printf("RMI-project2, robot %d\n\n\n", ROBOT);
	while(1)
	{		
		SensorReadings mySensorReadings;
		printf("Press start to continue\n");
		while(!startButton());
		//waitTick40ms();
		//while(stopButton());
		sensors_init();
		behaviors_init();
		//rotateRel_naive(M_PI/4);
		//rotateRel_naive(-M_PI/4);
		while(1){
			if(stopButton()){
				waitTick20ms();
				while(stopButton());
				break;
			}
			waitTick40ms();						// Wait for next 40ms tick
			mySensorReadings = get_new_sensorReadings();
			refresh_sensorReadings();
			execute_behavior(mySensorReadings);
		};
		behaviors_finish();
		sensors_finish();
		/*victory dance*/
		//rotateRel_naive(-M_PI/4);
		//rotateRel_naive(M_PI/2);
		//rotateRel_naive(-M_PI/4);
	}
	return 0;
}

