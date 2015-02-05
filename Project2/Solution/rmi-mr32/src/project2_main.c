//these should be libraries

#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include "bluetooth_comm.h"
#include "bluetooth_comm.c"

#include "helperFunctions.c"
#include "sensors.c"
#include "map.c"
#include "movements.c"
#include "behaviors.c"
#include "dijkstra.c"

#include <math.h>
#include <string.h>
#include <stdlib.h>

//~ #include "helperFunctions.h"
#include "sensors.h"
//~ #include "map.h"
//~ #include "movements.h"
#include "behaviors.h"

int main(void){
	initPIC32();
	
	configBTUart(3, 115200); // Configure Bluetooth UART
    bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART
	
	closedLoopControl( true );
	movement_stop();
	printf("RMI-project2, robot %d\n\n\n", ROBOT);
	while(1)
	{		
		int behavior_control = 0;
		//~ SensorReadings mySensorReadings;
		printf("Press start to continue\n");		
				
		while(!startButton()){
			//for demonstarion
			//for changing behaviors from all to avoid colisio, to follow wall and to follow and stop at beacon
			if(stopButton()){
				behavior_control++;
				behavior_control=behavior_control%4;
				leds(behavior_control);
				wait(3);
				while(stopButton());
			}			
		};
		//wait(1);
		//waitTick40ms();
		//while(stopButton());
		sensors_init();
		behaviors_init();
		
		printf("initialisation finished \n");
		//rotateRel_naive(M_PI/4);
		//rotateRel_naive(-M_PI/4);
		while(1){
			if(stopButton()){
				waitTick20ms();
				while(stopButton());
				break;
			}
			waitTick20ms();
			//waitTick40ms();						// Wait for next 40ms tick
			//~ wait(1);	
			//~ printf("################### behavior CALL ###################\n");
			if(execute_behavior(behavior_control)==1){				
				printf("!!!!!!!!!!!!!FINISHED!!!!!!!!!!!!!!!!!\n");
				behaviors_finish();
				sensors_finish();
				/*victory dance  :) */
				//~ rotateRel_naive(-M_PI/4);
				//~ rotateRel_naive(M_PI/2);
				//~ rotateRel_naive(-M_PI/4);
				return 0;
			}
			//~ printf("################### behavior FINISHED ###################\n");
			
		};
		behaviors_finish();
		sensors_finish();
	}
	return 0;
}

