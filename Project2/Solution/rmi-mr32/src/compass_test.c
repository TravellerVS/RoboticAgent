#include "rmi-mr32.c"
#include "rmi_compass.h"
#include <detpic32.h>

//void wait(int i);

int main(void)
{
	initCompass();

	while(1)
	{
		wait(1);
		printf("Comp=%d\n", getCompassValue());
	}
}



//void wait(int i)
//{
	//for(; i > 0; i--)
	//{
		//resetCoreTimer();
		//while(readCoreTimer() < 2000000);
	//}
//}
