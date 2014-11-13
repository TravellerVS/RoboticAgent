#ifndef __RMI_P2_CONFIG
#define __RMI_P2_CONFIG

#define RIGHT 			true
#define LEFT 			false

#define ROBOT_WIDTH 	20.0

#define TURN_RADIUS_SMALL	20.0
#define TURN_RADIUS_MEDIUM 	30.0
#define TURN_RADIUS_BIG		35.0	
#define TURN_RADIUS_GO_ROUND_CORNER		28.0

#define SPEED_SLOW 			30
#define SPEED_MEDIUM 		40
#define SPEED_FAST 			80
#define SPEED_ROTATE 		60
#define SPEED_ROTATE_SLOW	30
#define SPEED_FOLLOW_WALL 	40

#define DISTANCE_FOLLOW_WALL_IDEAL_MIN	400
#define DISTANCE_FOLLOW_WALL_IDEAL_MAX	320 
#define DISTANCE_FOLLOW_WALL_SIDES 		300 	
#define DISTANCE_FOLLOW_WALL_FRONT 		600 	
#define DISTANCE_COLISION_FRONT			600		
#define DISTANCE_COLISION_SIDES			550		
#define DISTANCE_NEAR_FRONT				200
#define DISTANCE_NEAR_SIDES				430	
#define DISTANCE_FAR_SIDES				200	

#define STARTING_POINT_BUFFER			50.0
#define CLOSE_TO_STARTING_POINT			1000.0

#define OTHER_POINT_BUFFER				100.0

#define DISTANCE_XY_CLOSE				500.0

#define STATE_DEFAULT					0
#define STATE_SEARCHING_FOR_BEACON		1
#define STATE_RETURNING_HOME			2

#endif


