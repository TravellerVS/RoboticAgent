#ifndef __RMI_FP_CONFIG_CONSTANTS_H
#define __RMI_FP_CONFIG_CONSTANTS_H

/** \brief left and right helper constants
 * */
#define RIGHT 			true
#define LEFT 			false

#define YES 			true
#define NO 				false

/** \brief robot width in cm
 * */
#define ROBOT_WIDTH 	20.0

/** \brief speed values [0,100]
 * */ 
#define TURN_RADIUS_SMALL				20.0
#define TURN_RADIUS_MEDIUM 				30.0
#define TURN_RADIUS_BIG					35.0	
#define TURN_RADIUS_GO_ROUND_CORNER		28.0

/** \brief speeds
 * */
#define SPEED_BEFORE_STOP 				10
#define SPEED_SLOW 						20
#define SPEED_MEDIUM 					50
#define SPEED_FAST 						80
#define SPEED_ROTATE_BEFORE_STOP		10
#define SPEED_ROTATE_SLOW				20
#define SPEED_ROTATE 					30
#define SPEED_ROTATE_FAST 				50
#define SPEED_FOLLOW_WALL 				30

//~ #define SPEED_SLOW 						10
//~ #define SPEED_MEDIUM 					10
//~ #define SPEED_FAST 						10
//~ #define SPEED_ROTATE 					10
//~ #define SPEED_ROTATE_SLOW				10
//~ #define SPEED_FOLLOW_WALL 				10

/** \brief aproximate distances in cm 
 * */
#define DISTANCE_FOLLOW_WALL_IDEAL_MIN		19.0 
#define DISTANCE_FOLLOW_WALL_IDEAL_MAX		24.0
#define DISTANCE_FOLLOW_WALL_SIDES 			23.0	
#define DISTANCE_FOLLOW_WALL_FRONT 			16.0	
#define DISTANCE_COLISION_FRONT				8.0
#define DISTANCE_COLISION_SIDES				8.0
#define DISTANCE_NEAR_FRONT					30.0
#define DISTANCE_NEAR_SIDES					30.0//35.0//40.0
#define DISTANCE_FAR_SIDES					50.0
#define DISTANCE_FAR_FRONT					50.0
#define DISTANCE_CLOSE_SIDES				25.0

#define DISTANCE_NEXT_POINT_FRONT			45.0//DISTANCE_FAR_FRONT//DISTANCE_NEAR_FRONT//30.0
#define DISTANCE_NEXT_POINT_SIDES			19.0//DISTANCE_CLOSE_SIDES//DISTANCE_CLOSE_SIDES//40.0
#define DISTANCE_NEXT_FRONT_POINT_SIDES_FREE	60.0
#define DISTANCE_NEXT_FRONT_POINT_SIDES_OCCUPIED	40.0//37.0
//~ #define DISTANCE_NEXT_SIDE_POINT_SIDES		DISTANCE_FAR_SIDES//40.0

#define DISTANCE_FROM_CENTER_TO_WALL_FRONT  20.0



/** \brief aproximate distance buffer vaulues for the return journey of the robot -> in mm
 * */
#define STARTING_POINT_BUFFER				20.0
#define CLOSE_TO_STARTING_POINT				40.0
#define OTHER_POINT_BUFFER					20.0
#define DISTANCE_XY_CLOSE					20.0

#define DISTANCE_BETWEEN_POINTS 			450.0//112.0
#define DISTANCE_FROM_CENTER_TO_WALL_TOLERANCE  30.0
/** \brief pseudo enumeration of the states
 * */
#define STATE_DEFAULT						0
#define STATE_INIT							1
#define STATE_SEARCHING_FOR_BEACON_AREA		2
#define STATE_RETURNING_HOME				3
#define STATE_SEARCHING_FOR_BEACON			4 //depricated

#endif


