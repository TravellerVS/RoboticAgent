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
#define SPEED_BEFORE_STOP 				15
#define SPEED_SLOW 						20
#define SPEED_MEDIUM 					40
#define SPEED_FAST 						60
#define SPEED_ROTATE 					40
#define SPEED_ROTATE_BEFORE_STOP		15
#define SPEED_ROTATE_SLOW				20
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
#define DISTANCE_COLISION_FRONT				15.0
#define DISTANCE_COLISION_SIDES				15.0
#define DISTANCE_NEAR_FRONT					30.0
#define DISTANCE_NEAR_SIDES					40.0//35.0
#define DISTANCE_FAR_SIDES					65.0
#define DISTANCE_FAR_FRONT					65.0
#define DISTANCE_CLOSE_SIDES				23.0

/** \brief aproximate distance buffer vaulues for the return journey of the robot 
 * */
#define STARTING_POINT_BUFFER				2.0
#define CLOSE_TO_STARTING_POINT				20.0
#define OTHER_POINT_BUFFER					10.0
#define DISTANCE_XY_CLOSE					5.0

#define DISTANCE_BETWEEN_POINTS 			112.0


/** \brief pseudo enumeration of the states
 * */
#define STATE_DEFAULT						0
#define STATE_INIT							1
#define STATE_SEARCHING_FOR_BEACON_AREA		2
#define STATE_RETURNING_HOME				3
#define STATE_SEARCHING_FOR_BEACON			4 //depricated

#endif


