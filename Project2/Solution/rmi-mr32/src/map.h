#ifndef __RMI_FP_MAP_H
#define __RMI_FP_MAP_H

/**\brief these states define the state of a connection or a field
 * */
#define MAP_STATE_UNDEFINED					0
#define MAP_STATE_FREE						1
#define MAP_STATE_OCCUPIED					2

#define MAP_DIR_LEFT						0
#define MAP_DIR_RIGHT						1
#define MAP_DIR_UP							2
#define MAP_DIR_DOWN						3

#define MAP_FIELD_NUM_CONNECTIONS			4

#define MAX_MAP_FIELDS 						128


/**\brief strcture containing the xy position of a given point
 * */
typedef struct{
	double x;
	double y;
} PositionXY;

//this is neccesary because both srtuctures contain either pointers or values of one annother
typedef struct MapFieldConnection MapFieldConnection;
typedef struct MapField MapField;

/**\brief MapFieldConnection defines the vertex between two fields, it is defined as a one way arrow or pointer to annother field
 * \param *field - contains the ponter to the field it is pointing to
 * \param state - contains the state of the connection (MAP_STATE_UNDEFINED, MAP_STATE_FREE, MAP_STATE_OCCUPIED), if the state contains a MAP_STATE_UNDEFINED value than the *field pointer does not contain a pointer to annother field
 * */
struct MapFieldConnection{
	MapField *field;
	int state;
	//~ double distance;
};

/**\brief MapField defines a point on the map with all of its connections to other fields
 * \param connections[MAP_FIELD_NUM_CONNECTIONS] - contains connections to other fields
 * \param state - contains the state of the field (MAP_STATE_UNDEFINED, MAP_STATE_FREE, MAP_STATE_OCCUPIED)
 * \param field_index - contains index of the field inside the map array used and stored in map.c
 * \param position - contains the position of the field
 * \param num_visits - contains number of times that the robot has come to this field
 * */
struct MapField{
	MapFieldConnection connections[MAP_FIELD_NUM_CONNECTIONS];
	//~ MapFieldConnection *left;
	//~ MapFieldConnection *right;
	//~ MapFieldConnection *up;
	//~ MapFieldConnection *down;
	int state;
	int num_visits;
	int field_index;
	PositionXY position;
};

//~ struct MapFieldPath{
	//~ MapField *current_field;
	//~ MapField *next_field;
	//~ int distance;
//~ };

/**\brief function adds a connection to an existing field (map node)
 * */
MapField *add_field(MapField *field, int direction, int connection_state, int new_field_state, PositionXY position);

/**\brief sets starting filed 
 * \detailed - sets the startingField as the provided field parameter
 * */
void set_StartingField(MapField *field);

/**\brief sets the current field
* \detailed - sets the startingField as the provided field parameter
* 			it also updated the number of visits the field has had
 * */
void set_CurrentFiled(MapField *field);

/**\brief contains pointer to the starting point field
 * */
static MapField *startingField;

/**\brief contains pointer to the field the robot is currently located on (or if in transot than the one it is traveling from)
 * */
static MapField *currentField;

/**\brief contains pointer to the goal point field - (BEACON AREA)
 * */
static MapField *goalField;

/**\brief initializes the map and sets the starting and current fields
 * */
void map_init(PositionXY position);

/**\brief calculates the shortest path to the nearest unexplored field
* \detailed it uses dijkstra's algorythm to calculate shortest paths to all fields and after that it finds the one that has
* an unexplored neighbour and is the closest one to the current position
* \param return_path - this is an area of pointers to fields that will be populated with pointers to fields in a path order from the 0 index considered as the starting point of the calculated path
* \param end_index - pointer to an integer value. This value pointed by this variable will contain the index (inside the return path array) of the destination field.
* 			this is the end point of the path returned in the return_path array. All other values of this array are not to be trusted and cannot be considered a part of the path.
 * */
void get_path_to_unexplored_field(MapField *return_path[MAX_MAP_FIELDS], int *end_index);

/**\brief calculates the shortest path to the starting point
* \detailed it uses dijkstra's algorythm to calculate shortest paths to all fields and after that it finds the path to the starting field
* \param return_path - this is an area of pointers to fields that will be populated with pointers to fields in a path order from the 0 index considered as the starting point of the calculated path
* \param end_index - pointer to an integer value. This value pointed by this variable will contain the index (inside the return path array) of the destination field.
* 			this is the end point of the path returned in the return_path array. All other values of this array are not to be trusted and cannot be considered a part of the path.
 * */
void get_path_to_starting_field(MapField *return_path[MAX_MAP_FIELDS], int *end_index);


/**\brief determins if the 3 points are all in a single line 
* */
bool points_are_in_line(PositionXY p1, PositionXY p2, PositionXY p3);

/**\brief returns pointer to closest field in map and the distance in the resulting_distance value
* */
MapField *get_closest_field(PositionXY position, double *resulting_distance);


/**\brief prints out the mapas the robot currently sees it. It prints it to the standard utput using the printf function
* */
void print_out_map();


#endif
