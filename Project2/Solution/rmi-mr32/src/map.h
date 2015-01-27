#ifndef __RMI_FP_MAP_H
#define __RMI_FP_MAP_H

#define MAP_STATE_UNDEFINED					0
#define MAP_STATE_FREE						1
#define MAP_STATE_OCCUPIED					2

#define MAP_DIR_LEFT						0
#define MAP_DIR_RIGHT						1
#define MAP_DIR_UP							2
#define MAP_DIR_DOWN						3

#define MAP_FIELD_NUM_CONNECTIONS			4

typedef struct{
	double x;
	double y;
} PositionXY;

typedef struct MapFieldConnection MapFieldConnection;

typedef struct MapField MapField;
struct MapField{
	MapFieldConnection *connections[MAP_FIELD_NUM_CONNECTIONS];
	//~ MapFieldConnection *left;
	//~ MapFieldConnection *right;
	//~ MapFieldConnection *up;
	//~ MapFieldConnection *down;
	int state;
	PositionXY position;
};
//todo: add distance to connections 
struct MapFieldConnection{
	MapField *field;
	int state;
	//~ double distance;
};

struct MapFieldPath{
	MapField *current_field;
	MapField *next_field;
	int distance;
}

/**\brief function adds a connection to an existing field (map node)
 * */
MapField *add_field(MapField *field, int direction, int connection_state, int new_field_state, PositionXY position);

/**\brief MapField constructor
 * \return pointer to allocated MapField structure
 * */
MapField *new_MapField();
/**\brief MapFieldConnection constructor
 * \return pointer to allocated MapFieldConnection structure
 * */
MapFieldConnection *new_MapFieldConnection();
/**\brief sets starting filed
 * \detailed used for calculating shortest path for calculating the return journey
 * */
void set_StartingField(MapField *field);
/**\brief sets the current field
* \detailed used for calculating shortest path for calculating the return journey, new unexplored areas...
 * */
void set_CurrentFiled(MapField *field);


int direction_to_narest_unxplored_field();

MapField *startingField;
MapField *currentField;

#endif
