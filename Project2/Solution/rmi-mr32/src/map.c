#include <stdlib.h>
#include "map.h"

#define MAX_MAP_FIELDS 1000;

MapField *map_fields[1000];

MapField *path[1000];

int map_num_fields = 0;

MapField *add_field(MapField *field, int direction, int connection_state, int new_field_state, PositionXY position){
	MapField *newField;
	bool is_old_field = false;
	int field_index = 0;
	for(field_index = 0; field_index<map_num_fields; field_index++){
		double distance = distance_between_points((*field).position.x, (*field).position.y, (*map_fields[field_index]).position.x, (*map_fields[field_index]).position.y);
		if(distance<=DISTANCE_XY_CLOSE){
			newField = map_fields[field_index];
			is_old_field = true;
			break;
		}
	}
	if(!is_old_field){
		newField = new_MapField();
	}
	
	(*newField).state = new_field_state;
	(*newField).position.x = position.x;
	(*newField).position.y = position.y;
	MapFieldConnection *newConnection;
	newConnection = new_MapFieldConnection();
	(*newConnection).field = newField;
	(*newConnection).state = connection_state;
	MapFieldConnection *returnConnection;
	returnConnection = new_MapFieldConnection();
	(*returnConnection).field = field;
	(*returnConnection).state = connection_state;
	
	int returnDirection = (direction+(MAP_FIELD_NUM_CONNECTIONS/2))%MAP_FIELD_NUM_CONNECTIONS;
	(*field).connections[direction] = newConnection;
	(*field).connections[returnDirection] = returnConnection;
	
	//~ switch(direction){
		//~ case MAP_DIR_LEFT:
			//~ (*field).left = newConnection;
			//~ (*newField).right = returnConnection;
			//~ break;
		//~ case MAP_DIR_RIGHT:
			//~ (*field).right = newConnection;
			//~ (*newField).left = returnConnection;
			//~ break;
		//~ case MAP_DIR_UP:
			//~ (*field).up = newConnection;
			//~ (*newField).down = returnConnection;
			//~ break;
		//~ case MAP_DIR_DOWN:
			//~ (*field).down = newConnection;
			//~ (*newField).up = returnConnection;
			//~ break;
		//~ default:
			//~ break; 
	//~ }
	return newField;
}

MapField *new_MapField(){
	MapField *mf = (MapField *) malloc(sizeof(MapField));
	(*mf).state = MAP_STATE_UNDEFINED;
	return mf;
}
MapFieldConnection *new_MapFieldConnection(){
	MapFieldConnection *mfc= (MapFieldConnection *) malloc(sizeof(MapFieldConnection));
	(*mfc).state = MAP_STATE_UNDEFINED;
	return mfc;
}

void set_StartingField(MapField *field){
	startingField = field;
}
void set_CurrentField(MapField *field){
	currentField = field;
}


void get_path_to_start()
{
	
}

void get_path_to_nearest_unexplored_field()
{
	
}

void get_path_recursion()
{
	
	
	
}


int direction_to_narest_unxplored_field(){
	MapFeldPath startPoint;
	startPoint//??
	return direction_to_narest_unxplored_field(currentField);
}
int direction_to_narest_unxplored_field_recursion(MapField *field)
{
	int i = 0;
	for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)
	{
		if((*field).connections[i].state == MAP_STATE_UNDEFINED){
			return i;
		}
	}
	for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)
	{
		if((*field).connections[i].state == MAP_STATE_FREE){
			int next_direction = direction_to_narest_unxplored_field_recursion((*field).connections[i].field);
			if(next_direction >= 0)
			{
				return next_direction;
			}
		}
	}
	return -1;
}

void init_map(PositionXY position){
	MapField *newField;
	newField = new_MapField();
	(*newField).state = MAP_STATE_FREE;
	(*newField).position.x = position.x;
	(*newField).position.y = position.y;
	set_StartingField(newField);
	set_CurrentField(newField);
}






