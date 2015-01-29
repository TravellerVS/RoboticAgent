#include <stdlib.h>
#include "map.h"

#define MAP_DEBUG 		false

/**\brief MapField constructor
 * \return pointer to allocated MapField structure
 * */
MapField new_MapField();
/**\brief MapFieldConnection constructor
 * \return pointer to allocated MapFieldConnection structure
 * */
MapFieldConnection new_MapFieldConnection();


MapField map_fields[MAX_MAP_FIELDS];

MapField *path[MAX_MAP_FIELDS];
int path_end_index = 0;

int map_num_fields = 0;

MapField *add_field(MapField *field, int direction, int connection_state, int new_field_state, PositionXY position){
	MapField *newField;
	bool is_old_field = false;
	int new_field_index = 0;
	//check if a field with almost the same position exists ad consider it as the new position
	for(new_field_index = 0; new_field_index<map_num_fields; new_field_index++){
		double distance = distance_between_points(position.x, position.y, map_fields[new_field_index].position.x, map_fields[new_field_index].position.y);
		if(distance<=DISTANCE_BETWEEN_POINTS/2.0){
			//newField = &map_fields[field_index];
			printf("\n\n\n###############  THE FIELD IS OLD  #################\n\n\n");
			is_old_field = true;
			break;
		}
	}
	if(is_old_field == false){
		new_field_index=map_num_fields;
		map_fields[new_field_index] = new_MapField();
		map_num_fields++;
	}
	newField = &map_fields[new_field_index];
	
	(*newField).state = new_field_state;
	(*newField).position.x = position.x;
	(*newField).position.y = position.y;
	//~ MapFieldConnection *newConnection;
	//~ newConnection = new_MapFieldConnection();
	//~ (*newConnection).field = newField;
	//~ (*newConnection).state = connection_state;
	//~ MapFieldConnection *returnConnection;
	//~ returnConnection = new_MapFieldConnection();
	//~ (*returnConnection).field = field;
	//~ (*returnConnection).state = connection_state;
	
	int returnDirection = (direction+(MAP_FIELD_NUM_CONNECTIONS/2))%MAP_FIELD_NUM_CONNECTIONS;
	//~ (*field).connections[direction].field_index = new_field_index;
	(*field).connections[direction].field = newField;
	(*field).connections[direction].state = connection_state;
	//~ (*newField).connections[returnDirection].field_index = new_field_index;
	(*newField).connections[returnDirection].field = field;
	(*newField).connections[returnDirection].state = connection_state;
	
	
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
	if(MAP_DEBUG){
		printf("New Field Added: newField: pointer=%d, X=%5.3f, Y=%5.3f,  state=%d  field: pointer=%d, X=%5.3f, Y=%5.3f,  state=%d\n", newField, (*newField).position.x, (*newField).position.y, (*newField).state, field, (*field).position.x, (*field).position.y, (*field).state);
		printf("Connection from new: direction=%d, pointer=%d, X=%5.3f, Y=%5.3f, state=%d \n", returnDirection, (*newField).connections[returnDirection].field, (*(*newField).connections[returnDirection].field).position.x, (*(*newField).connections[returnDirection].field).position.y, (*(*newField).connections[returnDirection].field).state);
		printf("Connection from original: direction=%d, pointer=%d, X=%5.3f, Y=%5.3f, state=%d \n", direction, (*field).connections[direction].field, (*(*field).connections[direction].field).position.x, (*(*field).connections[direction].field).position.y, (*(*field).connections[direction].field).state);
		
		printf("currentField=%d ,  startingField=%d\n", currentField, startingField);	
		int i=0;
		for(i = 0; i<map_num_fields; i++){
			printf("Field list: Filed (%d) : pointer=%d X=%5.3f, Y=%5.3f  state=%d\n", i, &map_fields[new_field_index], map_fields[i].position.x,map_fields[i].position.y,map_fields[i].state);	
			int j=0;
			for(j=0;j<MAP_FIELD_NUM_CONNECTIONS;j++)
			{
				if(map_fields[i].connections[j].state != MAP_STATE_UNDEFINED /*&& map_fields[i].connections[j].field == &map_fields[i]*/){
					//~ printf("\n###############  PROBLEM!!!!!!!!!  #################\n");
					//~ printf("i=%d, j=%d\n", i, j);	
					printf("i=%d, j=%d conn=%d, mapfield=%d \n", i, j, map_fields[i].connections[j].field, &map_fields[i]);	
				}
				else{
					printf("i=%d, j=%d, conn_state=%d, conn_state=%ds \n", i, j, MAP_STATE_UNDEFINED, connection_state);	
				}
			} 
		}
	}
	return newField;
}

MapField new_MapField(){
	MapField mf;// = malloc(sizeof(MapField));
	mf.state = MAP_STATE_UNDEFINED;
	int i;
	//~ printf("new_MapField state=%d \n",mf.state);
	for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)	{
		mf.connections[i] = new_MapFieldConnection();
		//~ printf("for connection i=%d state = %d\n",i,mf.connections[i].state);
	}
	return mf;
}
MapFieldConnection new_MapFieldConnection(){
	//~ printf("new_MapFieldConnection");
	MapFieldConnection mfc;//= (MapFieldConnection *) malloc(sizeof(MapFieldConnection));
	mfc.state = MAP_STATE_UNDEFINED;
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

void get_path_to_nearest_unexplored_field(int **return_path, int *end_index)
{
	*return_path = *path;
	*end_index = path_end_index;
}

void get_path_recursion()
{
		
}

//~ 
//~ int direction_to_narest_unxplored_field(){
	//~ MapFeldPath startPoint;
	//~ //startPoint//??
	//~ return direction_to_narest_unxplored_field(currentField);
//~ }
//~ int direction_to_narest_unxplored_field_recursion(MapField *field)
//~ {
	//~ int i = 0;
	//~ for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)
	//~ {
		//~ if((*field).connections[i].state == MAP_STATE_UNDEFINED){
			//~ return i;
		//~ }
	//~ }
	//~ for( i=0; i<MAP_FIELD_NUM_CONNECTIONS; i++)
	//~ {
		//~ if((*field).connections[i].state == MAP_STATE_FREE){
			//~ int next_direction = direction_to_narest_unxplored_field_recursion((*field).connections[i].field);
			//~ if(next_direction >= 0)
			//~ {
				//~ return next_direction;
			//~ }
		//~ }
	//~ }
	//~ return -1;
//~ }

void map_init(PositionXY position){
	printf("init_map \n");
	map_num_fields = 0;
	path_end_index = 0;
	int new_field_index = map_num_fields;
	map_fields[new_field_index] = new_MapField();
	map_num_fields++;
	map_fields[new_field_index].state = MAP_STATE_FREE;
	map_fields[new_field_index].position.x = position.x;
	map_fields[new_field_index].position.x = position.x;
	//~ MapField *newField;
	//~ newField = &map_fields[new_field_index];
	//~ (*newField).state = MAP_STATE_FREE;
	//~ (*newField).position.x = position.x;
	//~ (*newField).position.y = position.y;
	set_StartingField(&map_fields[new_field_index]);
	set_CurrentField(&map_fields[new_field_index]);
	
	
	//testing the path
	PositionXY path_position = (*startingField).position;
	int path_index = 0;
	path[path_index] = startingField;
	
	path_index++;
	path_position.x += DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 0, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	path_index++;
	path_position.x += DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 0, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	
	path_index++;
	path_position.y += DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 1, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	path_index++;
	path_position.y += DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 1, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	path_index++;
	path_position.x -= DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 2, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	
	path_index++;
	path_position.x -= DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 2, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	path_index++;
	path_position.y -= DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 3, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	path_index++;
	path_position.y -= DISTANCE_BETWEEN_POINTS;
	path[path_index] = add_field(path[path_index-1], 3, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	path_end_index++;
	
	
	if(MAP_DEBUG){
		printf("currentField=%d ,  startingField=%d\n", currentField, startingField);
		int i=0;
		for(i = 0; i<map_num_fields; i++){
			printf("Field list: Filed (%d) : pointer=%d X=%5.3f, Y=%5.3f  state=%d\n", i, &map_fields[i], map_fields[i].position.x,map_fields[i].position.y,map_fields[i].state);	
		}
	}
}


