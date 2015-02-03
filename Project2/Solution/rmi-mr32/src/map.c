#include <stdlib.h>
#include "dijkstra.h"
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


void get_dijkstra_weights(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES]);
void calculate_shortest_paths(MapField *myStartingField, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);
void copy_path_values(MapField *return_path[MAX_MAP_FIELDS], int *end_index);
void set_shortest_paths(int starting_index, int destination_index, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);


static MapField map_fields[MAX_MAP_FIELDS];

static MapField *path[MAX_MAP_FIELDS];
static int path_end_index = 0;

static int map_num_fields = 0;

MapField *add_field(MapField *field, int direction, int connection_state, int new_field_state, PositionXY position){
	MapField *newField;
	bool is_old_field = false;
	int new_field_index = 0;
	//check if a field with almost the same position exists ad consider it as the new position
	for(new_field_index = 0; new_field_index<map_num_fields; new_field_index++){
		double distance = distance_between_points(position.x, position.y, map_fields[new_field_index].position.x, map_fields[new_field_index].position.y);
		if(distance<=DISTANCE_BETWEEN_POINTS/2.0){
			//newField = &map_fields[field_index];
			//~ printf("\n\n\n###############  THE FIELD IS OLD  #################\n\n\n");
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
	
	int returnDirection = (direction+(MAP_FIELD_NUM_CONNECTIONS/2))%MAP_FIELD_NUM_CONNECTIONS;
	(*field).connections[direction].field = newField;
	(*field).connections[direction].state = connection_state;
	
	(*newField).connections[returnDirection].field = field;
	(*newField).connections[returnDirection].state = connection_state;
	(*newField).field_index = new_field_index;
	
	if(MAP_DEBUG){
		printf("New Field Added: newField: pointer=%p, X=%5.3f, Y=%5.3f,  state=%d  field: pointer=%p, X=%5.3f, Y=%5.3f,  state=%d\n", newField, (*newField).position.x, (*newField).position.y, (*newField).state, field, (*field).position.x, (*field).position.y, (*field).state);
		printf("Connection from new: direction=%d, pointer=%p, X=%5.3f, Y=%5.3f, state=%d \n", returnDirection, (*newField).connections[returnDirection].field, (*(*newField).connections[returnDirection].field).position.x, (*(*newField).connections[returnDirection].field).position.y, (*(*newField).connections[returnDirection].field).state);
		printf("Connection from original: direction=%d, pointer=%p, X=%5.3f, Y=%5.3f, state=%d \n", direction, (*field).connections[direction].field, (*(*field).connections[direction].field).position.x, (*(*field).connections[direction].field).position.y, (*(*field).connections[direction].field).state);
		
		printf("currentField=%p ,  startingField=%p\n", currentField, startingField);	
		int i=0;
		for(i = 0; i<map_num_fields; i++){
			printf("Field list: Filed (%d) : pointer=%p X=%5.3f, Y=%5.3f  state=%d\n", i, &map_fields[new_field_index], map_fields[i].position.x,map_fields[i].position.y,map_fields[i].state);	
			int j=0;
			for(j=0;j<MAP_FIELD_NUM_CONNECTIONS;j++)
			{
				if(map_fields[i].connections[j].state != MAP_STATE_UNDEFINED /*&& map_fields[i].connections[j].field == &map_fields[i]*/){
					//~ printf("\n###############  PROBLEM!!!!!!!!!  #################\n");
					//~ printf("i=%d, j=%d\n", i, j);	
					printf("i=%d, j=%d conn=%p, mapfield=%p \n", i, j, map_fields[i].connections[j].field, &map_fields[i]);	
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
	mf.num_visits = 0;
	mf.field_index = -1;
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
	(*currentField).num_visits++;
}



void copy_path_values(MapField *return_path[MAX_MAP_FIELDS], int *end_index){
	int i = 0;
	for(i=0;i<=path_end_index;i++){
		return_path[i] = path[i];
	}
	*end_index = path_end_index;
}

void calculate_shortest_paths(MapField *myStartingField, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]){
	int current_field_index= (*myStartingField).field_index;
	int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES];
	get_dijkstra_weights(weight);
	dijkstra_shortest_path(weight, current_field_index, map_num_fields, precede, distance);
}

void get_dijkstra_weights(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES])
{
	int i,j;
	for(i = 0;i<map_num_fields && i<DIJKSTRA_MAXNODES;i++){
		for(j= 0;j<DIJKSTRA_MAXNODES;j++){
			weight[i][j] = 0;// OR DIJKSTRA_INFINITY
		}
		for(j= 0;j<MAP_FIELD_NUM_CONNECTIONS;j++){
			if(map_fields[i].connections[j].state == MAP_STATE_FREE){
				int neighbour_index = (*map_fields[i].connections[j].field).field_index;
				weight[i][neighbour_index] = 1;
			}
		}
		if(MAP_DEBUG){
			for(j= 0;j<map_num_fields;j++){
				printf ("%d ",weight[i][j]);
			}
			printf("\n");
		}
	}
}


void set_shortest_paths(int starting_index, int destination_index, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES])
{
	int path_index = 0;
	path[path_index] = &map_fields[destination_index];
	int i;
	//get and set reverse path form precede array
	while(true){
		destination_index = precede[destination_index];
		path_index++;
		path[path_index] = &map_fields[destination_index];
		//~ printf("-> %d ", destination_index);
		if(destination_index == starting_index){
			break;
		}
	}
	path_end_index = path_index;
	//reverse the order of the path to get the correct path
	//~ printf("\nREVERSE\n");
	for(i=0;(2*i) < path_end_index;i++){
		//~ printf("%d %d   %d %d \n", i, (path_end_index-i),(*path[i]).field_index, (*path[path_end_index-i]).field_index );
		MapField *tempPointer = path[i];
		path[i] = path[path_end_index-i];
		path[path_end_index-i] = tempPointer;
	}
	
	if(MAP_DEBUG){
		printf("\nSHORTEST PATH from (%d) TO (%d): ",starting_index,destination_index);
		for(i=0;i <= path_end_index;i++){
			printf("-> %d ", (*path[i]).field_index);
		}
		printf("\n Distances: ");
		for(i= 0;i<map_num_fields;i++){
			printf(" %d",distance[i]);
		}
		printf("\n");
	}
}

void get_path_to_starting_field(MapField *return_path[MAX_MAP_FIELDS], int *end_index){
	
	int precede[DIJKSTRA_MAXNODES];
	int distance[DIJKSTRA_MAXNODES];
	int current_field_index= (*currentField).field_index;
	calculate_shortest_paths(currentField, precede, distance);
	int destination_index = (*startingField).field_index;
	//~ int min_distance = distance[(*startingField).field_index];
	set_shortest_paths(current_field_index, destination_index, precede, distance);
	
	copy_path_values(return_path, end_index);
}

void get_path_to_unexplored_field(MapField *return_path[MAX_MAP_FIELDS], int *end_index)
{	
	int precede[DIJKSTRA_MAXNODES];
	int distance[DIJKSTRA_MAXNODES];
	int current_field_index= (*currentField).field_index;
	calculate_shortest_paths(currentField, precede, distance);
	int i,j;
	int shortest_distance = DIJKSTRA_INFINITY;
	//get destination index
	int destination_index = -1; 
	//~ printf("shortest_distance\n");
	for(i=0; i<map_num_fields;i++){
		//~ printf("shortest_distance = %d, dist_i = %d \n", shortest_distance, distance[i]);
		if(distance[i]<shortest_distance){
			for(j= 0;j<MAP_FIELD_NUM_CONNECTIONS;j++){
				if(map_fields[i].connections[j].state == MAP_STATE_UNDEFINED){
					shortest_distance = distance[i];
					destination_index = i;
				}
			}
		}
	}
	if(destination_index >-1){
		set_shortest_paths(current_field_index, destination_index, precede, distance);
		copy_path_values(return_path, end_index);
	}
	else
	{
		printf("No path to unexplored filed discovered\n");
		*end_index = 0;
	}
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
	map_fields[new_field_index].field_index = new_field_index;
	//~ MapField *newField;
	//~ newField = &map_fields[new_field_index];
	//~ (*newField).state = MAP_STATE_FREE;
	//~ (*newField).position.x = position.x;
	//~ (*newField).position.y = position.y;
	set_StartingField(&map_fields[new_field_index]);
	set_CurrentField(&map_fields[new_field_index]);
	
	//~ //testing the path
	//~ PositionXY path_position = (*startingField).position;
	//~ int path_index = 0;
	//~ path[path_index] = startingField;
	//~ 
	//~ path_index++;
	//~ path_position.x += DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 0, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.x += DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 0, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.y += DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 1, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.y += DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 1, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.x -= DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 2, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.x -= DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 2, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.y -= DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 3, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ path_index++;
	//~ path_position.y -= DISTANCE_BETWEEN_POINTS;
	//~ path[path_index] = add_field(path[path_index-1], 3, MAP_STATE_FREE, MAP_STATE_FREE, path_position);
	//~ path_end_index++;
	//~ set_CurrentField(path[path_index]);
	//~ 
	//~ set_CurrentField(path[4]);
	//~ 
	//~ get_path_to_starting_field(path, &path_end_index);
	//~ get_path_to_unexplored_field(path, &path_end_index);
	
	if(MAP_DEBUG){
		printf("currentField=%p,  startingField=%p\n", currentField, startingField);
		int i=0;
		for(i = 0; i<map_num_fields; i++){
			printf("Field list: Filed (%d) : pointer=%p X=%5.3f, Y=%5.3f  state=%d\n", i, (void *)&map_fields[i], map_fields[i].position.x,map_fields[i].position.y,map_fields[i].state);	
		}
	}
}


