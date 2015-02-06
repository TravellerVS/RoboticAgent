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

/**\brief sets up the wights array for later use in the dijkstra shortest path function
 * */
void get_dijkstra_weights(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES]);

/**\brief calculates the shortest path to all field from the myStartingField 
 * \param precede - will be populated with preceding indexes for each field for getting the shotest path. example: precede[chosen_index] = index_of_of_field_that_precedes_the_chosen_index
 * \param distance - will contain the minimal distances between the myStartingField and all other fields. example: distance[destination_field] = distance fro the starting field to the destination filed 
 * */
void calculate_shortest_paths(MapField *myStartingField, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);

/**\brief copys values from the path_end_index and path[MAX_MAP_FIELDS] into return_path[MAX_MAP_FIELDS] and end_index values
 * 		it is used in multiple functions to return the path
 * */
void copy_path_values(MapField *return_path[MAX_MAP_FIELDS], int *end_index);

/**\brief calculates the shortest path form the precede array, starting_index and destination_index and sets the path[MAX_MAP_FIELDS] and path_end_index values
 * \param starting_index - index from map_fields[MAX_MAP_FIELDS] of the starting point field
 * \param destination_index - index from map_fields[MAX_MAP_FIELDS] of the destination point field
 * */
void set_shortest_paths(int starting_index, int destination_index, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);

/**\brief array contains all the fields currently contained in the map.
 * */
static MapField map_fields[MAX_MAP_FIELDS];
/**\brief used as a counter of the current number of fileds existing in the map and for managing the map_fields array
 * */
static int map_num_fields = 0;

/**\brief contains pointers to fields and represents a pathh that can be returned
 * */
static MapField *path[MAX_MAP_FIELDS];

/**\brief contains the end value of the path
 * */
static int path_end_index = 0;

#define PRINT_MAP_MAX 		40
#define PRINT_MAP_OFFSET 	20

void print_out_map(){
	//field dimentions are 5x5 fields big so the area has to be 2 times wide and long that because of the unknown initial position of the robot in the field and 2 times that and plus1 because of the connections so 25X25 just for good measure (min is 22)
	printf("\n##########################\n RESULTING MAP \n##########################\n ");
	char print_array[PRINT_MAP_MAX][PRINT_MAP_MAX];
	int offset = PRINT_MAP_OFFSET;//to center everything;
	
	int y,x;
	for(y = 0; y<PRINT_MAP_MAX; y++){
		for(x = 0; x<PRINT_MAP_MAX; x++){
			print_array[x][y] = ' ';
		}
	}
	int index = 0;
	for(index = 0; index<map_num_fields; index++){
		int indexX = offset;
		int indexY = offset;
		
		indexX += 2*((int)(map_fields[index].position.x)/DISTANCE_BETWEEN_POINTS);
		indexY -= 2*((int)(map_fields[index].position.y)/DISTANCE_BETWEEN_POINTS);
		
		int field_state = map_fields[index].state;
		char c = '!';
		if(field_state == MAP_STATE_FREE){
			c = 'O';
		}
		else if(field_state == MAP_STATE_OCCUPIED){
			c = 'X';
		}
		else{
			c = '?';
		}
		if(map_fields[index].field_index == (*startingField).field_index){
			c = 'S';
		}
		if(map_fields[index].field_index == (*goalField).field_index){
			c = 'F';
		}
		
		print_array[indexX][indexY] = c;
		int direction = 0;
		for(direction = 0; direction<MAP_FIELD_NUM_CONNECTIONS; direction++)
		{
			int conection_state = map_fields[index].connections[direction].state;
			int connection_offsetX = 0;
			int connection_offsetY = 0;
			char free_connection_char = '!';
			if(direction == 0){
				connection_offsetX = 1;
				connection_offsetY = 0;
				free_connection_char = '-';
			}
			else if(direction == 1){
				connection_offsetX = 0;
				connection_offsetY = -1;
				free_connection_char = '|';
			}
			else if(direction == 2){
				connection_offsetX = -1;
				connection_offsetY = 0;
				free_connection_char = '-';
			}
			else if(direction == 3){
				connection_offsetX = 0;
				connection_offsetY = 1;
				free_connection_char = '|';
			}
			c = ' ';
			if(conection_state == MAP_STATE_FREE){
				c = free_connection_char;
			}
			else if(conection_state == MAP_STATE_OCCUPIED){
				c = 'x';
			}
			else{
				c = '?';
			}
			print_array[indexX+connection_offsetX][indexY+connection_offsetY] = c;
		}
	}
	printf("\n##########################\n RESULTING MAP ##########################\n");
	for(y = 0; y<PRINT_MAP_MAX; y++){
		for(x= 0; x<PRINT_MAP_MAX; x++){
			printf("%c",print_array[x][y]);
		}
		printf("\n");
	}
}

MapField *get_closest_field(PositionXY position, double *resulting_distance){
	MapField *field;
	field = NULL;
	double smallest_distance = DISTANCE_BETWEEN_POINTS*100;
	int index = 0;
	for(index = 0; index<map_num_fields; index++){
		double current_distance = distance_between_points(position.x, position.y, map_fields[index].position.x, map_fields[index].position.y);
		if(current_distance<=smallest_distance){
			smallest_distance = current_distance;
			field = &map_fields[index];
		}
	}
	*resulting_distance = smallest_distance;	
	return field;
}

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
	
	(*newField).state = ((*newField).state == MAP_STATE_UNDEFINED) ? new_field_state : (*newField).state ;
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
void set_GoalField(MapField *field){
	goalField = field;
}
void set_CurrentField(MapField *field){
	currentField = field;
	(*currentField).num_visits++;
	printf("Current field is: %d\n",(*currentField).field_index);
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

bool points_are_in_line(PositionXY p1, PositionXY p2, PositionXY p3){
	bool result = false;
	double angle1_2 = atan2((p2.y-p1.y), (p2.x-p1.x));
	double angle2_3 = atan2((p3.y-p2.y), (p3.x-p2.x));
	//~ printf("\n points_are_in_line %5.3f %5.3f",angle1_2, angle1_2);	
	if( angle1_2 == angle2_3 ){		
		result = true;
	}else{
	}
	return result;
}
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
	
	//testing the path
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
	//~ set_GoalField(currentField);
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
	//~ print_out_map();
	
	
	if(MAP_DEBUG){
		printf("currentField=%p,  startingField=%p\n", currentField, startingField);
		int i=0;
		for(i = 0; i<map_num_fields; i++){
			printf("Field list: Filed (%d) : pointer=%p X=%5.3f, Y=%5.3f  state=%d\n", i, (void *)&map_fields[i], map_fields[i].position.x,map_fields[i].position.y,map_fields[i].state);	
		}
	}
}


