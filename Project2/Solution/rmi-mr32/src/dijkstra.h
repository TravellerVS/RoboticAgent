#ifndef __RMI_FP_DIJKSTRA_H
#define __RMI_FP_DIJKSTRA_H

#include "map.h"

#define DIJKSTRA_INFINITY 2000
#define DIJKSTRA_MAXNODES MAX_MAP_FIELDS
#define CHECKED 1
#define UNCHECKED 0

void dijkstra_shortest_path_to_destination(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES],int start_index, int destination_index, int num_nodes, int *distance_to_destination, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);

void dijkstra_shortest_path(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES],int start_index, int num_nodes, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]);

#endif
