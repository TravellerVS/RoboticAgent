#include <stdlib.h>
#include "dijkstra.h"

void dijkstra_shortest_path_to_destination(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES],int start_index, int destination_index, int num_nodes, int *distance_to_destination, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES]){
	dijkstra_shortest_path(weight, start_index, num_nodes, precede, distance);
	*distance_to_destination=distance[destination_index];
}

void dijkstra_shortest_path(int weight[DIJKSTRA_MAXNODES][DIJKSTRA_MAXNODES],int start_index, int num_nodes, int precede[DIJKSTRA_MAXNODES], int distance[DIJKSTRA_MAXNODES])
{
	//~ printf("\n Dijskstra SHORTEST PATH ");
	int checklist[DIJKSTRA_MAXNODES];
	int current_index;
	int i;	
	/* initialization of checklist and distance array */
	for(i=0;i<DIJKSTRA_MAXNODES;i++)
	{
		checklist[i]=UNCHECKED;
		distance[i]=DIJKSTRA_INFINITY;
	}
	checklist[start_index] = CHECKED;
	distance[start_index] = 0;
	precede[start_index]=start_index;
	current_index = start_index;
	if(num_nodes<=DIJKSTRA_MAXNODES && num_nodes>0)
	{
		while(true)
		{
			//~ //check if all nodes were visited
			//~ bool stop_execution = true;	
			//~ for(i=0;i<num_nodes && i<DIJKSTRA_MAXNODES;i++)
			//~ {
				//~ if(checklist[i]==UNCHECKED)
				//~ {
					//~ stop_execution = false;
					//~ break;
				//~ }
			//~ }
			//~ if(stop_execution){
				//~ break;			
			//~ }
			//~ 
			
			int current_distance = distance[current_index];
			
			for(i=0;i<DIJKSTRA_MAXNODES;i++)
			{
				if(checklist[i]==UNCHECKED && (weight[current_index][i]>0 && weight[current_index][i]<DIJKSTRA_INFINITY))
				{
					if(current_index == i){
						distance[i] = 0;
						continue;
					}
					int newdist = current_distance + weight[current_index][i];
					if(current_index == 5){
						//~ printf("\n current_index (%d - %d) :  dist: %d \n",current_index, i, newdist);
					}
					if(newdist < distance[i])
					{
						distance[i]=newdist;
						precede[i]=current_index;
					}
				} 
			}
			
			//~ printf("\n Dijskstra Distance (%d) : ",current_index);
			int smalldist=DIJKSTRA_INFINITY;
			int next_index = -1;
			for(i=0;i<DIJKSTRA_MAXNODES;i++)
			{
				//~ printf(" %d",distance[i]);
				if(checklist[i]==UNCHECKED && distance[i] <= smalldist){
					smalldist = distance[i];
					next_index = i;
				}
			}
			
			if(next_index>0){
				current_index = next_index;
				checklist[current_index]=CHECKED;
			}
			else
			{
				// end algorythm
				break;				
			}
			
		}
	}
	else
	{
		printf("\n ERROR: dijkstra.c - values are not correct\n");
		//values are not correct
	}
	//~ *pd=distance[t];
}


