/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   search.h
 * Author: ragubanc
 *
 * Created on March 18, 2020, 10:17 PM
 */

#ifndef SEARCH_H
#define SEARCH_H
#include "GlobalVariables.h"
#include "m1.h"
#include "m2.h"
//Uses the A* algorithm in order to find the shortest path interms of 
//travel time
bool a_star(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty);
//used to figure out the walk path
bool dijkstra_walk(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty, const double walking_speed, 
                        bool trace);

//
bool dijkstra(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty, bool trace);

bool dijkstra_set_time(const IntersectionIndex intersect_id_start,
        std::vector<IntersectionIndex> end_intersections,
        const double turn_penalty, bool trace);

bool dijkstra_local_graph(const IntersectionIndex intersect_id_start,
        std::vector<IntersectionIndex> end_intersections,
        const double turn_penalty, std::vector<node> & local_node_graph);

//used to get the path that was determined to be the shortest
void trace_back (node *dest_node, bool walk);


bool check_same_position (IntersectionIndex new_node_id, IntersectionIndex current_node_id);
//clears the node graph values
void clear_node_graph();

bool is_reached (std::vector<IntersectionIndex> end_intersections, int start);
bool is_reached_local (std::vector<IntersectionIndex> end_intersections, int start, std::vector<node> & local_node_graph);
void clear_local_graph (std::vector <node> & local_node_graph);
#endif /* SEARCH_H */


