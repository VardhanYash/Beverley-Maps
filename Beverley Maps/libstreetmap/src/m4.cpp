/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "search.h"
#include "GlobalVariables.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
#include <vector>
#include <bits/stdc++.h>
#include <chrono>
#include "optimization.h"
#include "is_legal.h"
#define TIME_LIMIT 50

std::vector<DeliveryInfo> global_deliveries;
float max_weight;
double time_to_beat;

IntersectionIndex find_closest_depot (const std::vector<int> & depots, IntersectionIndex current_intersection);
bool all_dropped_off();

std::vector<bool> is_pickedup;
std::vector<bool> is_droppedoff;
std::vector<CourierSubpath> traveling_courier(
		            const std::vector<DeliveryInfo>& deliveries,
                            const std::vector<int>& depots, 
		            const float turn_penalty, 
		            const float truck_capacity){
    
   
    auto startTime = std::chrono::high_resolution_clock::now();

    
    std::vector<CourierSubpath> route = {};
    
    //pre computed travel times based on what the type of intersection is
    std::vector<std::vector<double>> pick_to_pick;
    std::vector<std::vector<double>> pick_to_drop;
    std::vector<std::vector<double>> drop_to_pick;
    std::vector<std::vector<double>> drop_to_drop;
    
    
    pick_to_pick.resize(deliveries.size());
    pick_to_drop.resize(deliveries.size());
    drop_to_pick.resize(deliveries.size());
    drop_to_drop.resize(deliveries.size());
    
 
    //stores a vector of all the places it may potentially need to travel
    std::vector<int> picks_and_drops;
    for (int i = 0; i < deliveries.size(); i++) {
        picks_and_drops.push_back(deliveries[i].pickUp);
        picks_and_drops.push_back(deliveries[i].dropOff);
    }
    
#pragma omp parallel for
    for (int i = 0; i < deliveries.size(); i++) {
        std::vector<node> local_node_graph;
        
        //sets the times from the specified pickup to all the picks and drops
        dijkstra_local_graph(deliveries[i].pickUp, picks_and_drops, turn_penalty, local_node_graph);
        
        pick_to_pick[i].resize(deliveries.size());
        pick_to_drop[i].resize(deliveries.size());
        // filling the vector with the times 
        for (int j = 0; j < deliveries.size(); j++) {
            pick_to_pick[i][j] = (local_node_graph[deliveries[j].pickUp].cost);
            pick_to_drop[i][j] = (local_node_graph[deliveries[j].dropOff].cost);
        }
        
        //sets the times from the specified dropoff to all the picks and drops
        dijkstra_local_graph(deliveries[i].dropOff, picks_and_drops, turn_penalty, local_node_graph);
        
        drop_to_pick[i].resize(deliveries.size());
        drop_to_drop[i].resize(deliveries.size());
        
        //filling the other 2 vectors with the times
        for (int j = 0; j < deliveries.size(); j++) {
            drop_to_pick[i][j] = (local_node_graph[deliveries[j].pickUp].cost);
            drop_to_drop[i][j] = (local_node_graph[deliveries[j].dropOff].cost);
        }
    }
    auto currentTime1 = std::chrono::high_resolution_clock::now();
    auto wallClock1 = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime1 - startTime);
    std::cout << wallClock1.count() << std::endl;
    
        
    double best_travel_time = 1000000000000;
    std::vector<delivery_item> best_order = {};
    
    for (int i = 0; i < deliveries.size(); i++) {
        
        //initializes all packages as not picked up
        for (int j = 0; j < deliveries.size(); j++) {
            is_pickedup.push_back(false);
            if (deliveries[j].itemWeight > truck_capacity) {
                return route;
            }
        }
        
        //initializes all packages as not dropped off 
        for (int j = 0; j < deliveries.size(); j++) {
            is_droppedoff.push_back(false);
        }
        
        
        delivery_item current_item;
        current_item.delivery_id = i;
        current_item.pickup = true;
        delivery_item next_item;
        
        float current_weight = 0;
        double current_travel_time = 0;
        std::vector<delivery_item> delivery_order = {};
        
        delivery_order.push_back(current_item);
        current_weight = deliveries[i].itemWeight;
        is_pickedup[i] = true;
       
        //until all the deliveries have been made
        while(!all_dropped_off()){
  
            // if we see that the path time we are on has already exceeded best time,
            // no need to continue on that path
            if(current_travel_time >= best_travel_time){
                break;
            }
            
            
            double item_time = 10000000000;
            
            //if the current intersection is a pickup
            if(current_item.pickup) {
                
                for (int j = 0; j < deliveries.size(); j++) {

                    double potential_truck_weight = current_weight + deliveries[j].itemWeight;
                    
                    // the conditions to travel to another pickup after the current one
                    if (pick_to_pick[current_item.delivery_id][j] <= item_time && !is_pickedup[j] && potential_truck_weight < truck_capacity) {
                        item_time = pick_to_pick[current_item.delivery_id][j];
                        next_item.delivery_id = j;
                        next_item.pickup = true;
                    }
                    
                    // the condition to drop off a package that we currently have on the truck
                    if(pick_to_drop[current_item.delivery_id][j] <= item_time && is_pickedup[j] && !is_droppedoff[j]){
                        item_time = pick_to_drop[current_item.delivery_id][j];
                        next_item.delivery_id = j;
                        next_item.pickup = false;
                    }
                    
                    // no need to keep looping if we know we are going back to the same intersection
                    if (item_time == 0) {
                        break;
                    }
                    
                }
            }
            
            // if the current intersection is a dropoff
            else{
                for (int j = 0; j < deliveries.size(); j++) {

                    double potential_truck_weight = current_weight + deliveries[j].itemWeight;
                    // the conditions to travel to a pickup after the current dropoff
                    if (drop_to_pick[current_item.delivery_id][j] <= item_time && !is_pickedup[j] && potential_truck_weight < truck_capacity) {
                        item_time = drop_to_pick[current_item.delivery_id][j];
                        next_item.delivery_id = j;
                        next_item.pickup = true;
                    }
                    // the condition to drop off a package that we currently have on the truck
                    if(drop_to_drop[current_item.delivery_id][j] <= item_time && is_pickedup[j] && !is_droppedoff[j]){
                        item_time = drop_to_drop[current_item.delivery_id][j];
                        next_item.delivery_id = j;
                        next_item.pickup = false;
                    }
                    
                    if (item_time == 0) {
                        break;
                    }
                }
            }
            
            // updates the current path travel time
            current_travel_time += item_time;
            
            //adds the weight and sets the item to picked up if we are going to a pickup next
            if(next_item.pickup) {
                current_weight += deliveries[next_item.delivery_id].itemWeight;
                is_pickedup[next_item.delivery_id] = true;
            }
            
            //reduces the weight and sets the dropped off true 
            else {
                current_weight -= deliveries[next_item.delivery_id].itemWeight;
                is_droppedoff[next_item.delivery_id] = true;
            }

            current_item.delivery_id = next_item.delivery_id;
            current_item.pickup = next_item.pickup;
            
            //pushes back the current item into our vector of all deliveries for the current path
            delivery_order.push_back(current_item);
            

        }
        
        // if the current path is completed in less time than the best time so far
        if(current_travel_time < best_travel_time) {
            best_travel_time = current_travel_time;
            best_order = delivery_order;
            //std::cout << current_travel_time << std::endl;
        }
        
        is_pickedup.clear();
        is_droppedoff.clear();
    }
    
    //finds the depot closest to the start of our route
    int start_depot = find_closest_depot(depots, deliveries[best_order[0].delivery_id].pickUp);
    
    // the start of the route will be the closest depot to the best starting point
    CourierSubpath path;
    path.start_intersection = start_depot;
    path.end_intersection = deliveries[best_order[0].delivery_id].pickUp;
    path.subpath = find_path_between_intersections(path.start_intersection, path.end_intersection, turn_penalty);
    route.push_back(path);
    
    int size_of_path = best_order.size();
    // setting the path properties based on what type of intersection we are at
    for (int i = 0; i < size_of_path - 1; i++) {
        CourierSubpath path_1;
        if (best_order[i].pickup) {
            path_1.start_intersection = deliveries[best_order[i].delivery_id].pickUp;
            path_1.pickUp_indices.push_back(best_order[i].delivery_id);
        }
        
        else {
            path_1.start_intersection = deliveries[best_order[i].delivery_id].dropOff;
        }
        
        if(best_order[i+1].pickup) {
            path_1.end_intersection = deliveries[best_order[i+1].delivery_id].pickUp;
        }
        
        else {
            path_1.end_intersection = deliveries[best_order[i+1].delivery_id].dropOff;
        }
        
        path_1.subpath = find_path_between_intersections(path_1.start_intersection, path_1.end_intersection, turn_penalty);
        route.push_back(path_1); 
    }

    //finds the best depot based on the end of the path
    int end_depot = find_closest_depot(depots, deliveries[best_order[size_of_path - 1].delivery_id].dropOff);

    path.start_intersection = deliveries[best_order[size_of_path - 1].delivery_id].dropOff;
    path.end_intersection = end_depot;
    path.subpath = find_path_between_intersections(path.start_intersection, path.end_intersection, turn_penalty);
    route.push_back(path);
  
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);

    time_to_beat = compute_courier_path_travel_time(route, turn_penalty);
    int shuffle_1 = 1;
    int shuffle_2 = 2;
    
    //Do 2-opt
    while(wallClock.count() < TIME_LIMIT*0.8 && route.size() > 20){
        int cut_1 = std::rand()%route.size();
        int cut_2 = std::rand()%route.size();
        int cut_3 = std::rand()%route.size();
        
        if(cut_1 < cut_2 && cut_3 > cut_2 && cut_1 > 1 && cut_3 < route.size()-2 && std::abs(cut_1-cut_2) > 1 && std::abs(cut_2-cut_3) > 1){
            std::vector<CourierSubpath> potential_route = alpha_opt(route, cut_1, cut_2, cut_3, turn_penalty, depots);
        
            double potential_time = compute_courier_path_travel_time(potential_route, turn_penalty);
        
            if(ece297test::courier_path_is_legal_with_capacity_(deliveries, depots, potential_route, truck_capacity) && potential_time < time_to_beat){
                route = potential_route;
                time_to_beat = potential_time;
            }
        }
        
        currentTime = std::chrono::high_resolution_clock::now();
        wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
    }    

    while (wallClock.count() < TIME_LIMIT - 1 && shuffle_2 < route.size()) {
        if (route[shuffle_1].pickUp_indices.empty() && route[shuffle_2].pickUp_indices.empty()) {
            route = shuffle_drop(route, shuffle_1, shuffle_2, turn_penalty);
        } else if (!route[shuffle_1].pickUp_indices.empty() && !route[shuffle_2].pickUp_indices.empty()) {
            route = shuffle_pick(route, shuffle_1, shuffle_2, turn_penalty);
        }

        shuffle_1++;
        shuffle_2++;

        //Update the current time
        currentTime = std::chrono::high_resolution_clock::now();
        wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
    }
    
    std::cout << wallClock.count() << std::endl;
    return route;   
}

//finds the closest depot to a given intersection id
IntersectionIndex find_closest_depot (const std::vector<int> & depots, IntersectionIndex current_intersection) {
    int min_distance = 10000000;
    IntersectionIndex closest_depot = 0;
    for (int i = 0; i < depots.size(); i++) {
        std::pair<LatLon, LatLon> points (getIntersectionPosition(current_intersection), getIntersectionPosition(depots[i]));
        double potential = find_distance_between_two_points(points);
        
        if(potential < min_distance) {
            closest_depot = depots[i];
            min_distance = potential;
        }
    }
    return closest_depot;
}

bool all_dropped_off() {
    for (int i = 0; i < is_droppedoff.size(); i++) {
        if(is_droppedoff[i] == false) {
            return false;
        }
    }
    return true;
}