#include "m3.h"
#include "m2.h"
#include "m1.h"
#include "GlobalVariables.h"
#include <vector>
#include <bits/stdc++.h>
#include "search.h"

std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> walk_drive_path;

// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0.  The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given turn_penalty (in seconds) per turn implied by the path.  If there is
// no turn, then there is no penalty. Note that whenever the street id changes
// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, 
                                const double turn_penalty){
    
    if(path.empty()){
        return 0.0;
    }
    
    //Sets initial contions and grabs first street ID
    double travel_time = 0.0;
    int prev_streetID = getInfoStreetSegment(path[0]).streetID;
    
    for(int i = 0; i < path.size(); i++){
        //Add travel time of the street
        travel_time += find_street_segment_travel_time(path[i]);
        
        InfoStreetSegment current = getInfoStreetSegment(path[i]);
        
        //Check if we turned or not
        if(current.streetID != prev_streetID){
            travel_time += turn_penalty;
            prev_streetID = current.streetID;
        }
    }
    
    return travel_time;
    
}


// Returns a path (route) between the start intersection and the end
// intersection, if one exists. This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds).  If no path exists, this routine
// returns an empty (size == 0) vector.  If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the end intersection.
std::vector<StreetSegmentIndex> find_path_between_intersections(
		        const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty){
    //global variable starts off empty
    shortest_path.clear();
    
    //modifies 
    a_star(intersect_id_start, intersect_id_end, turn_penalty);
    
    return shortest_path;  
}


// Returns the time required to "walk" along the path specified, in seconds.
// The path is given as a vector of street segment ids. The vector can be of
// size = 0, and in this case, it the function should return 0. The walking
// time is the sum of the length/<walking_speed> for each street segment, plus
// the given turn penalty, in seconds, per turn implied by the path. If there
// is no turn, then there is no penalty.  As mentioned above, going from Bloor
// Street West to Bloor street East is considered a turn
double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path, 
                                const double walking_speed, 
                                const double turn_penalty){
    
    double walk_path_time = 0;
    for(int i = 0; i < path.size(); i++) {
        //the start intersection can't have a turn penalty so when i!=0 we do this
        if (i != 0) {
            //checks for turn penalty
            if (getInfoStreetSegment(path[i - 1]).streetID != getInfoStreetSegment(path[i]).streetID) {
                walk_path_time += turn_penalty;
            }
        }
        //calculates time to walk
        walk_path_time += street_segment_length[path[i]] / walking_speed;
        
    }
    
    return walk_path_time; 
}


// This is an "uber pool"-like function. The idea is to minimize driving travel
// time by walking to a pick-up intersection (within walking_time_limit secs)
// from start_intersection while waiting for the car to arrive.  While walking,
// you can ignore speed limits of streets and just account for given
// walking_speed [m/sec]. However, pedestrians should also wait for traffic
// lights, so turn_penalty applies to whether you are driving or walking.
// Walking in the opposite direction of one-way streets is fine. Driving is
// NOT!  The routine returns a pair of vectors of street segment ids. The first
// vector is the walking path starting at start_intersection and ending at the
// pick-up intersection. The second vector is the driving path from pick-up
// intersection to end_interserction.  Note that a start_intersection can be a
// pick-up intersection. If this happens, the first vector should be empty
// (size = 0).  If there is no driving path found, both returned vectors should
// be empty (size = 0). 
// If the end_intersection turns out to be within the walking time limit, 
// you can choose what to return, but you should not crash. If your user 
// interface can call this function for that case, the UI should handle
// whatever you return in that case.
std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> 
                                              find_path_with_walk_to_pick_up(const IntersectionIndex start_intersection, 
                                                                             const IntersectionIndex end_intersection,
                                                                             const double turn_penalty,
                                                                             const double walking_speed, 
                                                                             const double walking_time_limit){
    //makes global paths clear to begin with
    walk_path.clear();
    shortest_path.clear();
    
    //stores all the valid intersections
    std::vector<IntersectionIndex> walkable_intersections = {};
    //stores all the intersections that MAY be valid
    std::multimap<double, IntersectionIndex> possible_walk_intersections = {};
    
    //loops through all the intersections
    for (int i = 0; i < getNumIntersections(); i++) {
        //finds distance from the start intersect to each of the intersections
        std::pair<LatLon, LatLon> start_to_i (getIntersectionPosition(start_intersection), getIntersectionPosition(i));
        double dist = find_distance_between_two_points(start_to_i);
        
        //estimates the time to travel from start to the particular intersection
        double estimate_time = dist * 1/walking_speed;
        
        //the estimate time is always an underestimate so we will never miss any intersections
        if(estimate_time<= walking_time_limit) {            
            possible_walk_intersections.insert({dist,i});
        }  
    }
    
    //finds iterator to the end of multimap (the intersection furthest away from start)
    auto it = possible_walk_intersections.end();
    bool found = false;
    //advances iterator to the end of the map
    while(!found && it != possible_walk_intersections.begin()){
        //starts the loop by moving iterator backwards in the multimap
        it--;
        //gets the IntersectionIndex from the iterator
        int intersection = it->second;
        found = dijkstra_walk(start_intersection, intersection, turn_penalty, walking_speed, false);

        if(!found){
            clear_node_graph();
        }  
    }
    
    //finding if the intersections that are found are within the given walking time limit
    for(auto iterator = possible_walk_intersections.begin(); iterator != possible_walk_intersections.end(); iterator++){
        //gets the IntersectionIndex 
        int intersection = iterator->second;
        if(node_graph[intersection].cost <= walking_time_limit){
            walkable_intersections.push_back(intersection);
        }
    }
    
    clear_node_graph();
    
    
    //if no intersections are found to be walkable in the allocated time limit
    if (walkable_intersections.empty()) {
        //does the regular find path algorithm with just walk
        find_path_between_intersections(start_intersection, end_intersection, turn_penalty);
        walk_drive_path = std::make_pair(walk_path, shortest_path);
        
        return walk_drive_path;
    }
    
    //initial drive time is a very high value since we look for the lowest times
    double drive_time = 10000000000.000;
    int start_drive_intersection = 0;
    std::vector<StreetSegmentIndex> drive_path = {};
    
    
    //does a star for all the walkable intersections that are found and finds the
    // one that comes out with the shortest path travel time
    for (int i = 0; i < walkable_intersections.size(); i++) {
        // if a driving path is found
        bool path_exists = a_star(walkable_intersections[i], end_intersection, turn_penalty);
        //computes the time of the path that is found
        double time = compute_path_travel_time(shortest_path, turn_penalty);
        
        //checks if a new shortest path has been found from the end to a potential walking intersection
        if((time < drive_time) && path_exists) {
            drive_time = time;
            start_drive_intersection = walkable_intersections[i];
            //copies the new best path into drive_path
            drive_path.resize(shortest_path.size());
            std::copy(shortest_path.begin(), shortest_path.end(), drive_path.begin());
        }
        shortest_path.clear();
    }
   
    //finds the path from the start to the most optimal ending intersection and does trace_back since true is passed in
    dijkstra_walk(start_intersection, start_drive_intersection, turn_penalty, walking_speed, true);

    walk_drive_path = std::make_pair(walk_path, drive_path);
    
    return walk_drive_path;
}