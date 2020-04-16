#include "search.h"

std::vector<StreetSegmentIndex> shortest_path;
std::vector<StreetSegmentIndex> walk_path;

//A* algorithm
bool a_star(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty){
    
    //if the same 2 intersections are selected as start and end
    if(intersect_id_start == intersect_id_end){
        return true;
    }
    
    std::priority_queue<waveElements, std::vector<waveElements>, compare> waveFront;
    
    //Starting intersection data structure initialization
    int start_int = intersect_id_start;
   
    //finds distance from the start to the end
    std::pair<LatLon, LatLon> start_to_end(getIntersectionPosition(intersect_id_start), getIntersectionPosition(intersect_id_end));
    double dist = find_distance_between_two_points(start_to_end);
    
    waveElements start(&node_graph[start_int], -1, 0, dist, 0);
    waveFront.push(start);

    //Continue to search for path unitil the wavefront is empty
    while(!waveFront.empty()){
        //Save the top element in the waveFront
        waveElements wave = waveFront.top();
        
        //Remove the first element in the wave
        waveFront.pop();
        
        //Gets the current node from the wave
        node *current_node = wave.current; 
        
        //if a better travel time to a particular node has been found
        if(wave.travel_time <= current_node->cost){
            current_node->reachingEdge = wave.connectingEdge;
            current_node->cost = wave.travel_time;
            
            //if destination is reached
            if(current_node->node_ID == intersect_id_end){
                //trace back the path (false means its drive only)
                trace_back(current_node, false);
                clear_node_graph();
                //clears remaining wave front
                while(!waveFront.empty()){
                    waveFront.pop();
                }
                return true;
            }
            
            
            //Goes through the outgoing edges of the current node and 
            //adds them to the wave front 
            for(int i = 0; i < current_node->outEdges; i++){
                int edge = street_segments_of_intersection[current_node->node_ID][i];\
                
                //don't backtrack to the reaching edge
                if(edge == current_node->reachingEdge){
                    continue;
                }

                InfoStreetSegment seg_info = street_segment_info[edge];
                IntersectionIndex new_node_ID;
                double time = 0.0;
                
                //checks which intersection to assign the new node based on to and from of the segment
                if(seg_info.to == current_node->node_ID) {
                    new_node_ID = seg_info.from;
                }
                
                else {
                    new_node_ID = seg_info.to;
                }
                
                //if taking a path involves turning
                if(current_node->reachingEdge != -1){
                    if(street_segment_info[current_node->reachingEdge].streetID != seg_info.streetID){
                        time += turn_penalty;
                    }
                }
                
                //adds on the time taken to travel a particular segment
                time += find_street_segment_travel_time(edge);
                
                //finds distance from the current node to the end destination
                std::pair<LatLon, LatLon> new_dist(getIntersectionPosition(new_node_ID), getIntersectionPosition(intersect_id_end));
                double new_distance = find_distance_between_two_points(new_dist);
                
                //used to sort wavefront
                double new_heuristic = wave.travel_time + time + (new_distance * 1/max_speed_limit);

                //legality check for oneways and making for corner case where a node is both the to and from
                if((((seg_info.oneWay && current_node->node_ID == seg_info.from) || !seg_info.oneWay) && !check_same_position(new_node_ID, current_node->node_ID))) {
                    if((node_graph[new_node_ID].reachingEdge == -1 || (time + wave.travel_time) <= node_graph[new_node_ID].cost) && new_node_ID != start_int) {

                        node_graph[new_node_ID].outEdges = getIntersectionStreetSegmentCount(new_node_ID);
                        node_graph[new_node_ID].reachingEdge = edge;
                        node_graph[new_node_ID].cost = time + wave.travel_time;
                    }

                    waveElements new_wave(&node_graph[new_node_ID], edge, time + wave.travel_time, new_distance, new_heuristic);
                    waveFront.push(new_wave);
                }
                     
            }
        }
    }
    
    clear_node_graph();
    return  false;
}

//dijkstra algo for the walking
bool dijkstra_walk(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty, const double walking_speed, bool trace){
    
     //if the same 2 intersections are selected as start and end
    if(intersect_id_start == intersect_id_end){
        return true;
    }
    
    std::priority_queue<waveElements, std::vector<waveElements>, compare> waveFront;
    
    //Starting intersection data structure initialization
    int start_int = intersect_id_start;
   
    //finds distance from the start to the end
    std::pair<LatLon, LatLon> start_to_end(getIntersectionPosition(intersect_id_start), getIntersectionPosition(intersect_id_end));
    double dist = find_distance_between_two_points(start_to_end);
    
    waveElements start(&node_graph[start_int], -1, 0, dist, 0);
    waveFront.push(start);

    //Continue to search for path unitil the wavefront is empty
    while(!waveFront.empty()){
        //Save the top element in the waveFront
        waveElements wave = waveFront.top();
        
        //Remove the first element in the wave
        waveFront.pop();
        
        //Gets the current node from the wave
        node *current_node = wave.current; 
        
        //if a better travel time to a particular node has been found
        if(wave.travel_time <= current_node->cost){
            current_node->reachingEdge = wave.connectingEdge;
            current_node->cost = wave.travel_time;
            
            //if arrived at destination node
            if(current_node->node_ID == intersect_id_end){
                
                //only traces when true is passed in, only traces back the best walk path
                if(trace) {
                    trace_back(current_node, true);
                    clear_node_graph();
                }

                //clear the remaining wavefront
                while (!waveFront.empty()) {
                    waveFront.pop();
                }

                return true;
            }
            
            
            //Goes through the outgoing edges of the current node and 
            //adds them to the wave front 
            for(int i = 0; i < current_node->outEdges; i++){
                int edge = street_segments_of_intersection[current_node->node_ID][i];\
                
                //don't backtrack to the reaching edge
                if(edge == current_node->reachingEdge){
                    continue;
                }

                InfoStreetSegment seg_info = street_segment_info[edge];
                IntersectionIndex new_node_ID;
                double time = 0.0;
                
                if(seg_info.to == current_node->node_ID) {
                    new_node_ID = seg_info.from;
                }
                
                else {
                    new_node_ID = seg_info.to;
                }
                
                //accounts for turn penalty
                if(current_node->reachingEdge != -1){
                    if(street_segment_info[current_node->reachingEdge].streetID != seg_info.streetID){
                        time += turn_penalty;
                    }
                }
                
                //time it takes for the person to walk down a particular street segment
                time += street_segment_length[edge] / walking_speed;
                
                //finds the distance from the current node to the end destination
                std::pair<LatLon, LatLon> new_dist(getIntersectionPosition(new_node_ID), getIntersectionPosition(intersect_id_end));
                double new_distance = find_distance_between_two_points(new_dist);
                
                //used to sort the wavefront (for dijkstra it is only time dependent)
                double new_heuristic = wave.travel_time + time;
                
                //makes sure it is not a case where the to and from of a segment are the same node
                if(seg_info.from != seg_info.to){
                    //check to update best time
                    if(((node_graph[new_node_ID].reachingEdge == -1 || (wave.travel_time + time) <= node_graph[new_node_ID].cost) && new_node_ID != start_int) && !check_same_position(new_node_ID, current_node->node_ID)) {

                        node_graph[new_node_ID].outEdges = getIntersectionStreetSegmentCount(new_node_ID);
                        node_graph[new_node_ID].reachingEdge = edge;
                        node_graph[new_node_ID].cost = wave.travel_time + time;
                    }

                    waveElements new_wave(&node_graph[new_node_ID], edge, time + wave.travel_time, new_distance, new_heuristic);
                    waveFront.push(new_wave);
                }
                     
            }
        }
    }
    
    return  false;
}

//dijkstra algo for the walking
bool dijkstra(const IntersectionIndex intersect_id_start, 
                        const IntersectionIndex intersect_id_end,
                        const double turn_penalty, bool trace){
    
    //if the same 2 intersections are selected as start and end
    if(intersect_id_start == intersect_id_end){
        return true;
    }
    
    std::priority_queue<waveElements, std::vector<waveElements>, compare> waveFront;
    
    //Starting intersection data structure initialization
    int start_int = intersect_id_start;
   
    
    waveElements start(&node_graph[start_int], -1, 0, 0, 0);
    waveFront.push(start);

    //Continue to search for path unitil the wavefront is empty
    while(!waveFront.empty()){
        //Save the top element in the waveFront
        waveElements wave = waveFront.top();
        
        //Remove the first element in the wave
        waveFront.pop();
        
        //Gets the current node from the wave
        node *current_node = wave.current; 
        
        //if a better travel time to a particular node has been found
        if(wave.travel_time <= current_node->cost){
            current_node->reachingEdge = wave.connectingEdge;
            current_node->cost = wave.travel_time;
            
            //if destination is reached
            if(current_node->node_ID == intersect_id_end){
                //only traces when true is passed in, only traces back the best walk path
                if(trace) {
                    trace_back(current_node, true);
                    clear_node_graph();
                }

                //clear the remaining wavefront
                while (!waveFront.empty()) {
                    waveFront.pop();
                }

                return true;
            }
            
            
            //Goes through the outgoing edges of the current node and 
            //adds them to the wave front 
            for(int i = 0; i < current_node->outEdges; i++){
                int edge = street_segments_of_intersection[current_node->node_ID][i];\
                
                //don't backtrack to the reaching edge
                if(edge == current_node->reachingEdge){
                    continue;
                }

                InfoStreetSegment seg_info = street_segment_info[edge];
                IntersectionIndex new_node_ID;
                double time = 0.0;
                
                //checks which intersection to assign the new node based on to and from of the segment
                if(seg_info.to == current_node->node_ID) {
                    new_node_ID = seg_info.from;
                }
                
                else {
                    new_node_ID = seg_info.to;
                }
                
                //if taking a path involves turning
                if(current_node->reachingEdge != -1){
                    if(street_segment_info[current_node->reachingEdge].streetID != seg_info.streetID){
                        time += turn_penalty;
                    }
                }
                
                //adds on the time taken to travel a particular segment
                time += find_street_segment_travel_time(edge);
                
                //used to sort wavefront
                double new_heuristic = wave.travel_time + time;
                
                //legality check for oneways and making for corner case where a node is both the to and from
                if((((seg_info.oneWay && current_node->node_ID == seg_info.from) || !seg_info.oneWay) && seg_info.from != seg_info.to) && !check_same_position(new_node_ID, current_node->node_ID)){
                    //check to update best time
                    if((node_graph[new_node_ID].reachingEdge == -1 || (time + wave.travel_time) <= node_graph[new_node_ID].cost) && new_node_ID != start_int) {

                        node_graph[new_node_ID].outEdges = getIntersectionStreetSegmentCount(new_node_ID);
                        node_graph[new_node_ID].reachingEdge = edge;
                        node_graph[new_node_ID].cost = time + wave.travel_time;
                    }

                    waveElements new_wave(&node_graph[new_node_ID], edge, time + wave.travel_time, 0, new_heuristic);
                    waveFront.push(new_wave);
                }
                     
            }
        }
    }
    
    return  false;
}


bool dijkstra_set_time(const IntersectionIndex intersect_id_start, 
                        std::vector<IntersectionIndex> end_intersections,
                        const double turn_penalty, bool trace){

    std::priority_queue<waveElements, std::vector<waveElements>, compare> waveFront;

    //Starting intersection data structure initialization
    int start_int = intersect_id_start;

    waveElements start(&node_graph[start_int], -1, 0, 0, 0);
    waveFront.push(start);

    //Continue to search for path unitil the wavefront is empty
    while (!waveFront.empty()) {
        //Save the top element in the waveFront
        waveElements wave = waveFront.top();

        //Remove the first element in the wave
        waveFront.pop();

        //Gets the current node from the wave
        node *current_node = wave.current;

        //if a better travel time to a particular node has been found
        if (wave.travel_time <= current_node->cost) {
            current_node->reachingEdge = wave.connectingEdge;
            current_node->cost = wave.travel_time;

            //if all the nodes have been reached
            if (is_reached(end_intersections, start_int)) {
                //only traces when true is passed in, only traces back the best walk path
                if (trace) {
                    trace_back(current_node, false);
                    clear_node_graph();
                }

                //clear the remaining wavefront
                while (!waveFront.empty()) {
                    waveFront.pop();
                }

                return true;
            }

            //Goes through the outgoing edges of the current node and 
            //adds them to the wave front 
            for (int i = 0; i < current_node->outEdges; i++) {
                int edge = street_segments_of_intersection[current_node->node_ID][i];\
                
                //don't backtrack to the reaching edge
                if (edge == current_node->reachingEdge) {
                    continue;
                }

                InfoStreetSegment seg_info = street_segment_info[edge];
                IntersectionIndex new_node_ID;
                double time = 0.0;

                //checks which intersection to assign the new node based on to and from of the segment
                if (seg_info.to == current_node->node_ID) {
                    new_node_ID = seg_info.from;
                }
                else {
                    new_node_ID = seg_info.to;
                }

                //if taking a path involves turning
                if (current_node->reachingEdge != -1) {
                    if (street_segment_info[current_node->reachingEdge].streetID != seg_info.streetID) {
                        time += turn_penalty;
                    }
                }

                //adds on the time taken to travel a particular segment
                time += find_street_segment_travel_time(edge);

                //used to sort wavefront
                double new_heuristic = wave.travel_time + time;

                //legality check for oneways and making for corner case where a node is both the to and from
                if ((((seg_info.oneWay && current_node->node_ID == seg_info.from) || !seg_info.oneWay) && seg_info.from != seg_info.to) && !check_same_position(new_node_ID, current_node->node_ID)) {
                    //check to update best time
                    if ((node_graph[new_node_ID].reachingEdge == -1 || (time + wave.travel_time) <= node_graph[new_node_ID].cost) && new_node_ID != start_int) {

                        node_graph[new_node_ID].outEdges = getIntersectionStreetSegmentCount(new_node_ID);
                        node_graph[new_node_ID].reachingEdge = edge;
                        node_graph[new_node_ID].cost = time + wave.travel_time;
                    }

                    waveElements new_wave(&node_graph[new_node_ID], edge, time + wave.travel_time, 0, new_heuristic);
                    waveFront.push(new_wave);
                }

            }
        }
    }

    return false;
}

bool dijkstra_local_graph(const IntersectionIndex intersect_id_start,
        std::vector<IntersectionIndex> end_intersections,
        const double turn_penalty, std::vector<node> & local_node_graph) {

    local_node_graph.resize(getNumIntersections());
    for (int i = 0; i < getNumIntersections(); i++) {
        //high number to start since we want the minimal cost
        node new_intersection(i, SMALL, -1, getIntersectionStreetSegmentCount(i));
        local_node_graph[i] = new_intersection;
    }
    
    
    std::priority_queue<waveElements, std::vector<waveElements>, compare> waveFront;

    //Starting intersection data structure initialization
    int start_int = intersect_id_start;

    waveElements start(&local_node_graph[start_int], -1, 0, 0, 0);
    waveFront.push(start);

    //Continue to search for path unitil the wavefront is empty
    while (!waveFront.empty()) {
        //Save the top element in the waveFront
        waveElements wave = waveFront.top();

        //Remove the first element in the wave
        waveFront.pop();

        //Gets the current node from the wave
        node *current_node = wave.current;

        //if a better travel time to a particular node has been found
        if (wave.travel_time <= current_node->cost) {
            current_node->reachingEdge = wave.connectingEdge;
            current_node->cost = wave.travel_time;

            //if all the nodes have been reached
            if (is_reached_local(end_intersections, start_int, local_node_graph)) {

                //clear the remaining wavefront
                while (!waveFront.empty()) {
                    waveFront.pop();
                }

                return true;
            }

            //Goes through the outgoing edges of the current node and 
            //adds them to the wave front 
            for (int i = 0; i < current_node->outEdges; i++) {
                int edge = street_segments_of_intersection[current_node->node_ID][i];\
                
                //don't backtrack to the reaching edge
                if (edge == current_node->reachingEdge) {
                    continue;
                }

                InfoStreetSegment seg_info = street_segment_info[edge];
                IntersectionIndex new_node_ID;
                double time = 0.0;

                //checks which intersection to assign the new node based on to and from of the segment
                if (seg_info.to == current_node->node_ID) {
                    new_node_ID = seg_info.from;
                } else {
                    new_node_ID = seg_info.to;
                }

                //if taking a path involves turning
                if (current_node->reachingEdge != -1) {
                    if (street_segment_info[current_node->reachingEdge].streetID != seg_info.streetID) {
                        time += turn_penalty;
                    }
                }

                //adds on the time taken to travel a particular segment
                time += find_street_segment_travel_time(edge);

                //used to sort wavefront
                double new_heuristic = wave.travel_time + time;

                //legality check for oneways and making for corner case where a node is both the to and from
                if ((((seg_info.oneWay && current_node->node_ID == seg_info.from) || !seg_info.oneWay) && seg_info.from != seg_info.to) && !check_same_position(new_node_ID, current_node->node_ID)) {
                    //check to update best time
                    if ((local_node_graph[new_node_ID].reachingEdge == -1 || (time + wave.travel_time) <= local_node_graph[new_node_ID].cost) && new_node_ID != start_int) {

                        local_node_graph[new_node_ID].outEdges = getIntersectionStreetSegmentCount(new_node_ID);
                        local_node_graph[new_node_ID].reachingEdge = edge;
                        local_node_graph[new_node_ID].cost = time + wave.travel_time;
                    }

                    waveElements new_wave(&local_node_graph[new_node_ID], edge, time + wave.travel_time, 0, new_heuristic);
                    waveFront.push(new_wave);
                }

            }
        }
    }

    return false;
}


//follows back to create a vector of street segments used in the best path
void trace_back (node *dest_node, bool walk) {
    std::list<StreetSegmentIndex> traceback;
    //grabs the first reaching edge
    StreetSegmentIndex prev_edge = dest_node->reachingEdge;
    node* current_node = dest_node;
    
    //continues to traverse the edges until it hits the source node
    while(prev_edge!= -1) {
        traceback.push_front(prev_edge);
        
        //checks for what node is on the other side of the street segment
        //based on the to and from characteristics of the segment and updates
        //current node accordingly
        if(street_segment_info[prev_edge].to == current_node->node_ID) {
            current_node = &node_graph[street_segment_info[prev_edge].from];
        }
        
        else {
            current_node = &node_graph[street_segment_info[prev_edge].to];
        }
        
        //updates previous edge to reflect the new node
        prev_edge = current_node->reachingEdge;
    }
    
    //if traceback is used for finding the walk path, updates walk_path global
    if (walk) {
        walk_path.resize(traceback.size());
        std::copy(traceback.begin(), traceback.end(), walk_path.begin());
    }
    
    //otherwise updates the shortest_path global used for drive
    else {
        shortest_path.resize(traceback.size());
        std::copy(traceback.begin(), traceback.end(), shortest_path.begin());
    }
    
    traceback.clear();
}

//reinitializes the members of the node_graph
void clear_node_graph() {
    for(int i = 0; i < getNumIntersections(); i++) {
        //high number to start since we want the minimal cost
        node_graph[i].cost = 10000000;
        node_graph[i].reachingEdge = -1;
    }
}

//checks if the 2 intersections have different ids but are located at the same LatLon
bool check_same_position (IntersectionIndex new_node_id, IntersectionIndex current_node_id) {
    if(new_node_id != current_node_id) {    
        if(getIntersectionPosition(new_node_id).lat() == getIntersectionPosition(current_node_id).lat()) {
            if(getIntersectionPosition(new_node_id).lon() == getIntersectionPosition(current_node_id).lon()) {
                return true;
            }
        }
    }
    return false;
}

bool is_reached (std::vector<IntersectionIndex> end_intersections, int start) {
    for (int i = 0; i < end_intersections.size(); i++) {
        if (node_graph[end_intersections[i]].reachingEdge == -1 && end_intersections[i] != start) {
            return false;
        }
    }
    return true;
}

bool is_reached_local (std::vector<IntersectionIndex> end_intersections, int start, std::vector<node> & local_node_graph) {
    for (int i = 0; i < end_intersections.size(); i++) {
        if (local_node_graph[end_intersections[i]].reachingEdge == -1 && end_intersections[i] != start) {
            return false;
        }
    }
    return true;
}

void clear_local_graph (std::vector <node> & local_node_graph) {
    local_node_graph.resize(getNumIntersections());
    
    for (int i = 0; i < getNumIntersections(); i++) {
        //high number to start since we want the minimal cost
        local_node_graph[i].cost = 10000000;
        local_node_graph[i].reachingEdge = -1;
    }
}