#include "optimization.h"


//finds the closest depot to a given intersection id
IntersectionIndex find_closest_depot (const std::vector<int> & depots, IntersectionIndex current_intersection, double turn_penalty) {
    int max_distance = 0;
    
    IntersectionIndex furthest_depot = 0;
    for (int i = 0; i < depots.size(); i++) {
        std::pair<LatLon, LatLon> points (getIntersectionPosition(current_intersection), getIntersectionPosition(depots[i]));
        double potential = find_distance_between_two_points(points);
        
        if(potential > max_distance) {
            furthest_depot = depots[i];
            max_distance = potential;
        }
    }
    
    dijkstra(current_intersection, furthest_depot, turn_penalty, false);
    
    IntersectionIndex closest_depot = 0;
    double fastest_time = 10000000000000;
    for (int i = 0; i < depots.size(); i++) {
        if (node_graph[depots[i]].cost <= fastest_time) {
            fastest_time = node_graph[depots[i]].cost;
            closest_depot = depots[i];
        }
    }
    clear_node_graph();
    return closest_depot;
}

//Compute the courier path travel time
double compute_courier_path_travel_time(std::vector<CourierSubpath>& courier_route, const float turn_penalty) {
    double total_travel_time = 0.0; 
    for (size_t i = 0; i < courier_route.size(); i++){
        total_travel_time += compute_path_travel_time(courier_route[i].subpath, turn_penalty);
    }
    
    return total_travel_time;
}

std::vector<CourierSubpath> shuffle_drop(std::vector<CourierSubpath>& original_path, int shuffle_pos1, int shuffle_pos2, const double turn_penalty){
    std::vector<CourierSubpath> potential_route = original_path;
    
    
    
    //Setting the previous path 
    potential_route[shuffle_pos1 - 1].subpath = find_path_between_intersections(original_path[shuffle_pos1 - 1].start_intersection, original_path[shuffle_pos2].start_intersection, turn_penalty);
    potential_route[shuffle_pos1 - 1].end_intersection = original_path[shuffle_pos2].start_intersection;
    
    //Setting the shuffle 1 path 
    potential_route[shuffle_pos1].subpath = find_path_between_intersections(original_path[shuffle_pos2].start_intersection, original_path[shuffle_pos1].start_intersection, turn_penalty);
    potential_route[shuffle_pos1].start_intersection = original_path[shuffle_pos2].start_intersection;
    potential_route[shuffle_pos1].end_intersection = original_path[shuffle_pos1].start_intersection;
    
    //Setting the shuffle 2 path
    potential_route[shuffle_pos2].subpath = find_path_between_intersections(original_path[shuffle_pos1].start_intersection, original_path[shuffle_pos2].end_intersection, turn_penalty);
    potential_route[shuffle_pos2].start_intersection = original_path[shuffle_pos1].start_intersection;
    
    double potential_time = compute_courier_path_travel_time(potential_route, turn_penalty);
    if(potential_time < time_to_beat){
        time_to_beat = potential_time;
        return potential_route;
    }        
    else{
        return original_path;
    } 
}

std::vector<CourierSubpath> shuffle_pick(std::vector<CourierSubpath>& original_path, int shuffle_pos1, int shuffle_pos2, const double turn_penalty){
    std::vector<CourierSubpath> potential_route = original_path;
    
    //Setting the previous path 
    potential_route[shuffle_pos1 - 1].subpath = find_path_between_intersections(original_path[shuffle_pos1 - 1].start_intersection, original_path[shuffle_pos2].start_intersection, turn_penalty);
    potential_route[shuffle_pos1 - 1].end_intersection = original_path[shuffle_pos2].start_intersection;
    
    //Setting the shuffle 1 path 
    potential_route[shuffle_pos1].subpath = find_path_between_intersections(original_path[shuffle_pos2].start_intersection, original_path[shuffle_pos1].start_intersection, turn_penalty);
    potential_route[shuffle_pos1].start_intersection = original_path[shuffle_pos2].start_intersection;
    potential_route[shuffle_pos1].end_intersection = original_path[shuffle_pos1].start_intersection;
    potential_route[shuffle_pos1].pickUp_indices = original_path[shuffle_pos2].pickUp_indices;
    
    //Setting the shuffle 2 path
    potential_route[shuffle_pos2].subpath = find_path_between_intersections(original_path[shuffle_pos1].start_intersection, original_path[shuffle_pos2].end_intersection, turn_penalty);
    potential_route[shuffle_pos2].start_intersection = original_path[shuffle_pos1].start_intersection;
    potential_route[shuffle_pos2].pickUp_indices = original_path[shuffle_pos1].pickUp_indices;
    
    
    double potential_time = compute_courier_path_travel_time(potential_route, turn_penalty);
    if(potential_time < time_to_beat){
        time_to_beat = potential_time;
        return potential_route;
    }        
    else{
        return original_path;
    } 
}

std::vector<CourierSubpath> alpha_opt(std::vector<CourierSubpath>& original_path, int cut_1, int cut_2, int cut_3, const double turn_penalty, const std::vector<int>& depots){
    std::vector<CourierSubpath> potential_path = {};
    
    //Set up the 3 sections of the cut
    std::vector<CourierSubpath> segment_1 = {};
    std::vector<CourierSubpath> segment_2 = {};
    std::vector<CourierSubpath> segment_3 = {};
    std::vector<CourierSubpath> segment_4 = {};
    for(int i = 1; i < cut_1; i++){
        segment_1.push_back(original_path[i]);
    }
    for(int i = cut_1 + 1; i < cut_2; i++){
        segment_2.push_back(original_path[i]);
    }
    for(int i = cut_2 + 1; i < cut_3; i++){
        segment_3.push_back(original_path[i]);
    }
    for(int i = cut_3 + 1; i < original_path.size() - 1; i++){
        segment_4.push_back(original_path[i]);
    }

    int reconnect = std::rand()%23;
    
    // 1243
    if(reconnect == 0){
        CourierSubpath connect_12;
        connect_12.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_12.end_intersection = segment_2[0].start_intersection;
        connect_12.subpath = find_path_between_intersections(connect_12.start_intersection, connect_12.end_intersection,turn_penalty);
        connect_12.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_24;
        connect_24.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_24.end_intersection = segment_4[0].start_intersection;
        connect_24.subpath = find_path_between_intersections(connect_24.start_intersection, connect_24.end_intersection,turn_penalty);
        connect_24.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_34;
        connect_34.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_34.end_intersection = segment_3[0].start_intersection;
        connect_34.subpath = find_path_between_intersections(connect_34.start_intersection, connect_34.end_intersection,turn_penalty);
        connect_34.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        potential_path.push_back(original_path[0]);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_12);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_24);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_34);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //1324
    else if(reconnect == 1){
        CourierSubpath connect_13;
        connect_13.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_13.end_intersection = segment_3[0].start_intersection;
        connect_13.subpath = find_path_between_intersections(connect_13.start_intersection, connect_13.end_intersection,turn_penalty);
        connect_13.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_32;
        connect_32.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_32.end_intersection = segment_2[0].start_intersection;
        connect_32.subpath = find_path_between_intersections(connect_32.start_intersection, connect_32.end_intersection,turn_penalty);
        connect_32.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_24;
        connect_24.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_24.end_intersection = segment_4[0].start_intersection;
        connect_24.subpath = find_path_between_intersections(connect_24.start_intersection, connect_24.end_intersection,turn_penalty);
        connect_24.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        potential_path.push_back(original_path[0]);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_13);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_32);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_24);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(to_depot);
    }
    //1342
    else if(reconnect == 2){
        CourierSubpath connect_13;
        connect_13.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_13.end_intersection = segment_3[0].start_intersection;
        connect_13.subpath = find_path_between_intersections(connect_13.start_intersection, connect_13.end_intersection,turn_penalty);
        connect_13.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_34;
        connect_34.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_34.end_intersection = segment_4[0].start_intersection;
        connect_34.subpath = find_path_between_intersections(connect_34.start_intersection, connect_34.end_intersection,turn_penalty);
        connect_34.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_42;
        connect_42.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_42.end_intersection = segment_2[0].start_intersection;
        connect_42.subpath = find_path_between_intersections(connect_42.start_intersection, connect_42.end_intersection,turn_penalty);
        connect_42.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        potential_path.push_back(original_path[0]);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_13);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_34);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_42);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //1423
    else if(reconnect == 3){
        CourierSubpath connect_14;
        connect_14.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_14.end_intersection = segment_4[0].start_intersection;
        connect_14.subpath = find_path_between_intersections(connect_14.start_intersection, connect_14.end_intersection,turn_penalty);
        connect_14.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_42;
        connect_42.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_42.end_intersection = segment_2[0].start_intersection;
        connect_42.subpath = find_path_between_intersections(connect_42.start_intersection, connect_42.end_intersection,turn_penalty);
        connect_42.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath connect_23;
        connect_23.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_23.end_intersection = segment_3[0].start_intersection;
        connect_23.subpath = find_path_between_intersections(connect_23.start_intersection, connect_23.end_intersection,turn_penalty);
        connect_23.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        potential_path.push_back(original_path[0]);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_14);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_42);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_23);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //1432
    else if(reconnect == 4){
        CourierSubpath connect_14;
        connect_14.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_14.end_intersection = segment_4[0].start_intersection;
        connect_14.subpath = find_path_between_intersections(connect_14.start_intersection, connect_14.end_intersection,turn_penalty);
        connect_14.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_43;
        connect_43.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_43.end_intersection = segment_3[0].start_intersection;
        connect_43.subpath = find_path_between_intersections(connect_43.start_intersection, connect_43.end_intersection,turn_penalty);
        connect_43.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath connect_32;
        connect_32.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_32.end_intersection = segment_2[0].start_intersection;
        connect_32.subpath = find_path_between_intersections(connect_32.start_intersection, connect_32.end_intersection,turn_penalty);
        connect_32.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        potential_path.push_back(original_path[0]);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_14);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_43);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_32);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2134
    else if(reconnect == 5){
        CourierSubpath connect_21;
        connect_21.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_21.end_intersection = segment_1[0].start_intersection;
        connect_21.subpath = find_path_between_intersections(connect_21.start_intersection, connect_21.end_intersection,turn_penalty);
        connect_21.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_13;
        connect_13.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_13.end_intersection = segment_3[0].start_intersection;
        connect_13.subpath = find_path_between_intersections(connect_13.start_intersection, connect_13.end_intersection,turn_penalty);
        connect_13.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_34;
        connect_34.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_34.end_intersection = segment_4[0].start_intersection;
        connect_34.subpath = find_path_between_intersections(connect_34.start_intersection, connect_34.end_intersection,turn_penalty);
        connect_34.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_21);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_13);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_34);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2143
    else if(reconnect == 6){
        CourierSubpath connect_21;
        connect_21.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_21.end_intersection = segment_1[0].start_intersection;
        connect_21.subpath = find_path_between_intersections(connect_21.start_intersection, connect_21.end_intersection,turn_penalty);
        connect_21.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_14;
        connect_14.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_14.end_intersection = segment_4[0].start_intersection;
        connect_14.subpath = find_path_between_intersections(connect_14.start_intersection, connect_14.end_intersection,turn_penalty);
        connect_14.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_43;
        connect_43.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_43.end_intersection = segment_3[0].start_intersection;
        connect_43.subpath = find_path_between_intersections(connect_43.start_intersection, connect_43.end_intersection,turn_penalty);
        connect_43.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_21);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_14);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_43);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2314
    else if(reconnect == 7){
        CourierSubpath connect_23;
        connect_23.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_23.end_intersection = segment_3[0].start_intersection;
        connect_23.subpath = find_path_between_intersections(connect_23.start_intersection, connect_23.end_intersection,turn_penalty);
        connect_23.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_31;
        connect_31.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_31.end_intersection = segment_1[0].start_intersection;
        connect_31.subpath = find_path_between_intersections(connect_31.start_intersection, connect_31.end_intersection,turn_penalty);
        connect_31.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_14;
        connect_14.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_14.end_intersection = segment_4[0].start_intersection;
        connect_14.subpath = find_path_between_intersections(connect_14.start_intersection, connect_14.end_intersection,turn_penalty);
        connect_14.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_23);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_31);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_14);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2341
    else if(reconnect == 8){
        CourierSubpath connect_23;
        connect_23.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_23.end_intersection = segment_3[0].start_intersection;
        connect_23.subpath = find_path_between_intersections(connect_23.start_intersection, connect_23.end_intersection,turn_penalty);
        connect_23.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_34;
        connect_34.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_34.end_intersection = segment_4[0].start_intersection;
        connect_34.subpath = find_path_between_intersections(connect_34.start_intersection, connect_34.end_intersection,turn_penalty);
        connect_34.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_41;
        connect_41.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_41.end_intersection = segment_1[0].start_intersection;
        connect_41.subpath = find_path_between_intersections(connect_41.start_intersection, connect_41.end_intersection,turn_penalty);
        connect_41.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_23);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_34);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_41);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2413
    else if(reconnect == 9){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_1.end_intersection = segment_4[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_2.end_intersection = segment_1[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_3.end_intersection = segment_3[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //2431
    else if(reconnect == 10){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_1.end_intersection = segment_4[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_2.end_intersection = segment_3[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_3.end_intersection = segment_1[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_2[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3124
    else if(reconnect == 11){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_1[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_2.end_intersection = segment_2[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_3.end_intersection = segment_4[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() -1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3142
    else if(reconnect == 12){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_1[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_2.end_intersection = segment_4[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_3.end_intersection = segment_2[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3214
    else if(reconnect == 13){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_2[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_2.end_intersection = segment_1[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_3.end_intersection = segment_4[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3241
    else if(reconnect == 14){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_2[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_2.end_intersection = segment_4[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_3.end_intersection = segment_1[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3412
    else if(reconnect == 15){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_4[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_2.end_intersection = segment_1[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[original_path.size() - 1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_3.end_intersection = segment_2[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //3421
    else if(reconnect == 16){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_1.end_intersection = segment_4[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_2.end_intersection = segment_2[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_3.end_intersection = segment_1[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_3[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4123
    else if(reconnect == 17){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_1[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_2.end_intersection = segment_2[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_3.end_intersection = segment_3[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4132
    else if(reconnect == 18){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_1[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_2.end_intersection = segment_3[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_3.end_intersection = segment_2[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4213
    else if(reconnect == 19){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_2[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_2.end_intersection = segment_1[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_3.end_intersection = segment_3[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4231
    else if(reconnect == 20){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_2[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_2.end_intersection = segment_3[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_3.end_intersection = segment_1[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4312
    else if(reconnect == 21){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_3[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_2.end_intersection = segment_1[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        connect_3.end_intersection = segment_2[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(to_depot);
    }
    //4321
    else if(reconnect == 22){
        CourierSubpath connect_1;
        connect_1.start_intersection = segment_4[segment_4.size() -1].end_intersection;
        connect_1.end_intersection = segment_3[0].start_intersection;
        connect_1.subpath = find_path_between_intersections(connect_1.start_intersection, connect_1.end_intersection,turn_penalty);
        connect_1.pickUp_indices = original_path[original_path.size()-1].pickUp_indices;
        
        CourierSubpath connect_2;
        connect_2.start_intersection = segment_3[segment_3.size() -1].end_intersection;
        connect_2.end_intersection = segment_2[0].start_intersection;
        connect_2.subpath = find_path_between_intersections(connect_2.start_intersection, connect_2.end_intersection,turn_penalty);
        connect_2.pickUp_indices = original_path[cut_3].pickUp_indices;
        
        CourierSubpath connect_3;
        connect_3.start_intersection = segment_2[segment_2.size() -1].end_intersection;
        connect_3.end_intersection = segment_1[0].start_intersection;
        connect_3.subpath = find_path_between_intersections(connect_3.start_intersection, connect_3.end_intersection,turn_penalty);
        connect_3.pickUp_indices = original_path[cut_2].pickUp_indices;
        
        CourierSubpath to_depot;
        to_depot.start_intersection = segment_1[segment_1.size() -1].end_intersection;
        to_depot.end_intersection = find_closest_depot(depots, to_depot.start_intersection, turn_penalty);
        to_depot.subpath = find_path_between_intersections(to_depot.start_intersection, to_depot.end_intersection,turn_penalty);
        to_depot.pickUp_indices = original_path[cut_1].pickUp_indices;
        
        CourierSubpath from_depot;
        from_depot.start_intersection = find_closest_depot(depots, from_depot.end_intersection, turn_penalty);
        from_depot.end_intersection = segment_4[0].start_intersection;
        from_depot.subpath = find_path_between_intersections(from_depot.start_intersection, from_depot.end_intersection,turn_penalty);
        from_depot.pickUp_indices = original_path[0].pickUp_indices;
        
        potential_path.push_back(from_depot);
        
        for(int i = 0; i < segment_4.size(); i++){
            potential_path.push_back(segment_4[i]);
        }
        potential_path.push_back(connect_1);
        
        for(int i = 0; i < segment_3.size(); i++){
            potential_path.push_back(segment_3[i]);
        }
        potential_path.push_back(connect_2);
        
        for(int i = 0; i < segment_2.size(); i++){
            potential_path.push_back(segment_2[i]);
        }
        potential_path.push_back(connect_3);
        
        for(int i = 0; i < segment_1.size(); i++){
            potential_path.push_back(segment_1[i]);
        }
        potential_path.push_back(to_depot);
    }
    return potential_path;
}


std::vector<int> reverse(std::vector<int> forward){
    std::vector<int> reversed;
    
    reversed.resize(forward.size());
    int j = 0;
    for(int i = forward.size() - 1; i >= 0 ; i++){
        reversed[j] = forward[i];
        j++;
    }
    
    return reversed;
}