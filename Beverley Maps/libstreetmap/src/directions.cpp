#include "directions.h"
#include "StreetsDatabaseAPI.h"
int flag = 0;
int new_path = 0;
int walk_to_drive;

//Draws the main window for displaying the directions of a given path
void draw_directions(){
    GtkWidget *window;
    GtkWidget *scrolled_window; 
    GtkWidget *directions;
    std::string route = "";
    gint height = 200;
    gint width = 300;
            
    //Create a new window for the directions
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Directions");
    gtk_window_set_default_size(GTK_WINDOW(window), width, height);
    
    //Create a scrolling window for the directions
    scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_set_border_width(GTK_CONTAINER(scrolled_window), 5);
    
    //Add directions based on if its walk+drive
    if(drive_vs_walk){
        if(walk_drive_path.first.empty() && walk_drive_path.second.empty()){
            route += "No path found sorry :(";
        }
        else{
            if(walk_drive_path.first.empty() && !walk_drive_path.second.empty()){
                route += "No need to walk just wait for your Uber \n"; 
                flag++;
                route += get_directions(walk_drive_path.second);
            }
            else if(!walk_drive_path.first.empty() && !walk_drive_path.second.empty()){
                walk_to_drive = get_shared_intersection();
                
                route += get_directions(walk_drive_path.first);
                new_path++;
                route += get_directions(walk_drive_path.second);
            }
            
            else if(!walk_drive_path.first.empty() && walk_drive_path.second.empty()){
                route += "No need to drive, just walk \n";
                flag++;
                route += get_directions(walk_drive_path.first);
            }
            
        }
    }
    
    //Add directions based on if its drive only
    else if(!drive_vs_walk){
        if(shortest_path.empty()){
            route += "No path found sorry :(";
        }
        else{
            route += get_directions(shortest_path);
        }
    }
    
    char * label;
    label  = new char[route.size() + 1];
    std::copy(route.begin(), route.end(), label);
    label[route.size()] = '\0';
    
    directions = gtk_label_new(label);
    
        
    gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window),
                                  GTK_POLICY_AUTOMATIC,
                                  GTK_POLICY_AUTOMATIC);
    
    gtk_container_add(GTK_CONTAINER(scrolled_window), directions);

    gtk_container_add (GTK_CONTAINER (window), scrolled_window);

    gtk_widget_show_all (GTK_WIDGET(window));
    
    flag = 0;
    new_path = 0;
    
    delete [] label;
}


//Finds the directions on a given route and returns a string with the instructions
std::string get_directions(std::vector<StreetSegmentIndex> route){
    std::string directions = "";
    int reference_int = 0;
    int prev_ref_int = 0;
    StreetIndex prev_street = 0;
    int road_travel = 0;
    
    //Sets the starting direction greeting depending on the path find type
    if(!drive_vs_walk){
        directions =+ "Starting at " + getIntersectionName(first_int_search) + "\n";
        reference_int = first_int_search;
    }
    if(drive_vs_walk){
        if(new_path == 0){
          directions =+ "Starting at " + getIntersectionName(first_int_search) + "\n";
          reference_int = first_int_search;
          prev_ref_int = first_int_search;
        }
        if(new_path == 1){
          directions =+ "Starting at " + getIntersectionName(walk_to_drive) + "\n";
          reference_int = walk_to_drive;
          prev_ref_int  = walk_to_drive;
        }
    }
    
    
    //Checks if the walk or drive routes are empty
    if (route.size() > 0) {
        prev_street = getInfoStreetSegment(route[0]).streetID;
    }
    
    else {
        return "";
        flag++;
    }
    
    for(int i = 0; i < route.size(); i++){
        
        //turn directions
        if( getInfoStreetSegment(route[i]).streetID != prev_street && i != 0){
            if(route.size() > 1){
                if(road_travel > 0){
                    directions += "Head ";
                    directions += compass_direction(route[i-1], prev_ref_int);

                    if(road_travel > 1000){
                        double km = ((int)road_travel)/1000.0;
                        std::ostringstream get_time;
                        get_time  << std::setprecision(2);
                        get_time << km;
                        
                        directions += "for " + get_time.str() + "km";
                        road_travel = find_street_segment_length(route[i]);
                    }
                    else{
                        std::ostringstream get_time;
                        get_time <<std::setprecision(2);
                        get_time << road_travel;
                        
                        directions += "for " + get_time.str() + "m";
                        road_travel = find_street_segment_length(route[i]);
                    }

                    directions += " on " + getStreetName(getInfoStreetSegment(route[i-1]).streetID);
                    directions += "\n";
                }   
            }
            directions += get_turn(route[i-1], route[i]) + "\n";
        }
        
        if(getInfoStreetSegment(route[i]).streetID == prev_street){
            road_travel += find_street_segment_length(route[i]);
        }
        
        
        //update the previous street id
        prev_street = getInfoStreetSegment(route[i]).streetID;
        
        //Update the reference node
        if(getInfoStreetSegment(route[i]).to == reference_int){
            prev_ref_int = reference_int;
            reference_int = getInfoStreetSegment(route[i]).from;
        }
        
        else{
            prev_ref_int = reference_int;
            reference_int = getInfoStreetSegment(route[i]).to;
        }
        
        //If the path is done then make sure to print out the last travel directions
        if(i == route.size() - 1){
            if(road_travel > 0){
                directions += "Head ";
                directions += compass_direction(route[i-1], prev_ref_int);
                
                if(road_travel > 1000){
                        double km = ((int)road_travel)/1000.0;
                        
                        std::ostringstream get_km;
                        get_km  << std::setprecision(2);
                        get_km << km;
                        
                        directions += "for " + get_km.str() + "km";
                        road_travel = find_street_segment_length(route[i]);
                    }
                    else{
                        directions += "for " + std::to_string(road_travel) + "m";
                        road_travel = find_street_segment_length(route[i]);
                    }
                
                directions += " on " + getStreetName(getInfoStreetSegment(route[i]).streetID);
                directions += "\n";
            }    
        }
    }
    
    //Adds a dialog depending on if you're driving or walking
    if(drive_vs_walk && flag == 0){
        directions += "Hop into your Uber at ";
        directions += getIntersectionName(reference_int) + "\n";
        flag++;
    }
    
    else{
        directions += "You've arrived at your destination ";
        directions += getIntersectionName(second_int_search) + "\n";
        
        std::ostringstream get_time;
        get_time << std::setprecision(2);
        
        //Checks the total time and outputs the time for traveling on the path
        if(drive_vs_walk){
            double total_time = compute_path_travel_time(walk_drive_path.second, 15.000) + compute_path_walking_time(walk_drive_path.first, walk_speed, 15.00);
            directions += "The trip should only take ";
            std::ostringstream get_time_w;
            get_time  << std::setprecision(2);
            int time_type = 0;
            
            //Changes total time to minutes, hours or seconds
            if(total_time > 60){
                total_time = ((int)total_time)/60.0;
                get_time_w << total_time;
                time_type++;
                
                if(total_time > 60){
                    total_time = ((int)total_time)%60;
                    get_time_w << total_time;
                    time_type++;
                }
            }
            else{
                get_time_w << total_time;
            }
            
            //Displays the travel time 
            if(time_type == 0){
                directions += get_time_w.str() + " seconds";
            }
            else if(time_type == 1){
                directions += get_time_w.str() + " minutes";
            }
            else if(time_type == 2){
                directions += get_time_w.str() + " hours";
            }
            
        }
        
        else if(!drive_vs_walk){
            double total_time = compute_path_travel_time(shortest_path, 15.000);
            directions += "The trip should only take ";
            std::ostringstream get_time_d;
            get_time  << std::setprecision(2);
            int time_type = 0;
            
            //Changes total time to minutes, hours or seconds
            if(total_time > 60){
                total_time = ((int)total_time)/60.0;
                get_time_d << total_time;
                time_type++;
                
                if(total_time > 60){
                    total_time = ((int)total_time)%60;
                    get_time_d << total_time;
                    time_type++;
                }
            }
            else {
                get_time_d << total_time;
            }
            
            
            //Displays the travel time
            if(time_type == 0){
                directions += get_time_d.str() + " seconds";
            }
            else if(time_type == 1){
                directions += get_time_d.str() + " minutes";
            }
            else if(time_type == 2){
                directions += get_time_d.str() + " hours";
            }
            
            
        }
    }
    
    
    return directions;    
}


//Generates a string that tells the user which direction to turn
std::string get_turn(StreetSegmentIndex seg1, StreetSegmentIndex seg2){
    //First find out where the direction of travel is...
    int to_1 = getInfoStreetSegment(seg1).to;
    int from_1 = getInfoStreetSegment(seg1).from;
    int to_2 = getInfoStreetSegment(seg2).to;
    int from_2 = getInfoStreetSegment(seg2).from;
    
    int shared_int = 0;
    
    std::pair<double, double> v1, v2;
    
    //Find the shared intersection and create a pair of vectors
    if(to_1 == to_2){
        ezgl::point2d seg1_1 = latlon_to_point(getIntersectionPosition(from_1));
        ezgl::point2d shared = latlon_to_point(getIntersectionPosition(to_1));
        shared_int = to_1;
        ezgl::point2d seg2_2 = latlon_to_point(getIntersectionPosition(from_2));
        
        v1.first = shared.x - seg1_1.x;
        v1.second = shared.y - seg1_1.y;
        
        v2.first = seg2_2.x - shared.x;
        v2.second = seg2_2.y - shared.y;
    }
    
    else if(to_1 == from_2){
        ezgl::point2d seg1_1 = latlon_to_point(getIntersectionPosition(from_1));
        ezgl::point2d shared = latlon_to_point(getIntersectionPosition(to_1));
        shared_int = to_1;
        ezgl::point2d seg2_2 = latlon_to_point(getIntersectionPosition(to_2));
        
        v1.first = shared.x - seg1_1.x;
        v1.second = shared.y - seg1_1.y;
        
        v2.first = seg2_2.x - shared.x;
        v2.second = seg2_2.y - shared.y;
    }
    
    else if(from_1 == to_2){
        ezgl::point2d seg1_1 = latlon_to_point(getIntersectionPosition(to_1));
        ezgl::point2d shared = latlon_to_point(getIntersectionPosition(from_1));
        shared_int = from_1;
        ezgl::point2d seg2_2 = latlon_to_point(getIntersectionPosition(from_2));
        
        v1.first = shared.x - seg1_1.x;
        v1.second = shared.y - seg1_1.y;
        
        v2.first = seg2_2.x - shared.x;
        v2.second = seg2_2.y - shared.y;
    }
    
    else if(from_1 == from_2){
        ezgl::point2d seg1_1 = latlon_to_point(getIntersectionPosition(to_1));
        ezgl::point2d shared = latlon_to_point(getIntersectionPosition(from_1));
        shared_int = from_1;
        ezgl::point2d seg2_2 = latlon_to_point(getIntersectionPosition(to_2));
        
        v1.first = shared.x - seg1_1.x;
        v1.second = shared.y - seg1_1.y;
        
        v2.first = seg2_2.x - shared.x;
        v2.second = seg2_2.y - shared.y;
    }
    
    //Calculate the cross product, this tells us the circulation of the vector field
    double turn  = cross_product(v1, v2);
    
    if(turn > 0){
        return "Turn Left onto " + getStreetName(getInfoStreetSegment(seg2).streetID) + " ";
    }
    
    else if(turn < 0){
        return "Turn Right onto " + getStreetName(getInfoStreetSegment(seg2).streetID) + " ";
    }
    
    else {
        return "Head " + compass_direction(seg2, shared_int) + " onto " + getStreetName(getInfoStreetSegment(seg2).streetID) + " ";
    }
    
}


//Calculates the cross product between two vectors
double cross_product(std::pair<double, double> v1, std::pair<double, double> v2){
    return (v1.first * v2.second) - (v1.second * v2.first);
}


//Finds the compass direction 
std::string compass_direction(StreetSegmentIndex seg, IntersectionIndex reference){
    std::string compass;
    
    int to = getInfoStreetSegment(seg).to;
    int from = getInfoStreetSegment(seg).from;
    
    double dx;
    double dy;
    
    //Gets the directions
    if(reference == to){
        ezgl::point2d to_point = latlon_to_point(getIntersectionPosition(to));
        ezgl::point2d from_point = latlon_to_point(getIntersectionPosition(from));
        
        dx = from_point.x - to_point.x;
        dy = from_point.y - to_point.y;
    }
    else{
        ezgl::point2d to_point = latlon_to_point(getIntersectionPosition(to));
        ezgl::point2d from_point = latlon_to_point(getIntersectionPosition(from));
        
        dx = to_point.x - from_point.x;
        dy = to_point.y - from_point.y;
    }
    
    //Based on cartesian coordinates generate the correct direction
    if(dy > 0){
        compass += "North ";
    }
    else if(dy < 0){
        compass += "South ";
    }
    
    if(!compass.empty()){
        compass.erase(std::find(compass.begin(), compass.end(), ' '));
        compass += "";
    }
    
    if(dx > 0){
        compass += "East ";
    }
    else if(dx < 0){
        compass += "West ";
    }
    
    return compass;
}


//Finds the transition intersection between walking and driving
int get_shared_intersection(){
    InfoStreetSegment walk_end = getInfoStreetSegment(walk_drive_path.first[walk_drive_path.first.size() -1]);
    InfoStreetSegment drive_start = getInfoStreetSegment(walk_drive_path.second[0]);
    
    if(walk_end.to == drive_start.to){
        return walk_end.to;
    }
    else if(walk_end.to == drive_start.from){
        return walk_end.to;
    }
    else if(walk_end.from == drive_start.to){
        return walk_end.from;
    }
    else if(walk_end.from == drive_start.from){
        return walk_end.from;
    }
    
    return 0;
}

