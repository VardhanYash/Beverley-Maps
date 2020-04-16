/*
 * This file is used to de-clutter m2.cpp
 * In here is the subroutines and functions used to put different stuff on screen
 * For example intersections, street segments and features are all generated here
 */
#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "GlobalVariables.h"
#include "ezgl/point.hpp"
#include "ezgl/rectangle.hpp"
#include "draw_main_canvas.h"
#include "cartesian.h"
#include "altitude_data.h"
#include "map_path.h"
#include "m3.h"
#include "search_viz.h"
#include "search.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>

//Global variables
int previous_id = 0;
int ZOOM = 8;
long double prev_area = 0;
int LATLONSCALING = 1000000;
bool POI_TOGGLE = false;
bool PATH_TOGGLE = false;
bool FEATURE_NAME_TOGGLE = false;
bool ERROR_DETECT = false;
int prev_poi_index = 0;
int click_count = 0;
int first_int_search;
int second_int_search;
const double LARGE_ZOOM_SCALE = 0.03;
const double MEDIUM_ZOOM_SCALE = 0.025;
const double SMALL_ZOOM_SCALE = 0.02;
const double MIN_SEG_LENGTH = 55.0;
const int BIG_NUMBER = 9999999;
std::string URL_altitude;
double walk_speed = 0;
double walk_time = 0;
bool drive_vs_walk;

//Initial setup of the UI
void initial_setup(ezgl::application *application, bool new_window){
    new_window = new_window;
    
    application->update_message("CD-042");
    
    application->create_button("Find path: Text Entry", 12, find_button);
    
    application->create_button("Points of Interest", 8, poi_button);
    
    application->create_button("Find Streets", 9, street_button);
    
    application->create_button("Load Map", 0, city_select);
    
    application->create_button("Find Path: Mouse Entry", 11, path_button);
    
    application->create_button("Feature names",9, feature_name_button);
}


//Draws intersections
void draw_intersections(ezgl::renderer* g){
    
    if(ZOOM >= 17){
        for (size_t i = 0; i< intersections.size(); i++) {
             double x = intersections[i].cartesian.first;
             double y = intersections[i].cartesian.second;        

             if(intersections[i].highlight == true) {
                 g->set_color(ezgl::RED);
             }

             else {
                 g->set_color(ezgl::WHITE);
             }
             g->fill_elliptic_arc({x, y}, 0.00025, 0.00025, 0, 360);
        }
    }
}


//Overall street segement drawer
void draw_street_segments(ezgl::renderer* g){
    
    //Draws the streets themselves
    //For each street segement check if it curves:
    //If it does than you'll have to use draw_arc latter
    //If it doesn't then draw a line from intersection A to B
    
    
    for(int i = 0; i < getNumStreetSegments(); i++){
        InfoStreetSegment segment_info = getInfoStreetSegment(i);
        ezgl::point2d from (0,0);
        ezgl::point2d to (0,0);
        //If the segment has no curve points just draw the straight line
        
        if(segment_info.curvePointCount == 0){
            from = latlon_to_point(getIntersectionPosition(segment_info.from));
            to = latlon_to_point(getIntersectionPosition(segment_info.to));
            
            draw_street(i, g, from, to, segment_info);
        }
        
        //If it does have curved points then draw a bunch of smaller lines
        else{
            for(int j = 0; j < array_of_curve_points[i].size() - 1; j++){
                from = latlon_to_point(array_of_curve_points[i][j]);
                to = latlon_to_point(array_of_curve_points[i][j + 1]);
                
                draw_street(i, g, from, to, segment_info);

            }
            
        }
    }
}


//Draws the streets
void draw_street(int i, ezgl::renderer *g, ezgl::point2d from, ezgl::point2d to, InfoStreetSegment segment_info){
    
    if(ZOOM >= 16) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(SMALL_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    if (0.01 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.02 * MAXSTREETLENGTH && ZOOM >= 15) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(SMALL_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    else if (0.02 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.05 * MAXSTREETLENGTH && ZOOM >= 14) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(SMALL_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    

    else if (0.05 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.15 * MAXSTREETLENGTH && ZOOM >= 13) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(SMALL_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }

    else if (0.15 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.3 * MAXSTREETLENGTH && ZOOM >= 12) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(MEDIUM_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    else if (0.3*MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.4*MAXSTREETLENGTH && ZOOM >= 11) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(MEDIUM_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    else if (0.4*MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.5*MAXSTREETLENGTH && ZOOM >= 11) {
        g->set_color(ezgl::WHITE);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(LARGE_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    } 
    
    else if (street_length[getInfoStreetSegment(i).streetID] >= 0.5*MAXSTREETLENGTH && getStreetName(segment_info.streetID) != "<unknown>") {
        g->set_color(ezgl::HIGHWAY_OUT);
        g->set_line_width(LARGE_ZOOM_SCALE*pow(ZOOM, 2) + 2);
        g->draw_line(from, to);
        
        g->set_color(ezgl::HIGHWAY);
        
        if (streets[getInfoStreetSegment(i).streetID].highlight) {
            g->set_color(ezgl::STREET_FOUND_BLUE);
        }
        else if(PATH_TOGGLE && click_count == 2){
            g->set_color(ezgl::BLUE);
        }
        g->set_line_width(LARGE_ZOOM_SCALE*pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    
}


//Write street name
void write_street_name(int i, InfoStreetSegment segment_info, ezgl::renderer *g) {
    std::string segment_name = getStreetName(segment_info.streetID);
    //if the street segment is long enough to have text written on it
    bool draw_name = (segment_name != "<unknown>") && (street_segment_length[i] > MIN_SEG_LENGTH);
    //checks if the segment is of a one way street or if the segment is long enough
    
    bool one_way = segment_info.oneWay;
    
    if (draw_name || one_way) {
        LatLon segment_from;
        LatLon segment_to;
        
        segment_from = getIntersectionPosition(segment_info.from);
        segment_to = getIntersectionPosition(segment_info.to);
        //finds the center point of the street segment
        LatLon center_point((segment_from.lat() + segment_to.lat()) / 2.0, (segment_from.lon() + segment_to.lon()) / 2.0);
        ezgl::point2d center = latlon_to_point(center_point);
        
        if(segment_info.curvePointCount !=0) {
            double distance = BIG_NUMBER;
            int closest_segment_id = 0;
            for (int j = 0; j < array_of_curve_points[i].size(); j++) {
                std::pair<LatLon,LatLon> compare_pts (center_point, array_of_curve_points[i][j]);
                
                if(distance >find_distance_between_two_points(compare_pts)) {
                    distance = find_distance_between_two_points(compare_pts);
                    closest_segment_id = j;
                }
            }
            
            if(closest_segment_id + 1== array_of_curve_points[i].size()) {
                segment_from = array_of_curve_points[i][closest_segment_id-1];
                segment_to = array_of_curve_points[i][closest_segment_id];
            }
            
            else {
                segment_from = array_of_curve_points[i][closest_segment_id];
                segment_to = array_of_curve_points[i][closest_segment_id + 1];
            }
            
            LatLon center_point_curve((segment_from.lat() + segment_to.lat()) / 2.0, (segment_from.lon() + segment_to.lon()) / 2.0);
            center = latlon_to_point(center_point_curve);
            
        }
        
        g->set_color(ezgl::BLACK);
        g->set_font_size(MEDIUM_ZOOM_SCALE*pow(ZOOM,2.0));
        g->set_vert_text_just(ezgl::text_just::center);
        g->set_horiz_text_just(ezgl::text_just::center);

        //checks which direction the street is running in order to orient
        // the name of it correctly on the map
        double change_in_x = 0;
        double change_in_y = 0;
        change_in_x = std::abs(lon_to_x(segment_to.lon()) * LATLONSCALING - lon_to_x(segment_from.lon()) * LATLONSCALING);
        
        change_in_y = std::abs(lat_to_y(segment_to.lat()) * LATLONSCALING - lat_to_y(segment_from.lat()) * LATLONSCALING);
        double angle = (1 / DEGREE_TO_RADIAN) * atan(change_in_y / change_in_x);
        
        if(segment_to.lat() * LATLONSCALING > segment_from.lat() * LATLONSCALING && segment_to.lon() * LATLONSCALING < segment_from.lon() * LATLONSCALING) {
            angle = -angle;
        }
        
        else if (segment_to.lat() * LATLONSCALING < segment_from.lat() * LATLONSCALING && segment_to.lon() * LATLONSCALING > segment_from.lon() * LATLONSCALING) {
            angle = -angle;
        }
        
        g->set_text_rotation(angle);


        float strLen = 0.005;
        
        //if the street is a one way AND the segment is long enough
        if (draw_name && one_way) {
            std::string text_to_write;
           
            if (segment_from.lon() * LATLONSCALING > segment_to.lon() * LATLONSCALING) {
                text_to_write = "◄  " + segment_name;
            } 
            else {
                text_to_write = segment_name + "  ►";
            }
            
            g->draw_text(center, text_to_write, strLen, strLen);
        } 

        //if the segment is long enough but not a one way street
        
        if(draw_name && !one_way) {
            g->draw_text(center, segment_name ,strLen,strLen);
        } 
  
    }
}


//Draws each feature based on if it's a closed polygon and its type
void draw_features(ezgl::renderer* g){
    //Unknown -> grey
    //Park -> green
    //Beach -> sand/yellow
    //Lake -> light blue
    //River -> dark blue
    //Island -> light brown
    //Building -> light grey
    //Greenspace -> light green
    //Golfcourse -> light green
    //Stream -> light blue
    
    for(int feature_id = 0; feature_id < getNumFeatures(); feature_id++){
        int point_count = getFeaturePointCount(feature_id);
        FeatureType type = getFeatureType(feature_id);
        
        if(point_count > 2){
            
            if(type == Unknown){
                g->set_color(ezgl::GREY_75);
            }
            else if(type == Park || type == Golfcourse){
                g->set_color(ezgl::GMAPSPARK);
            }
            else if(type == Greenspace ){
                g->set_color(ezgl::DARK_GREEN);
            }
            else if(type == Beach){
                g->set_color(ezgl::BEACH);
            }
            else if(type == Lake || type == River || type == Stream){
                g->set_color(ezgl::LAKE);
            }
            else if(type == Island){
                g->set_color(ezgl::ISLAND);
            }
            else if(type == Building){
                g->set_color(ezgl::BUILDING);
            }
            
            bool large_objects =  (type == Lake) || (type == Island) || (type == Greenspace);
            bool medium_objects =  (type == Golfcourse) || (type == Beach) || (type == River) || (type == Park) ;
            bool small_objects =  (type == Unknown) || (type == Building) || (type == Stream) ;

            
            if(large_objects){
                draw_poly(g, feature_id, point_count);
                
                if(FEATURE_NAME_TOGGLE){
                   if(point_count > 2 && ZOOM >= 15 && feature_area[feature_id] >= 3000){
                    draw_feature_name(g,feature_id,point_count);
                    } 
                }
            }
            
            else if((ZOOM >= 13) && medium_objects){
                draw_poly(g, feature_id, point_count);
                
                if(FEATURE_NAME_TOGGLE){
                    if(point_count > 2 && ZOOM >= 16 && feature_area[feature_id] >= 3000){
                        draw_feature_name(g,feature_id,point_count);
                    }
                }
            }
            
            else if((ZOOM >= 17) && small_objects){
                draw_poly(g, feature_id, point_count);
                
                if(FEATURE_NAME_TOGGLE){
                    if(point_count > 2 && feature_area[feature_id] >= 3000){
                        draw_feature_name(g,feature_id,point_count);
                    }
                }
            }
            
            if(FEATURE_NAME_TOGGLE){
                if(large_objects && point_count > 2 && ZOOM >= 15 && feature_area[feature_id] >= 3000){ 
                    draw_feature_name(g,feature_id,point_count);
                }
            }
                
            if(FEATURE_NAME_TOGGLE){
                if(ZOOM >= 16 && medium_objects && point_count > 2 && feature_area[feature_id] >= 3000){
                    draw_feature_name(g,feature_id,point_count);
                }
            }
            
            if(FEATURE_NAME_TOGGLE){
                if(ZOOM >= 17 && small_objects && point_count > 2 && feature_area[feature_id] >= 3000){
                    draw_feature_name(g,feature_id,point_count);
                }               
            }
        }
    }
}


//Draws a polygon
void draw_poly(ezgl::renderer *g, int feature_id, int point_count){
    //Draw a closed polygon
    if(points_of_feature[feature_id][0] == points_of_feature[feature_id][point_count - 1]){
        const std::vector<ezgl::point2d> copy_(points_of_feature[feature_id].begin(), points_of_feature[feature_id].end() - 1);
        g->fill_poly(copy_);
    }

    //Draw a curved line
    else{
        g->set_line_width(0.02);
        for(int i  = 0; i < points_of_feature[feature_id].size() - 1; i++){
            g->draw_line(points_of_feature[feature_id][i], points_of_feature[feature_id][i + 1]);
        }
    }
}


//Draws the feature names only if the toggle is enabled
void draw_feature_name(ezgl::renderer *g, int feature_id, int point_count){
    std::string feature_name = getFeatureName(feature_id);
    if(feature_name != "<noname>"){
        double x_val = 0.0;
        double y_val = 0.0;
        
        int upper_range;
        if(points_of_feature[feature_id][0] == points_of_feature[feature_id][point_count - 1]){
            upper_range = point_count - 1;
        }
        else{
            upper_range = point_count;
        }
        
        for(int i = 0; i < upper_range; i++){ 
            x_val += points_of_feature[feature_id][i].x;
            y_val += points_of_feature[feature_id][i].y;
        }
        
        x_val = x_val/upper_range;
        y_val = y_val/upper_range;
        
        
        ezgl::point2d center(x_val,y_val);

        float strLen = 0.05;
       
        g->set_color(ezgl::BLACK);
        g->set_text_rotation(0.0);
        g->set_font_size(LARGE_ZOOM_SCALE*pow(ZOOM,2));
        g->draw_text(center,feature_name,strLen,strLen);
        
    }
}


//Callback function for performing actions on mouse click
void act_on_mouse_click(ezgl::application *app, GdkEventButton* event, double x, double y) {
    ezgl::renderer *g = app->get_renderer();
    
    //If clicking the poi toggle then highlight, and display information when the 
    //mouse clicks a poi
    if(event->button == 1){
        LatLon pos((float) y_to_lat(y), (float) x_to_lon(x));
        
        if(POI_TOGGLE){
            int poi_index = find_closest_poi(pos);
            
            if(pois[poi_index].highlight){
                pois[poi_index].highlight = false;
            }
            else{
                pois[poi_index].highlight = true;
            }
            
            pois[prev_poi_index].highlight = false;
            
            if(prev_poi_index == poi_index){
                pois[poi_index].highlight = true;
            }
            
            if(pois[poi_index].highlight){
                char* poi_name;
                char* lable_info;
                std::string str = getIntersectionName(poi_index);
                
                //Creates the labels for the pop up dialogue box
                std::string lable_str = "The name of the point of interest is: " + pois[poi_index].name;
                lable_str = lable_str + " \n" + "It's near " + getIntersectionName(find_closest_intersection(pos));
                
                
                poi_name = new char[str.size()+1];
                std::copy(str.begin(), str.end(), poi_name);
                poi_name[str.size()] = '\0';
                
                lable_info = new char[lable_str.size()+1];
                std::copy(lable_str.begin(), lable_str.end(), lable_info);
                lable_info[lable_str.size()] = '\0';
                
                // BEGIN: CODE FOR SHOWING DIALOG
                GObject *window;
                GtkWidget *content_area;
                GtkWidget *label;
                GtkWidget *dialog;
                
                // get a pointer to the main application window
                window = app->get_object(app->get_main_window_id().c_str());
                
                dialog = gtk_dialog_new_with_buttons(
                poi_name,
                (GtkWindow*) window,
                GTK_DIALOG_MODAL,
                ("OK"),
                GTK_RESPONSE_ACCEPT,
                NULL,
                GTK_RESPONSE_REJECT,
                NULL
                );
                
                // Create a label and attach it to the content area of the dialog
                content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
                label = gtk_label_new(lable_info);
                gtk_container_add(GTK_CONTAINER(content_area), label);
                
                // The main purpose of this is to show dialog’s child widget, label
                gtk_widget_show_all(dialog);
                
                g_signal_connect(
                GTK_DIALOG(dialog),
                "response",
                G_CALLBACK(intersection_response),
                NULL
                );
                
                //Delete char*
                delete [] poi_name;
                delete [] lable_info;
            }
            
            
            prev_poi_index = poi_index;
            
            app->refresh_drawing();
        }
        
        
        else if(PATH_TOGGLE){            
            click_count++; 
            if(click_count == 1){
                //Draws a flag at the starting intersection
                first_int_search =  find_closest_intersection(pos);
                
                g->set_color(ezgl::BLACK);
                
                draw_path_flag(g, first_int_search, "⚐");
                
                app->flush_drawing();
                
                app->update_message("Click another intersection to search");
   
            }
            
            if(click_count == 2){
                //Draws a flag on the second intersection and draws the path
                second_int_search = find_closest_intersection(pos);
                                
                g->set_color(ezgl::BLACK);
                
                draw_path_flag(g, second_int_search, "⚑");
                
                app->flush_drawing();
                
                if(!drive_vs_walk){
                    find_path_between_intersections(first_int_search, second_int_search, 15.0000);
                }
                else if(drive_vs_walk){
                   
                    find_path_with_walk_to_pick_up(first_int_search, second_int_search, 15.000, walk_speed, walk_time);
                }

                //draw_directions();
                
                click_count = 0;
                
                app->refresh_drawing();
                
                app->update_message("Did you find your way?");
            }
        }
        
        //Otherwise highlight and display info on the intersections
        else{
            int id = find_closest_intersection(pos);

            if(intersections[id].highlight)
                intersections[id].highlight = false;
            else
                intersections[id].highlight = true;

            intersections[previous_id].highlight = false;

            if(intersections[id].highlight){
                //Get the altitude data
                std::string URL_altitude_start = "https://api.airmap.com/elevation/v1/ele/?points=";
                std::string comma = ",";
                std::string first_lat = std::to_string(pos.lat());
                std::string first_lon = std::to_string(pos.lon());

                URL_altitude = URL_altitude_start + first_lat + comma + first_lon;
                altitude_api_set_up();
                
                char* int_name;
                char* lable_info;
                std::string str = getIntersectionName(id);
                
                std::string lable_str = "It's located at: " + first_lat + " " + first_lon;
                lable_str = lable_str + " \n" + elevation_info;
                
                
                int_name = new char[str.size()+1];
                std::copy(str.begin(), str.end(), int_name);
                int_name[str.size()] = '\0';
                
                lable_info = new char[lable_str.size()+1];
                std::copy(lable_str.begin(), lable_str.end(), lable_info);
                lable_info[lable_str.size()] = '\0';
                
                // BEGIN: CODE FOR SHOWING DIALOG
                GObject *window;
                GtkWidget *content_area;
                GtkWidget *label;
                GtkWidget *dialog;
                
                // get a pointer to the main application window
                window = app->get_object(app->get_main_window_id().c_str());
                
                dialog = gtk_dialog_new_with_buttons(
                    int_name,
                    (GtkWindow*) window,
                    GTK_DIALOG_MODAL,
                    ("OK"),
                    GTK_RESPONSE_ACCEPT,
                    NULL,
                    GTK_RESPONSE_REJECT,
                    NULL
                );
                
                // Create a label and attach it to the content area of the dialog
                content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
                label = gtk_label_new(lable_info);
                gtk_container_add(GTK_CONTAINER(content_area), label);
                
                // The main purpose of this is to show dialog’s child widget, label
                gtk_widget_show_all(dialog);
                
                g_signal_connect(
                    GTK_DIALOG(dialog),
                    "response",
                    G_CALLBACK(intersection_response),
                    NULL
                );
                
                //Delete Char*
                delete [] int_name;
                delete [] lable_info;
            }

            previous_id = id;

            //refreshes drawing to redraw map with highlighted intersection
            app->refresh_drawing();
        }
    }
}


//Do nothing, just destroy the dialog
void intersection_response(GtkDialog *dialog){
    gtk_widget_destroy(GTK_WIDGET (dialog));
}


//Used to take street inputs from the user and display the generated path
void find_button(GtkWidget */*widget*/, ezgl::application *application){
    //get the switch from the UI
    GtkSwitch *switch_dw = (GtkSwitch*) application->get_object("drive_vs_walk");

    drive_vs_walk = gtk_switch_get_state(switch_dw);

    //Update the walk speed and time when the find path text button is clicked
    if(drive_vs_walk){
        GtkWidget *window, *field_box_height, *field_box_width, *find, *close;
        gint height = 350;
        gint width = 100;

        //Create a tool tip

        //Create a new gtk window on the top level, this will hold all the other
        //elements we want to implement...
        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

        //Sets the size of the window
        gtk_widget_set_size_request(GTK_WIDGET(window), height, width);

        //Sets the title of the window
        gtk_window_set_title(GTK_WINDOW(window), "Enter Walk speed and time limit");

        //Close the window 
        g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

        //Now create a box which will eventually hold the search fields
        field_box_height = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

        //Add this box to the window
        gtk_container_add(GTK_CONTAINER(window), field_box_height);

        //Show this overall window
        gtk_widget_show(field_box_height);

        //Creates a new entry which will be used for searching an intersection
        //However, we have to allocate memory before we can do this...
        //Also, later when we call back to ints_of_two_streets we need to point to
        //our application so make data->app point to our application
        search_results *data = g_new0(search_results, 2);
        data->app = application;
        data->search_box_1 = gtk_entry_new();
        gtk_entry_set_text(GTK_ENTRY(data->search_box_1), "Walk speed");

        //Adds search_box_1 to the fileld_box because we want the search bars to
        //Be stacked one on top of another...
        gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_1, true, true, 1);

        gtk_widget_set_tooltip_text(data->search_box_1, "Type the walk speed in m/s");
        //Show this widget
        gtk_widget_show(data->search_box_1);


        //Do the exact same thing but for another search box...
        data->search_box_2 = gtk_entry_new();
        gtk_entry_set_text(GTK_ENTRY(data->search_box_2), "Walking time limit");
        gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_2, true, true, 1);

        gtk_widget_set_tooltip_text(data->search_box_2, "Type the walk limit in seconds");
        gtk_widget_show(data->search_box_2);
        
        //Ok now create another box similar to filed_box_height, but oriented horizontally
        //However, GTKWindow can only contain one widget, so field_box_width needs to
        //be added to the field_box_height container instead
        field_box_width = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
        gtk_container_add(GTK_CONTAINER(field_box_height), field_box_width);
        gtk_widget_show(field_box_width);

        //Now we need to create the buttons that will be used to actually do the
        //searching/interaction...
        find = gtk_button_new_with_label("Find path between intersections");

        //When this button gets clicked we need to use the find_intersections_
        //of_two_streets subroutine defined in m1.cpp
        g_signal_connect(find, "clicked", G_CALLBACK(walk_parameters), data);
        
        //Now add the button to the box so that it's layer underneath the two search boxes
        gtk_box_pack_start(GTK_BOX(field_box_height), find, true, true, 5);
        gtk_widget_set_can_default(find, true);
        gtk_widget_grab_default(find); 

        //Now show the buttton
        gtk_widget_show(find);

        //Now create a new button to close the window...
        close = gtk_button_new_with_label("Close");
        g_signal_connect(close, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(window));
        gtk_box_pack_start(GTK_BOX(field_box_height), close, true, true, 5);
        gtk_widget_show(close);

        //Finaly we can show the entier window...
        gtk_widget_show(window);

        gtk_main();
    }
    
    //We want to create a widget that has two search boxes and a cancel and find
    //button. First we need to create a window, two search boxes (gtk_entry) and
    //buttons for interaction...
    
    GtkWidget *window, *field_box_height, *field_box_width, *find, *close;
    gint height = 450;
    gint width = 100;
    
    //Create a tool tip
    
    //Create a new gtk window on the top level, this will hold all the other
    //elements we want to implement...
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    
    //Sets the size of the window
    gtk_widget_set_size_request(GTK_WIDGET(window), height, width);
   
    //Sets the title of the window
    gtk_window_set_title(GTK_WINDOW(window), "Find path between 2 intersections");
    
    //Close the window 
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    
    //Now create a box which will eventually hold the search fields
    field_box_height = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    
    //Add this box to the window
    gtk_container_add(GTK_CONTAINER(window), field_box_height);
    
    //Show this overall window
    gtk_widget_show(field_box_height);
    
    //Creates a new entry which will be used for searching an intersection
    //However, we have to allocate memory before we can do this...
    //Also, later when we call back to ints_of_two_streets we need to point to
    //our application so make data->app point to our application
    search_results *data = g_new0(search_results, 2);
    data->app = application;
    data->search_box_1 = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->search_box_1), "Street 1");
    
    //Adds search_box_1 to the fileld_box because we want the search bars to
    //Be stacked one on top of another...
    gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_1, true, true, 1);
    
    gtk_widget_set_tooltip_text(data->search_box_1, "Type the street of the first intersection");
    //Show this widget
    gtk_widget_show(data->search_box_1);
    
    
    //Do the exact same thing but for another search box...
    data->search_box_2 = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->search_box_2), "Street 2");
    gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_2, true, true, 1);
    
    gtk_widget_set_tooltip_text(data->search_box_2, "Type the street of the second intersection");
    gtk_widget_show(data->search_box_2);
    
    
    //Do the exact same thing but for another search box...
    data->search_box_3 = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->search_box_3), "Street 3");
    gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_3, true, true, 1);
    
    gtk_widget_set_tooltip_text(data->search_box_3, "Type the street of the third intersection");
    gtk_widget_show(data->search_box_3);
    
    
    //Do the exact same thing but for another search box...
    data->search_box_4 = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->search_box_4), "Street 4");
    gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_4, true, true, 1);
    
    gtk_widget_set_tooltip_text(data->search_box_4, "Type the street of the fourth intersection");
    gtk_widget_show(data->search_box_4);
    
    
    //Ok now create another box similar to filed_box_height, but oriented horizontally
    //However, GTKWindow can only contain one widget, so field_box_width needs to
    //be added to the field_box_height container instead
    field_box_width = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_add(GTK_CONTAINER(field_box_height), field_box_width);
    gtk_widget_show(field_box_width);
    
    //Now we need to create the buttons that will be used to actually do the
    //searching/interaction...
    find = gtk_button_new_with_label("Find path between intersections");
    
    //When this button gets clicked we need to use the find_intersections_
    //of_two_streets subroutine defined in m1.cpp
    g_signal_connect(find, "clicked", G_CALLBACK(text_path_find), data);
    
    if(ERROR_DETECT){
        gtk_window_close(GTK_WINDOW(window));
        
        char error[] = "Error";
        char lable_info[] = "Uh-oh looks like we could't find what you were looking for :(";
        
        // BEGIN: CODE FOR SHOWING DIALOG
        GObject *window_2;
        GtkWidget *content_area;
        GtkWidget *label;
        GtkWidget *dialog;

        // get a pointer to the main application window
        window_2 = data->app->get_object(data->app->get_main_window_id().c_str());

        dialog = gtk_dialog_new_with_buttons(
        error,
        (GtkWindow*) window_2,
        GTK_DIALOG_MODAL,
        ("OK"),
        GTK_RESPONSE_ACCEPT,
        NULL,
        GTK_RESPONSE_REJECT,
        NULL
        );

        // Create a label and attach it to the content area of the dialog
        content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
        label = gtk_label_new(lable_info);
        gtk_container_add(GTK_CONTAINER(content_area), label);

        // The main purpose of this is to show dialog’s child widget, label
        gtk_widget_show_all(dialog);

        g_signal_connect(
        GTK_DIALOG(dialog),
        "response",
        G_CALLBACK(intersection_response),
        NULL
        );
        
        ERROR_DETECT = false;
    }
    
    //Now add the button to the box so that it's layer underneath the two search boxes
    gtk_box_pack_start(GTK_BOX(field_box_height), find, true, true, 5);
    gtk_widget_set_can_default(find, true);
    gtk_widget_grab_default(find); 
    
    //Now show the buttton
    gtk_widget_show(find);
    
    //Now create a new button to close the window...
    close = gtk_button_new_with_label("Close");
    g_signal_connect(close, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(window));
    gtk_box_pack_start(GTK_BOX(field_box_height), close, true, true, 5);
    gtk_widget_show(close);
    
    //Finaly we can show the entier window...
    gtk_widget_show(window);
    
    gtk_main();   
}


//Finds the shared intersections of streets, as well as partial ids
void text_path_find(GtkWidget *, search_results *data){
    //Here is where we need to parse the data from find button in order
    //To find all the intersections between two inputted streets...
    const gchar *g_entry_1 = gtk_entry_get_text(GTK_ENTRY(data->search_box_1));
    const gchar *g_entry_2 = gtk_entry_get_text(GTK_ENTRY(data->search_box_2));
    const gchar *g_entry_3 = gtk_entry_get_text(GTK_ENTRY(data->search_box_3));
    const gchar *g_entry_4 = gtk_entry_get_text(GTK_ENTRY(data->search_box_4));
    
    std::string street_1 = g_entry_1;
    std::string street_2 = g_entry_2;
    std::string street_3 = g_entry_3;
    std::string street_4 = g_entry_4;
    
    if(street_1.empty() || street_2.empty() || street_3.empty() || street_4.empty()){
        ERROR_DETECT = true;
        return;
    }
    
    //Get potential streets
    std::vector<int> potential_streets_1 = find_street_ids_from_partial_street_name(street_1);
    std::vector<int> potential_streets_2 = find_street_ids_from_partial_street_name(street_2);
    std::vector<int> potential_streets_3 = find_street_ids_from_partial_street_name(street_3);
    std::vector<int> potential_streets_4 = find_street_ids_from_partial_street_name(street_4);
    
    if(potential_streets_1.empty() || potential_streets_2.empty() || potential_streets_3.empty() || potential_streets_4.empty()){
        ERROR_DETECT = true;
        return;
    }
    
    IntersectionIndex shared_intersections_1 = -1;
    IntersectionIndex shared_intersections_2 = -1;
    int flag = 0;
    
    //Get potential intersections for first set of streets
    for(int i = 0; (i < potential_streets_1.size()) && flag == 0; i++){
        
        for(int j = 0; (j < potential_streets_2.size()) && flag == 0; j++){
            std::pair<int, int> street_ids(potential_streets_1[i], potential_streets_2[j]);
            std::vector<int> shared_ints = find_intersections_of_two_streets(street_ids);
            
            if(!shared_ints.empty()){
                shared_intersections_1 = shared_ints[0];
                flag = 1;
                break;
            }
        }
    }
    
    flag = 0;
    
    //Get potential intersections for second set of streets
    for(int i = 0; (i < potential_streets_3.size()) && flag == 0; i++){
        
        for(int j = 0; (j < potential_streets_4.size()) && flag == 0; j++){
            std::pair<int, int> street_ids(potential_streets_3[i], potential_streets_4[j]);
            std::vector<int> shared_ints = find_intersections_of_two_streets(street_ids);
            
            if(!shared_ints.empty()){
                shared_intersections_2 = shared_ints[0];
                flag = 1;
                break;
            }
        }
    }
    
    if(shared_intersections_1 == -1 || shared_intersections_2 == -1){
        ERROR_DETECT = true;
        return;
    }
    
    first_int_search = shared_intersections_1;
    second_int_search = shared_intersections_2;
    
    if(!drive_vs_walk){
        find_path_between_intersections(shared_intersections_1, shared_intersections_2, 15);
    }
    else if(drive_vs_walk){
        find_path_with_walk_to_pick_up(first_int_search, second_int_search, 15.000, walk_speed, walk_time);
    }
    
    //draw_directions();
    
    PATH_TOGGLE = true;
    
    data->app->refresh_drawing();
}


//Find the zoom amount with respect to the original world size
void get_zoom(long double current_area){
    
    //Changes the zoom level based on visible world area
    if(current_area == INITIAL_AREA){
        ZOOM = 10;
        prev_area = current_area;
        return;
    }
    else if(current_area > prev_area){
        ZOOM--;
        prev_area = current_area;
        return;
    }
    else if(current_area < prev_area){
        ZOOM++;
        prev_area = current_area;
        return;
    }
}


//Toggles viewing pois
void poi_button(GtkWidget */*widget*/, ezgl::application *application){
    
    if(!POI_TOGGLE){
        POI_TOGGLE = true;
        application->update_message("Drawing points of interest...");
        application->refresh_drawing();        
    }
    else{
        POI_TOGGLE = false;
        application->refresh_drawing();
        application->update_message("Removing points of interest");
    }
    
}


//Draws the points of interests using dots
void draw_points_of_interest(ezgl::renderer *g){

    for(int i = 0; i < pois.size(); i++){
        ezgl::point2d draw_loc = latlon_to_point(pois[i].pos);
        
        if(pois[i].highlight){
            g->set_color(ezgl::MAGENTA);
            g->fill_elliptic_arc(draw_loc, 0.0008, 0.0008, 180, 360);
            //g->set_color(ezgl::BLACK);
            //g->set_font_size(0.025*pow(ZOOM, 2));
            //g->draw_text(draw_loc, pois[i].name);
        }
        else{
            g->set_color(ezgl::PINK);
            g->fill_elliptic_arc(draw_loc, 0.0007, 0.0007, 0, 360);
        }

        
        
    }
}


//Returns the nearest intersection to the given position
int find_closest_poi(LatLon my_position) {
    
    //Establishes a comparison point by setting the closest intersection to 
    //be intersection 0 to start
    std::pair<LatLon, LatLon> poi_0(pois[0].pos, my_position);
    double shortest_distance = find_distance_between_two_points(poi_0);
    int closest_poi = 0;
    
    //loops through the rest of the intersections
    for (int i = 1; i < getNumPointsOfInterest(); i++) {
        std::pair<LatLon, LatLon> check_dist(pois[i].pos, my_position);
        double distance = find_distance_between_two_points(check_dist);
        //checks if the intersection is closer than the current closest 
        //intersection
        if (distance <= shortest_distance) {
            shortest_distance = distance;
            closest_poi = i;         
        }
        
    }
    return closest_poi;
}


//Matches street name to search value
void search_street(GtkWidget *, search_results *data) {
    //removing previously highlighted streets
    for (int i = 0; i < getNumStreets(); i++) {
        streets[i].highlight = false;
    }
    
    //finding streets to highlight
    const gchar *g_entry_1 = gtk_entry_get_text(GTK_ENTRY(data->search_box_1));
    std::string search = g_entry_1;
    std::vector<int> streets_found = find_street_ids_from_partial_street_name(search);
    
    
    //highlighting streets
    for (int i = 0; i < streets_found.size(); i++) {
        streets[streets_found[i]].highlight = true;
    }
    data->app->refresh_drawing();
}


//Creates a button with a search box for finding streets
void street_button(GtkWidget */*widget*/, ezgl::application *application) {
    //We want to create a widget that has two search boxes and a cancel and find
    //button. First we need to create a window, two search boxes (gtk_entry) and
    //buttons for interaction...

    GtkWidget *window, *field_box_height, *field_box_width, *find, *close;
    gint height = 50;
    gint width = 100;

    //Create a new gtk window on the top level, this will hold all the other
    //elements we want to implement...
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    //Sets the size of the window
    gtk_widget_set_size_request(GTK_WIDGET(window), height, width);

    //Sets the title of the window
    gtk_window_set_title(GTK_WINDOW(window), "Search");

    //Close the window 
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    //Now create a box which will eventually hold the search fields
    field_box_height = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

    //Add this box to the window
    gtk_container_add(GTK_CONTAINER(window), field_box_height);

    //Show this overall window
    gtk_widget_show(field_box_height);

    //Creates a new entry which will be used for searching an intersection
    //However, we have to allocate memory before we can do this...
    //Also, later when we call back to ints_of_two_streets we need to point to
    //our application so make data->app point to our application
    search_results *data = g_new0(search_results, 1);
    data->app = application;
    data->search_box_1 = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->search_box_1), "");

    //Adds search_box_1 to the fileld_box because we want the search bars to
    //Be stacked one on top of another...
    gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_1, true, true, 1);

    //Show this widget
    gtk_widget_show(data->search_box_1);

    //Ok now create another box similar to filed_box_height, but oriented horizontally
    //However, GTKWindow can only contain one widget, so field_box_width needs to
    //be added to the field_box_height container instead
    field_box_width = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_add(GTK_CONTAINER(field_box_height), field_box_width);
    gtk_widget_show(field_box_width);


    //Now we need to create the buttons that will be used to actually do the
    //searching/interaction...
    find = gtk_button_new_with_label("Find Streets");

    //When this button gets clicked we need to use the find_intersections_
    //of_two_streets subroutine defined in m1.cpp
    g_signal_connect(find, "clicked", G_CALLBACK(search_street), data);

    //Now add the button to the box so that it's layer underneath the two search boxes
    gtk_box_pack_start(GTK_BOX(field_box_height), find, true, true, 5);
    gtk_widget_set_can_default(find, true);
    gtk_widget_grab_default(find);

    //Now show the buttton
    gtk_widget_show(find);

    //Now create a new button to close the window...
    close = gtk_button_new_with_label("Close");
    g_signal_connect(close, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(window));
    gtk_box_pack_start(GTK_BOX(field_box_height), close, true, true, 5);
    gtk_widget_show(close);

    //Finaly we can show the entier window...
    gtk_widget_show(window);

    gtk_main();
}


//Drawing the main ui for the weather data
void draw_weather(ezgl::renderer *g){
    g->set_coordinate_system(ezgl::SCREEN);
    
    //Drawing the container
    ezgl::point2d xy(g->get_visible_screen().top_right());
    ezgl::point2d xy2(g->get_visible_screen().center());
    
    ezgl::point2d scale_center(1.4, 1.5);
    
    ezgl::point2d border(1,1);
   
    
    xy2 = xy2*scale_center;
    
    ezgl::rectangle container(xy2, xy);
    ezgl::point2d container_center = container.center();
    
    
    g->set_color(ezgl::GREY_55);
    g->fill_rectangle(xy, (xy2 - border));
    
    g->set_color(ezgl::WHITE);
    g->fill_rectangle(container);
    //~~~~~~~~~~~~~~~~~~~~~
    
    //Drawing city name
    g->set_color(ezgl::BLACK);
    g->set_text_rotation(0);
    g->set_font_size(SCREEN_ZOOM + 20);
    g->format_font("sans-serif", ezgl::font_slant::normal, ezgl::font_weight::normal);
    
    ezgl::point2d text_scale(0.5, 0.45);
    
    ezgl::point2d text_place = ((xy+xy2) * text_scale);
    
    g->draw_text(text_place, city.name);
    //~~~~~~~~~~~~~~~~~~
    
    //Drawing weather icon    
    ezgl::point2d png_place(xy2.x - 14 + SCREEN_ZOOM, container_center.y -50);
    
    const char *weather_icon = surface_to_render();
    
    ezgl::surface *png_surface = ezgl::renderer::load_png(weather_icon);
    g->draw_surface(png_surface, png_place);
    ezgl::renderer::free_surface(png_surface);
    //~~~~~~~~~~~~~~~~~~~~~~
    
    //Drawing the main temperature
    g->set_horiz_text_just(ezgl::text_just::center);
    g->set_color(ezgl::BLACK);
    g->set_font_size(SCREEN_ZOOM*3 + 8);
    
    int rounded_temp = city.main_temperature;
    
    std::string temperature = "It's " + std::to_string(rounded_temp) + " °C outside";
    
    g->draw_text( container_center + ezgl::point2d{19, -5} ,temperature);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //Drawing the feels like temperature 
    int rounded_feel = city.main_feels_like;
    
    std::string feels_temperature = "It feels like " + std::to_string(rounded_feel) + " °C";
    
    g->draw_text(container_center + ezgl::point2d{19, 15}, feels_temperature);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //Drawing the wind direction 
    std::string direction = deg_to_compass();
    int rounded_speed = city.wind_speed;
    std::string wind_info = "Wind: " + std::to_string(rounded_speed) + " m/s " + direction;
    
    g->draw_text(container_center + ezgl::point2d{19, 55 + SCREEN_ZOOM}, wind_info);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    std::string description = city.weather_description;
    description[0] = std::toupper(description[0]);
    
    g->draw_text(container_center + ezgl::point2d{19, -24}, description);
    
    //Drawing the wind arrow    
    g->set_font_size(30 + SCREEN_ZOOM*1.2);
    g->set_text_rotation(360 - city.wind_deg);
    g->draw_text(ezgl::point2d{xy2.x + 40 + SCREEN_ZOOM, container_center.y + 55 + SCREEN_ZOOM}, "↑");
    //~~~~~~~~~~~~~~~~~~~~~~~~
    
    g->set_coordinate_system(ezgl::WORLD);
}


//Decides which png to render based on weather info
const char* surface_to_render(){
    if(city.weather_main == "Thunder Storm"){
        return "libstreetmap/resources/thunder_storm.png";
    }
    else if(city.weather_main == "Drizzle"){
        return "libstreetmap/resources/shower_rain.png";
    }
    else if(city.weather_main == "Rain"){
        return "libstreetmap/resources/light_rain.png";
    }
    else if(city.weather_main == "Snow"){
        return "libstreetmap/resources/snow.png";
    }
    else if(city.weather_main == "Clouds"){
        
        if(city.weather_description == "few clouds")
            return "libstreetmap/resources/few_clouds.png";
        
        if(city.weather_description == "scattered clouds")
            return "libstreetmap/resources/scattered_clouds.png";
        
        if(city.weather_description == "broken clouds")
            return "libstreetmap/resources/broken_clouds.png";
 
        if(city.weather_description == "overcast clouds")
            return "libstreetmap/resources/broken_clouds.png";
        
    }
    else if(city.weather_main == "Clear"){
        return "libstreetmap/resources/clear_sky.png";
    }
    else{
        return "libstreetmap/resources/mist.png";
    }
    
    return "libstreetmap/resources/clear_sky.png";
}


//Converts degres to compass coordinates
const char* deg_to_compass(){
    if(city.wind_deg > 348.75 && city.wind_deg <= 11.25){
        return "N";
    }
    else if(city.wind_deg > 11.25 && city.wind_deg <= 33.75){
        return "NNE";
    }
    else if(city.wind_deg > 33.75 && city.wind_deg <= 56.25){
        return "NE";
    }
    else if(city.wind_deg > 56.25 && city.wind_deg <= 78.75){
        return "ENE";
    }
    else if(city.wind_deg > 78.75 && city.wind_deg <= 101.25){
        return "E";
    }
    else if(city.wind_deg > 101.25 && city.wind_deg <= 123.75){
        return "ESE";
    }
    else if(city.wind_deg > 123.75 && city.wind_deg <= 146.25){
        return "SE";
    }
    else if(city.wind_deg > 146.25 && city.wind_deg <= 168.75){
        return "SSE";
    }
    else if(city.wind_deg > 168.75 && city.wind_deg <= 191.25){
        return "S";
    }
    else if(city.wind_deg > 191.25 && city.wind_deg <= 213.75){
        return "SSW";
    }
    else if(city.wind_deg > 213.75 && city.wind_deg <= 236.25){
        return "SW";
    }
    else if(city.wind_deg > 236.25 && city.wind_deg <= 258.75){
        return "WSW";
    }
    else if(city.wind_deg > 258.75 && city.wind_deg <= 281.25){
        return "W";
    }
    else if(city.wind_deg > 281.25 && city.wind_deg <= 303.75){
        return "WNW";
    }
    else if(city.wind_deg > 303.75 && city.wind_deg <= 326.25){
        return "NW";
    }
    else if(city.wind_deg > 326.25 && city.wind_deg <= 348.75){
        return "NNW";
    }
    else{
        return "N";
    }
}


//This button is used to select a new city to load
void city_select(GtkWidget */*widget*/, ezgl::application *application){
    GtkComboBoxText* city_box = (GtkComboBoxText*) application->get_object("CitySelect");
    
    
    const gchar* city_selected = gtk_combo_box_text_get_active_text(city_box);
    
    //Load a new map based on the text box
    std::string new_map = path(city_selected);
    
    //Clear all the data structures 
    intersections.clear();
    pois.clear();
    streets.clear();
    
    close_map();
    
    //Open the new map and load new data...
    load_map(new_map);
    
    
    //stores all intersections and change max/min lat/long accordingly
    intersections.resize(getNumIntersections());
    for (int i = 0; i < getNumIntersections(); i++) {
        intersections[i].position = getIntersectionPosition(i);
        intersections[i].name = getIntersectionName(i);
        intersections[i].cartesian = {SCALE * lon_to_x(getIntersectionPosition(i).lon()), SCALE * lat_to_y(getIntersectionPosition(i).lat())};
    }

    pois.resize(getNumPointsOfInterest());
    for(int i = 0; i <getNumPointsOfInterest(); i++){

        pois[i].type = getPointOfInterestType(i);
        pois[i].name = getPointOfInterestName(i);
        pois[i].pos = getPointOfInterestPosition(i);
    }

    streets.resize(getNumStreets());
    for(int i =0; i< getNumStreets(); i++) {
        streets[i].name = getStreetName(i);
    }     
    
    //Setting up the world view for the map, using a scaling factor 
    //of 1000
    double minx = lon_to_x(MINLON);
    double maxx = lon_to_x(MAXLON);
    double miny = lat_to_y(MINLAT);
    double maxy = lat_to_y(MAXLAT);
    
    ezgl::rectangle initial_world({SCALE * minx, SCALE * miny}, {SCALE * maxx, SCALE * maxy});
    
    application->change_canvas_world_coordinates("MainCanvas", initial_world);
    
    area_start = 0;
    
    application->refresh_drawing();
}


//Used for searching for the path
void path_button(GtkWidget */*widget*/, ezgl::application *application){
    //get the switch from the UI
    GtkSwitch *switch_dw = (GtkSwitch*) application->get_object("drive_vs_walk");

    drive_vs_walk = gtk_switch_get_state(switch_dw);
    
    //if the 
    if(drive_vs_walk && !PATH_TOGGLE){
        GtkWidget *window, *field_box_height, *field_box_width, *find, *close;
        gint height = 350;
        gint width = 100;

        //Create a tool tip

        //Create a new gtk window on the top level, this will hold all the other
        //elements we want to implement...
        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

        //Sets the size of the window
        gtk_widget_set_size_request(GTK_WIDGET(window), height, width);

        //Sets the title of the window
        gtk_window_set_title(GTK_WINDOW(window), "Enter Walk speed and time limit");

        //Close the window 
        g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

        //Now create a box which will eventually hold the search fields
        field_box_height = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

        //Add this box to the window
        gtk_container_add(GTK_CONTAINER(window), field_box_height);

        //Show this overall window
        gtk_widget_show(field_box_height);

        //Creates a new entry which will be used for searching an intersection
        //However, we have to allocate memory before we can do this...
        //Also, later when we call back to ints_of_two_streets we need to point to
        //our application so make data->app point to our application
        search_results *data = g_new0(search_results, 2);
        data->app = application;
        data->search_box_1 = gtk_entry_new();
        gtk_entry_set_text(GTK_ENTRY(data->search_box_1), "Walk speed");

        //Adds search_box_1 to the fileld_box because we want the search bars to
        //Be stacked one on top of another...
        gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_1, true, true, 1);

        gtk_widget_set_tooltip_text(data->search_box_1, "Type the walk speed in m/s");
        //Show this widget
        gtk_widget_show(data->search_box_1);


        //Do the exact same thing but for another search box...
        data->search_box_2 = gtk_entry_new();
        gtk_entry_set_text(GTK_ENTRY(data->search_box_2), "Walking time limit");
        gtk_box_pack_start(GTK_BOX(field_box_height), data->search_box_2, true, true, 1);

        gtk_widget_set_tooltip_text(data->search_box_2, "Type the walk limit in seconds");
        gtk_widget_show(data->search_box_2);
        
        //Ok now create another box similar to filed_box_height, but oriented horizontally
        //However, GTKWindow can only contain one widget, so field_box_width needs to
        //be added to the field_box_height container instead
        field_box_width = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
        gtk_container_add(GTK_CONTAINER(field_box_height), field_box_width);
        gtk_widget_show(field_box_width);

        //Now we need to create the buttons that will be used to actually do the
        //searching/interaction...
        find = gtk_button_new_with_label("Find path between intersections");

        //When this button gets clicked we need to use the find_intersections_
        //of_two_streets subroutine defined in m1.cpp
        g_signal_connect(find, "clicked", G_CALLBACK(walk_parameters), data);
        
        //Now add the button to the box so that it's layer underneath the two search boxes
        gtk_box_pack_start(GTK_BOX(field_box_height), find, true, true, 5);
        gtk_widget_set_can_default(find, true);
        gtk_widget_grab_default(find); 

        //Now show the buttton
        gtk_widget_show(find);

        //Now create a new button to close the window...
        close = gtk_button_new_with_label("Close");
        g_signal_connect(close, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(window));
        gtk_box_pack_start(GTK_BOX(field_box_height), close, true, true, 5);
        gtk_widget_show(close);

        //Finaly we can show the entier window...
        gtk_widget_show(window);

        gtk_main();  
    }
    
    if(!PATH_TOGGLE){
        PATH_TOGGLE = true;
        application->update_message("Click on intersections!");
        application->refresh_drawing();        
    }
    else{
        PATH_TOGGLE = false;
        application->refresh_drawing();
        application->update_message("Did you find your way");
    }
}


//Feature name toggle button
void feature_name_button(GtkWidget */*widget*/, ezgl::application *application){
    if(!FEATURE_NAME_TOGGLE){
        FEATURE_NAME_TOGGLE = true;
        application->update_message("Turning on feature names!");
        application->refresh_drawing();        
    }
    else{
        FEATURE_NAME_TOGGLE = false;
        application->refresh_drawing();
        application->update_message("Turning off feature names...");
    }
}


//Sets the parameters for the walking speed and walking time limit
void walk_parameters(GtkWidget *, search_results *data){
    const gchar *g_entry_1 = gtk_entry_get_text(GTK_ENTRY(data->search_box_1));
    const gchar *g_entry_2 = gtk_entry_get_text(GTK_ENTRY(data->search_box_2));
    
    std::stringstream get_entry_1(g_entry_1);
    std::stringstream get_entry_2(g_entry_2);
    
    //Save the walk speed and time into global variables 
    get_entry_1 >> walk_speed;
    get_entry_2 >> walk_time;
    
    
    return;
}