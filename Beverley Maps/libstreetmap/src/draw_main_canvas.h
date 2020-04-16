/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   draw_main_canvas.h
 * Author: ragubanc
 *
 * Created on February 21, 2020, 6:17 PM
 */

#ifndef DRAW_MAIN_CANVAS_H
#define DRAW_MAIN_CANVAS_H
#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/point.hpp"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "GlobalVariables.h"
#include "ezgl/point.hpp"
#include "draw_main_canvas.h"
#include "cartesian.h"
#include "map_path.h"
#include "directions.h"

//This structure is used as the data for the callback function for the find button
typedef struct {
    GtkWidget *search_box_1;
    GtkWidget *search_box_2;
    GtkWidget *search_box_3;
    GtkWidget *search_box_4;
    ezgl::application *app;
    ezgl::renderer *g;
}search_results;

//Initial setup for the main ui window
void initial_setup(ezgl::application *application, bool new_window);

//Makes tiny circles for all the intersections of a map
void draw_intersections(ezgl::renderer* g);

//Makes lines connecting intersections and curved roads
void draw_street_segments(ezgl::renderer* g);

//
void draw_street(int i, ezgl::renderer *g, ezgl::point2d from, ezgl::point2d to, InfoStreetSegment segment_info);

// Writes the name of the street on the road
void write_street_name(int i, InfoStreetSegment segment_info, ezgl::renderer *g);

//Draws all the different features (parks, etc...)
void draw_features(ezgl::renderer* g);

//Draws a closed or open? polygon, called in draw_features
void draw_poly(ezgl::renderer *g, int feature_id, int point_count);

//
void draw_feature_name(ezgl::renderer *g, int feature_id, int point_count);

//reacts to user mouse click input
void act_on_mouse_click(ezgl::application *app, GdkEventButton* event, double x, double y);

//
void intersection_response(GtkDialog *dialog);

//Used to take street inputs from the user and display the shared intersections
void find_button(GtkWidget *widget, ezgl::application *application);

//Finds the shared intersections of streets, as well as partial ids
void text_path_find(GtkWidget *, search_results *data);

//Gets the zoom of the world based on the initial conditions...
void get_zoom(long double current_area);

//Button for toggling poi dots
void poi_button(GtkWidget */*widget*/, ezgl::application *application);

//Draws points of interests 
void draw_points_of_interest(ezgl::renderer *g);

//Find the closests POI based on mouse location
int find_closest_poi(LatLon my_position); 

//Draws the UI for the weaher
void draw_weather(ezgl::renderer *g);

//Choses a png to display for weather
const char* surface_to_render();

//Converts degrees to compass directions
const char* deg_to_compass();

//Find a street
void search_street(GtkWidget *, search_results *data);

//Button created for finding streets
void street_button(GtkWidget */*widget*/, ezgl::application *application);

//Button used to load a new map
void city_select(GtkWidget */*widget*/, ezgl::application *application);

//Button used to input a path
void path_button(GtkWidget */*widget*/, ezgl::application *application);

//Button used to turn on and off the feature names
void feature_name_button(GtkWidget */*widget*/, ezgl::application *application);

//
void walk_parameters(GtkWidget *, search_results *data);
#endif /* DRAW_MAIN_CANVAS_H */

