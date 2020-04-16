/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GlobalVariables.h
 * Author: ragubanc
 *
 * Created on February 21, 2020, 12:45 PM
 */

#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#include "m1.h"
#include "m4.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "ezgl/point.hpp"
#include "cartesian.h"
#include "draw_main_canvas.h"
#include <vector>
#include <string>
#include <cmath>
#include <utility>
#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <set>
#include <bits/stdc++.h>
#include <queue>

/*~~~~~Global Variables~~~~~*/
extern double MINLON;
extern double MINLAT;
extern double MAXLON;
extern double street;
extern double MAXLAT;
extern double LARGE;
extern double SMALL;
extern int SCALE;
extern long double INITIAL_AREA;
extern long double INITIAL_SCREEN;
extern int ZOOM;
extern double SCREEN_ZOOM;
extern double MAXSTREETLENGTH;
extern bool POI_TOGGLE;
extern std::string map_path;
extern int area_start;
extern std::string elevation_info;
extern bool PATH_TOGGLE;
extern bool FEATURE_NAME_TOGGLE;
extern int click_count;
extern int second_int_search;
extern int first_int_search;
extern bool drive_vs_walk;
extern double walk_speed;
extern double walk_time;
extern double time_to_beat;
extern float max_weight;


struct delivery_item {
    //the id for the delivery
    int delivery_id;
    // if the intersection is a pickup or drop
    bool pickup;
};
//Node class for identifying nodes in a graph
class node {
    public:
        //The best time to reach this node
        double cost;
        
        //The outgoing edges of this node
        int outEdges;

        //ID of segment used to reach this node
        StreetSegmentIndex reachingEdge;
        
        //The intersection id of the given node
        IntersectionIndex node_ID;
        
        //Used for creating new nodes, used in waveElenents
        node(IntersectionIndex id, double heuristic, StreetSegmentIndex from_segment, int num_edges){
            node_ID = id;
            cost = heuristic;
            reachingEdge = from_segment;
            outEdges = num_edges;
        }
        
        node(){
            node_ID = 0;
        }
        
        node(const node & source) {
            node_ID = source.node_ID;
            cost = source.cost;
            reachingEdge = source.reachingEdge;
            outEdges = source.outEdges;
        }
        
        bool operator<(const node & n){
            return cost < n.cost;
        }
        
        void operator=(const node & source){
            node_ID = source.node_ID;
            cost = source.cost;
            reachingEdge = source.reachingEdge;
            outEdges = source.outEdges;
        }
        
        ~node(){
            
        }
};


//Static strucure for the waveFront
struct waveElements{
    //Time it takes to reach this element
    double travel_time;
    
    //ID of the street used to reach the current element
    int connectingEdge;
    
    //Current intersection node
    node *current;
    
    //Distance to the final destination
    double distance;
    
    //Heuristic
    double heuristic;
    
    waveElements(node* n, int connect, double time, double dist, double heur){
        current = n; 
        connectingEdge = connect;
        travel_time = time;
        distance = dist;
        heuristic = heur;
    }
    
    ~waveElements() {

    }
   
};


//Sorting the waveElements based on travel_tine, this will eventually need to
//be changed to a more complex compairison algorithm.
class compare{
    public:
        int operator () (const waveElements & w1, const waveElements &w2){
            return w1.heuristic >= w2.heuristic;
        }
};

struct intersection_data {
    LatLon position;
    std::string name;
    std::pair<double, double> cartesian;
    bool highlight = false;
};

struct POI_data {
    std::string type;
    std::string name;
    LatLon pos;
    bool highlight = false;
};

struct street_data {
    std::string name;
    bool highlight = false;
    
   
};

struct weather {
    std::string weather_main;
    std::string weather_description;
    std::string name;
    double main_temperature;
    double main_feels_like;
    double wind_speed;
    double wind_deg;
};

typedef struct MyCustomStruct {
    char *url = nullptr;
    unsigned int size = 0;
    char *response = nullptr;
} MyCustomStruct;


/*~~~~~Declare global data structures~~~~~*/
extern std::vector<std::vector<LatLon>> array_of_curve_points;

extern std::vector<std::pair<double, double>> cartesian_of_intersections;

extern std::vector<std::vector<int>> street_segments_of_intersection;

extern std::vector<InfoStreetSegment> street_segment_info;

extern std::vector<std::vector<int>> street_segments_of_street;

extern std::vector<LatLon> intersection_coordinates;

extern std::vector<double> travel_time_of_intersections;

extern std::vector<std::vector<int>> intersections_on_street;

extern std::multimap<std::string, int> street_names_nospaces;

extern std::vector<std::vector<InfoStreetSegment>> info_of_segments_of_street;

extern std::vector<int> segments_per_street;

extern std::vector<double> street_segment_length;

extern std::vector<double> street_length;

extern std::unordered_map<OSMID, double> way_length;

extern std::unordered_map<OSMID, const OSMNode*> node_location;

extern std::unordered_map<OSMID, std::vector<const OSMNode*>> nodes_of_OSM_way;

extern std::vector<std::vector<std::pair<double, double>>> feature_cartesian_points;

extern std::vector<std::vector<ezgl::point2d>> points_of_feature;

extern std::vector<double> feature_area;

extern std::vector<intersection_data> intersections;

extern std::vector<POI_data> pois;

extern std::vector<street_data> streets;

extern weather city;

extern std::string URL_altitude;

extern std::vector<node> node_graph;

extern std::vector<StreetSegmentIndex> shortest_path;

extern std::vector<StreetSegmentIndex> walk_path;

extern double max_speed_limit;

extern std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> walk_drive_path;

extern std::vector<double> lat_avg;

extern std::vector<DeliveryInfo> global_deliveries;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


#endif /* GLOBALVARIABLES_H */

