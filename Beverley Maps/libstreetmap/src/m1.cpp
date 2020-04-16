/* 
 * Copyright 2020 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*This file is pretty big and contains a lot of stuff, so let's break it down...
 * -Load map essentially creates all the data structures needed for everything else
 * in this file to work even remotely
 * -Close map gets rid of all the data structure
 * - Every other function basically extracts data from one of the created data
 * structures or applies calculations to the data structure
 * -For example. find_intersections_of_street just returns a vector created in load_map
 * -The only issue to note is find_street_names_of_intersection, which is of complexity
 * O(n^2). However, it currently poses no issue, when trying to create a data structure for it
 * bugs emerged so we didn't...
 * -Anyways, this file heavily utilizes the Streets api and OSM api to create all data
 * structures
 */

#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
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
#include "GlobalVariables.h"
#include "ezgl/point.hpp"
#include "weather_data.h"
#include <random>
#include <time.h>

/*~~~~~Global Variables~~~~~*/
double MINLON;
double MINLAT;
double MAXLON;
double MAXLAT;
double LARGE = -10000000;
double SMALL = 100000000;
double MAXSTREETLENGTH;

/*~~~~~Declare global data structures~~~~~*/
std::vector<std::vector<LatLon>> array_of_curve_points;

std::vector<std::pair<double, double>> cartesian_of_intersections;

std::vector<std::vector<int>> street_segments_of_intersection;

std::vector<InfoStreetSegment> street_segment_info;

std::vector<std::vector<int>> street_segments_of_street;

std::vector<double> street_length;

std::vector<LatLon> intersection_coordinates;

std::vector<double> travel_time_of_intersections;

std::vector<std::vector<int>> intersections_on_street;

std::multimap<std::string, int> street_names_nospaces;

std::vector<std::vector<InfoStreetSegment>> info_of_segments_of_street;

std::vector<int> segments_per_street;

std::vector<double> street_segment_length;

std::unordered_map<OSMID, double> way_length;

std::unordered_map<OSMID, const OSMNode*> node_location;

std::unordered_map<OSMID, std::vector<const OSMNode*>> nodes_of_OSM_way;

std::vector<std::vector<std::pair<double, double>>> feature_cartesian_points;

std::vector<std::vector<ezgl::point2d>> points_of_feature;

std::vector<double> feature_area;

std::vector<node> node_graph;

std::vector<double> lat_avg;

double max_speed_limit = 0.0;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


bool load_map(std::string map_name) {
    std::srand(std::time(NULL));
    
    //Setting up weather api
    char *city_to_load = nullptr;
    
    //Indicates whether the map has loaded 
    //successfully
    const int FILE_EXTENTION_REMOVE = 11;
    bool load_streets = loadStreetsDatabaseBIN(map_name);
    
    
    map_name.erase(map_name.end()-FILE_EXTENTION_REMOVE, map_name.end());
    map_name = map_name + "osm.bin";
    
    bool load_osm = loadOSMDatabaseBIN(map_name);
    bool load_successful = load_streets && load_osm; 
    
    if(!load_successful) {
        return false;
    }
    
    //checks what weather data to load based on the map path 
    if(boost::algorithm::contains(map_name, "beijing")) {
        std::string str = "q=beijing";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Beijing";
    }

    else if (boost::algorithm::contains(map_name, "cairo")) {
        std::string str = "q=cairo";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Cairo";
    }

    else if (boost::algorithm::contains(map_name, "cape")) {
        std::string str = "q=cape+town";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Cape Town";
    }

    else if (boost::algorithm::contains(map_name, "horse")) {
        std::string str = "q=hamilton,ca";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Golden Horseshoe";
    }

    else if (boost::algorithm::contains(map_name, "hamilton")) {
        std::string str = "q=hamilton,ca";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Hamilton";
    }

    else if (boost::algorithm::contains(map_name, "hong")) {
        std::string str = "q=hongkong";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Hong Kong";
    }

    else if (boost::algorithm::contains(map_name, "iceland")) {
        std::string str = "q=reykjavík";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Iceland";
    }

    else if (boost::algorithm::contains(map_name, "inter")) {
        std::string str = "q=interlaken,ch";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Interlaken";
    }

    else if (boost::algorithm::contains(map_name, "london")) {
        std::string str = "q=london,uk";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "London";
    }

    else if(boost::algorithm::contains(map_name, "moscow")) {
        std::string str = "q=moscow";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Moscow";
    }

    else if (boost::algorithm::contains(map_name, "delhi")) {
        std::string str = "q=new+delhi";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "New Delhi";
    }

    else if(boost::algorithm::contains(map_name, "york")) {
        std::string str = "q=new+york";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "New York";
    }

    else if (boost::algorithm::contains(map_name, "rio")) {
        std::string str = "q=rio+de+janeiro";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Rio De Janerio";
    }

    else if (boost::algorithm::contains(map_name, "saint")) {
        std::string str = "q=saint+helena";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Saint-Helena";
    }

    else if (boost::algorithm::contains(map_name, "singapore")) {
        std::string str = "q=singapore";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Singapore";
    }

    else if (boost::algorithm::contains(map_name, "sydney")) {
        std::string str = "q=sydney";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Sydney";
    }

    else if (boost::algorithm::contains(map_name, "tehran")) {
        std::string str = "q=tehran";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Tehran";

    }

    else if (boost::algorithm::contains(map_name, "tokyo")) {
        std::string str = "q=tokyo";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Tokyo";
    }

    else if (boost::algorithm::contains(map_name, "toronto")) {
        std::string str = "q=toronto";
        city_to_load = new char[str.size()+1];
        std::copy( str.begin(), str.end(), city_to_load);
        city_to_load[str.size()] = '\0';
        city.name = "Toronto";
    }
        
    weather_api_set_up(city_to_load);
    
    
    delete [] city_to_load;


    /*~~~~~Gets the max and min latitude/longitude from a map~~~~~*/
    double min_lon = getIntersectionPosition(0).lon();
    double min_lat = getIntersectionPosition(0).lat();
    double max_lon = getIntersectionPosition(0).lon();
    double max_lat = getIntersectionPosition(0).lat(); 
    
     for(int i = 0; i < getNumIntersections(); i++){

         if(getIntersectionPosition(i).lat() > max_lat)
            max_lat = getIntersectionPosition(i).lat();

        if(getIntersectionPosition(i).lat() < min_lat)
            min_lat = getIntersectionPosition(i).lat();
        
        if(getIntersectionPosition(i).lon() > max_lon) 
            max_lon = getIntersectionPosition(i).lon();
        
        if(getIntersectionPosition(i).lon() < min_lon)
            min_lon = getIntersectionPosition(i).lon();
         
        node new_intersection(i, SMALL, -1, getIntersectionStreetSegmentCount(i)); 
        node_graph.push_back(new_intersection);
         
     }
     MINLAT = min_lat;
     MINLON = min_lon;
     MAXLAT = max_lat;
     MAXLON = max_lon;
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    
    /*~~~~~~~~find_street_ids_from_partial_street_name~~~~~~~~*/
    //Loads in all the street names with no spaces and in all lowercase
    //into a vector (for corresponding streetid i)
        for(int i = 0; i < getNumStreets(); i++) {
            std::string streetName = getStreetName(i);
            //removes all spaces
            streetName.erase(remove(streetName.begin(), streetName.end(), ' '), streetName.end());
            boost::to_lower(streetName);//converts the string to all lowercase
            street_names_nospaces.insert({streetName, i});
        }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

     array_of_curve_points.resize(getNumStreetSegments());
    /*~~~~~street_segment_length~~~~~*/
    //Filling a vector with the length of each segment in a good ol' vector
    for(int i = 0; i < getNumStreetSegments(); i++){
        InfoStreetSegment target_segment = getInfoStreetSegment(i);
        
        if (target_segment.curvePointCount > 0) {
        //Filling array with LatLon point with curves and intersection to/from
        array_of_curve_points[i].push_back(getIntersectionPosition(target_segment.from));

            for (int j = 0; j < target_segment.curvePointCount; j++) {
                array_of_curve_points[i].push_back(getStreetSegmentCurvePoint(j, i));
            }

            array_of_curve_points[i].push_back(getIntersectionPosition(target_segment.to));

            double distance = 0.0;
            for (int k = 0; k < array_of_curve_points[i].size() - 1; k++) {
                std::pair<LatLon, LatLon> delta_d(array_of_curve_points[i][k], array_of_curve_points[i][k + 1]);
                distance = distance + find_distance_between_two_points(delta_d);
            }
            street_segment_length.push_back(distance);
        }
        
        //If the segment does NOT have any curved points than return the distance from intersection to and from...
    
        else {
            std::pair<LatLon, LatLon> points(getIntersectionPosition(target_segment.to), getIntersectionPosition(target_segment.from));
            street_segment_length.push_back(find_distance_between_two_points(points));
        }    
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~street_segments_of_intersection~~~~~*/

    //Filling street_segments_of_intersection with street segments ids corresponding to the 
    //Intersection id...
    street_segments_of_intersection.resize(getNumIntersections());
    for (int i = 0; i < getNumIntersections(); i++) {

        for (int j = 0; j < getIntersectionStreetSegmentCount(i); j++) {

            street_segments_of_intersection[i].push_back(getIntersectionStreetSegment(i, j));

        }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~street_segments_of_street~~~~~*/
    //Creates a vector with the number of segments per street...this
    //will be used in the next loop
     segments_per_street.resize(getNumStreets());
    
    #pragma omp parallel for
    for (int i = 0; i < getNumStreets(); i++) {
        segments_per_street[i] = 0;
    }
     
    #pragma omp parallel for
    for(int i = 0; i < getNumStreetSegments(); i++){
        segments_per_street[getInfoStreetSegment(i).streetID] = segments_per_street[getInfoStreetSegment(i).streetID] + 1;
    }
    
    
    /*~~~~~~~~~~~~street_segment_info~~~~~~~~~~*/
     street_segment_info.resize(getNumStreetSegments());
     
    #pragma omp parallel for
     for (int i = 0; i < getNumStreetSegments(); i++) {
         street_segment_info[i] = getInfoStreetSegment(i);
     }
     
    //Makes space in the street_segments_of_street vector
    for (int i = 0; i < getNumStreets(); i++) {
        street_segments_of_street.push_back(std::vector<int>());
    }    
    
    
    /*~~~~~~~~find_street_segments_of_street~~~~~~~~*/
    //Fills the vector of vectors(int) street_segments_of_street with 
    //the the corresponding street segments for the particular street id   
     
    for (int segment_id = 0; segment_id < getNumStreetSegments() ; segment_id++) {
        StreetIndex street_id_of_segment = getInfoStreetSegment(segment_id).streetID;
        street_segments_of_street[street_id_of_segment].push_back(segment_id);
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
     
     /*~~~~~~~~find the length of all streets & sets MAXSTREETLENGTH~~~~~~~~*/
     
     MAXSTREETLENGTH = 0;
     for(int i = 0; i < getNumStreets(); i++) {
         double length = 0.0;
         
        for(int j =0; j < segments_per_street[i]; j++) {
             int segment_id = street_segments_of_street[i][j];
             length+= street_segment_length[segment_id];
        }
         
        if (length > MAXSTREETLENGTH && getStreetName(i) != "<unknown>") {
            MAXSTREETLENGTH = length;
        }
         street_length.push_back(length);
     }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

     
    /*------------street_segment_travel_time-------------*/
    //filling a vector with travel time for all the street segments
    for(int i = 0; i < getNumStreetSegments(); i++){
        double time = 0.0;
        double total_distance = 0.0;
        const double KMH_TO_MS = 5.0/18.0;
        //getting the segment
        InfoStreetSegment segment = getInfoStreetSegment(i);
        total_distance = find_street_segment_length(i);
        
        //converting speed limit to m/s and finding time
        double speed_limit = segment.speedLimit * KMH_TO_MS;
        time = total_distance * 1/speed_limit;
        
        if(max_speed_limit < speed_limit) {
            max_speed_limit = speed_limit;
        }
        
        //adding to the vector
        travel_time_of_intersections.push_back(time);
    }
    
    // ----------------------------------------------------------------
    
    
    /*~~~~~info_of_segments_of_street~~~~~*/
    //Filling a 2D vector with info related to each segment on the street
    info_of_segments_of_street.resize(getNumStreets());
    for (int i = 0; i < getNumStreetSegments(); i++) {
        InfoStreetSegment to_be_added = getInfoStreetSegment(i);
        info_of_segments_of_street[to_be_added.streetID].push_back(to_be_added);
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~intersections_on_street~~~~~*/
    //Add all intersections to a vector
    for(int street_id = 0; street_id < getNumStreets(); street_id++){
        intersections_on_street.push_back(std::vector<int>());
        
        for (int i = 0; i < info_of_segments_of_street[street_id].size(); i++) {
            intersections_on_street[street_id].push_back(info_of_segments_of_street[street_id][i].from);
            intersections_on_street[street_id].push_back(info_of_segments_of_street[street_id][i].to);
        }
        
        //Sort and then delete duplicates
        std::sort(intersections_on_street[street_id].begin(), intersections_on_street[street_id].end());
        intersections_on_street[street_id].erase(std::unique(intersections_on_street[street_id].begin(), intersections_on_street[street_id].end()), intersections_on_street[street_id].end());
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~intersection_coordinates and cartesian_of_intersections~~~~~*/
    //Stores the LatLon position of all the intersections
    cartesian_of_intersections.resize(getNumIntersections());
    intersection_coordinates.resize(getNumIntersections());
    
    #pragma omp parallel for
    for (int i = 0; i< getNumIntersections(); i++) {
        intersection_coordinates[i] = (getIntersectionPosition(i));

        LatLon position = getIntersectionPosition(i);
        double x = position.lon() * (std::cos(DEGREE_TO_RADIAN * ((MAXLAT + MINLAT) / 2.0))) *DEGREE_TO_RADIAN;
        double y = position.lat() * DEGREE_TO_RADIAN;

        std::pair<double, double> point(x,y);

        cartesian_of_intersections[i] = (point);
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    
    /*~~~~~node_location~~~~~*/
    //Filling an unordered map with the key being the OSMID of a given node, and
    //The value is a pointer to the OSMNode
    for(int i = 0; i < getNumberOfNodes(); i++){
       const OSMNode* node = getNodeByIndex(i);
       OSMID node_id = node->id();
       std::pair<OSMID, const OSMNode*> node_cluster (node_id, node);
       
       node_location.insert(node_cluster); 
       
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~nodes_of_OSM_way~~~~~*/
    //Filling an unordered map with the key being the OSMID of a given OSMway and
    //The value being a vector of OSMNode pointers 
    for(int i = 0; i < getNumberOfWays(); i++){
        const OSMWay *path = getWayByIndex(i);
        const OSMID way_id = path->id();
        std::vector<OSMID> way_members = getWayMembers(path);
        std::vector<const OSMNode*> way_nodes;
        
        for(int j = 0; j < way_members.size(); j++){
            way_nodes.push_back(node_location.find(way_members[j])->second);
        }
        
        nodes_of_OSM_way.insert({way_id, way_nodes});
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~way_length~~~~~*/
    //Filling an unordered map of type <int, int>, where the key is the osmid of a given way index and the value is the
    //length of the way (path or polygon)
    for(int i = 0; i < getNumberOfWays(); i++){

        const OSMWay *path = getWayByIndex(i);
        const OSMID way_id = path->id();
        
        
        if(isClosedWay(path)){
            double distance = 0.0;
            
            //Find Distance from first to last node
            for(int node = 0; node < nodes_of_OSM_way.find(way_id)->second.size() - 1; node++){
                std::pair<LatLon, LatLon> delta_d(getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[node]), getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[node+1]));
                distance += find_distance_between_two_points(delta_d);
            }
            
            //Closed contour, so find distance between last to first...            
            std::pair<LatLon, LatLon> closed_loop(getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[0]), getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[(nodes_of_OSM_way.find(way_id)->second).size()-1] ));
            distance += find_distance_between_two_points(closed_loop);

            way_length.insert({way_id, distance});
        }
        
        //A line integral essentially so no need to calculate the distance
        //between the last and first element
        else{
            double distance = 0.0;
            for(int node = 0; node < nodes_of_OSM_way.find(way_id)->second.size() - 1; node++){
                std::pair<LatLon, LatLon> delta_d(getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[node]), getNodeCoords((nodes_of_OSM_way.find(way_id)->second)[node+1]));
                distance += find_distance_between_two_points(delta_d);
            }


            way_length.insert({way_id, distance});    
        }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~feaure_latlon, latitudes, lat_avg~~~~~*/
    //Filling a vector of vectos with the latitues and logitudes of nodes in a feature
    //Filling another vector with only the latitudes of the nodes in a feature
    //Finally, filling a vector with the average latitude of nodes in a feature
    std::vector<std::vector<LatLon>> feature_latlon;
    std::vector<std::vector<double>> latitudes;
    
    
    for(int i = 0; i < getNumFeatures(); i++) {
        
        latitudes.push_back(std::vector<double>());
        feature_latlon.push_back(std::vector<LatLon>());
        
        for(int j = 0; j < getFeaturePointCount(i); j++) {
            
            feature_latlon[i].push_back(getFeaturePoint(j, i));
            latitudes[i].push_back(getFeaturePoint(j, i).lat());
            
        }
        //Find the max and min latitudes and average them
        double average = (*std::min_element(latitudes[i].begin(), latitudes[i].end()) + *std::max_element(latitudes[i].begin(), latitudes[i].end())) /2.0;
        lat_avg.push_back(average);
        
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    /*~~~~~feature_cartesian_points, feature_area~~~~~*/
    //Filling a vector of vector with the cartesian points corresponding to a node
    //in a feature
    //Using the shoelace algorithm to then calculate the area of a non-intersecting
    //polygon, or 0 in the case of a path
    feature_cartesian_points.resize(getNumFeatures());
    feature_area.resize(getNumFeatures());
    
    #pragma omp parallel for
    for(int i = 0; i < getNumFeatures(); i++){
        int num_points = getFeaturePointCount(i);
        feature_cartesian_points[i].resize(num_points);
        
        for(int j = 0; j < num_points; j++){
            LatLon feature_point = getFeaturePoint(j, i);
           
            double x = EARTH_RADIUS_METERS * DEGREE_TO_RADIAN*(feature_point.lon())* std::cos(DEGREE_TO_RADIAN*lat_avg[i]);
            double y = EARTH_RADIUS_METERS * DEGREE_TO_RADIAN * feature_point.lat();
            
            std::pair<double, double> point(x,y);
            
            feature_cartesian_points[i][j] = point;
        }
        double area = 0;
        
        if(feature_latlon[i][0].lat() == feature_latlon[i][num_points-1].lat() && feature_latlon[i][0].lon() == feature_latlon[i][num_points-1].lon()){
            
            for (int k = 0; k < num_points - 1 ; k++) {
                area += (feature_cartesian_points[i][k].first) * (feature_cartesian_points[i][k+1].second) - (feature_cartesian_points[i][k+1].first) * (feature_cartesian_points[i][k].second);
            }
            
            area += (feature_cartesian_points[i][num_points-1].first) * (feature_cartesian_points[i][0].second) - (feature_cartesian_points[i][0].first) * (feature_cartesian_points[i][num_points-1].second);
            
            feature_area[i] = std::abs(area*0.5);
        }
        else{
            feature_area[i] = 0.0;
        }
    }
    
     
    points_of_feature.resize(getNumFeatures());
    for(int i = 0; i < getNumFeatures(); i++){
        int num_points = getFeaturePointCount(i);
        
        for(int j = 0; j < num_points; j++){
            LatLon feature_point = getFeaturePoint(j, i);
           
            double x_2d = SCALE * DEGREE_TO_RADIAN * (feature_point.lon())* std::cos(DEGREE_TO_RADIAN * ((MAXLAT + MINLAT) / 2.0));
            double y_2d = SCALE * DEGREE_TO_RADIAN * feature_point.lat();
            
            ezgl::point2d point_2d(x_2d, y_2d);
            
            points_of_feature[i].push_back(point_2d);
        }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    
    return load_successful;
}


//Remove and clear all created data structures
void close_map() {
    //Clean-up your map related data structures here
    street_segments_of_intersection.clear();
    street_segment_info.clear();
    street_segments_of_street.clear();
    intersection_coordinates.clear();
    travel_time_of_intersections.clear();
    intersections_on_street.clear();
    street_names_nospaces.clear();
    info_of_segments_of_street.clear();
    segments_per_street.clear();
    street_segment_length.clear();
    way_length.clear();
    nodes_of_OSM_way.clear();
    node_location.clear();
    feature_area.clear();
    array_of_curve_points.clear();
    cartesian_of_intersections.clear();
    street_length.clear();
    points_of_feature.clear();
    node_graph.clear();
    shortest_path.clear();
    
    closeOSMDatabase();
    closeStreetDatabase();
}


//Returns the distance between two coordinates in meters
double find_distance_between_two_points(std::pair<LatLon, LatLon> points) {
    //Calculate distance between two points in cartesian coordinates...
    double x2 = points.second.lon() * (std::cos(DEGREE_TO_RADIAN * ((points.first.lat() + points.second.lat()) / 2.0))) *DEGREE_TO_RADIAN;
    double x1 = points.first.lon() * (std::cos(DEGREE_TO_RADIAN * ((points.first.lat() + points.second.lat()) / 2.0))) * DEGREE_TO_RADIAN;
    
    double y1 = (points).second.lat() * DEGREE_TO_RADIAN;
    double y2 = (points).first.lat() * DEGREE_TO_RADIAN;
    
    return EARTH_RADIUS_METERS* std::sqrt(std::pow(x2 - x1, 2.0) + std::pow(y2 -y1 , 2.0));
}


//Returns the length of the given street segment in meters
double find_street_segment_length(int street_segment_id) {
    
    return street_segment_length[street_segment_id];
}


//Returns the travel time to drive a street segment in seconds 
//(time = distance/speed_limit)
double find_street_segment_travel_time(int street_segment_id) {
    
    return travel_time_of_intersections[street_segment_id];
}


//Returns the nearest intersection to the given position
int find_closest_intersection(LatLon my_position) {
    
    //Establishes a comparison point by setting the closest intersection to 
    //be intersection 0 to start
    std::pair<LatLon, LatLon> intersection_0(intersection_coordinates[0], my_position);
    double shortest_distance = find_distance_between_two_points(intersection_0);
    int closest_intersection = 0;
    
    //loops through the rest of the intersections
    for (int i = 1; i < getNumIntersections(); i++) {
        std::pair<LatLon, LatLon> check_dist(intersection_coordinates[i], my_position);
        double distance = find_distance_between_two_points(check_dist);
        //checks if the intersection is closer than the current closest 
        //intersection
        if (distance <= shortest_distance) {
            shortest_distance = distance;
            closest_intersection = i;         
        }
        
    }
    return closest_intersection;
}


//Returns the street segments for the given intersection 
std::vector<int> find_street_segments_of_intersection(int intersection_id) {
    
    return street_segments_of_intersection[intersection_id];
}


//Returns the street names at the given intersection (includes duplicate street 
//names in returned vector)
std::vector<std::string> find_street_names_of_intersection(int intersection_id) {
    
    std::vector<std::string> street_names_of_intersection;
    
    for (int i = 0; i < getIntersectionStreetSegmentCount(intersection_id); i++) {
            
            StreetSegmentIndex segment_id = getIntersectionStreetSegment(intersection_id, i);
            InfoStreetSegment segment_info = getInfoStreetSegment(segment_id);
            
            street_names_of_intersection.push_back(getStreetName(segment_info.streetID));
        }
    
    return street_names_of_intersection;
}


//Returns true if you can get from intersection_ids.first to intersection_ids.second using a single 
//street segment (hint: check for 1-way streets too)
//corner case: an intersection is considered to be connected to itself
bool are_directly_connected(std::pair<int, int> intersection_ids) {
    
    if(intersection_ids.first == intersection_ids.second){
        return true;
    }
    
    std::vector<int> from_intersection;
    std::vector<int> to_intersection;

    //Check to see if they have any segments in common
    for(int i = 0; i < street_segments_of_intersection[intersection_ids.first].size(); i++){

        for (int j = 0; j < street_segments_of_intersection[intersection_ids.second].size(); j++) {

            if (street_segments_of_intersection[intersection_ids.first][i] == street_segments_of_intersection[intersection_ids.second][j]) {
                
                StreetSegmentIndex segment_id = street_segments_of_intersection[intersection_ids.first][i];
                InfoStreetSegment oneway_check = getInfoStreetSegment(segment_id);

                if (oneway_check.oneWay && oneway_check.from == intersection_ids.first && oneway_check.to == intersection_ids.second) {
                    return true;
                } 
                else if (!oneway_check.oneWay) {
                    return true;
                }
            }
        }
    }

    return false;
}


//Returns all intersections reachable by traveling down one street segment 
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
std::vector<int> find_adjacent_intersections(int intersection_id) {
    //making a vector and unordered set to store the adjacent intersections
    std::vector<int> adjacent_intersections;
    std::unordered_set<int> adjacent_intersections_hash;
    
    //getting the number of street segments
    int street_segment_count = getIntersectionStreetSegmentCount(intersection_id);
    
    //looping through all the street segments of that intersection
    for(int i = 0; i < street_segment_count; i++){
        //getting info for that segment
        StreetSegmentIndex segment_index = getIntersectionStreetSegment(intersection_id,i);
        InfoStreetSegment segment = getInfoStreetSegment(segment_index);
        
        //if the segment is not oneway then it is adjacent
        if(!segment.oneWay){
            //if the from intersection is the intersection we are searching for then the to intersection is adjacent
            if(segment.from == intersection_id){
                //checking if it is a duplicate segment using a unordered set as it has a faster search time
                if(adjacent_intersections_hash.find(segment.to) == adjacent_intersections_hash.end()){
                    adjacent_intersections_hash.insert(segment.to);
                    adjacent_intersections.push_back(segment.to);
                }
                
            }
            else{
                
                if(adjacent_intersections_hash.find(segment.from) == adjacent_intersections_hash.end()){
                    adjacent_intersections_hash.insert(segment.from);
                    adjacent_intersections.push_back(segment.from);
                }
  
            }
        }
        //if the segment is oneway then an intersection is adjacent only is it originates from the current intersection
        else if(segment.from == intersection_id){
            
            if(adjacent_intersections_hash.find(segment.to) == adjacent_intersections_hash.end()){
                adjacent_intersections_hash.insert(segment.to);
                adjacent_intersections.push_back(segment.to);
                }
            
        }  
    }
    return adjacent_intersections;
}


//Returns all street segments for the given street
std::vector<int> find_street_segments_of_street(int street_id) {
    
    return street_segments_of_street[street_id];
}


//Returns all intersections along a given street
std::vector<int> find_intersections_of_street(int street_id) {
    
    return intersections_on_street[street_id];
}


//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids) {
    
    //initializing a vector for each street with its intersections
    std::vector<int> intersections_first_street = intersections_on_street[street_ids.first];
    std::vector<int> intersections_second_street = intersections_on_street[street_ids.second];
    
    //initializing a vector with the common intersections
    std::vector<int> intersections_of_two_streets(intersections_first_street.size() + intersections_second_street.size());
    
    //using a vector to find them
    auto it = std::set_intersection(intersections_first_street.begin(),intersections_first_street.end(),intersections_second_street.begin(),intersections_second_street.end(),intersections_of_two_streets.begin());
    //reducing the size to remove the zeroes
    intersections_of_two_streets.resize(it - intersections_of_two_streets.begin());
    
    return intersections_of_two_streets;
}


//Returns all street ids corresponding to street names that start with the given prefix
//The function should be case-insensitive to the street prefix. You should ignore spaces.
//For example, both "bloor " and "BloOrst" are prefixes to "Bloor Street East".
//If no street names match the given prefix, this routine returns an empty (length 0) 
//vector.
//You can choose what to return if the street prefix passed in is an empty (length 0) 
//string, but your program must not crash if street_prefix is a length 0 string.
std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix) {
    
    std::vector<int> street_ids_from_partial_street_name;
    //iterators used to find range of streets to be pushed
    auto lower = street_names_nospaces.begin(), upper = street_names_nospaces.end(); 
    if (street_prefix.length() == 0 || street_prefix == "") {
        return street_ids_from_partial_street_name;
    }
    
    //takes the passed in string prefix, gets rid of all spaces and converts to
    //lowercase for easy comparison
    street_prefix.erase(remove(street_prefix.begin(), street_prefix.end(), ' '), street_prefix.end());
    boost::to_lower(street_prefix);
    
    //next_prefix is what would would come next alphabetically after the street_prefix
    std::string next_prefix = street_prefix;
    char last_letter = next_prefix[next_prefix.length()-1];
    
    //if the last letter of the prefix is not z
    if(last_letter != 'z') {
        next_prefix[next_prefix.length()-1]++; 
        
        //Finds the range of streets that match the search prefix
        lower = street_names_nospaces.lower_bound(street_prefix);
        upper = street_names_nospaces.lower_bound(next_prefix);
    }
    
    else if(last_letter == 'z' && next_prefix.length()==1) {
        //if the user searches using only "z", it should go until end of streets
        lower = street_names_nospaces.lower_bound(street_prefix);
        upper = street_names_nospaces.end();
    }
    
    else {
        for (int i = 1; i < next_prefix.length(); i++) {
            if(next_prefix[next_prefix.length()-1-i] != 'z') {
                next_prefix[next_prefix.length()-1-i]++;
                break;
            }
            else {
                continue;
            }
        }
        
        //Finds the range of streets that match the search prefix
        lower = street_names_nospaces.lower_bound(street_prefix);
        upper = street_names_nospaces.lower_bound(next_prefix);
    }
    
    //if only one street matches search term
    if (lower == upper) {
        street_ids_from_partial_street_name.push_back(lower->second);
    }

    else {
        for (auto it = lower; it!=upper; it++) {
            street_ids_from_partial_street_name.push_back(it->second);
        }
    }
    
    //street_ids_from_partial_street_name.erase(std::unique(street_ids_from_partial_street_name.begin(), street_ids_from_partial_street_name.end()), street_ids_from_partial_street_name.end());
    return street_ids_from_partial_street_name;
}


//Returns the area of the given closed feature in square meters
//Assume a non self-intersecting polygon (i.e. no holes)
//Return 0 if this feature is not a closed polygon.
double find_feature_area(int feature_id) {
    
    return feature_area[feature_id];
}


//Returns the length of the OSMWay that has the given OSMID, in meters.
//To implement this function you will have to  access the OSMDatabaseAPI.h 
//functions.
double find_way_length(OSMID way_id) {
    
    return (way_length.find(way_id))->second;
}