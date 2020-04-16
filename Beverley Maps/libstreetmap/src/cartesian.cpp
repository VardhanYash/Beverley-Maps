#include "cartesian.h"
#include "m1.h"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "GlobalVariables.h"
#include "ezgl/point.hpp"

//Converts a longitude to a cartesian x, uses the maxlat and minlat based on the map
 double lon_to_x(double lon){
     return lon * (std::cos(DEGREE_TO_RADIAN * ((MAXLAT + MINLAT) / 2.0))) *DEGREE_TO_RADIAN;
 }
 
 //Converts a latitude to a cartesian y, uses the maxlat and minlat based on the map
 double lat_to_y(double lat){
     return lat * DEGREE_TO_RADIAN;
 }

 //Converts a latitude class to a cartesian point2d class
 ezgl::point2d latlon_to_point(LatLon node){
    ezgl::point2d x(SCALE * lon_to_x(node.lon()), SCALE * lat_to_y(node.lat()));
    return x;
 }

 //Converts cartesian x to longitude
 double x_to_lon(double x){
     return x / ((SCALE * std::cos(DEGREE_TO_RADIAN * ((MAXLAT + MINLAT) / 2.0))) *DEGREE_TO_RADIAN);
 }
 
 //Converts cartesian y to latitude
 double y_to_lat(double y){
     return y / (SCALE * DEGREE_TO_RADIAN);
 }