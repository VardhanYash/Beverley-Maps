/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   cartesian.h
 * Author: ragubanc
 *
 * Created on February 21, 2020, 6:08 PM
 */

#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "m1.h"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "GlobalVariables.h"
#include "ezgl/point.hpp"


//Converts a longitude to a cartesian x, uses the maxlat and minlat based on the map
 double lon_to_x(double lon);
 
 //Converts a latitude to a cartesian y, uses the maxlat and minlat based on the map
 double lat_to_y(double lat);
 
 //Converts cartesian x to longitude
 double x_to_lon(double x);
 
 //Converts cartesian y to latitude
 double y_to_lat(double y);

 //Converts a latitude class to a cartesian point2d class
 ezgl::point2d latlon_to_point(LatLon node);


#endif /* CARTESIAN_H */

