/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   weather_data.h
 * Author: ragubanc
 *
 * Created on February 28, 2020, 5:02 AM
 */

#ifndef WEATHER_DATA_H
#define WEATHER_DATA_H
#include <iostream>
#include <string.h>
#include <sstream>
#include <curl/curl.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include "GlobalVariables.h"

using boost::property_tree::ptree;
using boost::property_tree::read_json;

void write_weather_data(boost::property_tree::ptree &ptRoot);
void weather_api_set_up(char city_name []);


#endif /* WEATHER_DATA_H */

