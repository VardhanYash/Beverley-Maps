/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   altitude_data.h
 * Author: mengiaks
 *
 * Created on February 28, 2020, 10:04 PM
 */

#ifndef ALTITUDE_DATA_H
#define ALTITUDE_DATA_H

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




void write_altitude_data(boost::property_tree::ptree &ptRoot);
void altitude_api_set_up();


#endif /* ALTITUDE_DATA_H */

