/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   directions.h
 * Author: ragubanc
 *
 * Created on March 22, 2020, 7:20 PM
 */

#ifndef DIRECTIONS_H
#define DIRECTIONS_H
#include "GlobalVariables.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "streetsdatabase/StreetsDatabaseAPI.h"

void draw_directions();

std::string get_directions(std::vector<StreetSegmentIndex> route);

std::string get_turn(StreetSegmentIndex seg1, StreetSegmentIndex seg2);

double cross_product(std::pair<double, double> v1, std::pair<double, double> v2);

std::string compass_direction(StreetSegmentIndex seg, IntersectionIndex reference);

int get_shared_intersection();
#endif /* DIRECTIONS_H */

