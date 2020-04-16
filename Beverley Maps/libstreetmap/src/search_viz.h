/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   search_viz.h
 * Author: vardhan1
 *
 * Created on March 20, 2020, 12:35 AM
 */

#ifndef SEARCH_VIZ_H
#define SEARCH_VIZ_H
#include <chrono>
#include <thread>
#include "GlobalVariables.h"
#include "draw_main_canvas.h"
#include "ezgl/graphics.hpp"


void search_viz(ezgl::renderer *g, StreetSegmentIndex path, bool path_type);
void delay (int ms);
void draw_path_flag(ezgl::renderer *g, int intersection, std::string flag);
void draw_path_segment(int i, ezgl::renderer *g, ezgl::point2d from, ezgl::point2d to, InfoStreetSegment segment_info, bool path_type);

#endif /* SEARCH_VIZ_H */

