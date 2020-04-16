/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "search_viz.h"


//For visualizing the search algorithm
void search_viz(ezgl::renderer *g, StreetSegmentIndex path, bool path_type){
    InfoStreetSegment segment_info = getInfoStreetSegment(path);
    ezgl::point2d from (0,0);
    ezgl::point2d to (0,0);
    //If the segment has no curve points just draw the straight line

    if(segment_info.curvePointCount == 0){
        from = latlon_to_point(getIntersectionPosition(segment_info.from));
        to = latlon_to_point(getIntersectionPosition(segment_info.to));

        draw_path_segment(path, g, from, to, segment_info, path_type);
    }

    //If it does have curved points then draw a bunch of smaller lines
    else{
        for(int j = 0; j < array_of_curve_points[path].size() - 1; j++){
            from = latlon_to_point(array_of_curve_points[path][j]);
            to = latlon_to_point(array_of_curve_points[path][j + 1]);

            draw_path_segment(path, g, from, to, segment_info, path_type);
        }
    }           
}

//Aha delay
void delay (int ms){
    std::chrono::milliseconds duration (ms);
    std::this_thread::sleep_for(duration);
    return;
}


//Draw a flag on the starting and ending intersection of the find path function
void draw_path_flag(ezgl::renderer *g, int intersection, std::string flag){
    g->set_color(ezgl::BLACK);
    g->set_text_rotation(0);
    
    ezgl::point2d place_flag = latlon_to_point(getIntersectionPosition(intersection));
    
    g->set_horiz_text_just(ezgl::text_just::center);
    g->set_vert_text_just(ezgl::text_just::center);
    
    g->set_font_size(0.1*pow(ZOOM, 2));
    g->draw_text(place_flag, flag);          
}

//Draws the segments of a path
void draw_path_segment(int i, ezgl::renderer *g, ezgl::point2d from, ezgl::point2d to, InfoStreetSegment segment_info, bool path_type) {
    double SMALL_SCALE = 0.02;
    double MEDIUM_SCALE = 0.025;
    double LARGE_SCALE = 0.03;
    
    if(path_type){
        g->set_color(ezgl::STREET_FOUND_BLUE);
        g->set_line_dash(ezgl::line_dash::none);
    }
    else if(!path_type){
        g->set_color(ezgl::RED);
        g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
    }
    
    if (0.01 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < SMALL_SCALE * MAXSTREETLENGTH) {
        g->set_line_width(SMALL_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    else if (SMALL_SCALE * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.05 * MAXSTREETLENGTH ) {
        g->set_line_width(SMALL_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }

    else if (0.05 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.15 * MAXSTREETLENGTH) {
        g->set_line_width(SMALL_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    else if (0.15 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.3 * MAXSTREETLENGTH) {
        g->set_line_width(MEDIUM_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    else if (0.3 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.4 * MAXSTREETLENGTH) {
        g->set_line_width(MEDIUM_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    else if (0.4 * MAXSTREETLENGTH <= street_length[getInfoStreetSegment(i).streetID] && street_length[getInfoStreetSegment(i).streetID] < 0.5 * MAXSTREETLENGTH) {
        g->set_line_width(LARGE_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }

    else if (street_length[getInfoStreetSegment(i).streetID] >= 0.5 * MAXSTREETLENGTH && getStreetName(segment_info.streetID) != "<unknown>") {
        g->set_line_width(LARGE_SCALE * pow(ZOOM, 2));
        g->draw_line(from, to);
    }
    
    else{
        g->set_line_width(0.02 * pow(ZOOM, 2));
        g->draw_line(from, to);
    }

}