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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/camera.hpp"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "GlobalVariables.h"
#include "ezgl/point.hpp"
#include "draw_main_canvas.h"
#include "cartesian.h"
#include "weather_data.h"
#include "search_viz.h"
#include <vector>
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int SCALE  = 1000;
long double INITIAL_AREA;
long double INITIAL_SCREEN;
double SCREEN_ZOOM;
int area_start = 0;
int screen_start = 0;
long double screen_ratio;

std::vector<intersection_data> intersections;
std::vector<POI_data> pois;
std::vector<street_data> streets;
void draw_main_canvas(ezgl::renderer* g);

//Main function we call to draw the UI/window and establish what to do
//When an event (mouse click etc...) occurs
 void draw_map(){    
     //stores all intersections and change max/min lat/long accordingly
     intersections.resize(getNumIntersections());
     
     #pragma omp parallel for
     for (int i = 0; i < getNumIntersections(); i++) {
         intersections[i].position = getIntersectionPosition(i);
         intersections[i].name = getIntersectionName(i);
         intersections[i].cartesian = {SCALE * lon_to_x(getIntersectionPosition(i).lon()), SCALE * lat_to_y(getIntersectionPosition(i).lat())};
     }
     
     pois.resize(getNumPointsOfInterest());
     
    #pragma omp parallel for
    for(int i = 0; i <getNumPointsOfInterest(); i++){
       
         pois[i].type = getPointOfInterestType(i);
         pois[i].name = getPointOfInterestName(i);
         pois[i].pos = getPointOfInterestPosition(i);
     }
     
     streets.resize(getNumStreets());
     for(int i =0; i< getNumStreets(); i++) {
         streets[i].name = getStreetName(i);
     }
     
    //Seting up the main ui/window
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    ezgl::application::settings settings;

    // Path to the "main.ui" file that contains an XML description of the UI.
    settings.main_ui_resource = "libstreetmap/resources/main.ui";

    // Note: the "main.ui" file has a GtkWindow called "MainWindow".
    settings.window_identifier = "MainWindow";

    // Note: the "main.ui" file has a GtkDrawingArea called "MainCanvas".
    settings.canvas_identifier = "MainCanvas";

    // Create our EZGL application.
    ezgl::application application(settings);
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    //Setting up the world view for the map, using a scaling factor 
    //of 1000
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double minx = lon_to_x(MINLON);
    double maxx = lon_to_x(MAXLON);
    double miny = lat_to_y(MINLAT);
    double maxy = lat_to_y(MAXLAT);
    
    ezgl::rectangle initial_world({SCALE * minx, SCALE * miny}, {SCALE * maxx, SCALE * maxy});
    
    
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);

 }
 
void draw_main_canvas(ezgl::renderer* g) {
    long double current_area = g->get_visible_world().area();
    long double screen_area = g->get_visible_screen().area();
    
    //See draw_main_canvas.cpp for more details on how these functions work
    double minx = lon_to_x(MINLON);
    double maxx = lon_to_x(MAXLON);
    double miny = lat_to_y(MINLAT);
    double maxy = lat_to_y(MAXLAT);
    
    if (area_start < 2 || INITIAL_SCREEN != screen_area) {
        INITIAL_AREA = g->get_visible_world().area();
        area_start++;
        INITIAL_SCREEN = screen_area;
        g->set_visible_world(ezgl::rectangle({SCALE * minx, SCALE * miny}, {SCALE * maxx, SCALE * maxy}));
    }
    
    if (screen_start < 2){
        INITIAL_SCREEN = g->get_visible_screen().area();
        screen_ratio = g->get_visible_screen().area();       
        screen_start++;
    }
    
    SCREEN_ZOOM = screen_area/screen_ratio;
            
    get_zoom(current_area);
    
    g->set_color(0xF2, 0xF2,0xF2);
    g->fill_rectangle({SCALE * minx, SCALE * miny}, {SCALE * maxx, SCALE * maxy});
    
    draw_street_segments(g);
    
    if(ZOOM >= 18){
        draw_intersections(g); 
    }
    
    draw_features(g);
    
    
    if(POI_TOGGLE){
        draw_points_of_interest(g);
    }
    
    if(PATH_TOGGLE){
        
        if(!drive_vs_walk){
            for(int i = 0; i < shortest_path.size(); i++){
                search_viz(g, shortest_path[i], true);
            }
        } 
        else if(drive_vs_walk){
            for(int i = 0; i < walk_drive_path.second.size(); i++){
                search_viz(g, walk_drive_path.second[i], true);
            }
            
            for(int i = 0; i < walk_drive_path.first.size(); i++){
                search_viz(g, walk_drive_path.first[i], false);
            }
        }
 
        draw_path_flag(g, first_int_search, "⚐");
        draw_path_flag(g, second_int_search, "⚑");
    }
    
    for(int i = 0; i < getNumIntersections(); i++){
        InfoStreetSegment segment_info = getInfoStreetSegment(i);
        write_street_name(i, segment_info, g);
    }
    
    draw_weather(g);
        
}



 