
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
#include "search.h"
#include "GlobalVariables.h"
#include "is_legal.h"
//#include "unit_test_util.h"

IntersectionIndex find_closest_depot (const std::vector<int> & depots, IntersectionIndex current_intersection, double turn_penalty); 

//Computes the courier path travel time 
double compute_courier_path_travel_time(std::vector<CourierSubpath>& courier_route, const float turn_penalty);

//Shuffles the order of two drop off points to see if it's a better route
std::vector<CourierSubpath> shuffle_drop(std::vector<CourierSubpath>& original_path, int shuffle_pos1, int shuffle_pos2, const double turn_penalty);

//Shuffles the order of two pick up points to see if it's a better route
std::vector<CourierSubpath> shuffle_pick(std::vector<CourierSubpath>& original_path, int shuffle_pos1, int shuffle_pos2, const double turn_penalty);

//
std::vector<CourierSubpath> alpha_opt(std::vector<CourierSubpath>& original_path, int cut_1, int cut_2, int cut_3, const double turn_penalty, const std::vector<int>& depots);

std::vector<int> reverse(std::vector<int> forward);
