#include <map>
#include <iostream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <cassert>
#include <map>
#include <unordered_set>
#include "StreetsDatabaseAPI.h"
//#include "unit_test_util.h"

#include "is_legal.h"

//
//
//
// File-scope function declarations
//
//
//

namespace ece297test {

} //namespace

//
//
//
// Function Implementations
//
//
//

namespace ece297test {

bool courier_path_is_legal_(const std::vector<DeliveryInfo>& deliveries,
                           const std::vector<IntersectionIndex>& depots,
                           std::vector<StreetSegmentIndex>& path) {

    //
    //Ensuree we have a valid problem specification
    //
    if(!valid_courier_problem_(deliveries, depots)) {
        return false;
    }

    if (path.empty()) {
        //If it is a valid path it must contain at-least one segment
        //(since we gaurentee there is at-least one depot and delivery)

        return false; 

    } else {
        assert(!path.empty());

        //
        //Determine what our start intersection is (it must be a depot)
        //
        IntersectionIndex curr_intersection;
        if(!determine_start_intersection_(path, depots, curr_intersection)) {
            return false;
        }

        //curr_intersection must be a depot
        assert(is_depot_(depots, curr_intersection));


        //We store whether each delivery has been picked-up or dropped-off
        std::vector<bool> deliveries_picked_up(deliveries.size(), false);
        std::vector<bool> deliveries_dropped_off(deliveries.size(), false);

        //We also build fast-lookups from: intersection id to DeliveryInfo index for both
        //pickUp and dropOff intersections
        std::multimap<IntersectionIndex,size_t> intersections_to_pick_up; //Intersection_ID -> deliveries index
        std::multimap<IntersectionIndex,size_t> intersections_to_drop_off; //Intersection_ID -> deliveries index

        //Load the look-ups
        for(size_t delivery_idx = 0; delivery_idx < deliveries.size(); ++delivery_idx) {
            IntersectionIndex pick_up_intersection = deliveries[delivery_idx].pickUp;
            IntersectionIndex drop_off_intersection = deliveries[delivery_idx].dropOff;

            intersections_to_pick_up.insert(std::make_pair(pick_up_intersection, delivery_idx));
            intersections_to_drop_off.insert(std::make_pair(drop_off_intersection, delivery_idx));
        }

        //We verify the path by walking along each segment and:
        //  * Checking that the next step along the path is valid (see traverse_segment_())
        //  * Recording any package pick-ups (see pick_up_at_intersection_())
        //  * Recording any package drop-offs (see drop_off_at_intersection_())
        for (size_t path_idx = 0; path_idx < path.size(); ++path_idx) {

            IntersectionIndex next_intersection; //Set by traverse_segment_

            if(!traverse_segment_(path, path_idx, curr_intersection, next_intersection)) {
                return false;
            }

            //Process packages
            pick_up_at_intersection_(deliveries, 
                                    intersections_to_pick_up, 
                                    curr_intersection, 
                                    deliveries_picked_up);

            drop_off_at_intersection_(deliveries,
                                     intersections_to_drop_off, 
                                     deliveries_picked_up,
                                     curr_intersection, 
                                     deliveries_dropped_off);

            //Advance
            curr_intersection = next_intersection;
        }

        //
        //We should be at a depot
        //
        if (!is_depot_(depots, curr_intersection)) {
            //Not at a valid end intersection
            return false;
        }

        //
        //Check everything was delivered
        //
        if(!delivered_all_packages_(deliveries, deliveries_picked_up, deliveries_dropped_off)) {
            return false;
        }

    }

    //Everything validated
    return true;

}

bool courier_path_is_legal_with_capacity_(const std::vector<DeliveryInfo>& deliveries,
                           const std::vector<IntersectionIndex>& depots,
                           std::vector<CourierSubpath>& path, 
                           const float truck_capacity) {

    //
    //Ensuree we have a valid problem specification
    //
    if(!valid_courier_problem_(deliveries, depots)) {
        return false;
    }

    if (path.empty()) {

        return false; 

    } else {
        assert(!path.empty());

        //
        // Check that start_intersection of the first subpath is a depot
        //
        if (!is_depot_(depots, path[0].start_intersection)) {
            return false;
        }

        //
        //We should end at a depot
        //
        if (!is_depot_(depots, path[path.size() - 1].end_intersection)) {
            return false;
        }

        // Make sure that overall path of segments is not empty 
        std::vector<StreetSegmentIndex> overall_path;
        for (size_t sub_idx = 0; sub_idx < path.size(); sub_idx++)
            overall_path.insert(overall_path.end(), path[sub_idx].subpath.begin(), path[sub_idx].subpath.end());

        if (overall_path.empty()) {
            return false;
        }

        //We store whether each delivery has been picked-up or dropped-off
        std::vector<bool> deliveries_picked_up(deliveries.size(), false);
        std::vector<bool> deliveries_dropped_off(deliveries.size(), false);

        //We also build fast-lookups from: intersection id to DeliveryInfo index for both
        //pickUp and dropOff intersections
        std::multimap<IntersectionIndex,size_t> intersections_to_pick_up; //Intersection_ID -> deliveries index
        std::multimap<IntersectionIndex,size_t> intersections_to_drop_off; //Intersection_ID -> deliveries index

        //Load the look-ups
        for(size_t delivery_idx = 0; delivery_idx < deliveries.size(); ++delivery_idx) {
            IntersectionIndex pick_up_intersection = deliveries[delivery_idx].pickUp;
            IntersectionIndex drop_off_intersection = deliveries[delivery_idx].dropOff;

            intersections_to_pick_up.insert(std::make_pair(pick_up_intersection, delivery_idx));
            intersections_to_drop_off.insert(std::make_pair(drop_off_intersection, delivery_idx));
        }

        // 
        // Validating CourierSubpath and capacity
        //
        float current_total_weight = 0.0;
        for (size_t sub_idx = 0; sub_idx < path.size(); sub_idx++) {

            CourierSubpath courier_subpath = path[sub_idx];
            IntersectionIndex start_intersection = courier_subpath.start_intersection;
            IntersectionIndex end_intersection = courier_subpath.end_intersection;
            std::vector<unsigned> pickup_indices = courier_subpath.pickUp_indices;

            if (courier_subpath.subpath.empty() && courier_subpath.start_intersection != courier_subpath.end_intersection) {
                return false;
            }

            // Check if start_intersection = end_intersection of previous CourierSubpath
            if (sub_idx > 0)
            {
                if (start_intersection != path[sub_idx - 1].end_intersection)
                {
                    return false;
                }
                // Check that start intersection is at least a pickup intersection or drop-off intersection (already checked if first one is depot above)
                if (intersections_to_pick_up.find(start_intersection) == intersections_to_pick_up.end() &&
                    intersections_to_drop_off.find(start_intersection) == intersections_to_drop_off.end()) {
                    return false;
                }
            }

            // check that start_intersection of each subpath is actually at the beginning of the subpath
            if (!is_start_intersection_correct_(courier_subpath))
            {
                return false;
            }

            // check that end_intersection of each subpath is actually at the end of the subpath
            if (!is_end_intersection_correct_(courier_subpath))
            {
                return false;
            }

            // Checking capacity
            /*
             * Must ensure that the total weight of all picked up packages at each subpath does not 
             * exceed the truck capacity
             */
            
            // Process Packages
            if (!pick_up_at_intersection_capacity_(deliveries, 
                                            intersections_to_pick_up, 
                                            pickup_indices, 
                                            start_intersection, 
                                            deliveries_picked_up, 
                                            current_total_weight))
                return false;

            if (!drop_off_at_intersection_capacity_(deliveries, 
                                              intersections_to_drop_off,
                                              deliveries_picked_up, 
                                              start_intersection, 
                                              deliveries_dropped_off, 
                                              current_total_weight))
                return false;

            if (current_total_weight > truck_capacity + FLOAT_EPSILON)
            {
                return false;
            }

            //We verify the subpath by walking along each segment and:
            //  * Checking that the next step along the path is valid (see traverse_segment_())
            IntersectionIndex curr_intersection = start_intersection;
            for (size_t subpath_idx = 0; subpath_idx < courier_subpath.subpath.size(); ++subpath_idx) {

                IntersectionIndex next_intersection; //Set by traverse_segment_

                if(!traverse_segment_(courier_subpath.subpath, subpath_idx, curr_intersection, next_intersection)) {
                    return false;
                }

                //Advance
                curr_intersection = next_intersection;
            }
            
            // must end at end_intersection
            assert (curr_intersection == end_intersection);
        }

        //
        //Check everything was delivered
        //
        if(!delivered_all_packages_(deliveries, deliveries_picked_up, deliveries_dropped_off)) {
            return false;
        }

        // current_total_weight should be zero
        if (current_total_weight < -FLOAT_EPSILON ||
            current_total_weight > FLOAT_EPSILON) {
            return false;
        }
    }

    //Everything validated
    return true;
}


bool valid_courier_problem_(const std::vector<DeliveryInfo>& deliveries_vec,
                           const std::vector<IntersectionIndex>& depots_vec) {
    if(deliveries_vec.empty()) {
        return false;
    }

    if(depots_vec.empty()) {
        return false;
    }

    // Check for duplicates among:
    //   * Pick-Ups and depots
    //   * Drop-Offs and depots

    std::vector<IntersectionIndex> depots = depots_vec; //Copy since we need to sort the depots

    std::vector<IntersectionIndex> delivery_pick_ups;
    std::vector<IntersectionIndex> delivery_drop_offs;

    for(const DeliveryInfo& info : deliveries_vec) {
        delivery_pick_ups.push_back(info.pickUp);
        delivery_drop_offs.push_back(info.dropOff);
    }

    //Sort all the ids so we can quickly find the set intersections
    std::sort(delivery_pick_ups.begin(), delivery_pick_ups.end());
    std::sort(delivery_drop_offs.begin(), delivery_drop_offs.end());
    std::sort(depots.begin(), depots.end());

    //Verify that there are is not commonality between:
    //  * pick_ups and depots
    //  * drop_offs and depots
    //
    // Note: we allow duplicates between pick_ups and drop_offs
    std::vector<IntersectionIndex> common_pick_up_depots;
    std::vector<IntersectionIndex> common_drop_off_depots;

    //Common between pick-ups and depots
    std::set_intersection(delivery_pick_ups.begin(), delivery_pick_ups.end(),
                          depots.begin(), depots.end(),
                          std::back_inserter(common_pick_up_depots));
    
    //Common between drop-offs and depots
    std::set_intersection(delivery_drop_offs.begin(), delivery_drop_offs.end(),
                          depots.begin(), depots.end(),
                          std::back_inserter(common_drop_off_depots));

    if(!common_pick_up_depots.empty()) {
        return false;
    }

    if(!common_drop_off_depots.empty()) {
        return false;
    }

    //
    //Sanity check on id bounds
    //
    //Since vectors are sorted, can just query first and last elements
    assert(*(--depots.end()) <= (getNumIntersections()));
    assert(*(--delivery_pick_ups.end()) <= (getNumIntersections()));
    assert(*(--delivery_drop_offs.end()) <= (getNumIntersections()));

    return true;
}

bool determine_start_intersection_(std::vector<StreetSegmentIndex>& path,
                                  const std::vector<IntersectionIndex>& depots,
                                  IntersectionIndex& start_intersection) {
    assert(path.size() > 0);

    InfoStreetSegment first_seg = getInfoStreetSegment(path[0]);

    //Look for the from and to in the depot list
    auto to_iter = std::find(depots.begin(), depots.end(), first_seg.to);
    auto from_iter = std::find(depots.begin(), depots.end(), first_seg.from);

    //Initilize the current intersection
    if(to_iter != depots.end() && from_iter == depots.end()) {

        //To is exclusively a depot
        start_intersection = *to_iter;

    } else if(to_iter == depots.end() && from_iter != depots.end()) {

        //From is exclusivley a depot
        start_intersection = *from_iter;

    } else if(to_iter != depots.end() && from_iter != depots.end()) {
        //Both to and from are depots.
        //
        //This is the unlikely case where two depots are adjacent, and the student
        //returns the single segment joining them as the first element of the path.
        if(path.size() == 1) {
            //If we have a path consisting of only a single segment
            //we only need to take care when the segment is a one-way (i.e.
            //we should pick the start intersection to not violate the one-way)
            //
            //If it is not a one-way, we can pick an arbitrary start intersection.
            //
            //As a result we always pick the 'from' intersection as the start, as this
            //is gaurnteed to be correct for a one-way segment
            start_intersection = *from_iter;

        } else {
            //Multiple segments
            //
            //We can determine the start intersection by identify the common 
            //intersection between the two segments.
            assert(path.size() > 1);

            InfoStreetSegment second_seg = getInfoStreetSegment(path[1]);

            if(first_seg.to == second_seg.to || first_seg.to == second_seg.from) {
                //first_seg to connects to the second seg via it's 'to' intersection
                start_intersection = first_seg.from;
            } else {
                assert(first_seg.from == second_seg.to || first_seg.from == second_seg.from);

                //first_seg to connects to the second seg via it's 'from' intersection
                start_intersection = first_seg.to;
            }
        }

    } else {
        assert(to_iter == depots.end() && from_iter == depots.end());
        return false;
    }
#ifdef DEBUG_PICK_UP_DROP_OFF
    std::cout << "Starting at depot " <<  start_intersection << std::endl;
#endif
    return true;
}

bool is_depot_(const std::vector<IntersectionIndex>& depots,
              const IntersectionIndex intersection_id) {

    auto iter = std::find(depots.begin(), depots.end(), intersection_id);

    return (iter != depots.end());
}

bool traverse_segment_(std::vector<StreetSegmentIndex>& path,
                      const unsigned path_idx,
                      const IntersectionIndex curr_intersection,
                      IntersectionIndex& next_intersection) {
    assert(path_idx < path.size());

    InfoStreetSegment seg_info = getInfoStreetSegment(path[path_idx]);

    //
    //Are we moving forward or back along the segment?
    //
    bool seg_traverse_forward;
    if (seg_info.from == static_cast<int>(curr_intersection)) {
        //We take care to check 'from' first. This ensures
        //we get a reasonable traversal direction even in the 
        //case of a self-looping one-way segment

        //Moving forwards
        seg_traverse_forward = true;

    } else if (seg_info.to == static_cast<int>(curr_intersection)) {
        //Moving backwards
        seg_traverse_forward = false;

    } else {
        assert(seg_info.from != static_cast<int>(curr_intersection) && seg_info.to != static_cast<int>(curr_intersection));
        return false;
    }

    //
    //Are we going the wrong way along the segment?
    //
    if (!seg_traverse_forward && seg_info.oneWay) {
        return false;
    }

    //
    //Advance to the next intersection
    //
    next_intersection = (seg_traverse_forward) ? seg_info.to : seg_info.from;

    return true;
}

void pick_up_at_intersection_(const std::vector<DeliveryInfo>& /*deliveries*/,
                             const std::multimap<IntersectionIndex,size_t>& intersections_to_pick_up,
                             const IntersectionIndex curr_intersection,
                             std::vector<bool>& deliveries_picked_up) {
    //
    //Check if we are picking up packages
    //

    //Find all the deliveries picking-up from this intersection
    auto range_pair = intersections_to_pick_up.equal_range(curr_intersection);

    //Mark each delivery as picked-up
    for(auto key_value_iter = range_pair.first; key_value_iter != range_pair.second; ++key_value_iter) {
        size_t delivery_idx = key_value_iter->second; 

        deliveries_picked_up[delivery_idx] = true;

#ifdef DEBUG_PICK_UP_DROP_OFF
        std::cerr << "Info: Picked-up package for delivery " << delivery_idx
                  << " at intersection " << curr_intersection << "\n";
#endif

    }
}

bool pick_up_at_intersection_capacity_(const std::vector<DeliveryInfo>& deliveries,
                             const std::multimap<IntersectionIndex,size_t>& intersections_to_pick_up,
                             const std::vector<unsigned> & pickup_indices,
                             const IntersectionIndex curr_intersection,
                             std::vector<bool>& deliveries_picked_up, 
                             float & current_total_weight) {

    // check if start_intersection is a pickup order
    if (pickup_indices.size() > 0) // this is a pickup intersection
    {
        if(intersections_to_pick_up.find(curr_intersection) ==
            intersections_to_pick_up.end()) {
            return false;
        }

        for (unsigned idx = 0; idx < pickup_indices.size(); idx++)
        {
            if (deliveries_picked_up[pickup_indices[idx]])
                continue;

            deliveries_picked_up[pickup_indices[idx]] = true;

            current_total_weight += deliveries[pickup_indices[idx]].itemWeight;
        }
    }
    return true;
}

void drop_off_at_intersection_(const std::vector<DeliveryInfo>& /*deliveries*/,
                              const std::multimap<IntersectionIndex,size_t>& intersections_to_drop_off,
                              const std::vector<bool>& deliveries_picked_up,
                              const IntersectionIndex curr_intersection,
                              std::vector<bool>& deliveries_dropped_off) {
    //
    //Check if we are dropping-off packages
    //

    //Find all the deliveries dropping-off to this intersection
    auto range_pair = intersections_to_drop_off.equal_range(curr_intersection);

    //Mark each delivery dropped-off
    for(auto key_value_iter = range_pair.first; key_value_iter != range_pair.second; ++key_value_iter) {
        size_t delivery_idx = key_value_iter->second; 

        if(deliveries_picked_up[delivery_idx]) {
            //Can only drop-off if the delivery was already picked-up
            deliveries_dropped_off[delivery_idx] = true;

#ifdef DEBUG_PICK_UP_DROP_OFF
            std::cerr << "Info: Dropped-off package for delivery " << delivery_idx
                      << " at intersection " << curr_intersection << "\n";
        } else {
            std::cerr << "Info: Did not drop-off package for delivery " << delivery_idx
                      << " at intersection " << curr_intersection 
                      << " since it has not been picked-up" << "\n";
#endif

        }
    }
}

bool drop_off_at_intersection_capacity_(const std::vector<DeliveryInfo>& deliveries,
                              const std::multimap<IntersectionIndex,size_t>& intersections_to_drop_off,
                              const std::vector<bool>& deliveries_picked_up,
                              const IntersectionIndex curr_intersection,
                              std::vector<bool>& deliveries_dropped_off,
                              float & current_total_weight) {

	auto drop_off_it = intersections_to_drop_off.equal_range(curr_intersection);

    for (auto itr = drop_off_it.first; itr != drop_off_it.second; ++itr)
    {
        unsigned delivery_index = itr->second;
        if (deliveries_picked_up[delivery_index]) // cannot drop off before picking up
        {
            if (deliveries_dropped_off[delivery_index]) // Already dropped it off
                continue;

            deliveries_dropped_off[delivery_index] = true;
            current_total_weight = current_total_weight - deliveries[delivery_index].itemWeight;
        } 
    }

    if (current_total_weight < -FLOAT_EPSILON) {
        return false;
    }
    return true;
}

bool delivered_all_packages_(const std::vector<DeliveryInfo>& deliveries,
                            const std::vector<bool>& deliveries_picked_up,
                            const std::vector<bool>& deliveries_dropped_off) {

    //
    //Check how many undelivered packages there are
    //
    size_t undelivered_packages = 0;
    for(size_t delivery_idx = 0; delivery_idx < deliveries.size(); ++delivery_idx) {
        if(!deliveries_dropped_off[delivery_idx]) {
            ++undelivered_packages;
        } else {
            //If it was dropped-off it must have been picked-up
            assert(deliveries_picked_up[delivery_idx]);
        }
    }

    //
    //Report to the user the missing packages
    //
    if(undelivered_packages > 0) {
        for(size_t delivery_idx = 0; delivery_idx < deliveries.size(); ++delivery_idx) {
            //Un-picked up
            if(!deliveries_picked_up[delivery_idx]) {
                
            }

            //Un-delivered up
            if(!deliveries_dropped_off[delivery_idx]) {

            }
        }

        //Some un-delivered
        return false;
    }

    //All delivered
    return true;
}


bool is_start_intersection_correct_(const CourierSubpath& cs) {
    if (cs.subpath.empty()) {
        return cs.start_intersection == cs.end_intersection;
    }

    InfoStreetSegment seg_info = getInfoStreetSegment(cs.subpath.at(0));
	if (seg_info.from != static_cast<int>(cs.start_intersection) &&
		seg_info.to != static_cast<int>(cs.start_intersection))
		return false;
	
	return true;
}

bool is_end_intersection_correct_(const CourierSubpath& cs) {
    if (cs.subpath.empty()) {
        return cs.start_intersection == cs.end_intersection;
    }

	InfoStreetSegment seg_info = getInfoStreetSegment(cs.subpath.at(cs.subpath.size() - 1));
	if (seg_info.from != static_cast<int>(cs.end_intersection) &&
		seg_info.to != static_cast<int>(cs.end_intersection))
		return false;

	return true;
}

double compute_courier_path_travel_time_(const std::vector<CourierSubpath>& courier_route, 
                                        const float turn_penalty) {
    double total_travel_time = 0.0; 
    for (size_t i = 0; i < courier_route.size(); i++)
        total_travel_time += compute_path_travel_time(courier_route[i].subpath, turn_penalty);

    return total_travel_time;
}

} //namespace
