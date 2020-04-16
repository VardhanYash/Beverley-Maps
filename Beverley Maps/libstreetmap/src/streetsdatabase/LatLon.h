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
/*
 * LatLon.h
 *
 *  Created on: Dec 17, 2015
 *      Author: jcassidy
 */

#ifndef LATLON_H_
#define LATLON_H_

#include <limits>
#include <iostream>
#include <utility>
#include <array>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wundef"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wctor-dtor-privacy"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#endif
#include <boost/serialization/access.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

/** Latitude and longitude in decimal degrees
 *
 * Students only need to use the .lat() and .lon() functions. 
 *
 * (lat,lon) are held as single-precision float to save space and match the precision given by OSM, but returned as double because
 * single-precision arithmetic has been found to cause precision errors. Always use double math. The return cast ensures this is
 * the default case.
 *
 *
 * +lat is north
 * +lon is east
 */

class LatLon
{
public: 
    // Accessors, which convert to lat or lon to double and return it
    double lat() const { return m_lat; }
    double lon() const { return m_lon; }

    // Constructors
    LatLon(){}
    explicit LatLon(float lat_, float lon_) : m_lat(lat_),m_lon(lon_){}

private:
    float m_lat = std::numeric_limits<float>::quiet_NaN();
    float m_lon = std::numeric_limits<float>::quiet_NaN();

    // For creating .bin files for rapid loading; students don't need to use
    friend class boost::serialization::access;
    template<class Archive>void serialize(Archive& ar, unsigned int)
    	{ ar & m_lat & m_lon; }
};

std::ostream& operator<<(std::ostream& os,LatLon);


// Convert two (lat,lon) pairs to 4 corners of a bounding box.
std::array<LatLon,4> bounds_to_corners(std::pair<LatLon,LatLon> bounds);


#endif /* LATLON_H_ */
