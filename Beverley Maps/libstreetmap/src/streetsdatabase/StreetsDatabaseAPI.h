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
#pragma once //protects against multiple inclusions of this header file

#include <string>
#include "LatLon.h"
#include "OSMID.h"


/*********************************************************************************
 * LAYER-2 API (libstreetsdatabase) INTRODUCTION
 *
 * The libstreetsdatabase "layer 2" API provides a simplified way of interacting 
 * with the OSM map data. For your convenience, we have extracted a subset of the 
 * information in the OSM schema of nodes, ways, and relations with attributes and 
 * pre-processed it into a form that is easier to use, consisting of familiar 
 * concepts like streets, intersections, points of interest, etc. You should start 
 * working with this layer first.
 *
 * The streets database is generated by the provided "osm2bin" tool, and stored in 
 * a binary file called {cityname}.streets.bin.
 *
 * For access to additional feature types and attribute information, you can use
 * the underlying "layer 1" API which presents the OSM data model without 
 * modification. It is more flexible but less easy to understand, however there 
 * are many resources on the web including the OSM wiki and interactive online 
 * viewers to help you.
 *
 * The "layer 1" API is described in OSMDatabaseAPI.h. To match objects between 
 * layers, the this API provides OSM IDs for all objects.
 */

// load a {map}.streets.bin file. This function must be called before any 
// other function in this API can be used. Returns true if the load succeeded,
// false if it failed.
bool loadStreetsDatabaseBIN(std::string fn);

// unloads a map / frees the memory used by the API. No other api calls can
// be made until the load function is called again for some map.
// You can only have one map open at a time.
void closeStreetDatabase(); 



/** The extracted objects are:
 *
 * Intersections      A point (LatLon) where a street terminates, or meets one 
                      or more other streets
 * Street segments    The portion of a street running between two intersections
 * Streets            A named, ordered collection of street segments running 
                      between two or more intersections
 * Points of Interest (POI)   Points of significance (eg. shops, tourist 
                      attractions) with a LatLon position and a name
 * Features           Marked polygonal areas which may have names (eg. parks, 
                      bodies of water)
 *
 *
 * Each of the entities in a given map file is labeled with an index running from 
 * 0..N-1 where N is the number of entities of that type in the map database that 
 * is currently loaded. These indices are not globally unique; they depend on the 
 * subset of objects in the present map, and the order in which they were loaded 
 * by osm2bin.
 *
 * The number of entities of each type can be queried using getNum[...], 
 * eg. getNumStreets() 
 * Additional information about the i'th entity of a given type can be accessed 
 * with the functions defined in the API below.
 *
 * A std::out_of_range exception is thrown if any of the provided indices are 
 *  invalid.
 *
 * Each entity also has an associated OSM ID that is globally unique in the OSM 
 * database, and should never change. The OSM ID of the OSM entity (Node, Way, or 
 * Relation) that produced a given feature is accessible. You can use this OSMID 
 * to access additional information through attribute tags, and to coordinate 
 * with other OSM programs that use the IDs.
 */


/** For clarity reading the API below, the index types are all typedef'ed from 
 * int. Valid street indices range from 0 .. N-1 where N=getNumStreets()
 */

typedef int FeatureIndex;
typedef int POIIndex;
typedef int StreetIndex;
typedef int StreetSegmentIndex;
typedef int IntersectionIndex;

int getNumStreets();
int getNumStreetSegments();
int getNumIntersections();
int getNumPointsOfInterest();
int getNumFeatures();



/*********************************************************************************
 * Intersection
 *
 * Each intersection has at least one street segment incident on it. Each street 
 * segment ends at another intersection.
 *
 * Intersection names are generated in a systematic way so that they are unique
 * in a map, but parsing them is not recommended.
 */

std::string    getIntersectionName(IntersectionIndex intersectionIdx);
LatLon         getIntersectionPosition(IntersectionIndex intersectionIdx);
OSMID          getIntersectionOSMNodeID(IntersectionIndex intersectionIdx);

// access the street segments incident on the intersection (get the count Nss 
// first, then iterate through segmentNumber=0..Nss-1)
int     getIntersectionStreetSegmentCount(IntersectionIndex intersectionIdx);
StreetSegmentIndex getIntersectionStreetSegment(IntersectionIndex intersectionIdx, int segmentNumber); 



/*********************************************************************************
 * Street segment
 *
 * A street segment connects two intersections. It has a speed limit, from- and 
 * to-intersections, and an associated street (which has a name).
 *
 * When navigating or drawing, the street segment may have zero or more "curve 
 * points" that specify its shape.
 *
 * Information about the street segment is returned in the InfoStreetSegment 
 * struct defined below.
 */

struct InfoStreetSegment {
    OSMID wayOSMID;   // OSM ID of the source way
                      // NOTE: Multiple segments may match a single OSM way ID

    IntersectionIndex from, to;  // intersection ID this segment runs from/to
    bool oneWay;            // if true, then can only travel in from->to direction

    int curvePointCount;    // number of curve points between the ends
    float speedLimit;            // in km/h

    StreetIndex	streetID;        // index of street this segment belongs to
};

InfoStreetSegment getInfoStreetSegment(StreetSegmentIndex streetSegmentIdx);

// fetch the latlon of the i'th curve point (number of curve points specified in 
// InfoStreetSegment)
LatLon getStreetSegmentCurvePoint(int i, StreetSegmentIndex streetSegmentIdx);



/*********************************************************************************
 * Street
 *
 * A street is made of multiple StreetSegments, which hold most of the 
 * fine-grained information (one-way status, intersections, speed limits...). 
 * The street is just a named identifier for a collection of segments.
 */

std::string getStreetName(StreetIndex streetIdx);




/*********************************************************************************
 * Points of interest
 *
 * Points of interest are derived from OSM nodes. More detailed information can be 
 * accessed from the layer-1 API using the OSM ID.
 */

std::string getPointOfInterestType(POIIndex poiIdx);
std::string getPointOfInterestName(POIIndex poiIdx);
LatLon      getPointOfInterestPosition(POIIndex poiIdx);
OSMID       getPointOfInterestOSMNodeID(POIIndex poiIdx);




/*********************************************************************************
 * Natural features
 *
 * Each natural feature has a type (e.g. Park), a name (e.g. "High Park"),
 * and some number of LatLon points that define the feature. If the first 
 * point (idx 0) and the last point (idx getFeaturePointCount-1) are the
 * same location, the feature is a closed polygon; otherwise it is a polyline.
 *
 * Natural features may be derived from OSM nodes, ways, or relations. 
 * The TypedOSMID returned by getFeatureOSMID() can be used to match 
 * features with the layer 1 API information corresponding to that OSMEntity.
 */

enum FeatureType {
    Unknown = 0,
    Park,
    Beach,
    Lake,
    River,
    Island,
    Building,
    Greenspace,
    Golfcourse,
    Stream
};

const std::string&  getFeatureName(FeatureIndex featureIdx);
FeatureType         getFeatureType(FeatureIndex featureIdx);
TypedOSMID          getFeatureOSMID(FeatureIndex featureIdx);
int            getFeaturePointCount(FeatureIndex featureIdx);
LatLon              getFeaturePoint(int idx, FeatureIndex featureIdx);

// Calling asString on a FeatureType enumerated constant will return a string.
// This is handy for printing.
const std::string& asString(FeatureType t);
