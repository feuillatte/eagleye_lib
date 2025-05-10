// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef COORDINATE_H
#define COORDINATE_H

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

struct GeoPoint {
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
};

struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

struct GeoPose {
    GeoPoint position{};
    Quaternion orientation{};
};


class UTMPoint
{
 public:

  /** Null constructor. Makes a 2D, invalid point object. */
  UTMPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::quiet_NaN()),
    zone(0),
    band(' ')
  {}

  /** Copy constructor. */
  UTMPoint(const UTMPoint &that):
    easting(that.easting),
    northing(that.northing),
    altitude(that.altitude),
    zone(that.zone),
    band(that.band)
  {}

  UTMPoint(const GeoPoint &pt);

  /** Create a flattened 2-D grid point. */
  UTMPoint(double _easting, double _northing, uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(std::numeric_limits<double>::quiet_NaN()),
    zone(_zone),
    band(_band)
  {}

  /** Create a 3-D grid point. */
  UTMPoint(double _easting, double _northing, double _altitude,
            uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(_altitude),
    zone(_zone),
    band(_band)
  {}

  // data members
  double easting;           ///< easting within grid zone [meters]
  double northing;          ///< northing within grid zone [meters]
  double altitude;          ///< altitude above ellipsoid [meters] or NaN
  uint8_t zone;             ///< UTM longitude zone number
  char   band;              ///< MGRS latitude band letter

}; // class UTMPoint

/** Universal Transverse Mercator (UTM) pose */
class UTMPose
{
 public:

  /** Null constructor. Makes a 2D, invalid pose object. */
  UTMPose():
    position(),
    orientation()
  {}

  /** Copy constructor. */
  UTMPose(const UTMPose &that):
    position(that.position),
    orientation(that.orientation)
  {}

  /** Create from a WGS 84 geodetic pose. */
  UTMPose(const GeoPose &pose):
    position(pose.position),
    orientation(pose.orientation)
  {}

  /** Create from a UTMPoint and a quaternion. */
  UTMPose(UTMPoint pt,
          const Quaternion &q):
    position(pt),
    orientation(q)
  {}

  /** Create from a WGS 84 geodetic point and a quaternion. */
  UTMPose(const GeoPoint &pt,
          const Quaternion &q):
    position(pt),
    orientation(q)
  {}

  // data members
  UTMPoint position;
  Quaternion orientation;

}; // class UTMPose

static char UTMBand(double Lat, double Lon)
{
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  // '_' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = ' ';

  return LetterDesignator;
}


const double WGS84_E{std::sqrt(2*GeographicLib::Constants::WGS84_f() - GeographicLib::Constants::WGS84_f() * GeographicLib::Constants::WGS84_f())};
const double UTM_E2{WGS84_E * WGS84_E};

extern void fromMsg(const GeoPoint &from, UTMPoint &to,
        const bool& force_zone = false, const char& band = 'A', const uint8_t& zone = 0);

extern bool isValid(const UTMPoint& pt);

class ConvertHeight
{
public:
  ConvertHeight();

  double convert2altitude();
  double convert2ellipsoid();
  void setLLH(double, double, double);

private:
  double _latitude;
  double _longitude;
  double _height;
  double geoid;
  double converted_height;
  double** geoid_map_data;
};

extern void ll2xy(int, double*, double*);
extern void ll2xy_mgrs(double*, double*);
extern void ecef2llh(double*, double*);
extern void enu2llh(double*, double*, double*);
extern void llh2xyz(double*, double*);
extern void xyz2enu(double*, double*, double*);
extern void xyz2enu_vel(double*, double*, double*);
extern double geoid_per_minute(double, double,double**);
extern double** read_geoid_map();


#endif /*COORDINATE_H */
