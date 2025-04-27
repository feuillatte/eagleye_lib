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

#include <coordinate/coordinate.hpp>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include "angles.h"

void fromMsg(const GeoPoint &from, UTMPoint &to,
        const bool& force_zone, const char& band, const uint8_t& zone)
{
  double Lat = from.latitude;
  double Long = from.longitude;

  double a = GeographicLib::Constants::WGS84_a();
  double eccSquared = UTM_E2;
  double k0 = GeographicLib::Constants::UTM_k0();

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  // (JOQ: this is broken for Long < -180, do a real normalize)
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
  double LatRad = angles::from_degrees(Lat);
  double LongRad = angles::from_degrees(LongTemp);
  double LongOriginRad;

  to.altitude = from.altitude;
  if (!force_zone)
    to.zone = int((LongTemp + 180)/6) + 1;
  else
    to.zone = zone;

  if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    to.zone = 32;

  // Special zones for Svalbard
  if( Lat >= 72.0 && Lat < 84.0 )
    {
      if(      LongTemp >= 0.0  && LongTemp <  9.0 ) to.zone = 31;
      else if( LongTemp >= 9.0  && LongTemp < 21.0 ) to.zone = 33;
      else if( LongTemp >= 21.0 && LongTemp < 33.0 ) to.zone = 35;
      else if( LongTemp >= 33.0 && LongTemp < 42.0 ) to.zone = 37;
    }
  // +3 puts origin in middle of zone
  LongOrigin = (to.zone - 1)*6 - 180 + 3;
  LongOriginRad = angles::from_degrees(LongOrigin);

  // compute the UTM band from the latitude
  if (!force_zone)
    to.band = UTMBand(Lat, LongTemp);
  else
    to.band = band;



#if 0
  if (to.band == ' ')
    throw std::range_error;
#endif

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
          - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
         - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
         + (15*eccSquared*eccSquared/256
            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
         - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

  to.easting = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
           + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
     + 500000.0);

  to.northing = (double)
    (k0*(M+N*tan(LatRad)
         *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
           + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

  if(Lat < 0)
    {
      //10000000 meter offset for southern hemisphere
      to.northing += 10000000.0;
    }
}

/** @return true if point is valid. */
bool isValid(const UTMPoint &pt)
{
  if (pt.zone < 1 || pt.zone > 60)
    return false;

  if (!isupper(pt.band) || pt.band == 'I' || pt.band == 'O')
    return false;

  // The Universal Polar Stereographic bands are not currently
  // supported.  When they are: A, B, Y and Z will be valid (and the
  // zone number will be ignored).
  if (pt.band < 'C' || pt.band > 'X')
    return false;

  return true;
}

