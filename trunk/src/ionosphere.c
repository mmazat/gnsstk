/**
\file    ionosphere.c
\brief   GNSS core 'c' function library: GPS ionospheric calculations.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2005-08-14

\remarks
- Get better klobuchar (alpha and beta parameters from:
http://www.aiub.unibe.ch/ionosphere/ \n
ftp://ftp.unibe.ch/aiub/CODE/ \n
http://www.aiub.unibe.ch/download/CODE/ \n
e.g. Download CGIM3280.07N from ftp://ftp.unibe.ch/aiub/CODE/2007/ 
for 2007-11-29.

\b "LICENSE INFORMATION" \n
Copyright (c) 2007, refer to 'author' doxygen tags \n
All rights reserved. \n

Redistribution and use in source and binary forms, with or without
modification, are permitted provided the following conditions are met: \n

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer. \n
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution. \n
- The name(s) of the contributor(s) may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission. \n

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.
*/

#include <math.h>
#include "ionosphere.h"
#include "constants.h"

#define TWO_TO_THE_POWER_OF_M30  (0.000000000931322574615478515625)
#define TWO_TO_THE_POWER_OF_M27  (0.000000007450580596923828125)
#define TWO_TO_THE_POWER_OF_M24  (0.000000059604644775390625)
#define TWO_TO_THE_POWER_OF_11   (2048)
#define TWO_TO_THE_POWER_OF_14   (16384)
#define TWO_TO_THE_POWER_OF_16   (65536) 


BOOL IONOSPHERE_GetL1KlobucharCorrection(   
  const double  alpha0,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s]
  const double  alpha1,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
  const double  alpha2,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^2]
  const double  alpha3,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^3]  
  const double  beta0,      //!< coefficients of a cubic equation representing the period of the model [s]
  const double  beta1,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle]
  const double  beta2,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^2]
  const double  beta3,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^3]
  const double  latitude,   //!< user geodetic latitude  [rad]
  const double  longitude,  //!< user geodetic longitude [rad]
  const double  elevation,  //!< elevation angle between the user and the satellite [rad]
  const double  azimuth,    //!< azimuth angle between the user and the satellite, measured clockwise positive from the true North [rad]   
  const double  gpstow,     //!< receiver computed gps time of week [s]
  double* ionospheric_delay //!< computed ionospheric correction [m]
  )
{
  double E;     // elevation angle between the user and the satellite [semi-circles]
  double x;     // phase [rad]
  double F;     // obliquity factor []
  double t;     // local time [s]
  double AMP;   // amplitude of the vertical delay [s]
  double PER;   // period of the model [s]
  double lat;   // user latitude [semi-circles]
  double lon;   // user longitude [semi-circles]
  double phi_m; // geomagnetic latitude of the earth projection of the ionospheric intersection point (mean ionospheric height assumed 350 km)[semi-circles]
  double lon_i; // geodetic longitude of the earth projection of the ionospheric intersection point [semi-circles]
  double lat_i; // geodetic latitude of the earth projection of the ionospheric intersection point [semi-circles]
  double central_angle; // earth's central angle between the user position and the earth projection of the ionospheric intersection point [semi-circles]

  // Check the input parameters. 
  // Refer to page 116 of the GPS Interface Control Document. Tabble 20-X Ionospheric Parameters.
  if( fabs(alpha0) > 128 * pow(2.0, -30.0) ) return FALSE;
  if( fabs(alpha1) > 128 * pow(2.0, -27.0) ) return FALSE;
  if( fabs(alpha2) > 128 * pow(2.0, -24.0) ) return FALSE;
  if( fabs(alpha3) > 128 * pow(2.0, -24.0) ) return FALSE;
  if( fabs(beta0)  > 128 * pow(2.0,  11.0) ) return FALSE;
  if( fabs(beta1)  > 128 * pow(2.0,  14.0) ) return FALSE;
  if( fabs(beta2)  > 128 * pow(2.0,  16.0) ) return FALSE;
  if( fabs(beta3)  > 128 * pow(2.0,  16.0) ) return FALSE;
  if( latitude > PI/2 || latitude < -PI/2 )
    return FALSE;
  if( gpstow < 0.0 )
    return FALSE;
   
  // convert to semi-circles
  lat = latitude  / PI;
  lon = longitude / PI;  
  E   = elevation / PI;
  
  // compute the central angle
  central_angle = 0.0137 / (E + 0.11) - 0.022;

  // lat of the intersection point
  lat_i = lat + central_angle * cos( azimuth );
  if( lat_i > 0.416 )
    lat_i = 0.416;
  else if( lat_i < -0.416 )
    lat_i = -0.416;

  // lon of the intersection point
  lon_i = lon + central_angle * sin( azimuth ) / cos( lat_i*PI );

  // local time [s], bounded (0-86400)
  t = 4.32e4 * lon_i + gpstow;
  while( !(t >= 0.0 && t <= 86400.0) )
  {
    if( t < 0.0 )
      t += 86400.0;
    else
      t -= 86400.0;
  }

  // geomagnetic latitude
  phi_m = lat_i + 0.064 * cos( (lon_i-1.617)*PI );

  // obliquity factor
  F = 1.0 + 16.0 * (0.53 - E)*(0.53 - E)*(0.53 - E);

  // period of the model
  PER = beta0 + beta1*phi_m + beta2*phi_m*phi_m + beta3*phi_m*phi_m*phi_m;
  if( PER < 72000.0 )
    PER = 72000.0;

  // amplitude of the vertical delay
  AMP = alpha0 + alpha1*phi_m + alpha2*phi_m*phi_m + alpha3*phi_m*phi_m*phi_m;
  if( AMP < 0.0 )
    AMP = 0.0;

  // phase
  x = TWOPI * ( t - 50400.0 ) / PER;

  // compute the ionospheric delay [s]
  if( x >= 1.57 || x <= -1.57 )
  {
    *ionospheric_delay = F * 5.0e-09;
  }
  else
  {
    *ionospheric_delay = F * (5.0e-09 + AMP * ( 1.0 - x*x/2.0 + x*x*x*x/24.0 ));     
  }

  // convert to [m]
  *ionospheric_delay *= LIGHTSPEED;

  return TRUE;
}

