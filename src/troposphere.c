/**
\file    troposphere.c
\brief   GNSS core 'c' function library: GPS tropospheric calculations
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2005-08-22

\b REFERENCES \n
- RTCA, 2001. MINIMUM OPERATIONAL PERFORMANCE STANDARDS FOR GLOBAL POSITIONING SYSTEM/WIDE AREA 
  AUGMENTATION SYSTEM AIRBORNE EQUIPMENT. RTCA/DO-229C. Prepared by SC-159. November 28, 2001. 
  Supersedes DO-229B. Available at http://www.rtca.org/doclist.asp . pp. 338-340 of 586 in PDF. \n
- Guo, J. and R. B. Langely 2003. A New Tropospheric Propagation Delay Mapping Function for 
  Elevation Angles Down to 2 degrees. ION GPS 2003, 9-12 Sept. 2003, Portland OR. \n
- Parkinson, B. and J. J. Spilker (1996). Global Positioning System: Theory And
  Applications Volume 1. American Institute of Aeronautics and Astronautics, Inc.,
  Washinton D.C. \n

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
#include "gnss_error.h"
#include "troposphere.h"
#include "constants.h"

/*
Local Preprocessor Constants
*/

// Constants used in Equation [A-6] and [A-7], p. 339
#define TROPOSPHERE_K1      (77.604)  // K/mbar
#define TROPOSPHERE_K2    (382000.0)  // K^2/mbar
#define TROPOSPHERE_RD     (287.054)  // J/kg/K
#define TROPOSPHERE_GM       (9.784)  // m/s^2

// Constant used in Equation [A-8] and [A-9], p. 340
#define TROPOSPHERE_G      (9.80665)  // m/s^2

// Average Standard Meteorological parameters from Table A-2, p. 339
#define TROPOSPHERE_AVERAGE_PRESSURE                {1013.25, 1017.25, 1015.75, 1011.75, 1013.00}  // mbar
#define TROPOSPHERE_AVERAGE_TEMPERATURE             { 299.65,  294.15,  283.15,  272.15,  263.65}  // K
#define TROPOSPHERE_AVERAGE_WATER_VAPOR_PRESSURE    {  26.31,   21.79,   11.66,    6.78,    4.11}  // mbar
#define TROPOSPHERE_AVERAGE_TEMPERATURE_LAPSE_RATE  {6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3}  // K/m
#define TROPOSPHERE_AVERAGE_WATER_VAPOR_LAPSE_RATE  {   2.77,    3.15,    2.57,    1.81,    1.55}  // dimensionless

// Seasonal Variation value from Table A-2, p. 339
#define TROPOSPHERE_SEASONAL_VARIATION_PRESSURE               {0.0,   -3.75,   -2.25,   -1.75,   -0.50}  // mbar
#define TROPOSPHERE_SEASONAL_VARIATION_TEMPERATURE            {0.0,    7.00,   11.00,   15.00,   14.50}  // K
#define TROPOSPHERE_SEASONAL_VARIATION_WATER_VAPOR_PRESSURE   {0.0,    8.85,    7.24,    5.36,    3.39}  // mbar
#define TROPOSPHERE_SEASONAL_VARIATION_TEMPERATURE_LAPSE_RATE {0.0, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3}  // K/m
#define TROPOSPHERE_SEASONAL_VARIATION_WATER_VAPOR_LAPSE_RATE {0.0,    0.33,    0.46,    0.74,    0.30}  // dimensionless


// local constant global variables
static const double  P_Table[5] = TROPOSPHERE_AVERAGE_PRESSURE;
static const double dP_Table[5] = TROPOSPHERE_SEASONAL_VARIATION_PRESSURE;
static const double  T_Table[5] = TROPOSPHERE_AVERAGE_TEMPERATURE;
static const double dT_Table[5] = TROPOSPHERE_SEASONAL_VARIATION_TEMPERATURE;
static const double  E_Table[5] = TROPOSPHERE_AVERAGE_WATER_VAPOR_PRESSURE;
static const double dE_Table[5] = TROPOSPHERE_SEASONAL_VARIATION_WATER_VAPOR_PRESSURE;
static const double  B_Table[5] = TROPOSPHERE_AVERAGE_TEMPERATURE_LAPSE_RATE;
static const double dB_Table[5] = TROPOSPHERE_SEASONAL_VARIATION_TEMPERATURE_LAPSE_RATE;
static const double  L_Table[5] = TROPOSPHERE_AVERAGE_WATER_VAPOR_LAPSE_RATE;
static const double dL_Table[5] = TROPOSPHERE_SEASONAL_VARIATION_WATER_VAPOR_LAPSE_RATE;


/*
DESRIPTION
Performs linear interpolation given (x1,y1), (x2,y2), and x, returns y
*/
static void _TROPOSPHERE_Interpolate( 
 const double x1,
 const double y1,
 const double x2,
 const double y2,
 const double x,
 double* y );




void TROPOSPHERE_GetDayAndWetMappingValues_UsingThe_UNBabc_MappingFunction( 
  const double elevation,  // satellite elevation angle (must be >= 2 deg)  [rad]
  const double latitude,   // user latitude                                 [rad]
  const double height,     // user height (orthometric, ie above sea level) [m]
  double* drymap,          // dry delay scale factor                        []
  double* wetmap )         // wet delay scale factor                        []
{
  double a,    // mapping value parameter (see Eqn 2, [2])
         b,    // mapping value parameter (see Eqn 2, [2])
         c,    // mapping value parameter (see Eqn 2, [2])
         num,  // numerator
         dem,  // denominator
         sine; // sin(elevation)

  if( elevation > HALFPI )
    sine = 1.0; // user error, sine(HALFPI)
  else if( elevation < 2.0*DEG2RAD )
    sine = sin(2.0*DEG2RAD); // this mapping function is only good to 2 degrees
  else
    sine = sin(elevation);
  
  // dry
  a = (1.18972 - 0.026855*height/1000.0 + 0.10664*cos(latitude)) / 1000.0;
  b = 0.0035716;
  c = 0.082456;
  
  num = 1.0 + ( a / ( 1.0 + ( b / ( 1.0 + c ) ) ) );
  dem = sine + ( a / ( sine + ( b / ( sine + c ) ) ) );
  
  *drymap = num/dem;  
  
  // wet
  a = (0.61120 - 0.035348*height/1000.0 - 0.01526*cos(latitude)) / 1000.0;
  b = 0.0018576;
  c = 0.062741;
  
  num = 1.0 + ( a / ( 1.0 + ( b / ( 1.0 + c ) ) ) );
  dem = sine + ( a / ( sine + ( b / ( sine + c ) ) ) );
  
  *wetmap = num/dem;  
}


void TROPOSPHERE_DetermineZenithDelayValues_WAAS_Model(
  const double latitude,          //!< user latitude        [rad]
  const double height,            //!< user height          [m]
  const unsigned short dayofyear, //!< day of year (1-366)  [days]    
  double* zenith_dry_delay,       //!< dry zenith delay     [m]
  double* zenith_wet_delay        //!< wet zenith delay     [m]
  )
{
  // interpolated meteorological values
  double P; // pressure,               [mbar]
  double T; // temperature,            [K]
  double E; // wator vapor pressure,   [mbar]
  double B; // temperature lapse rate, [K/m]
  double L; // water vapor lapse rate, []

  double Dmin;    //day of year min constant   
  double d;       // temporary double
  double abs_lat; // absolute latitude [deg]
  double common;  // a common factor
  double base;
  double power;
  int i;

  abs_lat = fabs(latitude)*RAD2DEG;

  // determine day of year min constant   
  if( latitude < 0 )
    Dmin = 211.0; // Southern hemisphere
  else
    Dmin = 28.0;  // Northern hemisphere
  
  // common part of Equation [A-3]
  common = cos( (TWOPI * (dayofyear - Dmin)) / 365.25 );

  if( abs_lat <= 15.0 )
  {
    P = P_Table[0] - dP_Table[0] * common;
    T = T_Table[0] - dT_Table[0] * common;
    E = E_Table[0] - dE_Table[0] * common;
    B = B_Table[0] - dB_Table[0] * common;
    L = L_Table[0] - dL_Table[0] * common;
  }
  else if( abs_lat > 15.0 && abs_lat < 75.0 )
  {
    i = ((int)abs_lat) / 15;

    _TROPOSPHERE_Interpolate( i*15.0,  P_Table[i-1], (i+1)*15.0,  P_Table[i], abs_lat, &P );
    _TROPOSPHERE_Interpolate( i*15.0, dP_Table[i-1], (i+1)*15.0, dP_Table[i], abs_lat, &d );    
    P = P - d * common;

    _TROPOSPHERE_Interpolate( i*15.0,  T_Table[i-1], (i+1)*15.0,  T_Table[i], abs_lat, &T );
    _TROPOSPHERE_Interpolate( i*15.0, dT_Table[i-1], (i+1)*15.0, dT_Table[i], abs_lat, &d );    
    T = T - d * common;

    _TROPOSPHERE_Interpolate( i*15.0,  E_Table[i-1], (i+1)*15.0,  E_Table[i], abs_lat, &E );
    _TROPOSPHERE_Interpolate( i*15.0, dE_Table[i-1], (i+1)*15.0, dE_Table[i], abs_lat, &d );    
    E = E - d * common;

    _TROPOSPHERE_Interpolate( i*15.0,  B_Table[i-1], (i+1)*15.0,  B_Table[i], abs_lat, &B );
    _TROPOSPHERE_Interpolate( i*15.0, dB_Table[i-1], (i+1)*15.0, dB_Table[i], abs_lat, &d );    
    B = B - d * common;

    _TROPOSPHERE_Interpolate( i*15.0,  L_Table[i-1], (i+1)*15.0,  L_Table[i], abs_lat, &L );
    _TROPOSPHERE_Interpolate( i*15.0, dL_Table[i-1], (i+1)*15.0, dL_Table[i], abs_lat, &d );    
    L = L - d * common;    
  }
  else // abs_lat >= 75.0
  {
    P = P_Table[4] - dP_Table[4] * common;
    T = T_Table[4] - dT_Table[4] * common;
    E = E_Table[4] - dE_Table[4] * common;
    B = B_Table[4] - dB_Table[4] * common;
    L = L_Table[4] - dL_Table[4] * common;
  }

  // zero altitude zenith dry delay, Equation [A-6]
  *zenith_dry_delay = (1.0e-6 * TROPOSPHERE_K1 * TROPOSPHERE_RD * P) / TROPOSPHERE_GM;

  // zero altitude zenith wet delay, Equation [A-7]
  *zenith_wet_delay = (((1.0e-6 * TROPOSPHERE_K2 * TROPOSPHERE_RD) / (TROPOSPHERE_GM * (L + 1.0) - B * TROPOSPHERE_RD)) * (E / T));

  // for Equations [A-8] and [A-9]
  base  = 1.0 - ((B * height) / T);

  // zenith dry delay with height compensation, Equation [A-8]  
  power = (TROPOSPHERE_G / (TROPOSPHERE_RD * B));
  *zenith_dry_delay = pow(base, power) * (*zenith_dry_delay);

  // zenith wet delay with height compensation, Equation [A-9]
  power = (((L + 1.0) * TROPOSPHERE_G) / (TROPOSPHERE_RD * B)) - 1.0;
  *zenith_wet_delay = pow(base, power) * (*zenith_wet_delay);
}



void TROPOSPHERE_GetDryAndWetDelay_UsingThe_UNBabc_MappingFunction(  
  const double zenith_dry_delay,  //!< dry zenith delay                              [m]
  const double zenith_wet_delay,  //!< wet zenith delay                              [m]
  const double elevation,         //!< satellite elevation angle (must be >= 2 deg)  [rad]
  const double latitude,          //!< user latitude                                 [rad]
  const double height,            //!< user height (orthometric, ie above sea level) [m]  
  double*      drydelay,          //!< dry delay mapped to this elevation angle      [m]
  double*      wetdelay           //!< wet delay mapped to this elevation angle      [m]
  )
{
  double drymap;
  double wetmap;

  TROPOSPHERE_GetDayAndWetMappingValues_UsingThe_UNBabc_MappingFunction(
    elevation,
    latitude,
    height,
    &drymap,
    &wetmap );

  *drydelay = zenith_dry_delay*drymap;
  *wetdelay = zenith_wet_delay*wetmap;
} 




/*
DESRIPTION
Performs linear interpolation given (x1,y1), (x2,y2), and x, returns y
*/
static void _TROPOSPHERE_Interpolate( 
 const double x1,
 const double y1,
 const double x2,
 const double y2,
 const double x,
 double* y )
{
  double dx;
  dx = x2 - x1;

  // check divide by zero
  if( dx < 1e-25 && dx > -1e-25 )
    *y = y1;
    
  *y = y1 + (x - x1)/dx * (y2 - y1);
}




