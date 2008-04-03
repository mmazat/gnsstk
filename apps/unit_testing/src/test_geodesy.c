/** 
\file    test_geodesy.c
\brief   unit tests for geodesy.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-11-26
\since   2007-11-26

2007-11-26, GDM, Creation \n

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
#include <stdio.h>
#include "Basic.h"     // CUnit/Basic.h
#include "gnss_error.h"
#include "geodesy.h"
#include "constants.h"


int init_suite_GEODESY(void)
{
  return 0;
}

int clean_suite_GEODESY(void)
{
  return 0;
}



void test_GEODESY_GetReferenceEllipseParameters(void)
{
  GEODESY_enumReferenceEllipse ellipse; // reference ellipse enumerated          []
  double a;      // semi-major axis of the reference ellipse                     [m]
  double b;      // semi-minor axis of the reference ellipse (b = a - a*f_inv)   [m] 
  double f_inv;  // inverse of the flattening of the reference ellipse           []
  double e2;     // eccentricity of the reference ellipse (e2 = (a*a-b*b)/(a*a)) [] 
  BOOL result;
  
  ellipse = GEODESY_REFERENCE_ELLIPSE_WGS84;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,     GEODESY_REFERENCE_ELLIPSE_WGS84_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,     GEODESY_REFERENCE_ELLIPSE_WGS84_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv, GEODESY_REFERENCE_ELLIPSE_WGS84_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,    GEODESY_REFERENCE_ELLIPSE_WGS84_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_AIRY_1830;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_AIRY_1830_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_AIRY_1830_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_AIRY_1830_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_AIRY_1830_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_MODIFED_AIRY;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_MODIFED_AIRY_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_MODIFED_AIRY_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_MODIFED_AIRY_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_MODIFED_AIRY_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_AUSTRALIAN_NATIONAL;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_AUSTRALIAN_NATIONAL_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_AUSTRALIAN_NATIONAL_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_AUSTRALIAN_NATIONAL_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_AUSTRALIAN_NATIONAL_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_BESSEL_1841;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_BESSEL_1841_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_BESSEL_1841_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_BESSEL_1841_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_BESSEL_1841_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_CLARKE_1866;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_CLARKE_1866_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_CLARKE_1866_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_CLARKE_1866_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_CLARKE_1866_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_CLARKE_1880;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_CLARKE_1880_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_CLARKE_1880_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_CLARKE_1880_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_CLARKE_1880_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_EVEREST_INDIA_1830;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_EVEREST_INDIA_1830_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_EVEREST_INDIA_1830_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_EVEREST_INDIA_1830_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_EVEREST_INDIA_1830_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_EVEREST_BRUNEI_E_MALAYSIA;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_EVEREST_BRUNEI_E_MALAYSIA_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_EVEREST_BRUNEI_E_MALAYSIA_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_EVEREST_BRUNEI_E_MALAYSIA_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_EVEREST_BRUNEI_E_MALAYSIA_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_EVEREST_W_MALAYSIA_SINGAPORE;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_EVEREST_W_MALAYSIA_SINGAPORE_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_EVEREST_W_MALAYSIA_SINGAPORE_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_EVEREST_W_MALAYSIA_SINGAPORE_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_EVEREST_W_MALAYSIA_SINGAPORE_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_GRS_1980;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_GRS_1980_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_GRS_1980_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_GRS_1980_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_GRS_1980_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_HELMERT_1906;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_HELMERT_1906_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_HELMERT_1906_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_HELMERT_1906_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_HELMERT_1906_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_HOUGH_1960;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_HOUGH_1960_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_HOUGH_1960_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_HOUGH_1960_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_HOUGH_1960_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_INTERNATIONAL_1924;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_INTERNATIONAL_1924_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_INTERNATIONAL_1924_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_INTERNATIONAL_1924_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_INTERNATIONAL_1924_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_SOUTH_AMERICAN_1969;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_SOUTH_AMERICAN_1969_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_SOUTH_AMERICAN_1969_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_SOUTH_AMERICAN_1969_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_SOUTH_AMERICAN_1969_E2,    1e-18 );

  ellipse = GEODESY_REFERENCE_ELLIPSE_WGS72;
  GEODESY_GetReferenceEllipseParameters( 
    ellipse,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT_DOUBLE_EQUAL( a,      GEODESY_REFERENCE_ELLIPSE_WGS72_A,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( b,      GEODESY_REFERENCE_ELLIPSE_WGS72_B,     1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( f_inv,  GEODESY_REFERENCE_ELLIPSE_WGS72_F_INV, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( e2,     GEODESY_REFERENCE_ELLIPSE_WGS72_E2,    1e-18 );

  GNSS_ERROR_MSG( "Performing an invalid call to GEODESY_GetReferenceEllipseParameters on purpose" );
  result = GEODESY_GetReferenceEllipseParameters( 
    1000,
    &a,
    &b,
    &f_inv,
    &e2 );
  CU_ASSERT( result == FALSE );  
}


void test_GEODESY_ConvertCoordinates(void)
{
  int i, j;
  double lat,lon,hgt;
  double x,y,z;
  BOOL result;

  // test special cases
  lat = 0;
  lon = 0;
  hgt = 0;

  result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    lat,
    lon,
    hgt,
    &x,
    &y,
    &z );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( x, GEODESY_REFERENCE_ELLIPSE_WGS84_A, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( y, 0.0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( z, 0.0, 1e-7 );

  lat = 90;
  lon = 0;
  hgt = 0;

  result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    lat*DEG2RAD,
    lon*DEG2RAD,
    hgt,
    &x,
    &y,
    &z );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( x, 0.0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( y, 0.0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( z, GEODESY_REFERENCE_ELLIPSE_WGS84_B, 1e-7 );

  lat = -90;
  lon = 0;
  hgt = 0;

  result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    lat*DEG2RAD,
    lon*DEG2RAD,
    hgt,
    &x,
    &y,
    &z );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( x, 0.0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( y, 0.0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( z, -1.0*GEODESY_REFERENCE_ELLIPSE_WGS84_B, 1e-7 );


  for( i = -90; i <= 90; i++ )
  {
    for( j = -180; j <= 180; j++ )
    {
      lat = (double)i;
      lon = (double)j;

      result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        lat*DEG2RAD,
        lon*DEG2RAD,
        2000.0,
        &x,
        &y,
        &z );
      CU_ASSERT_FATAL( result );
      
      result = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        x,
        y,
        z,
        &lat,
        &lon,
        &hgt );
      CU_ASSERT_FATAL( result );

      lat *= RAD2DEG;
      lon *= RAD2DEG;
      CU_ASSERT_DOUBLE_EQUAL( lat, (double)(i), 1e-11 );      
      CU_ASSERT_DOUBLE_EQUAL( lon, (double)(j), 1e-11 );      
      CU_ASSERT_DOUBLE_EQUAL( hgt, 2000.0, 1e-5 );      
    }
  }  
}


void test_GEODESY_ComputeNorthingEastingVertical(void)
{
  double referenceLatitude;  // datum geodetic latitude  [rad]
  double referenceLongitude; // datum geodetic longitude [rad]
  double referenceHeight;    // datum geodetic height    [m]
  double latitude;           // geodetic latitude        [rad]
  double longitude;          // geodetic longitude       [rad]
  double height;             // geodetic height          [m]
  double northing;           // local geodetic northing  [m]
  double easting;            // local geodetic easting   [m]
  double vertical;           // local geodetic vertical  [m]
  double M;                  // meridian radius of curvature [m]

  double arc;
  double deltaNorthing;
  BOOL result;

  
  referenceLatitude = 0.0;
  referenceLongitude = 0.0;
  referenceHeight = 10.0;

  latitude  = 0.00001;
  longitude = 0.0;
  height = 0.0;
  
  result = GEODESY_ComputeNorthingEastingVertical(    
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    referenceHeight,
    latitude,
    longitude,
    height,
    &northing,
    &easting,
    &vertical
    );
  CU_ASSERT_FATAL( result );

  // The arc difference should be very similar for small differences.
  result = GEODESY_ComputeMeridianRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude, 
    &M );
  CU_ASSERT_FATAL( result );

  deltaNorthing = (M)*latitude;
  CU_ASSERT_DOUBLE_EQUAL( northing, deltaNorthing, 1e-4 );
  CU_ASSERT_DOUBLE_EQUAL( easting, 0.0, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( vertical, -10.0, 1e-2 );

  latitude  = 0.0;
  longitude = 0.00001;
  height = 0.0;
  
  result = GEODESY_ComputeNorthingEastingVertical(    
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    referenceHeight,
    latitude,
    longitude,
    height,
    &northing,
    &easting,
    &vertical
    );
  CU_ASSERT_FATAL( result );

  // The arc distance should be very similar for short distances.
  result = GEODESY_ComputeParallelArcBetweenTwoLongitudes( 
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    longitude,
    &arc );
  CU_ASSERT_FATAL( result );  
  CU_ASSERT_DOUBLE_EQUAL( northing, 0.0, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( easting, arc, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( vertical, -10.0, 1e-2 );
}


void test_GEODESY_ComputePositionDifference(void)
{
  double referenceLatitude;  // datum geodetic latitude  [rad]
  double referenceLongitude; // datum geodetic longitude [rad]
  double referenceHeight;    // datum geodetic height    [m]
  double latitude;           // geodetic latitude        [rad]
  double longitude;          // geodetic longitude       [rad]
  double height;             // geodetic height          [m]
  double deltaNorthing;      // northing difference  [m]
  double deltaEasting;       // easting difference   [m]
  double deltaVertical;      // vertical difference  [m]
  double M;                  // meridian radius of curvature [m]

  double dNorthing;
  double arc;
  BOOL result;
  
  referenceLatitude  = 0.0;
  referenceLongitude = 0.0;
  referenceHeight = 10.0;

  latitude  = 0.00001;
  longitude = 0.0;
  height = 0.0;
  
  result = GEODESY_ComputePositionDifference(    
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    referenceHeight,
    latitude,
    longitude,
    height,
    &deltaNorthing,
    &deltaEasting,
    &deltaVertical
    );
  CU_ASSERT_FATAL( result );

  result = GEODESY_ComputeMeridianRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude, 
    &M );
  CU_ASSERT_FATAL( result );

  dNorthing = (M)*latitude;
  CU_ASSERT_DOUBLE_EQUAL( deltaNorthing, dNorthing, 1e-4 );
  CU_ASSERT_DOUBLE_EQUAL( deltaEasting, 0.0, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( deltaVertical, -10.0, 1e-2 );

  latitude  = 0.0;
  longitude = 0.00001;
  height = 0.0;
  
  result = GEODESY_ComputePositionDifference(    
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    referenceHeight,
    latitude,
    longitude,
    height,
    &deltaNorthing,
    &deltaEasting,
    &deltaVertical
    );
  CU_ASSERT_FATAL( result );

  
  // The arc distance should be very similar for short distances.
  result = GEODESY_ComputeParallelArcBetweenTwoLongitudes( 
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude,
    referenceLongitude,
    longitude,
    &arc );
  CU_ASSERT_FATAL( result ); 

  CU_ASSERT_DOUBLE_EQUAL( deltaNorthing, 0.0, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( deltaEasting, arc,  1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( deltaVertical, -10.0, 1e-2 );
}


void test_GEODESY_ComputeMeridianRadiusOfCurvature(void)
{
  double latitude; // [rads]
  double M; // [m]
  double tM; // test value for M [m]
  BOOL result;

  // test the equitorial and polar values

  latitude = 0;
  result = GEODESY_ComputeMeridianRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    latitude,
    &M );
  CU_ASSERT_FATAL( result );
  tM = GEODESY_REFERENCE_ELLIPSE_WGS84_A*(1.0-GEODESY_REFERENCE_ELLIPSE_WGS84_E2);
  CU_ASSERT_DOUBLE_EQUAL( M, tM, 1e-06 );

  latitude = 90;
  result = GEODESY_ComputeMeridianRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    latitude*DEG2RAD,
    &M );
  CU_ASSERT_FATAL( result );
  tM = GEODESY_REFERENCE_ELLIPSE_WGS84_A/sqrt(1.0-GEODESY_REFERENCE_ELLIPSE_WGS84_E2);
  CU_ASSERT_DOUBLE_EQUAL( M, tM, 1e-06 );
}

void test_GEODESY_ComputePrimeVerticalRadiusOfCurvature(void)
{
  double latitude; // [rads]
  double N; // [m]
  double tN; // test value for N [m]
  BOOL result;

  // test the equitorial and polar values
  latitude = 0;
  result = GEODESY_ComputePrimeVerticalRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    latitude,
    &N );
  CU_ASSERT_FATAL( result );
  tN = GEODESY_REFERENCE_ELLIPSE_WGS84_A;
  CU_ASSERT_DOUBLE_EQUAL( N, GEODESY_REFERENCE_ELLIPSE_WGS84_A, 1e-06 );

  latitude = 90;
  result = GEODESY_ComputePrimeVerticalRadiusOfCurvature(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    latitude*DEG2RAD,
    &N );
  CU_ASSERT_FATAL( result );
  tN = GEODESY_REFERENCE_ELLIPSE_WGS84_A/sqrt(1.0-GEODESY_REFERENCE_ELLIPSE_WGS84_E2);
  CU_ASSERT_DOUBLE_EQUAL( N, tN, 1e-06 );
}


void test_GEODESY_ComputeMeridianArcBetweenTwoLatitudes(void)
{
  double referenceLatitude = 40; // [deg]
  double latitude = 40.0 + 1.0/3600.0; // [deg]
  double arc; //[m]
  BOOL result;

  // see [1] Schwartz, K. P. (1997). ENGO 421 Lecture Notes - Fundamentals of Geodesy. 
  //     Chapter 3, pp. 61

  result = GEODESY_ComputeMeridianArcBetweenTwoLatitudes(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude*DEG2RAD,
    latitude*DEG2RAD,
    &arc ); 
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( arc, 30.9, 0.1 );
}

void test_GEODESY_ComputeParallelArcBetweenTwoLongitudes(void)
{
  double referenceLatitude = 40; // [deg]
  double referenceLongitude = 0; // [deg]
  double longitude = 1.0/3600.0; // [deg]
  double arc; //[m]
  BOOL result;

  // see [1] Schwartz, K. P. (1997). ENGO 421 Lecture Notes - Fundamentals of Geodesy. 
  //     Chapter 3, pp. 62

  result = GEODESY_ComputeParallelArcBetweenTwoLongitudes(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    referenceLatitude*DEG2RAD,
    referenceLongitude*DEG2RAD,
    longitude*DEG2RAD,
    &arc ); 
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( arc, 23.7, 0.1 );
}

void test_GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(void)
{
  double referenceLatitude;  // reference geodetic latitude                 [rad]
  double referenceLongitude; // reference geodetic longitude                [rad]
  double dX;                 // earth centered earth fixed vector component [m]
  double dY;                 // earth centered earth fixed vector component [m]
  double dZ;                 // earth centered earth fixed vector component [m]
  double dN;                 // local geodetic northing vector component    [m]
  double dE;                 // local geodetic easting  vector component    [m]
  double dUp;                // local geodetic vertical vector component    [m]
  BOOL result;

  referenceLatitude = 0;
  referenceLongitude = 0;
  dN = 0.0;
  dE = 1.0;
  dUp = 0.0;  
  
  result = GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
    referenceLatitude,
    referenceLongitude,
    dN,
    dE,
    dUp,
    &dX,
    &dY,
    &dZ );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( dX, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dY, 1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dZ, 0.0, 1.0e-06 );

  dN = 1.0;
  dE = 0.0;
  dUp = 0.0;  
  
  result = GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
    referenceLatitude,
    referenceLongitude,
    dN,
    dE,
    dUp,
    &dX,
    &dY,
    &dZ );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( dX, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dY, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dZ, 1.0, 1.0e-06 );

  dN = 0.0;
  dE = 0.0;
  dUp = 1.0;  
  
  result = GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
    referenceLatitude,
    referenceLongitude,
    dN,
    dE,
    dUp,
    &dX,
    &dY,
    &dZ );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( dX, 1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dY, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dZ, 0.0, 1.0e-06 );

  referenceLatitude = 89.99999999*DEG2RAD;
  referenceLongitude = 0;  

  dN = 1.0;
  dE = 2.0;
  dUp = 3.0;  

  result = GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
    referenceLatitude,
    referenceLongitude,
    dN,
    dE,
    dUp,
    &dX,
    &dY,
    &dZ );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( dX, -1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dY, 2.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dZ, 3.0, 1.0e-06 );
}

void test_GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(void)
{
  double referenceLatitude;  // reference geodetic latitude                 [rad]
  double referenceLongitude; // reference geodetic longitude                [rad]
  double dX;                 // earth centered earth fixed vector component [m]
  double dY;                 // earth centered earth fixed vector component [m]
  double dZ;                 // earth centered earth fixed vector component [m]
  double dN;                 // local geodetic northing vector component    [m]
  double dE;                 // local geodetic easting  vector component    [m]
  double dUp;                // local geodetic vertical vector component    [m]
  BOOL result;

  referenceLatitude = 0;
  referenceLongitude = 0;
  dX = 1.0;
  dY = 0.0;
  dZ = 0.0;
  
  result = GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(
    referenceLatitude,
    referenceLongitude,
    dX,
    dY,
    dZ,
    &dN,
    &dE,
    &dUp );
  CU_ASSERT_FATAL( result );    
  CU_ASSERT_DOUBLE_EQUAL( dN, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dE, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dUp, 1.0, 1.0e-06 );

  dX = 0.0;
  dY = 1.0;
  dZ = 0.0;  

  result = GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(
    referenceLatitude,
    referenceLongitude,
    dX,
    dY,
    dZ,
    &dN,
    &dE,
    &dUp );
  CU_ASSERT_FATAL( result );    
  CU_ASSERT_DOUBLE_EQUAL( dN, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dE, 1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dUp, 0.0, 1.0e-06 );

  dX = 0.0;
  dY = 0.0;
  dZ = 1.0;  

  result = GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(
    referenceLatitude,
    referenceLongitude,
    dX,
    dY,
    dZ,
    &dN,
    &dE,
    &dUp );
  CU_ASSERT_FATAL( result );    
  CU_ASSERT_DOUBLE_EQUAL( dN, 1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dE, 0.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dUp, 0.0, 1.0e-06 );
 
  referenceLatitude = 89.99999999*DEG2RAD;
  referenceLongitude = 0;  

  dX = -1.0;
  dY = 2.0;
  dZ = 3.0;  

  result = GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(
    referenceLatitude,
    referenceLongitude,
    dX,
    dY,
    dZ,
    &dN,
    &dE,
    &dUp );
  CU_ASSERT_FATAL( result );    
  CU_ASSERT_DOUBLE_EQUAL( dN,  1.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dE,  2.0, 1.0e-06 );
  CU_ASSERT_DOUBLE_EQUAL( dUp, 3.0, 1.0e-06 );

}

void test_GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(void)
{
  double elevation;  // elevation angle [rad]
  double azimuth;    // azimuth angle   [rad]
  BOOL result;
    
  result = GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    &elevation,
    &azimuth 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( elevation*RAD2DEG, 90.0, 1.0e-4 );
  CU_ASSERT_DOUBLE_EQUAL( azimuth, 0.0, 1.0e-8 );


  result = GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    1.0,
    &elevation,
    &azimuth 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( elevation*RAD2DEG, 45.0, 1.0e-4 );
  CU_ASSERT_DOUBLE_EQUAL( azimuth, PI, 1.0E-07 );
  
}
