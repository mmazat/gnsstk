/**
\file    geodesy_main.c
\brief   'c' main program to use the function library geodesy.h/.c
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2007-04-20

\b "LICENSE INFORMATION" \n
Copyright (c) 2006, refer to 'author' doxygen tags \n
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
#include <stdlib.h>
#include "geodesy.h"
#include "constants.h"

/// \brief  A  main() for a geodesy application.
int main( int argc, char* argv[] )
{
  const int min_nr_args = 3;
  int ellipse=0;
  int fn_choice=0;
  GEODESY_enumReferenceEllipse  referenceEllipse = GEODESY_REFERENCE_ELLIPSE_WGS84;
  BOOL result = FALSE;

  if( argc < min_nr_args )
  {
    printf("\nUSAGE\n");
    printf("geodesy <ellipse> <function> <additional arguments for function ... >\n");
    printf("\nellipse:\n");
    printf("0,   WGS84\n");
    printf("1,   Airy 1830\n");
    printf("2,   Modified Airy\n");
    printf("3,   Australian National\n");
    printf("4,   Bessel 1841\n");
    printf("5,   Clarke 1866\n");
    printf("6,   Clarke 1880\n");
    printf("7,   Everest(India 1830)\n");
    printf("8,   Everest(Brunei & E.Malaysia)\n");
    printf("9,   Everest(W.Malaysia & Singapore)\n");
    printf("10,  Geodetic Reference System 1980\n");
    printf("11,  Helmert 1906\n");
    printf("12,  Hough 1960\n");
    printf("13,  International 1924\n");
    printf("14,  South American 1969\n");
    printf("15,  World Geodetic System 1972\n");

    printf("\nfunction and additional arguments:\n");
    printf("0, GEODESY_GetReferenceEllipseParameters\n");
    printf("   requires no additions arguments\n");
    printf("1, GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates\n");
    printf("   latitude[deg], longitude[deg], height[m]\n");
    printf("2, GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates\n");
    printf("   userX[m], userY[m], userZ[m]\n");
    printf("3, GEODESY_ComputeNorthingEastingVertical\n");
    printf("   ref_lat[deg], ref_lon[deg] ref_hgt[m], lat[deg], lon[deg], hgt[m]\n");
    printf("4  ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame\n" );
    printf("   fromX[m], fromY[m], fromZ[m], toX[m], toY[m], toZ[m]\n" );
    printf("5, GEODESY_ComputePositionDifference\n");
    printf("   ref_lat[deg], ref_lon[deg] ref_hgt[m], lat[deg], lon[deg], hgt[m]\n");

    return 0;
  }

  ellipse = atoi(argv[1]);
  if( ellipse > 15 )
  { 
    printf("Invalid ellipse argument\n");
    return -1;
  }
  referenceEllipse = (GEODESY_enumReferenceEllipse)ellipse;

  fn_choice = atoi(argv[2]);
  if( fn_choice < 0 || fn_choice > 5 )
  { 
    printf("Invalid function argument\n");
    return -1;
  }


  // Check the number of required arguments
  switch(fn_choice)
  {
    case 0: if( argc != 3 ){ printf("Invalid function arguments\n"); return -1; } break;
    case 1: if( argc != 6 ){ printf("Invalid function arguments\n"); return -1; } break;
    case 2: if( argc != 6 ){ printf("Invalid function arguments\n"); return -1; } break;
    case 3: if( argc != 9 ){ printf("Invalid function arguments\n"); return -1; } break;
    case 4: if( argc != 9 ){ printf("Invalid function arguments\n"); return -1; } break;
    case 5: if( argc != 6 ){ printf("Invalid function arguments\n"); return -1; } break;
    default: break;
  }

  switch(fn_choice)
  {
    case 0: 
    {
      double a;      //!< semi-major axis of the reference ellipse                     [m]
      double b;      //!< semi-minor axis of the reference ellipse (b = a - a*f_inv)   [m] 
      double f_inv;  //!< inverse of the flattening of the reference ellipse           []
      double e2;     //!< eccentricity of the reference ellipse (e2 = (a*a-b*b)/(a*a)) [] 

      result = GEODESY_GetReferenceEllipseParameters( referenceEllipse, //!< reference ellipse enumerated    []
        &a,      //!< semi-major axis of the reference ellipse                     [m]
        &b,      //!< semi-minor axis of the reference ellipse (b = a - a*f_inv)   [m] 
        &f_inv,  //!< inverse of the flattening of the reference ellipse           []
        &e2      //!< eccentricity of the reference ellipse (e2 = (a*a-b*b)/(a*a)) [] 
      );
      if( result == FALSE )
      {
        printf("GEODESY_GetReferenceEllipseParameters returned FALSE\n");
        return -1;
      }

      printf( "GEODESY_GetReferenceEllipseParameters\n" );
      printf( "ellipse = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "a       [m] = %20.3f\n", a );
      printf( "b       [m] = %20.9f\n", b );
      printf( "f_inv   []  = %20.11f\n", f_inv );
      printf( "e2      []  = %.15f\n\n", e2 );
      break;
    }
    case 1: 
    {
      double latitude;   //!< geodetic latitude                [rad]
      double longitude;  //!< geodetic longitude               [rad]
      double height;     //!< geodetic height                  [m]
      double x;          //!< earth fixed cartesian coordinate [m]
      double y;          //!< earth fixed cartesian coordinate [m]
      double z;          //!< earth fixed cartesian coordinate [m]

      latitude  = atof( argv[3] )*DEG2RAD;
      longitude = atof( argv[4] )*DEG2RAD;
      height    = atof( argv[5] );

      result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        referenceEllipse,  //!< reference ellipse enumerated []
        latitude,   //!< geodetic latitude                [rad]
        longitude,  //!< geodetic longitude               [rad]
        height,     //!< geodetic height                  [m]
        &x,               //!< earth fixed cartesian coordinate [m]
        &y,               //!< earth fixed cartesian coordinate [m]
        &z                //!< earth fixed cartesian coordinate [m]
      );
      if( result == FALSE )
      {
        printf("GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned FALSE\n");
        return -1;
      }
      printf( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates\n" );
      printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "latitude  [deg] = %.12lf\n", latitude*RAD2DEG );
      printf( "longitude [deg] = %.12lf\n", longitude*RAD2DEG );
      printf( "height    [m]   = %.4lf\n", height );
      printf( "x         [m]   = %.4lf\n", x );
      printf( "y         [m]   = %.4lf\n", y );
      printf( "z         [m]   = %.4lf\n\n", z );
      break;
    }
    case 2:
    {
      double latitude;   //!< geodetic latitude                [rad]
      double longitude;  //!< geodetic longitude               [rad]
      double height;     //!< geodetic height                  [m]
      double x;          //!< earth fixed cartesian coordinate [m]
      double y;          //!< earth fixed cartesian coordinate [m]
      double z;          //!< earth fixed cartesian coordinate [m]

      x = atof( argv[3] );
      y = atof( argv[4] );
      z = atof( argv[5] );

      result = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
        referenceEllipse,  //!< reference ellipse enumerated []     
        x,              // earth fixed cartesian coordinate [m]
        y,              // earth fixed cartesian coordinate [m]
        z,              // earth fixed cartesian coordinate [m]  
        &latitude,      // geodetic latitude                [rad]
        &longitude,     // geodetic longitude               [rad]
        &height         // geodetic height                  [m]
      );
      if( result == FALSE )
      {
        printf("GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates returned FALSE\n");
        return -1;
      }

      printf( "GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates\n" );
      printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "latitude  [deg] = %.12lf\n", latitude*RAD2DEG );
      printf( "longitude [deg] = %.12lf\n", longitude*RAD2DEG );
      printf( "height    [m]   = %.4lf\n", height );
      printf( "x         [m]   = %.4lf\n", x );
      printf( "y         [m]   = %.4lf\n", y );
      printf( "z         [m]   = %.4lf\n\n", z );
      break;
    }
    case 3:
    {
      double referenceLatitude;  //!< datum geodetic latitude  [rad]
      double referenceLongitude; //!< datum geodetic longitude [rad]
      double referenceHeight;    //!< datum geodetic height    [m]
      double latitude;           //!< geodetic latitude        [rad]
      double longitude;          //!< geodetic longitude       [rad]
      double height;             //!< geodetic height          [m]
      double northing;           //!< local geodetic northing  [m]
      double easting;            //!< local geodetic easting   [m]
      double vertical;           //!< local geodetic vertical  [m]
      
      referenceLatitude = atof(argv[3])*DEG2RAD;
      referenceLongitude= atof(argv[4])*DEG2RAD;
      referenceHeight   = atof(argv[5]);
      
      latitude = atof(argv[6])*DEG2RAD;
      longitude= atof(argv[7])*DEG2RAD;
      height   = atof(argv[8]);
  
      result = GEODESY_ComputeNorthingEastingVertical(
        referenceEllipse,  //!< reference ellipse enumerated []
        referenceLatitude,  //!< datum geodetic latitude  [rad]
        referenceLongitude, //!< datum geodetic longitude [rad]
        referenceHeight,    //!< datum geodetic height    [m]
        latitude,           //!< geodetic latitude        [rad]
        longitude,          //!< geodetic longitude       [rad]
        height,             //!< geodetic height          [m]
        &northing,                //!< local geodetic northing  [m]
        &easting,                 //!< local geodetic easting   [m]
        &vertical                 //!< local geodetic vertical  [m]
        );
      if( result == FALSE )
      {
        printf("GEODESY_ComputeNorthingEastingVertical returned FALSE\n");
        return -1;
      }

      printf( "GEODESY_ComputeNorthingEastingVertical\n" );
      printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "reference_latitude  [deg] = %.12lf\n",  referenceLatitude*RAD2DEG );
      printf( "reference_longitude [deg] = %.12lf\n",  referenceLongitude*RAD2DEG );
      printf( "reference_height    [m]   = %.4lf\n",   referenceHeight );
      printf( "latitude            [deg] = %.12lf\n",  latitude*RAD2DEG );
      printf( "longitude           [deg] = %.12lf\n",  longitude*RAD2DEG );
      printf( "height              [m]   = %.4lf\n",   height );
      printf( "northing            [m]   = %.4lf\n",   northing );
      printf( "easting             [m]   = %.4lf\n",   easting );
      printf( "vertical            [m]   = %.4lf\n\n", vertical );
      break;
    }
    case 4:
    {
      double elevation=0;  //!< elevation angle [rad]
      double azimuth=0;    //!< azimuth angle   [rad]
      double fromX = atof( argv[3] );
      double fromY = atof( argv[4] );
      double fromZ = atof( argv[5] );
      double toX = atof( argv[6] );
      double toY = atof( argv[7] );
      double toZ = atof( argv[8] );

      result = GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(
        referenceEllipse,  //!< reference ellipse enumerated []
        fromX, //!< earth centered earth fixed vector from point X component [m]
        fromY, //!< earth centered earth fixed vector from point Y component [m]
        fromZ, //!< earth centered earth fixed vector from point Z component [m]
        toX,   //!< earth centered earth fixed vector to point X component   [m]
        toY,   //!< earth centered earth fixed vector to point Y component   [m]
        toZ,   //!< earth centered earth fixed vector to point Z component   [m]
        &elevation,  //!< elevation angle [rad]
        &azimuth     //!< azimuth angle   [rad]
      );
      if( result == FALSE )
      {
        printf("GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame returned FALSE\n");
        return -1;
      }
      printf( "GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame\n" );
      printf( "ellipse         = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "fromX     [m]   = %.4lf\n",  fromX );
      printf( "fromY     [m]   = %.4lf\n",  fromY );
      printf( "fromZ     [m]   = %.4lf\n",  fromZ );
      printf( "toX       [m]   = %.4lf\n",  toX );
      printf( "toY       [m]   = %.4lf\n",  toY );
      printf( "toZ       [m]   = %.4lf\n",  toZ );
      printf( "azimuth   [deg] = %.12lf\n",  azimuth*RAD2DEG );
      printf( "elevation [deg] = %.12lf\n",  elevation*RAD2DEG );

      break;
    }    
    case 5:
    {
      double referenceLatitude;  //!< datum geodetic latitude  [rad]
      double referenceLongitude; //!< datum geodetic longitude [rad]
      double referenceHeight;    //!< datum geodetic height    [m]
      double latitude;           //!< geodetic latitude        [rad]
      double longitude;          //!< geodetic longitude       [rad]
      double height;             //!< geodetic height          [m]
      double northing;           //!< local geodetic northing  [m]
      double easting;            //!< local geodetic easting   [m]
      double vertical;           //!< local geodetic vertical  [m]
      
      referenceLatitude = atof(argv[3])*DEG2RAD;
      referenceLongitude= atof(argv[4])*DEG2RAD;
      referenceHeight   = atof(argv[5]);
      
      latitude = atof(argv[6])*DEG2RAD;
      longitude= atof(argv[7])*DEG2RAD;
      height   = atof(argv[8]);
  
      result = GEODESY_ComputeNorthingEastingVertical(
        referenceEllipse,   // reference ellipse enumerated []
        referenceLatitude,  // datum geodetic latitude  [rad]
        referenceLongitude, // datum geodetic longitude [rad]
        referenceHeight,    // datum geodetic height    [m]
        latitude,           // geodetic latitude        [rad]
        longitude,          // geodetic longitude       [rad]
        height,             // geodetic height          [m]
        &northing,          // local geodetic northing  [m]
        &easting,           // local geodetic easting   [m]
        &vertical           // local geodetic vertical  [m]
        );
      if( result == FALSE )
      {
        printf("GEODESY_ComputeNorthingEastingVertical returned FALSE\n");
        return -1;
      }

      printf( "GEODESY_ComputePositionDifference\n" );
      printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      printf( "reference_latitude  [deg] = %.12lf\n",  referenceLatitude*RAD2DEG );
      printf( "reference_longitude [deg] = %.12lf\n",  referenceLongitude*RAD2DEG );
      printf( "reference_height    [m]   = %.4lf\n",   referenceHeight );
      printf( "latitude            [deg] = %.12lf\n",  latitude*RAD2DEG );
      printf( "longitude           [deg] = %.12lf\n",  longitude*RAD2DEG );
      printf( "height              [m]   = %.4lf\n",   height );
      printf( "northing            [m]   = %.4lf\n",   northing );
      printf( "easting             [m]   = %.4lf\n",   easting );
      printf( "vertical            [m]   = %.4lf\n\n", vertical );
      break;
    }
    default:
    {
      break;
    }
  }

  return 0;
}
