//
//============================================================================
/// \file    geodesy_main.c
/// \brief   'c' main program to use the function library geodesy.h/.c
/// \author  Glenn D. MacGougan (GDM)
/// \date    2007-04-20
/// \since   2007-04-20
/// 
/// 2007-04-20, GDM, Creation \n
///
/// \b "LICENSE INFORMATION" \n
/// Copyright (c) 2006, refer to 'author' doxygen tags \n
/// All rights reserved. \n
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided the following conditions are met: \n
///
/// - Redistributions of source code must retain the above copyright
///   notice, this list of conditions and the following disclaimer. \n
/// - Redistributions in binary form must reproduce the above copyright
///   notice, this list of conditions and the following disclaimer in the
///   documentation and/or other materials provided with the distribution. \n
/// - The name(s) of the contributor(s) may not be used to endorse or promote 
///   products derived from this software without specific prior written 
///   permission. \n
///
/// THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
/// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
/// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
/// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
/// SUCH DAMAGE.
//=============================================================================

#include <stdio.h>
#include <stdlib.h>
#include "geodesy.h"

int main( int argc, char* argv[] )
{
  const int min_nr_args = 4;
  int ellipse=0;
  int fn_choice=0;
  GEODESY_enumReferenceEllipse  referenceEllipse = GEODESY_REFERENCE_ELLIPSE_WGS84;
  FILE* fid=NULL;
  BOOL result = FALSE;
  BOOL echo_on = FALSE;

  if( argc < min_nr_args )
  {
    printf("GNSS_ESSENTIALS:GEODESY_MAIN\nUSAGE\n");
    printf("geodesy.bin <ellipse> <function> <outputfile> <additional arguments for function ... >\n");
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

    printf("\nfunction:\n");
    printf("0, GetReferenceEllipseParameters\n");
    printf("   requires no additions arguments\n");
    printf("1, ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates\n");
    printf("   latitude[rad], longitude[rad], height[m]\n");
    printf("2, ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates\n");
    printf("   userX[m], userY[m], userZ[m]\n");
    printf("3, ComputeNorthingEastingVertical\n");
    printf("   ref_lat[rad], ref_lon[rad], ref_hgt[m], lat[rad], lon[rad], hgt[m]\n");
    printf("4  ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame\n" );
    printf("   fromX[m], fromY[m], fromZ[m], toX[m], toY[m], toZ[m]\n" );

    printf("\n\nAll output in base units. i.e. metres, radians, etc\n" );
    return 0;
  }

  ellipse = atoi(argv[1]);
  if( ellipse > 15 )
  { 
    perror("GNSS_ESSENTIALS:GEODESY_MAIN::Invalid ellipse argument\n");
    return 1;
  }
  referenceEllipse = (GEODESY_enumReferenceEllipse)ellipse;

  fn_choice = atoi(argv[2]);
  if( fn_choice < 0 || fn_choice > 4 )
  { 
    perror("GNSS_ESSENTIALS:GEODESY_MAIN::Invalid function argument\n");
    return 1;
  }


  // Check the number of required arguments
  switch(fn_choice)
  {
    case 0: break;
    case 1: if( argc !=  7 ){ perror("GNSS_ESSENTIALS:GEODESY_MAIN::Invalid function arguments\n"); return 1; } break;
    case 4: if( argc != 10 ){ perror("GNSS_ESSENTIALS:GEODESY_MAIN::Invalid function arguments\n"); return 1; } break;
    default: break;
  }

  fid = fopen( argv[3], "w" );
  if( fid == NULL )
  {
    perror("GNSS_ESSENTIALS:GEODESY_MAIN::Invalid output file argument\n");
    return 1;
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

      if( result == TRUE )
      {
        if( echo_on )
        {
          printf( "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_GetReferenceEllipseParameters\n" );
          printf( "ellipse = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
          printf( "a       = %20.3f\n", a );
          printf( "b       = %20.9f\n", b );
          printf( "f_inv   = %20.11f\n", f_inv );
          printf( "e2      = %.15f\n\n", e2 );
        }

        fprintf( fid, "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_GetReferenceEllipseParameters\n" );
        fprintf( fid, "ellipse = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
        fprintf( fid, "a       = %20.3f\n", a );
        fprintf( fid, "b       = %20.9f\n", b );
        fprintf( fid, "f_inv   = %20.11f\n", f_inv );
        fprintf( fid, "e2      = %.15f\n\n", e2 );
      }        
      else
      { 
        perror("GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_GetReferenceEllipseParameters returned FALSE\n");
      }      
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

      latitude  = atof( argv[4] );
      longitude = atof( argv[5] );
      height    = atof( argv[6] );

      result = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        referenceEllipse,  //!< reference ellipse enumerated []
        latitude,   //!< geodetic latitude                [rad]
        longitude,  //!< geodetic longitude               [rad]
        height,     //!< geodetic height                  [m]
        &x,               //!< earth fixed cartesian coordinate [m]
        &y,               //!< earth fixed cartesian coordinate [m]
        &z                //!< earth fixed cartesian coordinate [m]
      );
      if( result == TRUE )
      {
        if( echo_on )
        { 
          printf( "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates\n" );
          printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
          printf( "latitude  = %.12lf\n", latitude );
          printf( "longitude = %.12lf\n", longitude );
          printf( "height    = %.4lf\n", height );
          printf( "x         = %.4lf\n", x );
          printf( "y         = %.4lf\n", y );
          printf( "z         = %.4lf\n\n", z );
        }

        fprintf( fid, "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates\n" );
        fprintf( fid, "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
        fprintf( fid, "latitude  = %.12lf\n", latitude );
        fprintf( fid, "longitude = %.12lf\n", longitude );
        fprintf( fid, "height    = %.4lf\n", height );
        fprintf( fid, "x         = %.4lf\n", x );
        fprintf( fid, "y         = %.4lf\n", y );
        fprintf( fid, "z         = %.4lf\n\n", z );
      }        
      else
      { 
        perror("GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned FALSE\n");
      }      
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

      x = atof( argv[4] );
      y = atof( argv[5] );
      z = atof( argv[6] );

      GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
        referenceEllipse,  //!< reference ellipse enumerated []     
        x,              // earth fixed cartesian coordinate [m]
        y,              // earth fixed cartesian coordinate [m]
        z,              // earth fixed cartesian coordinate [m]  
        &latitude,            // geodetic latitude                [rad]
        &longitude,           // geodetic longitude               [rad]
        &height               // geodetic height                  [m]
      );

      if( echo_on )
      { 
        printf( "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates\n" );
        printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
        printf( "latitude  = %.12lf\n", latitude );
        printf( "longitude = %.12lf\n", longitude );
        printf( "height    = %.4lf\n", height );
        printf( "x         = %.4lf\n", x );
        printf( "y         = %.4lf\n", y );
        printf( "z         = %.4lf\n\n", z );
      }

      fprintf( fid, "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates\n" );
      fprintf( fid, "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      fprintf( fid, "latitude  = %.12lf\n", latitude );
      fprintf( fid, "longitude = %.12lf\n", longitude );
      fprintf( fid, "height    = %.4lf\n", height );
      fprintf( fid, "x         = %.4lf\n", x );
      fprintf( fid, "y         = %.4lf\n", y );
      fprintf( fid, "z         = %.4lf\n\n", z );
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
      
      referenceLatitude = atof(argv[4]);
      referenceLongitude= atof(argv[5]);
      referenceHeight   = atof(argv[6]);
      
      latitude = atof(argv[7]);
      longitude= atof(argv[8]);
      height   = atof(argv[9]);
  
      GEODESY_ComputeNorthingEastingVertical(
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

      if( echo_on )
      { 
        printf( "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ComputeNorthingEastingVertical\n" );
        printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
        printf( "reference_latitude  = %.12lf\n",  referenceLatitude );
        printf( "reference_longitude = %.12lf\n",  referenceLongitude );
        printf( "reference_height    = %.4lf\n",   referenceHeight );
        printf( "latitude            = %.12lf\n",  latitude );
        printf( "longitude           = %.12lf\n",  longitude );
        printf( "height              = %.4lf\n", height );
        printf( "northing            = %.4lf\n",   northing );
        printf( "easting             = %.4lf\n",   easting );
        printf( "vertical            = %.4lf\n\n", vertical );
      }

      fprintf( fid, "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ComputeNorthingEastingVertical\n" );
      fprintf( fid, "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      fprintf( fid, "reference_latitude  = %.12lf\n",  referenceLatitude );
      fprintf( fid, "reference_longitude = %.12lf\n",  referenceLongitude );
      fprintf( fid, "reference_height    = %.4lf\n",   referenceHeight );
      fprintf( fid, "latitude            = %.12lf\n",  latitude );
      fprintf( fid, "longitude           = %.12lf\n",  longitude );
      fprintf( fid, "height              = %.4lf\n", height );
      fprintf( fid, "northing            = %.4lf\n",   northing );
      fprintf( fid, "easting             = %.4lf\n",   easting );
      fprintf( fid, "vertical            = %.4lf\n\n", vertical );
      
      break;
    }
    case 4:
    {
      double elevation=0;  //!< elevation angle [rad]
      double azimuth=0;    //!< azimuth angle   [rad]
      double fromX = atof( argv[4] );
      double fromY = atof( argv[5] );
      double fromZ = atof( argv[6] );
      double toX = atof( argv[7] );
      double toY = atof( argv[8] );
      double toZ = atof( argv[9] );


      GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(
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

      if( echo_on )
      { 
        printf( "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame\n" );
        printf( "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
        printf( "fromX     = %.4lf\n",  fromX );
        printf( "fromY     = %.4lf\n",  fromY );
        printf( "fromZ     = %.4lf\n",  fromZ );
        printf( "toX       = %.4lf\n",  toX );
        printf( "toY       = %.4lf\n",  toY );
        printf( "toZ       = %.4lf\n",  toZ );
        printf( "azimuth   = %.12lf\n",  azimuth );
        printf( "elevation = %.12lf\n",  elevation );
      }

      fprintf( fid, "GNSS_ESSENTIALS:GEODESY_MAIN::GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame\n" );
      fprintf( fid, "ellipse   = %s\n", GEODESY_REFERENCE_ELLIPSE_STRING_DESCRIPTION[ellipse] );
      fprintf( fid, "fromX     = %.4lf\n",  fromX );
      fprintf( fid, "fromY     = %.4lf\n",  fromY );
      fprintf( fid, "fromZ     = %.4lf\n",  fromZ );
      fprintf( fid, "toX       = %.4lf\n",  toX );
      fprintf( fid, "toY       = %.4lf\n",  toY );
      fprintf( fid, "toZ       = %.4lf\n",  toZ );
      fprintf( fid, "azimuth   = %.12lf\n",  azimuth );
      fprintf( fid, "elevation = %.12lf\n",  elevation );

      break;
    }    
    default:
    {
      break;
    }
  }

  fclose(fid);

  return 0;
}
