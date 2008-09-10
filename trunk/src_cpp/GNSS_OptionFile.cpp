/**
\file    GNSS_OptionFile.cpp
\brief   The option file for EssentialGNSS.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-28
\since   2006-12-30

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

#include <memory.h>
#include "constants.h"
#include "GNSS_OptionFile.h"
#include "gnss_error.h"
#include "rinex.h"
#include "geodesy.h"
#include "StdStringUtils.h"

namespace GNSS
{

  GNSS_OptionFile::~GNSS_OptionFile()
  {
  }


  GNSS_OptionFile::GNSS_OptionFile()
    : m_processDGPSOnly(true),
    m_RoverIsStatic(true),
    m_elevationMask(0.0),
    m_cnoMask(0.0),
    m_locktimeMask(0.0),
    m_isPositionFixed(false),
    m_isHeightConstrained(false)
  {     
    memset( &m_klobuchar, 0, sizeof(m_klobuchar) );
  }



  bool GNSS_OptionFile::ReadAndInterpretOptions( const std::string OptionFilePath )
  {
    double d[4];
    unsigned n;
    bool result;
    bool useECEF = false;
    bool useLLH = false;
    BOOL resultBOOL = FALSE;
    double angleInDegs;
    double lat;
    double lon;
    double hgt;

    if( !ReadOptionFile( OptionFilePath ) )
    {
      GNSS_ERROR_MSG( "ReadOptionFile returned false." );
      return false;
    }

    m_OptionFilePath = OptionFilePath;

    if( !GetValue( "OutputFilePath", m_OutputFilePath ) )
    {
      GNSS_ERROR_MSG( "Invalid option: OutputFilePath" );
      return false;
    }

    if( !GetValue( "ProcessingMethod", m_ProcessingMethod ) )
    {
      GNSS_ERROR_MSG( "Invalid option: ProcessingMethod" );
      return false; 
    }
    if( m_ProcessingMethod != "LSQ" 
		 && m_ProcessingMethod != "EKF" 
     && m_ProcessingMethod != "RTK4" 
     && m_ProcessingMethod != "RTK8" 
     && m_ProcessingMethod != "TRIPLEDIFF" 
		 && m_ProcessingMethod != "RTKDD" )
    {
      GNSS_ERROR_MSG( "Invalid option: ProcessingMethod" );
      return false;
    }

    if( m_ProcessingMethod == "RTK4" )
    {
      if( !GetValue( "RTK4_sigmaNorth", m_KalmanOptions.RTK4_sigmaNorth ) )
      {
        GNSS_ERROR_MSG( "Invalid option: RTK4_sigmaNorth" );
        return false; 
      }
      if( !GetValue( "RTK4_sigmaEast", m_KalmanOptions.RTK4_sigmaEast ) )
      {
        GNSS_ERROR_MSG( "Invalid option: RTK4_sigmaEast" );
        return false; 
      }
      if( !GetValue( "RTK4_sigmaUp", m_KalmanOptions.RTK4_sigmaUp ) )
      {
        GNSS_ERROR_MSG( "Invalid option: RTK4_sigmaUp" );
        return false; 
      }
      if( !GetValue( "RTK4_sigmaClock", m_KalmanOptions.RTK4_sigmaClock ) )
      {
        GNSS_ERROR_MSG( "Invalid option: RTK4_sigmaClock" );
        return false; 
      }
    }
    else if( m_ProcessingMethod == "EKF" || m_ProcessingMethod == "RTK8" )
    {
      if( !GetValue( "alpha_Vn", m_KalmanOptions.alphaVn ) )
      {
        GNSS_ERROR_MSG( "Invalid option: alpha_Vn" );
        return false; 
      }
      if( !GetValue( "alpha_Ve", m_KalmanOptions.alphaVe ) )
      {
        GNSS_ERROR_MSG( "Invalid option: alpha_Ve" );
        return false; 
      }
      if( !GetValue( "alpha_Vup", m_KalmanOptions.alphaVup ) )
      {
        GNSS_ERROR_MSG( "Invalid option: alpha_Vup" );
        return false; 
      }
      if( !GetValue( "alpha_ClkDrift", m_KalmanOptions.alphaClkDrift ) )
      {
        GNSS_ERROR_MSG( "Invalid option: alpha_ClkDrift" );
        return false; 
      }

      if( !GetValue( "sigma_Vn", m_KalmanOptions.sigmaVn ) )
      {
        GNSS_ERROR_MSG( "Invalid option: sigma_Vn" );
        return false; 
      }
      if( !GetValue( "sigma_Ve", m_KalmanOptions.sigmaVe ) )
      {
        GNSS_ERROR_MSG( "Invalid option: sigma_Ve" );
        return false; 
      }
      if( !GetValue( "sigma_Vup", m_KalmanOptions.sigmaVup ) )
      {
        GNSS_ERROR_MSG( "Invalid option: sigma_Vup" );
        return false; 
      }
      if( !GetValue( "sigma_ClkDrift", m_KalmanOptions.sigmaClkDrift ) )
      {
        GNSS_ERROR_MSG( "Invalid option: sigma_ClkDrift" );
        return false; 
      }
    }

    if( !GetValue( "StartGPSWeek", m_StartTime.GPSWeek ) )
    {
      GNSS_ERROR_MSG( "Invalid option: StartGPSWeek" );
      return false;
    }
    if( !GetValue( "StartGPSTimeOfWeek", m_StartTime.GPSTimeOfWeek ) )
    {
      GNSS_ERROR_MSG( "Invalid option: StartGPSTimeOfWeek" );
      return false;
    }

    if( !GetValue( "EndGPSWeek", m_EndTime.GPSWeek ) )
    {
      GNSS_ERROR_MSG( "Invalid option: EndGPSWeek" );
      return false;
    }
    if( !GetValue( "EndGPSTimeOfWeek", m_EndTime.GPSTimeOfWeek ) )
    {
      GNSS_ERROR_MSG( "Invalid option: EndGPSTimeOfWeek" );
      return false;
    }

    if( !GetValue( "ElevationMask", m_elevationMask ) )
    {
      GNSS_ERROR_MSG( "Invalid option: ElevationMask" );
      return false;
    }
    if( !GetValue( "CNoMask", m_cnoMask ) )
    {
      GNSS_ERROR_MSG( "Invalid option: CNoMask" );
      return false;
    }
    if( !GetValue( "LockTimeMask", m_locktimeMask ) )
    {
      GNSS_ERROR_MSG( "Invalid option: LockTimeMask" );
      return false;
    }

    if( !GetValue( "ProcessOnlyDGPS", m_processDGPSOnly ) )
    {
      GNSS_ERROR_MSG( "Invalid option: ProcessOnlyDGPS" );
      return false;
    }

    GetValue( "UseDopplerMeasurements", m_UseDopplerMeasurements );

    GetValue( "RINEXNavigationDataPath", m_RINEXNavDataPath );      

    GetValue( "Reference_DataPath", m_Reference.DataPath );

    m_Reference.isValid = false;
    if( !m_Reference.DataPath.empty() )
    {
      if( DoesFileExist( m_Reference.DataPath ) )
      { 
        m_Reference.isValid = true;

        if( !GetValue( "Reference_DataType", m_Reference.DataTypeStr ) )
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_DataType" );
          return false;
        }

        StdStringUtils::MakeUpper(m_Reference.DataTypeStr);

        if( m_Reference.DataTypeStr.compare("NOVATELOEM4") == 0 )
        {
          m_Reference.DataType = GNSS_RXDATA_NOVATELOEM4;
        }
        else if( m_Reference.DataTypeStr.compare("RINEX2.1") == 0 )
        {
          m_Reference.DataType = GNSS_RXDATA_RINEX21;
        }
        else if( m_Reference.DataTypeStr.compare("RINEX2.11") == 0 )
        {
          m_Reference.DataType = GNSS_RXDATA_RINEX211;
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_DataType" );
          return false;
        }

        if( GetValue( "Reference_stdev_GPSL1_psr", m_Reference.stdev_GPSL1_psr ) )
        {
          if( m_Reference.stdev_GPSL1_psr <= 0 )
            m_Reference.stdev_GPSL1_psr = 0.8;
        }
        if( GetValue( "Reference_stdev_GPSL1_doppler", m_Reference.stdev_GPSL1_doppler ) )
        {
          if( m_Reference.stdev_GPSL1_doppler <= 0 )
            m_Reference.stdev_GPSL1_doppler = 0.09;
        }
        if( GetValue( "Reference_stdev_GPSL1_adr", m_Reference.stdev_GPSL1_adr ) )
        {
          if( m_Reference.stdev_GPSL1_adr <= 0 )
            m_Reference.stdev_GPSL1_adr = 0.03;
        }

        if( !GetValue( "Reference_UseECEF", useECEF ) )
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_UseECEF" );
          return false;
        }
        if( useECEF )
        {
          if( !GetValue( "Reference_ECEF_X", m_Reference.x ) )
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_ECEF_X" );
            return false;
          }
          if( !GetValue( "Reference_ECEF_Y", m_Reference.y ) )
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_ECEF_Y" );
            return false;
          }
          if( !GetValue( "Reference_ECEF_Z", m_Reference.z ) )
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_ECEF_Z" );
            return false;
          }

          resultBOOL = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
            GEODESY_REFERENCE_ELLIPSE_WGS84,
            m_Reference.x,
            m_Reference.y,
            m_Reference.z,
            &(m_Reference.latitudeRads),
            &(m_Reference.longitudeRads),
            &(m_Reference.height)
            );
          if( resultBOOL == FALSE )
          {
            GNSS_ERROR_MSG( "GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates returned FALSE. Check Reference_ECEF values." );
            return false;
          }
          m_Reference.latitudeDegrees  = m_Reference.latitudeRads  * RAD2DEG;
          m_Reference.longitudeDegrees = m_Reference.longitudeRads * RAD2DEG;
        }
        else
        {
          GetValueArray( "Reference_Latitude", d, 4, n );
          if( n == 1 )
          {
            m_Reference.latitudeDegrees = d[0];
          }
          else if( n == 3 )
          {
            GetDMSValue( "Reference_Latitude", m_Reference.latitudeDegrees );
          }
          else
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_Latitude" );
            return false;
          }
          m_Reference.latitudeRads = m_Reference.latitudeDegrees*DEG2RAD;

          GetValueArray( "Reference_Longitude", d, 4, n );
          if( n == 1 )
          {
            m_Reference.longitudeDegrees = d[0];
          }
          else if( n == 3 )
          {
            GetDMSValue( "Reference_Longitude", m_Reference.longitudeDegrees );
          }
          else
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_Longitude" );
            return false;
          }
          m_Reference.longitudeRads = m_Reference.longitudeDegrees*DEG2RAD;

          if( !GetValue( "Reference_Height", m_Reference.height ) )
          {
            GNSS_ERROR_MSG( "Invalid option: Reference_Height" );
            return false;
          }

          resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
            GEODESY_REFERENCE_ELLIPSE_WGS84,
            m_Reference.latitudeRads,
            m_Reference.longitudeRads,
            m_Reference.height,
            &m_Reference.x,
            &m_Reference.y,
            &m_Reference.z 
            );
          if( resultBOOL == FALSE )
          {
            GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned false. Check Reference_Latitude, Reference_Longitude, Reference_Height." );
            return false;
          }
        }

        // The reference is known to sub mm.
        m_Reference.uncertaintyLatitudeOneSigma = 1.0e-04;
        m_Reference.uncertaintyLongitudeOneSigma = 1.0e-04;
        m_Reference.uncertaintyHeightOneSigma = 1.0e-04;

        if( !GetValue( "Reference_EnableTropoCorrection", m_Reference.useTropo ) )
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_EnableTropoCorrection" );
          return false;
        }
        if( !GetValue( "Reference_EnableIonoCorrection", m_Reference.useIono ) )
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_EnableIonoCorrection" );
          return false;
        }
        if( !GetValueArray( "Reference_ExcludeSatellites", m_Reference.satsToExclude, 64, n ) )
        {
          GNSS_ERROR_MSG( "Invalid option: Reference_ExcludeSatellites" );
          return false;
        }
        m_Reference.nrSatsToExclude = n;
      }
    }



    result = false;
    GetValue( "Iono_KlobucharIsValid", result );
    if( result )
    {
      m_klobuchar.isValid = 1;
      GetValue( "Iono_KlobucharReferenceWeek", m_klobuchar.week );
      GetValue( "Iono_KlobucharReferenceTime", m_klobuchar.tow );

      GetValueArray( "Iono_KlobucharAlphaParameters", d, 4, n );
      if( n == 4 )
      {
        m_klobuchar.alpha0 = d[0];
        m_klobuchar.alpha1 = d[1];
        m_klobuchar.alpha2 = d[2];
        m_klobuchar.alpha3 = d[3];
      }
      GetValueArray( "Iono_KlobucharBetaParameters", d, 4, n );
      if( n == 4 )
      {
        m_klobuchar.beta0 = d[0];
        m_klobuchar.beta1 = d[1];
        m_klobuchar.beta2 = d[2];
        m_klobuchar.beta3 = d[3];
      }
    }
    else
    {
      result = false;
      GetValue( "Iono_ObtainFromRINEXNavFile", result );
      if( result )
      {
        resultBOOL = RINEX_GetKlobucharIonoParametersFromNavFile(
          m_RINEXNavDataPath.c_str(),
          &m_klobuchar
          );
        if( resultBOOL == FALSE )
        {
          GNSS_ERROR_MSG( "Invalid option: Iono_ObtainFromRINEXNavFile, RINEX_GetKlobucharIonoParametersFromNavFile returned FALSE." );
          return false;       
        }
      }
      else
      {
        m_klobuchar.isValid = 0;
      }
    }

    m_Rover.isValid = false;
    if( !GetValue( "Rover_DataPath", m_Rover.DataPath ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_DataPath" );
      return false;
    }

    if( m_Rover.DataPath.empty() )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_DataPath" );
      return false;
    }
    if( !DoesFileExist( m_Rover.DataPath ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_DataPath" );
      return false;
    }

    if( !GetValue( "Rover_DataType", m_Rover.DataTypeStr ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_DataType" );
      return false;
    }

    StdStringUtils::MakeUpper(m_Rover.DataTypeStr);

    if( m_Rover.DataTypeStr.compare("NOVATELOEM4") == 0 )
    {
      m_Rover.DataType = GNSS_RXDATA_NOVATELOEM4;
    }
    else if( m_Rover.DataTypeStr.compare("RINEX2.1") == 0 )
    {
      m_Rover.DataType = GNSS_RXDATA_RINEX21;
    }
    else if( m_Rover.DataTypeStr.compare("RINEX2.11") == 0 )
    {
      m_Rover.DataType = GNSS_RXDATA_RINEX211;
    }
    else
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_DataType" );
      return false;
    }

    if( GetValue( "Rover_stdev_GPSL1_psr", m_Rover.stdev_GPSL1_psr ) )
    {
      if( m_Rover.stdev_GPSL1_psr <= 0 )
        m_Rover.stdev_GPSL1_psr = 0.8;
    }
    if( GetValue( "Rover_stdev_GPSL1_doppler", m_Rover.stdev_GPSL1_doppler ) )
    {
      if( m_Rover.stdev_GPSL1_doppler <= 0 )
        m_Rover.stdev_GPSL1_doppler = 0.09;
    }
    if( GetValue( "Rover_stdev_GPSL1_adr", m_Rover.stdev_GPSL1_adr ) )
    {
      if( m_Rover.stdev_GPSL1_adr <= 0 )
        m_Rover.stdev_GPSL1_adr = 0.03;
    }

    if( !GetValue( "Rover_UseECEF", useECEF ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_UseECEF" );
      return false;
    }

    if( useECEF )
    {
      if( !GetValue( "Rover_ECEF_X", m_Rover.x ) )
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_ECEF_X" );
        return false;
      }
      if( !GetValue( "Rover_ECEF_Y", m_Rover.y ) )
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_ECEF_Y" );
        return false;
      }
      if( !GetValue( "Rover_ECEF_Z", m_Rover.z ) )
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_ECEF_Z" );
        return false;
      }

      resultBOOL = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        m_Rover.x,
        m_Rover.y,
        m_Rover.z,
        &(m_Rover.latitudeRads),
        &(m_Rover.longitudeRads),
        &(m_Rover.height)
        );
      if( resultBOOL == FALSE )
      {
        GNSS_ERROR_MSG( "GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates returned FALSE. Check Rover_ECEF values." );
        return false;
      }
      m_Rover.latitudeDegrees  = m_Rover.latitudeRads  * RAD2DEG;
      m_Rover.longitudeDegrees = m_Rover.longitudeRads * RAD2DEG;
    }
    else
    {
      GetValueArray( "Rover_Latitude", d, 4, n );
      if( n == 1 )
      {
        m_Rover.latitudeDegrees = d[0];
      }
      else if( n == 3 )
      {
        GetDMSValue( "Rover_Latitude", m_Rover.latitudeDegrees );
      }
      else
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_Latitude" );
        return false;
      }
      m_Rover.latitudeRads = m_Rover.latitudeDegrees*DEG2RAD;

      GetValueArray( "Rover_Longitude", d, 4, n );
      if( n == 1 )
      {
        m_Rover.longitudeDegrees = d[0];
      }
      else if( n == 3 )
      {
        GetDMSValue( "Rover_Longitude", m_Rover.longitudeDegrees );
      }
      else
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_Longitude" );
        return false;
      }
      m_Rover.longitudeRads = m_Rover.longitudeDegrees*DEG2RAD;

      if( !GetValue( "Rover_Height", m_Rover.height ) )
      {
        GNSS_ERROR_MSG( "Invalid option: Rover_Height" );
        return false;
      }

      resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        m_Rover.latitudeRads,
        m_Rover.longitudeRads,
        m_Rover.height,
        &m_Rover.x,
        &m_Rover.y,
        &m_Rover.z 
        );
      if( resultBOOL == FALSE )
      {
        GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned false. Check Rover_Latitude, Rover_Longitude, Rover_Height." );
        return false;
      }
    }

    if( !GetValue( "Rover_UncertaintyLatitude", m_Rover.uncertaintyLatitudeOneSigma ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_UncertaintyLatitude" );
      return false;
    }
    if( !GetValue( "Rover_UncertaintyLongitude", m_Rover.uncertaintyLongitudeOneSigma ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_UncertaintyLongitude" );
      return false;
    }
    if( !GetValue( "Rover_UncertaintyHeight", m_Rover.uncertaintyHeightOneSigma ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_UncertaintyHeight" );
      return false;
    }

    if( !GetValue( "Rover_EnableTropoCorrection", m_Rover.useTropo ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_EnableTropoCorrection" );
      return false;
    }
    if( !GetValue( "Rover_EnableIonoCorrection", m_Rover.useIono ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_EnableIonoCorrection" );
      return false;
    }
    if( !GetValueArray( "Rover_ExcludeSatellites", m_Rover.satsToExclude, 64, n ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Rover_ExcludeSatellites" );
      return false;
    }
    m_Rover.nrSatsToExclude = n;


    if( GetValue( "Rover_EnableFixedPositionConstraint", m_isPositionFixed ) )
    {
      if( m_isPositionFixed )
      {
        m_Rover.uncertaintyLatitudeOneSigma  = 1e-06;
        m_Rover.uncertaintyLongitudeOneSigma = 1e-06;
        m_Rover.uncertaintyHeightOneSigma = 1e-06;
      }
    }

    GetValue( "Rover_EnableHeightConstraint", m_isHeightConstrained );


    m_RoverDatum.isValid = true;
    GetValueArray( "Rover_DatumLatitude", d, 4, n );
    if( n == 1 )
    {
      m_RoverDatum.latitudeRads = d[0];
      m_RoverDatum.latitudeRads *= DEG2RAD;
    }
    else if( n == 3 )
    {
      GetDMSValue( "Rover_DatumLatitude", m_RoverDatum.latitudeRads );
      m_RoverDatum.latitudeRads *= DEG2RAD;
    }
    else
    {
      m_RoverDatum.isValid = false;
    }

    GetValueArray( "Rover_DatumLongitude", d, 4, n );
    if( n == 1 )
    {
      m_RoverDatum.longitudeRads = d[0];
      m_RoverDatum.longitudeRads *= DEG2RAD;
    }
    else if( n == 3 )
    {
      GetDMSValue( "Rover_DatumLongitude", m_RoverDatum.longitudeRads );
      m_RoverDatum.longitudeRads *= DEG2RAD;
    }
    else
    {
      m_RoverDatum.isValid = false;
    }
    
    if( !GetValue( "Rover_DatumHeight", m_RoverDatum.height ) )
      m_RoverDatum.isValid = false;

    GetValue( "Rover_IsStatic", m_RoverIsStatic );
    
    m_Rover.isValid = true;

#ifdef GDM_UWB_RANGE_HACK    
    GetValue( "Rover_UWBFilePath", m_UWBFilePath );      

    if( !GetValue( "ReferenceUWB1_ID", m_UWB_a.id ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Reference_UWB1_ID" );
      return false;
    }      
    if( m_UWB_a.id >= 0 )
    {
      if( !GetValue( "ReferenceUWB1_UseECEF", useECEF ) )
      {
        GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_UseECEF" );
        return false;
      }
      if( useECEF )
      {
        // Station 1
        if( !GetValue( "ReferenceUWB1_ECEF_X", m_UWB_a.x ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_ECEF_X" );
          return false;
        }
        if( !GetValue( "ReferenceUWB1_ECEF_Y", m_UWB_a.y ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_ECEF_Y" );
          return false;
        }
        if( !GetValue( "ReferenceUWB1_ECEF_Z", m_UWB_a.z ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_ECEF_Z" );
          return false;
        }
      }
      else
      {
        if( !GetValue( "ReferenceUWB1_UseLLH", useLLH ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_UseLLH" );
          return false;
        }
        if( !useLLH )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_UseLLH" );
          return false;
        }
        
        GetValueArray( "ReferenceUWB1_Latitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB1_Latitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_Latitude" );
          return false;
        }
        lat = angleInDegs*DEG2RAD;

        GetValueArray( "ReferenceUWB1_Longitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB1_Longitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_Longitude" );
          return false;
        }
        lon = angleInDegs*DEG2RAD;

        if( !GetValue( "ReferenceUWB1_Height", hgt ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB1_Height" );
          return false;
        }

        resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
          GEODESY_REFERENCE_ELLIPSE_WGS84,
          lat,
          lon,
          hgt,
          &m_UWB_a.x,
          &m_UWB_a.y,
          &m_UWB_a.z
        );
        if( resultBOOL == FALSE )
        {
          GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned false. Check UWB1 Latitude, Longitude, Height." );
          return false;
        }
      }
    }

    if( !GetValue( "ReferenceUWB2_ID", m_UWB_b.id ) )
    {
      GNSS_ERROR_MSG( "Invalid option: Reference_UWB2_ID" );
      return false;
    }      
    if( m_UWB_b.id >= 0 )
    {
      if( !GetValue( "ReferenceUWB2_UseECEF", useECEF ) )
      {
        GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_UseECEF" );
        return false;
      }
      if( useECEF )
      {
        // Station 2
        if( !GetValue( "ReferenceUWB2_ECEF_X", m_UWB_b.x ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_ECEF_X" );
          return false;
        }
        if( !GetValue( "ReferenceUWB2_ECEF_Y", m_UWB_b.y ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_ECEF_Y" );
          return false;
        }
        if( !GetValue( "ReferenceUWB2_ECEF_Z", m_UWB_b.z ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_ECEF_Z" );
          return false;
        }
      }
      else
      {
        if( !GetValue( "ReferenceUWB2_UseLLH", useLLH ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_UseLLH" );
          return false;
        }
        if( !useLLH )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_UseLLH" );
          return false;
        }
        
        GetValueArray( "ReferenceUWB2_Latitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB2_Latitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_Latitude" );
          return false;
        }
        lat = angleInDegs*DEG2RAD;

        GetValueArray( "ReferenceUWB2_Longitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB2_Longitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_Longitude" );
          return false;
        }
        lon = angleInDegs*DEG2RAD;

        if( !GetValue( "ReferenceUWB2_Height", hgt ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB2_Height" );
          return false;
        }

        resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
          GEODESY_REFERENCE_ELLIPSE_WGS84,
          lat,
          lon,
          hgt,
          &m_UWB_b.x,
          &m_UWB_b.y,
          &m_UWB_b.z
        );
        if( resultBOOL == FALSE )
        {
          GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned false. Check UWB2 Latitude, Longitude, Height." );
          return false;
        }
      }
    }

    if( !GetValue( "ReferenceUWB3_ID", m_UWB_c.id ) )
    {
      GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_ID" );
      return false;
    }      
    if( m_UWB_c.id >= 0 )
    {
      if( !GetValue( "ReferenceUWB3_UseECEF", useECEF ) )
      {
        GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_UseECEF" );
        return false;
      }
      if( useECEF )
      {
        // Station 2
        if( !GetValue( "ReferenceUWB3_ECEF_X", m_UWB_c.x ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_ECEF_X" );
          return false;
        }
        if( !GetValue( "ReferenceUWB3_ECEF_Y", m_UWB_c.y ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_ECEF_Y" );
          return false;
        }
        if( !GetValue( "ReferenceUWB3_ECEF_Z", m_UWB_c.z ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_ECEF_Z" );
          return false;
        }
      }
      else
      {
        if( !GetValue( "ReferenceUWB3_UseLLH", useLLH ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_UseLLH" );
          return false;
        }
        if( !useLLH )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_UseLLH" );
          return false;
        }
        
        GetValueArray( "ReferenceUWB3_Latitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB3_Latitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_Latitude" );
          return false;
        }
        lat = angleInDegs*DEG2RAD;

        GetValueArray( "ReferenceUWB3_Longitude", d, 4, n );
        if( n == 1 )
        {
          angleInDegs = d[0];
        }
        else if( n == 3 )
        {
          GetDMSValue( "ReferenceUWB3_Longitude", angleInDegs );
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_Longitude" );
          return false;
        }
        lon = angleInDegs*DEG2RAD;

        if( !GetValue( "ReferenceUWB3_Height", hgt ) )
        {
          GNSS_ERROR_MSG( "Invalid option: ReferenceUWB3_Height" );
          return false;
        }

        resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
          GEODESY_REFERENCE_ELLIPSE_WGS84,
          lat,
          lon,
          hgt,
          &m_UWB_c.x,
          &m_UWB_c.y,
          &m_UWB_c.z
        );
        if( resultBOOL == FALSE )
        {
          GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned false. Check UWB3 Latitude, Longitude, Height." );
          return false;
        }
      }
    }

#endif

    return true;
  }

} // end of namespace GNSS;


